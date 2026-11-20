#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
建图任务“补跑”工具：从已经落盘的 pcd 继续完成后处理与上报，不重飞、不重启 LIO。

适用场景
--------
建图飞行结束、Super-LIO 的 /map_save 已经执行，但 task_dispatcher 在
process_point_cloud_data() 里找不到 PCD_FILE_PATH（LIO 与 task_dispatcher 的
HOME/用户不一致等），返回空 file_id，任务既没上传也没上报。

本脚本只做 process_point_cloud_data() 的后半段：
    读 pcd -> 0.25 体素下采样 -> 裁剪 -> 自动 z 裁剪 -> 栅格图 map.png
    -> 上传 png(fileId) / pcd(pcdFileId) -> 发 BuildMap type=1 status=200

用法（机器人上、已 source ROS 环境）
------------------------------------
    python3 -m task_dispatcher.resume_build_map --task-id 46614
    python3 -m task_dispatcher.resume_build_map --task-id 46614 \
        --pcd /home/cat/slam_data/pcd/test.pcd --dry-run
    python3 -m task_dispatcher.resume_build_map --task-id 46614 \
        --pcd-file-id <已上传的pcdId> --file-id <已上传的pngId>

MQTT/服务器参数默认读取与 launch 相同的端点覆盖文件
（UAV_ENDPOINT_CONFIG > ~/.uav_service/endpoint.json > 仓内 endpoint.local.json），
也可用命令行参数逐项覆盖。
"""

import argparse
import base64
import json
import logging
import os
import sys
import threading
import time
import uuid

import open3d as o3d
import paho.mqtt.client as mqtt
import requests
import urllib3
from PIL import Image

# 兼容两种运行方式：
#   1) python3 -m task_dispatcher.resume_build_map
#   2) python3 <install>/lib/task_dispatcher/resume_build_map.py
# 包内的 auth_utils / ros_client_cache 是顶层导入，需要把本目录加入 sys.path。
_HERE = os.path.dirname(os.path.realpath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from auth_utils import AuthManager  # noqa: E402
from task_dispatcher.config import PCD_FILE_PATH, PCD_FILE_COMPRESS_PATH  # noqa: E402
from task_dispatcher.point_cloud_crop import PointCloudCrop  # noqa: E402
from task_dispatcher.point_cloud_manager import PointCloudManager  # noqa: E402

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

logger = logging.getLogger('task_dispatcher.resume_build_map')

# 与 launch/task_dispatcher.launch.py 的内置默认值保持一致，实际值由端点覆盖文件提供
DEFAULTS = {
    'mqtt_broker': 'localhost',
    'mqtt_port': 1883,
    'mqtt_username': '',
    'mqtt_password': '',
    'mqtt_response_topic': '/robot/up',
    'robot_sn': 'UAV00001',
    'server_url': 'https://192.168.200.131:8075',
    'version': '1.1',
}

VALID_KEYS = ('mqtt_broker', 'mqtt_port', 'mqtt_username', 'mqtt_password',
              'mqtt_response_topic', 'robot_sn', 'server_url')


def _candidate_endpoint_files():
    env_path = os.environ.get('UAV_ENDPOINT_CONFIG', '').strip()
    yield env_path or None
    # 源码目录：.../uav_service/endpoint.local.json（本文件在 .../src/task_dispatcher/task_dispatcher/）
    yield os.path.normpath(os.path.join(_HERE, '..', '..', '..', 'endpoint.local.json'))
    yield os.path.join(os.path.expanduser('~'), '.uav_service', 'endpoint.json')


def load_endpoint_defaults():
    """按 launch 相同的顺序加载端点覆盖，失败则回落到内置默认值。"""
    defaults = dict(DEFAULTS)
    for path in _candidate_endpoint_files():
        if not path or not os.path.isfile(path):
            continue
        try:
            with open(path, 'r', encoding='utf-8') as fp:
                data = json.load(fp)
        except (OSError, ValueError) as exc:
            logger.warning('端点覆盖文件读取失败，忽略 %s: %s', path, exc)
            continue
        if not isinstance(data, dict):
            logger.warning('端点覆盖文件不是 JSON 对象，忽略 %s', path)
            continue
        overlay = {k: data[k] for k in VALID_KEYS if data.get(k) not in (None, '')}
        defaults.update(overlay)
        logger.info('已加载端点覆盖: %s', path)
        return defaults
    logger.info('未找到端点覆盖文件，使用内置默认值')
    return defaults


def build_png_and_pcd(pcd_path, image_path):
    """执行 process_point_cloud_data 的后半段（不调用 /map_save）。"""
    if not os.path.isfile(pcd_path):
        raise SystemExit(f'点云文件不存在: {pcd_path}')

    logger.info('读取点云: %s', pcd_path)
    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        raise SystemExit(f'点云读取后为空: {pcd_path}')

    logger.info('点云下采样(0.25)')
    pcd = pcd.voxel_down_sample(0.25)
    crop = PointCloudCrop()
    pcd = crop.preprocess(pcd)
    raster = crop.auto_crop_pointcloud(pcd, x_ratio=0.0, y_ratio=0.0, z_ratio=0.35)

    os.makedirs(os.path.dirname(image_path) or '.', exist_ok=True)
    # pcd_to_raster 不依赖实例状态，这里直接以无实例方式调用，避免构造完整 ROS 管理器
    ok, col_radar, row_radar = PointCloudManager.pcd_to_raster(None, raster, image_path)
    if not ok:
        raise SystemExit('点云栅格化失败，未生成地图图片')

    os.makedirs(os.path.dirname(PCD_FILE_COMPRESS_PATH), exist_ok=True)
    o3d.io.write_point_cloud(PCD_FILE_COMPRESS_PATH, pcd)
    logger.info('压缩点云已保存: %s', PCD_FILE_COMPRESS_PATH)

    return col_radar, row_radar


def upload_file(upload_url, file_path, filename, content_type, timeout):
    with open(file_path, 'rb') as fp:
        files = {'file': (filename, fp, content_type)}
        logger.info('上传 %s -> %s', file_path, upload_url)
        response = requests.post(upload_url, files=files, data={}, timeout=timeout, verify=False)

    if response.status_code != 200:
        logger.error('上传失败: 状态码=%s, 响应=%s', response.status_code, response.text)
        return ''
    result = response.json()
    file_id = (result.get('data') or {}).get('id', '')
    if not file_id:
        logger.error('上传响应中未找到 data.id: %s', result)
    else:
        logger.info('上传成功: %s -> fileId=%s', file_path, file_id)
    return file_id


def publish_build_map(args, body):
    """复刻 MapManager.report_build_map_progress 的报文结构。"""
    message = {
        'sn': args.sn,
        'ver': args.version,
        'seq': str(uuid.uuid4()).replace('-', ''),
        'type': 1,
        'ts': int(time.time() * 1000),
        'cmd': 'BuildMap',
        'status': 200,
        'error': 'ok',
        'body': body,
    }

    client = mqtt.Client(client_id=f'resume_build_map_{uuid.uuid4().hex[:8]}')
    if args.username and args.password:
        client.username_pw_set(args.username, args.password)

    connected = threading.Event()
    conn_result = {'rc': None}

    def _rc_value(reason_code):
        # paho 2.x 传 ReasonCode，paho 1.x 传 int
        return getattr(reason_code, 'value', reason_code)

    def on_connect(_client, _userdata, _flags, reason_code, *extra):
        code = _rc_value(reason_code)
        conn_result['rc'] = code
        if code == 0:
            connected.set()
        else:
            logger.error('MQTT 连接被拒绝，rc=%s（4=用户名或密码错误，5=未授权）', code)

    def on_disconnect(_client, _userdata, *extra):
        logger.warning('MQTT 连接断开: %s', extra)

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect

    client.connect(args.broker, args.port, 60)
    client.loop_start()
    try:
        if not connected.wait(timeout=15):
            raise RuntimeError(
                f'MQTT 连接失败（{args.broker}:{args.port}, rc={conn_result["rc"]}），'
                '请检查 broker/端口是否正确、用户名密码是否传入'
            )
        # 与 task_dispatcher 一致使用 QoS 0，避免依赖 broker 的 PUBACK
        info = client.publish(args.response_topic, json.dumps(message), qos=0)
        info.wait_for_publish(timeout=10)
        if info.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f'BuildMap 消息发布失败, rc={info.rc}')
        if not info.is_published():
            raise RuntimeError('BuildMap 消息发布超时')
        time.sleep(0.5)
    finally:
        client.loop_stop()
        client.disconnect()

    printable = dict(body)
    printable.pop('picBase64', None)
    logger.info('已上报 BuildMap(200): %s', json.dumps(printable, ensure_ascii=False))


def parse_args():
    defaults = load_endpoint_defaults()
    parser = argparse.ArgumentParser(description='从已保存的 pcd 续跑建图任务的后处理与上报')
    parser.add_argument('--task-id', required=True, help='平台原始建图任务 ID (BuildMap body.taskId)')
    parser.add_argument('--pcd', default=PCD_FILE_PATH, help='原始点云路径，默认 PCD_FILE_PATH')
    parser.add_argument('--image', default=os.path.join(os.path.expanduser('~'), 'slam_data', 'map.png'),
                        help='栅格图输出/复用路径')
    parser.add_argument('--pcd-file-id', default='', help='复用已上传的 pcd 文件 ID（跳过 pcd 上传）')
    parser.add_argument('--file-id', default='', help='复用已上传的 png 文件 ID（跳过 png 上传）')
    parser.add_argument('--broker', default=str(defaults['mqtt_broker']))
    parser.add_argument('--port', type=int, default=int(defaults['mqtt_port']))
    parser.add_argument('--username', default=str(defaults['mqtt_username']))
    parser.add_argument('--password', default=str(defaults['mqtt_password']))
    parser.add_argument('--response-topic', default=str(defaults['mqtt_response_topic']))
    parser.add_argument('--sn', default=str(defaults['robot_sn']))
    parser.add_argument('--version', default=str(defaults['version']))
    parser.add_argument('--server-url', default=str(defaults['server_url']))
    parser.add_argument('--dry-run', action='store_true', help='只生成图片并打印报文，不上传/不发布')
    return parser.parse_args()


def main():
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    )
    args = parse_args()
    args.server_url = args.server_url.rstrip('/')
    logger.info('MQTT: %s:%s (user=%s) sn=%s topic=%s server=%s',
                args.broker, args.port, args.username or '<empty>',
                args.sn, args.response_topic, args.server_url)
    if not args.username:
        logger.warning('未提供 MQTT 用户名，若 broker 开启鉴权会连接被拒（rc=4/5）')

    col_radar, row_radar = 0.0, 0.0
    if not (args.pcd_file_id and args.file_id and os.path.isfile(args.image)):
        col_radar, row_radar = build_png_and_pcd(args.pcd, args.image)
    else:
        logger.info('复用已有 png/pcd 文件 ID，跳过点云处理: %s', args.image)

    with open(args.image, 'rb') as fp:
        image_base64 = base64.b64encode(fp.read()).decode('utf-8')
    with Image.open(args.image) as img:
        map_width, map_height = img.size

    auth = None if args.dry_run else AuthManager(args.server_url)

    def _upload(local_path, filename, content_type, timeout):
        token = auth.get_upload_token()
        if not token:
            raise SystemExit(f'获取上传 token 失败，无法上传: {local_path}')
        return upload_file(f'{args.server_url}/file/file/upload/{token}', local_path,
                           filename, content_type, timeout)

    if args.file_id:
        file_id = args.file_id
    elif args.dry_run:
        file_id = '<dry-run-png-id>'
    else:
        file_id = _upload(args.image, f'map_image_{int(time.time() * 1000)}.jpg',
                          'image/jpeg', timeout=10)

    if args.pcd_file_id:
        pcd_file_id = args.pcd_file_id
    elif args.dry_run:
        pcd_file_id = '<dry-run-pcd-id>'
    else:
        pcd_file_id = _upload(PCD_FILE_COMPRESS_PATH,
                              f'point_cloud_{int(time.time() * 1000)}.pcd',
                              'application/octet-stream', timeout=300)

    if not file_id:
        raise SystemExit('地图图片上传失败')
    if not pcd_file_id:
        raise SystemExit('点云文件上传失败')

    body = {
        'taskId': args.task_id,
        'estimatedTime': 0,
        'message': '地图构建完成',
        'progress': 100,
        'fileId': file_id,
        'pcdFileId': pcd_file_id,
        'picBase64': image_base64,
        'marks': [],
        'mapWidth': map_width,
        'mapHeight': map_height,
        'originPoint': {
            'x': col_radar,
            'y': row_radar,
            'z': 0.0,
            'w': 0.0,
        },
    }

    if args.dry_run:
        printable = dict(body)
        printable.pop('picBase64', None)
        logger.info('dry-run，未上传未发布。报文: %s', json.dumps(printable, ensure_ascii=False))
        return

    publish_build_map(args, body)


if __name__ == '__main__':
    main()
