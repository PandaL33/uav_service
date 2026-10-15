# -*- coding: utf-8 -*-
"""端点参数本地覆盖加载器（launch/task_dispatcher.launch.py 与
launch/online_sim_control.launch.py 共用；凭证不入库）。

查找顺序（先命中先返回）：
    1. 环境变量 UAV_ENDPOINT_CONFIG 指向的文件
    2. 仓内 uav_service/endpoint.local.json（由 launch 目录上溯三级，已在 .gitignore）
    3. install 空间 share/task_dispatcher/endpoint.local.json（launch 目录上一级）
    4. ~/.uav_service/endpoint.json

文件格式（JSON 对象，键与 launch 参数同名，缺项各自回落内置默认）：
    {
      "mqtt_broker": "<平台broker域名>",
      "mqtt_port": "10883",
      "mqtt_username": "<账号>",
      "mqtt_password": "<密码>",
      "mqtt_topic": "/UAV/<SN>/dn",
      "mqtt_response_topic": "/robot/up",
      "mqtt_client_id": "<客户端ID>",
      "robot_sn": "<SN>",
      "server_url": "https://<平台地址>"
    }
"""

import json
import os

VALID_KEYS = ('mqtt_broker', 'mqtt_port', 'mqtt_username', 'mqtt_password', 'mqtt_topic',
              'mqtt_response_topic', 'mqtt_client_id', 'robot_sn', 'server_url')


def candidate_paths(launch_dir: str) -> list:
    env = os.environ.get('UAV_ENDPOINT_CONFIG', '').strip()
    return [
        env or None,
        os.path.normpath(os.path.join(launch_dir, '..', '..', '..', 'endpoint.local.json')),
        os.path.normpath(os.path.join(launch_dir, '..', 'endpoint.local.json')),
        os.path.join(os.path.expanduser('~'), '.uav_service', 'endpoint.json'),
    ]


def load_endpoint_overlay(launch_dir: str) -> dict:
    """返回覆盖字典（只收 VALID_KEYS）；任何文件不存在/解析失败都安全回落空字典。"""
    for path in candidate_paths(launch_dir):
        if not path or not os.path.isfile(path):
            continue
        try:
            with open(path, 'r', encoding='utf-8') as fp:
                data = json.load(fp)
        except (OSError, ValueError) as exc:
            print(f'[endpoint_config] 读取失败，忽略 {path}: {exc}')
            continue
        if not isinstance(data, dict):
            print(f'[endpoint_config] {path} 不是 JSON 对象，忽略')
            continue
        overlay = {k: data[k] for k in VALID_KEYS if data.get(k) not in (None, '')}
        print(f'[endpoint_config] 端点覆盖已加载: {path}（{len(overlay)} 项）')
        return overlay
    print('[endpoint_config] 未找到端点覆盖文件，使用 launch 内置默认值')
    return {}
