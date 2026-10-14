#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真环境离线航线测试入口（Gazebo + PX4 SITL + ego_planner）。

设计原则：在线文件零改动，流程完全复用
    MQTT on_message -> process_task_message('SETCRUISETASK')
      -> CruiseUavTaskManager.process_cruise_task
      -> _process_set_cruise_task(enable=true) -> 线程 _execute_task_thread
      -> _processing_cruise_task: 逐航点 perform_action_walk_to_pos_3d(/goal_pose_3d)
         + 航点失效分级处置(同点重试>=3跳点 / 连续2点不可达返航 / 返航受阻迫降)
         + endActions(stay/ptz/gpio/snap/algo) -> exec_land_command -> 状态上报

与在线的唯一差别只在“边界适配”，全部通过构造参数注入或模块属性替换实现：
    1. 任务来源: MQTT 报文 -> 本地航点 JSON（拼成与 SETCRUISETASK 同构的报文后仍走
       process_cruise_task，命令分发/校验/线程编排一行未改）
    2. 上报通道: paho MQTT 客户端 -> SimMqttClient（同样的 publish(topic, payload) 接口，
       落盘为 JSONL，便于离线复盘；_report_task_status/send_ack_response 逻辑不变）
    3. 地图同步: MapManager(HTTP 拉图) -> SimMapManager（update_pcd_map 直接返回成功）
    4. 导航服务: sudo systemctl start/stop ego_planner.service -> SimSubprocess 桩
       （只替换 cruise_uav_task_manager 模块内的 subprocess 名字，真 subprocess 不受影响）
    5. 位置来源: 在线由激光定位发布 /pcl_pose；仿真里不造这个话题，直接订阅 Gazebo
       odom 真值(<ns>/odometry)后调用在线的 _pcl_pose_callback 喂位姿
       （SimOdomPoseFeeder），因此 Ros2TopicSubscriber 零改动
    6. 命名空间: /fmu/in|out/... 为绝对名，仿真里载具带 /x500_depth_0/ 前缀，
       由启动文件用 remap 处理（不改代码）

用法（通常由 uav_service/src/task_dispatcher/launch/offline_route_sim.launch.py 调用）：
    python3 offline_cruise_sim_node.py --route routes/sim_route_demo.json --use-sim-time true
"""

import argparse
import importlib.util
import json
import logging
import math
import os
import sys
import threading
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple

# --------------------------------------------------------------------------------------
# 路径与导入引导
# --------------------------------------------------------------------------------------

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
# 本目录 = <repo>/uav_service/src/task_dispatcher/task_dispatcher/offline_sim
# 包内模块目录（存在顶层导入，如 `from uav_command_sender import UavCommandSender`）
MOD_ROOT = os.path.dirname(THIS_DIR)
# 包的父目录（内含 task_dispatcher 包本体与 launch/）
PKG_ROOT = os.path.dirname(MOD_ROOT)
# <repo> = .../XTDrone2_ego_planner
REPO_ROOT = os.path.abspath(os.path.join(PKG_ROOT, '..', '..', '..'))
DEFAULT_ROUTES_DIR = os.path.join(THIS_DIR, 'routes')
# 规划页导出的航点文件常直接放在 launch 目录旁边，裸文件名也能找到
LAUNCH_DIR = os.path.join(PKG_ROOT, 'launch')
ROUTE_SEARCH_DIRS = (THIS_DIR, DEFAULT_ROUTES_DIR, LAUNCH_DIR, REPO_ROOT)
DEFAULT_REPORT_FILE = '/tmp/uav_offline_sim/action_report.jsonl'

logger = logging.getLogger('offline_cruise_sim')


def bootstrap_sys_path() -> None:
    """让在线包既支持 `task_dispatcher.xxx` 包导入，也支持其内部混用的顶层导入。"""
    for p in (MOD_ROOT, PKG_ROOT):
        if p not in sys.path:
            sys.path.insert(0, p)


# 仿真下必须改写的在线配置项（其余沿用 config.py 原值，保证与在线一致）
SIM_CONFIG_OVERRIDES: Dict[str, Any] = {
    'ROBOT_TYPE': 'uav',              # 与在线一致，走无人机分支（含 /fmu/out 状态订阅）
    'ENABLE_PREFLIGHT_CHECK': False,  # 自检依赖机载设备话题
    'ENABLE_DOCK_CONTROL': False,     # 仿真无机舱
    'ENABLE_MANUAL_CONTROL': False,   # 自动执行航线，不等待人工起降
    'CAMERA01_URL': '',               # 无 RTSP 相机
    'CAMERA02_URL': '',
    'PTZ_URL': '',                    # 无云台
    'RECORD_FILE_PATH': '/tmp/uav_offline_sim/rtsp',
    'PCD_FILE_PATH': '/tmp/uav_offline_sim/test.pcd',
    'PCD_FILE_COMPRESS_PATH': '/tmp/uav_offline_sim/test_compress.pcd',
}

# 离线仿真没有文件服务器；保留 URL 形态仅为让在线构造器不报错，实际不会被访问
SIM_SERVER_URL = 'http://127.0.0.1:9'


def install_sim_config(overrides: Dict[str, Any]) -> None:
    """把在线 config.py 原样加载后覆盖仿真项，并同时注册 `task_dispatcher.config` 与
    顶层 `config`（service_manager 等模块用的是 `from config import ROBOT_TYPE`）。

    必须在导入任何 task_dispatcher 模块之前调用，否则 `from .config import X`
    取到的仍是在线值。
    """
    config_path = os.path.join(MOD_ROOT, 'config.py')
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f'未找到在线配置文件: {config_path}')

    for name in ('task_dispatcher.config', 'config'):
        if name in sys.modules:
            del sys.modules[name]

    spec = importlib.util.spec_from_file_location('task_dispatcher.config', config_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules['task_dispatcher.config'] = module
    spec.loader.exec_module(module)
    sys.modules['config'] = module

    for key, value in overrides.items():
        if not hasattr(module, key):
            logger.warning('在线配置缺少字段 %s（忽略，仅覆盖已有字段）', key)
            continue
        old = getattr(module, key)
        setattr(module, key, value)
        if old != value:
            logger.info('仿真配置覆盖: %s: %r -> %r', key, old, value)


# --------------------------------------------------------------------------------------
# 边界桩：MQTT / 地图 / systemd
# --------------------------------------------------------------------------------------


class SimMqttClient:
    """替代 paho MQTT 客户端：保留 publish(topic, payload) 接口，落盘 + 打日志。

    CruiseUavTaskManager 的 _report_task_status / send_ack_response 原样调用本对象，
    因此上报的报文结构、seq 缓存、进度枚举全部与在线一致。
    """

    def __init__(self, report_path: str):
        self.report_path = report_path
        self.published: List[Tuple[str, Dict[str, Any]]] = []
        self._lock = threading.Lock()
        os.makedirs(os.path.dirname(report_path) or '.', exist_ok=True)
        logger.info('离线上报文件: %s', report_path)

    def publish(self, topic: str, payload: Any = None, *args, **kwargs) -> None:
        try:
            data = json.loads(payload) if isinstance(payload, (str, bytes)) else payload
        except Exception:
            data = {'raw': str(payload)}

        with self._lock:
            self.published.append((topic, data))
            with open(self.report_path, 'a', encoding='utf-8') as fp:
                fp.write(json.dumps({'topic': topic, 'payload': data}, ensure_ascii=False) + '\n')

        body = data.get('body', {}) if isinstance(data, dict) else {}
        cmd = data.get('cmd', '?') if isinstance(data, dict) else '?'
        if cmd == 'ActionReport':
            logger.info('[上报] cmd=ActionReport taskId=%s taskProgress=%s msg=%s',
                        body.get('taskId'), body.get('taskProgress'), body.get('message'))
        else:
            logger.info('[上报] cmd=%s status=%s error=%s', cmd,
                        data.get('status') if isinstance(data, dict) else None,
                        data.get('error') if isinstance(data, dict) else None)

    # 以下接口在线由 paho 提供，这里只需存在即可（离线不使用真实 MQTT）
    def subscribe(self, *args, **kwargs) -> None:
        pass

    def loop_start(self) -> None:
        pass

    def loop_stop(self) -> None:
        pass

    def connect(self, *args, **kwargs) -> None:
        pass

    def disconnect(self, *args, **kwargs) -> None:
        pass


class SimMapManager:
    """替代 MapManager：仿真不做 HTTP 拉图，直接宣告地图就绪。"""

    def update_pcd_map(self, map_file_id: str, map_file_uri: str) -> bool:
        logger.info('[离线] 跳过地图同步 mapFileId=%s mapFileUri=%s（仿真使用本地已建地图）',
                    map_file_id, map_file_uri)
        return True

    def handle_map_data(self, *args, **kwargs) -> None:
        pass


class SimVideoPreviewManager:
    """Ros2TopicSubscriber.update_services_and_cameras_status 会无条件读取相机状态。"""

    def get_camera_status(self) -> list:
        return []


class SimOdomPoseFeeder:
    """离线仿真的位置来源：直接订 Gazebo odom 真值，喂进在线的 /pcl_pose 回调。

    在线里位置由激光定位发布 /pcl_pose，Ros2TopicSubscriber 订阅后写入 _robot_state['position']，
    供 PerformActionManager 判到达/算停滞。仿真里没有定位节点，也不需要造 /pcl_pose 话题：
    这里把 odom 真值组装成同一条 PoseWithCovarianceStamped，直接调用在线那个回调函数，
    于是“判到达 + yaw + 停滞 watchdog + 上报”走的仍是同一段代码（在线文件零改动）。

    QoS 与 tf_publisher 订同一话题时保持一致（默认 depth=10），已在同套仿真里验证可用。
    """

    def __init__(self, node, topic_subscriber, odom_topic: str, frame_id: str = 'map'):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry

        self._topic_subscriber = topic_subscriber
        self._frame_id = frame_id
        self._msg_type = PoseWithCovarianceStamped
        self._count = 0
        self._last_pos = None
        self.sub = node.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        logger.info('位姿来源: 直接订阅 odom 真值 %s -> 在线 _pcl_pose_callback（不发布 /pcl_pose）',
                    odom_topic)

    def _on_odom(self, msg) -> None:
        pose_msg = self._msg_type()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = self._frame_id
        pose_msg.pose.pose = msg.pose.pose
        # 复用在线回调：位置、yaw(由四元数取欧拉角)、锁与状态上报路径全部一致
        self._topic_subscriber._pcl_pose_callback(pose_msg)

        self._count += 1
        p = msg.pose.pose.position
        self._last_pos = (p.x, p.y, p.z)
        if self._count == 1 or self._count % 100 == 0:
            logger.info('odom 真值已喂入 %d 帧, 当前位置 x=%.2f y=%.2f z=%.2f',
                        self._count, p.x, p.y, p.z)


class SimCompletedProcess:
    returncode = 0
    stdout = ''
    stderr = ''


class SimPopen:
    """记录被跳过的 systemd 调用，不真正执行。"""

    def __init__(self, cmd: Any):
        self.cmd = cmd
        logger.info('[离线] 跳过进程调用: %s（仿真导航服务由启动文件负责）', cmd)
        self.returncode = 0

    def poll(self) -> int:
        return 0

    def wait(self, timeout: Optional[float] = None) -> int:
        return 0

    def communicate(self, *args, **kwargs) -> Tuple[str, str]:
        return '', ''

    def terminate(self) -> None:
        pass


class SimSubprocess:
    """替换 cruise_uav_task_manager 模块内的 subprocess 名字。

    在线的 `_execute_task_thread` 会 `sudo systemctl start/stop ego_planner.service`，
    在仿真实机上既无该 unit 也需要 sudo 口令，因此桩掉；其余真实 subprocess 行为
    （如 service_manager 的 `systemctl list-units`，属只读命令）不受影响。
    """

    CompletedProcess = SimCompletedProcess

    @staticmethod
    def run(cmd: Any, *args, **kwargs) -> SimCompletedProcess:
        logger.info('[离线] 跳过 subprocess.run: %s（仿真导航服务由启动文件负责）', cmd)
        return SimCompletedProcess()

    @staticmethod
    def Popen(cmd: Any, *args, **kwargs) -> SimPopen:
        return SimPopen(cmd)

    @staticmethod
    def check_output(cmd: Any, *args, **kwargs) -> bytes:
        logger.info('[离线] 跳过 subprocess.check_output: %s', cmd)
        return b''


# --------------------------------------------------------------------------------------
# 航点配置读取（自动识别两种格式）
# --------------------------------------------------------------------------------------


def _pos_of(item: Dict[str, Any], idx: int) -> Tuple[Dict[str, float], bool]:
    """把一条航点记录归一化成在线 pos 结构 {x,y,z,d}。

    返回 (pos, 是否显式给了方向)。方向支持 d/yaw/heading 三种键名（弧度）；
    没给就不写 d，交给 compute_route_yaws 按航线几何补上。
    """
    explicit = any(item.get(k) is not None for k in ('d', 'yaw', 'heading'))
    pos: Dict[str, float] = {
        'x': float(item.get('x', 0.0)),
        'y': float(item.get('y', 0.0)),
        'z': float(item.get('z', 0.0)),
    }
    if explicit:
        pos['d'] = float(item.get('d', item.get('yaw', item.get('heading'))))
    if pos['z'] <= 0.0:
        logger.warning('航点 %s 高度 z=%s <= 0，仿真中会被视为地面/下降目标，请确认', idx, pos['z'])
    return pos, explicit


def compute_route_yaws(points: List[Dict[str, float]], mode: str = 'smooth',
                       fixed_rad: float = 0.0) -> List[float]:
    """按航线几何推导每个航点的机头朝向（弧度，[-pi, pi]）。

    为什么需要：在线 pos 必须含 d，且 xtd2_traj_server 会在“距目标 2m 内/轨迹结束时”
    把机头转到目标四元数的 yaw。若统一填 0，每个航点都会把机头拧回 +X。

      smooth   : Catmull-Rom 中心切线 (P[k+1]-P[k-1])，端点重复自身——朝向沿曲线平滑过渡
      tangent  : 来向切线 (P[k]-P[k-1])，首点用第一段方向（与 RViz goal_pose_marker 同一做法）
      zero     : 全 0（保持 +X）
      fixed    : 全部用 --yaw-fixed
    """
    n = len(points)
    if n == 0:
        return []
    if mode == 'zero':
        return [0.0] * n
    if mode == 'fixed':
        return [float(fixed_rad)] * n

    yaws: List[float] = []
    last_valid = float(fixed_rad)
    for k in range(n):
        if mode == 'tangent':
            prev = points[k - 1] if k > 0 else points[0]
            cur = points[k] if k > 0 else points[1] if n > 1 else points[0]
            if k == 0 and n > 1:
                prev, cur = points[0], points[1]
            dx, dy = cur['x'] - prev['x'], cur['y'] - prev['y']
        else:  # smooth
            p_prev = points[k - 1] if k > 0 else points[0]
            p_next = points[k + 1] if k < n - 1 else points[n - 1]
            dx, dy = p_next['x'] - p_prev['x'], p_next['y'] - p_prev['y']

        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            yaws.append(last_valid)          # 重合点：沿用上一有效朝向
            continue
        yaw = math.atan2(dy, dx)
        last_valid = yaw
        yaws.append(yaw)
    return yaws


def resolve_route_path(name: str) -> Optional[str]:
    """解析航点文件路径：原样/绝对路径优先，再在 offline_sim、routes、launch、仓库根里找裸文件名。"""
    if os.path.isfile(name):
        return os.path.abspath(name)
    for base in ROUTE_SEARCH_DIRS:
        candidate = os.path.join(base, name)
        if os.path.isfile(candidate):
            return candidate
    return None


def load_route_file(path: str, yaw_mode: str = 'smooth',
                    yaw_fixed_deg: float = 0.0) -> Tuple[Dict[str, Any], str]:
    """读取航点配置，返回 (SETCRUISETASK 的 body, 来源格式说明)。

    支持：
      A) web PCD 航点规划页导出：{"frame":"map","count":N,"waypoints":[{index,x,y,z[,d]}]}
         —— 页面只给坐标，这里自动按 yaw_mode 补方向（写了 d/yaw/heading 的以文件为准）
      B) 完整任务 body（与在线 MQTT body 同构，含 cruises/ingActions/endActions）
      C) 整条在线报文 {"cmd":"SetCruiseTask","body":{...}}（取其中 body）
    文件内可写 "yaw_mode" / "yaw_fixed_deg"，优先级高于命令行。
    """
    with open(path, 'r', encoding='utf-8') as fp:
        raw = json.load(fp)

    if not isinstance(raw, dict):
        raise ValueError('航点配置根节点必须是对象')

    if 'body' in raw and isinstance(raw['body'], dict):
        body = dict(raw['body'])
        source = '在线完整报文(cmd=%s)' % raw.get('cmd', 'SetCruiseTask')
    elif 'cruises' in raw:
        body = dict(raw)
        source = '完整 cruises 任务配置'
    elif 'waypoints' in raw:
        source = 'web 航点规划页导出(waypoints)'
        legs: List[Dict[str, Any]] = []
        for i, w in enumerate(raw['waypoints']):
            if not isinstance(w, dict):
                raise ValueError(f'waypoints[{i}] 必须是对象')
            pos, explicit = _pos_of(w, i + 1)
            legs.append({
                'idx': int(w.get('index', w.get('idx', i + 1))),
                'pos': pos,
                '_explicit_yaw': explicit,
                'ingActions': w.get('ingActions', []),
                'endActions': w.get('endActions', []),
            })
        body = dict(raw)
        body['cruises'] = legs
    else:
        raise ValueError('无法识别的航点配置：需要 waypoints / cruises / body 之一')

    # 配置项优先级：文件 > 命令行
    mode = str(body.pop('yaw_mode', None) or yaw_mode).lower()
    fixed_deg = body.pop('yaw_fixed_deg', None)
    fixed_deg = yaw_fixed_deg if fixed_deg is None else float(fixed_deg)
    if mode not in ('smooth', 'tangent', 'zero', 'fixed'):
        raise ValueError(f'未知 yaw_mode={mode}（可选 smooth/tangent/zero/fixed）')

    for key in [k for k in list(body.keys()) if k.startswith('_')]:
        body.pop(key)          # 以 _ 开头的键是注释/用法说明，不进入报文
    for key in ('waypoints', 'frame', 'count'):
        body.pop(key, None)    # 格式A 的原始航点字段已转成 cruises

    # 补齐在线 _process_set_cruise_task 的必填字段（缺失才补，用户值优先）
    defaults = {
        'taskName': 'offline-sim-route',
        'taskId': f'offline_sim_{int(time.time())}',
        'type': 'once',
        'mapName': 'sim',
        'mapFileId': 'sim-local',
        'mapFileUri': 'sim-local',
        'savePointCloud': False,
        'enable': True,
    }
    for key, value in defaults.items():
        body.setdefault(key, value)

    if not isinstance(body.get('cruises'), list) or not body['cruises']:
        raise ValueError('cruises 为空，无可执行航点')

    # 归一化 pos 并记录哪些航点需要自动补方向
    points: List[Dict[str, float]] = []
    need_yaw: List[int] = []
    for i, cruise in enumerate(body['cruises']):
        if not isinstance(cruise, dict) or 'pos' not in cruise:
            raise ValueError('每个航点必须包含 pos 字段')
        pos = cruise['pos']
        if not isinstance(pos, dict):
            raise ValueError(f'cruises[{i}].pos 必须是对象')
        explicit = bool(cruise.pop('_explicit_yaw', False))
        for field in ('x', 'y', 'z'):
            if field not in pos:
                raise ValueError(f'cruises[{i}].pos 缺少字段 {field}')
        if 'd' in pos:
            pos['d'] = float(pos['d'])
            explicit = True
        else:
            pos['d'] = 0.0
        cruise['pos'] = pos
        points.append(pos)
        if not explicit:
            need_yaw.append(i)
        cruise.setdefault('idx', i + 1)

    if need_yaw:
        yaws = compute_route_yaws(points, mode=mode, fixed_rad=math.radians(fixed_deg))
        for i in need_yaw:
            body['cruises'][i]['pos']['d'] = yaws[i]
        logger.info('方向补全: yaw_mode=%s 自动补 %d/%d 个航点（其余为文件显式值）',
                    mode, len(need_yaw), len(body['cruises']))
        if fixed_deg:
            logger.info('fixed 方向=%.1f°', fixed_deg)

    body['savePointCloud'] = bool(body.get('savePointCloud', False))
    return body, source


# --------------------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------------------


def build_managers(node, report_file: str) -> Tuple[Any, Any]:
    """按在线 TaskDispatcherNode 的装配顺序构造管理器（同一批类、同一条依赖链）。"""
    from task_dispatcher.cruise_uav_task_manager import CruiseUavTaskManager
    from task_dispatcher.perform_action_manager import PerformActionManager
    from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
    import task_dispatcher.cruise_uav_task_manager as cruise_module

    # 旁路 systemd（仅替换该模块内的名字引用）
    cruise_module.subprocess = SimSubprocess()

    mqtt_stub = SimMqttClient(report_file)
    ros_topic_subscriber = Ros2TopicSubscriber(
        node,
        mqtt_client=mqtt_stub,
        enable_status_report=False,   # 离线不向 MQTT 周期上报整机状态
        robot_sn='SIM-UAV-00001',
        mqtt_response_topic='offline/sim/up',
        version='1.0',
        video_preview_manager=SimVideoPreviewManager(),
    )
    perform_action_manager = PerformActionManager(
        node=node,
        ptz_controller=None,                # 仿真无云台；ptz 动作会按 onFailed 处理
        topic_subscriber=ros_topic_subscriber,
    )
    task_manager = CruiseUavTaskManager(
        node=node,
        mqtt_client=mqtt_stub,
        response_topic='offline/sim/up',
        ros_topic_subscriber=ros_topic_subscriber,
        version='1.0',
        robot_sn='SIM-UAV-00001',
        ptz_controller=None,
        server_url=SIM_SERVER_URL,
        perform_action_manager=perform_action_manager,
        map_manager=SimMapManager(),
    )
    # 离线无文件服务器：停掉点云补传定时器，避免持续网络重试噪声
    try:
        task_manager.point_cloud_persistor.stop_timer()
    except Exception as exc:  # pragma: no cover - 仅防御
        logger.warning('停止点云补传定时器失败: %s', exc)

    return task_manager, ros_topic_subscriber


def wait_until_task_finished(task_manager, timeout: float = 0.0) -> bool:
    """等待航线线程结束（timeout<=0 表示不限时）。"""
    start = time.time()
    while task_manager.is_executing and rclpy_ok():
        if timeout > 0 and time.time() - start > timeout:
            logger.warning('航线执行超时(%.0fs)仍未结束', timeout)
            return False
        time.sleep(0.5)
    return True


# --------------------------------------------------------------------------------------
# 结束判定：离线测试要能当回归用，“假通过”比不通过更糟
# --------------------------------------------------------------------------------------

TASK_PROGRESS_NAME = {0: 'STARTED', 1: 'IN_PROGRESS', 2: 'COMPLETED', 3: 'ABORTED', 4: 'CANCELED'}


def _last_report(mqtt_client, cmd: str) -> Optional[Dict[str, Any]]:
    """取 SimMqttClient 里最后一条指定 cmd 的上报报文（Ack / ActionReport）。"""
    for _, data in reversed(list(getattr(mqtt_client, 'published', []) or [])):
        if isinstance(data, dict) and data.get('cmd') == cmd:
            return data
    return None


def wait_task_accepted(task_manager, timeout: float = 5.0) -> Tuple[bool, str]:
    """确认在线链路真的收下了任务：先有 Ack(200)，再看到执行痕迹。

    必须查的原因：process_cruise_task 没有返回值，任务被在线校验拒绝时
    is_executing 从未置真，wait_until_task_finished 会“秒过”并被当成正常完成退出。
    """
    client = task_manager.mqtt_client
    deadline = time.time() + timeout
    ack = None
    while True:
        ack = _last_report(client, 'Ack')       # 先查一次，避免条件把首轮整个跳过
        if ack or not rclpy_ok() or time.time() >= deadline:
            break
        time.sleep(0.2)
    if not ack:
        return False, '下发后 %.0fs 内没有任何 Ack 上报（命令没进入分发）' % timeout
    status = ack.get('status')
    if status != 200:
        return False, '在线校验拒绝: status=%s error=%s' % (status, ack.get('error'))

    deadline = time.time() + timeout
    while True:
        if task_manager.is_executing or _last_report(client, 'ActionReport'):
            return True, 'Ack=200，任务已进入执行'
        if not rclpy_ok() or time.time() >= deadline:
            break
        time.sleep(0.2)
    return False, 'Ack=200 但既未 is_executing 也无 ActionReport，任务实际未启动'


def final_exit_code(task_manager) -> int:
    """按最后一条 ActionReport 的 taskProgress 定退出码：2=完成->0，3/4=中断/取消->7。

    在线的正常成功路径末尾会上报 COMPLETED(2)，异常/返航/迫降分支上报 ABORTED(3)。
    """
    rep = _last_report(task_manager.mqtt_client, 'ActionReport')
    if not rep:
        logger.error('全程没有任何 ActionReport 上报，按失败退出')
        return 7
    body = rep.get('body') if isinstance(rep.get('body'), dict) else {}
    progress = body.get('taskProgress')
    name = TASK_PROGRESS_NAME.get(progress, '未知(%r)' % progress)
    logger.info('终态上报: taskProgress=%s(%s) message=%s', progress, name, body.get('message'))
    if progress == 2:
        return 0
    if progress in (3, 4):
        logger.error('任务未正常完成: %s (%s)', name, body.get('message'))
        return 7
    logger.warning('终态为 %s（非 COMPLETED），退出码仍记 0，请结合上报文件确认', name)
    return 0


def rclpy_ok() -> bool:
    import rclpy
    return rclpy.ok()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='仿真离线航线测试（复用在线任务调度流程）')
    parser.add_argument('--route', required=True,
                        help='航点配置文件（支持 web 导出的 waypoints 格式 / 完整 cruises 格式）')
    parser.add_argument('--yaw-mode', default='smooth', choices=['smooth', 'tangent', 'zero', 'fixed'],
                        help='航点没写 d 时的方向补全方式：smooth=Catmull-Rom 曲线切线（默认）、'
                             'tangent=来向切线、zero=恒朝 +X、fixed=统一 --yaw-fixed')
    parser.add_argument('--yaw-fixed', type=float, default=0.0, metavar='DEG',
                        help='yaw-mode=fixed 时统一的机头朝向（度）')
    parser.add_argument('--task-id', default='', help='任务ID，默认自动生成 offline_sim_<时间戳>')
    parser.add_argument('--use-sim-time', default='true', choices=['true', 'false'],
                        help='是否使用 Gazebo 仿真时钟（仿真必须 true，否则目标点时间戳与规划器不一致）')
    parser.add_argument('--report-file', default=DEFAULT_REPORT_FILE,
                        help='离线上报报文落盘路径(JSONL)')
    parser.add_argument('--feed-odom-pose', default='true', choices=['true', 'false'],
                        help='位置来源：true=直接订阅 Gazebo odom 真值喂给在线 _pcl_pose 回调'
                             '（离线默认，不依赖 /pcl_pose 话题）；false=沿用外部已发布的 /pcl_pose')
    parser.add_argument('--odom-topic', default='/x500_depth_0/odometry',
                        help='Gazebo 真值里程计话题（world -> base_footprint）')
    parser.add_argument('--pose-frame-id', default='map',
                        help='喂给在线回调的坐标系名（与在线一致用 map）')
    parser.add_argument('--max-wait', type=float, default=0.0,
                        help='航线最长等待秒数，0=不限；超时则退出码 3 结束（配合 --exit-on-finish '
                             '关闭会话），用于兜底降落结果一直不确认的卡死场景')
    parser.add_argument('--exit-on-finish', default='true', choices=['true', 'false'],
                        help='航线结束后是否退出节点（默认 true；启动文件会据此收掉整个会话）')
    parser.add_argument('--remap', action='append', default=[], metavar='FROM:=TO',
                        help='话题重映射（仿真载具带命名空间时用，可重复）。'
                             '例: --remap /fmu/in/vehicle_command:=/x500_depth_0/fmu/in/vehicle_command')
    parser.add_argument('--dry-run', action='store_true',
                        help='只打印将要下发的 SETCRUISETASK 报文，不启动任务')
    return parser.parse_args()


def parse_remaps(items: List[str]) -> List[Tuple[str, str]]:
    """把 `--remap /a:=/ns/a` 解析成 rclpy Node 需要的 (源, 目标) 列表。"""
    rules: List[Tuple[str, str]] = []
    for item in items or []:
        if ':=' not in item:
            logger.warning('忽略非法 remap（缺少 :=）: %s', item)
            continue
        src, dst = item.split(':=', 1)
        rules.append((src.strip(), dst.strip()))
    return rules


def main() -> int:
    args = parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s] [%(name)s] [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
    )

    bootstrap_sys_path()
    install_sim_config(SIM_CONFIG_OVERRIDES)

    route_path = resolve_route_path(args.route)
    if not route_path:
        logger.error('找不到航点配置: %s（裸文件名会在这些目录里找: %s）',
                     args.route, ', '.join(ROUTE_SEARCH_DIRS))
        return 2

    body, source = load_route_file(route_path, yaw_mode=args.yaw_mode,
                                   yaw_fixed_deg=args.yaw_fixed)
    task_id = args.task_id or f"offline_sim_{int(time.time())}"
    body['taskId'] = task_id

    message_data = {
        'sn': 'SIM-UAV-00001',
        'ver': '1.0',
        'seq': str(uuid.uuid4()).replace('-', ''),
        'type': 1,
        'ts': int(time.time() * 1000),
        'cmd': 'SETCRUISETASK',
        'body': body,
    }
    logger.info('航点来源=%s 文件=%s 航点数=%d taskId=%s',
                source, os.path.basename(route_path), len(body['cruises']), task_id)
    for cruise in body['cruises']:
        pos = cruise['pos']
        logger.info('  航点 idx=%s (x=%.2f, y=%.2f, z=%.2f) d=%.3f rad (%.1f°) endActions=%d',
                    cruise.get('idx'), pos['x'], pos['y'], pos['z'], pos['d'],
                    math.degrees(pos['d']), len(cruise.get('endActions', [])))

    if args.dry_run:
        logger.info('[dry-run] 报文如下：\n%s', json.dumps(message_data, ensure_ascii=False, indent=2))
        return 0

    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.parameter import Parameter

    rclpy.init()
    use_sim_time = args.use_sim_time.lower() == 'true'
    remappings = parse_remaps(args.remap)
    # Humble 的 rclpy.node.Node 没有 remappings 参数，重映射走 rcl 的 -r 命令行规则
    cli_args: List[str] = []
    for src, dst in remappings:
        cli_args += ['-r', f'{src}:={dst}']
    if remappings:
        logger.info('话题重映射: %s', ', '.join(f'{a}->{b}' for a, b in remappings))
    # 目标点 PoseStamped 的时标必须与规划器一致，仿真里要跟 Gazebo 时钟
    node = Node('task_dispatcher_sim',
                cli_args=cli_args,
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])
    logger.info('use_sim_time=%s（%s）', use_sim_time, 'Gazebo 仿真时钟' if use_sim_time else '系统时钟')

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_stop = threading.Event()

    def spin_loop():
        while rclpy.ok() and not spin_stop.is_set():
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                break
            except Exception as exc:  # pragma: no cover - 防御
                logger.error('执行器异常: %s', exc)
                time.sleep(0.2)

    spin_thread = threading.Thread(target=spin_loop, daemon=True)
    spin_thread.start()

    task_manager = None
    pose_feeder = None
    exit_code = 0
    try:
        task_manager, ros_topic_subscriber = build_managers(node, args.report_file)

        # 位置来源：离线仿真不发布 /pcl_pose，直接把 Gazebo odom 真值喂进在线回调
        if args.feed_odom_pose.lower() == 'true':
            pose_feeder = SimOdomPoseFeeder(node, ros_topic_subscriber,
                                           args.odom_topic, args.pose_frame_id)
            # 先拿到至少一帧位姿再下发任务：否则位置恒为 (0,0,0)，停滞 watchdog 会立刻误判不可达
            deadline = time.time() + 15.0
            while pose_feeder._count == 0 and time.time() < deadline and rclpy_ok():
                time.sleep(0.2)
            if pose_feeder._count == 0:
                logger.error('15 秒内没收到 %s 的任何一帧位姿，中止（请确认仿真已启动）',
                             args.odom_topic)
                return 5
            logger.info('位姿已就绪: %s 当前位姿=%s', args.odom_topic, pose_feeder._last_pos)
        else:
            logger.info('位姿来源=外部 /pcl_pose（--feed-odom-pose false）')

        # 与在线完全一致的入口：走命令分发 -> 校验 -> 线程编排
        task_manager.process_cruise_task(message_data)

        accepted, reason = wait_task_accepted(task_manager)
        if not accepted:
            logger.error('任务受理检查不通过: %s', reason)
            exit_code = 6
        else:
            logger.info('任务受理检查通过: %s', reason)
            if not wait_until_task_finished(task_manager, timeout=args.max_wait):
                exit_code = 3      # 超时未结束（--max-wait 兜底），退出码 3
            else:
                logger.info('航线任务结束，上报统计: %s 条（文件: %s）',
                            len(task_manager.mqtt_client.published), args.report_file)
                exit_code = final_exit_code(task_manager)
    except KeyboardInterrupt:
        logger.warning('收到中断信号')
        if task_manager is not None and task_manager.is_executing:
            task_manager.process_cruise_task({'cmd': 'STOPCRUISETASK', 'body': {'taskId': task_id}})
            time.sleep(1.0)
        exit_code = 130
    except Exception as exc:
        logger.exception('离线航线执行异常: %s', exc)
        exit_code = 4
    finally:
        if args.exit_on_finish.lower() == 'true':
            spin_stop.set()
            try:
                executor.shutdown()
            except Exception:
                pass
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        else:
            logger.info('航线已结束，保持节点在线（--exit-on-finish false）')
            spin_thread.join()

    return exit_code


if __name__ == '__main__':
    sys.exit(main())
