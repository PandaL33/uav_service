#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真适配共用件：离线航线测试节点与“在线节点 + 仿真环境”启动方案共用本模块。

为什么要有这个文件（多入口一致性约束）：
    仿真侧存在多个入口——离线测试节点（offline_cruise_sim_node.py）、
    在线控制仿真的启动文件（launch/online_sim_control.launch.py，内含位姿桥
    sim_pcl_pose_bridge.py 与命令桥 sim_uav_command_bridge.py）。
    话题名、QoS、/fmu 重映射表、报文结构、配置覆盖这些“边界适配”如果各写一份，
    很容易出现“仿真测过的行为 != 真机链路”的漂移。这里作为唯一出处：
    所有入口引用同一份常量与实现，真机在线代码（task_dispatcher 包）依旧零改动。

内容一览：
    - 路径引导与在线配置覆盖            bootstrap_sys_path / install_sim_config
    - 边界桩（MQTT/地图/systemd）        SimMqttClient / SimMapManager / SimSubprocess
    - 位置来源                          SimOdomPoseFeeder（离线直喂回调）
                                          wrap_odom_to_pcl_pose + pcl_pose_qos（桥节点发 /pcl_pose）
    - 话题/命名空间/重映射单一事实来源    ABSOLUTE_FMU_TOPICS / fmu_remap_* / truth_odom_topic ...
    - 平台即时指令注入共用件             wrap_platform_command / make_platform_command_callback
"""

import importlib.util
import json
import logging
import os
import sys
import threading
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple

# --------------------------------------------------------------------------------------
# 路径与导入引导
# --------------------------------------------------------------------------------------

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
# 本目录（realpath 后） = <repo>/uav_service/src/task_dispatcher/task_dispatcher/offline_sim
# （symlink-install 时解析回源树；copy 安装时位于 share/task_dispatcher/offline_sim）
# 包内模块目录（存在顶层导入，如 `from uav_command_sender import UavCommandSender`）
MOD_ROOT = os.path.dirname(THIS_DIR)
# 包的父目录（内含 task_dispatcher 包本体与 launch/）
PKG_ROOT = os.path.dirname(MOD_ROOT)
# <repo> = .../XTDrone2_ego_planner
REPO_ROOT = os.path.abspath(os.path.join(PKG_ROOT, '..', '..', '..'))

logger = logging.getLogger('sim_support')


def bootstrap_sys_path() -> None:
    """让在线包既支持 `task_dispatcher.xxx` 包导入，也支持其内部混用的顶层导入。"""
    for p in (MOD_ROOT, PKG_ROOT):
        if p not in sys.path:
            sys.path.insert(0, p)


# --------------------------------------------------------------------------------------
# 在线配置覆盖（其余字段沿用 config.py 原值，保证与在线一致）
# --------------------------------------------------------------------------------------

SIM_CONFIG_OVERRIDES: Dict[str, Any] = {
    'ROBOT_TYPE': 'uav',              # 与在线一致，走无人机分支（含 /fmu/out 状态订阅）
    'ENABLE_PREFLIGHT_CHECK': False,  # 自检依赖机载设备话题
    'ENABLE_DOCK_CONTROL': False,     # 仿真无机舱
    'ENABLE_MANUAL_CONTROL': False,   # 自动执行航线，不等待人工起降
    'CAMERA01_URL': '',               # 无 RTSP 相机
    'CAMERA02_URL': '',
    'ENABLE_VIDEO': False,            # 仿真无相机，关闭视频预览/录制，避免对空 URL 探测刷屏
    'PTZ_URL': '',                    # 无云台
    'RECORD_FILE_PATH': '/tmp/uav_offline_sim/rtsp',
    'PCD_FILE_PATH': '/tmp/uav_offline_sim/test.pcd',
    'PCD_FILE_COMPRESS_PATH': '/tmp/uav_offline_sim/test_compress.pcd',
}

# 离线仿真没有文件服务器；保留 URL 形态仅为让在线构造器不报错，实际不会被访问
SIM_SERVER_URL = 'http://127.0.0.1:9'


def _config_source_candidates() -> list:
    """config.py 的可能位置：源树（含 symlink-install realpath 回到源树）优先，
    其次是 colcon 已安装的 task_dispatcher 包内文件（copy 安装场景）。"""
    candidates = [os.path.join(MOD_ROOT, 'config.py')]
    try:
        import importlib.util
        spec = importlib.util.find_spec('task_dispatcher')
        if spec and spec.submodule_search_locations:
            root = list(spec.submodule_search_locations)[0]
            candidates.append(os.path.join(os.path.realpath(root), 'config.py'))
    except (ImportError, ValueError):
        pass
    return candidates


def install_sim_config(overrides: Dict[str, Any] = None) -> None:
    """把在线 config.py 原样加载后覆盖仿真项，并同时注册 `task_dispatcher.config` 与
    顶层 `config`（service_manager 等模块用的是 `from config import ROBOT_TYPE`）。

    必须在导入任何 task_dispatcher 模块之前调用，否则 `from .config import X`
    取到的仍是在线值。
    """
    if overrides is None:
        overrides = SIM_CONFIG_OVERRIDES
    config_path = next((p for p in _config_source_candidates() if os.path.isfile(p)), None)
    if config_path is None:
        raise FileNotFoundError('未找到在线配置文件 config.py（尝试过: %s）'
                                % ', '.join(_config_source_candidates()))

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
# 边界桩：MQTT / 地图 / systemd / 相机
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
# 话题 / 命名空间 / 重映射：单一事实来源
# --------------------------------------------------------------------------------------

# 在线代码以绝对名使用的飞控话题：仿真里需要加载具命名空间前缀
ABSOLUTE_FMU_TOPICS = [
    '/fmu/in/vehicle_command',
    '/fmu/out/vehicle_status',
    '/fmu/out/battery_status',
    '/fmu/out/failsafe_flags',
    '/fmu/out/vehicle_global_position',
    '/fmu/out/vehicle_local_position',
    '/fmu/out/vehicle_attitude',
]


def ns_prefix(namespace: str) -> str:
    """'/x500_depth_0/' -> '/x500_depth_0'；空/None -> ''。"""
    base_ns = (namespace or '').strip('/')
    return f'/{base_ns}' if base_ns else ''


def base_namespace(namespace: str) -> str:
    return (namespace or '').strip('/')


def truth_odom_topic(namespace: str, override: str = '') -> str:
    """Gazebo 真值里程计话题（world -> base_footprint）。"""
    if override:
        return override
    prefix = ns_prefix(namespace)
    return f'{prefix}/odometry' if prefix else '/odometry'


def px4_odom_topic(namespace: str) -> str:
    prefix = ns_prefix(namespace)
    return f'{prefix}/fmu/out/vehicle_odometry' if prefix else '/fmu/out/vehicle_odometry'


def cmd_service_name(namespace: str) -> str:
    base_ns = base_namespace(namespace)
    return f'/xtdrone2/{base_ns}/cmd' if base_ns else '/xtdrone2/cmd'


def fmu_remap_tuples(namespace: str) -> List[Tuple[str, str]]:
    """launch_ros Node(remappings=...) 用的 (源, 目标) 列表。"""
    prefix = ns_prefix(namespace)
    if not prefix:
        return []
    return [(topic, f'{prefix}{topic}') for topic in ABSOLUTE_FMU_TOPICS]


def fmu_remap_cli_args(namespace: str) -> List[str]:
    """ExecuteProcess 用的 `--remap a:=b` 参数列表（rcl 命令行规则）。"""
    args: List[str] = []
    for src, dst in fmu_remap_tuples(namespace):
        args += ['--remap', f'{src}:={dst}']
    return args


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


# --------------------------------------------------------------------------------------
# 位置来源：Gazebo odom 真值 -> 在线 /pcl_pose 链路
# --------------------------------------------------------------------------------------


def wrap_odom_to_pcl_pose(msg, frame_id: str = 'map'):
    """把 nav_msgs/Odometry 组装成在线定位节点同款的 PoseWithCovarianceStamped。

    在线 Ros2TopicSubscriber._pcl_pose_callback 只读 pose.pose 的位置与四元数，
    这里保持同样的载荷形态，两条仿真路径共用：
      - 离线节点：直接调用在线回调（SimOdomPoseFeeder）
      - 在线节点 + 仿真：由 sim_pcl_pose_bridge.py 发布到 /pcl_pose
    """
    from geometry_msgs.msg import PoseWithCovarianceStamped

    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = msg.header.stamp
    pose_msg.header.frame_id = frame_id
    pose_msg.pose.pose = msg.pose.pose
    return pose_msg


def pcl_pose_qos():
    """与在线订阅 /pcl_pose 的 amcl_qos 一致：RELIABLE + TRANSIENT_LOCAL + depth=1。"""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1,
    )


class SimOdomPoseFeeder:
    """离线仿真的位置来源：直接订 Gazebo odom 真值，喂进在线的 /pcl_pose 回调。

    在线里位置由激光定位发布 /pcl_pose，Ros2TopicSubscriber 订阅后写入 _robot_state['position']，
    供 PerformActionManager 判到达/算停滞。仿真里没有定位节点，也不需要造 /pcl_pose 话题：
    这里把 odom 真值组装成同一条 PoseWithCovarianceStamped，直接调用在线那个回调函数，
    于是“判到达 + yaw + 停滞 watchdog + 上报”走的仍是同一段代码（在线文件零改动）。

    在线节点跑仿真时改走 sim_pcl_pose_bridge.py 发 /pcl_pose（换算与 QoS 共用上面的函数）。
    """

    def __init__(self, node, topic_subscriber, odom_topic: str, frame_id: str = 'map'):
        from nav_msgs.msg import Odometry

        self._topic_subscriber = topic_subscriber
        self._frame_id = frame_id
        self._count = 0
        self._last_pos = None
        self.sub = node.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        logger.info('位姿来源: 直接订阅 odom 真值 %s -> 在线 _pcl_pose_callback（不发布 /pcl_pose）',
                    odom_topic)

    def _on_odom(self, msg) -> None:
        pose_msg = wrap_odom_to_pcl_pose(msg, self._frame_id)
        # 复用在线回调：位置、yaw(由四元数取欧拉角)、锁与状态上报路径全部一致
        self._topic_subscriber._pcl_pose_callback(pose_msg)

        self._count += 1
        p = msg.pose.pose.position
        self._last_pos = (p.x, p.y, p.z)
        if self._count == 1 or self._count % 100 == 0:
            logger.info('odom 真值已喂入 %d 帧, 当前位置 x=%.2f y=%.2f z=%.2f',
                        self._count, p.x, p.y, p.z)


# --------------------------------------------------------------------------------------
# 平台即时飞控指令：报文包装 + 话题注入回调（离线测试/半在线调试共用）
# --------------------------------------------------------------------------------------

SIM_SN = 'SIM-UAV-00001'


def wrap_platform_command(raw: str) -> Dict[str, Any]:
    """把字符串指令包成与云端 MQTT 下发同构的报文（cmd=UAVCOMMAND + body.command）。"""
    return {
        'sn': SIM_SN,
        'ver': '1.0',
        'seq': str(uuid.uuid4()).replace('-', ''),
        'type': 1,
        'ts': int(time.time() * 1000),
        'cmd': 'UAVCOMMAND',
        'body': {'command': raw, 'actionId': f'sim_cmd_{raw}_{int(time.time())}'},
    }


def make_platform_command_callback(task_manager):
    """生成平台即时飞控指令的话题回调（std_msgs/String，data=指令枚举）。

    没有 MQTT（或想绕过 MQTT 直接验证接收之后的链路）时，外部向注入话题
    pub 字符串即可，报文包装与分发入口与在线完全一致（process_uav_command）。
    """

    def on_platform_command(msg) -> None:
        raw = (msg.data or '').strip()
        if not raw:
            logger.warning('[平台指令] 空内容，忽略（合法指令见 commands.txt）')
            return
        message_data = wrap_platform_command(raw)
        logger.info('[平台指令] 收到 "%s" -> process_uav_command', raw)
        try:
            task_manager.process_uav_command(message_data)
        except Exception as exc:
            logger.error('[平台指令] 执行异常: %s', exc)

    return on_platform_command
