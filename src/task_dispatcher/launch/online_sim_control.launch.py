#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线 task_dispatcher 直连平台、控制仿真环境的启动文件（半在线仿真）。

与离线套件（offline_route_sim.launch.py）的分工：
    - 离线版：不连 MQTT，本地航点 JSON + 话题注入模拟平台，验证“接收之后”的处理链；
    - 本文件：跑真正的在线节点（colcon 安装的 task_dispatcher_node），
      真连 MQTT broker、真注册心跳、任务与即时飞控指令都从平台下发；
      仿真缺失的“真机外围”用共用适配件补齐（在线包代码零改动）。

前置条件
    1. 仿真已按 demo 跑起来（Gazebo + PX4 SITL + ego_planner + tf_publisher）：
         ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py
    2. task_dispatcher 包已 colcon 构建安装（在线节点按 package 方式启动）。
    3. 起本文件（桥脚本与 sim_support 已随包安装到 share/task_dispatcher/offline_sim；
       改动 setup.py 安装清单后需重新构建一次）：
         ros2 launch task_dispatcher online_sim_control.launch.py
       MQTT/服务器等默认值与原始部署配置一致（同真机连同一平台）；
       仿真机与真机同时在线时，务必覆盖 sn 与订阅主题避免混台抢单：
         robot_sn:=UAV_SIM_01 mqtt_topic:=/UAV/UAV_SIM_01/dn

组成（5 个进程）
    - sim_pcl_pose_bridge   : <namespace>/odometry 真值 -> /pcl_pose
                              （在线链路的位置来源是激光定位，仿真里没有，用桥补齐；
                                换算与 QoS 和离线直喂共用 sim_support，同一份逻辑）
    - sim_uav_command_bridge: /uav_command(LAND 等) -> multirotor_communication 的 XTD2Cmd
                              （验证“返航受阻 -> 迫降”分支；不需要可关）
    - mediamtx              : 本机 RTSP 服务器（8554），模拟真机云台相机的出流服务
                              （enable_rtsp_camera:=false 或找不到二进制时跳过）
    - sim_rtsp_camera_bridge: gz 相机 /<ns>/IMX214/image_raw -> ffmpeg 推流 -> mediamtx，
                              在线配置经 SIM_CAMERA01_URL 注入录像/预览 URL（与真机同链路）；
                              仿真侧同时把 CODEC 从 h265 改为 h264 透传（x86 无 h264_rkmpp），
                              并让在线节点的 ffmpeg 调用优先命中 venv 的 imageio-ffmpeg
    - task_dispatcher_node  : 在线节点本体；/fmu 绝对话题经 remap 指向载具命名空间

真机差异与风险（重要）
    - MQTT/注册/上报全部真实：平台会把这台“仿真机”当成在线设备并可派单，
      请用测试 broker 或隔离的 robot_sn，勿与真机同号。
    - 任务下发时在线代码会走 map_manager.update_pcd_map(HTTP 拉图)：
      连不上文件服务器只是拖慢约 30s，不阻断任务（返回值未被检查，实测确认）。
    - 在线代码的 `sudo systemctl start/stop ego_planner.service` 已被 sim_support 的
      SimSubprocess 旁路（online_sim_bootstrap 打桩；sim_paths:=false 时原样执行，
      仿真机无此 unit 会拖慢并中断任务线程，勿关）。
    - config.py 业务开关（ROBOT_TYPE=uav、PREFLIGHT/DOCK/MANUAL 关闭）与本场景一致；
      sim_paths:=true（默认）时仅覆盖“机载路径/相机/云台”这几个边界项到 /tmp
      （与离线套件同一注入套路，见 offline_sim/online_sim_bootstrap.py），
      其余业务代码与真机同源；给 false 则完全原样运行（需本机建好 /home/cat 目录）。

会话为长驻（模拟真机），航线跑完不会自动退出；Ctrl-C 收摊。
"""

import os
import shutil
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))   # symlink-install 时解析回源树
# launch -> task_dispatcher -> src -> uav_service -> <repo>
REPO_ROOT = os.path.abspath(os.path.join(LAUNCH_DIR, '..', '..', '..', '..'))


def _find_offline_dir() -> str:
    """仿真适配件目录（与离线套件共用 sim_support / 桥节点）：
    源树布局优先；copy 安装后位于 share/task_dispatcher/offline_sim，
    支持 `ros2 launch task_dispatcher online_sim_control.launch.py`。"""
    for cand in (os.path.normpath(os.path.join(LAUNCH_DIR, '..', 'task_dispatcher', 'offline_sim')),
                 os.path.normpath(os.path.join(LAUNCH_DIR, '..', 'offline_sim'))):
        if os.path.isfile(os.path.join(cand, 'sim_support.py')):
            return cand
    raise FileNotFoundError('找不到 offline_sim/sim_support.py（launch 目录: %s）' % LAUNCH_DIR)


OFFLINE_DIR = _find_offline_dir()
if OFFLINE_DIR not in sys.path:
    sys.path.insert(0, OFFLINE_DIR)
import sim_support  # noqa: E402  （话题/重映射/QoS 的单一事实来源）

# 端点参数本地覆盖（endpoint.local.json，凭证不入库；读不到用下面的占位默认）
if LAUNCH_DIR not in sys.path:
    sys.path.insert(0, LAUNCH_DIR)
from endpoint_config import load_endpoint_overlay  # noqa: E402

OVERLAY = load_endpoint_overlay(LAUNCH_DIR)


def _default(name, builtin):
    return str(OVERLAY.get(name, builtin))


def _default_python() -> str:
    """优先使用离线依赖 venv（setup_offline_sim_env.sh 生成），否则用系统 python3。"""
    venv_py = os.path.join(REPO_ROOT, 'uav_service', '.venv', 'bin', 'python3')
    return venv_py if os.path.isfile(venv_py) else 'python3'


def _find_mediamtx(explicit: str) -> str:
    """定位 MediaMTX 二进制：显式参数 > PATH > ~/.local/bin > /usr/local/bin。"""
    for cand in (explicit.strip(), shutil.which('mediamtx'),
                 os.path.expanduser('~/.local/bin/mediamtx'), '/usr/local/bin/mediamtx'):
        if cand and os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    return ''


def launch_setup(context, *args, **kwargs):
    def value_of(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    namespace = value_of('namespace')
    prefix = sim_support.ns_prefix(namespace)
    truth_odom = sim_support.truth_odom_topic(namespace, value_of('truth_odom_topic'))
    cmd_service = sim_support.cmd_service_name(namespace)
    python_bin = value_of('python_bin') or _default_python()
    use_sim_time = value_of('use_sim_time')
    remap_pairs = sim_support.fmu_remap_tuples(namespace)

    # 1) 位姿桥：仿真没有激光定位，用 odom 真值发在线链路认的 /pcl_pose
    pose_bridge = ExecuteProcess(
        cmd=[python_bin, os.path.join(OFFLINE_DIR, 'sim_pcl_pose_bridge.py'),
             '--odom-topic', truth_odom,
             '--pose-topic', '/pcl_pose',
             '--frame-id', 'map',
             '--use-sim-time', use_sim_time],
        output='screen',
        name='sim_pcl_pose_bridge',
        shell=False,
    )

    # 2) 命令桥：_emergency_land 等发出的 /uav_command 在仿真里落到 XTD2Cmd 服务
    command_bridge = ExecuteProcess(
        cmd=[python_bin, os.path.join(OFFLINE_DIR, 'sim_uav_command_bridge.py'),
             '--command-topic', '/uav_command',
             '--cmd-service', cmd_service,
             '--use-sim-time', use_sim_time],
        output='screen',
        name='sim_uav_command_bridge',
        shell=False,
    )

    # 3) RTSP 相机源（仅 online_sim 需要视频；offline 套件不启动）：
    #    mediamtx 本机服务器 + gz 相机图像推流桥，在线配置经 SIM_CAMERA01_URL 注入
    enable_rtsp = value_of('enable_rtsp_camera').lower() == 'true'
    rtsp_url = value_of('rtsp_url')
    image_topic = (value_of('camera_image_topic').strip()
                   or (f'{prefix}/IMX214/image_raw' if prefix else '/IMX214/image_raw'))
    mediamtx_bin = _find_mediamtx(value_of('mediamtx_bin')) if enable_rtsp else ''
    rtsp_actions = []
    if enable_rtsp and mediamtx_bin:
        # 显式指定配置：只开 RTSP（默认空配置会额外监听 HLS 8888 等，与本机服务撞端口）
        mtx_cfg = os.path.expanduser('~/.config/mediamtx/mediamtx.yml')
        mtx_cmd = [mediamtx_bin, mtx_cfg] if os.path.isfile(mtx_cfg) else [mediamtx_bin]
        rtsp_actions.append(ExecuteProcess(
            cmd=mtx_cmd, output='screen', name='mediamtx', shell=False))
        rtsp_actions.append(ExecuteProcess(
            cmd=[python_bin, os.path.join(OFFLINE_DIR, 'sim_rtsp_camera_bridge.py'),
                 '--image-topic', image_topic,
                 '--rtsp-url', rtsp_url,
                 '--fps', value_of('camera_fps')],
            output='screen', name='sim_rtsp_camera_bridge', shell=False))
    elif enable_rtsp:
        rtsp_actions.append(LogInfo(
            msg='enable_rtsp_camera=true 但未找到 mediamtx：安装到 ~/.local/bin/mediamtx '
                '或用 mediamtx_bin:= 指定（github.com/bluenviron/mediamtx releases 单二进制）'))

    # 4) 在线节点本体：与真机同一份 task_dispatcher_node.py、同一组参数
    dispatcher_params = {
        'mqtt_broker': value_of('mqtt_broker'),
        'mqtt_port': value_of('mqtt_port'),
        'mqtt_username': value_of('mqtt_username'),
        'mqtt_password': value_of('mqtt_password'),
        'mqtt_topic': value_of('mqtt_topic'),
        'mqtt_response_topic': value_of('mqtt_response_topic'),
        'mqtt_client_id': value_of('mqtt_client_id'),
        'robot_sn': value_of('robot_sn'),
        'server_url': value_of('server_url'),
        'use_sim_time': use_sim_time.lower() == 'true',
    }

    # 解释器选择：x86 仿真机上入口脚本用的系统 python 没有 paho/cv2 等依赖，
    # 有离线 venv 时直接用 venv 跑同一份源码（PYTHONPATH 补包内两层目录，
    # 与机载部署 README §7 的 PYTHONPATH 前置等效）；没有 venv（真机）保持
    # colcon 安装入口 Node(package=...)，行为与真机 launch 完全一致。
    explicit_py = value_of('dispatcher_python').strip()
    cpu_pin = value_of('cpu_pin').strip()
    venv_py = os.path.join(REPO_ROOT, 'uav_service', '.venv', 'bin', 'python3')
    # 包内模块目录（task_dispatcher/）与其父目录（src/task_dispatcher/）
    mod_dir = os.path.normpath(os.path.join(LAUNCH_DIR, '..', 'task_dispatcher'))
    pkg_root = os.path.normpath(os.path.join(mod_dir, '..'))
    node_script = os.path.join(mod_dir, 'task_dispatcher_node.py')
    use_venv_runner = os.path.isfile(venv_py) if not explicit_py else os.path.isfile(explicit_py)
    # sim_paths=true（默认）：venv 分支经 online_sim_bootstrap 起节点——只把录像/点云
    # 路径与相机/云台 URL 这几个边界项按离线同款方式覆盖到 /tmp（仿真机没有
    # config.py 里写死的机载 /home/cat，构造 RtspRecordManager 会直接崩）；
    # false：直跑在线脚本原样行为（需本机已建好 /home/cat/slam_data 并 chown）。
    # 两种模式的业务代码完全同一份 task_dispatcher_node.py。
    sim_paths = value_of('sim_paths').lower() == 'true'
    if use_venv_runner:
        py = explicit_py or venv_py
        runner_script = (os.path.join(mod_dir, 'offline_sim', 'online_sim_bootstrap.py')
                         if sim_paths else node_script)
        cmd = [py, runner_script, '--ros-args']
        for key, val in dispatcher_params.items():
            cmd += ['-p', f'{key}:={val}']
        for src_t, dst_t in remap_pairs:
            cmd += ['-r', f'{src_t}:={dst_t}']
        extra_env = {'PYTHONPATH': os.pathsep.join(
            p for p in (mod_dir, pkg_root, os.environ.get('PYTHONPATH', '')) if p)}
        if enable_rtsp:
            # online_sim_bootstrap 据此覆盖 CAMERA01_URL、打开 ENABLE_VIDEO，
            # 并把 CODEC 从 h265(h264_rkmpp) 切到 h264 透传（x86 无 Rockchip 编码器）
            extra_env['SIM_CAMERA01_URL'] = rtsp_url
            extra_env['SIM_ENABLE_VIDEO'] = 'true'
        if cpu_pin:
            cmd = ['bash', '-c', f'taskset -c {cpu_pin} ' + ' '.join(f"'{a}'" for a in cmd)]
        dispatcher = ExecuteProcess(cmd=cmd, output='screen', name='task_dispatcher_node',
                                    additional_env=extra_env, shell=False)
        runner_desc = f'venv 直跑源码 {py}' + ('（边界路径覆盖到 /tmp）' if sim_paths else '（config 原路径）')
    else:
        node_kwargs = dict(
            package='task_dispatcher',
            executable='task_dispatcher_node',
            name='task_dispatcher_node',
            output='screen',
            parameters=[dispatcher_params],
            remappings=remap_pairs,
        )
        if cpu_pin:
            node_kwargs['prefix'] = [f'taskset -c {cpu_pin}']   # 真机上才建议绑核
        dispatcher = Node(**node_kwargs)
        runner_desc = 'colcon 安装入口（package=task_dispatcher）'

    video_injected = enable_rtsp and use_venv_runner and sim_paths
    video_desc = f'相机={rtsp_url}' if enable_rtsp else '相机关闭'
    print(f'[online_sim_control] 载具命名空间={prefix or "(无)"} 位姿来源={truth_odom} -> /pcl_pose '
          f'fmu重映射={len(remap_pairs)}条 MQTT={value_of("mqtt_broker")}:{value_of("mqtt_port")} '
          f'sn={value_of("robot_sn")} 节点运行方式={runner_desc} '
          f'视频={video_desc}({"已注入" if video_injected else "未注入配置"}) '
          f'mediamtx={"已找到" if mediamtx_bin else "未找到"}')

    actions = [
        LogInfo(msg='在线仿真控制：task_dispatcher 按原始配置真连平台 MQTT，平台会把本仿真机'
                    '当作在线设备派单；与真机同时在线时请用 robot_sn/mqtt_topic 覆盖隔离'),
    ]
    if enable_rtsp and not video_injected:
        actions.append(LogInfo(
            msg='enable_rtsp_camera=true 但视频配置未注入在线节点：需要存在离线 venv'
                '（或 dispatcher_python 指定解释器）且 sim_paths:=true；否则节点仍用 config.py 的'
                ' CAMERA01_URL/CODEC=h265，平台会显示相机离线、点播推流失败'))
    if value_of('enable_pose_bridge').lower() == 'true':
        actions.append(pose_bridge)
    if value_of('enable_command_bridge').lower() == 'true':
        actions.append(command_bridge)
    actions.extend(rtsp_actions)
    actions.append(dispatcher)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/x500_depth_0/',
                              description='仿真载具命名空间（决定 /fmu 话题前缀与 XTD2Cmd 服务名）'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='是否跟随 Gazebo 仿真时钟（仿真必须 true）'),
        DeclareLaunchArgument('truth_odom_topic', default_value='',
                              description='Gazebo 真值里程计话题，留空自动用 <namespace>/odometry'),
        DeclareLaunchArgument('enable_pose_bridge', default_value='true',
                              description='是否启动 /pcl_pose 位姿桥；外部已有定位源时置 false'),
        DeclareLaunchArgument('enable_command_bridge', default_value='true',
                              description='是否启动 /uav_command 命令桥（验证返航受阻->迫降分支）'),
        DeclareLaunchArgument('enable_rtsp_camera', default_value='true',
                              description='是否启动本机 RTSP 相机源（mediamtx + gz 相机推流桥），'
                                          '并把它注入在线配置的 CAMERA01_URL/ENABLE_VIDEO'),
        DeclareLaunchArgument('rtsp_url', default_value='rtsp://127.0.0.1:8554/main.264',
                              description='推流与配置注入用的 RTSP 地址（与真机同构：MediaMTX 8554）'),
        DeclareLaunchArgument('camera_image_topic', default_value='',
                              description='gz 桥出来的相机图像话题，留空自动用 <ns>/IMX214/image_raw'),
        DeclareLaunchArgument('camera_fps', default_value='15',
                              description='推流帧率（仿真相机 30Hz 抽帧）'),
        DeclareLaunchArgument('mediamtx_bin', default_value='',
                              description='mediamtx 可执行文件路径，留空自动在 PATH/~/.local/bin 查找'),
        # ---- 端点参数：占位默认 + endpoint.local.json 覆盖（与 task_dispatcher.launch.py 同一份文件）----
        DeclareLaunchArgument('mqtt_broker', default_value=_default('mqtt_broker', 'test'),
                              description='MQTT代理服务器地址'),
        DeclareLaunchArgument('mqtt_port', default_value=_default('mqtt_port', '10883'),
                              description='MQTT代理服务器端口'),
        DeclareLaunchArgument('mqtt_username', default_value=_default('mqtt_username', 'ropeok'),
                              description='MQTT用户名'),
        DeclareLaunchArgument('mqtt_password', default_value=_default('mqtt_password', 'test'),
                              description='MQTT密码'),
        DeclareLaunchArgument('mqtt_topic', default_value=_default('mqtt_topic', '/UAV/UAV00002/dn'),
                              description='MQTT订阅主题'),
        DeclareLaunchArgument('mqtt_response_topic', default_value=_default('mqtt_response_topic', '/robot/up'),
                              description='MQTT响应主题'),
        DeclareLaunchArgument('mqtt_client_id',
                              default_value=_default('mqtt_client_id', 'task_dispatcher_002_server'),
                              description='MQTT客户端ID'),
        DeclareLaunchArgument('robot_sn', default_value=_default('robot_sn', 'UAV00002'),
                              description='机器人序列号（仿真实例建议覆盖成与真机隔离的 sn）'),
        DeclareLaunchArgument('server_url', default_value=_default('server_url', 'https://test.com:81'),
                              description='服务器URL地址'),
        DeclareLaunchArgument('sim_paths', default_value='true',
                              description='true=在线节点经 online_sim_bootstrap 把录像/点云/相机等'
                                          '机载边界路径覆盖到 /tmp（仿真机无 /home/cat，默认推荐）；'
                                          'false=完全按 config.py 原路径运行（与真机一致，需自建目录）'),
        DeclareLaunchArgument('cpu_pin', default_value='',
                              description='可选绑核（如 "0,1,2,3"）；仿真与 Gazebo 同机时建议留空'),
        DeclareLaunchArgument('dispatcher_python', default_value='',
                              description='在线节点解释器：留空自动——存在离线 venv 时用其 python '
                                          '直跑源码（x86 仿真机系统 python 缺 paho/cv2 等依赖），'
                                          '否则用 colcon 安装入口（与真机一致）；也可显式给解释器路径'),
        DeclareLaunchArgument('python_bin', default_value='',
                              description='运行桥节点的 python，留空自动优先 uav_service/.venv'),
        OpaqueFunction(function=launch_setup),
    ])
