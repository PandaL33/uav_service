#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真离线航线启动文件（独立于 ros2_single_vehicle_demo_launch.py，只起调度侧）。

前置条件
    1. 先把仿真跑起来（Gazebo 世界 + PX4 SITL + 载具 + tf_publisher + ego_planner + CP）：
         ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py
    2. 装好离线依赖（只装依赖，不改在线代码）：
         bash uav_service/scripts/setup_offline_sim_env.sh
    3. 起本文件，启动后会等规划器就绪 -> 立刻按航点执行航线（默认写死读本目录旁的
       waypoints_3_3.json，无需再传 route_file）：
         ros2 launch task_dispatcher offline_route_sim.launch.py
       （需 colcon 构建过 task_dispatcher；直接给本文件路径启动同样有效）
       换航线：route_file:=<绝对路径或裸文件名>；只看报文不真飞：追加 dry_run:=true
       航线途中注入平台即时飞控指令（模拟 MQTT 下发，另开一个终端 pub 字符串枚举即可）：
         ros2 topic pub --once /offline_sim/platform_command std_msgs/msg/String "{data: 'RTL_ROUTE'}"
       测试用的完整 pub 指令清单在启动时生成到 task_dispatcher/offline_sim/commands.txt

本启动文件包含 2 个进程 + 1 个就绪门（航线进程退出后会话自动结束）
    - sim_uav_command_bridge   : /uav_command(LAND/RTL/...) -> multirotor_communication 的 XTD2Cmd 服务
                                 （用于验证“连续航点不可达 -> 返航受阻 -> 迫降”分支）
    - wait_sim_ready           : 等 PX4 SITL 里程计已发布 + ego_planner 已订阅目标点话题
    - offline_cruise_sim_node  : 读航点 JSON（页面只给坐标，方向按 yaw_mode 自动补），走与在线同一条入口
                                 process_cruise_task('SETCRUISETASK') -> _execute_task_thread
                                 -> _processing_cruise_task -> perform_action_walk_to_pos_3d
    航线进程一退出（正常完成 / max_wait 超时 / 快速失败），本会话即 Shutdown，命令桥一起收掉；
    仿真侧 ros2_single_vehicle_demo_launch.py 是另一个会话，不受影响，想停要自己 Ctrl-C。

关于位置来源
    在线由激光定位发布 /pcl_pose；离线仿真不提供这个话题，也不额外造话题——
    调度节点内部直接订阅 Gazebo odom 真值(<namespace>/odometry)，把位姿喂给在线的
    _pcl_pose_callback（SimOdomPoseFeeder），判到达/停滞 watchdog 的代码路径与在线一致。
    如外部已有 /pcl_pose，可用 feed_odom_pose:=false 退回在线原路径。

关于命名空间
    在线代码里 /fmu/in|out/... 是绝对名（真机单机无命名空间）；仿真载具带 /x500_depth_0/ 前缀，
    因此这里用 --remap 注入 rcl 的重映射规则，不改任何在线源文件。
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))   # symlink-install 时解析回源树
# launch -> task_dispatcher -> src -> uav_service -> <repo>
REPO_ROOT = os.path.abspath(os.path.join(LAUNCH_DIR, '..', '..', '..', '..'))


def _find_offline_dir() -> str:
    """offline_sim 定位：源树布局优先；copy 安装后在 share/task_dispatcher/offline_sim，
    两种布局都能找到，支持 `ros2 launch task_dispatcher offline_route_sim.launch.py`。"""
    for cand in (os.path.normpath(os.path.join(LAUNCH_DIR, '..', 'task_dispatcher', 'offline_sim')),
                 os.path.normpath(os.path.join(LAUNCH_DIR, '..', 'offline_sim'))):
        if os.path.isfile(os.path.join(cand, 'sim_support.py')):
            return cand
    raise FileNotFoundError('找不到 offline_sim/sim_support.py（launch 目录: %s）' % LAUNCH_DIR)


OFFLINE_DIR = _find_offline_dir()
# 话题/命名空间/重映射表的单一事实来源（与在线仿真启动文件 online_sim_control.launch.py 同一份）
if OFFLINE_DIR not in sys.path:
    sys.path.insert(0, OFFLINE_DIR)
import sim_support  # noqa: E402


def _resolve_route(name: str) -> str:
    """航点文件解析：绝对路径原样返回；裸文件名依次在 launch 目录、offline_sim/routes、仓库根里找。"""
    if os.path.isabs(name):
        return name
    for base in (LAUNCH_DIR, os.path.join(OFFLINE_DIR, 'routes'), REPO_ROOT):
        candidate = os.path.join(base, name)
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(REPO_ROOT, name)


def _default_route() -> str:
    """默认航线：写死 launch 目录旁的 waypoints_3_3.json（PCD 规划页导出）。"""
    return _resolve_route('waypoints_rk_garden.json')
    # return _resolve_route('waypoints_3_3.json')


def _default_python() -> str:
    """优先使用离线依赖 venv（setup_offline_sim_env.sh 生成），否则用系统 python3。"""
    venv_py = os.path.join(REPO_ROOT, 'uav_service', '.venv', 'bin', 'python3')
    return venv_py if os.path.isfile(venv_py) else 'python3'


def launch_setup(context, *args, **kwargs):
    def value_of(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    namespace = value_of('namespace')
    prefix = sim_support.ns_prefix(namespace)

    use_sim_time = value_of('use_sim_time')
    route_file = _resolve_route(value_of('route_file'))

    truth_odom = sim_support.truth_odom_topic(namespace, value_of('truth_odom_topic'))
    px4_odom = sim_support.px4_odom_topic(namespace)
    cmd_service = sim_support.cmd_service_name(namespace)
    python_bin = value_of('python_bin') or _default_python()

    remaps = sim_support.fmu_remap_cli_args(namespace)

    # 1) 命令桥：让 _emergency_land 的 'LAND' 在仿真里真正生效（走 PX4 原生 LAND，不与规划器抢控制）
    command_bridge = ExecuteProcess(
        cmd=[python_bin, os.path.join(OFFLINE_DIR, 'sim_uav_command_bridge.py'),
             '--command-topic', '/uav_command',
             '--cmd-service', cmd_service,
             '--use-sim-time', use_sim_time],
        output='screen',
        name='sim_uav_command_bridge',
        shell=False,
    )

    # 2) 就绪等待：PX4 SITL 里程计已发布 + ego_planner 已订阅目标点话题
    #    （与 demo 里 wait_for_px4_odom 同一思路：确认“在线链路的两端”都在，才下发航线）
    wait_ready = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'''
echo "等待仿真就绪: {px4_odom} 有发布者 且 /move_base_simple/goal 有订阅者(ego_planner)"
timeout=300; start=$(date +%s)
pub_ok() {{ ros2 topic info "$1" --verbose 2>/dev/null | grep -q "Publisher count: [1-9]"; }}
sub_ok() {{ ros2 topic info "$1" --verbose 2>/dev/null | grep -q "Subscription count: [1-9]"; }}
while [ $(($(date +%s) - start)) -lt $timeout ]; do
    if pub_ok "{px4_odom}" && sub_ok "/move_base_simple/goal"; then
        echo "仿真就绪，5 秒后下发航线任务"
        sleep 5
        exit 0
    fi
    echo "等待中... $(( $(date +%s) - start ))s"
    sleep 2
done
echo "错误: 仿真在 $timeout 秒内未就绪（请先启动 ros2_single_vehicle_demo_launch.py）"
exit 1
'''
        ],
        output='screen',
        name='wait_sim_ready',
        shell=False,
    )

    # 3) 航线执行：进程退出即代表本轮离线航线跑完（上报已落盘 JSONL）
    #    位置来源：直接读 Gazebo odom 真值（离线不发布 /pcl_pose）
    route_cmd = [
        python_bin, os.path.join(OFFLINE_DIR, 'offline_cruise_sim_node.py'),
        '--route', route_file,
        '--yaw-mode', value_of('yaw_mode'),
        '--yaw-fixed', value_of('yaw_fixed_deg'),
        '--use-sim-time', use_sim_time,
        '--report-file', value_of('report_file'),
        '--feed-odom-pose', value_of('feed_odom_pose'),
        '--odom-topic', truth_odom,
        '--max-wait', value_of('max_wait'),
        '--exit-on-finish', value_of('exit_on_finish'),
    ]
    print(f'[offline_route_sim] 航点文件 = {route_file} '
          f'(存在: {os.path.isfile(route_file)}), yaw_mode = {value_of("yaw_mode")}, '
          f'位姿来源 = {truth_odom} (feed_odom_pose={value_of("feed_odom_pose")})')
    task_id = value_of('task_id')
    if task_id:
        route_cmd += ['--task-id', task_id]
    if remaps:
        route_cmd += remaps
    if value_of('dry_run').lower() == 'true':
        route_cmd += ['--dry-run']

    route_runner = ExecuteProcess(cmd=route_cmd, output='screen', name='offline_cruise_sim', shell=False)

    # 就绪后再开跑，保证第一个目标点不会因规划器未起来而丢失
    start_route = RegisterEventHandler(
        OnProcessExit(target_action=wait_ready, on_exit=[route_runner])
    )

    # 航线进程退出 = 本轮离线测试结束（成功降落 / 超时 / 快速失败都会走到这里）。
    # 命令桥是长驻节点，不主动关掉的话 ros2 launch 会话会一直挂着不返回 shell。
    # exit_on_finish:=false 时航线进程不退出，因此也不会触发关闭。
    close_on_finish = RegisterEventHandler(
        OnProcessExit(
            target_action=route_runner,
            on_exit=[
                LogInfo(msg='离线航线进程已结束，关闭本次启动会话（仿真侧 demo 不受影响）'),
                Shutdown(),
            ],
        )
    )

    actions = []
    if value_of('enable_command_bridge').lower() == 'true':
        actions.append(command_bridge)
    actions += [wait_ready, start_route, close_on_finish]
    return actions


def generate_launch_description():
    default_route = _default_route()
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/x500_depth_0/',
                              description='仿真载具命名空间（决定 /fmu 话题前缀与 XTD2Cmd 服务名）'),
        DeclareLaunchArgument('route_file', default_value=default_route,
                              description='航点配置 JSON；默认写死为本 launch 目录旁的 waypoints_3_3.json，'
                                          '也可传绝对路径或裸文件名(在 offline_sim/routes 与仓库根里找)'),
        DeclareLaunchArgument('yaw_mode', default_value='smooth',
                              description='航点没写 d 时的方向补全：smooth(曲线切线)/tangent(来向切线)/zero/fixed；'
                                          '文件里显式写了 d 的航点优先用文件值'),
        DeclareLaunchArgument('yaw_fixed_deg', default_value='0',
                              description='yaw_mode=fixed 时统一的机头朝向（度）'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='是否跟随 Gazebo 仿真时钟（仿真必须 true）'),
        DeclareLaunchArgument('truth_odom_topic', default_value='',
                              description='Gazebo 真值里程计话题（离线版的位置来源），留空自动用 <namespace>/odometry'),
        DeclareLaunchArgument('feed_odom_pose', default_value='true',
                              description='true=直接订 odom 真值喂在线回调（默认，不依赖 /pcl_pose）；'
                                          'false=沿用外部已发布的 /pcl_pose'),
        DeclareLaunchArgument('task_id', default_value='',
                              description='任务ID，留空自动生成 offline_sim_<时间戳>'),
        DeclareLaunchArgument('report_file', default_value='/tmp/uav_offline_sim/action_report.jsonl',
                              description='离线上报报文落盘路径'),
        DeclareLaunchArgument('max_wait', default_value='0',
                              description='航线最长等待秒数，0=不限；超时以退出码 3 结束（同样会关闭会话），'
                                          '用于兜底“降落结果一直不确认”的卡死场景'),
        DeclareLaunchArgument('exit_on_finish', default_value='true',
                              description='航线结束后是否退出调度节点；true 时本启动会话随之整体退出'
                                          '（false 仅调试用，会让会话一直挂着）'),
        DeclareLaunchArgument('enable_command_bridge', default_value='true',
                              description='是否启动 /uav_command 命令桥（验证返航受阻->迫降分支）'),
        DeclareLaunchArgument('dry_run', default_value='false',
                              description='只打印将要下发的 SETCRUISETASK 报文，不真飞'),
        DeclareLaunchArgument('python_bin', default_value='',
                              description='运行离线节点的 python，留空自动优先 uav_service/.venv'),
        OpaqueFunction(function=launch_setup),
    ])
