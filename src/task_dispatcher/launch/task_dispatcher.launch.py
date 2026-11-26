import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

# ---- 端点参数本地覆盖（凭证不进仓库） ----
# 依次尝试：环境变量 UAV_ENDPOINT_CONFIG 指向的文件 > 仓内 endpoint.local.json（已 gitignore）
#           > install 空间 share/task_dispatcher/endpoint.local.json > ~/.uav_service/endpoint.json
# 读不到则全部回落到下面的占位默认值（`ros2 launch ... mqtt_broker:=x` 仍可逐项覆盖）。
_LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, _LAUNCH_DIR)
from endpoint_config import load_endpoint_overlay  # noqa: E402

OVERLAY = load_endpoint_overlay(_LAUNCH_DIR)


def _default(name, builtin):
    return str(OVERLAY.get(name, builtin))


def generate_launch_description():
    # 声明启动参数并设置默认值，保持与节点文件中的默认值一致
    mqtt_broker = LaunchConfiguration('mqtt_broker')
    mqtt_port = LaunchConfiguration('mqtt_port')
    mqtt_username = LaunchConfiguration('mqtt_username')
    mqtt_password = LaunchConfiguration('mqtt_password')
    mqtt_topic = LaunchConfiguration('mqtt_topic')
    mqtt_response_topic = LaunchConfiguration('mqtt_response_topic')
    mqtt_client_id = LaunchConfiguration('mqtt_client_id')
    robot_sn = LaunchConfiguration('robot_sn')
    server_url = LaunchConfiguration('server_url')
    px4_time_sync_device = LaunchConfiguration('px4_time_sync_device')
    px4_time_sync_baudrate = LaunchConfiguration('px4_time_sync_baudrate')
    px4_time_sync_window = LaunchConfiguration('px4_time_sync_window')
    
    # 创建task_dispatcher节点
    task_dispatcher_node = Node(
        package='task_dispatcher',
        executable='task_dispatcher_node',
        name='task_dispatcher_node',
        output='screen',
        prefix=['taskset -c 0,1,2,3'],  # 绑定CPU核心0,1,2,3
        parameters=[
            {
                'mqtt_broker': mqtt_broker,
                'mqtt_port': mqtt_port,
                'mqtt_username': mqtt_username,
                'mqtt_password': mqtt_password,
                'mqtt_topic': mqtt_topic,
                'mqtt_response_topic': mqtt_response_topic,
                'mqtt_client_id': mqtt_client_id,
                'robot_sn': robot_sn,
                'server_url': server_url,
                'px4_time_sync_device': px4_time_sync_device,
                'px4_time_sync_baudrate': px4_time_sync_baudrate,
                'px4_time_sync_window': px4_time_sync_window
            }
        ]
    )
    
    # 创建启动描述
    return LaunchDescription([
        # 声明启动参数（占位默认；真值放 endpoint.local.json，见文件头注释）
        DeclareLaunchArgument(
            'mqtt_broker',
            default_value=_default('mqtt_broker', 'test'),
            description='MQTT代理服务器地址'
        ),
        DeclareLaunchArgument(
            'mqtt_port',
            default_value=_default('mqtt_port', '10883'),
            description='MQTT代理服务器端口'
        ),
        DeclareLaunchArgument(
            'mqtt_username',
            default_value=_default('mqtt_username', 'ropeok'),
            description='MQTT用户名'
        ),
        DeclareLaunchArgument(
            'mqtt_password',
            default_value=_default('mqtt_password', 'test'),
            description='MQTT密码'
        ),
        DeclareLaunchArgument(
            'mqtt_topic',
            default_value=_default('mqtt_topic', '/UAV/UAV00002/dn'),
            description='MQTT订阅主题'
        ),
        DeclareLaunchArgument(
            'mqtt_response_topic',
            default_value=_default('mqtt_response_topic', '/robot/up'),
            description='MQTT响应主题'
        ),
        DeclareLaunchArgument(
            'mqtt_client_id',
            default_value=_default('mqtt_client_id', 'task_dispatcher_002_server'),
            description='MQTT客户端ID'
        ),
        DeclareLaunchArgument(
            'robot_sn',
            default_value=_default('robot_sn', 'UAV00002'),
            description='机器人序列号'
        ),
        DeclareLaunchArgument(
            'server_url',
            default_value=_default('server_url', 'https://test.com:81'),
            description='服务器URL地址'
        ),
        DeclareLaunchArgument(
            'version',
            default_value='1.1',
            description='版本号'
        ),
        DeclareLaunchArgument(
            'px4_time_sync_device',
            default_value='',
            description='PX4 时间同步串口设备，如 /dev/ttyACM0。留空则自动探测'
        ),
        DeclareLaunchArgument(
            'px4_time_sync_baudrate',
            default_value='57600',
            description='PX4 时间同步串口波特率，默认 57600（TELEM 口）'
        ),
        DeclareLaunchArgument(
            'px4_time_sync_window',
            default_value='30.0',
            description='PX4 时间同步尝试窗口（秒）。仅在节点启动后这段时间内尝试，'
                        '成功或超时即退出，运行期不再修改飞控时钟。设 0 表示禁用'
        ),

        # 添加节点
        task_dispatcher_node
    ])
