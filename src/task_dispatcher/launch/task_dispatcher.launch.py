from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

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
                'server_url': server_url
            }
        ]
    )
    
    # 创建启动描述
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'mqtt_broker',
            default_value='test',
            description='MQTT代理服务器地址'
        ),
        DeclareLaunchArgument(
            'mqtt_port',
            default_value='10883',
            description='MQTT代理服务器端口'
        ),
        DeclareLaunchArgument(
            'mqtt_username',
            default_value='ropeok',
            description='MQTT用户名'
        ),
        DeclareLaunchArgument(
            'mqtt_password',
            default_value='test',
            description='MQTT密码'
        ),
        DeclareLaunchArgument(
            'mqtt_topic',
            default_value='/UAV/UAV00002/dn',
            description='MQTT订阅主题'
        ),
        DeclareLaunchArgument(
            'mqtt_response_topic',
            default_value='/robot/up',
            description='MQTT响应主题'
        ),
        DeclareLaunchArgument(
            'mqtt_client_id',
            default_value='task_dispatcher_002_server',
            description='MQTT客户端ID'
        ),
        DeclareLaunchArgument(
            'robot_sn',
            default_value='UAV00002',
            description='机器人序列号'
        ),
        DeclareLaunchArgument(
            'server_url',
            default_value='https://test.com:81',
            description='服务器URL地址'
        ),
        DeclareLaunchArgument(
            'version',
            default_value='1.1',
            description='版本号'
        ),
        
        # 添加节点
        task_dispatcher_node
    ])
