
from std_msgs.msg import String
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleLocalPosition
import math
import gpiod
import logging
import time
import asyncio
from task_dispatcher.config import ENABLE_DOCK_CONTROL
from typing import Optional, Dict, Any
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
from geometry_msgs.msg import PoseStamped
from dock_control_client import DockControlClient


# 配置日志
logger = logging.getLogger('task_dispatcher.uav_mission_manager')

class UavMissionManager():
    """
    无人机自主飞行任务服务器节点
    使用话题而非服务来控制无人机
    """

    def __init__(self, node, topic_subscriber):

        self.node = node
        self.topic_subscriber: Optional[Ros2TopicSubscriber] = topic_subscriber
        # 创建兼容PX4的QoS配置
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建话题订阅器，接收命令
        self.command_sub = self.node.create_subscription(
            String,
            'uav_command',
            self.command_callback,
            10
        )

        # 创建发布者
        self.offboard_control_mode_publisher = self.node.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.trajectory_setpoint_publisher = self.node.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.vehicle_command_publisher = self.node.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # 创建订阅者，使用与PX4兼容的QoS
        self.vehicle_status_sub = self.node.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.vehicle_local_position_sub = self.node.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile
        )

        # 初始化参数
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        
        # 设置定时器，每0.1秒触发一次回调函数
        self.timer = self.node.create_timer(0.1, self.timer_callback)

        # 初始位置和目标位置
        self.initial_x = None
        self.initial_y = None
        self.target_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 状态管理
        self.current_state = 'IDLE'  # 可能的状态: IDLE, OFFBOARD, ARMING, TAKING_OFF, FLYING_TO_POINT, HOVERING, LANDING, DISARMING
        self.flying_state = 'TAKEOFF'  # 飞行阶段: TAKEOFF, CRUISE, RETURN
        self.setpoint_send_count = 0
        self.is_flying = False
        self.takeoff_altitude = 1.0  # 默认起飞高度1米，可以通过命令修改
        
        self.goal_pose_pub_3d = self.node.create_publisher(PoseStamped, "/goal_pose_3d", 10)
        
        self.last_log_time = time.time() 
        self.log_interval = 10.0  # 设置间隔为 10 秒
        
        # 机舱控制
        self.dock_control_client = DockControlClient(node)

        logger.info(f'无人机自主飞行任务服务器已启动，current_state: {self.current_state}')

    def command_callback(self, msg):
        """处理无人机任务命令"""
        command = msg.data
        logger.info(f'收到命令：{command}')
        
        # 处理 GPIO 控制命令，格式为"GPIO:value"
        if command.startswith('GPIO:'):
            try:
                parts = command.split(':')
                if len(parts) == 2:
                    value = int(parts[1])
                    self.control_gpio(value)
                else:
                    logger.error(f'GPIO 命令格式错误：{command}，应为 GPIO:value')
            except ValueError as e:
                logger.error(f'GPIO 命令参数解析失败：{command}, 错误：{e}')
        elif command == 'OFFBOARD':
            self.current_state = 'OFFBOARD'
            logger.info(f'无人机切换OFFBOARD模式，current_state: {self.current_state}')
        elif command == 'TAKEOFF':
            logger.info('开始执行起飞任务，默认高度 1 米')
            self.current_state = 'ARMING'
            self.flying_state = 'TAKEOFF'
        elif command.startswith('TAKEOFF:'):
            # 解析高度参数，格式为"TAKEOFF:height"
            try:
                height_str = command.split(':')[1]
                height = float(height_str)
                self.takeoff_altitude = height
                logger.info(f'开始执行起飞任务，高度设置为：{height}米')
                self.current_state = 'ARMING'
                self.flying_state = 'TAKEOFF'
            except ValueError:
                logger.error(f'无法解析高度参数：{command}')
                logger.info('使用默认高度 1 米执行起飞任务')
                self.current_state = 'ARMING'
                self.flying_state = 'TAKEOFF'
        elif command == 'LAND':
            logger.info('开始执行降落任务')
            #self.land()
            self.current_state = 'LANDING'
            self.flying_state = 'RETURN'
        elif command.startswith('GOTO:'):
            # 解析坐标，格式为"GOTO:x,y,z"
            try:
                coords_str = command.split(':')[1]
                x, y, z = map(float, coords_str.split(','))
                logger.info(f'开始执行巡航任务，目标点：({x}, {y}, {z})')
                self.target_position['x'] = x
                self.target_position['y'] = y
                self.target_position['z'] = -abs(z)  # 转换为 NED 坐标系
                self.current_state = 'FLYING_TO_POINT'
                self.flying_state = 'CRUISE'
            except ValueError:
                logger.error(f'无法解析坐标：{command}')
        else:
            logger.warning(f'未知命令：{command}')

    def control_gpio(self, value):
        """
        使用 python3-libgpiod 库控制 GPIO 引脚的电平
        gpio:GPIO3_C1 编号:113
        参数:
            value: 电平值 (0 或 1)
        """
        if value not in [0, 1]:
            logger.error(f'GPIO 值必须为 0 或 1，当前值：{value}')
            return
        
        # 固定配置：控制器编号为 3，引脚编号为 17
        controller_id = 3
        gpio_pin = 17
        
        try:
            logger.info(f'执行 GPIO 控制：控制器={controller_id}, pin={gpio_pin}, value={value}')
            
            # 打开 GPIO 芯片设备
            chip = gpiod.Chip("3", gpiod.Chip.OPEN_BY_NUMBER)
            
            # 获取线路请求
            gpio3_c1 = chip.get_line(gpio_pin)
            
            # 请求线路并设置输出方向
            gpio3_c1.request(consumer="gpio", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
            
            # 设置电平值
            gpio3_c1.set_value(value)
            
            logger.info(f'GPIO {gpio_pin} 设置为 {"高电平" if value == 1 else "低电平"} 成功')
            
            # 释放线路
            gpio3_c1.release()
            chip.close()
                
        except FileNotFoundError as e:
            logger.error(f'未找到 GPIO 设备：gpio3_c1, 请确保已加载 gpio 模块')
        except PermissionError as e:
            logger.error(f'GPIO 权限不足：{e}, 请运行 sudo chmod 666 gpio3_c1 或将用户加入 gpio 组')
        except Exception as e:
            logger.error(f'GPIO 控制异常：{e}')

    def vehicle_status_callback(self, msg):
        """车辆状态回调函数"""
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg):
        """车辆本地位置回调函数"""
        self.vehicle_local_position = msg

    def timer_callback(self):
        """定时器回调函数，控制无人机状态转换"""
        if self.current_state == 'IDLE':
            current_time = time.time()
            if current_time - self.last_log_time >= self.log_interval:
                logger.info(f'无人机自主飞行状态，current_state: {self.current_state}')
                self.last_log_time = current_time  # 更新上次打印时间
        else:
            # 发布offboard控制模式信号 - 必须持续发送
            #self.publish_offboard_control_heartbeat_signal()

            # 根据当前状态发布轨迹设定点
            if self.current_state == 'ARMING':
                self.handle_arming_state()
            elif self.current_state == 'TAKING_OFF':
                self.handle_taking_off_state()
            elif self.current_state == 'FLYING_TO_POINT':
                self.handle_flying_to_point_state()
            elif self.current_state == 'HOVERING':
                self.handle_hovering_state()
            elif self.current_state == 'LANDING':
                self.handle_landing_state()
            elif self.current_state == 'DISARMING':
                self.handle_disarming_state()
            # else:
                    # OFFBOARD状态或其他状态，发布安全设定点
            #     self.publish_setpoint(x=float('nan'), y=float('nan'), z=float('nan'))

            # self.setpoint_send_count += 1

    def handle_arming_state(self):
        """处理解锁状态"""
        # 记录初始位置
        if self.initial_x is None or self.initial_y is None:
            self.initial_x = self.vehicle_local_position.x
            self.initial_y = self.vehicle_local_position.y
            logger.info(f'记录初始位置: X={self.initial_x:.2f}, Y={self.initial_y:.2f}')

        # 启用offboard模式
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.engage_offboard_mode()

        # 持续发送解锁命令直到成功
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            logger.info('无人机已解锁，切换到起飞状态')
            self.current_state = 'TAKING_OFF'
        else:
            self.arm()

        # 发布基础设定点
        self.publish_setpoint(x=self.initial_x, y=self.initial_y, z=-0.1)

    def  handle_taking_off_state(self):
        """处理起飞状态"""
        takeoff_altitude = self.takeoff_altitude  # 起飞高度
        
        # 发布起飞设定点
        self.publish_setpoint(x=self.initial_x, y=self.initial_y, z=-takeoff_altitude)

        # 检查是否达到目标高度
        if self.vehicle_local_position.z_valid:
            current_altitude = abs(self.vehicle_local_position.z)
            target_altitude = takeoff_altitude * 0.9  # 达到90%的目标高度

            logger.info(f'当前X: {self.vehicle_local_position.x:.2f}, 当前Y: {self.vehicle_local_position.y:.2f}, 当前高度: {current_altitude:.2f}, 目标: {target_altitude:.2f}')

            if current_altitude >= target_altitude:
                logger.info(f'达到目标高度 {takeoff_altitude}m')
                
                # 如果是巡航请求，则继续前往目标点
                if self.flying_state == 'CRUISE':
                    self.current_state = 'FLYING_TO_POINT'
                # 如果是返航请求，则进入悬停状态
                elif self.flying_state == 'RETURN':
                    self.current_state = 'HOVERING'
                else:
                    self.current_state = 'OFFBOARD'

    def handle_flying_to_point_state(self):
        """处理飞向目标点状态"""
        # 发布目标点设定点
        self.publish_setpoint(
            x=self.target_position['x'], 
            y=self.target_position['y'], 
            z=self.target_position['z']
        )

        # 检查是否到达目标点（距离小于0.1米）
        distance_to_target = math.sqrt(
            (self.vehicle_local_position.x - self.target_position['x'])**2 +
            (self.vehicle_local_position.y - self.target_position['y'])**2 +
            (self.vehicle_local_position.z - self.target_position['z'])**2
        )
        
        # 打印目标点位置信息和到目标点的距离
        logger.info(f'目标点位置: ({self.target_position["x"]:.2f}, {self.target_position["y"]:.2f}, {self.target_position["z"]:.2f}), \
                               当前点位置: ({self.vehicle_local_position.x:.2f}, {self.vehicle_local_position.y:.2f}, {self.vehicle_local_position.z:.2f}), \
                               距离: {distance_to_target:.2f}m')

        if distance_to_target < 0.1:  # 距离小于0.1米认为到达
            logger.info(f'已到达目标点: ({self.target_position["x"]:.2f}, {self.target_position["y"]:.2f}, {self.target_position["z"]:.2f})')
            self.current_state = 'OFFBOARD'

    def handle_hovering_state(self):
        """处理悬停状态"""
        # 在目标点悬停
        if self.current_state == 'FLYING_TO_POINT':
            hover_pos = self.target_position
        else:
            # 在当前位置悬停
            hover_pos = {
                'x': self.vehicle_local_position.x,
                'y': self.vehicle_local_position.y,
                'z': self.vehicle_local_position.z
            }

        self.publish_setpoint(
            x=hover_pos['x'],
            y=hover_pos['y'],
            z=hover_pos['z']
        )
    
    def _uav_is_flying_to_point(self, target_position):
        """
        判断无人机是否到达目标点。
        
        增强点：
        1. 防御性检查：处理 None、缺失键、非数字类型。
        2. 维度兼容：自动处理 2D/3D 坐标差异。
        3. 异常捕获：防止数学计算错误。
        """
        
        # 1. 获取位置数据
        try:
            robot_pos = self.topic_subscriber.get_position()
            
            # 2. 基础有效性检查
            if robot_pos is None:
                logger.warning("获取无人机位置失败: 返回值为 None")
                return False
            
            # 3. 安全提取坐标 (使用 .get() 防止 KeyError，并转换为 float)
            def safe_get_coord(data, key, default=0.0):
                val = data.get(key, default)
                if val is None or not isinstance(val, (int, float)):
                    raise ValueError(f"坐标 {key} 无效: {val}")
                # 检查 NaN (Not a Number)
                if math.isnan(val) or math.isinf(val):
                    raise ValueError(f"坐标 {key} 为 NaN 或 Inf")
                return float(val)

            try:
                rx = safe_get_coord(robot_pos, 'x')
                ry = safe_get_coord(robot_pos, 'y')
                rz = safe_get_coord(robot_pos, 'z', default=robot_pos.get('z', 0.0)) # 兼容无 Z 轴情况
                
                # 处理目标点 (假设 target_position 是对象，具有 x,y,z 属性)
                # 如果 target_position 可能是字典，也需要做类似处理，这里假设是对象
                tx = float(getattr(target_position, 'x', 0.0))
                ty = float(getattr(target_position, 'y', 0.0))
                tz = float(getattr(target_position, 'z', 0.0))
                
            except (ValueError, TypeError, AttributeError) as e:
                logger.error(f"坐标数据格式错误: {e}")
                return False

            # 4. 计算距离 (使用 hypot 更稳健，或者手动计算)
            # 如果是 2D 巡检 (Z 轴忽略)，可以只算 xy 距离。这里默认 3D。
            dx = rx - tx
            dy = ry - ty
            dz = rz - tz
            
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # 6. 判断结果
            if distance < 0.2 and rz < 0.2:
                return True
                
            return False

        except Exception as e:
            # 捕获所有未预料的异常，防止节点崩溃
            logger.exception(f"判断飞行状态时发生未知错误: {e}")
            return False
    
    def perform_action_walk_to_origin_pos(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        # 0 角度直接写死
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        self.goal_pose_pub_3d.publish(pose_msg)
        
        return pose_msg
        
    def handle_landing_state(self):
        
        """处理降落状态"""
        self.perform_action_walk_to_origin_pos()
        
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            robot_pos = self.topic_subscriber.get_position()
            logger.info(f'执行降落任务，当前点位置: ({robot_pos})')
            self.last_log_time = current_time  # 更新上次打印时间
                
        # 检查是否已降落到地面
        # 定义原点
        origin = type('obj', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})
        # 判断是否到达原点
        arrived = self._uav_is_flying_to_point(origin)
        if arrived:
            logger.info('已着陆')
            self.current_state = 'DISARMING'
                
    def handle_disarming_state(self):
        """处理上锁状态"""
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            logger.info('无人机已上锁，任务完成')
            # 切换到定点飞行模式（Position 模式）
            self.engage_position_mode()
            self.current_state = 'IDLE'
            
            
            if ENABLE_DOCK_CONTROL:
                # 任务完成
                try:
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    
                    dock_status, dock_msg = loop.run_until_complete(
                        self.dock_control_client.land_sequence()
                    )
                    logger.info(f"机舱控制: {dock_status}, {dock_msg}")
                    loop.close()
                    
                except Exception as e:
                    logger.error(f'降落序列执行异常: {str(e)}')
                    dock_status, dock_msg = False, str(e)
                        
        else:
            # 持续发送上锁命令直到成功
            logger.info('无人机发送Land命令')
            self.land()
            self.disarm()

    def arm(self):
        """发送命令使无人机解锁并准备飞行"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )
        logger.info("发送解锁命令")

    def disarm(self):
        """发送命令使无人机上锁"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0
        )
        logger.info("发送上锁命令")
    def land(self):
        """发送降落命令"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )
        logger.info("发送降落命令:Land command sent")

    def engage_position_mode(self):
        """启用定点飞行模式（Position 模式）"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,  # 主模式
            9.0  # POSITION 模式
        )
        logger.info("切换到定点飞行模式（Position 模式）")

    def engage_offboard_mode(self):
        """启用offboard模式"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,  # 主模式
            6.0  # OFFBOARD模式
        )
        logger.info("启用offboard模式")

    def publish_offboard_control_heartbeat_signal(self):
        """发布offboard控制模式信号"""
        msg = OffboardControlMode()
        msg.position = True  # 启用位置控制
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)  # 时间戳（微秒）
        self.offboard_control_mode_publisher.publish(msg)

    def publish_setpoint(self, x=0.0, y=0.0, z=None):
        """发布轨迹设定点"""
        if z is None:
            z = -2.5  # 默认高度2.5米

        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]  # 使用NaN表示不控制速度
        msg.acceleration = [float('nan'), float('nan'), float('nan')]  # 使用NaN表示不控制加速度
        msg.jerk = [float('nan'), float('nan'), float('nan')]  # 使用NaN表示不控制急动度
        msg.yaw = float('nan')  # 不控制偏航角
        msg.yawspeed = float('nan')  # 不控制偏航速率
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)  # 时间戳（微秒）
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """发布车辆命令"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)  # 时间戳（微秒）
        self.vehicle_command_publisher.publish(msg)

