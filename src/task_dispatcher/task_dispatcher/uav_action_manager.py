import logging
import json
from rclpy.client import Client
import threading
import numpy as np
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from geometry_msgs.msg import PoseStamped
from typing import Optional, Dict, Any
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
import math
import gpiod
import time
# 配置日志
logger = logging.getLogger('task_dispatcher.uav_action_manager')

class UavActionManager:
    """
    执行动作管理器，负责处理各种巡检任务动作
    """
    
    def __init__(self, node, topic_subscriber):
        """
        初始化执行动作管理器
        
        Args:
            node: ROS2节点实例
            topic_subscriber: ROS2主题订阅器实例
        """
        self.node = node
        self.topic_subscriber: Optional[Ros2TopicSubscriber] = topic_subscriber
        
        self.vehicle_command_publisher = self.node.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )
        
        self.uav_current_state = "IDLE"
        self.is_landed = False
        self.is_armed = False
        
        self.goal_pose_pub_3d = self.node.create_publisher(PoseStamped, "/goal_pose_3d", 10)
        
        self.last_log_time = time.time() 
        self.log_interval = 10.0  # 设置间隔为 10 秒
        
        logger.info('执行动作管理器初始化完成')
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
        
    def disarm(self):
        """发送命令使无人机上锁"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0
        )
        logger.info("发送上锁命令")
        
    def exec_land_command(self):
        """处理降落状态"""
        self.current_state = 'LANDING'
        while self.current_state == 'LANDING':
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
                time.sleep(1)
                # timeout_count = 0
                while self.current_state == 'DISARMING':
                    takeofftime, arming_state = self.topic_subscriber.get_takeoff_time_and_arming_state()
                    logger.info(f"arming_state: {arming_state}")
                    if arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                        logger.info('无人机已上锁，任务完成')
                        # 切换到定点飞行模式（Position 模式）
                        self.engage_position_mode()
                        self.current_state = 'IDLE'
                        
                        return            
                    else:
                        # 持续发送上锁命令直到成功
                        logger.info('无人机发送Land命令')
                        self.land()
                        self.disarm()
                    time.sleep(0.5)
                
                # timeout_count += 1
                # if timeout_count > 240: # 超时 2分钟 (240 * 0.5s)
                #     logger.error("强制超时退出")
                #     self.current_state = 'IDLE'
                #     break
                
            time.sleep(0.5)
            
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
            logger.warning(f"判断飞行状态时发生未知错误: {e}")
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