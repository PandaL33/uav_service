#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人状态订阅处理类
负责订阅ROS话题并提供状态数据访问接口
包括机器人状态、位置、告警等信息的订阅
"""

from shlex import join
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from robot_interface.msg import RobotState
from robot_interface.msg import MotionData
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
import json
import math
from typing import Dict, Any, Optional, Callable
import logging
import copy 
import paho.mqtt.client as mqtt
from .status_report import StatusReport
from .device_status_report import DeviceStatusReport
from .video_preview_manager import VideoPreviewManager
from .service_manager import handle_service_list_request
import tf_transformations
from action_msgs.msg import GoalStatusArray
import requests  # 添加requests库用于HTTP请求
from .config import FFR_SERVER_URL, ROBOT_TYPE  # 引入配置文件中的URL
#from explore_lite.msg import ExploreStatus
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import BatteryStatus
from px4_msgs.msg import FailsafeFlags
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# 导入 PX4 原生消息
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude


logger = logging.getLogger('task_dispatcher')

class Ros2TopicSubscriber:
    """
    机器人状态订阅处理类
    作为订阅处理类而不是独立节点，需要被其他节点实例化和使用
    """
    
    def __init__(self, node: Node, mqtt_client: Optional[mqtt.Client] = None,
                 enable_status_report: bool = True, robot_sn: str = "",
                 mqtt_response_topic: str = "", version: str = "",
                 video_preview_manager: Optional[VideoPreviewManager] = None):
        """
        初始化订阅处理类
        
        Args:
            node: ROS2节点实例，用于创建订阅者
            mqtt_client: MQTT客户端实例
            enable_status_report: 是否启用状态上报功能
            min_report_interval: 最小上报间隔（秒）
            max_report_interval: 最大上报间隔（秒）
            robot_sn: 机器人序列号
            mqtt_response_topic: MQTT响应话题
            version: 版本号
            video_preview_manager: 视频预览管理器实例
        """
        self.node = node
        self._lock = threading.RLock()
        self._alarm_callback = None
        self._map_callback = None
        self._current_robot_state = None
        self._enable_status_report = enable_status_report
        self._status_report = None
        self._device_status_report = None
        self._video_preview_manager: Optional[VideoPreviewManager] = video_preview_manager
        
        # 如果需要启用状态上报
        if self._enable_status_report:
            # 创建StatusReport实例
            self._status_report = StatusReport(
                mqtt_client=mqtt_client,
                robot_sn=robot_sn,
                min_interval=5,
                max_interval=60,
                mqtt_response_topic=mqtt_response_topic,
                version=version
            )
            
            logger.info('Status report feature enabled')
            
            # 创建StatusReport实例
            self._device_status_report = DeviceStatusReport(
                mqtt_client=mqtt_client,
                robot_sn=robot_sn,
                min_interval=5,
                max_interval=60,
                mqtt_response_topic=mqtt_response_topic,
                version=version
            )
            
            logger.info('Device status report feature enabled')
        
        # 初始化状态数据字典
        self._robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'd': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
            'battery_level': 100.0,
            'battery_status': 0,  # 0: 未充电, 1: 充电中, 2: 充电完成
            'is_connected': True,
            'motion_status': 0,  # 0: 停止, 1: 前进, 2: 后退, 3: 旋转
            'algo_status': [0, 0, 0, 0],  # 四个算法状态
            'camera_status': [],  # 摄像头状态
            'ctrl_state': None,  # 控制状态
            'cruises': [],  # 巡航任务列表
            'services': [],  # 服务状态列表
            "takeoff_time": 0,  # 起飞时间
            "arming_state": 1,  # arming状态    
        }
        
        self._device_state = {
            # 全球定位信息（GPS/北斗等）
            "global_position": {
                # 纬度（单位：度，正数北纬，负数南纬）
                "latitude": 0.0,
                # 经度（单位：度，正数东经，负数西经）
                "longitude": 0.0,
                # 海拔高度（单位：米，相对于平均海平面 AMSL）
                "altitude_amsl": 0.0
            },
            # 本地相对位置（相对于起飞点）
            "local_position": {
                # 相对高度（单位：米，相对于起飞点的高度）
                "altitude_relative": 0.0,
                # 水平速度（单位：米/秒，地面坐标系下的水平方向速度）
                "velocity_horizontal": 0.0,
                # 垂直速度（单位：米/秒，正值上升，负值下降）
                "velocity_vertical": 0.0
            },
            # 无人机姿态（机身朝向）
            "attitude": {
                # 横滚角（单位：度，绕机身横轴旋转，-180~180）
                "roll_deg": 0.0,
                # 俯仰角（单位：度，绕机身纵轴旋转，-90~90）
                "pitch_deg": 0.0,
                # 偏航角（单位：度，绕机身竖轴旋转，0~360）
                "yaw_deg": 0.0
            }
        }

        self._navigate_to_pose_status = None
        
        # 最新状态数据
        self.vehicle_status: Optional[VehicleStatus] = None
        self.battery_status: Optional[BatteryStatus] = None
        self.failsafe_flags: Optional[FailsafeFlags] = None
        
        if ROBOT_TYPE == "default":
            # 创建订阅者
            self._create_subscribers() 
        # 消防机器人
        elif ROBOT_TYPE == "ffr":
            # 创建订阅者
            self._create_ffr_subscribers()
        elif ROBOT_TYPE == "uav":
            # 创建订阅者
            self._create_uav_subscribers()

        # 定时更新服务和摄像头状态
        # 立即执行一次
        self.update_services_and_cameras_status()
        # 然后每5分钟定时执行
        self.node.create_timer(10, self.update_services_and_cameras_status)
        
        logger.info('Ros2TopicSubscriber initialized')
    
    def _create_subscribers(self):
        """
        创建各种ROS话题的订阅者
        """
        # 订阅机器人状态话题
        self.node.create_subscription(
            RobotState,  # 假设robotstate使用String类型，包含JSON数据
            '/rkbot/robotstate',
            self._robot_state_callback,
            10
        )
        
        # 订阅运动状态话题
        self.node.create_subscription(
            MotionData,
            '/rkbot/motiondata',
            self._motion_data_callback,
            10
        )

        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # 订阅位置估计话题
        self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            amcl_qos
        )

        self.node.create_subscription(
            GoalStatusArray,  # 使用String类型接收JSON格式的告警信息
            '/navigate_to_pose/_action/status',
            self._navigate_to_pose_status_callback,
            10
        )

        self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/pcl_pose',
            self._pcl_pose_callback,
            amcl_qos
        )
        
        # self.node.create_subscription(
        #     ExploreStatus,  # 使用ExploreStatus类型接收地图信息
        #    '/explore/status',
        #    self._map_callback_handler,
        #    10
        #)
        
        # # 订阅告警信息话题
        # self.node.create_subscription(
        #     String,  # 使用String类型接收JSON格式的告警信息
        #     '/rkbot/alarm',
        #     self._alarm_callback_handler,
        #     10
        # )

        self.node.create_subscription(
            String,  # 使用String类型接收JSON格式的告警信息
            '/algo_result/trash_detect',
            self._trash_detect_callback,
            10
        )

        self.node.create_subscription(
            String,  # 使用String类型接收JSON格式的告警信息
            '/algo_result/device_check',
            self._device_check_callback,
            10
        )

        # self.node.create_subscription(
        #     String,  # 使用String类型接收JSON格式的告警信息
        #     '/algo_result/channel_monitor',
        #     self._channel_monitor_callback,
        #     10
        # )

        # self.node.create_subscription(
        #     String,  # 使用String类型接收JSON格式的告警信息
        #     '/algo_result/line_integrity',
        #     self._line_integrity_callback,
        #     10
        # )
    
    def _create_ffr_subscribers(self):
        """
        创建消防机器人的订阅者
        """
        try:
            # 创建定时器，每5秒调用一次HTTP接口获取机器人状态
            self.node.create_timer(10.0, self._get_ffr_robot_info)
            logger.info('消防机器人状态获取定时器已创建')
        except Exception as e:
            logger.error(f'创建消防机器人订阅者失败: {e}')

    def _create_uav_subscribers(self):
        """
        创建无人机的订阅者
        """
        try:
            logger.info('创建无人机状态订阅者')
            
            # 为PX4话题创建兼容的QoS配置
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            
            # PX4通常使用BEST_EFFORT可靠性和VOLATILE耐久性
            px4_qos_profile = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE
            )
            
            self.node.create_subscription(
                VehicleStatus,  # 假设vehicle_status使用VehicleStatus类型，包含JSON数据
                '/fmu/out/vehicle_status',
                self._vehicle_status_callback,
                px4_qos_profile
            )

            self.node.create_subscription(
                BatteryStatus,  # 假设battery_status使用BatteryStatus类型，包含JSON数据
                '/fmu/out/battery_status',
                self._battery_status_callback,
                px4_qos_profile
            )
            
            # 订阅故障状态
            self.failsafe_flags_sub = self.node.create_subscription(
                FailsafeFlags,
                '/fmu/out/failsafe_flags',
                self.failsafe_flags_callback,
                px4_qos_profile
            )
            
            self.node.create_subscription(
                VehicleGlobalPosition, 
                '/fmu/out/vehicle_global_position', 
                self._cb_device_global_pos, 
                px4_qos_profile)
            
            self.node.create_subscription(
                VehicleLocalPosition, 
                '/fmu/out/vehicle_local_position', 
                self._cb_device_local_pos, 
                px4_qos_profile)
            
            self.node.create_subscription(
                VehicleAttitude,
                '/fmu/out/vehicle_attitude', 
                self._cb_device_attitude,
                px4_qos_profile)

            pcl_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
            
            self.node.create_subscription(
                PoseWithCovarianceStamped,
                '/pcl_pose',
                self._pcl_pose_callback,
                pcl_qos
            )
            
        except Exception as e:
            logger.error(f'创建无人机订阅者失败: {e}')

    def get_vehicle_status(self):
        with self._lock:
            if self.vehicle_status is None:
                return None
            return copy.deepcopy(self.vehicle_status)
    
    def get_battery_status(self):
        with self._lock:
            if self.battery_status is None:
                return None
            return copy.deepcopy(self.battery_status)
    
    def get_failsafe_flags(self):
        with self._lock:
            if self.failsafe_flags is None:
                return None
            return copy.deepcopy(self.failsafe_flags)
    
    def _cb_device_global_pos(self, msg: VehicleGlobalPosition):
        try:
            with self._lock:
                if msg.lat == 0.0 and msg.lon == 0.0:
                    return

                self._device_state["global_position"]["latitude"] = round(float(msg.lat), 3)
                self._device_state["global_position"]["longitude"] = round(float(msg.lon), 3)
                self._device_state["global_position"]["altitude_amsl"] = round(float(msg.alt), 3)
                
                # logger.info(f"无人机定位信息:经度:{msg.lat}, 维度:{msg.lon},高度:{msg.alt}")
                
                # 如果启用了状态上报，更新状态
                if self._enable_status_report and self._device_status_report:
                    self._device_status_report.update_status(self.get_device_global_position_status())
        except Exception as e:
            logger.error(f'处理无人机状态消息时出错: {e}')

    def _cb_device_local_pos(self, msg: VehicleLocalPosition):
        try:
            with self._lock:
                self._device_state["local_position"]["altitude_relative"] = round(float(-msg.z),3)
                
                vx = float(msg.vx)
                vy = float(msg.vy)
                vz = float(msg.vz)
                
                self._device_state["local_position"]["velocity_horizontal"] = round(math.sqrt(vx**2 + vy**2),3)
                self._device_state["local_position"]["velocity_vertical"] = round(float(-vz), 3)
                
                # logger.info(f"无人机本地相对位置:相对高度:{-msg.z}, 水平速度:{math.sqrt(vx**2 + vy**2)},垂直速度:{-vz}")
                
                # 如果启用了状态上报，更新状态
                if self._enable_status_report and self._device_status_report:
                    self._device_status_report.update_status(self.get_device_local_position_status())
                    
        except Exception as e:
            logger.error(f'处理无人机状态消息时出错: {e}')
        
    def _cb_device_attitude(self, msg: VehicleAttitude):
        """
        核心修复:PX4 的 VehicleAttitude 通常包含四元数 q[4] (w, x, y, z)
        我们需要手动将其转换为欧拉角 (Roll, Pitch, Yaw)
        """
        try:
            # 1. 尝试读取四元数 (PX4 标准格式通常是 [w, x, y, z])
            # 注意：有些版本可能是 [x, y, z, w]，如果发现角度不对，可能需要调整顺序
            # 在 px4_msgs 中，q 通常是 float32[4]
            q = list(msg.q)
            
            # 假设顺序是 [w, x, y, z] (这是 ROS/px4 常见顺序，如果是 [x,y,z,w] 需调整)
            # 让我们先打印一下长度确认，如果报错再调整
            if len(q) != 4:
                logger.error(f"收到错误的四元数长度: {len(q)}")
                return

            # px4_msgs 的 q 定义通常是 [w, x, y, z]
            w, x, y, z = q[0], q[1], q[2], q[3]

            # 2. 四元数转欧拉角 (Z-Y-X 顺序，即 Yaw-Pitch-Roll)
            # Roll (x-axis rotation)
            sinr_cosp = 2 * (w * x + y * z)
            cosr_cosp = 1 - 2 * (x * x + y * y)
            roll = math.atan2(sinr_cosp, cosr_cosp)

            # Pitch (y-axis rotation)
            sinp = 2 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = math.copysign(math.pi / 2, sinp) # 使用 90 度如果超出范围
            else:
                pitch = math.asin(sinp)

            # Yaw (z-axis rotation)
            siny_cosp = 2 * (w * z + x * y)
            cosy_cosp = 1 - 2 * (y * y + z * z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            # 3. 更新缓存
            self._device_state["attitude"]["roll_deg"] = round(math.degrees(roll),3)
            self._device_state["attitude"]["pitch_deg"] = round(math.degrees(pitch),3)
            self._device_state["attitude"]["yaw_deg"] = round(math.degrees(yaw),3)
            
            # logger.info(f"无人机角度信息:rool:{math.degrees(roll)}, pitch:{math.degrees(pitch)}, yaw:{math.degrees(yaw)}")
            # 如果启用了状态上报，更新状态
            if self._enable_status_report and self._device_status_report:
                self._device_status_report.update_status(self.get_device_attitude_status())

        except AttributeError as e:
            # 如果连 msg.q 都没有，那可能是消息定义完全不同，打印可用属性调试
            logger.error(f"无法解析姿态数据: {e}")
            logger.error(f"可用属性: {[attr for attr in dir(msg) if not attr.startswith('_')]}")
        except Exception as e:
            logger.error(f"姿态解算错误: {e}")
            
    def _vehicle_status_callback(self, msg: VehicleStatus):
        """
        处理无人机状态消息
        
        Args:
            msg: 包含无人机状态信息的VehicleStatus消息
        """
        try:
            with self._lock:
                self.vehicle_status = msg
                # 记录无人机状态信息到日志
                self._robot_state['takeoff_time'] = msg.takeoff_time
                # ARMING_STATE_DISARMED = 1 —— 未解锁 / 上锁状态
                # ARMING_STATE_ARMED = 2 —— 已解锁 / 可飞行状态
                self._robot_state['arming_state'] = msg.arming_state
                # logger.info(f"无人机解锁状态: {msg.arming_state}, 起飞时间: {msg.takeoff_time}")

        except Exception as e:
            logger.error(f'处理无人机状态消息时出错: {e}')
    
    def _battery_status_callback(self, msg: BatteryStatus):
        """
        处理电池状态消息
        
        Args:
            msg: 包含电池状态信息的BatteryStatus消息
        """
        try:
            with self._lock:
                self.battery_status = msg
                # 从消息中获取电池信息
                remaining = msg.remaining if msg.remaining != 0.0 else 0.0
                battery_level = remaining * 100.0  # 将0-1范围转换为百分比
                
                # 根据电流判断充电状态
                # 当电流为负时，表示电池正在充电
                if msg.current_a < 0.0:
                    battery_status = 1  # 充电中
                else:
                    battery_status = 0  # 未充电
                # battery_status = 1 # test
                # 更新机器人状态
                self._robot_state['battery_level'] = battery_level
                self._robot_state['battery_status'] = battery_status
                
                # 记录详细的电池信息到日志
                logger.debug(f"电池状态更新: 电量: {battery_level:.2f}%, 电压: {msg.voltage_v:.3f}V, 电流: {msg.current_a:.3f}A, 滤波电压: {msg.voltage_filtered_v:.3f}V, 滤波电流: {msg.current_filtered_a:.3f}A, 剩余电量比例: {msg.remaining:.4f}, 电池单元数: {msg.cell_count}")
                
                # 如果启用了状态上报，更新状态
                if self._enable_status_report and self._status_report:
                    self._status_report.update_status(self.get_robot_status())
                    
        except Exception as e:
            logger.error(f'处理电池状态消息时出错: {e}')
    def failsafe_flags_callback(self, msg: FailsafeFlags):
        """故障状态回调函数"""
        self.failsafe_flags = msg
        
    def _get_ffr_robot_info(self):
        """
        调用HTTP接口获取消防机器人状态
        """
        try:
            url = f'{FFR_SERVER_URL}/fireRobot/getRobotInfo?isQueryPos=true'
            response = requests.get(url, timeout=2)
            
            if response.status_code == 200:
                data = response.json()
                
                if data.get('code') == 200:
                    robot_data = data.get('data', {})
                    logger.info(f'获取消防机器人状态成功: {json.dumps(robot_data)}')
                    
                    with self._lock:
                        # 更新充电状态
                        charging = robot_data.get('charging', False)
                        if charging:
                            self._robot_state['battery_status'] = 1  # 充电中
                        elif robot_data.get('soc', 0) >= 100:
                            self._robot_state['battery_status'] = 2  # 充电完成
                        else:
                            self._robot_state['battery_status'] = 0  # 未充电
                        
                        # 更新电量信息
                        self._robot_state['battery_level'] = robot_data.get('soc', 0)
                        
                        # 更新位置信息
                        pos = robot_data.get('pos', {})
                        # 注意：这里假设接口返回的位置是x, y, z, w格式
                        # 需要根据实际机器人坐标系统进行调整
                        self._robot_state['position'] = {
                            'x': float(pos.get('x', 0)) if pos.get('x') is not None else 0.0,
                            'y': float(pos.get('y', 0)) if pos.get('y') is not None else 0.0,
                            'z': float(pos.get('z', 0)) if pos.get('z') is not None else 0.0,
                            'w': float(pos.get('w', 0)) if pos.get('w') is not None else 0.0
                        }
                        
                        # 更新连接状态
                        self._robot_state['is_connected'] = True
                        
                        # 更新控制状态（假设默认是空闲状态）
                        self._robot_state['ctrl_state'] = 0
                        
                        logger.debug(f'获取消防机器人状态成功: {json.dumps(robot_data)}')
                        # 如果启用了状态上报，则更新状态
                        if self._enable_status_report and self._status_report:
                            self._status_report.update_status(self.get_robot_status())
                else:
                    logger.warning(f'获取消防机器人状态失败: {data.get("message", "未知错误")}')
            else:
                logger.error(f'HTTP请求失败，状态码: {response.status_code}')
                
        except requests.RequestException as e:
            logger.error(f'HTTP请求异常: {e}')
        except json.JSONDecodeError as e:
            logger.error(f'解析响应JSON失败: {e}')
        except Exception as e:
            logger.error(f'获取消防机器人状态异常: {e}')
        
    
    def _robot_state_callback(self, msg):
        """
        处理机器人状态消息
        支持两种消息格式：原生RobotState类型和JSON字符串格式
        根据RobotState.msg定义，包含connect(连接状态)、battery(电量)和state(控制状态)
        
        Args:
            msg: 机器人状态消息，可能是RobotState类型或String类型
        """
        try:
            # 处理原生RobotState消息类型
            if hasattr(msg, 'state') and hasattr(msg, 'battery') and hasattr(msg, 'connect'):
                with self._lock:
                    # 保存完整的状态消息以供外部访问
                    self._current_robot_state = msg
                    # 更新控制状态
                    self._robot_state['ctrl_state'] = msg.state.value
                    # 更新电池状态（直接使用uint32类型的battery字段）
                    self._robot_state['battery_level'] = msg.battery
                    # 更新连接状态
                    self._robot_state['is_connected'] = msg.connect
                    # 根据电池电量确定充电状态
                    if msg.battery >= 100:
                        self._robot_state['battery_status'] = 2  # 充电完成
                    elif msg.battery < 100 and msg.battery > 0:
                        self._robot_state['battery_status'] = 0  # 未充电
                    else:
                        self._robot_state['battery_status'] = 1  # 充电中（假设）
                    
                    logger.debug(f'接收到机器人状态: state={msg.state.value}, battery={msg.battery}, connect={msg.connect}')
            else:
                logger.warning(f'未知的机器人状态消息类型: {type(msg)}')
            
            # 如果启用了状态上报，则更新状态
            if self._enable_status_report and self._status_report:
                self._status_report.update_status(self.get_robot_status())
                    
        except json.JSONDecodeError as e:
            logger.error(f'Failed to parse robot state message: {e}')
        except Exception as e:
            logger.error(f'Error in robot state callback: {e}')
    
    def _motion_data_callback(self, msg: MotionData):
        """
        处理运动状态消息
        
        Args:
            msg: 包含运动状态的MotionData消息
        """
        try:
            pass
            #logger.debug(f'_motion_data_callback: {msg}')    
        except json.JSONDecodeError as e:
            logger.error(f'Failed to parse motion data message: {e}')
        except Exception as e:
            logger.error(f'Error in motion data callback: {e}')
    
    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        处理AMCL位置估计消息
        
        Args:
            msg: AMCL位置估计消息
        """
        try:
            with self._lock:
                # 更新位置信息
                self._robot_state['position']['x'] = msg.pose.pose.position.x
                self._robot_state['position']['y'] = msg.pose.pose.position.y
                self._robot_state['position']['z'] = msg.pose.pose.position.z
                ox = msg.pose.pose.orientation.x
                oy = msg.pose.pose.orientation.y
                oz = msg.pose.pose.orientation.z
                ow = msg.pose.pose.orientation.w
                # 转换为欧拉角（roll, pitch, yaw）
                roll, pitch, yaw = tf_transformations.euler_from_quaternion([ox, oy, oz, ow])
                self._robot_state['position']['d'] = yaw
                
                # 更新姿态信息
                #self._robot_state['orientation']['x'] = msg.pose.pose.orientation.x
                #self._robot_state['orientation']['y'] = msg.pose.pose.orientation.y
                #self._robot_state['orientation']['z'] = msg.pose.pose.orientation.z
                #self._robot_state['orientation']['w'] = msg.pose.pose.orientation.w
                #logger.info(f'接收到AMCL位置估计: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z}')
            # 如果启用了状态上报，则更新状态
            if self._enable_status_report and self._status_report:
                self._status_report.update_status(self.get_robot_status())
                
        except Exception as e:
            logger.error(f'Error in amcl pose callback: {e}')
            
    def _pcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        处理PCL位置估计消息
        
        Args:
            msg: PCL位置估计消息
        """
        try:
            with self._lock:
                # 更新位置信息
                self._robot_state['position']['x'] = msg.pose.pose.position.x
                self._robot_state['position']['y'] = msg.pose.pose.position.y
                self._robot_state['position']['z'] = msg.pose.pose.position.z
                ox = msg.pose.pose.orientation.x
                oy = msg.pose.pose.orientation.y
                oz = msg.pose.pose.orientation.z
                ow = msg.pose.pose.orientation.w
                # 转换为欧拉角（roll, pitch, yaw）
                roll, pitch, yaw = tf_transformations.euler_from_quaternion([ox, oy, oz, ow])
                self._robot_state['position']['d'] = yaw
                
                # 更新姿态信息
                #self._robot_state['orientation']['x'] = msg.pose.pose.orientation.x
                #self._robot_state['orientation']['y'] = msg.pose.pose.orientation.y
                #self._robot_state['orientation']['z'] = msg.pose.pose.orientation.z
                #self._robot_state['orientation']['w'] = msg.pose.pose.orientation.w
                #logger.info(f'接收到AMCL位置估计: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z}')
            # 如果启用了状态上报，则更新状态
            if self._enable_status_report and self._status_report:
                self._status_report.update_status(self.get_robot_status())
                
        except Exception as e:
            logger.error(f'Error in amcl pose callback: {e}')

    def _navigate_to_pose_status_callback(self, status: GoalStatusArray):
        """
        处理导航到目标位置状态消息
        
        Args:
            status: 包含导航状态的GoalStatusArray消息
        """
        try:
            # 处理导航状态消息
            sz = len(status.status_list)
            if sz > 0:
                status = status.status_list[sz-1]
                stamp = status.goal_info.stamp
                state = status.status
                logger.info(f'导航状态: stamp={stamp}, status={state}')
            self._navigate_to_pose_status = status
        except Exception as e:
            logger.error(f'Error in navigate to pose status callback: {e}')

    def _trash_detect_callback(self, msg: String):
        """
        处理垃圾检测消息
        
        Args:
            msg: 包含垃圾检测信息的String消息
        """
        try:
            self._process_alarm_message(msg)
            #logger.debug(f'_trash_detect_callback: {msg.data}')    
        except json.JSONDecodeError as e:
            logger.error(f'Failed to parse trash detect message: {e}')
        except Exception as e:
            logger.error(f'Error in trash detect callback: {e}')

    def _device_check_callback(self, msg: String):
        """
        处理设备检查消息
        
        Args:
            msg: 包含设备检查信息的String消息
        """
        try:
            self._process_alarm_message(msg)
            #logger.debug(f'_device_check_callback: {msg.data}')    
        except json.JSONDecodeError as e:
            logger.error(f'Failed to parse device check message: {e}')
        except Exception as e:
            logger.error(f'Error in device check callback: {e}')

    def _process_alarm_message(self, msg: String):
        """
        处理告警消息
        
        Args:
            msg: 包含告警信息的String消息
        """
        try:
            data = json.loads(msg.data)
            cruises = self.get_cruises()
            taskId = ''
            if len(cruises) == 1:
                taskId = cruises[0].get('taskId', '')
            data['taskId'] = taskId
            position = self.get_position()
            if position:
                data['pos'] = {
                    'd': position.get('d', 0.0),
                    'x': position.get('x', 0.0),
                    'y': position.get('y', 0.0),
                    'z': position.get('z', 0.0)
                }
            
            logger.info(f'收到告警信息: taskId={data.get("taskId")}, algo_name={data.get("algo_name")}')
            # 如果设置了回调函数，则调用外部回调处理告警
            if self._alarm_callback is not None:
                self._alarm_callback(data)
            else:
                logger.warning('收到告警信息但未设置告警处理回调函数')
        except json.JSONDecodeError as e:
            logger.error(f'Failed to parse alarm message: {e}')
        except Exception as e:
            logger.error(f'Error in alarm callback: {e}')
    def get_device_global_position_status(self) ->Dict[str, Any]:
        with self._lock:
            return self._device_state["global_position"].copy()
        
    def get_device_local_position_status(self) ->Dict[str, Any]:
        with self._lock:
            return self._device_state["local_position"].copy()
        
    def get_device_attitude_status(self) ->Dict[str, Any]:
        with self._lock:
            return self._device_state["attitude"].copy()
        
    def get_robot_status(self) -> Dict[str, Any]:
        """
        获取机器人完整状态数据
        
        Returns:
            包含所有机器人状态的字典
        """
        with self._lock:
            return self._robot_state.copy()
    
    def get_current_robot_state(self):
        """
        获取当前原始机器人状态消息
        
        Returns:
            RobotState消息对象
        """
        with self._lock:
            return self._current_robot_state if hasattr(self, '_current_robot_state') else None
    
    def get_current_ctrl_state(self) -> Optional[int]:
        """
        获取当前控制状态
        
        Returns:
            控制状态值
        """
        with self._lock:
            return self._robot_state.get('ctrl_state', 99)
    
    def get_position(self) -> Dict[str, float]:
        """
        获取机器人位置信息
        
        Returns:
            包含x, y, z坐标的字典
        """
        with self._lock:
            return self._robot_state['position'].copy()
    
    def get_battery_info(self) -> Dict[str, Any]:
        """
        获取电池信息
        
        Returns:
            包含电池电量和状态的字典
        """
        with self._lock:
            return {
                'battery_level': self._robot_state['battery_level'],
                'battery_status': self._robot_state['battery_status']
            }
    
    def get_algo_status(self) -> list:
        """
        获取算法状态
        
        Returns:
            算法状态列表
        """
        with self._lock:
            return self._robot_state['algo_status'].copy()
    
    def get_camera_status(self) -> list:
        """
        获取摄像头状态
        
        Returns:
            摄像头状态列表
        """
        return self._robot_state['camera_status'].copy()

    def get_services_status(self) -> list:
        """
        获取服务状态
        
        Returns:
            服务状态列表
        """
        return self._robot_state['services'].copy()

    def update_services_and_cameras_status(self):
        """
        更新服务和摄像头状态
        """
        # 更新服务状态
        # logger.info('更新服务状态')
        self._robot_state['services'] = handle_service_list_request()
        
        # 更新摄像头状态
        # logger.info('更新摄像头状态')
        self._robot_state['camera_status'] = self._video_preview_manager.get_camera_status()
        
        # 如果启用了状态上报，则更新状态
        if self._enable_status_report and self._status_report:
            self._status_report.update_status(self.get_robot_status())
    
    def update_algorithm_status(self, index: int, status: int) -> bool:
        """
        更新指定索引的算法状态
        
        Args:
            index: 算法索引 (0-3)
            status: 状态值
            
        Returns:
            更新是否成功
        """
        if 0 <= index < len(self._robot_state['algo_status']):
            with self._lock:
                self._robot_state['algo_status'][index] = status
            
            # 如果启用了状态上报，则更新状态
            if self._enable_status_report and self._status_report:
                self._status_report.update_status(self.get_robot_status())
            
            return True
        return False
    
    def _alarm_callback_handler(self, msg: String):
        """
        处理告警信息回调
        
        Args:
            msg: 包含告警信息的String消息
        """
        try:
            # 解析告警消息
            alarm_data = json.loads(msg.data)
            logger.info(f'收到告警信息: taskId={alarm_data.get("taskId")}, algo_name={alarm_data.get("algo_name")}')
            
            # 如果设置了回调函数，则调用外部回调处理告警
            if self._alarm_callback is not None:
                self._alarm_callback(alarm_data)
            else:
                logger.warning('收到告警信息但未设置告警处理回调函数')
                
        except json.JSONDecodeError as e:
            logger.error(f'解析告警消息失败: {e}')
        except Exception as e:
            logger.error(f'处理告警信息时出错: {e}')
    
    def set_alarm_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        设置告警处理回调函数
        
        Args:
            callback: 处理告警数据的回调函数
        """
        self._alarm_callback = callback
        logger.info('已设置告警处理回调函数')

    def set_map_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """
        设置地图回调函数
        
        Args:
            callback: 处理地图数据的回调函数
        """
        self._map_callback = callback
        logger.info('已设置地图回调函数')

    # def _map_callback_handler(self, msg: ExploreStatus):
    #     """
    #     处理地图信息回调
        
    #     Args:
    #         msg: 包含地图信息的ExploreStatus消息
    #     """
    #     try:
    #         # 解析地图消息
    #         map_data = msg
    #         logger.info(f'收到地图信息: {map_data}')
            
    #         # 如果设置了回调函数，则调用外部回调处理地图信息
    #         if self._map_callback is not None:
    #             self._map_callback(map_data)
    #         else:
    #             logger.warning('收到地图信息但未设置地图处理回调函数')
                
    #     except Exception as e:
    #         logger.error(f'处理地图信息时出错: {e}')

    def set_cruises(self, cruises: list):
        """
        设置当前巡航任务列表
        
        Args:
            cruises: 巡航任务列表
        """
        with self._lock:
            self._robot_state['cruises'] = cruises
            # 如果启用了状态上报，则更新状态
            if self._enable_status_report and self._status_report:
                self._status_report.update_status(self.get_robot_status())

    def get_cruises(self) -> list:
        """
        获取当前巡航任务列表
        
        Returns:
            当前巡航任务列表
        """
        with self._lock:
            return self._robot_state['cruises'].copy()

    def get_navigate_to_pose_status(self):
        """
        获取导航到目标位置状态
        
        Returns:
            导航到目标位置状态
        """
        return self._navigate_to_pose_status
    
    def set_navigate_to_pose_status(self, status = None):
        """
        设置导航到目标位置状态
        
        Args:
            status: 导航到目标位置状态
        """
        self._navigate_to_pose_status = status

    def get_takeoff_time_and_arming_state(self) -> tuple:
        """
        获取起飞时间和arming状态
        
        Returns:
            起飞时间和arming状态
        """
        with self._lock:
            return self._robot_state['takeoff_time'], self._robot_state['arming_state'] 


# 用于测试的示例函数
# 当这个文件作为独立脚本运行时使用
# 实际使用时应在其他节点中实例化此类
def main():
    rclpy.init()
    test_node = rclpy.create_node('test_ros2_topic_subscriber')
    
    subscriber = Ros2TopicSubscriber(test_node)
    
    try:
        # 运行一段时间，接收消息
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()