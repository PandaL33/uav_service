#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import json
from typing import Dict, Any, Optional
from multi_algo_interfaces.srv import AlgoControl
import rclpy
from robot_interface.srv import Move, SetCtrlMode
import time
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32
import tf_transformations
from action_msgs.msg import GoalStatusArray, GoalStatus
from task_dispatcher.ptz_control import PtzControl
import threading
import numpy as np
from uav_command_sender import UavCommandSender

# 配置日志
logger = logging.getLogger('perform_action_manager')

class PerformActionManager:
    """
    执行动作管理器，负责处理各种巡检任务动作
    """
    
    def __init__(self, node, ptz_controller=None, topic_subscriber=None, speed_topic_prefix=''):
        """
        初始化执行动作管理器
        
        Args:
            node: ROS2节点实例
            ptz_controller: PTZ控制器实例
            topic_subscriber: ROS2主题订阅器实例
            speed_topic_prefix: 速度话题前缀，默认 ''（即 /goal_vel_max 与 /goal_vel_max_ack）
        """
        self.node = node
        self.ptz_controller: Optional[PtzControl] = ptz_controller
        self.topic_subscriber: Optional[Ros2TopicSubscriber] = topic_subscriber
        self.uav_command_sender = UavCommandSender(node, topic_name='/uav_command')

        self._lock = threading.Lock()
        self.stop_action = False

        self.goal_pose_pub = self.node.create_publisher(PoseStamped, "/goal_pose", 10)
        self.goal_pose_pub_3d = self.node.create_publisher(PoseStamped, "/goal_pose_3d", 10)

        # 失效降级互通：发布 /mission_event（String+JSON，复用方案 §14 事件集与字段）
        self.mission_event_pub = self.node.create_publisher(String, "/mission_event", 10)

        # ---- 逐航点实时调速 ----
        # 请求 /goal_vel_max，planner 收到后用 /goal_vel_max_ack 回确认；
        # 等不到 ack 时最多重发 3 次，仍无 ack 则乐观记录并继续（不阻塞任务）。
        # 默认速度 2.0 m/s，与 planner manager/max_vel、网页面板三处保持一致。
        self.speed_topic_prefix = speed_topic_prefix
        self._default_speed = 2.0
        self._speed_adopted = self._default_speed   # 已生效速度缓存（未变速则不再发）
        self._speed_ack_value: Optional[float] = None
        self._speed_ack_event = threading.Event()
        self._speed_ack_retry = 3                    # 无 ack 时的最大重发次数
        self._speed_ack_timeout = 0.3                # 单次等待 ack 超时(s)
        self.speed_pub = self.node.create_publisher(
            Float32, f"{self.speed_topic_prefix}/goal_vel_max", 10)
        self.speed_ack_sub = self.node.create_subscription(
            Float32, f"{self.speed_topic_prefix}/goal_vel_max_ack", self._on_speed_ack, 10)

        # 航点停滞判定参数（不简化，实际环境日飞，判据可配置）
        #   stall_time_s    : 连续无有效推进的秒数阈值，触发一次停滞判定
        #   stall_dist_min  : 单次判定期内必须至少推进的水平距离(m)，否则视为停滞
        #   stall_progress  : 停滞判定期的采样间隔(s)
        self.waypoint_stall_time_s = 2.0          # 2s 无推进 → 判定不可达
        self.waypoint_progress_window = 2.0       # 横滚窗口，每 2s 评估一次推进
        self.waypoint_min_progress = 0.10         # 2s 内须推进 >= 0.10m，否则停滞计数+1

        # ROS2 服务客户端（只创建一次，避免 WaitSet 溢出）
        self.algo_client = self.node.create_client(AlgoControl, '/algo_control')
        self.move_client = self.node.create_client(Move, '/rkbot/move')
        self.mode_client = self.node.create_client(SetCtrlMode, '/rkbot/setctrlmode')

        logger.info('执行动作管理器初始化完成')
    
    def angle_to_quat(self, angle_rad, axis='z'):
        """
        将弧度角转换为四元数 (x, y, z, w)。
        适用于 2D 平面旋转。
        
        参数:
            angle_rad: 旋转角度（弧度，float）
            axis: 旋转轴，可选 'z' (默认), 'y', 或 'x'。
                'z': 绕 Z 轴旋转 (常用于 XY 平面，如地面移动)
                'y': 绕 Y 轴旋转 (常用于 XZ 平面，如俯视地图)
                'x': 绕 X 轴旋转
        
        返回:
            quat: numpy array [x, y, z, w]
        """
        half_angle = angle_rad * 0.5
        sin_half = np.sin(half_angle)
        cos_half = np.cos(half_angle)
        
        if axis == 'z':
            # 绕 Z 轴旋转 (XY 平面)
            return np.array([0.0, 0.0, sin_half, cos_half])
        elif axis == 'y':
            # 绕 Y 轴旋转 (XZ 平面)
            return np.array([0.0, sin_half, 0.0, cos_half])
        elif axis == 'x':
            # 绕 X 轴旋转 (YZ 平面)
            return np.array([sin_half, 0.0, 0.0, cos_half])
        else:
            raise ValueError("axis 必须是 'x', 'y', 或 'z'")
    
    def safe_float(self, value, default=0.0):
        """尝试将值转换为 float，如果失败（如 None, 非法字符串），返回默认值"""
        if value is None:
            return default
        try:
            return float(value)
        except (ValueError, TypeError):
            return default
    
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
            # 根据目标位置设置不同的距离阈值
            if tx == 0.0 and ty == 0.0 and tz == 1.0:
                threshold = 0.2
            else:
                threshold = 0.4
            if distance < threshold:
                return True
                
            return False

        except Exception as e:
            # 捕获所有未预料的异常，防止节点崩溃
            logger.exception(f"判断飞行状态时发生未知错误: {e}")
            return False

    def _on_speed_ack(self, msg):
        """planner 的速度确认回调：记录采纳值并唤醒等待方"""
        try:
            self._speed_ack_value = float(msg.data)
        except (TypeError, ValueError):
            self._speed_ack_value = None
        self._speed_ack_event.set()

    def _ensure_speed(self, pos: Dict[str, Any]):
        """
        按航点下发速度（仅在速度变化时发），并等待 planner 的 ack 确认。

        规则：
          - pos 无 speed 字段（返航/迫降/悬停等临时点）→ 不发，沿用上一航点速度；
          - 与已生效速度差值 <= 1e-3 → 视为未变速，不发；
          - 否则发布并等待 ack，最多重发 _speed_ack_retry 次；
            收到任意 ack 即以 ack 携带的采纳值更新缓存；全部超时则乐观更新本地值并告警继续。
        """
        if not isinstance(pos, dict):
            return
        raw = pos.get('speed', None)
        if raw is None:
            # 未指定速度：不改不发，沿用上一航点速度
            return

        target = self.safe_float(raw, self._default_speed)
        if abs(target - self._speed_adopted) <= 1e-3:
            # 速度未变，无需下发
            return

        for attempt in range(1, self._speed_ack_retry + 1):
            self._speed_ack_value = None
            self._speed_ack_event.clear()

            msg = Float32()
            msg.data = float(target)
            self.speed_pub.publish(msg)
            logger.info(f'下发速度请求: {target} m/s (第{attempt}次), 当前已生效={self._speed_adopted} m/s')

            if self._speed_ack_event.wait(self._speed_ack_timeout):
                adopted = self._speed_ack_value if self._speed_ack_value is not None else target
                self._speed_adopted = adopted
                logger.info(f'速度请求已确认: 请求={target} m/s, planner 采纳={adopted} m/s')
                return

            logger.warning(f'等待速度 ack 超时 ({attempt}/{self._speed_ack_retry}), target={target} m/s')

        # 重发耗尽仍无 ack：乐观记录本地值，继续执行不阻塞任务
        self._speed_adopted = target
        logger.warning(f'速度请求未收到 ack，已重发 {self._speed_ack_retry} 次后放弃，'
                       f'本地乐观记录为 {target} m/s（实际可能未生效）')

    def _publish_mission_event(self, event: str, **fields):
        """
        发布 /mission_event（String+JSON）。事件名与字段按完整失效降级方案 §14 对齐：
        复用既有事件集与必带字段 flight_epoch/goal_seq，仅在 JSON 内扩展可选字段。

        事件集（方案 §14）：GOAL_UNREACHABLE / GOAL_SUBSTITUTED / RETURN_INIT /
        RETURNED_LANDED / RETURN_BLOCKED / LIO_RED / ...
        必带字段：flight_epoch / goal_seq（维护于调用方；无则传 'NA'）
        可选字段（本次新增语义）：count / consecutive_goals / next / reason
        """
        try:
            payload = {"event": event}
            payload.update(fields)
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.mission_event_pub.publish(msg)
            logger.info(f"[mission_event] {msg.data}")
        except Exception as e:
            logger.warning(f"发布 mission_event 失败: {e}")

    def _get_current_target_dist(self, tx: float, ty: float, tz: float) -> Optional[float]:
        """返回当前无人机到目标点的水平距离；取不到位置返回 None"""
        try:
            robot_pos = self.topic_subscriber.get_position()
            if not robot_pos:
                return None
            rx = float(robot_pos.get('x', 0.0))
            ry = float(robot_pos.get('y', 0.0))
            return math.hypot(rx - tx, ry - ty)
        except Exception:
            return None

    def _is_airborne(self) -> bool:
        """飞机是否已起飞（解锁且 takeoff_time>0）。起桨/离地前返回 False。"""
        try:
            takeoff_time, arming_state = self.topic_subscriber.get_takeoff_time_and_arming_state()
            return takeoff_time > 0 and arming_state == 2
        except Exception:
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
    
    def perform_action_walk_to_pos_3d(self, pos: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行飞行动作到目标点
        
        Args:
            pos: 坐标点
                d: 方向角
                x: X坐标
                y: Y坐标
                z: Z坐标
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if not pos:
                logger.error('缺少必须参数: pos')
                return {
                    'success': False,
                    'message': '缺少必须参数: pos'
                }
            
            # 验证pos参数的结构
            required_pos_fields = ['d', 'x', 'y', 'z']
            for field in required_pos_fields:
                if field not in pos:
                    logger.error(f'pos参数缺少必须字段: {field}')
                    return {
                        'success': False,
                        'message': f'pos参数缺少必须字段: {field}'
                    }
            
            d = pos['d']
            x = pos['x']
            y = pos['y']
            z = pos['z']
            
            logger.info(f'执行飞行动作: 目标坐标点(x={x}, y={y}, z={z}, d={d})')

            # 下发本航点速度（仅在速度变化时发，并等待 planner ack）
            self._ensure_speed(pos)

            # 发送目标点给导航服务
            while True:
                if self.stop_action:
                    logger.info(f'收到停止指令，飞行动作提前结束, 当前坐标点: {self.topic_subscriber.get_position()}')
                    return {
                        'success': False,
                        'message': '收到停止指令，飞行动作提前结束'
                    }

                # 清空导航到目标位置状态
                # self.topic_subscriber.set_navigate_to_pose_status(None)
                # 1.将目标点发送给导航服务
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.node.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"

                
                x_pos = self.safe_float(pos.get("x"), 0.0)
                y_pos = self.safe_float(pos.get("y"), 0.0)
                z_pos = self.safe_float(pos.get("z"), 0.0)
                yaw = self.safe_float(pos.get("d"), 0.0)
                
                pose_msg.pose.position.x = x_pos
                pose_msg.pose.position.y = y_pos
                pose_msg.pose.position.z = z_pos

                # 注意：四元数分量必须用独立变量名，绝不能复用 x/y/z，
                # 否则会把航点坐标覆盖成四元数分量，停滞 watchdog 会误把 (0,0) 当目标
                qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                pose_msg.pose.orientation.x = float(qx)
                pose_msg.pose.orientation.y = float(qy)
                pose_msg.pose.orientation.z = float(qz)
                pose_msg.pose.orientation.w = float(qw)
                logger.info(f"publish goal_pose_3d: {pose_msg} angle={math.degrees(yaw):.2f}")

                # 2.等待是否到达目标点（含停滞 watchdog，避免 0918「悬停至耗尽电量」）
                # 停滞判据：每 waypoint_progress_window 秒评估一次水平推进；
                #   连续 waypoint_stall_time_s 秒无有效推进 → 判定本航点不可达（REACH_FAIL）。
                # 返回码语义：success=False 且 message 前缀 GOAL_UNREACHABLE 表示「停滞不可达」，
                #   由编排层(cruise)累计连续失败并决定 跳点/返航/迫降。
                st_stall_accum = 0.0          # 累计停滞秒数
                st_last_dist = self._get_current_target_dist(x, y, z)
                st_last_sample_t = time.time()
                st_pause_logged = False       # 起飞前暂停判定的提示只打一次
                while True:
                    self.goal_pose_pub_3d.publish(pose_msg)

                    if self.stop_action:
                        logger.info(f'收到停止指令，飞行动作提前结束, 当前坐标点: {self.topic_subscriber.get_position()}')
                        return {
                            'success': False,
                            'message': '收到停止指令，飞行动作提前结束'
                        }

                    to_target = self._uav_is_flying_to_point(pose_msg.pose.position)
                    if to_target:
                        logger.info(f'飞行动作执行成功, 当前坐标点: {self.topic_subscriber.get_position()}')
                        time.sleep(1.0)
                        return {
                            'success': True,
                            'message': f'飞行动作执行成功: 目标坐标点(x={x}, y={y}, z={z}, d={d})'
                        }

                    # ---- 起飞前暂停判定 ----
                    # 航点不可达的判断必须在起飞后：起桨/离地阶段飞机静止属正常，
                    # 此时不累计停滞，持续重发目标点并复位采样基准，起飞后再开始判定。
                    if not self._is_airborne():
                        st_stall_accum = 0.0
                        st_last_dist = self._get_current_target_dist(x, y, z)
                        st_last_sample_t = time.time()
                        if not st_pause_logged:
                            logger.info(f'飞机尚未起飞，暂停停滞不可达判定, 目标坐标点: ({x}, {y}, {z})')
                            st_pause_logged = True
                        time.sleep(0.2)
                        continue

                    # ---- 停滞 watchdog ----
                    now_t = time.time()
                    if st_last_sample_t is not None and now_t - st_last_sample_t >= self.waypoint_progress_window:
                        cur_dist = self._get_current_target_dist(x, y, z)
                        if cur_dist is None or st_last_dist is None:
                            pass  # 位置暂不可得，本轮不判定，保持采样
                        else:
                            progressed = st_last_dist - cur_dist   # >0 表示在靠近
                            if progressed < self.waypoint_min_progress:
                                st_stall_accum += self.waypoint_progress_window
                                if st_stall_accum >= self.waypoint_stall_time_s:
                                    logger.error(f'航点停滞 {st_stall_accum:.1f}s 无有效推进，判定不可达: '
                                                 f'目标(x={x}, y={y}, z={z})')
                                    # topic 互通：向编排层/外部广播不可达（字段按方案 §14，goal_seq/flight_epoch 由编排层累计）
                                    self._publish_mission_event(
                                        'GOAL_UNREACHABLE',
                                        reason='STALLED',
                                        goal_seq='NA',
                                        flight_epoch='NA',
                                        target=(f'{x:.2f},{y:.2f},{z:.2f}'))
                                    return {
                                        'success': False,
                                        'message': f'GOAL_UNREACHABLE: 航点停滞不可达 (x={x}, y={y}, z={z})'
                                    }
                            else:
                                st_stall_accum = 0.0   # 有效推进，停滞累计清零
                            st_last_dist = cur_dist
                        st_last_sample_t = now_t

                    logger.debug(f'无人机飞行中, 当前坐标点: {self.topic_subscriber.get_position()}')
                    time.sleep(0.2)
        except Exception as e:
            logger.error(f'执行飞行动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'飞行动作执行失败: {str(e)}'
            }
               
    def perform_action_walk_to_pos(self, pos: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行行走动作到目标点
        
        Args:
            pos: 坐标点
                d: 方向角
                x: X坐标
                y: Y坐标
                z: Z坐标
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if not pos:
                logger.error('缺少必须参数: pos')
                return {
                    'success': False,
                    'message': '缺少必须参数: pos'
                }
            
            # 验证pos参数的结构
            required_pos_fields = ['d', 'x', 'y', 'z']
            for field in required_pos_fields:
                if field not in pos:
                    logger.error(f'pos参数缺少必须字段: {field}')
                    return {
                        'success': False,
                        'message': f'pos参数缺少必须字段: {field}'
                    }
            
            d = pos['d']
            x = pos['x']
            y = pos['y']
            z = pos['z']
            
            logger.info(f'执行行走动作: 目标坐标点(x={x}, y={y}, z={z}, d={d})')
            
            # 发送目标点给导航服务
            cnt = 0
            while cnt < 3:
                if self.stop_action:
                    logger.info(f'收到停止指令，行走动作提前结束, 当前坐标点: {self.topic_subscriber.get_position()}')
                    return {
                        'success': False,
                        'message': '收到停止指令，行走动作提前结束'
                    }

                # 清空导航到目标位置状态
                self.topic_subscriber.set_navigate_to_pose_status(None)
                # 1.将目标点发送给导航服务
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.node.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.position.x = pos.get("x", 0.0)
                pose_msg.pose.position.y = pos.get("y", 0.0)
                pose_msg.pose.position.z = 0.0 #pos.get("z", 0.0)
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                yaw = pos.get("d", 0.0)
                x, y, z, w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
                pose_msg.pose.orientation.z = float(z)
                pose_msg.pose.orientation.w = float(w)
                logger.info(f"publish goal_pose: {pose_msg} angle={math.degrees(yaw):.2f}")
                self.goal_pose_pub.publish(pose_msg)
                # 2.等待导航服务状态更新响应
                start_time = time.time()
                #navigate_status = self.topic_subscriber.get_navigate_to_pose_status()
                while True:
                    if self.stop_action:
                        logger.info(f'收到停止指令，行走动作提前结束, 当前坐标点: {self.topic_subscriber.get_position()}')
                        return {
                            'success': False,
                            'message': '收到停止指令，行走动作提前结束'
                        }
                    # 能够接收到导航服务状态更新
                    navigate_status = self.topic_subscriber.get_navigate_to_pose_status()
                    if navigate_status:
                        logger.info(f"导航服务状态更新: {navigate_status}")
                        break
                    # 超时处理
                    elapsed = time.time() - start_time
                    if elapsed > 30.0:
                        logger.error(f"等待导航姿态变化超时, 当前坐标点: {self.topic_subscriber.get_position()}")
                        return {
                            'success': False,
                            'message': f'行走动作执行超时: 目标坐标点(x={x}, y={y}, z={z}, d={d})'
                        }
                    time.sleep(0.1)

                # 3. 检查是否到达目标点
                start_time = time.time()
                posestat_pre = self.topic_subscriber.get_navigate_to_pose_status()
                posestat_chkcnt = 0
                while True:
                    if self.stop_action:
                        logger.info(f'收到停止指令，行走动作提前结束, 当前坐标点: {self.topic_subscriber.get_position()}')
                        return {
                            'success': False,
                            'message': '收到停止指令，行走动作提前结束'
                        }
                    posestat_cur = self.topic_subscriber.get_navigate_to_pose_status()
                    status = posestat_cur.status
                    logger.info(f"navipose status={status}")
                    # 执行成功
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        logger.info(f'行走动作执行成功, 导航服务状态: {status}, 当前坐标点: {self.topic_subscriber.get_position()}')
                        return {
                            'success': True,
                            'message': f'行走动作执行成功: 目标坐标点(x={x}, y={y}, z={z}, d={d})'
                        }
                    # 执行失败
                    elif status == GoalStatus.STATUS_CANCELED or status == GoalStatus.STATUS_ABORTED:
                        logger.error(f'行走动作执行失败, 导航服务状态: {status}, 当前坐标点: {self.topic_subscriber.get_position()}')
                        return {
                            'success': False,
                            'message': f'行走动作执行失败: 目标坐标点(x={x}, y={y}, z={z}, d={d})'
                        }
                    # 执行超时（状态未更新）
                    if posestat_pre == posestat_cur:
                        posestat_chkcnt += 1
                        if posestat_chkcnt > 360:
                            logger.error(f"更新导航姿态超时, 当前坐标点: {self.topic_subscriber.get_position()}")
                            return {
                                'success': False,
                                'message': f'行走动作执行超时: 目标坐标点(x={x}, y={y}, z={z}, d={d})'
                            }
                    # 状态更新，证明机器人还在走
                    else:
                        posestat_chkcnt = 0
                        posestat_pre = posestat_cur
                    time.sleep(1.0)
            
            return {
                'success': True,
                'message': f'行走动作执行成功: 目标坐标点(x={x}, y={y}, z={z}, d={d})'
            }
            
        except Exception as e:
            logger.error(f'执行行走动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'行走动作执行失败: {str(e)}'
            }
    
    def perform_action_stay(self, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行停留动作
        
        Args:
            argv: 停留参数
                duration: 持续时间，单位毫秒
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if 'duration' not in argv:
                return {
                    'success': False,
                    'message': '缺少必须参数: duration'
                }
            
            # 先停止移动
            result = self.perform_action_stop()
            if not result['success']:
                return result

            duration = int(argv['duration'] / 1000.0)
            logger.info(f'执行停留动作: 持续时间={duration}秒')
            start_time = time.time()
            while time.time() - start_time < duration:
                if self.stop_action:
                    logger.info('收到停止指令，停留动作提前结束')
                    break
                time.sleep(0.5)
            
            return {
                'success': True,
                'message': f'停留动作执行成功: 持续时间={duration}秒'
            }
            
        except Exception as e:
            logger.error(f'执行停留动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'停留动作执行失败: {str(e)}'
            }
    
    def _move_service_callback(self, future, service_name: str):
        """
        移动服务回调函数
        
        Args:
            future: 异步调用结果
            service_name: 服务名称
        """
        try:
            response = future.result()
            logger.info(f'{service_name} 服务调用成功, result={response}')
        except Exception as e:
            logger.error(f'{service_name} 服务调用异常: {str(e)}')
    
    def perform_action_algo(self, algoId: str, enable: bool, cameraId: str = None) -> Dict[str, Any]:
        """
        执行算法动作
        
        Args:
            algoId: 算法ID
            cameraId: 摄像机ID
            enable: 使能，true-开启、false-关闭
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if not algoId:
                return {
                    'success': False,
                    'message': '缺少必须参数: algoId'
                }
            
            logger.info(f'执行算法动作: 算法ID={algoId}, 使能={enable}')

            # 等待服务可用（复用 __init__ 中创建的客户端）
            if not self.algo_client.wait_for_service(timeout_sec=3.0):
                logger.error('算法控制服务不可用')
                return {
                    'success': False,
                    'message': '算法控制服务不可用'
                }
            
            # 调用服务
            request = AlgoControl.Request()
            if algoId == "trash_detect":
                request.algo_name = "TrashDetect"
            elif algoId == "device_check":
                request.algo_name = "DeviceCheck"
            elif algoId == "channel_monitor":
                request.algo_name = "ChannelMonitor"
            elif algoId == "line_integrity":
                request.algo_name = "LineIntegrity"
            else:
                logger.error(f"不支持的算法ID: {algoId}")
                return {
                    'success': False,
                    'message': f'不支持的算法ID: {algoId}'
                }
            if enable:
                request.action = "start"
            else:
                request.action = "stop"
            
            future = self.algo_client.call_async(request)
            future.add_done_callback(lambda f: self._algo_service_callback(f, algoId))
            
            return {
                'success': True,
                'message': f'算法动作执行成功: 算法ID={algoId}, 使能={enable}'
            }
            
        except Exception as e:
            logger.error(f'执行算法动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'算法动作执行失败: {str(e)}'
            }
    
    def _algo_service_callback(self, future, algoId: str):
        """
        处理算法控制服务的异步响应
        """
        try:
            response = future.result()
            logger.info(f'算法控制服务调用成功: algoId={algoId}, result={response}')
        except Exception as e:
            logger.error(f'算法控制服务调用异常: algoId={algoId}, {str(e)}')
            
    def perform_action_gpio(self, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行PTZ控制动作
        
        Args:
            argv: GPIO控制参数
            level: 电平直
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if 'level' not in argv or not argv['level']:
                return {
                    'success': False,
                    'message': '缺少必须参数: level'
                }
            
            level = argv['level']
            
            logger.info(f'执行GPIO控制动作: 电平直={level}')
            success = self.uav_command_sender.send_command(f'GPIO:{level}')
            if success:
                return {
                    'success': True,
                    'message': f'GPIO控制动作执行成功,电平直:{level}'
                }
            else:
                return {
                    'success': False,
                    'message': f'GPIO控制动作执行失败,电平直:{level}'
                }   
            
        except Exception as e:
            logger.error(f'执行GPIO控制动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'GPIO控制动作执行失败: {str(e)}'
            }
            
    def perform_action_ptz(self, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行PTZ控制动作
        
        Args:
            argv: PTZ控制参数
                cameraId: 摄像机ID
                ctrlType: 控制类型
                ctrlParams: 控制参数
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if 'cameraId' not in argv or not argv['cameraId']:
                return {
                    'success': False,
                    'message': '缺少必须参数: cameraId'
                }
            if 'ctrlType' not in argv or not argv['ctrlType']:
                return {
                    'success': False,
                    'message': '缺少必须参数: ctrlType'
                }
            if 'ctrlParams' not in argv or not argv['ctrlParams']:
                return {
                    'success': False,
                    'message': '缺少必须参数: ctrlParams'
                }
            
            camera_id = argv['cameraId']
            ctrl_type = argv['ctrlType']
            ctrl_params = argv['ctrlParams']
            
            logger.info(f'执行PTZ控制动作: 摄像机ID={camera_id}, 控制类型={ctrl_type}, 控制参数={ctrl_params}')
            success = self.ptz_controller.execute_ptz_control(camera_id, ctrl_type, ctrl_params)
            if success:
                return {
                    'success': True,
                    'message': f'PTZ控制动作执行成功: 摄像机ID={camera_id}, 控制类型={ctrl_type}'
                }
            else:
                return {
                    'success': False,
                    'message': f'PTZ控制动作执行失败: 摄像机ID={camera_id}, 控制类型={ctrl_type}'
                }   
            
        except Exception as e:
            logger.error(f'执行PTZ控制动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'PTZ控制动作执行失败: {str(e)}'
            }
    
    def perform_action_rtspsnap(self, rtsp, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行抓拍动作
        
        Args:
            argv: 抓拍参数
                cameraId: 摄像机ID
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            # if 'cameraId' not in argv or not argv['cameraId']:
            #     raise ValueError('缺少必须参数: cameraId')
            # if 'interval' not in argv:
            #     raise ValueError('缺少必须参数: interval')
            # if 'count' not in argv:
            #     raise ValueError('缺少必须参数: count')
            
            # camera_id = argv['cameraId']
            # interval = argv['interval']
            # count = argv['count']
            
            # # 验证参数值
            # if not isinstance(interval, int) or interval <= 0:
            #     raise ValueError('interval参数必须是正整数')
            # if not isinstance(count, int) or count <= 0:
            #     raise ValueError('count参数必须是正整数')
            
            # logger.info(f'执行抓拍动作: 摄像机ID={camera_id}, 抓拍间隔={interval}, 抓拍次数={count}')
            
            # 具体控制逻辑后续补充
            # TODO: 实现抓拍动作控制
            
            return {
                'success': True,
                'message': f'抓拍动作执行成功'
            }
            
        except Exception as e:
            logger.error(f'执行抓拍动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'抓拍动作执行失败: {str(e)}'
            }
    def perform_action_snap(self, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行抓拍动作
        
        Args:
            argv: 抓拍参数
                cameraId: 摄像机ID
                interval: 抓拍间隔
                count: 抓拍次数
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if 'cameraId' not in argv or not argv['cameraId']:
                raise ValueError('缺少必须参数: cameraId')
            if 'interval' not in argv:
                raise ValueError('缺少必须参数: interval')
            if 'count' not in argv:
                raise ValueError('缺少必须参数: count')
            
            camera_id = argv['cameraId']
            interval = argv['interval']
            count = argv['count']
            
            # 验证参数值
            if not isinstance(interval, int) or interval <= 0:
                raise ValueError('interval参数必须是正整数')
            if not isinstance(count, int) or count <= 0:
                raise ValueError('count参数必须是正整数')
            
            logger.info(f'执行抓拍动作: 摄像机ID={camera_id}, 抓拍间隔={interval}, 抓拍次数={count}')
            
            # 具体控制逻辑后续补充
            # TODO: 实现抓拍动作控制
            
            return {
                'success': True,
                'message': f'抓拍动作执行成功: 摄像机ID={camera_id}, 抓拍间隔={interval}, 抓拍次数={count}'
            }
            
        except Exception as e:
            logger.error(f'执行抓拍动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'抓拍动作执行失败: {str(e)}'
            }
    
    def perform_action_record(self, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行录像动作
        
        Args:
            argv: 录像参数
                cameraId: 摄像机ID
                duration: 持续时间，单位毫秒
                enable: 使能，true-开启、false-关闭
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if 'cameraId' not in argv or not argv['cameraId']:
                raise ValueError('缺少必须参数: cameraId')
            if 'duration' not in argv:
                raise ValueError('缺少必须参数: duration')
            if 'enable' not in argv:
                raise ValueError('缺少必须参数: enable')
            
            camera_id = argv['cameraId']
            duration = argv['duration']
            enable = argv['enable']
            
            # 验证参数值
            if not isinstance(duration, int) or duration <= 0:
                raise ValueError('duration参数必须是正整数')
            if not isinstance(enable, bool):
                raise ValueError('enable参数必须是布尔值')
            
            logger.info(f'执行录像动作: 摄像机ID={camera_id}, 持续时间={duration}毫秒, 使能={enable}')
            
            # 具体控制逻辑后续补充
            # TODO: 实现录像动作控制
            
            return {
                'success': True,
                'message': f'录像动作执行成功: 摄像机ID={camera_id}, 持续时间={duration}毫秒, 使能={enable}'
            }
            
        except Exception as e:
            logger.error(f'执行录像动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'录像动作执行失败: {str(e)}'
            }
    
    def perform_action_skip(self, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行跳过动作
        
        Args:
            argv: 跳过参数
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            logger.info(f'执行跳过动作')
            
            return {
                'success': True,
                'message': '跳过动作执行成功'
            }
            
        except Exception as e:
            logger.error(f'执行跳过动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'跳过动作执行失败: {str(e)}'
            }
    
    def perform_action_stop(self, argv: Dict[str, Any] = {}) -> Dict[str, Any]:
        """
        执行停止动作
        
        Args:
            argv: 停止参数
                force: 是否强行停止，默认为false
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            force = argv.get('force', False)
            
            logger.info(f'执行停止动作: 强行停止={force}')

            # 等待服务可用（复用 __init__ 中创建的客户端）
            if not self.move_client.wait_for_service(timeout_sec=3.0):
                # 降级：/rkbot/move 缺失时不阻断任务（如 stay 悬停由规划器自身保位），
                # 仅告警并以 success 继续，避免「悬停动作」因缺服务直接失败
                logger.warning('移动服务 /rkbot/move 不可用，跳过停止移动指令（降级继续）')
                return {
                    'success': True,
                    'message': '移动服务不可用，已跳过停止移动指令（降级继续）'
                }

            # 准备请求
            request = Move.Request()
            request.move.vx = 0.0
            request.move.vy = 0.0
            request.move.yaw_rate = 0.0

            # 发送异步请求
            future = self.move_client.call_async(request)
            future.add_done_callback(lambda future: self._move_service_callback(future, 'move'))
            
            return {
                'success': True,
                'message': f'停止动作执行成功: 强行停止={force}'
            }
            
        except Exception as e:
            logger.error(f'执行停止动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'停止动作执行失败: {str(e)}'
            }
    
    def perform_action_back(self, pos: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行回到原点动作
        
        Args:
            pos: 回到原点
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            logger.info(f'执行回到原点动作: 回到原点={pos}')
            
            return self.perform_action_walk_to_pos_3d(pos)

        except Exception as e:
            logger.error(f'执行回到原点动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'回到原点动作执行失败: {str(e)}'
            }
    
    def perform_action_stand(self, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行站立动作
        
        Args:
            argv: 站立参数
                mode: 0-趴下、1-四足直站、2-蹲站
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if 'mode' not in argv:
                logger.error('缺少必须参数: mode')
                return {
                    'success': False,
                    'message': '缺少必须参数: mode'
                }
            
            mode = argv['mode']
            
            mode_map = {
                0: '趴下',
                1: '四足直站',
                2: '蹲站'
            }
            
            logger.info(f'执行站立动作: 模式={mode_map[mode]}({mode})')

            # 等待服务可用（复用 __init__ 中创建的客户端）
            if not self.mode_client.wait_for_service(timeout_sec=3.0):
                logger.error('控制模式服务不可用')
                return {
                    'success': False,
                    'message': '控制模式服务不可用'
                }

            # 准备请求
            request = SetCtrlMode.Request()

            # 设置模式值, 0-趴下、1-四足直站、2-蹲站;
            request.mode.value = mode

            logger.info(f'调用控制模式服务: mode_value={request.mode.value}')

            # 异步调用服务
            future = self.mode_client.call_async(request)
            future.add_done_callback(lambda f: self._mode_service_callback(f, mode_map[mode]))

            # 等待状态
            start_time = time.time()
            while True:
                current_state = self.topic_subscriber.get_current_ctrl_state()
                if current_state in [2, 3] and mode == 1:
                    logger.info(f'机器人当前状态: {current_state}，四足直站完成')
                    break
                if current_state == 4 and mode == 2:
                    logger.info(f'机器人当前状态: {current_state}，蹲站完成')
                    break
                # 趴下不等待
                if mode == 0:
                    logger.info(f'机器人当前状态: {current_state}，趴下完成')
                    break
                
                # 超时处理
                if time.time() - start_time > 30.0:
                    logger.error(f'{mode_map[mode]}超时')
                    return {
                        'success': False,
                        'message': f'{mode_map[mode]}超时'
                    }
                
                time.sleep(0.1)
            
            return {
                'success': True,
                'message': f'站立动作执行成功: 模式={mode_map[mode]}({mode})'
            }
            
        except Exception as e:
            logger.error(f'执行站立动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'站立动作执行失败: {str(e)}'
            }
    

    def _mode_service_callback(self, future, act_name):
        """
        处理控制模式服务的异步响应
        """
        try:
            response = future.result()
            logger.info(f'控制模式服务调用成功: act_name={act_name}, result={response}')
        except Exception as e:
            logger.error(f'控制模式服务调用异常: act_name={act_name}, {str(e)}')

    
    def perform_action_drive(self, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        执行驾驶动作
        
        Args:
            argv: 驾驶参数
                direction: 方向：left、right、up、down、leftUp、leftdown、rightUp、rightDown
                speed: 速度: low、mid、high
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            # 验证必须参数
            if 'direction' not in argv or not argv['direction']:
                raise ValueError('缺少必须参数: direction')
            if 'speed' not in argv or not argv['speed']:
                raise ValueError('缺少必须参数: speed')
            
            direction = argv['direction']
            speed = argv['speed']
            
            # 验证参数值
            valid_directions = ['left', 'right', 'up', 'down', 'leftUp', 'leftdown', 'rightUp', 'rightDown']
            if direction not in valid_directions:
                raise ValueError(f'direction参数值无效，必须是: {valid_directions}')
            
            valid_speeds = ['low', 'mid', 'high']
            if speed not in valid_speeds:
                raise ValueError(f'speed参数值无效，必须是: {valid_speeds}')
            
            logger.info(f'执行驾驶动作: 方向={direction}, 速度={speed}')
            
            # 具体控制逻辑后续补充
            # TODO: 实现驾驶动作控制
            
            return {
                'success': True,
                'message': f'驾驶动作执行成功: 方向={direction}, 速度={speed}'
            }
            
        except Exception as e:
            logger.error(f'执行驾驶动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'驾驶动作执行失败: {str(e)}'
            }
    
    def perform_action(self, action_type: str, argv: Dict[str, Any]) -> Dict[str, Any]:
        """
        根据动作类型执行相应的动作
        
        Args:
            action_type: 动作类型
            argv: 动作参数
        
        Returns:
            Dict[str, Any]: 动作执行结果
        """
        try:
            action_type = action_type.lower()
            
            # 根据动作类型调用相应的方法
            if action_type == 'walk':
                return self.perform_action_walk_to_pos(argv)
            elif action_type == 'stay':
                return self.perform_action_stay(argv)
            elif action_type == 'algo':
                return self.perform_action_algo(argv)
            elif action_type == 'ptz':
                return self.perform_action_ptz(argv)
            elif action_type == 'snap':
                return self.perform_action_snap(argv)
            elif action_type == 'record':
                return self.perform_action_record(argv)
            elif action_type == 'skip':
                return self.perform_action_skip(argv)
            elif action_type == 'stop':
                return self.perform_action_stop(argv)
            elif action_type == 'back':
                return self.perform_action_back(argv)
            elif action_type == 'stand':
                return self.perform_action_stand(argv)
            elif action_type == 'drive':
                return self.perform_action_drive(argv)
            else:
                raise ValueError(f'未知的动作类型: {action_type}')
                
        except Exception as e:
            logger.error(f'执行动作失败: {str(e)}')
            return {
                'success': False,
                'message': f'执行动作失败: {str(e)}'
            }

    def set_stop_action(self, stop: bool):
        """
        设置是否停止当前动作
        
        Args:
            stop: 是否停止当前动作
        """
        with self._lock:
            self.stop_action = stop
        logger.info(f'设置停止动作: {stop}')
