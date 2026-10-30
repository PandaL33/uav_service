#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import logging
import math
import time
import threading
import os
import base64
import requests
import rclpy
import asyncio
import subprocess
from std_msgs.msg import String, Empty
from px4_msgs.msg import VehicleCommand, VehicleStatus
from typing import Dict, List, Optional, Any, Tuple
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
import uuid
from task_dispatcher.message_cache import message_cache_manager
import task_dispatcher.ptz_control as PtzControl
from task_dispatcher.perform_action_manager import PerformActionManager
from task_dispatcher.map_manager import MapManager
from task_dispatcher.point_cloud_persistor import PointCloudPersistor
from task_dispatcher.rtsp_record_manager import RtspRecordManager
from enum import Enum
from std_srvs.srv import Trigger
from rclpy.client import Client
from requests.auth import HTTPBasicAuth
from .config import ROBOT_TYPE, PCD_FILE_PATH, ENABLE_MANUAL_CONTROL, AUTH_CONFIG
from .config import ENABLE_PREFLIGHT_CHECK, ENABLE_DOCK_CONTROL, WIND_SPEED_THRESHOLD, RAINFALL_THRESHOLD  # 引入配置文件中的URL和点云文件路径
from .config import WAYPOINT_CONSEC_LIMIT  # 连续不可达航点数 → 返航的阈值
from uav_command_sender import UavCommandSender
from dock_control_client import DockControlClient
from task_dispatcher.preflight_check_node import PreFlightCheckNode
from task_dispatcher.uav_action_manager import UavActionManager
from auth_utils import AuthManager
from task_dispatcher.pcd_chunk_upload import FileChunkUploader
import open3d as o3d
# 配置日志
logger = logging.getLogger('task_dispatcher.cruise_uav_task_manager')

class TaskStatus(Enum):
    # 启动
    STARTED = 0
    # 进行中
    IN_PROGRESS = 1
    # 完成
    COMPLETED = 2
    # 中断
    ABORTED = 3
    # 取消
    CANCELED = 4

class CruiseUavTaskManager:
    """
    无人机巡检任务管理器，负责处理巡检任务相关命令
    支持：设置任务、获取任务、执行任务和停止任务
    """
    
    def __init__(self, node, mqtt_client, response_topic, ros_topic_subscriber, version='1.0', robot_sn='', 
        ptz_controller=None, server_url=None, perform_action_manager=None, map_manager=None):
        """
        初始化巡检任务管理器
        
        Args:
            node: ROS2节点实例
            mqtt_client: MQTT客户端实例
            response_topic: MQTT响应主题
            ros_topic_subscriber: 机器人状态订阅器实例
            version: 版本号
            robot_sn: 机器人序列号
            ptz_controller: PTZ控制器实例
            server_url: 地图服务器URL
            perform_action_manager: 执行动作管理器实例
            map_manager: 地图管理器实例
        """
        self.node = node
        self.mqtt_client = mqtt_client
        self.mqtt_response_topic = response_topic
        self.ros_topic_subscriber: Ros2TopicSubscriber = ros_topic_subscriber
        self.perform_action_manager: PerformActionManager = perform_action_manager
        self.map_manager: MapManager = map_manager
        self.server_url = server_url
        self.auth_manager = AuthManager(server_url)

        # 分片上传器（用于大文件如点云的分片上传）
        self.file_chunk_uploader = FileChunkUploader(
            base_url=server_url,
            service_user=AUTH_CONFIG['USERNAME'],
            service_password=AUTH_CONFIG['PASSWORD'],
        )

        # 点云持久化管理器
        self.point_cloud_persistor = PointCloudPersistor(node, server_url)
        # 设置上传回调函数
        self.point_cloud_persistor.set_upload_callback(self.upload_point_cloud_to_file_server)
        # 巡检录像功能
        self.rtst_record_manager = RtspRecordManager(server_url)
        # ros2_uav_service 控制指令发送
        self.uav_command_sender = UavCommandSender(node, topic_name='/uav_command')
        # 机舱控制
        self.dock_control_client = DockControlClient(node)
        # 降落后「闭合推杆 → 打开充电」串行线程（与点云上传并行执行）
        self._dock_seq_lock = threading.Lock()
        self._dock_seq_thread: Optional[threading.Thread] = None
        self._dock_seq_triggered = False
        # 导航服务是否已停止（点云落盘后提前停止，任务收尾兜底时不重复执行）
        self._ego_planner_stopped = False
        # 已降落并进入点云上传阶段：此阶段允许充电巡检介入（任务尚未结束，is_executing 仍为 True）
        self._post_landing_phase = False
        # 每 3 分钟补发一次打开充电
        if ENABLE_DOCK_CONTROL:
            threading.Thread(target=self._dock_charge_watchdog_loop, daemon=True,
                             name='dock_charge_watchdog').start()
        # 点云保存服务客户端（只创建一次，避免 WaitSet 溢出）
        self.map_save_client = self.node.create_client(Trigger, '/map_save')
        # 自检
        self.preflight_check_node = PreFlightCheckNode(
                    node=self.node,
                    topic_subscriber=self.ros_topic_subscriber,
                )
        
        self.uav_action_manager = UavActionManager(node, topic_subscriber=self.ros_topic_subscriber)
    
        # 存储巡检任务的字典，key为taskId
        self.tasks: Dict[str, Dict] = {}
        # 当前正在执行的任务ID
        self.current_task_id: Optional[str] = None
        # 任务执行状态标志
        self.is_executing: bool = False
        self._task_thread: Optional[threading.Thread] = None  # 当前任务线程引用（即时指令接管时 join 用）
        # 线程锁，用于保护共享变量
        self._lock = threading.Lock()
        # 巡航任务列表
        self.cruises: List[Dict[str, Any]] = []

        # 巡航任务中的PTZ控制信息
        self.ptz_controller: Optional[PtzControl] = ptz_controller
        # 版本号
        self.version = version
        
        # 机器人序列号
        self.robot_sn = robot_sn
        
        # 速度映射表
        self.speed_map = {
            'low': 0.2,
            'mid': 0.5,
            'high': 0.8
        }

        # 失效降级互通：发布 /mission_event（String+JSON，复用方案 §14）
        self.mission_event_pub = self.node.create_publisher(String, "/mission_event", 10)

        # 失效降级互通：订阅 /mission_event，接收 ego planner 侧的 OBSTACLE 上报，
        # 与 perform_action 返回值（STALLED）共同计入不可达判定（时间窗去重防双计）。
        self.mission_event_sub = self.node.create_subscription(
            String, "/mission_event", self._mission_event_callback, 10)
        self.last_obstacle_ts = 0.0   # 同 reason 去重时间窗起点
        self._unreachable_dedup_win = 2.0  # 2s 内同一 reason 只计一次（间隔 >=2s 视为新的有效上报）

        # 航点失效判定状态（编排侧，跨航点累计）
        #  same_point_retry : 同一航点不可达重试计数（>=3 → 跳过该航点）
        #  consec_goals     : 连续不同航点不可达计数（>=WAYPOINT_CONSEC_LIMIT → 返航）
        self.same_point_retry = 0
        self.consec_goals = 0
        self.goal_seq_counter = 0
        self.waypoint_max_retry = 3        # 同一航点最多重试 3 次
        self.waypoint_consec_limit = WAYPOINT_CONSEC_LIMIT  # 连续 N 个航点不可达 → 返航（config.py 可配）

        # ===== 平台即时飞控指令（悬停(暂停) / 续航 / 降落（即迫降）/ 直线返航 / 沿线返航）=====
        # ego_planner 的控制入口：traj_server 收到 /stop_planning 会停止发送轨迹并清掉
        # 目标点与 SZD 状态（xtd2_traj_server.cpp stopPlanningCallback）。没新目标点时它
        # 改为持续发送“当前位置”设定点，即飞机原地悬停 —— 这是降落前必须先把控制权
        # 交回飞控的原因，否则 planner 的 offboard 设定点会与 PX4 原生 LAND/RTL 抢控制。
        self.stop_planning_pub = self.node.create_publisher(Empty, "/stop_planning", 10)
        # 已成功到达的航点（“沿线返航”按此倒序退回）。只在新任务开始时清空，
        # reset_status() 不清 —— 否则停止任务后就找不到来路航点了。
        self.visited_waypoints: List[Dict[str, Any]] = []
        # 即时指令互斥：同一时刻只跑一条，避免降落与返航互相抢控制权
        self.instant_cmd: Optional[str] = None
        # HOVER 暂停标志：航线线程停在当前航点的停靠门等 CONTINUE 续航，
        # 任务状态（task_id/cruises/visited_waypoints）原样保留
        self.task_paused: bool = False

        logger.info('巡检任务管理器初始化完成')

    def _mission_event_callback(self, msg: String):
        """订阅 /mission_event：记录 ego planner 的不可达上报（只做观测/去重）。

        不可达计数统一由 _handle_waypoint_unreachable 在每次 move 尝试失败时计一次，
        本回调不再累加 same_point_retry，避免 planner 事件与 STALLED 返回值重复计数。
        """
        try:
            data = json.loads(msg.data)
            event = data.get('event')
            source = data.get('source', '')
            if event != 'GOAL_UNREACHABLE':
                return
            reason = data.get('reason', '')
            # planner 侧三类不可达事件均为真失效，保留日志便于排查（2s 时间窗去重防刷屏）：
            #   OBSTACLE      : 目标点落在膨胀障碍区，planner 直接拒绝该目标
            #   PLAN_FAILED   : GEN_NEW_TRAJ 连续 3 次规划失败 → 急停悬停并清目标回 WAIT_TARGET
            #   REPLAN_FAILED : REPLAN_TRAJ 连续 3 次重规划失败 → 急停悬停并清目标回 WAIT_TARGET
            if source == 'planner' and reason in ('OBSTACLE', 'PLAN_FAILED', 'REPLAN_FAILED'):
                now = time.time()
                with self._lock:
                    if now - self.last_obstacle_ts < self._unreachable_dedup_win:
                        return
                    self.last_obstacle_ts = now
                logger.warning(f"[mission_event] ego planner 上报 {reason}，"
                               f"不可达计数由 STALLED 统一累计")
        except Exception as e:
            logger.warning(f"解析 /mission_event 失败: {e}")

    def _publish_mission_event(self, event: str, **fields):
        """发布 /mission_event（String+JSON），字段按完整失效降级方案 §14 对齐。"""
        try:
            payload = {"event": event}
            payload.update(fields)
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.mission_event_pub.publish(msg)
            logger.info(f"[mission_event] {msg.data}")
        except Exception as e:
            logger.warning(f"发布 mission_event 失败: {e}")

    def _handle_waypoint_unreachable(self, task_id: str, pos: Dict[str, Any]) -> bool:
        """
        处理单个航点不可达（GOAL_UNREACHABLE）。返回 True=任务应终止(返航/迫降)，False=继续。

        判定规则（编排侧，完整判断不简化）：
          - 同一航点不可达累计 count >= waypoint_max_retry(3) → 跳过该航点(GOAL_SUBSTITUTED)，
            累计 consec_goals += 1；未到 3 次则等待重试(GOAL_UNREACHABLE 继续累计)。
          - 连续不同航点不可达 count >= waypoint_consec_limit（config.WAYPOINT_CONSEC_LIMIT，默认 3）→ RETURN_INIT 触发沿线返航。
          - 返航段（RETURN_BLOCKED）：沿线关键点一路跳过，全部不可达则退化为直线返航；
            安全点均不可达时由 _emergency_land 迫降兜底。
        """
        # 共享计数器读写与 _mission_event_callback 共锁（两线程：任务线程 + executor 回调线程）
        with self._lock:
            self.same_point_retry += 1
            cnt = self.same_point_retry
            consec = self.consec_goals
            seq = self.goal_seq_counter
        self._publish_mission_event(
            'GOAL_UNREACHABLE',
            goal_seq=seq,
            flight_epoch=getattr(self, 'flight_epoch', 'NA'),
            count=cnt,
            reason='STALLED')
        logger.warning(f"[航点失效] task={task_id} 航点不可达, 同点重试={cnt}/{self.waypoint_max_retry}, "
                       f"连续跨点失败={consec}/{self.waypoint_consec_limit}")

        # 1) 同一航点未达重试上限 → 返回继续，让外层重试同一航点
        if cnt < self.waypoint_max_retry:
            return False

        # 2) 同一航点达上限 → 跳过该航点，去下一航点；累计跨点失败
        with self._lock:
            self.same_point_retry = 0
            self.consec_goals += 1
            self.goal_seq_counter += 1
            consec = self.consec_goals
            seq = self.goal_seq_counter
        next_seq = seq + 1
        self._publish_mission_event(
            'GOAL_SUBSTITUTED',
            goal_seq=seq,
            next=next_seq,
            flight_epoch=getattr(self, 'flight_epoch', 'NA'),
            reason='SKIP_3X')
        logger.warning(f"[跳点] task={task_id} 跳过航点 go to next={next_seq}, 连续跨点失败={consec}/{self.waypoint_consec_limit}")

        # 3) 连续跨点失败达上限 → 返航
        if consec >= self.waypoint_consec_limit:
            logger.error(f"[返航] task={task_id} 连续 {consec} 个航点不可达，触发返航")
            self._publish_mission_event(
                'RETURN_INIT',
                goal_seq=seq,
                flight_epoch=getattr(self, 'flight_epoch', 'NA'),
                consec_goals=consec,
                trigger='REACH_FAIL')
            # 返航执行：按已到达的关键点沿线退回（途中不可达的关键点一路跳过，
            # 全部跳过则退化为直线返航，仍不可达则由 _emergency_land 迫降兜底）。
            # 在锁外执行，避免长阻塞持锁。
            params = {'land_timeout': 60.0, 'route_max_points': 0}
            self._return_home_along_route(task_id, params)
            return True

        # 4) 尚未达连续上限，跳过本航点继续下一航点（外层循环自然 continue）
        return False

    def _return_to_origin(self, task_id: str) -> bool:
        """返航回原点；返航途中遇障无法推进 → RETURN_BLOCKED 并转迫降兜底。"""
        self._publish_mission_event(
            'RETURN_INIT',
            goal_seq=self.goal_seq_counter,
            flight_epoch=getattr(self, 'flight_epoch', 'NA'),
            consec_goals=self.consec_goals,
            trigger='RETURN')
        # 返航两段式（与直线返航语义一致）：先发 (0,0,1) 回原点上方悬停点，
        # 到达后再发 (0,0,0)（planner 进 ±0.5m 安全区自动切 SZD 降落），
        # 避免直接发 (0,0,0) 让飞机在未知障碍的高度上贴地穿越
        hover_pos = {'x': 0.0, 'y': 0.0, 'z': 1.0, 'd': 0.0}
        result = self.perform_action_manager.perform_action_walk_to_pos_3d(hover_pos)
        if not result.get('success', False):
            logger.error(f"[返航受阻] task={task_id} 去原点悬停点 (0,0,1) 不可达，转入迫降兜底")
            self._publish_mission_event(
                'RETURN_BLOCKED',
                goal_seq=self.goal_seq_counter,
                flight_epoch=getattr(self, 'flight_epoch', 'NA'),
                reason='OBSTACLE_BLOCKED')
            self._emergency_land(task_id)
            return True
        origin_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'd': 0.0}
        result = self.perform_action_manager.perform_action_walk_to_pos_3d(origin_pos)
        success = result.get('success', False)
        if not success:
            logger.error(f"[返航受阻] task={task_id} 返航末段降落不可达，原地迫降")
            self._publish_mission_event(
                'RETURN_BLOCKED',
                goal_seq=self.goal_seq_counter,
                flight_epoch=getattr(self, 'flight_epoch', 'NA'),
                reason='OBSTACLE_BLOCKED')
            # 已在原点上方悬停点，原地迫降比再飞回远处安全点更安全
            self._emergency_land(task_id, try_safe_point=False)
            return True
        logger.info(f"[返航完成] task={task_id} 已回原点")
        return True

    def _emergency_land(self, task_id: str, try_safe_point: bool = True) -> bool:
        """迫降兜底：先退到已确认安全的位置再迫降，避免在未知地形/人群上方垂直下降。

        候选安全点按序尝试，任一到达即在该处下发 LAND：
          1) 最近一个已到达的关键点（航线未标记关键点时取来路最后一个航点）
          2) 起飞点上方悬停点 (0,0,1)
        全部不可达才在原地迫降。try_safe_point=False 用于「已到达原点悬停点、
        仅末段下降受阻」的场景：此时原地迫降比再飞回远处航点更安全。
        """
        if try_safe_point:
            with self._lock:
                history = list(self.visited_waypoints)
            key_points = [p for p in history if p.get('key')]
            if key_points:
                candidates = [key_points[-1]]
            elif history:
                candidates = [history[-1]]
            else:
                candidates = []
            candidates.append({'x': 0.0, 'y': 0.0, 'z': 1.0, 'd': 0.0})

            reached = False
            for pos in candidates:
                target = {'x': pos['x'], 'y': pos['y'], 'z': pos['z'],
                          'd': pos.get('d', 0.0)}
                logger.warning(f"[迫降] task={task_id} 先尝试退到安全点: {target}")
                try:
                    result = self.perform_action_manager.perform_action_walk_to_pos_3d(target)
                except Exception as e:
                    logger.error(f"[迫降] 退往 {target} 异常: {e}")
                    continue
                if result.get('success', False):
                    logger.warning(f"[迫降] task={task_id} 已到达安全点 {target}，就地迫降")
                    reached = True
                    break
                logger.warning(f"[迫降] 安全点不可达: {target}, {result.get('message', '')}")
            if not reached:
                logger.error(f"[迫降] task={task_id} 所有安全点均不可达，原地迫降")

        logger.error(f"[迫降] task={task_id} 执行应急迫降")
        # 先停规划，把控制权交回飞控，避免 planner 的 offboard 设定点与 PX4 原生 LAND 抢控制
        self._publish_stop_planning()
        try:
            self.uav_command_sender.send_command('LAND')
            return True
        except Exception as e:
            logger.error(f"[迫降] 下发 LAND 失败: {e}")
            return False

    # ==================================================================================
    # 平台即时飞控指令：悬停(暂停) / 续航 / 降落（即迫降）/ 直线返航 / 沿线返航
    #
    # 统一走同一条链路：先接管（停当前航点目标点发布）-> 停规划（/stop_planning 把控制权
    # 交回飞控）-> 再执行；回执分两级：Ack 只代表“已受理并接管”，落地/返航完成由
    # ActionReport 收尾（taskProgress 2=完成 3=中断）。
    # 例外：HOVER 对在飞任务是“暂停”（可在航点原地恢复），CONTINUE 只解锁停靠门。
    # ==================================================================================

    # 归一后的指令种类：即时指令里触发的降落都属迫降（任务外紧急落地），统一按 LAND 执行
    UAV_COMMAND_KINDS = ('HOVER', 'CONTINUE', 'LAND', 'RTL', 'RTL_ROUTE')

    # 平台可能的多种写法（命令码与 body.command 都先过这张表）
    UAV_COMMAND_ALIASES = {
        'HOVER': 'HOVER', 'HOLD': 'HOVER', 'PAUSE': 'HOVER', 'SUSPEND': 'HOVER', '悬停': 'HOVER',
        'CONTINUE': 'CONTINUE', 'CONTINUE_TASK': 'CONTINUE', 'CONTINUETASK': 'CONTINUE',
        'RESUME': 'CONTINUE', 'RESUME_TASK': 'CONTINUE', 'RESUMETASK': 'CONTINUE',
        '继续任务': 'CONTINUE', '恢复任务': 'CONTINUE', '继续': 'CONTINUE',
        'LAND': 'LAND', 'LANDING': 'LAND', 'DESCEND': 'LAND', '降落': 'LAND', '原地降落': 'LAND',
        '迫降': 'LAND', 'FORCE_LAND': 'LAND', 'EMERGENCY_LAND': 'LAND', 'EMERGENCYLAND': 'LAND',
        'RTL': 'RTL', 'RTL_STRAIGHT': 'RTL', 'RETURN': 'RTL', 'RETURN_HOME': 'RTL',
        '返航': 'RTL', '直线返航': 'RTL',
        'RTL_ROUTE': 'RTL_ROUTE', 'RTLROUTE': 'RTL_ROUTE', 'RTL_BY_WAYPOINTS': 'RTL_ROUTE',
        'RETURN_ROUTE': 'RTL_ROUTE', '沿线返航': 'RTL_ROUTE',
    }

    # 返航终点＝原点；z=0 会让 planner 在 ±0.5m 安全区内自动切 SZD 降落
    RETURN_HOME_POS = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'd': 0.0}

    @staticmethod
    def normalize_uav_command(raw) -> Optional[str]:
        """把平台写法归一成指令种类，不认识返回 None（英文大小写/分隔符都容忍）。"""
        aliases = CruiseUavTaskManager.UAV_COMMAND_ALIASES
        if not isinstance(raw, str) or not raw.strip():
            return None
        key = raw.strip().upper().replace('-', '_').replace(' ', '_')
        key = key.replace('__', '_').strip('_')
        if key in aliases:
            return aliases[key]
        return aliases.get(raw.strip())      # 中文键不受 upper() 影响，原样再查一次

    @staticmethod
    def flip_yaw(rad: float) -> float:
        """机头反向 180 度并归一到 [-pi, pi]（沿线返航回程朝向与来时相反）。"""
        return (rad + math.pi + math.pi) % (2 * math.pi) - math.pi

    def parse_uav_command(self, message_data) -> Tuple[Optional[str], str, Dict[str, Any]]:
        """从报文取即时指令：优先 body.command，其次命令码本身。返回 (kind, actionId, params)。"""
        body = message_data.get('body')
        body = body if isinstance(body, dict) else {}
        raw = body.get('command') or body.get('cmd') or message_data.get('cmd', '')
        # 命令码本身不是具体动作（UAVCOMMAND/COMMANDDELIVER 只是通道），必须给 body.command
        if isinstance(raw, str) and raw.strip().upper() in ('UAVCOMMAND', 'COMMANDDELIVER'):
            raw = ''
        kind = self.normalize_uav_command(raw)
        action_id = str(body.get('actionId') or message_data.get('seq') or f'uavcmd_{int(time.time())}')
        try:
            land_timeout = float(body.get('timeoutSec', body.get('timeout', 60.0)))
        except (TypeError, ValueError):
            land_timeout = 60.0
        try:
            route_max_points = int(body.get('maxPoints', 0) or 0)
        except (TypeError, ValueError):
            route_max_points = 0
        params = {'land_timeout': land_timeout if land_timeout > 0 else 60.0,
                  'route_max_points': max(0, route_max_points)}
        return kind, action_id, params

    def process_uav_command(self, message_data):
        """接收平台即时飞控指令（由 task_dispatcher_node 的命令表分发进来）。"""
        kind, action_id, params = self.parse_uav_command(message_data)
        if kind is None:
            body = message_data.get('body') if isinstance(message_data.get('body'), dict) else {}
            raw = body.get('command') or body.get('cmd') or ''
            if not raw:
                logger.error('[即时指令] 缺少 body.command，可选: %s', '/'.join(self.UAV_COMMAND_KINDS))
                self.send_ack_response(message_data, 400, f'missing body.command ({"/".join(self.UAV_COMMAND_KINDS)})', {})
            else:
                logger.error(f'[即时指令] 无法识别的命令: {raw}')
                self.send_ack_response(message_data, 400, f'unknown command: {raw}', {})
            return

        with self._lock:
            busy = None
            reject = None
            aborted_task = None
            paused_task = None
            if self.instant_cmd is not None:
                busy = self.instant_cmd
            elif kind == 'CONTINUE' and not (self.task_paused and self.is_executing):
                reject = 'no paused task to continue'
            else:
                self.instant_cmd = kind
                in_flight = self.is_executing and self.current_task_id is not None
                if kind == 'HOVER' and in_flight:
                    # 悬停=暂停：不清任务状态，航线线程在当前航点停靠，等 CONTINUE 续航
                    paused_task = self.current_task_id
                    self.task_paused = True
                elif kind == 'CONTINUE':
                    paused_task = self.current_task_id      # 执行体负责真正解开停靠门
                elif in_flight:
                    aborted_task = self.current_task_id
                    self.is_executing = False
                    self.current_task_id = None
                    self.cruises = []
                    self.task_paused = False                # 暂停停靠中则唤醒线程使其退出

        if busy or reject:
            reason = f'another instant command in progress: {busy}' if busy else reject
            logger.warning(f'[即时指令] 拒绝 {kind}: {reason}')
            self.send_ack_response(message_data, 400, reason, {})
            return

        try:
            # 1) 接管：停止当前航点的目标点发布（航线线程会因此走“其他失败”分支停靠/终止）
            #    CONTINUE 例外：它不接管，只放行
            if kind != 'CONTINUE':
                self.perform_action_manager.set_stop_action(True)
            if aborted_task:
                self.ros_topic_subscriber.set_cruises(self.cruises)
                self._report_task_status(aborted_task, TaskStatus.CANCELED.value,
                                         f'平台即时指令 {kind} 接管，任务终止')

            # 2) 停规划：把控制权从 ego_planner 交回飞控（原生 LAND/RTL 与悬停都需要；
            #    CONTINUE 不停规划——续航后线程马上重新发目标点，规划器自然恢复）
            if kind != 'CONTINUE':
                self._publish_stop_planning()

            # 3) 受理回执
            self.send_ack_response(message_data, 200, 'ok',
                                   {'command': kind, 'actionId': action_id,
                                    'abortedTaskId': aborted_task or '',
                                    'pausedTaskId': paused_task or ''})
            logger.info(f'[即时指令] 受理 {kind} actionId={action_id} 中止任务={aborted_task} '
                        f'暂停任务={paused_task}')

            threading.Thread(target=self._execute_uav_command,
                             args=(kind, action_id, params, aborted_task, paused_task),
                             daemon=True).start()
        except Exception as e:
            logger.error(f'[即时指令] 处理 {kind} 异常: {e}')
            with self._lock:
                self.instant_cmd = None
            self.send_ack_response(message_data, 500, f'internal error: {e}', {})

    def _execute_uav_command(self, kind, action_id, params, aborted_task, paused_task=None):
        """即时指令执行体（独立线程，避免占住 MQTT 网络线程导致后续指令收不到）。"""
        status, message = TaskStatus.ABORTED.value, f'{kind} 执行失败'

        # 接管竞态修复：set_stop_action(True) 后必须等旧任务线程真正退出，
        # 再放开 stop 让返航/降落线程发新目标点；否则旧 move 循环（0.2s 轮询 stop）
        # 可能与新目标发布并发，planner 收到交替目标点来回抖动。
        if aborted_task:
            old_thread = self._task_thread
            if old_thread is not None and old_thread.is_alive():
                old_thread.join(timeout=5.0)
                if old_thread.is_alive():
                    logger.error('[即时指令] 旧任务线程 5s 内未退出，继续接管（存在双目标发布风险）')

        try:
            if kind == 'HOVER':
                if paused_task:
                    # 任务暂停：线程已停靠在当前航点，任务状态保留，等 CONTINUE 续航
                    status, message = TaskStatus.COMPLETED.value, \
                        f'HOVER_PAUSED 任务 {paused_task} 已暂停悬停，可发 CONTINUE 续航'
                else:
                    # 无在飞任务：已停规划且不再下发目标点，traj_server 持续发当前位置＝原地悬停
                    status, message = TaskStatus.COMPLETED.value, 'HOVERING 已停止规划，原地悬停'
            elif kind == 'CONTINUE':
                # 续航：先恢复目标点下发能力，再解开停靠门（顺序不能反：线程醒来即重试当前航点）
                self.perform_action_manager.set_stop_action(False)
                with self._lock:
                    self.task_paused = False
                status, message = TaskStatus.COMPLETED.value, f'RESUMED 已继续任务 {paused_task}'
            elif kind == 'LAND':
                self.uav_action_manager.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                logger.info(f'[即时指令] {kind}（迫降语义）已下发 PX4 原生 LAND（落地后由 comms 自动上锁）')
                status, message = self._finish_after_descent(kind, params)
            elif kind == 'RTL':
                status, message = self._return_home_straight(action_id, params)
            elif kind == 'RTL_ROUTE':
                status, message = self._return_home_along_route(action_id, params)
        except Exception as e:
            logger.exception(f'[即时指令] {kind} 执行异常: {e}')
            status, message = TaskStatus.ABORTED.value, f'执行异常: {e}'
        finally:
            self._report_task_status(action_id, status, message)
            with self._lock:
                self.instant_cmd = None
            # 交还“可下发目标点”的能力；不重新起规划，等下一条任务/指令
            self.perform_action_manager.set_stop_action(False)
            logger.info(f'[即时指令] {kind} 结束: progress={status} message={message}')

    def _finish_after_descent(self, kind: str, params) -> Tuple[int, str]:
        """下降/迫降的落地确认：只读 arming 状态，不重复发命令。"""
        if self._wait_landed(params['land_timeout']):
            return TaskStatus.COMPLETED.value, 'LANDED 原地降落完成并已上锁'
        self.uav_action_manager.disarm()
        return TaskStatus.ABORTED.value, f'{kind} 在 {params["land_timeout"]}s 内未确认落地，已补发上锁命令'

    def _return_home_straight(self, action_id, params) -> Tuple[int, str]:
        """直线返航（平台语义）：先给 planner 发 (0,0,1) 自主规划回原点悬停点，
        到达后再发 (0,0,0)，末段由 planner 的 SZD 安全区自动切降落。"""
        self.perform_action_manager.set_stop_action(False)   # 允许下发新目标点，planner 重新起规划
        hover_pos = {'x': 0.0, 'y': 0.0, 'z': 1.0, 'd': 0.0}
        result = self.perform_action_manager.perform_action_walk_to_pos_3d(hover_pos)
        if not result.get('success', False):
            logger.error(f"[直线返航] 去原点悬停点 (0,0,1) 受阻: {result.get('message', '')}")
            self._publish_mission_event(
                'RETURN_BLOCKED',
                goal_seq=self.goal_seq_counter,
                flight_epoch=getattr(self, 'flight_epoch', 'NA'),
                reason='OBSTACLE_BLOCKED')
            self._emergency_land(action_id)
            if self._wait_landed(params['land_timeout']):
                return TaskStatus.ABORTED.value, 'RTL 受阻已迫降并上锁'
            self.uav_action_manager.disarm()
            return TaskStatus.ABORTED.value, 'RTL 受阻已迫降，但未确认落地，已补发上锁命令'
        # 到达 (0,0,1) 后发 (0,0,0)：复用现有返航收尾（SZD 安全区自动降落，含受阻迫降）
        self._return_to_origin(action_id)
        if self._wait_landed(params['land_timeout']):
            return TaskStatus.COMPLETED.value, 'RTL_DONE 已回原点并降落上锁'
        self.uav_action_manager.disarm()
        return TaskStatus.ABORTED.value, f'已回原点但 {params["land_timeout"]}s 内未确认落地，已补发上锁命令'

    def _return_home_along_route(self, action_id, params) -> Tuple[int, str]:
        """沿线返航：不必逐点都过，只回溯“关键点”（航点配置里 pos.key 打钩，
        pcd 规划页导出）。按已到达的关键点倒序逐点退回（机头 yaw 反转 180 度），
        每段仍由 planner 自主避障；单段不可达就跳过退回更近的点；
        航线里没有任何关键点标记时退回“全部已到达航点”逐个倒返（保持旧行为）；
        最后走直线返航回原点完成降落。"""
        self.perform_action_manager.set_stop_action(False)
        with self._lock:
            history = list(self.visited_waypoints)
        if not history:
            logger.warning('[沿线返航] 无已到达航点记录，退化为直线返航')
            return self._return_home_straight(action_id, params)

        key_points = [p for p in history if p.get('key')]
        if key_points:
            back_points = list(reversed(key_points))
            logger.info(f'[沿线返航] 已到达 {len(history)} 点，其中关键点 {len(back_points)} 个，只回溯关键点')
        else:
            back_points = list(reversed(history))
            logger.info(f'[沿线返航] 航线未标记关键点，倒序退回全部 {len(back_points)} 个来路航点')
        if params['route_max_points'] > 0:
            back_points = back_points[:params['route_max_points']]
        flown = skipped = 0
        for pos in back_points:
            back = {'x': pos['x'], 'y': pos['y'], 'z': pos['z'], 'd': self.flip_yaw(pos.get('d', 0.0))}
            result = self.perform_action_manager.perform_action_walk_to_pos_3d(back)
            if result.get('success', False):
                flown += 1
                continue
            msg = result.get('message', '')
            if msg.startswith('GOAL_UNREACHABLE'):
                logger.warning(f'[沿线返航] 该航点不可达，跳过继续退回更近的点: {back}')
                skipped += 1
                continue
            logger.error(f'[沿线返航] 退回过程被中止: {msg}')
            break
        logger.info(f'[沿线返航] 回溯航点 {len(back_points)} 个: 实退={flown} 跳过={skipped}')
        return self._return_home_straight(action_id, params)

    def _publish_stop_planning(self):
        """ego_planner 控制入口：通知 traj_server 停止发送轨迹并清掉目标点/SZD 状态。"""
        try:
            self.stop_planning_pub.publish(Empty())
            logger.info('[即时指令] 已发布 /stop_planning，planner 停止轨迹发送')
        except Exception as e:
            logger.error(f'[即时指令] 发布 /stop_planning 失败: {e}')

    def _stop_ego_planner_service(self):
        """停止导航服务（幂等）：点云落盘后调用一次，任务收尾再调一次不会重复执行。"""
        if self._ego_planner_stopped:
            logger.info('ego_planner.service 已停止，跳过重复停止')
            return
        self._ego_planner_stopped = True
        logger.info('停止ego_planner.service服务...')
        subprocess.Popen(['sudo', 'systemctl', 'stop', 'ego_planner.service'])
        logger.info('停止ego_planner.service服务操作完成')

    def _wait_landed(self, timeout: float) -> bool:
        """等 PX4 落定上锁（arming_state=DISARMED）。上锁由 comms 的落地检测流程负责，
        这里只做确认，超时返回 False 由调用方补发。"""
        deadline = time.time() + max(5.0, float(timeout))
        while time.time() < deadline and rclpy.ok():
            try:
                _, arming = self.ros_topic_subscriber.get_takeoff_time_and_arming_state()
            except Exception as e:
                logger.warning(f'读取解锁状态失败: {e}')
                arming = None
            if arming == VehicleStatus.ARMING_STATE_DISARMED:
                logger.info('[即时指令] 已确认落地并上锁')
                return True
            time.sleep(0.5)
        return False

    def _remember_visited_waypoint(self, pos: Dict[str, Any]) -> None:
        """记录成功到达的航点（供沿线返航倒序退回），只保留必要字段并限制长度；
        pos.key（规划页打钩的关键点）一并记录，沿线返航优先只走关键点。"""
        try:
            item = {'x': float(pos['x']), 'y': float(pos['y']), 'z': float(pos['z']),
                    'd': float(pos.get('d', 0.0) or 0.0),
                    'key': bool(pos.get('key', False))}
        except (KeyError, TypeError, ValueError):
            return
        with self._lock:
            self.visited_waypoints.append(item)
            if len(self.visited_waypoints) > 200:
                self.visited_waypoints = self.visited_waypoints[-200:]

    def process_cruise_task(self, message_data):
        """
        处理巡检任务相关命令
        支持 SETCRUISETASK、GETCRUISETASK、EXECCRUISETASK、STOPCRUISETASK
        """
        try:
            # 提取必要信息
            cmd = message_data.get('cmd', '').upper()
            
            # 根据命令类型调用不同的处理方法
            if cmd == 'SETCRUISETASK':
                self._process_set_cruise_task(message_data)
            elif cmd == 'GETCRUISETASK':
                self._process_get_cruise_task(message_data)
            elif cmd == 'EXECCRUISETASK':
                self._process_exec_cruise_task(message_data)
            elif cmd == 'STOPCRUISETASK':
                self._process_stop_cruise_task(message_data)
            else:
                logger.warning(f'未知的巡检任务命令: {cmd}')
                self.send_ack_response(message_data, 400, f'unknown command: {cmd}')
                
        except Exception as e:
            logger.error(f'处理巡检任务命令时出错: {str(e)}')
            self.send_ack_response(message_data, 500, str(e))
    
    def _process_set_cruise_task(self, message_data):
        """
        处理SETCRUISETASK命令，设置巡检任务
        """
        try:
            body = message_data.get('body', {})
            
            # 验证必要字段
            required_fields = ['taskName', 'taskId', 'type', 'mapName', 'cruises', 'mapFileId', 'mapFileUri', 'savePointCloud']
            for field in required_fields:
                if field not in body:
                    logger.error(f'SETCRUISETASK消息缺少必要字段: {field}')
                    self.send_ack_response(message_data, 400, f'missing {field}')
                    return
            
            # 验证cruises格式
            if not isinstance(body['cruises'], list):
                logger.error('cruises字段必须是列表格式')
                self.send_ack_response(message_data, 400, 'cruises must be list')
                return
            
            # 验证每个路径点的必要字段
            for cruise_point in body['cruises']:
                if 'idx' not in cruise_point or 'pos' not in cruise_point:
                    logger.error('巡检路径点缺少必要字段: idx或pos')
                    self.send_ack_response(message_data, 400, 'cruise point missing idx or pos')
                    return

            task_id = body['taskId']
            # 验证任务ID
            if not task_id:
                logger.error('ExecCruiseTask消息缺少taskId')
                self.send_ack_response(message_data, 400, 'missing taskId')
                return
            
            # 检查是否已经在执行任务
            with self._lock:
                if self.is_executing:
                    logger.warning('已有巡检任务在执行中')
                    self.send_ack_response(message_data, 409, 'task already executing')
                    return
            
            # 存储任务
            self.tasks[task_id] = body
            
            logger.info(f'成功设置巡检任务: {task_id}, 名称: {body["taskName"]}')

            # 检查是否启用
            if "enable" in body and body["enable"]:
                # 设置执行状态
                with self._lock:
                    self.is_executing = True
                    self.current_task_id = task_id
                # 下发巡检任务到机器人（放入线程中执行，避免阻塞MQTT处理）
                self._task_thread = threading.Thread(target=self._execute_task_thread, args=(task_id, body), daemon=True)
                self._task_thread.start()

            # 发送成功响应
            self.send_ack_response(message_data, 200, 'ok')
            
        except Exception as e:
            logger.error(f'设置巡检任务失败: {str(e)}')
            self.send_ack_response(message_data, 500, str(e))
    
    def _process_get_cruise_task(self, message_data):
        """
        处理GetCruiseTask命令，获取巡检任务
        """
        try:
            body = message_data.get('body', {})
            task_id = body.get('taskId')
            
            # 准备响应数据
            response_body = {}
            
            if task_id:
                # 获取指定任务
                if task_id in self.tasks:
                    response_body['tasks'] = [self.tasks[task_id]]
                else:
                    logger.warning(f'未找到指定的巡检任务: {task_id}')
                    response_body['tasks'] = []
            else:
                # 获取所有任务
                response_body['tasks'] = list(self.tasks.values())
            
            logger.info(f'获取巡检任务，taskId: {task_id}, 找到任务数: {len(response_body["tasks"])}')
            
            # 发送响应，包含任务数据
            self.send_ack_response(message_data, 200, 'ok', response_body)
            
        except Exception as e:
            logger.error(f'获取巡检任务失败: {str(e)}')
            self.send_ack_response(message_data, 500, str(e))
    
    def _process_exec_cruise_task(self, message_data):
        """
        处理ExecCruiseTask命令，启动巡检任务（异步）
        """
        try:
            body = message_data.get('body', {})
            task_id = body.get('taskId')
            
            # 验证任务ID
            if not task_id:
                logger.error('ExecCruiseTask消息缺少taskId')
                self.send_ack_response(message_data, 400, 'missing taskId')
                return
            
            # 检查任务是否存在
            if task_id not in self.tasks:
                logger.error(f'未找到指定的巡检任务: {task_id}')
                self.send_ack_response(message_data, 404, f'task not found: {task_id}')
                return
            
            # 检查是否已经在执行任务
            with self._lock:
                if self.is_executing:
                    logger.warning('已有巡检任务在执行中')
                    self.send_ack_response(message_data, 409, 'task already executing')
                    return
                
                # 设置执行状态
                self.is_executing = True
                self.current_task_id = task_id

            # 下发巡检任务到机器人（放入线程中执行，避免阻塞MQTT处理）
            self._task_thread = threading.Thread(target=self._execute_task_thread, args=(task_id, self.tasks[task_id]), daemon=True)
            self._task_thread.start()
            
            # 发送确认响应
            self.send_ack_response(message_data, 200, 'ok')

            logger.info(f'开始执行巡检任务: {task_id}')
            
        except Exception as e:
            logger.error(f'启动巡检任务失败: {str(e)}')
            self.send_ack_response(message_data, 500, str(e))
    
    def _process_stop_cruise_task(self, message_data):
        """
        处理StopCruiseTask命令，停止巡检任务
        """
        try:
            body = message_data.get('body', {})
            task_id = body.get('taskId')

            # 标记为未执行
            with self._lock:
                is_executing = self.is_executing
                self.is_executing = False
                current_task = self.current_task_id
                self.current_task_id = None
                self.cruises = []
                self.task_paused = False    # 暂停停靠中的线程据此唤醒并退出
            
            self.ros_topic_subscriber.set_cruises(self.cruises)
            self._report_task_status(task_id, TaskStatus.CANCELED.value, '巡检任务取消')
            self.perform_action_manager.set_stop_action(True)
            
            if task_id and str(task_id) != str(current_task):
                logger.warning(f'指定的任务ID与当前执行的任务不匹配: {task_id} vs {current_task}')
                self.send_ack_response(message_data, 400, 'taskId mismatch')
                return
            
            # 如果没有正在执行的任务
            if not is_executing:
                logger.warning('没有正在执行的巡检任务')
                self.send_ack_response(message_data, 404, 'no task executing')
                return
            
            logger.info(f'巡检任务已停止: {task_id}')
            
            # 发送成功响应
            self.send_ack_response(message_data, 200, 'ok')
            
        except Exception as e:
            logger.error(f'停止巡检任务失败: {str(e)}')
            self.send_ack_response(message_data, 500, str(e))

    
    def _report_task_status(self, task_id, taskProgress, error_msg='', file_id='', file_type=''):
        """
        上报任务执行状态
        
        Args:
            task_id: 任务ID
            status: 状态 (start, executing, completed, stopped, error)
            taskProgress: task执行进度：0-启动、1-进行中、2-完成、3-中断
            error_msg: 错误信息（如果有）
            file_id: 文件ID（如果有）
            file_type: 文件类型（如果有）
        """
        try:
            # 如果task_id包含"buildmap"，直接返回
            if 'buildmap' in str(task_id):
                logger.info(f'task_id包含buildmap，跳过状态上报: taskId={task_id}')
                return
            
            # 构建状态上报消息
            status_msg = {
                'sn': self.robot_sn,  # 这里可以从配置或输入消息中获取
                'ver': self.version,
                'seq': str(uuid.uuid4()).replace('-', ''),  # 简单生成序列号
                'type': 1,
                'ts': int(time.time() * 1000),
                'cmd': 'ActionReport',
                #'status': 200 if status != 'error' else 500,
                #'error': error_msg if error_msg else 'ok',
                'body': {
                    'taskId': task_id,
                    'pos': self.ros_topic_subscriber.get_position(),
                    'message': error_msg if error_msg else 'ok',
                    #'status': status,
                    'taskProgress': taskProgress,
                    'tsReport': int(time.time() * 1000),
                    'fileId': file_id,
                    'fileType': file_type,
                }
            }
            
            # 将消息添加到缓存
            message_cache_manager.add_message(status_msg['seq'], status_msg)
            logger.debug(f'已将任务状态消息加入缓存: seq={status_msg["seq"]}')
            
            # 发布状态消息
            self.mqtt_client.publish(self.mqtt_response_topic, json.dumps(status_msg))
            logger.debug(f'已上报任务状态: taskId={task_id}, taskProgress={taskProgress}, message={error_msg}')
            
        except Exception as e:
            logger.error(f'上报任务状态失败: {str(e)}')
    
    def send_ack_response(self, original_message, status, error_msg, body=None):
        """
        发送Ack响应到MQTT
        
        Args:
            original_message: 原始消息
            status: 状态码
            error_msg: 错误消息
            body: 响应体（可选）
        """
        try:
            response = {
                'sn': original_message.get('sn', ''),
                'ver': original_message.get('ver', '1.0'),
                'seq': original_message.get('seq', 0),
                'type': 2,  # 响应类型
                'ts': int(time.time() * 1000),
                'cmd': 'Ack',
                'status': status,
                'error': error_msg
            }
            
            # 如果有响应体，添加到响应中
            if body:
                response['body'] = body
            
            # 发布到响应主题
            self.mqtt_client.publish(self.mqtt_response_topic, json.dumps(response))
            logger.info(f'已发送Ack响应: status={status}, error={error_msg}')
            
        except Exception as e:
            logger.error(f'发送Ack响应失败: {str(e)}')
    def on_video_uploaded(self, task_id, local_path, server_file_id):
        """
        当单个视频文件上传成功后，自动执行此函数
        """
        self._report_task_status(
            task_id, 
            TaskStatus.IN_PROGRESS.value, 
            f'执行巡检任务task_id={task_id}, 完成巡检录像上传file_id:{server_file_id}',
            server_file_id,
            '.mp4')
        
    def _processing_cruise_task(self, task_id, task_data):
        """
        处理巡航任务
        
        Args:
            task_id: 任务ID
            task_data: 任务数据
        """
        starting_algo_ids = []
        try:
            # self._report_task_status(task_id, TaskStatus.STARTED.value, '任务已开始')
            
            # 等待服务
            time.sleep(15)
                
            # 标记为正在执行（已经在线程外部设置）
            with self._lock:
                self.cruises = [{'taskId': task_id, 'status': True}]
                # 新任务开始：来路航点重新记录（供平台“沿线返航”倒序退回）
                self.visited_waypoints = []
                # 仅当接的是别的任务才清暂停标志：唤醒可能还停在 HOVER 停靠门上的旧线程，
                # 其会因 task_id 不符自行退出；本任务自己的暂停（受理先于线程启动）不能碰
                if self.current_task_id != task_id:
                    self.task_paused = False
            self.ros_topic_subscriber.set_cruises(self.cruises)
            self.perform_action_manager.set_stop_action(False)

            # # 检查地图是否存在，如果不存在则进行更新
            # map_file_id = task_data['mapFileId']
            # map_file_uri = task_data['mapFileUri']
            
            # rmap = self.map_manager.update_pcd_map(map_file_id, map_file_uri)
            # logger.info(f"更新pcd地图完成: {rmap}")

            # 第二步：执行巡检任务
            # 开始巡检录像
            # self.rtst_record_manager.start_recording(task_id, save_record=True, upload_callback=self.on_video_uploaded)
            
            if ENABLE_MANUAL_CONTROL:
                logger.info(f"等待无人机解锁并起飞...")
                while self.is_executing:
                    takeoff_time, arming_state = self.ros_topic_subscriber.get_takeoff_time_and_arming_state()
                    logger.debug(f"当前无人机解锁状态: {arming_state}, 起飞时间: {takeoff_time}")
                    # 无人机已经解锁并起飞
                    if takeoff_time > 0 and arming_state == 2:
                        break
                    time.sleep(1)

                logger.info(f"等待无人机降落...")
                while self.is_executing:
                    takeoff_time, arming_state = self.ros_topic_subscriber.get_takeoff_time_and_arming_state()
                    logger.debug(f"当前无人机解锁状态: {arming_state}, 起飞时间: {takeoff_time}")
                    # 无人机已经降落
                    if takeoff_time == 0 and arming_state == 1:
                        break
                    time.sleep(1)
                    
                logger.info(f"无人机已经降落，任务已完成")
            else:    
                cruises = task_data.get('cruises', [])            
                # 记录启动中的算法
                
                action_idx = 0
                for cruise in cruises:
                    pos = cruise.get('pos')
                    ing_actions = cruise.get('ingActions', [])
                    end_actions = cruise.get('endActions', [])
                    cruise_idx = cruise.get('idx', 0)

                    logger.debug(f"ing_actions: {ing_actions}, end_actions:{end_actions}")
                    
                    # 1、处理ingActions
                    # 是否需要停止任务, 最新的动作索引, 启动的算法列表
                    should_stop, action_idx, _ = self._process_ing_actions(task_id, ing_actions, starting_algo_ids, action_idx)
                    if should_stop:
                        self.rtst_record_manager.stop_recording()
                        return

                    # 2、处理move动作（含航点失效判定：同点重试≥3跳点 / 连续 WAYPOINT_CONSEC_LIMIT 点不可达返航 / 返航遇障迫降）
                    with self._lock:
                        self.goal_seq_counter += 1
                    skipped_current = False
                    while True:  # 同一航点重试循环
                        result_action = self.perform_action_manager.perform_action_walk_to_pos_3d(pos)
                        if result_action.get('success', False):
                            # 到达成功：重置同点重试与连续跨点失败累计
                            with self._lock:
                                self.same_point_retry = 0
                                self.consec_goals = 0
                            # 记入来路（平台可下发“沿线返航”按此倒序退回）
                            self._remember_visited_waypoint(pos)
                            break
                        # 成功=False：区分「停滞不可达(GOAL_UNREACHABLE)」与「其他失败」
                        msg = result_action.get('message', '')
                        if msg.startswith('GOAL_UNREACHABLE'):
                            should_end = self._handle_waypoint_unreachable(task_id, pos)
                            if should_end:  # 已触发返航/迫降，任务终止
                                self._report_task_status(task_id, TaskStatus.ABORTED.value, '航点不可达，已转入返航/迫降')
                                self.rtst_record_manager.stop_recording()
                                return
                            # False：同点未达3次继续重试；或已跳过本航点(consec<2 继续下一航点)
                            with self._lock:
                                retry_now = self.same_point_retry
                                consec_now = self.consec_goals
                            if retry_now == 0 and consec_now > 0:
                                # 已跳过本航点且未达连续上限 → 跳过该段巡检动作，进入下一航点
                                skipped_current = True
                                break
                            # 仍在同点重试计数中 → 继续 while 重试同一航点
                            continue
                        else:
                            # 即时指令下发的停止：任务被 HOVER 暂停 → 停靠在当前航点等续航；
                            # 任务被其它指令接管/停止 → 收尾已由其完成，这里静默退出
                            if self.task_paused:
                                logger.info(f'任务{task_id}已在第{cruise_idx}个航点暂停，等待平台 CONTINUE 续航')
                                while self.task_paused and self.is_executing and rclpy.ok():
                                    time.sleep(0.5)
                                if (not self.is_executing) or self.current_task_id != task_id:
                                    logger.info(f'暂停任务{task_id}已被终止或替换，航线执行结束')
                                    return
                                logger.info(f'收到续航指令，任务{task_id}继续第{cruise_idx}个航点')
                                continue
                            if not self.is_executing or self.current_task_id != task_id:
                                logger.info(f'任务{task_id}已被平台指令中止或替换，退出航线执行')
                                return
                            # 其他失败（非不可达）→ 按原逻辑中止任务
                            logger.error(f'移动动作执行失败，已停止: {msg}')
                            self._report_task_status(task_id, TaskStatus.ABORTED.value, '移动动作执行失败，已停止')
                            self.rtst_record_manager.stop_recording()
                            return

                    action_idx += 1
                    if skipped_current:
                        # 本航点不可达已被跳过：不执行该点 endActions 巡检动作（飞机不在该点）
                        self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{cruise_idx}段航点不可达，已跳过巡检动作')
                        logger.warning(f"执行巡检任务task_id={task_id}, 【第{cruise_idx}段】航点不可达已跳过，不执行该段巡检动作")
                        continue

                    self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{action_idx}个动作执行完成，移动到目标位置')
                    logger.info(f"执行巡检任务task_id={task_id}, 【第{action_idx}个动作】执行完成, 移动到目标位置")
                                    
                    # 3、处理endActions中的动作
                    # 是否需要停止任务, 最新的动作索引, 启动的算法列表
                    should_stop, action_idx, _ = self._process_end_actions(task_id, end_actions, starting_algo_ids, action_idx)
                    if should_stop:
                        self.rtst_record_manager.stop_recording()
                        return
                    
                    self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 完成【第{cruise_idx}段巡检】')
                    logger.info(f"执行巡检任务task_id={task_id}, 完成【第{cruise_idx}段巡检】")
                
                # 发送降落指令
                if ENABLE_DOCK_CONTROL:
                    self.dock_control_client.land_alarm()
                # self.uav_command_sender.send_command('LAND')                                               
                self.uav_action_manager.exec_land_command()
            
                # 第三步：巡检任务完成，关闭所有未关闭的算法
                for algo_id in starting_algo_ids:
                    result_action = self.perform_action_manager.perform_action_algo(algo_id, False)
                    if not result_action.get('success', False):
                        logger.error(f'巡检结束，关闭算法 {algo_id} 失败')
                        self._report_task_status(task_id, TaskStatus.ABORTED.value, f'巡检结束，关闭算法 {algo_id} 失败')
                        return
            
            # 巡检任务完成,停止录像
            # self.rtst_record_manager.stop_recording()
            # 处理点云数据（开始上传时已并行触发「闭合推杆 → 打开充电」，此处不再等待上传完成）
            file_id = self.process_point_cloud_data(task_data.get('savePointCloud', False))
            # 未触发点云上传时（如 savePointCloud=False 或点云保存失败），补触发一次闭合推杆/充电
            # self._start_dock_close_charge()
            if ENABLE_DOCK_CONTROL:               
                try:
                    dock_status, dock_msg = self.dock_control_client.land_sequence()
                except Exception as e:
                    logger.error(f'降落序列执行异常: {str(e)}')
                    dock_status, dock_msg = False, str(e)
                        
            logger.info(f"执行巡检任务task_id={task_id}, 任务已完成")
            self._report_task_status(task_id, TaskStatus.COMPLETED.value, '任务已完成', file_id, 'pcd')
            
        except Exception as e:
            logger.error(f'下发巡检任务到无人机时出错: {str(e)}')
            # 上报任务失败状态
            self._report_task_status(task_id, TaskStatus.ABORTED.value, str(e))

        finally:
            # 确保重置状态
            self.reset_status()

            # 关闭所有未关闭的算法
            for algo_id in starting_algo_ids:
                result_action = self.perform_action_manager.perform_action_algo(algo_id, False)
                if not result_action.get('success', False):
                    logger.error(f'巡检结束，关闭算法 {algo_id} 失败')
                    self._report_task_status(task_id, TaskStatus.ABORTED.value, f'巡检结束，关闭算法 {algo_id} 失败')

    def _execute_task_thread(self, task_id, body):
        """
        在独立线程中执行巡检任务
        
        Args:
            task_id: 任务ID
            body: 任务数据
        """
        try:
            if ROBOT_TYPE == "default":
                self._processing_cruise_task(task_id, body)
            elif ROBOT_TYPE == "uav":
                logger.info(f"开始执行无人机巡检任务task_id={task_id}")
                self._report_task_status(task_id, TaskStatus.STARTED.value, '任务已开始')
                #切换为位置模式
                self.uav_action_manager.engage_position_mode()
                
                if ENABLE_DOCK_CONTROL:
                    # 这里进行无人机起飞前检查
                    try:
                        device_data = self.dock_control_client._get_device_data()
                        logger.info(f"气象数据:{device_data}")
                        
                        rainfall = device_data.get('rainfall', 0)
                        if rainfall > RAINFALL_THRESHOLD:
                            msg = f"雨量过大，禁止起飞. 雨量:{rainfall}"
                            logger.warning(msg)
                            self._report_task_status(task_id, TaskStatus.ABORTED.value, msg)
                            self.reset_status()
                            return
                        
                        wind_speed = device_data.get('wind_speed', 0.0)
                        if wind_speed > WIND_SPEED_THRESHOLD:
                            msg = f"风速过大，禁止起飞. 风速:{wind_speed}"
                            logger.warning(msg)
                            self._report_task_status(task_id, TaskStatus.ABORTED.value, msg)
                            self.reset_status()
                            return
                        
                        dock_status, dock_msg = self.dock_control_client.takeoff_sequence()
                        if not dock_status:
                            self._report_task_status(task_id, TaskStatus.ABORTED.value, dock_msg)
                            self.reset_status()
                            return

                    except Exception as e:
                        logger.error(f'起飞序列执行异常: {str(e)}')
                        # 机库动作异常必须中止任务，避免推杆/舱盖未就绪就起飞
                        self._report_task_status(task_id, TaskStatus.ABORTED.value, f'起飞序列执行异常: {str(e)}')
                        self.reset_status()
                        return

                # 起飞前状态检测
                if ENABLE_PREFLIGHT_CHECK:
                    # if self.node.get_uav_fault_inspection() == False:
                    #     return
                    
                    check_code, msg = self.preflight_check_node._start_check()
                    if check_code == 1:
                        logger.warning(msg)
                        self._report_task_status(task_id, TaskStatus.ABORTED.value, msg)
                        self.reset_status()
                        return
                    
                # 检查地图是否存在，如果不存在则进行更新
                map_file_id = body['mapFileId']
                map_file_uri = body['mapFileUri']
                
                rmap = self.map_manager.update_pcd_map(map_file_id, map_file_uri)
                logger.info(f"更新pcd地图完成: {rmap}")
                
                logger.info('启动ego_planner.service服务...')
                subprocess.run(['sudo', 'systemctl', 'start', 'ego_planner.service'], timeout=120)
                self._ego_planner_stopped = False
                logger.info('启动ego_planner.service服务操作完成')
                
                # 执行巡检任务    
                self._processing_cruise_task(task_id, body)
                
                
                # 关闭导航服务（兜底：点云已保存时 process_point_cloud_data 中已提前关闭，
                # 这里覆盖 savePointCloud=False / 点云保存失败等未提前关闭的情况）
                self._stop_ego_planner_service()
                
            else:
                logger.info(f"机器人类型为{ROBOT_TYPE},不执行巡检任务task_id={task_id}")
        except Exception as e:
            logger.error(f'线程执行巡检任务时出错: {str(e)}')
            # 上报任务失败状态
            self._report_task_status(task_id, TaskStatus.ABORTED.value, str(e))
            self.reset_status()

    def reset_status(self):
        """
        重置任务执行状态

        """
        # 标记为未执行
        with self._lock:
            self.is_executing = False
            self.current_task_id = None
            self.cruises = []
        # 下一次任务重新触发降落后推杆/充电流程
        with self._dock_seq_lock:
            self._dock_seq_triggered = False
        self._post_landing_phase = False
        self.ros_topic_subscriber.set_cruises(self.cruises)
        self.perform_action_manager.set_stop_action(False)

    def _start_dock_close_charge(self):
        """
        异步执行降落后的「闭合推杆 → 打开充电」，与点云上传并行，不阻塞上传;
        每个任务只触发一次。
        """
        if not ENABLE_DOCK_CONTROL:
            return
        with self._dock_seq_lock:
            if self._dock_seq_triggered:
                return
            self._dock_seq_triggered = True
            self._dock_seq_thread = threading.Thread(
                target=self._dock_close_charge_worker, daemon=True,
                name='dock_close_charge')
            self._dock_seq_thread.start()

    def _dock_close_charge_worker(self):
        try:
            success, msg = self.dock_control_client.land_charge_sequence()
            if success:
                logger.info(f'降落后闭合推杆并充电: {msg}')
            else:
                logger.error(f'降落后闭合推杆并充电失败: {msg}，等待充电巡检兜底')
        except Exception as e:
            logger.error(f'降落后闭合推杆并充电异常: {str(e)}')

    def _dock_charge_watchdog_loop(self):
        """每 3 分钟巡检一次：补发一次打开充电"""
        while True:
            time.sleep(180)
            try:
                self._check_dock_charge()
            except Exception as e:
                logger.error(f'充电状态巡检异常: {str(e)}')

    def _check_dock_charge(self):
        # 任务执行中（含起飞、飞行、降落）不做干预；已降落进入点云上传阶段例外
        if self.is_executing and not self._post_landing_phase:
            logger.info('充电巡检: 任务执行中，跳过')
            return
        # 降落后推杆/充电流程仍在执行，避免重复下发
        if self._dock_seq_thread is not None and self._dock_seq_thread.is_alive():
            logger.info('充电巡检: 推杆/充电流程执行中，跳过')
            return
        logger.info('充电巡检: 发送一次打开充电')
        success, msg = self.dock_control_client.open_charge_once()
        if not success:
            logger.error(f'充电巡检: 打开充电失败: {msg}')
    
    def _handle_action_failure(self, task_id, on_failed_config=None, default_action='skip'):
        """
        统一处理动作失败的逻辑
        
        Args:
            task_id: 任务ID
            on_failed_config: 失败处理配置
            default_action: 默认失败处理动作
            
        Returns:
            bool: 是否需要停止整个任务
        """
        on_failed = on_failed_config.get('actName', default_action) if on_failed_config else default_action
        pos_back = on_failed_config.get('pos') if on_failed_config else None
        logger.error(f'动作执行失败，失败处理配置: {on_failed_config}')
        if on_failed == 'skip':
            self._report_task_status(task_id, TaskStatus.ABORTED.value, '动作执行失败，继续下一个操作')
            return False  # 跳过当前操作，不停止任务
        elif on_failed == 'stop':
            self.perform_action_manager.perform_action_stop()
            self._report_task_status(task_id, TaskStatus.ABORTED.value, '动作执行失败，已停止')
            return True  # 需要停止整个任务
        elif on_failed == 'back':
            if pos_back:
                self.perform_action_manager.perform_action_back(pos_back)
            self._report_task_status(task_id, TaskStatus.ABORTED.value, '动作执行失败，已停止')
            return True  # 需要停止整个任务
        return False
    
    def _manage_algorithm(self, task_id, algo_id, start, starting_algo_ids, on_failed_config=None):
        """
        统一管理算法的启动和关闭
        
        Args:
            task_id: 任务ID
            algo_id: 算法ID
            start: True表示启动，False表示关闭
            starting_algo_ids: 当前启动中的算法列表
            on_failed_config: 失败处理配置
            
        Returns:
            bool: 是否需要停止整个任务
        """
        result_action = self.perform_action_manager.perform_action_algo(algo_id, start)
        if not result_action.get('success', False):
            return self._handle_action_failure(task_id, on_failed_config)
            
        if start:
            starting_algo_ids.append(algo_id)
            logger.debug(f"算法 {algo_id} 已成功启动, 当前启动中的算法列表: {starting_algo_ids}")
        else:
            starting_algo_ids.remove(algo_id)
            logger.debug(f"算法 {algo_id} 已成功关闭, 当前启动中的算法列表: {starting_algo_ids}")
            
        return False
    
    def _process_ing_actions(self, task_id, ing_actions, starting_algo_ids, action_idx):
        """
        处理ingActions中的动作
        
        Args:
            task_id: 任务ID
            ing_actions: ingActions列表
            starting_algo_ids: 当前启动中的算法列表
            action_idx: 当前动作索引
            
        Returns:
            tuple: (是否需要停止任务, 最新的动作索引, 启动的算法列表)
        """
        algo_ids = []
        
        for ing_act in ing_actions:
            if ing_act.get('actName') == 'algo':
                algo_ids = ing_act.get('argv', {}).get('algoId', [])
                on_failed_config = ing_act.get('onFailed', {})
                
                for algo_id in algo_ids:
                    if algo_id not in starting_algo_ids:
                        # 启动算法
                        if self._manage_algorithm(task_id, algo_id, True, starting_algo_ids, on_failed_config):
                            return True, action_idx, algo_ids
                
                action_idx += 1
                self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{action_idx}个动作执行完成, 启动算法{algo_ids}')
                logger.info(f"执行巡检任务task_id={task_id}, 【第{action_idx}个动作】执行完成, 启动算法{algo_ids}")
        
        # 关闭已开启的算法
        algos_to_stop = set(starting_algo_ids) - set(algo_ids)
        for algo_id in algos_to_stop:
                # 关闭算法
                if self._manage_algorithm(task_id, algo_id, False, starting_algo_ids):
                    return True, action_idx, algo_ids
        
        if algos_to_stop:
            action_idx += 1
            self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{action_idx}个动作执行完成，关闭算法{algos_to_stop}')
            logger.info(f"执行巡检任务task_id={task_id}, 【第{action_idx}个动作】执行完成, 关闭算法{algos_to_stop}")
            
        return False, action_idx, algo_ids
    
    def _process_end_actions(self, task_id, end_actions, starting_algo_ids, action_idx):
        """
        处理endActions中的动作
        
        Args:
            task_id: 任务ID
            end_actions: endActions列表
            starting_algo_ids: 当前启动中的算法列表
            action_idx: 当前动作索引
            
        Returns:
            tuple: (是否需要停止任务, 最新的动作索引, 启动的算法列表)
        """
        algo_ids = []
        
        for end_act in end_actions:
            act_name = end_act.get('actName')
            on_failed_config = end_act.get('onFailed', {})
            
            if act_name == 'algo':
                algo_ids = end_act.get('argv', {}).get('algoId', [])
                
                for algo_id in algo_ids:
                    if algo_id not in starting_algo_ids:
                        # 启动算法
                        if self._manage_algorithm(task_id, algo_id, True, starting_algo_ids, on_failed_config):
                            return True, action_idx, algo_ids
                
                action_idx += 1
                self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{action_idx}个动作执行完成，启动算法{algo_ids}')
                logger.info(f"执行巡检任务task_id={task_id}, 【第{action_idx}个动作】执行完成, 启动算法{algo_ids}")
                
            elif act_name == 'stay':
                argv = end_act.get('argv', {})
                result_action = self.perform_action_manager.perform_action_stay(argv)
                if not result_action.get('success', False):
                    if self._handle_action_failure(task_id, on_failed_config):
                        return True, action_idx, algo_ids
                        
                action_idx += 1
                self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{action_idx}个动作执行完成，停留')
                logger.info(f"执行巡检任务task_id={task_id}, 【第{action_idx}个动作】执行完成, 停留")
                
            elif act_name == 'ptz':
                argv = end_act.get('argv', {})
                result_action = self.perform_action_manager.perform_action_ptz(argv)
                if not result_action.get('success', False):
                    if self._handle_action_failure(task_id, on_failed_config):
                        return True, action_idx, algo_ids
                        
                action_idx += 1
                self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{action_idx}个动作执行完成，PTZ控制')
                logger.info(f"执行巡检任务task_id={task_id}, 【第{action_idx}个动作】执行完成, PTZ控制")
            elif act_name == 'gpio':
                argv = end_act.get('argv', {})
                result_action = self.perform_action_manager.perform_action_gpio(argv)
                # if not result_action.get('success', False):
                #     if self._handle_action_failure(task_id, on_failed_config):
                #         return True, action_idx, algo_ids
                        
                action_idx += 1
                self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{action_idx}个动作执行完成，GPIO控制')
                logger.info(f"执行巡检任务task_id={task_id}, 【第{action_idx}个动作】执行完成, GPIO控制")  
            elif act_name == 'snap':
                argv = end_act.get('argv', {})
                file_id = self.rtst_record_manager.capture_rtsp_snapshot()
                # if file_id == '':
                #     if self._handle_action_failure(task_id, on_failed_config):
                #         return True, action_idx, algo_ids
                # result_action = self.perform_action_manager.perform_action_rtspsnap(argv)
                # if not result_action.get('success', False):
                #     if self._handle_action_failure(task_id, on_failed_config):
                #         return True, action_idx, algo_ids
                        
                action_idx += 1
                self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{action_idx}个动作执行完成，截图控制', file_id, 'jpg')
                logger.info(f"执行巡检任务task_id={task_id}, 【第{action_idx}个动作】执行完成, 截图控制")     
            else:
                # 未知动作
                logger.error(f'未知动作 {act_name} 执行失败')
                self._report_task_status(task_id, TaskStatus.ABORTED.value, f'未知动作 {act_name} 执行失败')
                continue  # 跳过当前动作，继续处理下一个
        
        # 关闭已开启的算法
        algos_to_stop = set(starting_algo_ids) - set(algo_ids)
        for algo_id in algos_to_stop:
                # 关闭算法
                if self._manage_algorithm(task_id, algo_id, False, starting_algo_ids):
                    return True, action_idx, algo_ids
        
        if algos_to_stop:
            action_idx += 1
            self._report_task_status(task_id, TaskStatus.IN_PROGRESS.value, f'执行巡检任务task_id={task_id}, 第{action_idx}个动作执行完成，关闭算法{algos_to_stop}')
            logger.info(f"执行巡检任务task_id={task_id}, 【第{action_idx}个动作】执行完成, 关闭算法{algos_to_stop}")
            
        return False, action_idx, algo_ids


    def process_point_cloud_data(self, save_point_cloud):
        '''
        处理点云数据，根据save_point_cloud参数判断是否上传到文件服务器
        
        Args:
            save_point_cloud: 是否保存点云数据
            
        Returns:
            str or None: 上传成功返回文件ID，失败返回None
        '''

        if not save_point_cloud:
            return ''

        try:
            logger.info("开始处理点云数据保存和上传")
            
            # 1. 等待点云保存服务可用（复用 __init__ 中创建的客户端）
            if not self.map_save_client.wait_for_service(timeout_sec=5.0):
                logger.error("点云保存服务 /map_save 不可用")
                return ''
            
            # 3. 创建请求并调用服务
            request = Trigger.Request()
            future = self.map_save_client.call_async(request)
            
            # 4. 等待服务调用结果（添加超时机制防止死循环）
            service_timeout = 60.0  # 服务调用超时时间（秒）
            #rclpy.spin_until_future_complete(self.node, future, timeout_sec=service_timeout)
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < service_timeout:
                time.sleep(0.1)

            #检查是否超时
            if not future.done():
                logger.error(f"点云保存服务调用超时（{service_timeout}秒）")
                return ''
                   
            # 5. 检查服务调用结果
            if future.result():
                if future.result().success:
                    logger.info("点云保存成功")
                else:
                    logger.error(f"点云保存失败: {future.result().message}")
                    return ''
            else:
                logger.error("点云保存服务调用失败")
                return ''
            
            # 6. 检查点云文件是否存在
            pcd_file_path = PCD_FILE_PATH
            if not os.path.exists(pcd_file_path):
                logger.error(f"点云文件不存在: {pcd_file_path}")
                return ''

            # 点云已落盘，导航链路不再需要，立即关闭 ego_planner，不必等后续持久化/上传完成
            self._stop_ego_planner_service()
            
            # 点云降采样
            pcd = o3d.io.read_point_cloud(pcd_file_path)
            origin_points = len(pcd.points)
            origin_size = os.path.getsize(pcd_file_path)
            logger.info(f"点云降采样前: 点数={origin_points}, 文件大小={origin_size / 1024 / 1024:.2f}MB ({origin_size}字节)")
            down_pcd = pcd.voxel_down_sample(0.05)
            o3d.io.write_point_cloud(pcd_file_path, down_pcd)
            down_points = len(down_pcd.points)
            down_size = os.path.getsize(pcd_file_path)
            logger.info(f"点云降采样后: 点数={down_points}, 文件大小={down_size / 1024 / 1024:.2f}MB ({down_size}字节)")
            
            # 持久化点云文件和记录
            task_id = self.current_task_id if hasattr(self, 'current_task_id') and self.current_task_id else 'unknown_task'
            success, record = self.point_cloud_persistor.persist_point_cloud(pcd_file_path, task_id)
            if success and record.get('id'):
                logger.info(f"点云文件已持久化，记录ID: {record['id']}")
                # 立即更新记录状态为上传中，避免定时器重复处理
                self.point_cloud_persistor._update_record_status(record['id'], 'uploading')
                logger.info(f"点云记录状态已更新为上传中，记录ID: {record['id']}")
            else:
                logger.error(f"点云文件持久化失败")
            
            # 7. 开始上传点云的同时闭合推杆，推杆闭合后发送一次充电（与上传并行，不阻塞上传）
            self._post_landing_phase = True
            self._start_dock_close_charge()
            # 上传点云文件到文件服务器（带重试机制）
            file_id = self.upload_point_cloud_with_retry(pcd_file_path)
            if file_id:
                logger.info(f"点云文件上传成功，文件ID: {file_id}")
                # 更新记录状态为已上传
                self.point_cloud_persistor._update_record_status(record.get('id'), 'uploaded')
                # 删除本地文件
                os.remove(record.get('file_path'))
                logger.info(f'已删除本地点云文件: {record.get("file_path")}')
                # 删除记录
                self.point_cloud_persistor._remove_record(record.get('id'))
                logger.info(f'已删除点云记录: {record.get("id")}')
                return file_id
            else:
                logger.error("点云文件上传失败，已达到最大重试次数")  
                # 上传失败，将记录状态更新回pending，以便定时器在网络恢复后重新尝试
                self.point_cloud_persistor._update_record_status(record.get('id'), 'pending')
                logger.info(f"点云记录状态已更新为待上传，记录ID: {record.get('id')}")
                return ''
                
        except Exception as e:
            logger.error(f"处理点云数据时出错: {str(e)}")
            return ''

    
    def _check_network_connection(self):
        """
        检查网络连接状态
        
        Returns:
            bool: 网络连接正常返回True，否则返回False
        """
        try:
            # 尝试连接到文件服务器
            requests.get(self.server_url, timeout=5, verify=False)
            return True
        except requests.ConnectionError:
            return False
        except Exception as e:
            logger.warning(f"网络连接检查异常: {str(e)}")
            return False
    
    def upload_point_cloud_to_file_server(self, pcd_file_path, is_retry=False, task_id=None):
        """
        直接上传点云文件到文件服务器，需要先获取token
        
        Args:
            pcd_file_path: 点云文件路径    
            is_retry: 是否为重试上传，默认False
            task_id: 任务ID，用于重试上传时报告任务状态
            
        Returns:
            str: 文件ID，失败返回空字符串
        """
        try:
            # 获取上传token
            token = self.auth_manager.get_upload_token()
            
            if not token or token == '':
                logger.error('Failed to get upload token')
                return ''
            
            # 构建带token的上传URL
            upload_url = f'{self.server_url}/file/file/upload/{token}'
            
            # 准备上传数据，使用时间戳确保文件名唯一性
            timestamp = int(time.time() * 1000)
            unique_filename = f'point_cloud_{timestamp}.pcd'
            
            # 直接打开文件进行上传
            with open(pcd_file_path, 'rb') as f:
                files = {
                    'file': (unique_filename, f, 'application/octet-stream')
                }
                logger.info(f'准备上传点云到文件服务器: {upload_url}, 文件名={unique_filename}, 文件路径={pcd_file_path}')
                # 发送请求到文件服务器，添加verify参数控制SSL证书验证
                response = requests.post(upload_url, files=files, data={}, timeout=300, verify=False)  # 生产环境建议设置为True并提供证书
            
            if response.status_code == 200:
                result = response.json()
                # 从data对象中获取id字段
                if result.get('data') and 'id' in result['data']:
                    if is_retry:
                        logger.info(f'重试上传点云成功，文件ID: {result["data"]["id"]}')
                        self._report_task_status(task_id, TaskStatus.COMPLETED.value, '任务已完成', result['data']['id'], 'pcd')
                    return result['data']['id']
                else:
                    logger.warning(f'响应中未找到预期的data.id字段: {result}')
                    return ''
            else:
                logger.error(f'点云上传失败: 状态码={response.status_code}, 响应={response.text}')
                return ''
                
        except requests.ConnectionError as e:
            logger.error(f'点云上传网络连接错误，请检查网络状态: {str(e)}')
            return ''
        except requests.Timeout as e:
            logger.error(f'点云上传请求超时: {str(e)}')
            return ''
        except Exception as e:
            logger.error(f'点云上传异常: {str(e)}')
            return ''
    
    def upload_point_cloud_chunk_to_file_server(self, pcd_file_path, is_retry=False, task_id=None):
        """
        使用分片上传方式上传点云文件到文件服务器

        Args:
            pcd_file_path: 点云文件路径
            is_retry: 是否为重试上传，默认False
            task_id: 任务ID，用于重试上传时报告任务状态

        Returns:
            str: 文件ID，失败返回空字符串
        """
        try:
            logger.info(f'准备使用分片上传点云到文件服务器: 文件路径={pcd_file_path}')
            file_id = self.file_chunk_uploader.upload_file(pcd_file_path)

            if file_id:
                if is_retry:
                    logger.info(f'重试上传点云成功，文件ID: {file_id}')
                    self._report_task_status(task_id, TaskStatus.COMPLETED.value, '任务已完成', file_id, 'pcd')
                return file_id
            else:
                logger.error('点云分片上传失败，未获取到文件ID')
                return ''

        except Exception as e:
            logger.error(f'点云分片上传异常: {str(e)}')
            return ''
    
    def upload_point_cloud_with_retry(self, pcd_file_path):
        """
        带重试机制的点云文件上传函数
        检查网络状态，在网络恢复后重新上传，直到上传成功
        
        Args:
            pcd_file_path: 点云文件路径
            
        Returns:
            str: 文件ID，成功返回文件ID，失败返回空字符串
        """
        retry_interval = 10  # 初始重试间隔（秒）
        max_retry_interval = 60  # 最大重试间隔（秒）
        retry_count = 0
        max_retries = 2  # 最大重试次数，防止无限重试
        
        while rclpy.ok() and retry_count < max_retries:
            # 检查点云文件是否仍然存在
            if not os.path.exists(pcd_file_path):
                logger.error(f"点云文件不存在: {pcd_file_path}")
                return ''
            
            # 检查网络连接
            if not self._check_network_connection():
                logger.warning(f"网络连接不可用，将在{retry_interval}秒后重试点云上传")
                time.sleep(retry_interval)
                # 指数退避策略
                retry_interval = min(retry_interval * 2, max_retry_interval)
                retry_count += 1
                continue
            
            # 尝试上传点云文件
            file_id = self.upload_point_cloud_chunk_to_file_server(pcd_file_path)
            if file_id:
                return file_id
            
            # 上传失败，等待后重试
            logger.warning(f"点云上传失败，将在{retry_interval}秒后重试")
            time.sleep(retry_interval)
            # 指数退避策略
            retry_interval = min(retry_interval * 2, max_retry_interval)
            retry_count += 1
        
        logger.error(f"点云上传已达到最大重试次数（{max_retries}次），请检查网络和服务器状态")
        return ''
    
    def shutdown(self):
        """
        关闭巡检任务管理器，清理资源
        """
        # 停止定时任务
        if hasattr(self, 'point_cloud_persistor'):
            self.point_cloud_persistor.stop_timer()
        logger.info("巡检任务管理器已关闭")
