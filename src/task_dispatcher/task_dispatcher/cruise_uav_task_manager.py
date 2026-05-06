#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import logging
import time
import threading
import os
import base64
import requests
import rclpy
import asyncio
import subprocess
from typing import Dict, List, Optional, Any
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
from .config import ROBOT_TYPE, PCD_FILE_PATH, ENABLE_MANUAL_CONTROL
from .config import ENABLE_PREFLIGHT_CHECK, ENABLE_DOCK_CONTROL, WIND_SPEED_THRESHOLD, RAINFALL_THRESHOLD  # 引入配置文件中的URL和点云文件路径
from uav_command_sender import UavCommandSender
from dock_control_client import DockControlClient
from task_dispatcher.preflight_check_node import PreFlightCheckNode
from task_dispatcher.uav_action_manager import UavActionManager
from auth_utils import AuthManager 

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
        
        logger.info('巡检任务管理器初始化完成')
    
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
                threading.Thread(target=self._execute_task_thread, args=(task_id, body), daemon=True).start()

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
            threading.Thread(target=self._execute_task_thread, args=(task_id, self.tasks[task_id]), daemon=True).start()
            
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
            if 'buildmap' in task_id:
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
            logger.info(f'已将任务状态消息加入缓存: seq={status_msg["seq"]}')
            
            # 发布状态消息
            self.mqtt_client.publish(self.mqtt_response_topic, json.dumps(status_msg))
            logger.info(f'已上报任务状态: taskId={task_id}, taskProgress={taskProgress}, message={error_msg}')
            
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
                    logger.info(f"当前无人机解锁状态: {arming_state}, 起飞时间: {takeoff_time}")
                    # 无人机已经解锁并起飞
                    if takeoff_time > 0 and arming_state == 2:
                        break
                    time.sleep(1)

                logger.info(f"等待无人机降落...")
                while self.is_executing:
                    takeoff_time, arming_state = self.ros_topic_subscriber.get_takeoff_time_and_arming_state()
                    logger.info(f"当前无人机解锁状态: {arming_state}, 起飞时间: {takeoff_time}")
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

                    logger.info(f"ing_actions: {ing_actions}, end_actions:{end_actions}")
                    
                    # 1、处理ingActions
                    # 是否需要停止任务, 最新的动作索引, 启动的算法列表
                    should_stop, action_idx, _ = self._process_ing_actions(task_id, ing_actions, starting_algo_ids, action_idx)
                    if should_stop:
                        self.rtst_record_manager.stop_recording()
                        return

                    # 2、处理move动作
                    result_action = self.perform_action_manager.perform_action_walk_to_pos_3d(pos)
                    if not result_action.get('success', False):
                        logger.error(f'移动动作执行失败，已停止')
                        self._report_task_status(task_id, TaskStatus.ABORTED.value, '移动动作执行失败，已停止')
                        self.rtst_record_manager.stop_recording()
                        return

                    action_idx += 1
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
            # 处理点云数据
            file_id = self.process_point_cloud_data(task_data.get('savePointCloud', False))
            
            time.sleep(1)
            
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
                            return
                        
                        wind_speed = device_data.get('wind_speed', 0.0)
                        if wind_speed > WIND_SPEED_THRESHOLD:
                            msg = f"风速过大，禁止起飞. 风速:{wind_speed}"
                            logger.warning(msg)
                            self._report_task_status(task_id, TaskStatus.ABORTED.value, msg)
                            return
                        
                        dock_status, dock_msg = self.dock_control_client.takeoff_sequence()
                        if not dock_status:
                            self._report_task_status(task_id, TaskStatus.ABORTED.value, dock_msg)
                            return

                    except Exception as e:
                        logger.error(f'起飞序列执行异常: {str(e)}')
                        dock_status, dock_msg = False, str(e)

                # 起飞前状态检测
                if ENABLE_PREFLIGHT_CHECK:
                    # if self.node.get_uav_fault_inspection() == False:
                    #     return
                    
                    check_code, msg = self.preflight_check_node._start_check()
                    if check_code == 1:
                        logger.warning(msg)
                        self._report_task_status(task_id, TaskStatus.ABORTED.value, msg)
                        return
                    
                # 检查地图是否存在，如果不存在则进行更新
                map_file_id = body['mapFileId']
                map_file_uri = body['mapFileUri']
                
                rmap = self.map_manager.update_pcd_map(map_file_id, map_file_uri)
                logger.info(f"更新pcd地图完成: {rmap}")
                
                logger.info('启动ego_planner.service服务...')
                subprocess.run(['sudo', 'systemctl', 'start', 'ego_planner.service'], timeout=120)
                logger.info('启动ego_planner.service服务操作完成')
                
                # 执行巡检任务    
                self._processing_cruise_task(task_id, body)
                
                if ENABLE_DOCK_CONTROL:
                    # 任务完成
                    try:
                        dock_status, dock_msg = self.dock_control_client.land_sequence()
                        
                    except Exception as e:
                        logger.error(f'降落序列执行异常: {str(e)}')
                        dock_status, dock_msg = False, str(e)
                        
                # 关闭导航服务
                logger.info('停止ego_planner.service服务...')
                subprocess.Popen(['sudo', 'systemctl', 'stop', 'ego_planner.service'])
                logger.info('停止ego_planner.service服务操作完成')
                
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
        self.ros_topic_subscriber.set_cruises(self.cruises)
        self.perform_action_manager.set_stop_action(False)
    
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
            logger.info(f"算法 {algo_id} 已成功启动, 当前启动中的算法列表: {starting_algo_ids}")
        else:
            starting_algo_ids.remove(algo_id)
            logger.info(f"算法 {algo_id} 已成功关闭, 当前启动中的算法列表: {starting_algo_ids}")
            
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
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=service_timeout)
            
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
            
            # 7. 上传点云文件到文件服务器（带重试机制）
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
        max_retries = 1  # 最大重试次数，防止无限重试
        
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
            file_id = self.upload_point_cloud_to_file_server(pcd_file_path)
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
