#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人注册保活模块
负责设备注册和心跳保活功能
"""

import rclpy
from rclpy.node import Node
import json
import time
import logging
from threading import Thread, Event
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
from task_dispatcher.message_cache import message_cache_manager
from typing import Optional
import uuid
import hashlib
from task_dispatcher.service_manager import SERVICE_CONFIG

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(name)s] [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger('registration')

class Registration:
    """
    设备注册保活类，负责设备注册和心跳保活功能
    """
    
    def __init__(self, mqtt_client, response_topic, node=None, version='1.0'):
        """
        初始化设备注册保活模块
        
        Args:
            mqtt_client: MQTT客户端实例
            response_topic: MQTT响应主题
            node: ROS2节点实例（可选）
            version: 版本号
        """
        self.mqtt_client = mqtt_client
        self.response_topic = response_topic
        self.node = node
        self.version = version
        
        # 注册相关状态
        self.registration_status = False
        self.last_registration_time = 0
        self.registration_retry_count = 0
        self.max_registration_retries = 5
        
        # 心跳相关状态
        self.keepalive_interval = 60000  # 默认60秒
        self.last_keepalive_time = 0
        self.keepalive_response_count = 0
        self.max_keepalive_no_response = 3
        
        # 配置信息
        self.config = {
            'subscribe': [],
            'uploadUrl': None,
            'ntp': None,
            'sn': '',  # 默认SN
            'username': 'default_user',
            'password': 'default_password',
            'model': 'D1Ultra',
            'softwareVer': self.version,
            'hardwareVer': self.version
        }
        
        # 设备信息
        self.device_info = {
            'cameras': [],
            'radars': [],
            'algos': []
        }
        
        # 运行控制
        self._running = False
        self._stop_event = Event()
        self._keepalive_thread = None
        
        # 序列号生成器
        self.seq_counter = 0
        
        # 状态数据提供者
        self.status_provider: Optional[Ros2TopicSubscriber] = None
    
    def start(self):
        """
        启动注册保活服务
        """
        if self._running:
            logger.warning('注册保活服务已经在运行')
            return
        
        self._running = True
        self._stop_event.clear()
        
        # 启动注册
        # 等待10秒，确保MQTT连接已建立
        time.sleep(2)
        self.register()
        time.sleep(5)
        
        # 启动心跳线程
        self._keepalive_thread = Thread(target=self._keepalive_loop, daemon=True)
        self._keepalive_thread.start()
        
        logger.info('注册保活服务已启动')
    
    def stop(self):
        """
        停止注册保活服务
        """
        if not self._running:
            logger.warning('注册保活服务未运行')
            return
        
        self._running = False
        self._stop_event.set()
        
        if self._keepalive_thread:
            self._keepalive_thread.join(timeout=5.0)
        
        logger.info('注册保活服务已停止')
    
    def register(self):
        """
        执行设备注册
        """
        try:
            # 生成时间戳
            ts = int(time.time() * 1000)
            
            # 使用MD5(sn+username+password+str(ts))生成授权码
            sn = self.config.get('sn', '')
            username = self.config.get('username', '')
            password = self.config.get('password', '')
            
            # 构建MD5输入字符串
            md5_input = f"{sn}{username}{password}{ts}"
            
            # 计算MD5哈希值
            md5_hash = hashlib.md5(md5_input.encode('utf-8')).hexdigest()
            authorization = md5_hash
            
            # 构建注册请求，与示例格式完全匹配
            register_request = {
                'sn': self.config['sn'],
                'ver': self.version,
                'seq': self._get_next_seq(),
                'type': 1,
                'ts': ts,
                'cmd': 'Register',
                'body': {
                    'username': self.config['username'],
                    'authorization': authorization,
                    'extensions': {}
                },
                'extensions': {
                    'model': self.config['model'],
                    'softwareVer': '',  # 空字符串，按照示例
                    'hardwareVer': '',  # 空字符串，按照示例
                    'cameras': [
                        {'cameraId': 'camera01', 'cameraName': '头部摄像头'},
                        {'cameraId': 'camera02', 'cameraName': '机顶摄像头'}
                    ],
                    'radars': [],
                    'algos': [
                        {'algoId': 'trash_detect', 'algoName': '垃圾检测算法'},
                        {'algoId': 'device_check', 'algoName': '设备归位算法'},
                        {'algoId': 'channel_monitor', 'algoName': '通道监测算法'},
                        {'algoId': 'line_integrity', 'algoName': '线路完整性算法'}
                    ],
                    'services': [
                        {'serviceId': service_id, 'name': service_name}
                        for service_id, service_name in SERVICE_CONFIG.items()
                    ]
                }
            }
            
            logger.info(f'发送注册请求: {json.dumps(register_request)}, self.response_topic: {self.response_topic}')
            
            # 将注册请求添加到缓存
            message_cache_manager.add_message(register_request['seq'], register_request)
            logger.info(f'已将注册请求加入缓存: seq={register_request["seq"]}')
            
            # 发布注册请求
            self.mqtt_client.publish(self.response_topic, json.dumps(register_request))
            
            # 更新最后注册时间
            self.last_registration_time = time.time()
            self.registration_retry_count += 1
            
        except Exception as e:
            logger.error(f'注册失败: {str(e)}')
    
    def handle_register_response(self, response_data):
        """
        处理注册响应
        
        Args:
            response_data: 注册响应数据
        """
        try:
            # 检查响应状态
            if response_data.get('status') == "200":
                body = response_data.get('body', {})
                
                # 更新配置
                if 'subscribe' in body:
                    self.config['subscribe'] = body['subscribe']
                
                if 'cfg' in body:
                    cfg = body['cfg']
                    if 'uploadUrl' in cfg:
                        self.config['uploadUrl'] = cfg['uploadUrl']
                    if 'ntp' in cfg:
                        self.config['ntp'] = cfg['ntp']
                    if 'keepaliveInteval' in cfg:
                        self.keepalive_interval = cfg['keepaliveInteval']
                
                # 更新注册状态
                self.registration_status = True
                self.registration_retry_count = 0
                logger.info(f'注册成功，心跳间隔设置为: {self.keepalive_interval}ms')
                
                # 同步时间
                if self.config['ntp']:
                    self._sync_time_with_ntp()
                
            else:
                error_msg = response_data.get('error', 'Unknown error')
                logger.error(f'注册失败: {error_msg}')
                self.registration_status = False
                
                # 如果注册失败次数过多，等待一段时间后重试
                # if self.registration_retry_count >= self.max_registration_retries:
                #     logger.info(f'注册失败次数过多，将在30秒后重试')
                #     time.sleep(30)
                #     self.registration_retry_count = 0
        
        except Exception as e:
            logger.error(f'处理注册响应时出错: {str(e)}')
    
    def send_keepalive(self, status_data=None):
        """
        发送心跳保活包
        
        Args:
            status_data: 可选的状态数据
        """
        try:
            # 构建心跳请求，使用大写命令
            keepalive_request = {
                'sn': self.config['sn'],
                'ver': self.version,
                'seq': self._get_next_seq(),
                'type': 1,
                'ts': int(time.time() * 1000),
                'cmd': 'KeepAlive', 
                'body': {}
            }
            
            # 获取订阅配置
            subscribe_config = self.config.get('subscribe', [])
            body = keepalive_request['body']
            
            logger.info(f'发送心跳保活包，订阅配置: {subscribe_config}')
            # 根据订阅配置添加相应的字段
            if 'status' in subscribe_config:
                body['status'] = {
                    'pos': {'d': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'soc': 0.0,
                    'charging': False
                }
            
            if 'algo' in subscribe_config:
                body['algos'] = [
                    {'algoId': 'trash_detect', 'status': 0},
                    {'algoId': 'device_check', 'status': 0},
                    {'algoId': 'channel_monitor', 'status': 0},
                    {'algoId': 'line_integrity', 'status': 0}
                ]
            
            if 'camera' in subscribe_config:
                body['cameras'] = [
                    {'cameraId': 'camera01', 'status': 1},
                    {'cameraId': 'camera02', 'status': 1}
                ]
            
            if 'cruise' in subscribe_config:
                body['cruises'] = [{'taskId': ''}]  # 默认空任务ID
            
            if 'service' in subscribe_config:
                body['services'] = []  # 默认空服务列表
            
            # 添加radars字段（如果有任何订阅）
            if subscribe_config:
                body['radars'] = []
            
            # 优先使用从状态提供者获取的数据
            if self.status_provider:
                try:
                    body = keepalive_request['body']
                    subscribe_config = self.config.get('subscribe', [])
                    
                    # 更新算法状态
                    if 'algo' in subscribe_config:
                        algo_status = self.status_provider.get_algo_status()
                        if algo_status:
                            body['algos'] = [
                                {'algoId': 'trash_detect', 'status': algo_status[0]},
                                {'algoId': 'device_check', 'status': algo_status[1]},
                                {'algoId': 'channel_monitor', 'status': algo_status[2]},
                                {'algoId': 'line_integrity', 'status': algo_status[3]}
                            ]
                    
                    # 更新摄像头状态
                    if 'camera' in subscribe_config:
                        try:
                            body['cameras'] = self.status_provider.get_camera_status()
                        except Exception as e:
                            logger.debug(f'获取摄像头状态失败: {str(e)}')
                    
                    # 更新位置和电池信息
                    if 'status' in subscribe_config:
                        position = self.status_provider.get_position()
                        if position:
                            body['status']['pos'] = {
                                'd': position.get('d', 0.0),
                                'x': position.get('x', 0.0),
                                'y': position.get('y', 0.0),
                                'z': position.get('z', 0.0)
                            }
                        
                        battery_info = self.status_provider.get_battery_info()
                        if battery_info:
                            body['status']['soc'] = battery_info.get('battery_level', 0.0)
                            body['status']['charging'] = battery_info.get('battery_status', 0) == 1
                    
                    # 更新巡航状态
                    if 'cruise' in subscribe_config:
                        try:
                            body['cruises'] = self.status_provider.get_cruises()
                        except Exception as e:
                            logger.debug(f'获取巡航状态失败: {str(e)}')
                    
                    # 更新服务状态
                    if 'service' in subscribe_config:
                        try:
                            body['services'] = self.status_provider.get_services_status()
                        except Exception as e:
                            logger.debug(f'获取服务状态失败: {str(e)}')
                    
                    logger.debug('已从状态提供者更新心跳包数据')
                except Exception as e:
                    logger.error(f'从状态提供者获取数据失败: {str(e)}')
            
            # 如果提供了状态数据，可以覆盖之前的值
            # if status_data:
            #     body = keepalive_request['body']
            #     # 更新算法状态
            #     if 'algos' in status_data:
            #         body['algos'] = status_data['algos']
            #     # 更新摄像头状态
            #     if 'cameras' in status_data:
            #         body['cameras'] = status_data['cameras']
            #     # 更新机器人状态
            #     if 'status' in status_data:
            #         body['status'] = status_data['status']
            
            logger.info(f'发送心跳包: {json.dumps(keepalive_request)}')
            
            # 将心跳请求添加到缓存
            message_cache_manager.add_message(keepalive_request['seq'], keepalive_request)
            logger.info(f'已将心跳请求加入缓存: seq={keepalive_request["seq"]}')
            
            # 发布心跳请求
            self.mqtt_client.publish(self.response_topic, json.dumps(keepalive_request))
            
            # 更新最后心跳时间
            self.last_keepalive_time = time.time()
            
        except Exception as e:
            logger.error(f'发送心跳失败: {str(e)}')
    
    def handle_keepalive_response(self, response_data):
        """
        处理心跳响应
        
        Args:
            response_data: 心跳响应数据
        """
        try:
            # 检查响应状态
            if response_data.get('status') == "200":
                logger.info('心跳响应成功')
                self.keepalive_response_count = 0
            else:
                error_msg = response_data.get('error', 'Unknown error')
                logger.warning(f'心跳响应失败: {error_msg}')
                # 心跳响应失败也视为失败，保留计数器状态
                logger.debug(f'心跳响应失败，当前失败计数: {self.keepalive_response_count}')
                # 如果失败次数达到阈值，标记设备需要重新注册
                if self.keepalive_response_count >= self.max_keepalive_no_response:
                    logger.warning(f'连续{self.max_keepalive_no_response}次心跳失败，设备下线，需要重新注册')
                    self.registration_status = False
        
        except Exception as e:
            logger.error(f'处理心跳响应时出错: {str(e)}')
    
    def update_device_info(self, device_info):
        """
        更新设备信息
        
        Args:
            device_info: 设备信息字典
        """
        if 'cameras' in device_info:
            self.device_info['cameras'] = device_info['cameras']
        if 'radars' in device_info:
            self.device_info['radars'] = device_info['radars']
        if 'algos' in device_info:
            self.device_info['algos'] = device_info['algos']
        
        # 如果已注册，重新注册以更新设备信息
        if self.registration_status:
            self.register()
    
    def set_config(self, config):
        """
        设置配置信息
        
        Args:
            config: 配置字典
        """
        for key, value in config.items():
            if key in self.config:
                self.config[key] = value
    
    def set_status_provider(self, provider: Ros2TopicSubscriber):
        """
        设置状态数据提供者
        
        Args:
            provider: 提供状态数据的对象，应具有get_robot_status方法
        """
        self.status_provider = provider
        logger.info('已设置状态数据提供者')
    
    def _keepalive_loop(self):
        """
        心跳保活循环
        """
        while self._running and not self._stop_event.is_set():
            # 检查是否需要重新注册
            if not self.registration_status:
                time.sleep(60)  # 注册失败后等待60秒再重试，符合用户要求的每分钟注册一次
                self.register()
                logger.info('注册失败，将在60秒后再次尝试')
                continue
            
            # 检查是否需要发送心跳
            current_time = time.time()
            keepalive_interval_seconds = self.keepalive_interval / 1000.0
            
            if (current_time - self.last_keepalive_time) >= keepalive_interval_seconds:
                self.send_keepalive()
                
                # 增加无响应计数
                self.keepalive_response_count += 1
                logger.debug(f'发送心跳包，当前无响应计数: {self.keepalive_response_count}')
                
                # 检查是否需要重新注册
                if self.keepalive_response_count >= self.max_keepalive_no_response:
                    logger.warning(f'连续{self.max_keepalive_no_response}次心跳未收到响应，设备下线，需要重新注册')
                    self.registration_status = False
            
            # 等待1秒后再次检查
            self._stop_event.wait(1)
    
    def _get_next_seq(self):
        """
        获取下一个序列号
        """
        seq = str(uuid.uuid4()).replace('-', '')
        return seq
    
    def _sync_time_with_ntp(self):
        """
        与NTP服务器同步时间
        注意：实际实现可能需要使用ntplib或系统命令
        """
        try:
            logger.info(f'尝试与NTP服务器同步时间: {self.config["ntp"]}')
            # 这里可以添加实际的NTP同步代码
            # 由于涉及系统时间修改，可能需要特殊权限
            # 此处仅作为示例
            
        except Exception as e:
            logger.error(f'NTP时间同步失败: {str(e)}')
    
    def is_registered(self):
        """
        检查设备是否已注册
        
        Returns:
            bool: 注册状态
        """
        return self.registration_status