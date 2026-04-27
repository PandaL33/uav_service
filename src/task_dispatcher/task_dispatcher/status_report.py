#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态变更上报模块
负责检测机器人状态变化并通过MQTT推送状态报告
"""

import json
import time
import threading
from typing import Dict, Any, Optional
import logging
import paho.mqtt.client as mqtt
from task_dispatcher.message_cache import message_cache_manager
import uuid


logger = logging.getLogger('task_dispatcher')

class StatusReport:
    """
    状态变更上报类
    检测机器人状态变化并通过MQTT推送状态报告
    """
    
    def __init__(self, mqtt_client: mqtt.Client, robot_sn: str = "", 
                 min_interval: float = 1.0, max_interval: float = 60.0,
                 mqtt_response_topic: str = "", version: str = ""):
        """
        初始化状态上报类
        
        Args:
            mqtt_client: MQTT客户端实例
            robot_sn: 机器人序列号
            min_interval: 最小上报间隔（秒），默认1秒
            max_interval: 最大上报间隔（秒），默认60秒
            mqtt_response_topic: MQTT响应话题，默认""
            version: 版本号，默认"1.0"
        """
        self.mqtt_client = mqtt_client
        self.robot_sn = robot_sn
        self.mqtt_response_topic = mqtt_response_topic
        self.version = version

        self.sequence = 0
        self.min_interval = max(0.1, min_interval)  # 确保最小间隔大于0
        self.max_interval = max(self.min_interval + 1, max_interval)  # 确保最大间隔大于最小间隔
        
        # 上次上报时间和状态
        self._last_report_time = 0
        self._last_status = {
            'status': { 'soc': 0.0,'charging':False, 'pos': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'd': 0} },
            'algos': [],
            'cruises': [],
            'cameras': [],
            'services': []
        }
        
        self._lock = threading.RLock()
        logger.info(f'StatusReport initialized with min_interval={min_interval}s, max_interval={max_interval}s')
    
    def _is_status_changed(self, current_status: Dict[str, Any]) -> bool:
        """
        检查状态是否发生变化
        
        Args:
            current_status: 当前状态
            
        Returns:
            是否发生变化
        """
        # 深度比较状态字典
        def deep_compare(d1, d2):
            if type(d1) != type(d2):
                return False
            
            if isinstance(d1, dict):
                if set(d1.keys()) != set(d2.keys()):
                    return False
                for k in d1:
                    if not deep_compare(d1[k], d2[k]):
                        return False
                return True
            elif isinstance(d1, (int, float)):
                # 对于数值类型，使用近似比较
                if isinstance(d1, float) and isinstance(d2, float):
                    return abs(d1 - d2) < 0.001  # 浮点数精度
                return d1 == d2
            else:
                return d1 == d2
        
        return not deep_compare(current_status, self._last_status)
    
    def _should_report(self, status_changed: bool) -> bool:
        """
        决定是否应该上报状态
        
        Args:
            status_changed: 状态是否发生变化
            
        Returns:
            是否应该上报
        """
        current_time = time.time()
        time_since_last = current_time - self._last_report_time
        
        # 如果状态变化且距离上次上报超过最小间隔，则上报
        if status_changed and time_since_last >= self.min_interval:
            return True
        
        # 如果距离上次上报超过最大间隔，无论状态是否变化都上报
        if time_since_last >= self.max_interval:
            return True
        
        return False
    
    def _create_status_report(self, robot_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        根据机器人状态创建状态报告
        
        Args:
            robot_state: 机器人状态
            
        Returns:
            格式化的状态报告
        """
        # 构建状态报告
        report = {
            "sn": self.robot_sn,
            "ver": self.version,
            "seq": self._get_next_sequence(),
            "type": 1,
            "ts": int(time.time() * 1000),  # 毫秒时间戳
            "cmd": "StatusReport",
            "body": {
                "status": {
                    "soc": float(robot_state.get('battery_level', 0.0)),
                    "charging": robot_state.get('battery_status', 0) == 1,
                    "pos": {
                        "x": float(robot_state.get('position', {}).get('x', 0.0)),
                        "y": float(robot_state.get('position', {}).get('y', 0.0)),
                        "z": float(robot_state.get('position', {}).get('z', 0.0)),
                        "d": float(robot_state.get('position', {}).get('d', 0.0))  # 默认方向，可根据需要从orientation计算
                    }
                },
                "algos": [],
                "cruises": robot_state.get('cruises', []),
                "cameras": robot_state.get('camera_status', []),
                'services': robot_state.get('services', [])
            }
        }
        
        return report
    
    def _get_next_sequence(self) -> int:
        """
        获取下一个序列号
        
        Returns:
            序列号
        """
        seq = str(uuid.uuid4()).replace('-', '')
        return seq
    
    def update_status(self, robot_state: Dict[str, Any]):
        """
        更新状态并在需要时上报
        
        Args:
            robot_state: 最新的机器人状态
        """
        try:
            # 创建状态报告
            current_status = self._create_status_report(robot_state)
            body = current_status['body']
            
            # 检查状态是否变化
            with self._lock:
                # 检查状态是否变化, 这里先假设状态变化
                #status_changed = self._is_status_changed(body)
                status_changed = True
                should_report = self._should_report(status_changed)
                
                # 如果需要上报，则推送MQTT消息
                if should_report:
                    # 序列化状态报告
                    report_json = json.dumps(current_status)
                    #logger.info(f'准备上报状态: {report_json}')
                    
                    # 将消息添加到缓存
                    message_cache_manager.add_message(str(current_status['seq']), current_status)
                    #logger.info(f'已将状态报告消息加入缓存: seq={current_status["seq"]}')
                    
                    # 发布到MQTT
                    result = self.mqtt_client.publish(
                        topic=self.mqtt_response_topic,
                        payload=report_json,
                        qos=1,
                        retain=False
                    )
                    
                    # 更新最后上报时间和状态
                    self._last_report_time = time.time()
                    self._last_status = body.copy()
                    
                    if result.rc == mqtt.MQTT_ERR_SUCCESS:
                        logger.info(f'Status report published: seq={current_status["seq"]}')
                    else:
                        logger.error(f'Failed to publish status report: {result.rc}')
                
        except Exception as e:
            logger.error(f'Error in status update: {e}')