#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
状态变更上报模块
负责检测机器人状态变化并通过MQTT推送状态报告
"""

import json
import time
import threading
from typing import Dict, Any, Optional, List
import logging
import paho.mqtt.client as mqtt
from task_dispatcher.message_cache import message_cache_manager
import uuid


logger = logging.getLogger('task_dispatcher')

class DeviceStatusReport:
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
        self._cached_status_map: Dict[str, Dict[str, Any]] = {}
        self._last_fault_report_time = 0
        self._last_status = {}
        self._last_fault_status = {}
        
        # 初始化一些默认值，防止第一次运行时为空
        self._init_default_status()
        
        self._lock = threading.RLock()
        logger.info(f'DeviceStatusReport initialized with min_interval={min_interval}s, max_interval={max_interval}s')
    
    def _init_default_status(self):
        """初始化默认状态模板，确保第一次运行也有基础数据"""
        defaults = [
            ("altitude_relative", "相对高度", 0.0, "m"),
            ("velocityHorizontal", "水平速度", 0.0, "m/s"),
            ("velocityVertical", "垂直速度", 0.0, "m/s"),
            ("latitude", "纬度", 0.0, "°"),
            ("longitude", "经度", 0.0, "°"),
            ("roll_deg", "横滚角", 0.0, "°"),
            ("pitch_deg", "俯仰角", 0.0, "°"),
            ("yaw_deg", "偏航角", 0.0, "°"),
        ]
        for key, label, value, unit in defaults:
            self._cached_status_map[key] = {
                "key": key,
                "label": label,
                "value": value,
                "unit": unit
            }
            
    def _is_value_valid(self, value: Any) -> bool:
        """
        判断值是否有效。
        根据你的需求：值不是0才视为有效。
        注意：如果 0 在你的业务中是合法数据（例如高度确实是0），需要修改此逻辑。
        """
        if value is None:
            return False
        # 这里假设 0 代表无效/未更新。如果 0 是有效值，请改为: return value is not None
        if isinstance(value, (int, float)) and value == 0:
            return False
        return True
    
    def _should_report(self, status_changed: bool, last_report_time: float) -> bool:
        """
        决定是否应该上报状态
        
        Args:
            status_changed: 状态是否发生变化
            
        Returns:
            是否应该上报
        """
        current_time = time.time()
        time_since_last = current_time - last_report_time
        
        # 如果状态变化且距离上次上报超过最小间隔，则上报
        if status_changed and time_since_last >= self.min_interval:
            return True
        
        # 如果距离上次上报超过最大间隔，无论状态是否变化都上报
        if time_since_last >= self.max_interval:
            return True
        
        return False
    
    def _create_device_status_report(self, device_state: Dict[str, Any]) -> Dict[str, Any]:
        """
        生成报告：仅当新数据有效时更新，否则沿用上次缓存的值。
        """
        
        # --- 1. 提取并更新缓存逻辑 ---
        
        # 辅助更新函数
        def update_cache_if_valid(cache_key: str, new_value: Any, label: str, unit: str):
            if self._is_value_valid(new_value):
                # 新数据有效，更新缓存
                self._cached_status_map[cache_key]["value"] = new_value
                # 可选：如果标签或单位也可能变，也可以在这里更新
                # self._cached_status_map[cache_key]["label"] = label
                # self._cached_status_map[cache_key]["unit"] = unit
                logger.debug(f"更新缓存 [{cache_key}] 为新值: {new_value}")
            else:
                # 新数据无效，保持缓存不变（即使用旧值）
                logger.debug(f"数据 [{cache_key}] 无效 (值:{new_value})，沿用旧值: {self._cached_status_map[cache_key]['value']}")

        # 处理本地相对位置
        # 注意：这里直接取 device_state 顶层的键，根据你的原代码逻辑
        update_cache_if_valid(
            "altitude_relative", 
            device_state.get("altitude_relative"), 
            "相对高度", "m"
        )
        
        update_cache_if_valid(
            "velocityHorizontal", 
            device_state.get("velocity_horizontal"), 
            "水平速度", "m/s"
        )
        
        update_cache_if_valid(
            "velocityVertical", 
            device_state.get("velocity_vertical"), 
            "垂直速度", "m/s"
        )

        # 处理全球定位
        update_cache_if_valid(
            "latitude", 
            device_state.get("latitude"), 
            "纬度", "°"
        )
        
        update_cache_if_valid(
            "longitude", 
            device_state.get("longitude"), 
            "经度", "°"
        )

        # 处理姿态
        update_cache_if_valid(
            "roll_deg", 
            device_state.get("roll_deg"), 
            "横滚角", "°"
        )
        
        update_cache_if_valid(
            "pitch_deg", 
            device_state.get("pitch_deg"), 
            "俯仰角", "°"
        )
        
        update_cache_if_valid(
            "yaw_deg", 
            device_state.get("yaw_deg"), 
            "偏航角", "°"
        )

        # --- 2. 组装最终列表 ---
        # 将缓存字典转换回列表格式 (保持原有的顺序可能需要额外处理，如果顺序不重要直接.values())
        # 为了保持和你原来代码一致的输出顺序，我们可以手动构建列表，但值取自缓存
        status_list = [
            self._cached_status_map["altitude_relative"],
            self._cached_status_map["velocityHorizontal"],
            self._cached_status_map["velocityVertical"],
            self._cached_status_map["latitude"],
            self._cached_status_map["longitude"],
            self._cached_status_map["roll_deg"],
            self._cached_status_map["pitch_deg"],
            self._cached_status_map["yaw_deg"],
        ]

        # --- 3. 构建最终报告 ---
        report = {
            "sn": self.robot_sn,
            "ver": self.version,
            "seq": self._get_next_sequence(),
            "type": 1,
            "ts": int(time.time() * 1000),
            "cmd": "DeviceStatusReport",
            "body": {
                "deviceType": "uav",
                "statusList": status_list
            }
        }
        
        # 调试日志：展示当前使用的值（包含旧值和新值）
        current_vals = {item['key']: item['value'] for item in status_list}
        # logger.info(f"生成报告，当前状态值: {current_vals}")
        
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
            current_status = self._create_device_status_report(robot_state)
            body = current_status['body']
            
            # 检查状态是否变化
            with self._lock:
                # 检查状态是否变化, 这里先假设状态变化
                status_changed = True
                should_report = self._should_report(status_changed, self._last_report_time)
                
                # 如果需要上报，则推送MQTT消息
                if should_report:
                    # 序列化状态报告
                    report_json = json.dumps(current_status)
                    #logger.info(f'准备上报状态: {report_json}')
                    
                    # 将消息添加到缓存
                    message_cache_manager.add_message(str(current_status['seq']), current_status)
                    # logger.info(f'已将状态报告消息加入缓存: seq={current_status["seq"]}')
                    
                    # 发布到MQTT
                    result = self.mqtt_client.publish(
                        topic=self.mqtt_response_topic,
                        payload=report_json,
                        qos=1,
                        retain=False
                    )
                    
                    # 更新最后上报时间和状态
                    self._last_report_time = time.time()
                    
                    if result.rc != mqtt.MQTT_ERR_SUCCESS:
                        logger.error(f'Failed to publish status report: {result.rc}')
                
        except Exception as e:
            logger.error(f'Error in status update: {e}')
            
    def _create_device_fault_status_report(self, device_fault_state: List[dict]) -> Dict[str, Any]:
        """
        生成报告：仅当新数据有效时更新，否则沿用上次缓存的值
        
        Args:
            device_fault_state: 设备故障状态数据
            
        Returns:
            故障状态报告
        """

        # 构建最终报告
        report = {
            "sn": self.robot_sn,
            "ver": self.version,
            "seq": self._get_next_sequence(),
            "type": 1,
            "ts": int(time.time() * 1000),
            "cmd": "FaultNotify",
            "body": {
                "error": "",  # 故障描述
                "code": 0,  # 故障码
                "faultList": device_fault_state
            }
        }
        
        # 调试日志：展示当前使用的值
        current_vals = {item['key']: item['value'] for item in device_fault_state}
        logger.debug(f"生成报告，当前故障状态值: {current_vals}")
        
        return report

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
    
    def update_fault_status(self, device_fault_state: List[dict]):
        """
        更新故障状态并在需要时上报
        
        Args:
            device_fault_state: 最新的机器人故障状态
        """
        try:
            # 创建故障状态报告
            current_status = self._create_device_fault_status_report(device_fault_state)
            body = current_status['body']
            
            # 检查故障状态是否变化
            with self._lock:
                # 检查故障状态是否变化
                # status_changed = self._is_status_changed(body, self._last_fault_status)
                status_changed = True
                logger.debug(f"故障状态是否变化: {status_changed}")
                should_report = self._should_report(status_changed, self._last_fault_report_time)
                
                # 如果需要上报，则推送MQTT消息
                if should_report:
                    # 序列化故障状态报告
                    report_json = json.dumps(current_status)
                    
                    # 将消息添加到缓存
                    message_cache_manager.add_message(str(current_status['seq']), current_status)
                    logger.info(f'已将故障状态报告消息加入缓存: seq={current_status["seq"]}')
                    
                    # 发布到MQTT
                    result = self.mqtt_client.publish(
                        topic=self.mqtt_response_topic,
                        payload=report_json,
                        qos=1,
                        retain=False
                    )
                    
                    # 更新最后上报时间和故障状态
                    self._last_fault_report_time = time.time()
                    self._last_fault_status = body.copy()
                    
                    if result.rc == mqtt.MQTT_ERR_SUCCESS:
                        logger.info(f'Fault status report published: seq={current_status["seq"]}')
                    else:
                        logger.error(f'Failed to publish fault status report: {result.rc}')
                
        except Exception as e:
            logger.error(f'Error in fault status update: {e}')