#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import logging
import time
import rclpy
import requests
from rclpy.node import Node
from .config import ROBOT_TYPE, FFR_SERVER_URL

# 配置日志
logger = logging.getLogger('task_dispatcher.device_control')

class DeviceControl:
    """
    设备控制类，负责处理DeviceControl命令并调用相应的设备控制接口
    """
    
    def __init__(self, node, mqtt_client, response_topic):
        """
        初始化设备控制类
        +
        Args:
            node: ROS2节点实例
            mqtt_client: MQTT客户端实例
            response_topic: MQTT响应主题
        """
        self.node = node
        self.mqtt_client = mqtt_client
        self.mqtt_response_topic = response_topic
        
        # 支持的设备名称列表
        self.supported_devices = ['alarmLight', 'fireDevice', 'lidar']
        
        # fireDevice支持的模式
        self.fire_device_modes = [0, 1, 2]  # 0-直喷；1-上下喷；2-左右喷
        
        logger.info('设备控制类初始化完成')
    
    def process_device_control(self, message_data):
        """
        处理DeviceControl设备控制命令
        """
        try:
            # 提取必要信息
            cmd = message_data['cmd']
            body = message_data['body']
            sn = message_data.get('sn', '')
            seq = message_data.get('seq', 0)
            
            # 验证body内容
            if not isinstance(body, dict):
                logger.error('消息body格式错误')
                self.send_ack_response(message_data, 400, 'invalid body format')
                return
            
            # 检查必要字段
            if 'deviceName' not in body:
                logger.error('DeviceControl消息缺少必要字段: deviceName')
                self.send_ack_response(message_data, 400, 'missing deviceName')
                return
                
            if 'enable' not in body:
                logger.error('DeviceControl消息缺少必要字段: enable')
                self.send_ack_response(message_data, 400, 'missing enable')
                return
                
            if 'argv' not in body:
                logger.error('DeviceControl消息缺少必要字段: argv')
                self.send_ack_response(message_data, 400, 'missing argv')
                return
            
            device_name = body['deviceName']
            enable = body['enable']
            argv = body['argv']
            
            # 验证设备名称
            if device_name not in self.supported_devices:
                logger.error(f'不支持的设备名称: {device_name}')
                self.send_ack_response(message_data, 400, f'unsupported device: {device_name}')
                return
            
            # 验证enable类型
            if not isinstance(enable, bool):
                logger.error(f'enable字段类型错误，应为bool: {enable}')
                self.send_ack_response(message_data, 400, 'enable must be boolean')
                return
            
            # 验证argv类型
            if not isinstance(argv, dict):
                logger.error(f'argv字段类型错误，应为object: {argv}')
                self.send_ack_response(message_data, 400, 'argv must be object')
                return
            
            # 对于fireDevice，验证mode参数
            if device_name == 'fireDevice':
                if enable and 'mode' not in argv:
                    logger.error('fireDevice缺少必要参数: mode')
                    self.send_ack_response(message_data, 400, 'fireDevice missing mode')
                    return
                    
                mode = argv.get('mode', 0)
                if not isinstance(mode, int) or mode not in self.fire_device_modes:
                    logger.error(f'fireDevice mode参数错误: {mode}，必须是0-2的整数')
                    self.send_ack_response(message_data, 400, 'fireDevice mode must be 0-2 integer')
                    return
            
            logger.info(f'设备控制命令: deviceName={device_name}, enable={enable}, argv={argv}')
            
            # 根据机器人类型处理设备控制
            if ROBOT_TYPE == 'ffr':
                # FFR机器人，调用HTTP接口
                is_success = self._call_ffr_device_control(device_name, enable, argv)
                if not is_success:
                    self.send_ack_response(message_data, 500, 'ffr device control failed')
                    return
            else:
                # 其他类型机器人，预留接口
                logger.info(f'非FFR机器人类型，设备控制命令已记录: {device_name}={enable}')
            
            # 发送成功响应
            self.send_ack_response(message_data, 200, 'ok')
            
        except Exception as e:
            logger.error(f'处理设备控制命令异常: {str(e)}')
            self.send_ack_response(message_data, 500, f'internal error: {str(e)}')
    
    def _call_ffr_device_control(self, device_name, enable, argv):
        """
        调用FFR机器人的设备控制HTTP接口
        """
        try:
            # 构建请求参数
            payload = {
                'deviceName': device_name,
                'isEnable': enable,
                'argv': argv
            }
            
            url = f'{FFR_SERVER_URL}/fireRobot/deviceControl'
            headers = {
                'accept': '*/*',
                'Content-Type': 'application/json'
            }
            
            logger.info(f'调用FFR设备控制接口: url={url}, payload={payload}')
            
            response = requests.post(url, json=payload, headers=headers, timeout=1.5)
            response.raise_for_status()  # 检查请求是否成功
            
            result = response.json()
            if result.get("code") != 200:
                logger.error(f'设备控制接口调用失败: {result}')
                return False
            logger.info(f'设备控制接口调用成功: {result}')
            return True
        
        except requests.RequestException as e:
            logger.error(f'调用FFR设备控制接口失败: {str(e)}')
            return False
        except Exception as e:
            logger.error(f'处理设备控制响应异常: {str(e)}')
            return False    
    
    def send_ack_response(self, original_message, status, error):
        """
        发送ACK响应
        """
        try:
            # 构建响应消息
            ack_response = {
                'sn': original_message.get('sn', ''),
                'ver': original_message.get('ver', '1.0'),
                'seq': original_message.get('seq', 0),
                'type': 2,  # ACK响应类型
                'ts': int(time.time() * 1000),
                'cmd': 'Ack',
                'status': status,
                'error': error
            }
            
            # 发送响应
            self.mqtt_client.publish(self.mqtt_response_topic, json.dumps(ack_response))
            logger.info(f'发送ACK响应: {ack_response}')
            
        except Exception as e:
            logger.error(f'发送ACK响应失败: {str(e)}')