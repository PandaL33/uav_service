#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import logging
import time
import requests
from .config import ROBOT_TYPE, FFR_SERVER_URL

# 配置日志
logger = logging.getLogger('task_dispatcher.command_deliver')

class CommandDeliver:
    """
    远程指令下发类，负责处理CommandDeliver命令并调用相应的执行接口
    """
    
    def __init__(self, node, mqtt_client, response_topic):
        """
        初始化远程指令下发类
        
        Args:
            node: ROS2节点实例
            mqtt_client: MQTT客户端实例
            response_topic: MQTT响应主题
        """
        self.node = node
        self.mqtt_client = mqtt_client
        self.mqtt_response_topic = response_topic
        
        logger.info('远程指令下发类初始化完成')
    
    def process_command_deliver(self, message_data):
        """
        处理CommandDeliver远程指令下发命令
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
            if 'command' not in body:
                logger.error('CommandDeliver消息缺少必要字段: command')
                self.send_ack_response(message_data, 400, 'missing command')
                return
            
            command = body['command']
            
            # 验证command类型
            if not isinstance(command, str):
                logger.error(f'command字段类型错误，应为string: {command}')
                self.send_ack_response(message_data, 400, 'command must be string')
                return
            
            logger.info(f'远程指令下发命令: {command}')
            
            # 根据机器人类型处理指令下发
            if ROBOT_TYPE == 'ffr':
                # FFR机器人，调用HTTP接口
                is_success = self._call_ffr_execute_cmd(command)
                if not is_success:
                    self.send_ack_response(message_data, 500, 'command deliver failed')
                    return
            else:
                # 其他类型机器人，预留接口
                logger.info(f'非FFR机器人类型，指令下发命令已记录: {command}')
            
            # 发送成功响应
            self.send_ack_response(message_data, 200, 'ok')
            
        except Exception as e:
            logger.error(f'处理远程指令下发命令异常: {str(e)}')
            self.send_ack_response(message_data, 500, f'internal error: {str(e)}')
    
    def _call_ffr_execute_cmd(self, command):
        """
        调用FFR机器人的指令执行HTTP接口
        """
        try:
            # 构建请求参数
            payload = {
                'cmd': command
            }
            
            url = f'{FFR_SERVER_URL}/fireRobot/executeCmd'
            headers = {
                'accept': '*/*',
                'Content-Type': 'application/json'
            }
            
            logger.info(f'调用FFR指令执行接口: url={url}, payload={payload}')
            
            response = requests.post(url, json=payload, headers=headers, timeout=1.5)
            response.raise_for_status()  # 检查请求是否成功
            result = response.json()
            if result.get('code') != 200:
                logger.error(f'FFR指令执行接口调用失败: {result}')  
                return False

            logger.info(f'FFR指令执行接口调用成功')
            return True
            
        except requests.RequestException as e:
            logger.error(f'调用FFR指令执行接口异常: {str(e)}')
            return False
        except Exception as e:
            logger.error(f'处理FFR指令执行响应异常: {str(e)}')
            return False
    
    def send_ack_response(self, original_message, status, error):
        """
        发送ACK响应消息
        """
        try:
            # 构建ACK响应
            ack_response = {
                'sn': original_message.get('sn', ''),
                'ver': original_message.get('ver', '1.0'),
                'seq': original_message.get('seq', 0),
                'type': 2,  # 响应类型
                'ts': int(time.time() * 1000),
                'cmd': 'Ack',
                'status': status,
                'error': error
            }
            
            # 发布响应
            self.mqtt_client.publish(
                topic=self.mqtt_response_topic,
                payload=json.dumps(ack_response, ensure_ascii=False),
                qos=0
            )
            
            logger.info(f'已发送ACK响应: status={status}, error={error}')
            
        except Exception as e:
            logger.error(f'发送ACK响应异常: {str(e)}')