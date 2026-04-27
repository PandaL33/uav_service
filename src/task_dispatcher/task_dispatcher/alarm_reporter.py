#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
告警上报模块
负责处理机器人告警信息，上传图像并发送到MQTT服务器
"""

import json
import time
import uuid
import base64
import logging
import requests
# 抑制urllib3的SSL证书验证警告
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
from requests.auth import HTTPBasicAuth
import paho.mqtt.client as mqtt
from typing import Optional
from task_dispatcher.message_cache import message_cache_manager
from auth_utils import AuthManager 

logger = logging.getLogger(__name__)

class AlarmReporter:
    """
    告警上报类
    处理告警信息、上传图像、构建并发送告警消息到MQTT
    """
    
    def __init__(self, mqtt_client:Optional[mqtt.Client] = None, server_url='http://112.48.30.69:19000', 
                 mqtt_alarm_topic='/robot/up', device_sn='', version='1.0'):
        """
        初始化告警上报器
        
        Args:
            mqtt_client: MQTT客户端实例
            server_url: 服务器URL
            mqtt_alarm_topic: MQTT告警上报主题
            device_sn: 设备序列号
            version: 版本号 
        """
        self.mqtt_client = mqtt_client
        self.server_url = server_url
        self.file_server_url = f'{server_url}/upload'
        self.mqtt_alarm_topic = mqtt_alarm_topic
        self.device_sn = device_sn  # 设备序列号
        self.version = version  # 版本号
        self.auth_manager = AuthManager(server_url)
    
    def process_alarm_message(self, alarm_data):
        """
        处理告警消息，上传图像并发送到MQTT
        
        Args:
            alarm_data: 告警数据字典
        """
        try:
            # 解析必要字段
            task_id = alarm_data.get('taskId', '')
            pos = alarm_data.get('pos', {'x': 0, 'y': 0, 'z': 0, 'd': 0})
            image_base64 = alarm_data.get('image_base64', '')
            timestamp = alarm_data.get('timestamp', int(time.time() * 1000))
            algo_name = alarm_data.get('algo_name', '')
            
            # 直接使用algo_name作为algo_id
            algo_id = algo_name
            
            # 上传图像到文件服务器
            file_id = ''
            if image_base64:
                file_id = self.upload_image_to_file_server(image_base64)
                if not file_id or file_id == '':
                    logger.error('图像上传失败，跳过告警发送')
                    return
            
            # 构建预警消息结构体
            alarm_notify_msg = self.build_alarm_notify_message(
                task_id, algo_id, timestamp, pos, file_id
            )
            
            # 发送到MQTT
            self.send_alarm_to_mqtt(alarm_notify_msg)
            
        except Exception as e:
            logger.error(f'处理告警消息时出错: {str(e)}')
    
    def upload_image_to_file_server(self, image_base64):
        """
        上传base64编码的图像到文件服务器，需要先获取token
        
        Args:
            image_base64: base64编码的图像数据
            
        Returns:
            str: 文件ID
        """
        try:
            # 获取上传token
            token = self.auth_manager.get_upload_token()
            
            if not token or token == '':
                logger.error('Failed to get upload token')
                # 生成模拟的fileId作为降级方案
                return ''
            
            # 构建带token的上传URL
            upload_url = f'{self.server_url}/file/file/upload/{token}'
            
            # 准备上传数据，使用时间戳确保文件名唯一性
            timestamp = int(time.time() * 1000)
            unique_filename = f'alarm_image_{timestamp}.jpg'
            files = {
                'file': (unique_filename, base64.b64decode(image_base64), 'image/jpeg')
            }
            logger.info(f'准备上传图像到文件服务器: {upload_url}, 文件名={unique_filename}')
            # 发送请求到文件服务器，添加verify参数控制SSL证书验证
            response = requests.post(upload_url, files=files, data={}, timeout=10, verify=False)  # 生产环境建议设置为True并提供证书
            
            if response.status_code == 200:
                result = response.json()
                # 从data对象中获取id字段
                if result.get('data') and 'id' in result['data']:
                    return result['data']['id']
                else:
                    logger.warning(f'响应中未找到预期的data.id字段: {result}')
                    return ''
            else:
                logger.error(f'图像上传失败: 状态码={response.status_code}, 响应={response.text}')
                # 如果上传失败，生成一个模拟的fileId
                return ''
                
        except Exception as e:
            logger.error(f'图像上传异常: {str(e)}')
            # 异常情况下也生成一个模拟的fileId
            return ''
    
      
    def build_alarm_notify_message(self, task_id, algo_id, timestamp, pos, file_id):
        """
        构建预警通知消息结构体
        
        Args:
            task_id: 任务ID
            algo_id: 算法ID
            timestamp: 告警时间戳
            pos: 位置信息
            file_id: 文件ID
            
        Returns:
            dict: 告警通知消息
        """
        # 生成唯一的seq
        seq = str(uuid.uuid4()).replace('-', '')
        
        return {
            'body': {
                'taskId': task_id,
                'algoId': algo_id,
                'tsReport': timestamp,  # 转换为毫秒时间戳
                'pos': pos,
                'fileId': file_id
            },
            'cmd': 'AlarmNotify',
            'seq': seq,
            'sn': self.device_sn,
            'ts': int(time.time() * 1000),
            'type': 1,
            'ver': self.version
        }
    
    def send_alarm_to_mqtt(self, alarm_message):
        """
        发送告警消息到MQTT
        
        Args:
            alarm_message: 告警消息字典
        """
        try:
            # 将消息添加到缓存
            message_cache_manager.add_message(alarm_message['seq'], alarm_message)
            logger.info(f'已将告警消息加入缓存: seq={alarm_message["seq"]}')
            
            # 发布到告警主题
            self.mqtt_client.publish(self.mqtt_alarm_topic, json.dumps(alarm_message))
            logger.info(f'已发送告警通知到MQTT: taskId={alarm_message["body"]["taskId"]}, fileId={alarm_message["body"]["fileId"]}')
            
        except Exception as e:
            logger.error(f'发送告警通知失败: {str(e)}')


if __name__ == '__main__':
    # 测试代码
    import paho.mqtt.client as mqtt
    
    # 模拟MQTT客户端
    class MockMQTTClient:
        def publish(self, topic, message):
            print(f'模拟发布到MQTT: topic={topic}, message={message[:100]}...')
    
    # 创建模拟客户端
    mock_client = MockMQTTClient()
    
    # 创建告警上报器
    reporter = AlarmReporter(mock_client)
    
    # 测试告警处理
    test_alarm_data = {
        'taskId': 'test_task_123',
        'algo_name': 'object_detection',
        'timestamp': int(time.time() * 1000),
        'pos': {'x': 1.0, 'y': 2.0, 'z': 0.0, 'd': 0.5},
        'image_base64': ''  # 测试时可以留空，避免实际上传
    }
    
    print('测试告警上报...')
    reporter.process_alarm_message(test_alarm_data)
    print('测试完成')