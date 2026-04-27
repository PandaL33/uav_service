#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
import json
import threading
from unittest.mock import MagicMock, patch
from task_dispatcher.cruise_task_manager import CruiseTaskManager

class TestCruiseTaskManager(unittest.TestCase):
    """
    巡检任务管理器的单元测试
    """
    
    def setUp(self):
        """
        测试前的设置
        """
        # 创建模拟对象
        self.mock_node = MagicMock()
        self.mock_mqtt_client = MagicMock()
        self.mock_ros_topic_subscriber = MagicMock()
        
        # 初始化CruiseTaskManager
        self.manager = CruiseTaskManager(
            node=self.mock_node,
            mqtt_client=self.mock_mqtt_client,
            response_topic='/test/response',
            ros_topic_subscriber=self.mock_ros_topic_subscriber
        )
        
        # 模拟时间模块，避免测试中实际等待
        self.time_patcher = patch('task_dispatcher.cruise_task_manager.time')
        self.mock_time = self.time_patcher.start()
    
    def tearDown(self):
        """
        测试后的清理
        """
        # 停止任务执行
        self.manager.stop_execution = True
        if hasattr(self.manager, 'execution_thread') and self.manager.execution_thread:
            self.manager.execution_thread.join(timeout=1.0)
        
        # 停止模拟
        self.time_patcher.stop()
    
    def test_set_cruise_task_success(self):
        """
        测试成功设置巡检任务
        """
        # 准备测试数据
        message_data = {
            'sn': 'D1Ultra001',
            'ver': '1.0',
            'seq': 1001,
            'type': 1,
            'ts': 1701436000000,
            'cmd': 'SetCruiseTask',
            'body': {
                'taskName': '测试巡检任务',
                'taskId': 'cruise_test_001',
                'type': 'once',
                'mapName': 'test_map',
                'cruises': [
                    {
                        'idx': 0,
                        'point': {'x': 1.0, 'y': 1.0, 'z': 0.0, 'd': 0.0},
                        'speed': 'mid',
                        'ingActions': [],
                        'endActions': []
                    }
                ]
            }
        }
        
        # 调用方法
        self.manager._process_set_cruise_task(message_data)
        
        # 验证结果
        self.assertIn('cruise_test_001', self.manager.tasks)
        self.assertEqual(self.manager.tasks['cruise_test_001']['taskName'], '测试巡检任务')
        self.mock_mqtt_client.publish.assert_called_once()
    
    def test_set_cruise_task_missing_field(self):
        """
        测试设置巡检任务时缺少必要字段
        """
        # 缺少taskId的测试数据
        message_data = {
            'sn': 'D1Ultra001',
            'ver': '1.0',
            'seq': 1001,
            'type': 1,
            'ts': 1701436000000,
            'cmd': 'SetCruiseTask',
            'body': {
                'taskName': '测试巡检任务',
                'type': 'once',
                'mapName': 'test_map',
                'cruises': []
            }
        }
        
        # 调用方法
        self.manager._process_set_cruise_task(message_data)
        
        # 验证结果
        self.assertEqual(len(self.manager.tasks), 0)
        self.mock_mqtt_client.publish.assert_called_once()
    
    def test_get_cruise_task(self):
        """
        测试获取巡检任务
        """
        # 先设置一个任务
        task_data = {
            'taskName': '测试巡检任务',
            'taskId': 'cruise_test_001',
            'type': 'once',
            'mapName': 'test_map',
            'cruises': []
        }
        self.manager.tasks['cruise_test_001'] = task_data
        
        # 获取指定任务的测试数据
        message_data = {
            'sn': 'D1Ultra001',
            'ver': '1.0',
            'seq': 1002,
            'type': 1,
            'ts': 1701436100000,
            'cmd': 'GetCruiseTask',
            'body': {'taskId': 'cruise_test_001'}
        }
        
        # 调用方法
        self.manager._process_get_cruise_task(message_data)
        
        # 验证调用了publish方法
        self.mock_mqtt_client.publish.assert_called_once()
        
        # 重置mock
        self.mock_mqtt_client.publish.reset_mock()
        
        # 获取所有任务的测试数据
        message_data['body'] = {}
        
        # 调用方法
        self.manager._process_get_cruise_task(message_data)
        
        # 验证调用了publish方法
        self.mock_mqtt_client.publish.assert_called_once()
    
    def test_exec_cruise_task(self):
        """
        测试执行巡检任务
        """
        # 先设置一个任务
        task_data = {
            'taskName': '测试巡检任务',
            'taskId': 'cruise_test_001',
            'type': 'once',
            'mapName': 'test_map',
            'cruises': [
                {
                    'idx': 0,
                    'point': {'x': 1.0, 'y': 1.0, 'z': 0.0, 'd': 0.0},
                    'ingActions': [],
                    'endActions': []
                }
            ]
        }
        self.manager.tasks['cruise_test_001'] = task_data
        
        # 执行任务的测试数据
        message_data = {
            'sn': 'D1Ultra001',
            'ver': '1.0',
            'seq': 1003,
            'type': 1,
            'ts': 1701436200000,
            'cmd': 'ExecCruiseTask',
            'body': {'taskId': 'cruise_test_001'}
        }
        
        # 模拟执行线程完成
        with patch.object(self.manager, '_execute_task_thread') as mock_execute:
            # 调用方法
            self.manager._process_exec_cruise_task(message_data)
            
            # 验证结果
            self.assertEqual(self.manager.current_task_id, 'cruise_test_001')
            self.mock_mqtt_client.publish.assert_called_once()
            mock_execute.assert_called_once()
    
    def test_stop_cruise_task(self):
        """
        测试停止巡检任务
        """
        # 模拟正在执行的任务
        self.manager.is_executing = True
        self.manager.current_task_id = 'cruise_test_001'
        
        # 停止任务的测试数据
        message_data = {
            'sn': 'D1Ultra001',
            'ver': '1.0',
            'seq': 1004,
            'type': 1,
            'ts': 1701436300000,
            'cmd': 'StopCruiseTask',
            'body': {'taskId': 'cruise_test_001'}
        }
        
        # 调用方法
        self.manager._process_stop_cruise_task(message_data)
        
        # 验证结果
        self.assertFalse(self.manager.is_executing)
        self.assertIsNone(self.manager.current_task_id)
        self.mock_mqtt_client.publish.assert_called_once()
    
    def test_process_cruise_task_dispatcher(self):
        """
        测试process_cruise_task方法对不同命令的分发
        """
        # 模拟各个处理方法
        with patch.object(self.manager, '_process_set_cruise_task') as mock_set, \
             patch.object(self.manager, '_process_get_cruise_task') as mock_get, \
             patch.object(self.manager, '_process_exec_cruise_task') as mock_exec, \
             patch.object(self.manager, '_process_stop_cruise_task') as mock_stop:
            # 测试SetCruiseTask
            message_data = {'cmd': 'SetCruiseTask'}
            self.manager.process_cruise_task(message_data)
            mock_set.assert_called_once_with(message_data)
            
            # 测试GetCruiseTask
            mock_set.reset_mock()
            message_data = {'cmd': 'GetCruiseTask'}
            self.manager.process_cruise_task(message_data)
            mock_get.assert_called_once_with(message_data)
            
            # 测试ExecCruiseTask
            mock_get.reset_mock()
            message_data = {'cmd': 'ExecCruiseTask'}
            self.manager.process_cruise_task(message_data)
            mock_exec.assert_called_once_with(message_data)
            
            # 测试StopCruiseTask
            mock_exec.reset_mock()
            message_data = {'cmd': 'StopCruiseTask'}
            self.manager.process_cruise_task(message_data)
            mock_stop.assert_called_once_with(message_data)
    
    def test_send_ack_response(self):
        """
        测试发送Ack响应
        """
        # 测试数据
        original_message = {
            'sn': 'D1Ultra001',
            'ver': '1.0',
            'seq': 1001
        }
        
        # 调用方法
        self.manager.send_ack_response(original_message, 200, 'ok', {'data': 'test'})
        
        # 验证结果
        self.mock_mqtt_client.publish.assert_called_once()
        # 获取调用参数
        publish_args = self.mock_mqtt_client.publish.call_args[0]
        topic = publish_args[0]
        payload = json.loads(publish_args[1])
        
        # 验证响应格式
        self.assertEqual(topic, '/test/response')
        self.assertEqual(payload['sn'], 'D1Ultra001')
        self.assertEqual(payload['seq'], 1001)
        self.assertEqual(payload['status'], 200)
        self.assertEqual(payload['error'], 'ok')
        self.assertEqual(payload['body'], {'data': 'test'})

if __name__ == '__main__':
    unittest.main()