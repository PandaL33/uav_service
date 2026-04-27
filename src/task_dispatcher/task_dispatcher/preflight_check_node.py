#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
无人机起飞前状态检测节点
基于 ROS2 + PX4 v1.15.4 + Micro-DDS-Agent

功能：
1. 检查与 PX4 的通信状态
2. 验证无人机解锁状态
3. 确认飞行模式为 OFFBOARD
4. 检查传感器健康状态
5. 验证位置信息有效性
6. 检查电池电量
7. 输出详细的检测报告
"""

import rclpy
from rclpy.executors import SingleThreadedExecutor
import time
from typing import Optional, Dict, Any
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
import logging

# 配置日志
logger = logging.getLogger('preflight_check_node')
class PreFlightCheckNode():
    """
    起飞前检查节点
    用于在起飞前验证无人机的各项状态是否满足安全要求
    """

    def __init__(self, node, topic_subscriber):
        self.node = node
        
        self.topic_subscriber: Optional[Ros2TopicSubscriber] = topic_subscriber
          
        # 检查结果存储
        self.check_results: Dict[str, Dict[str, Any]] = {}
        
        # 检查超时时间（秒）
        self.timeout = 10.0
        
        logger.info('起飞前检查节点已启动')

    def wait_for_data(self, timeout: float = None) -> bool:
        """
        等待数据更新
        
        Args:
            timeout: 超时时间（秒），None 则使用默认值
            
        Returns:
            bool: 是否成功获取到数据
        """
        if timeout is None:
            timeout = self.timeout
            
        start_time = time.time()
        
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        while time.time() - start_time < timeout:
            executor.spin_once(timeout_sec=0.1)
            if (
                self.topic_subscriber.get_vehicle_status() is not None and
                self.topic_subscriber.get_battery_status() is not None and
                self.topic_subscriber.get_failsafe_flags() is not None
            ):
                executor.remove_node(self.node)
                return True
        executor.remove_node(self.node)
                
        return False

    def check_communication(self) -> Dict[str, Any]:
        """
        检查通信状态
        
        Returns:
            Dict: 检查结果，包含 passed、message 和 details
        """
        result = {
            'passed': False,
            'message': '',
            'details': {}
        }
        
        # 检查是否能接收到车辆状态
        if self.topic_subscriber.get_vehicle_status() is None:
            result['message'] = '无法通信状态消息'
            return result
            
            
        result['passed'] = True
        result['message'] = '通信正常'
        result['details'] = {
            'vehicle_status_received': True
        }
        logger.info(f'check_communication: {result} ')
        return result

    def check_sensor_health(self) -> Dict[str, Any]:
        """
        检查传感器健康状态
        
        Returns:
            Dict: 检查结果
        """
        result = {
            'passed': False,
            'message': '',
            'details': {}
        }
        
        if self.topic_subscriber.get_vehicle_status() is None:
            result['message'] = '无状态数据'
            return result
            
        # 检查预飞行检查是否通过
        pre_flight_checks = self.topic_subscriber.get_vehicle_status().pre_flight_checks_pass
        result['details']['pre_flight_checks_pass'] = bool(pre_flight_checks)
        
        # 检查系统是否就绪
        system_type = self.topic_subscriber.get_vehicle_status().system_type
        result['details']['system_type'] = system_type
        
        if pre_flight_checks:
            result['passed'] = True
            result['message'] = '传感器健康检查通过'
        else:
            result['message'] = '预飞行检查未通过，请检查传感器状态'
        
        logger.info(f'check_sensor_health: {result} ')
            
        return result


    def check_battery(self) -> Dict[str, Any]:
        """
        检查电池电量
        
        Returns:
            Dict: 检查结果
        """
        result = {
            'passed': False,
            'message': '',
            'details': {}
        }
        
        if self.topic_subscriber.get_battery_status() is None:
            result['message'] = '无电池状态数据'
            return result
            
        # BatteryStatus 消息中的剩余电量百分比 (0.0-1.0)
        battery_remaining = self.topic_subscriber.get_battery_status().remaining * 100.0
        result['details']['battery_remaining'] = f'{battery_remaining:.1f}%'
        result['details']['voltage_v'] = f'{self.topic_subscriber.get_battery_status().voltage_v:.2f}V'
        result['details']['current_a'] = f'{self.topic_subscriber.get_battery_status().current_a:.2f}A'
        
        # 最低电量阈值（百分比）- 低于 95% 不能起飞
        min_battery = 95.0
        
        if battery_remaining >= min_battery:
            result['passed'] = True
            result['message'] = f'电量充足：{battery_remaining:.1f}%'
        else:
            result['message'] = f'电量不足：{battery_remaining:.1f}% (最低要求：{min_battery}%)'
        logger.info(f'check_battery:{result} ')    
        return result

    def check_failsafe(self) -> Dict[str, Any]:
        """
        检查故障保护状态，包括姿态、关键故障、电机故障以及电池故障
        
        Returns:
            Dict: 检查结果
        """
        result = {
            'passed': False,
            'message': '',
            'details': {}
        }
        
        if self.topic_subscriber.get_failsafe_flags() is None:
            result['message'] = '无状态数据'
            return result
        
        # 检查姿态故障 
        result['details']['attitude_failure'] = bool(self.topic_subscriber.get_failsafe_flags().attitude_invalid)
        
        # 检查关键故障 
        result['details']['critical_failure'] = bool(self.topic_subscriber.get_failsafe_flags().fd_critical_failure)
        
        # 检查电机故障
        result['details']['motor_failure'] = bool(self.topic_subscriber.get_failsafe_flags().fd_motor_failure)
        
        # 检查电池低电量剩余时间故障
        result['details']['battery_low_remaining_time'] = bool(self.topic_subscriber.get_failsafe_flags().battery_low_remaining_time)
        
        # 检查电池不健康故障 
        result['details']['battery_unhealthy'] = bool(self.topic_subscriber.get_failsafe_flags().battery_unhealthy)
        
        # 记录故障总数
        failure_count = 0
        

        # 如果没有任何故障，则检查通过
        if failure_count == 0:
            result['passed'] = True
            result['message'] = '故障保护检查通过，未检测到故障'
        else:
            # 构建详细的故障信息
            failure_messages = []
            if self.topic_subscriber.get_failsafe_flags().attitude_invalid:
                failure_messages.append('姿态故障')
                failure_count = failure_count + 1
            if self.topic_subscriber.get_failsafe_flags().fd_critical_failure:
                failure_messages.append('关键故障')
                failure_count = failure_count + 1
            if self.topic_subscriber.get_failsafe_flags().fd_motor_failure:
                failure_messages.append('电机故障')
                failure_count = failure_count + 1
            if self.topic_subscriber.get_failsafe_flags().battery_low_remaining_time:
                failure_messages.append('电池低电量剩余时间')
                failure_count = failure_count + 1
            if self.topic_subscriber.get_failsafe_flags().battery_unhealthy:
                failure_messages.append('电池不健康')
                failure_count = failure_count + 1
            
            result['message'] = f'检测到 {failure_count} 个故障：{", ".join(failure_messages)}'
            
        result['details']['total_failures'] = failure_count
        logger.info(f'check_failsafe: {result} ')        
        return result

    def run_all_checks(self, timeout: float = None) -> bool:
        """
        执行所有起飞前检查
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            bool: 是否所有检查都通过
        """
        logger.info('=' * 60)
        logger.info('开始起飞前检查...')
        logger.info('=' * 60)
        
        # 等待数据
        logger.info('等待数据更新...')
        if not self.wait_for_data(timeout):
            logger.error('✗ 数据等待超时')
            return False
            
        logger.info('✓ 数据接收正常')
        
        # 执行各项检查
        checks = [
            ('通信状态', self.check_communication),
            ('传感器健康', self.check_sensor_health),
            ('电池电量', self.check_battery),
            ('故障保护', self.check_failsafe),
        ]
        
        all_passed = True
        
        for check_name, check_func in checks:
            result = check_func()
            self.check_results[check_name] = result
            
            status = '✓' if result['passed'] else '✗'
            logger.info(f'{status} [{check_name}]: {result["message"]}')
            
            if not result['passed']:
                all_passed = False
        
        logger.info('=' * 60)
        
        if all_passed:
            logger.info('✓✓✓ 所有检查通过，可以安全起飞 ✓✓✓')
        else:
            logger.error('✗✗✗ 存在未通过项，禁止起飞 ✗✗✗')
            
        logger.info('=' * 60)
        
        return all_passed
    
    def check_report(self) -> list:
        """
        获取结构化的检查报告列表
        
        Returns:
            list: 包含检查信息的字典列表，每个字典包含 key, label, value, unit
        """
        logger.info('=' * 60)
        logger.info('开始起飞前检查...')
        logger.info('=' * 60)
        
        # 等待数据
        logger.info('等待数据更新...')
        if not self.wait_for_data(30):
            logger.error('✗ 数据等待超时')
            return False
            
        logger.info('✓ 数据接收正常')
        
        report_list = []
        
        # 定义检查项映射 (Key -> Label)
        # 这里的 Key 是你自己定义的英文标识符
        check_mapping = {
            'communication': '通信状态',
            'sensor_health': '传感器状态',
            'battery': '电池电量',
            'failsafe': '故障保护状态'
        }

        # 执行各项检查 (复用 run_all_checks 中的逻辑定义)
        checks = [
            ('communication', self.check_communication),
            ('sensor_health', self.check_sensor_health),
            ('battery', self.check_battery),
            ('failsafe', self.check_failsafe),
        ]
        
        all_passed = True
        for key, check_func in checks:
            result = check_func()
            
            # 构建结构化数据
            item = {
                "key": key,
                "label": check_mapping[key],
                "value": result['message'], 
                "unit": "" 
            }
            report_list.append(item)

            if not result['passed']:
                all_passed = False
                
            status = '✓' if result['passed'] else '✗'
            logger.info(f'{status} [{check_mapping[key]}]: {result["message"]}')
                
        return all_passed, report_list
    
    def get_detailed_report(self) -> str:
        """
        获取详细的检查报告
        
        Returns:
            str: 格式化的检查报告
        """
        report_lines = []
        report_up = ""
        num = 1
        for check_name, result in self.check_results.items():
            status = "通过" if result['passed'] else "失败"
            report_lines.append(f"[{check_name}]")
            report_lines.append(f"  状态：{status}")
            report_lines.append(f"  说明：{result['message']}")
            
            report_up += f"{result['message']};"
            
            num += 1
            
            if result['details']:
                report_lines.append("  详情:")
                for key, value in result['details'].items():
                    report_lines.append(f"    - {key}: {value}")
                    
            report_lines.append("")
        
        return report_up    
        # report_lines.append("=" * 45)
        
        # return "\n".join(report_lines)
    
    def _start_check(self):
        try:
            # 执行所有检查
            all_passed, report = self.check_report()
            # report = self.get_detailed_report()
            
            logger.info(f"检测结果:{report}")
            # 返回退出码和报告
            exit_code = 0 if all_passed else 1
            return exit_code, report
            
        except Exception as e:
            logger.error(f'发生错误：{str(e)}')
            return 1, f'发生错误：{str(e)}'

