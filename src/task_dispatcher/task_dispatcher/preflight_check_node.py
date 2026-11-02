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

import time
from typing import Optional, Dict, Any, Tuple
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
import logging

# 配置日志
logger = logging.getLogger('preflight_check_node')

# PX4 VehicleStatus.failure_detector_status 故障位
_FAILURE_DETECTOR_BITS = [
    (1 << 0, '检测到横滚姿态故障'),
    (1 << 1, '检测到俯仰姿态故障'),
    (1 << 2, '检测到高度故障'),
    (1 << 3, '外部系统触发故障'),
    (1 << 4, '电调解锁失败'),
    (1 << 5, '电池故障'),
    (1 << 6, '螺旋桨不平衡'),
    (1 << 7, '电机故障'),
]

# failsafe_flags 中受「当前飞行模式要求」约束的检查项：
# (标志字段, 模式要求字段, 失败说明)
_MODE_REQUIREMENT_CHECKS = [
    ('angular_velocity_invalid', 'mode_req_angular_velocity', '角速度估计无效（陀螺仪数据缺失或超时）'),
    ('attitude_invalid', 'mode_req_attitude', '姿态估计无效'),
    ('local_altitude_invalid', 'mode_req_local_alt', '高度估计无效'),
    ('local_position_invalid', 'mode_req_local_position', '本地位置估计无效'),
    ('local_position_invalid_relaxed', 'mode_req_local_position_relaxed', '本地位置估计精度不足'),
    ('global_position_invalid', 'mode_req_global_position', '全局位置估计无效'),
    ('offboard_control_signal_lost', 'mode_req_offboard_signal', 'OFFBOARD 控制信号丢失（未收到 offboard_control_mode 或所需估计量无效）'),
    ('home_position_invalid', 'mode_req_home_position', 'Home 点未设置'),
    ('auto_mission_missing', 'mode_req_mission', '未上传航线任务'),
    ('manual_control_signal_lost', 'mode_req_manual_control', '遥控器信号丢失'),
]

# BatteryStatus.warning 中会导致无法起飞的等级
_BATTERY_WARNING_TEXT = {
    1: '电池低电量告警',
    2: '电池电压临界',
    3: '电池电压紧急（需立即降落）',
    4: '电池已失效',
    6: '电池不健康',
}

# BatteryStatus.faults 故障位
_BATTERY_FAULT_BITS = [
    (1 << 0, '深度放电'),
    (1 << 1, '电压尖峰'),
    (1 << 2, '电芯失效'),
    (1 << 3, '过流'),
    (1 << 4, '过温'),
    (1 << 5, '低温'),
    (1 << 6, '电压不兼容'),
    (1 << 7, '固件不兼容'),
    (1 << 8, '型号不支持'),
    (1 << 9, '硬件故障'),
]

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
        
        while time.time() - start_time < timeout:
            # 缓存数据由主线程 executor 的回调持续刷新，这里只轮询缓存值。
            # 不能在任务线程里再 spin 同一 node：会和主线程 rclpy.spin() 竞争，
            # 抛 ValueError: generator already executing，关机阶段还会打断主 spin。
            if (
                self.topic_subscriber.get_vehicle_status() is not None and 
                self.topic_subscriber.get_battery_status() is not None and 
                self.topic_subscriber.get_failsafe_flags() is not None
            ):
                return True
            time.sleep(0.1)

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

    def _mode_requirement_applies(self, failsafe_flags, requirement_field: str, nav_state: int) -> bool:
        """
        判断某项模式要求是否作用于当前飞行模式
        mode_req_* 为按 nav_state 置位的位图，第 nav_state 位为 1 表示该模式要求此项检查
        """
        return bool(getattr(failsafe_flags, requirement_field) & (1 << nav_state))

    def _collect_preflight_failures(self, vehicle_status, details: Dict[str, Any]) -> list:
        """
        根据当前已订阅的 PX4 话题，尽可能还原 pre_flight_checks_pass 为 False 的具体原因

        Returns:
            list: 中文失败原因列表，无法从已订阅话题还原时返回空列表
        """
        failures = []
        nav_state = int(vehicle_status.nav_state)

        # 1) 故障检测器（VehicleStatus.failure_detector_status）
        for bit, desc in _FAILURE_DETECTOR_BITS:
            if vehicle_status.failure_detector_status & bit:
                failures.append(desc)

        # 2) 系统级解锁检查（VehicleStatus 直接暴露的状态）
        if vehicle_status.safety_button_available and not vehicle_status.safety_off:
            failures.append('安全开关未解锁（需先按下安全按钮）')
        if vehicle_status.usb_connected:
            failures.append('USB 已连接（PX4 禁止带 USB 起飞，需断开并重启飞控）')
        if not vehicle_status.power_input_valid:
            failures.append('电源输入无效')
        if vehicle_status.avoidance_system_required and not vehicle_status.avoidance_system_valid:
            failures.append('避障系统未就绪')
        if vehicle_status.open_drone_id_system_present and not vehicle_status.open_drone_id_system_healthy:
            failures.append('Open Drone ID 系统未就绪')
        if vehicle_status.parachute_system_present and not vehicle_status.parachute_system_healthy:
            failures.append('降落伞系统未就绪')
        if vehicle_status.rc_calibration_in_progress:
            failures.append('遥控器校准进行中')
        if vehicle_status.calibration_enabled:
            failures.append('传感器校准进行中')
        if vehicle_status.is_vtol and vehicle_status.in_transition_mode:
            failures.append('VTOL 处于过渡状态')
        if vehicle_status.is_vtol and vehicle_status.vehicle_type != 1:  # 1 = VEHICLE_TYPE_ROTARY_WING
            failures.append('VTOL 当前不处于多旋翼模式')

        # 3) 模式要求的估计量/链路状态（FailsafeFlags，仅保留对当前模式生效的项）
        failsafe_flags = self.topic_subscriber.get_failsafe_flags()
        if failsafe_flags is not None:
            for flag_field, requirement_field, desc in _MODE_REQUIREMENT_CHECKS:
                if (getattr(failsafe_flags, flag_field)
                        and self._mode_requirement_applies(failsafe_flags, requirement_field, nav_state)):
                    failures.append(desc)

            if self._mode_requirement_applies(failsafe_flags, 'mode_req_prevent_arming', nav_state):
                failures.append('当前模式不允许解锁（请先切换到其他模式）')

            if (failsafe_flags.wind_limit_exceeded or failsafe_flags.flight_time_limit_exceeded) \
                    and self._mode_requirement_applies(failsafe_flags, 'mode_req_wind_and_flight_time_compliance', nav_state):
                failures.append('超出风速或最长飞行时间限制')

            if failsafe_flags.local_velocity_invalid:
                failures.append('本地速度估计无效')
            if failsafe_flags.local_position_accuracy_low:
                failures.append('本地位置估计精度偏低')
            if failsafe_flags.geofence_breached:
                failures.append('已触发地理围栏')
            if failsafe_flags.battery_unhealthy:
                failures.append('电池不健康')

            details['failsafe_flags'] = {
                flag_field: bool(getattr(failsafe_flags, flag_field))
                for flag_field, _, _ in _MODE_REQUIREMENT_CHECKS
            }

        # 4) 电池状态
        battery = self.topic_subscriber.get_battery_status()
        if battery is not None:
            details['battery_warning'] = int(battery.warning)
            details['battery_faults'] = int(battery.faults)
            if battery.warning in _BATTERY_WARNING_TEXT:
                failures.append(f"{_BATTERY_WARNING_TEXT[battery.warning]}（warning={battery.warning}）")
            if not battery.connected:
                failures.append('电池未连接')
            for bit, desc in _BATTERY_FAULT_BITS:
                if battery.faults & bit:
                    failures.append(f'电池故障：{desc}')
            if 0.0 <= battery.remaining < 0.4:
                failures.append(f'电量过低：{battery.remaining * 100.0:.1f}%（低于 40%）')

        # 5) 位置估计质量
        local_position = self.topic_subscriber.get_vehicle_local_position()
        if local_position is not None:
            details['position'] = {
                'xy_valid': bool(local_position.xy_valid),
                'z_valid': bool(local_position.z_valid),
                'v_xy_valid': bool(local_position.v_xy_valid),
                'v_z_valid': bool(local_position.v_z_valid),
                'heading_good_for_control': bool(local_position.heading_good_for_control),
                'dead_reckoning': bool(local_position.dead_reckoning),
                'eph_m': round(float(local_position.eph), 3),
                'epv_m': round(float(local_position.epv), 3),
                'evh_mps': round(float(local_position.evh), 3),
            }
            if local_position.dead_reckoning:
                failures.append('位置估计处于死推算（dead reckoning）状态')
            if not local_position.heading_good_for_control:
                failures.append('航向估计未收敛到可控制精度')

        # 去重并保持顺序
        return list(dict.fromkeys(failures))

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
        
        vehicle_status = self.topic_subscriber.get_vehicle_status()
        if vehicle_status is None:
            result['message'] = '无状态数据'
            return result
            
        # 检查预飞行检查是否通过
        pre_flight_checks = vehicle_status.pre_flight_checks_pass
        result['details']['pre_flight_checks_pass'] = bool(pre_flight_checks)
        
        # 检查系统是否就绪
        system_type = vehicle_status.system_type
        result['details']['system_type'] = system_type
        result['details']['nav_state'] = int(vehicle_status.nav_state)
        result['details']['arming_state'] = int(vehicle_status.arming_state)
        
        if pre_flight_checks:
            result['passed'] = True
            result['message'] = '传感器健康检查通过'
            logger.info(f'check_sensor_health: {result} ')
            return result

        # 预飞行检查未通过：还原具体失败原因
        failures = self._collect_preflight_failures(vehicle_status, result['details'])
        result['details']['failures'] = failures

        if failures:
            result['message'] = '预飞行检查未通过：' + '；'.join(failures)
        else:
            result['message'] = (
                '预飞行检查未通过，且未从已订阅话题中还原出具体原因'
                '（可能为传感器校准/健康、EKF 新息检验等 PX4 未通过 DDS 发布的检查项），'
                '请查看 PX4 控制台中的 “Preflight Fail” 日志'
            )

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
        min_battery = 40.0
        
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
    
    def check_report(self) -> Tuple[bool, list]:
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
            missing = [
                label for label, getter in (
                    ('vehicle_status(/fmu/out/vehicle_status)', self.topic_subscriber.get_vehicle_status),
                    ('battery_status(/fmu/out/battery_status)', self.topic_subscriber.get_battery_status),
                    ('failsafe_flags(/fmu/out/failsafe_flags)', self.topic_subscriber.get_failsafe_flags),
                ) if getter() is None
            ]
            message = '30s 内未收到 PX4 状态数据：' + ('、'.join(missing) if missing else '未知')
            logger.error(f'✗ 数据等待超时：{message}')
            # 超时也必须保持 (passed, report) 二元组契约，否则调用方解包抛 TypeError 会掩盖真实原因
            self.check_results['data'] = {'passed': False, 'message': message, 'details': {}}
            return False, [{'key': 'data', 'label': '数据等待', 'value': message, 'unit': ''}]
            
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
