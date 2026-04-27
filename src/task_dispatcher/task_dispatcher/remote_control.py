#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import logging
import time
import platform
import rclpy
import requests
from rclpy.node import Node
from robot_interface.srv import Move, SetCtrlMode
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
from task_dispatcher.ptz_control import PtzControl
from .config import ROBOT_TYPE, FFR_SERVER_URL
from uav_command_sender import UavCommandSender
from uav_mission_manager import UavMissionManager
from dock_control_client import DockControlClient
# 配置日志
logger = logging.getLogger('task_dispatcher.remote_control')

class RemoteControl:
    """
    远程控制类，负责处理RemoteControl命令并调用相应的ROS服务
    """
    
    def __init__(self, node, mqtt_client, response_topic, ros_topic_subscriber, ptz_controller):
        """
        初始化远程控制类
        
        Args:
            node: ROS2节点实例
            mqtt_client: MQTT客户端实例
            response_topic: MQTT响应主题
            ros_topic_subscriber: 机器人状态订阅器实例
            ptz_controller: PTZ控制器实例
        """
        self.node = node
        self.mqtt_client = mqtt_client
        self.mqtt_response_topic = response_topic
        self.ros_topic_subscriber: Ros2TopicSubscriber = ros_topic_subscriber
        self.uav_mission_manager = UavMissionManager(node, ros_topic_subscriber)
        self.uav_command_sender = UavCommandSender(node, topic_name='/uav_command')
        # 机舱控制
        self.dock_control_client = DockControlClient(node)
        
        
        # 速度映射表 (low:0.1, mid:0.25, high:0.5)
        if ROBOT_TYPE == 'ffr':
            self.speed_map = {
            'low': 0.1,
            'mid': 0.25,
            'high': 0.5
            }
        else:
            self.speed_map = {
            'low': 0.2,
            'mid': 0.5,
            'high': 0.8
            }
        # 方向到角度的映射表
        self.direction_angle_map = {
            'up': 0.0,              # 向前
            'down': 0.0,            # 向后
            #'left': -32.0,          # 向左
            #'right': 32.0,          # 向右
            'leftUp': 32.0,        # 左前
            'rightUp': -32.0,        # 右前
            'leftDown': 32.0,      # 左后
            'rightDown': -32.0       # 右后
        }
        
        # 需要移动的动作
        self.move_actions = ['walk', 'face']
        
        # 初始化PTZ控制器
        self.ptz_controller: PtzControl = ptz_controller
        
        logger.info('远程控制类初始化完成')
    
    def process_remote_control(self, message_data):
        """
        处理RemoteControl远程控制命令
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
                return
            
            # 检查必要字段
            if 'actName' not in body:
                logger.error('RemoteControl消息缺少必要字段: actName')
                self.send_ack_response(message_data, 400, 'missing actName')
                return
            
            act_name = body['actName']
            action_id = body.get('actionId', '')  # actionId非必填，提供默认值
            
            # 对于非stop动作，需要检查argv是否存在
            if act_name != 'stop' and 'argv' not in body:
                logger.error('RemoteControl消息缺少必要字段: argv')
                self.send_ack_response(message_data, 400, 'missing argv')
                return
            
            # 获取argv，对于stop动作提供默认空字典
            argv = body.get('argv', {})
            
            logger.info(f'收到RemoteControl命令: actName={act_name}, actionId={action_id}, argv={argv}')
            
            # 验证actName是否有效
            valid_act_names = ['stay', 'face', 'stand', 'walk', 'jump', 'stop', 
                               'record', 'snap', 'algo', 'forceStop', 'ptz',
                               'drive', 'charge', 'takeoff', 'land', 'gpio',
                               'cover_open', 'cover_close', 'putter_close', 'putter_open', 
                               'drone_open', 'drone_close', 'charge_open', 'charge_close',
                               'battery_activation']
            
            if act_name not in valid_act_names:
                logger.error(f'无效的actName: {act_name}')
                self.send_ack_response(message_data, 400, f'invalid actName: {act_name}')
                return
            
            # 根据不同的actName进行参数验证
            if not self.validate_argv(act_name, argv):
                logger.error(f'{act_name}参数验证失败: {argv}')
                self.send_ack_response(message_data, 400, f'invalid argv for {act_name}')
                return
            
            if ROBOT_TYPE == 'ffr':
                # 调用FFR机器人服务
                self.call_ffr_service(act_name, action_id, argv)
            else:
                # 调用对应的ROS服务
                self.call_robot_control_service(act_name, action_id, argv)  
            
            # 发送确认响应
            self.send_ack_response(message_data, 200, 'ok')
            
        except Exception as e:
            logger.error(f'处理RemoteControl命令时出错: {str(e)}')
            self.send_ack_response(message_data, 500, str(e))
    
    def validate_argv(self, act_name, argv):
        """
        根据不同的actName验证argv参数
        """
        try:
            if act_name == 'walk':
                # walk参数验证
                return isinstance(argv, dict) and \
                       'speed' in argv and argv['speed'] in ['low', 'mid', 'high'] and \
                       'direction' in argv and argv['direction'] in ['forward', 'back', 'left', 'right', 'leftRotation', 'rightRotation']
            
            elif act_name == 'stand':
                # stand参数验证
                return isinstance(argv, dict) and \
                       'mode' in argv and argv['mode'] in [0, 1, 2]
            
            elif act_name == 'stop':
                # stop参数验证，force是可选的
                if not isinstance(argv, dict):
                    return False
                if 'force' in argv and not isinstance(argv['force'], bool):
                    return False
                return True
            
            elif act_name == 'ptz':
                # ptz参数验证
                if not isinstance(argv, dict):
                    return False
                if 'cameraId' not in argv:
                    logger.error('PTZ控制缺少必要参数: cameraId')
                    return False
                if 'ctrlType' not in argv:
                    logger.error('PTZ控制缺少必要参数: ctrlType')
                    return False
                if 'ctrlParams' not in argv:
                    logger.error('PTZ控制缺少必要参数: ctrlParams')
                    return False
                
                # 使用PTZ控制器验证参数
                return self.ptz_controller.validate_ptz_params(argv['ctrlType'], argv['ctrlParams'])
            
            # 其他actName的参数验证可以在这里添加
            # 目前默认其他类型的参数只要是object格式就通过
            return isinstance(argv, dict)
            
        except Exception as e:
            logger.error(f'验证argv参数时出错: {str(e)}')
            return False
    
    def send_ack_response(self, original_message, status, error_msg):
        """
        发送Ack响应到MQTT
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
            
            # 发布到初始化时设置的响应主题
            self.mqtt_client.publish(self.mqtt_response_topic, json.dumps(response))
            logger.info(f'已发送Ack响应: response={response}')
            
        except Exception as e:
            logger.error(f'发送Ack响应失败: {str(e)}')
            
    
    def wait_for_stand_state(self, timeout=10.0):
        """
        等待机器人进入站立状态
        返回是否在超时时间内成功站立
        """
        start_time = time.time()
        logger.info(f'等待机器人站立状态，超时时间: {timeout}秒')
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            current_state = self.ros_topic_subscriber.get_current_ctrl_state()
            if current_state == 2:  # STAND_UP = 2
                logger.info('机器人已进入站立状态')
                return True
            time.sleep(0.5)
            # 只在x86架构上执行模拟机器人状态更新
            if 'x86' in platform.machine():
                logger.info(f'模拟机器人状态检查中')
                time.sleep(0.5)
                current_state = 2
                if current_state == 2:  # STAND_UP = 2
                    logger.info('机器人已进入站立状态')
                    return True
        
        logger.warning(f'等待站立状态超时')
        return False
    
    def call_robot_control_service(self, act_name, action_id, argv):
        """
        调用机器人控制服务
        根据actName调用不同的ROS服务来控制机器人动作
        """
        try:
            logger.info(f'准备调用机器人控制服务: actName={act_name}, actionId={action_id}, argv={argv}')
            
            # 如果是移动相关动作，检查机器人状态
            current_state = self.ros_topic_subscriber.get_current_ctrl_state()
            if act_name in self.move_actions and current_state is not None:
                # 阻尼状态(PASSIVE=0)、趴下状态(LIE_DOWN=1)、站低状态(STAND_LOW=4)
                if current_state in [0, 1, 4]:
                    logger.info(f'机器人当前状态: {current_state}，需要先站立')
                    # 调用站立服务
                    self._call_set_ctrl_mode_service('stand', 1)
                    # 等待站立状态，超时10秒
                    if not self.wait_for_stand_state(timeout=10.0):
                        logger.error('机器人站立失败，取消移动操作')
                        return
                # 站立状态(STAND_UP=2)或移动状态(MOVING=3)，可以直接控制
                elif current_state in [2, 3]:
                    logger.info(f'机器人当前状态: {current_state}，可以直接控制')
                # 其他状态（如动作状态ACTION=5），不进行控制
                else:
                    logger.warning(f'机器人当前状态: {current_state}，不适合进行移动控制')
                    return
            
            # 根据actName调用不同的服务
            if act_name in ['walk', 'stop']:
                # 处理行走和停止动作
                self._call_move_service(act_name, argv, self.speed_map)
            elif act_name in ['stand']:
                # 处理站立和趴下动作
                # 0-趴下、1-四足直站、2-蹲站;
                mode = argv.get('mode', 1)
                self._call_set_ctrl_mode_service(act_name, mode)
            elif act_name == 'face':
                # 处理朝向动作（使用旋转）
                direction = argv.get('direction', '')
                speed = argv.get('speed', 'low')
                if direction == 'left':
                    self._call_move_service('leftRotation', {'speed': speed}, self.speed_map)
                elif direction == 'right':
                    self._call_move_service('rightRotation', {'speed': speed}, self.speed_map)
            elif act_name == 'stay':
                # 停留动作（设置为停止）
                self._call_move_service('stop', {}, self.speed_map)
            elif act_name == 'forceStop':
                # 强制停止所有动作
                self._call_move_service('stop', {}, self.speed_map)
            elif act_name == 'ptz':
                # 处理PTZ控制动作
                camera_id = argv.get('cameraId', '')
                ctrl_type = argv.get('ctrlType', '')
                ctrl_params = argv.get('ctrlParams', {})
                self.ptz_controller.execute_ptz_control(camera_id, ctrl_type, ctrl_params)
            elif act_name == 'takeoff':
                height = argv.get('takeoffHeight', 0.5)
                self._call_uav_takeoff(act_name, argv, height)
            elif act_name == 'land':
                self._call_uav_land(act_name, argv)
                
                # test
                # self._call_uav_gpio(act_name, argv, 1)
            elif act_name == 'gpio':
                level = argv.get('level', 0)
                self._call_uav_gpio(act_name, argv, level)
            # elif act_name in ['cover_open', 'cover_close', 'putter_close', 'putter_open', 
            #                   'drone_open', 'drone_close', 'charge_open', 'charge_close', 
            #                   'battery_activation']:
            #     self.dock_control_client.call_uav_hangar_control_service(act_name)
            else:
                logger.warning(f'未实现的动作类型: {act_name}')
            
        except Exception as e:
            logger.error(f'调用机器人控制服务失败: {str(e)}')
    
    def _call_uav_takeoff(self, act_name, argv, height):
        """处理起飞"""
        logger.info(f'收到{act_name}命令')
        if self.uav_command_sender.send_command(f'TAKEOFF:{height}'):
            logger.info('起飞成功')
        else:
            logger.error('起飞失败')
        
        # self.uav_command_sender.shutdown()
    def _call_uav_land(self, act_name, argv):
        """处理降落"""
        logger.info(f'收到{act_name}命令')
        if self.uav_command_sender.send_command('LAND'):
            logger.info('降落成功')
        else:
            logger.error('降落失败')
        
        # self.uav_command_sender.shutdown()
    def _call_uav_gpio(self, act_name, argv, level):
        """处理补光灯"""
        logger.info(f'收到{act_name}命令')
        if self.uav_command_sender.send_command(f'GPIO:{level}'):
            logger.info('补光灯设置成功')
        else:
            logger.error('补光灯设置失败')
                
    def _call_move_service(self, act_name, argv, speed_map):
        """
        调用移动服务
        """
        # 创建服务客户端
        move_client = self.node.create_client(Move, '/rkbot/move')
        
        # 等待服务可用
        if not move_client.wait_for_service(timeout_sec=3.0):
            logger.error('移动服务不可用')
            return
        
        # 准备请求
        request = Move.Request()
        
        # 根据动作类型设置参数
        if act_name == 'walk':
            direction = argv.get('direction', 'forward')
            speed = argv.get('speed', 'mid')
            speed_value = speed_map.get(speed, 0.5)
            
            if direction == 'forward':
                request.move.vx = speed_value
            elif direction == 'back':
                request.move.vx = -speed_value
            elif direction == 'left':
                request.move.vy = speed_value
            elif direction == 'right':
                request.move.vy = -speed_value
            elif direction == 'leftRotation':
                request.move.yaw_rate = speed_value
            elif direction == 'rightRotation':
                request.move.yaw_rate = -speed_value
        elif act_name == 'stop':
            request.move.vx = 0.0
            request.move.vy = 0.0
            request.move.yaw_rate = 0.0
        elif act_name in ['leftRotation', 'rightRotation']:
            speed = argv.get('speed', 'low')
            speed_value = speed_map.get(speed, 0.1)
            if act_name == 'leftRotation':
                request.move.yaw_rate = speed_value
            else:
                request.move.yaw_rate = -speed_value
        
        logger.info(f'调用移动服务: act_name={act_name}, vx={request.move.vx}, vy={request.move.vy}, yaw_rate={request.move.yaw_rate}')
        
        # 异步调用服务
        future = move_client.call_async(request)
        future.add_done_callback(lambda f: self._move_service_callback(f, act_name))
    
    def _move_service_callback(self, future, act_name):
        """
        处理移动服务的异步响应
        """
        try:
            response = future.result()
            logger.info(f'移动服务调用成功: act_name={act_name}, result={response}')
        except Exception as e:
            logger.error(f'移动服务调用异常: act_name={act_name}, {str(e)}')
    
    def _call_set_ctrl_mode_service(self, act_name, mode=1):
        """
        调用设置控制模式服务
        """
        # 创建服务客户端
        mode_client = self.node.create_client(SetCtrlMode, '/rkbot/setctrlmode')
        
        # 等待服务可用
        if not mode_client.wait_for_service(timeout_sec=3.0):
            logger.error('控制模式服务不可用')
            return
        
        # 准备请求
        request = SetCtrlMode.Request()
        
        # 设置模式值, 0-趴下、1-四足直站、2-蹲站;
        request.mode.value = mode
        
        logger.info(f'调用控制模式服务: mode_value={request.mode.value}')
        
        # 异步调用服务
        future = mode_client.call_async(request)
        future.add_done_callback(lambda f: self._mode_service_callback(f, act_name))
    
    def _mode_service_callback(self, future, act_name):
        """
        处理控制模式服务的异步响应
        """
        try:
            response = future.result()
            logger.info(f'控制模式服务调用成功: act_name={act_name}, result={response}')
        except Exception as e:
            logger.error(f'控制模式服务调用异常: act_name={act_name}, {str(e)}')

    def call_ffr_service(self, act_name, action_id, argv):
        """
        调用FFR机器人服务
        """

        logger.info(f'调用FFR机器人服务: act_name={act_name}, action_id={action_id}, argv={argv}')
        
        if act_name == 'drive':
            # 调用移动服务
            self._call_drive_service(argv)
        elif act_name == 'stop':
            # 调用停止服务
            self._call_stop_service(argv)
        elif act_name == 'charge':
            # 调用充电服务
            self._call_charge_service(argv)
        else:
            logger.warning(f'FFR机器人不支持{act_name}动作')


    def _call_drive_service(self, argv):
        """
        调用移动服务
        """
        direction = argv.get('direction', 'up')
        speed = argv.get('speed', 'mid')
        
        # 映射方向到角度
        angle = self.direction_angle_map.get(direction, 0.0)
        
        # 映射速度
        speed_value = self.speed_map.get(speed, 0.25)
        
        # 确定速度的正负（向前为正，向后为负）
        if direction in ['down', 'leftDown', 'rightDown']:
            speed_value = -speed_value
        
        logger.info(f'移动参数: direction={direction}, angle={angle}, speed={speed}, speed_value={speed_value}')
        
        # 调用FFR机器人的移动接口
        try:
            url = f'{FFR_SERVER_URL}/fireRobot/move'
            payload = {
                'angle': angle,
                'speed': speed_value
            }
            headers = {
                'Content-Type': 'application/json',
                'accept': '*/*'
            }
            
            response = requests.post(url, json=payload, headers=headers, timeout=1.5)
            response.raise_for_status()  # 检查请求是否成功
            
            result = response.json()
            logger.info(f'移动服务调用成功: {result}')
            
        except requests.RequestException as e:
            logger.error(f'移动服务调用失败: {str(e)}')
        except Exception as e:
            logger.error(f'移动服务处理异常: {str(e)}')
    

    def _call_stop_service(self, argv):
        """
        调用停止服务
        """
        # 调用FFR机器人的停止接口
        try:
            url = f'{FFR_SERVER_URL}/fireRobot/stopMove'
            headers = {
                'accept': '*/*'
            }
            
            response = requests.post(url, data='', headers=headers, timeout=1.5)
            response.raise_for_status()  # 检查请求是否成功
            
            result = response.json()
            logger.info(f'停止服务调用成功: {result}')
            
        except requests.RequestException as e:
            logger.error(f'停止服务调用失败: {str(e)}')
        except Exception as e:
            logger.error(f'停止服务处理异常: {str(e)}')
    

    def _call_charge_service(self, argv):
        """
        调用充电服务
        """
        # 调用FFR机器人的充电接口
        try:
            url = f'{FFR_SERVER_URL}/fireRobot/recharge'
            headers = {
                'accept': '*/*'
            }
            
            response = requests.post(url, data='', headers=headers, timeout=1.5)
            response.raise_for_status()  # 检查请求是否成功
            
            result = response.json()
            logger.info(f'充电服务调用成功: {result}')
            
        except requests.RequestException as e:
            logger.error(f'充电服务调用失败: {str(e)}')
        except Exception as e:
            logger.error(f'充电服务处理异常: {str(e)}')