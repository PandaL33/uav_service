#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
import logging
from threading import Thread
import time
# 导入String消息类型
from std_msgs.msg import String
# 导入设备注册保活模块
from task_dispatcher.registration import Registration
# 导入机器人状态订阅处理类
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
# 导入告警上报器
from task_dispatcher.alarm_reporter import AlarmReporter
# 导入远程控制类
from task_dispatcher.remote_control import RemoteControl
# 导入巡检任务管理器
from task_dispatcher.cruise_task_manager import CruiseTaskManager
from task_dispatcher.cruise_uav_task_manager import CruiseUavTaskManager
# 导入视频预览管理器
from task_dispatcher.video_preview_manager import VideoPreviewManager
# 导入消息缓存管理器
from task_dispatcher.message_cache import message_cache_manager
# 导入PTZ控制器
from task_dispatcher.ptz_control import PtzControl
# 导入服务管理器
from task_dispatcher.service_manager import handle_service_list_request, handle_service_control_request
# 导入初始化定位管理器
from task_dispatcher.initial_pose_manager import InitialPoseManager
# 导入执行动作管理器
from task_dispatcher.perform_action_manager import PerformActionManager
# 导入地图管理器
from task_dispatcher.map_manager import MapManager
# 导入设备控制类
from task_dispatcher.device_control import DeviceControl
# 导入远程指令下发类
from task_dispatcher.command_deliver import CommandDeliver
# uav设备故障信息检查类
from task_dispatcher.preflight_check_node import PreFlightCheckNode

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(name)s] [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger('task_dispatcher')

class TaskDispatcherNode(Node):
    """
    任务派发节点，负责订阅MQTT消息并调用下游ROS服务
    """
    
    def __init__(self):
        super().__init__('task_dispatcher_node')
        
        # MQTT相关参数
        self.declare_parameter('mqtt_broker', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_username', '')
        self.declare_parameter('mqtt_password', '')
        self.declare_parameter('mqtt_topic', '/UAV/UAV00001/dn')
        self.declare_parameter('mqtt_response_topic', '/robot/up')
        self.declare_parameter('mqtt_client_id', 'task_dispatcher_86_server')
        self.declare_parameter('robot_sn', 'UAV00001')
        self.declare_parameter('server_url', 'https://192.168.200.131:8075')
        self.declare_parameter('version', '1.0')
        
        # 获取参数
        self.mqtt_broker = self.get_parameter('mqtt_broker').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.mqtt_username = self.get_parameter('mqtt_username').get_parameter_value().string_value
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value
        self.mqtt_topic = self.get_parameter('mqtt_topic').get_parameter_value().string_value
        self.mqtt_response_topic = self.get_parameter('mqtt_response_topic').get_parameter_value().string_value
        self.mqtt_client_id = self.get_parameter('mqtt_client_id').get_parameter_value().string_value
        self.robot_sn = self.get_parameter('robot_sn').get_parameter_value().string_value
        self.server_url = self.get_parameter('server_url').get_parameter_value().string_value
        self.version = self.get_parameter('version').get_parameter_value().string_value
        
        # 初始化MQTT客户端
        logger.info(f'初始化MQTT客户端，客户端ID: {self.mqtt_client_id}')
        self.mqtt_client = mqtt.Client(client_id=self.mqtt_client_id)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # 设置MQTT连接参数
        if self.mqtt_username and self.mqtt_password:
            self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)

        # 初始化视频预览管理器
        self.video_preview_manager = VideoPreviewManager(
            node=self,
            mqtt_client=self.mqtt_client,
            response_topic=self.mqtt_response_topic
        )

        # 初始化Ros2TopicSubscriber，用于订阅机器人状态和告警信息
        self.ros_topic_subscriber = Ros2TopicSubscriber(
            self,
            mqtt_client=self.mqtt_client,
            enable_status_report=True,
            robot_sn=self.robot_sn,
            mqtt_response_topic=self.mqtt_response_topic,
            version=self.version,
            video_preview_manager=self.video_preview_manager
        )

        # 初始化告警上报器
        self.alarm_reporter = AlarmReporter(
            mqtt_client=self.mqtt_client,
            server_url=self.server_url,
            mqtt_alarm_topic=self.mqtt_response_topic,
            device_sn=self.robot_sn,
            version=self.version
        )
        # 设置告警处理回调
        self.ros_topic_subscriber.set_alarm_callback(self.alarm_reporter.process_alarm_message)

        # 初始化PTZ控制器
        self.ptz_controller = PtzControl(node=self)
        
        # 初始化定位管理器
        self.initial_pose_manager = InitialPoseManager(
            mqtt_client=self.mqtt_client,
            response_topic=self.mqtt_response_topic,
            node=self
        )

        # 初始化执行动作管理器
        self.perform_action_manager = PerformActionManager(
            node=self,
            ptz_controller=self.ptz_controller,
            topic_subscriber=self.ros_topic_subscriber
        )

        # 初始化远程控制类
        self.remote_control = RemoteControl(
            node=self,
            mqtt_client=self.mqtt_client,
            response_topic=self.mqtt_response_topic,
            ros_topic_subscriber=self.ros_topic_subscriber,
            ptz_controller=self.ptz_controller
        )

        # 初始化地图管理器
        self.map_manager = MapManager(
            node=self,
            mqtt_client=self.mqtt_client,
            mqtt_response_topic=self.mqtt_response_topic,
            ros_topic_subscriber=self.ros_topic_subscriber,
            server_url=self.server_url, 
            robot_sn=self.robot_sn, 
            version=self.version,
            perform_action_manager=self.perform_action_manager
        )
        # self.ros_topic_subscriber.set_map_callback(self.map_manager.handle_map_data)
        
        # 初始化设备控制类
        self.device_control = DeviceControl(
            node=self,
            mqtt_client=self.mqtt_client,
            response_topic=self.mqtt_response_topic
        )
        
        # 初始化远程指令下发类
        self.command_deliver = CommandDeliver(
            node=self,
            mqtt_client=self.mqtt_client,
            response_topic=self.mqtt_response_topic
        )
        
        # 初始化巡检任务管理器
        self.cruise_uav_task_manager = CruiseUavTaskManager(
            node=self,
            mqtt_client=self.mqtt_client,
            response_topic=self.mqtt_response_topic,
            ros_topic_subscriber=self.ros_topic_subscriber,
            version=self.version,
            robot_sn=self.robot_sn,
            ptz_controller=self.ptz_controller,
            server_url=self.server_url,
            perform_action_manager=self.perform_action_manager,
            map_manager=self.map_manager
        )
        
        logger.info(f'已初始化远程控制节点，MQTT主题: {self.mqtt_topic}')
        
        # 启动MQTT连接线程
        self.mqtt_connected = False
        self.mqtt_thread = Thread(target=self.connect_mqtt, daemon=True)
        self.mqtt_thread.start()
        
        # 启动心跳定时器
        self.create_timer(1.0, self.check_mqtt_connection)
        self.create_timer(60, self.check_ros_service_availability)
        
        # 初始化设备注册保活模块
        self.registration_manager = Registration(
            mqtt_client=self.mqtt_client,
            response_topic=self.mqtt_response_topic,
            node=self,
            version=self.version
        )
        
        # 设置设备配置信息
        self.registration_manager.set_config({
            'sn': self.robot_sn,  # 使用客户端ID作为SN
            'username': "ropeokRobot",
            'password': "pwRopeokRobot32145",
            'model': 'D1Ultra',
            'softwareVer': 'v1.0',
            'hardwareVer': 'v1.0'
        })
        
        # 将机器人状态订阅器传递给注册保活模块
        # 这样心跳包就可以获取实时的机器人状态数据
        self.registration_manager.set_status_provider(self.ros_topic_subscriber)
        
        # 启动注册保活服务
        self.registration_manager.start()
        
        # 定时检查无人机设备故障信息，并上报
        self.uav_fault_inspection = True
        self.preflight_check_node = PreFlightCheckNode(
            node=self,
            topic_subscriber=self.ros_topic_subscriber,
        )
        #self.create_timer(300, self.check_uav_fault)
        
        logger.info('任务派发节点已启动')
    
    def connect_mqtt(self):
        """
        连接到MQTT代理
        """
        while rclpy.ok():
            try:
                if not self.mqtt_connected:
                    logger.info(f'连接到MQTT代理: {self.mqtt_broker}:{self.mqtt_port}')
                    self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
                    self.mqtt_client.loop_start()
                    self.mqtt_connected = True
                time.sleep(1)
            except Exception as e:
                logger.error(f'MQTT连接错误: {str(e)}')
                self.mqtt_connected = False
                time.sleep(5)  # 重连前等待5秒
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """
        MQTT连接回调
        """
        if rc == 0:
            logger.info('已成功连接到MQTT代理')
            self.mqtt_client.subscribe(self.mqtt_topic)
            logger.info(f'已订阅主题: {self.mqtt_topic}')
        else:
            logger.error(f'MQTT连接失败，返回码: {rc}')
            self.mqtt_connected = False
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """
        MQTT断开连接回调
        """
        logger.warning(f'MQTT断开连接，返回码: {rc}')
        self.mqtt_connected = False
    
    def on_mqtt_message(self, client, userdata, msg):
        """
        MQTT消息回调
        """
        try:
            logger.info(f'收到MQTT消息: 主题={msg.topic}, 载荷={msg.payload.decode()}')
            
            # 解析MQTT消息
            message_data = json.loads(msg.payload.decode())
            
            # 处理注册和心跳的响应（不区分大小写）
            cmd = message_data.get('cmd', '').upper()
            if cmd == 'ACK':
                # 获取ACK中的seq
                seq = message_data.get('seq')
                if seq:
                    # 从缓存中查找原始上报消息
                    original_message = message_cache_manager.get_message_by_seq(seq)
                    if original_message:
                        original_cmd = original_message.get('cmd', '')
                        logger.debug(f'收到ACK响应，已找到原始消息: seq={seq}, cmd={original_cmd}')
                        # 根据原始cmd做相应处理
                        if original_cmd == 'Register':
                            self.registration_manager.handle_register_response(message_data)
                            logger.info(f'收到注册响应，已处理: seq={seq}')
                        elif original_cmd == 'KeepAlive':
                            self.registration_manager.handle_keepalive_response(message_data)
                            logger.info(f'收到心跳响应，已处理: seq={seq}')
                        elif original_cmd == 'ActionReport':
                            # ActionReport的ACK处理逻辑
                            logger.info(f'收到任务状态上报响应，已处理: seq={seq}')
                        elif original_cmd == 'AlarmNotify':
                            # AlarmNotify的ACK处理逻辑
                            logger.info(f'收到告警通知响应，已处理: seq={seq}')
                        elif original_cmd == 'StatusReport':
                            # StatusReport的ACK处理逻辑
                            logger.info(f'收到状态报告响应，已处理: seq={seq}')
                        elif original_cmd == 'BuildMap':
                            # BuildMap的ACK处理逻辑
                            logger.info(f'收到地图构建响应，已处理: seq={seq}')
                        else:
                            logger.warning(f'收到未知类型的ACK响应: seq={seq}, original_cmd={original_cmd}')
                    else:
                        logger.warning(f'收到ACK响应，但未找到对应的原始消息: seq={seq}')
                else:
                    logger.warning('收到ACK响应，但未包含seq字段')
            else:
                # 处理其他任务消息
                self.process_task_message(message_data)
            
        except json.JSONDecodeError as e:
            logger.error(f'解析MQTT消息失败: {str(e)}')
        except Exception as e:
            logger.error(f'处理MQTT消息时出错: {str(e)}')
    
    def process_task_message(self, message_data):
        """
        处理任务消息，调用相应的服务处理不同类型的命令
        根据命令码规范，命令不区分大小写
        """
        try:
            # 获取命令码（不区分大小写）
            cmd = message_data.get('cmd', '').upper()
            # 标准化命令码为大写格式
            message_data['cmd'] = cmd
            
            # 定义命令处理函数字典
            command_handlers = {
                #'REGISTER': lambda: self._handle_register(message_data),
                #'KEEPALIVE': lambda: self._handle_keepalive(message_data),
                #'STATUSREPORT': lambda: self._handle_status_report(message_data),
                #'SETCONFIG': lambda: self._handle_set_config(message_data),
                #'GETCONFIG': lambda: self._handle_get_config(message_data),
                #'GETSTATUS': lambda: self._handle_get_status(message_data),
                'PLAYVIDEO': lambda: self._handle_process_play_video(message_data, '播放视频'),
                'STOPVIDEO': lambda: self._handle_process_stop_video(message_data, '停止视频'),
                'REMOTECONTROL': lambda: self._handle_remote_control(message_data, '远程控制'),
                'BUILDMAP': lambda: self._handle_build_map(message_data, '启动地图构建'),
                'STOPBUILDMAP': lambda: self._handle_stop_build_map(message_data, '停止地图构建'),
                'SETCRUISETASK': lambda: self._handle_cruise_task(message_data, '设置并启动巡检任务'),
                'GETCRUISETASK': lambda: self._handle_cruise_task(message_data, '获取巡检任务'),
                'EXECRUISetask': lambda: self._handle_cruise_task(message_data, '启动巡检任务'),
                'STOPCRUISETASK': lambda: self._handle_cruise_task(message_data, '停止巡检任务'),
                #'ALARMNOTIFY': lambda: self._handle_alarm_notify(message_data)
                'GETPTZINFO': lambda: self._handle_get_ptz_info(message_data, '获取PTZ信息'),
                'SERVICELIST': lambda: self._handle_get_service_list(message_data, '获取服务列表'),
                'SERVICECONTROL': lambda: self._handle_service_control(message_data, '服务控制'),
                'INITIALPOSE': lambda: self._handle_initial_pose(message_data, '初始化定位信息'),
                'DEVICECONTROL': lambda: self._handle_device_control(message_data, '设备控制'),
                'COMMANDDELIVER': lambda: self._handle_command_deliver(message_data, '远程指令下发'),
            }
            
            # 根据命令码调用相应的处理函数
            if cmd in command_handlers:
                command_handlers[cmd]()
            else:
                # 未知命令
                logger.warning(f'收到未知命令: {cmd}')
                self.send_ack_response(message_data, 400, '未知命令', {})
                
        except json.JSONDecodeError as e:
            logger.error(f'解析消息失败: {str(e)}')
            self.send_ack_response(message_data, 400, f'解析错误: {str(e)}', {})
        except Exception as e:
            logger.error(f'处理任务消息时出错: {str(e)}')
            self.send_ack_response(message_data, 500, f'处理错误: {str(e)}', {})
    
    def _handle_register(self, message_data):
        """处理注册命令"""
        logger.info('收到注册命令')
        #self.registration_manager.handle_register_request(message_data)
    
    def _handle_keepalive(self, message_data):
        """处理心跳上报命令"""
        logger.info('收到心跳上报命令')
        #self.registration_manager.handle_keepalive_request(message_data)
    
    def _handle_status_report(self, message_data):
        """处理状态上报命令"""
        logger.info('收到状态上报命令')
        # 状态上报通常是从设备到服务器，这里可能需要根据实际需求调整
        #self.send_ack_response(message_data, 200, 'ok', {})
    
    def _handle_set_config(self, message_data):
        """处理参数配置命令"""
        logger.info('收到参数配置命令')
        #if hasattr(self.registration_manager, 'handle_set_config'):
        #    self.registration_manager.handle_set_config(message_data)
        #else:
        #    self.send_ack_response(message_data, 501, '功能未实现', {})
    
    def _handle_get_config(self, message_data):
        """处理获取参数配置命令"""
        logger.info('收到获取参数配置命令')
        #if hasattr(self.registration_manager, 'handle_get_config'):
        #    self.registration_manager.handle_get_config(message_data)
        #else:
        #    self.send_ack_response(message_data, 501, '功能未实现', {})
    
    def _handle_get_status(self, message_data):
        """处理获取状态命令"""
        logger.info('收到获取状态命令')
        #if hasattr(self.ros_topic_subscriber, 'get_current_status'):
        #    status = self.ros_topic_subscriber.get_current_status()
        #    self.send_ack_response(message_data, 200, 'ok', {'status': status})
        #else:
        #    self.send_ack_response(message_data, 501, '功能未实现', {})
    
    def _handle_not_implemented(self, message_data, command_name):
        """处理未实现的功能"""
        logger.info(f'收到{command_name}命令')
        #self.send_ack_response(message_data, 501, '功能未实现', {})

    def _handle_build_map(self, message_data, command_name):
        """处理构建地图命令"""
        logger.info(f'收到{command_name}命令')
        try:
            # 调用地图管理器的build_map方法
            self.map_manager.build_map(message_data)
        except Exception as e:
            logger.error(f'处理{command_name}命令时出错: {str(e)}')
            self.send_ack_response(message_data, 500, f'处理错误: {str(e)}', {})

    def _handle_stop_build_map(self, message_data, command_name):
        """处理停止构建地图命令"""
        logger.info(f'收到{command_name}命令')
        try:
            # 调用地图管理器的stop_build_map方法
            self.map_manager.stop_build_map(message_data)
        except Exception as e:
            logger.error(f'处理{command_name}命令时出错: {str(e)}')
            self.send_ack_response(message_data, 500, f'处理错误: {str(e)}', {})
    
    def _handle_process_play_video(self, message_data, command_name):
        """处理播放视频命令"""
        logger.info(f'收到{command_name}命令')
        self.video_preview_manager.process_play_video(message_data)
    
    def _handle_process_stop_video(self, message_data, command_name):
        """处理停止视频命令"""
        logger.info(f'收到{command_name}命令')
        self.video_preview_manager.process_stop_video(message_data)

    def _handle_remote_control(self, message_data, command_name):
        """处理远程控制命令"""
        logger.info(f'收到{command_name}命令')
        self.remote_control.process_remote_control(message_data)
    
    def _handle_cruise_task(self, message_data, command_name):
        """处理巡检任务相关命令"""
        logger.info(f'收到{command_name}命令')
        self.cruise_uav_task_manager.process_cruise_task(message_data)
    
    def _handle_alarm_notify(self, message_data, command_name):
        """处理告警上报命令"""
        logger.info(f'收到{command_name}命令')
        #if hasattr(self.alarm_reporter, 'process_alarm_notify'):
        #    self.alarm_reporter.process_alarm_notify(message_data)
        #else:
        #    self.send_ack_response(message_data, 200, 'ok', {})

    def _handle_get_ptz_info(self, message_data, command_name):
        """处理获取PTZ信息命令"""
        logger.info(f'收到{command_name}命令')
        ptz_info = self.ptz_controller.get_ptz_info(message_data)
        self.send_ack_response(message_data, 200, 'ok', ptz_info)
    
    def _handle_device_control(self, message_data, command_name):
        """处理设备控制命令"""
        logger.info(f'收到{command_name}命令')
        self.device_control.process_device_control(message_data)
    
    def _handle_command_deliver(self, message_data, command_name):
        """处理远程指令下发命令"""
        logger.info(f'收到{command_name}命令')
        self.command_deliver.process_command_deliver(message_data)

    def _handle_get_service_list(self, message_data, command_name):
        """处理获取服务列表命令"""
        logger.info(f'收到{command_name}命令')
        service_list = handle_service_list_request()
        self.send_ack_response(message_data, 200, 'ok', service_list)
    
    def _handle_service_control(self, message_data, command_name):
        """处理服务控制命令"""
        logger.info(f'收到{command_name}命令')
        service_control_result = handle_service_control_request(message_data.get('body', {'services': []}).get('services', []))
        self.send_ack_response(message_data, 200, 'ok', service_control_result)
    
    def _handle_initial_pose(self, message_data, action_name):
        """处理初始化定位信息命令"""
        logger.info(f'收到{action_name}命令')
        self.initial_pose_manager.process_initial_pose(message_data)
        
    def send_ack_response(self, original_message, status, error, body):
        """
        发送ACK响应消息
        
        Args:
            original_message: 原始请求消息
            status: 状态码 (200=成功, 其他=失败)
            error: 错误描述
            body: 响应体数据
        """
        try:
            # 构建响应消息
            response = {
                'sn': original_message.get('sn', ''),
                'ver': original_message.get('ver', '1.0'),
                'seq': original_message.get('seq', 0),
                'type': 2,
                'ts': int(time.time() * 1000),
                'cmd': 'Ack',
                'status': status,
                'error': error,
                'body': body
            }
            
            # 发送响应
            response_json = json.dumps(response, ensure_ascii=False)
            self.mqtt_client.publish(self.mqtt_response_topic, response_json)
            logger.info(f'已发送ACK响应: {response_json}')
            
        except Exception as e:
            logger.error(f'发送ACK响应失败: {str(e)}')
    
    def check_mqtt_connection(self):
        """
        检查MQTT连接状态，确保连接正常
        """
        if not self.mqtt_connected:
            logger.warning('MQTT连接已断开，尝试重新连接...')
            # 如果连接断开，启动新的连接线程
            if not self.mqtt_thread.is_alive():
                self.mqtt_thread = Thread(target=self.connect_mqtt, daemon=True)
                self.mqtt_thread.start()
    def check_uav_fault(self):
        """
        检查无人机设备故障信息
        """
        logger.info('检查无人机设备故障信息')
        if self.ros_topic_subscriber._enable_status_report:
            self.uav_fault_inspection = True
            check_code, report = self.preflight_check_node._start_check()
            if check_code == 1:
                self.uav_fault_inspection = False
                self.ros_topic_subscriber._device_status_report.update_fault_status(report)
    
    def get_uav_fault_inspection(self):
        return self.uav_fault_inspection
                
    def check_ros_service_availability(self):
        """
        检查服务是否可用
        """
        logger.info('ROS服务正常运行中...')
    
    def shutdown(self):
        """
        关闭节点，清理资源
        """
        logger.info('正在关闭任务派发节点...')
        
        # 停止注册保活服务
        if hasattr(self, 'registration_manager'):
            self.registration_manager.stop()
            
        # 停止MQTT客户端
        if hasattr(self, 'mqtt_client'):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        
        # 清理告警上报器引用
        if hasattr(self, 'alarm_reporter'):
            self.alarm_reporter = None
        
        # 清理机器人状态订阅器引用
        if hasattr(self, 'ros_topic_subscriber'):
            self.ros_topic_subscriber = None
            
        # 清理远程控制引用
        if hasattr(self, 'remote_control'):
            self.remote_control = None
        
        # 清理巡检任务管理器引用
        if hasattr(self, 'cruise_uav_task_manager'):
            self.cruise_uav_task_manager.shutdown()
            self.cruise_uav_task_manager = None
        
        # 清理视频预览管理器引用
        if hasattr(self, 'video_preview_manager'):
            self.video_preview_manager.shutdown()
            self.video_preview_manager = None
        
        if hasattr(self, 'initial_pose_manager'):
            self.initial_pose_manager.shutdown()
            self.initial_pose_manager = None

        logger.info('任务派发节点已关闭')

def main(args=None):
    """
    主函数
    """
    rclpy.init(args=args)
    
    try:
        # 创建节点
        task_dispatcher_node = TaskDispatcherNode()
        
        # 运行节点
        rclpy.spin(task_dispatcher_node)
        
    except KeyboardInterrupt:
        logger.info('接收到中断信号，准备退出...')
    finally:
        # 关闭节点
        if 'task_dispatcher_node' in locals():
            task_dispatcher_node.shutdown()
        rclpy.shutdown()
        logger.info('程序已退出')

if __name__ == '__main__':
    main()
