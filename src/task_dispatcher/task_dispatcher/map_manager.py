#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rclpy
import requests
import logging
import yaml
import time
import threading
import uuid
import json
import subprocess
from PIL import Image
from typing import Optional, Dict, Any
from rclpy.node import Node
from rclpy.client import Client
# from nav2_msgs.srv import LoadMap
from .config import ROBOT_TYPE, FFR_SERVER_URL, ENABLE_PREFLIGHT_CHECK
import base64
from requests.auth import HTTPBasicAuth
# from explore_lite.msg import ExploreStatus
from task_dispatcher.perform_action_manager import PerformActionManager
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
from task_dispatcher.point_cloud_manager import PointCloudManager
from auth_utils import AuthManager 
from task_dispatcher.preflight_check_node import PreFlightCheckNode

# 配置日志
logger = logging.getLogger('task_dispatcher.map_manager')

class MapManager:
    """
    地图管理器，负责检查地图是否存在、更新地图、构建地图和停止构建地图
    """
    
    def __init__(self, node, mqtt_client, mqtt_response_topic,ros_topic_subscriber, map_folder: Optional[str] = None, 
    server_url: Optional[str] = None, robot_sn: Optional[str] = None, version: Optional[str] = None,
    perform_action_manager=None):
        """
        初始化地图管理器
        
        Args:
            node: ROS2节点实例
            mqtt_client: MQTT客户端实例
            mqtt_response_topic: MQTT响应主题
            ros_topic_subscriber: 机器人状态订阅器实例
            map_folder: 地图存储文件夹路径，如果为None则使用默认路径
            server_url: 地图服务器URL，如果为None则使用默认URL
            robot_sn: 机器人序列号，如果为None则使用默认值
            version: 地图版本号，如果为None则使用默认值
            perform_action_manager: 执行动作管理器实例
        """
        self.node = node
        self.mqtt_client = mqtt_client
        self.mqtt_response_topic = mqtt_response_topic
        self.ros_topic_subscriber: Ros2TopicSubscriber = ros_topic_subscriber
        self.robot_sn = robot_sn
        self.version = version
        self.perform_action_manager: PerformActionManager = perform_action_manager
        self.point_cloud_manager = PointCloudManager(
            node, 
            mqtt_client, 
            mqtt_response_topic, 
            ros_topic_subscriber,
            version,
            robot_sn,
            server_url,
            perform_action_manager
        )
             
        self.auth_manager = AuthManager(server_url)
        
        # 自检
        self.preflight_check_node = PreFlightCheckNode(
                    node=self.node,
                    topic_subscriber=self.ros_topic_subscriber,
                )
        
        # test
        # self.point_cloud_manager.process_point_cloud_data()
        
        # 设置地图文件夹路径
        if map_folder is None:
            # 默认使用当前用户目录下的maps文件夹
            self.map_folder = os.path.expanduser('/home/cat/slam_data/grid_map')
        else:
            self.map_folder = map_folder
        self.pcd_map_folder = '/home/cat/slam_data/3d_map'
        
        # 确保地图文件夹存在
        if not os.path.exists(self.map_folder):
            try:
                os.makedirs(self.map_folder)
                logger.info(f'创建地图文件夹: {self.map_folder}')
            except Exception as e:
                logger.error(f'创建地图文件夹失败: {str(e)}')
                
        if not os.path.exists(self.pcd_map_folder):
            try:
                os.makedirs(self.pcd_map_folder)
                logger.info(f'创建pcd地图文件夹: {self.pcd_map_folder}')
            except Exception as e:
                logger.error(f'创建pcd地图文件夹失败: {str(e)}')
        
        # 设置地图服务器URL
        if server_url is None:
            self.server_url = "https://112.48.30.69:19000"
        else:
            self.server_url = server_url
        
        # 任务管理
        self.build_map_tasks = {}  # 存储正在进行的地图构建任务，key: task_id, value: task_info
        self.task_lock = threading.Lock()  # 任务管理锁
        
        logger.info(f'地图管理器初始化完成，地图存储路径: {self.map_folder}')
        
    def check_pcd_map_existence(self) -> bool:
        try:
            # 构建地图文件路径
            pcd_map_path = os.path.join(self.pcd_map_folder, f'3dmap.pcd')
            
            # 检查文件是否存在
            if not os.path.exists(pcd_map_path):
                logger.info(f'地图pcd文件不存在: {pcd_map_path}')
                return False
        except Exception as e:
            logger.error(f'检查地图存在性时出错: {str(e)}')
            return False
        
    def check_map_existence(self, map_file_id: str) -> bool:
        """
        检查地图是否存在
        
        地图存在的条件：
        1. {map_folder}/{map_file_id}.png 文件存在
        2. {map_folder}/{map_file_id}.yaml 文件存在
        
        Args:
            map_file_id: 地图文件ID
            
        Returns:
            bool: 地图是否存在
        """
        try:
            # 构建地图文件路径
            png_map_path = os.path.join(self.map_folder, f'{map_file_id}.png')
            yaml_map_path = os.path.join(self.map_folder, f'{map_file_id}.yaml')
            
            # 检查文件是否存在
            if not os.path.exists(png_map_path):
                logger.info(f'地图PNG文件不存在: {png_map_path}')
                return False
            
            if not os.path.exists(yaml_map_path):
                logger.info(f'地图YAML文件不存在: {yaml_map_path}')
                return False
            
            # 读取并验证YAML文件内容
            try:
                with open(yaml_map_path, 'r') as f:
                    yaml_data = yaml.safe_load(f)
                
                # 检查必要字段
                required_fields = ['image', 'mode', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh']
                for field in required_fields:
                    if field not in yaml_data:
                        logger.warning(f'YAML文件缺少必要字段: {field}')
                        return False
                
                # 验证image字段值
                if yaml_data['image'] != f'{map_file_id}.png':
                    logger.warning(f'YAML文件image字段值不正确，应为"{map_file_id}.png"，实际为: {yaml_data["image"]}')
                    # 这里不返回False，因为我们可以修复这个问题
                    yaml_data['image'] = f'{map_file_id}.png'
                    # 保存修复后的YAML文件
                    with open(yaml_map_path, 'w') as f:
                        yaml.dump(yaml_data, f, default_flow_style=False)
                        logger.info(f'已修复YAML文件image字段值: {yaml_map_path}')
                        
            except yaml.YAMLError as e:
                logger.error(f'解析YAML文件失败: {str(e)}')
                return False
            except Exception as e:
                logger.error(f'读取YAML文件时出错: {str(e)}')
                return False
            
            logger.info(f'地图 {map_file_id} 存在且有效')
            return True
            
        except Exception as e:
            logger.error(f'检查地图存在性时出错: {str(e)}')
            return False
    
    def update_pcd_map(self, map_file_id: str, map_file_uri: str) -> bool:
        """
        更新地图，从服务器下载地图文件
        
        Args:
            map_file_id: 地图文件ID
            map_file_uri: 地图文件URI
            
        Returns:
            bool: 更新是否成功
        """
        try:
            logger.info(f'开始更新pcd地图: {map_file_id}')
            
            # 构建下载URL
            download_url = f'{self.server_url}/file/{map_file_uri}'
            logger.info(f'下载pcd地图URL: {download_url}')
            
            # 下载地图文件（假设返回的是PNG格式）
            response = requests.get(
                download_url,
                stream=True,
                timeout=30,  # 设置超时时间
                verify=False  # 忽略SSL证书验证，生产环境建议启用
            )
            
            if response.status_code != 200:
                logger.error(f'下载地图失败，HTTP状态码: {response.status_code}')
                return False
            
            # 保存PNG文件
            png_map_path = os.path.join(self.pcd_map_folder, f'3dmap.pcd')
            with open(png_map_path, 'wb') as f:
                for chunk in response.iter_content(chunk_size=8192):
                    if chunk:
                        f.write(chunk)
            
            logger.info(f'地图pcd文件已保存: {png_map_path}')
            
            return True
            
        except requests.RequestException as e:
            logger.error(f'下载pcd地图时网络错误: {str(e)}')
            return False
        except Exception as e:
            logger.error(f'更新pcd地图时出错: {str(e)}')
            return False
        
    def update_map(self, map_file_id: str, map_file_uri: str) -> bool:
        """
        更新地图，从服务器下载地图文件
        
        Args:
            map_file_id: 地图文件ID
            map_file_uri: 地图文件URI
            
        Returns:
            bool: 更新是否成功
        """
        try:
            logger.info(f'开始更新地图: {map_file_id}')
            
            # 构建下载URL
            download_url = f'{self.server_url}/file/{map_file_uri}'
            logger.info(f'下载地图URL: {download_url}')
            
            # 下载地图文件（假设返回的是PNG格式）
            response = requests.get(
                download_url,
                stream=True,
                timeout=30,  # 设置超时时间
                verify=False  # 忽略SSL证书验证，生产环境建议启用
            )
            
            if response.status_code != 200:
                logger.error(f'下载地图失败，HTTP状态码: {response.status_code}')
                return False
            
            # 保存PNG文件
            png_map_path = os.path.join(self.map_folder, f'{map_file_id}.png')
            with open(png_map_path, 'wb') as f:
                for chunk in response.iter_content(chunk_size=8192):
                    if chunk:
                        f.write(chunk)
            
            logger.info(f'地图PNG文件已保存: {png_map_path}')
            
            # 创建YAML文件
            yaml_map_path = os.path.join(self.map_folder, f'{map_file_id}.yaml')
            yaml_content = {
                'image': f'{map_file_id}.png',
                'mode': 'trinary',
                'resolution': 0.05,
                'origin': [0, 0, 0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.1,
            }
            
            with open(yaml_map_path, 'w') as f:
                yaml.dump(yaml_content, f, default_flow_style=None, sort_keys=False)
            
            logger.info(f'地图YAML文件已创建: {yaml_map_path}')
       
            logger.info(f'地图 {map_file_id} 更新成功')
            return True
            
        except requests.RequestException as e:
            logger.error(f'下载地图时网络错误: {str(e)}')
            return False
        except Exception as e:
            logger.error(f'更新地图时出错: {str(e)}')
            return False
            
    def push_map_to_service(self, map_file_id: str) -> bool:
        """
        将地图推送到地图服务器
        
        使用ROS2服务调用将地图加载到地图服务器，相当于执行:
        ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '/path/to/{map_file_id}.yaml'}"
        
        Args:
            map_file_id: 地图文件ID
            
        Returns:
            bool: 推送是否成功
        """
        try:
            # 构建YAML地图文件的绝对路径
            yaml_map_path = os.path.join(self.map_folder, f'{map_file_id}.yaml')
            
            # 确保地图文件存在
            if not os.path.exists(yaml_map_path):
                logger.error(f'地图YAML文件不存在: {yaml_map_path}')
                return False
            
            logger.info(f'准备推送地图到服务: {yaml_map_path}')
            
            # 创建服务客户端
            load_map_client: Client = self.node.create_client(LoadMap, '/map_server/load_map')
            
            # 等待服务可用
            if not load_map_client.wait_for_service(timeout_sec=10.0):
                logger.error(f'地图服务 /map_server/load_map 不可用')
                return False
            
            # 创建请求
            request = LoadMap.Request()
            request.map_url = yaml_map_path
            
            # 发送请求
            future = load_map_client.call_async(request)
            
            # 同步等待响应，使用轮询方式避免rclpy.spin_until_future_complete的问题
            import time
            start_time = time.time()
            timeout = 1.5  # 10秒超时
            
            while not future.done() and (time.time() - start_time) < timeout:
                # 短暂休眠避免CPU占用过高
                time.sleep(0.1)
            
            # 检查结果
            if future.done():
                try:
                    response = future.result()      
                    # 根据用户提供的信息：result=0表示成功
                    if response.result != 0:
                        logger.error(f"地图加载服务调用失败: map_file_id={map_file_id}, result={response.result}")
                        return False
                    else:
                        # 检查地图数据是否为空
                        if hasattr(response, 'map') and hasattr(response.map, 'data') and len(response.map.data) == 0:
                            logger.error(f"地图加载服务调用失败: map_file_id={map_file_id}, 地图数据为空")
                            return False
                        else:
                            logger.info(f"地图加载服务调用成功: map_file_id={map_file_id}, map_info={response.map.info}")
                            return True
                            
                except Exception as e:
                    logger.error(f'获取地图加载服务响应时出错: {str(e)}')
                    return False
            else:
                logger.error(f'地图加载服务调用超时: map_file_id={map_file_id}, 超时时间={timeout}秒')
                return False
                
        except Exception as e:
            logger.error(f'推送地图到服务时出错: {str(e)}')
            return False
    
    def build_map(self, message_data: Dict[str, Any]) -> None:
        """
        构建地图
        
        Args:
            message_data: 包含地图构建请求的消息数据
        """
        try:
            logger.info(f'开始构建地图请求: {message_data}')
            

            # 解析请求参数
            body = message_data.get('body', {})
            map_name = body.get('mapName')
            map_type = body.get('mapType')
            
            if not map_name or not map_type:
                logger.error('地图名称和地图类型不能为空')
                self.send_ack_response(message_data, 400, '地图名称和地图类型不能为空')
                return
            
            # 验证地图类型
            valid_map_types = ['occupancy_grid', 'elevation_map']
            if map_type not in valid_map_types:
                logger.error(f'无效的地图类型: {map_type}，支持的类型: {valid_map_types}')
                self.send_ack_response(message_data, 400, f'无效的地图类型: {map_type}')
                return
            
            # 生成任务ID
            task_id = f'buildmap_{time.strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:8]}'
            
            # 构建任务信息
            task_info = {
                'task_id': task_id,
                'map_name': map_name,
                'map_type': map_type,
                'resolution': body.get('resolution', 0.05),  # 默认分辨率0.05米/像素
                'upload_info': body.get('uploadInfo'),
                'is_upload_pic': body.get('isUploadPic', False),
                'start_time': time.time(),
                'status': 'running'
            }
            
            # 保存任务信息
            with self.task_lock:
                self.build_map_tasks[task_id] = task_info
            
            # 启动异步构建任务
            threading.Thread(target=self._async_build_map, args=(task_id,), daemon=True).start()
            
            logger.info(f'地图构建任务已启动: {task_id}')
            
            # 返回初始状态
            self.send_ack_response(message_data, 200, '地图构建任务已启动', {'task_id': task_id})
            
        except Exception as e:
            logger.error(f'构建地图请求处理失败: {str(e)}')
            self.send_ack_response(message_data, 500, f'处理请求失败: {str(e)}')
    
    def _async_build_map(self, task_id: str):
        """
        异步构建地图的内部方法
        
        Args:
            task_id: 任务ID
        """
        try:
            logger.info(f'异步构建地图开始: {task_id}')
            
            if ENABLE_PREFLIGHT_CHECK:
                # if self.node.get_uav_fault_inspection() == False:
                #     return
                
                check_code, msg = self.preflight_check_node._start_check()
                if check_code == 1:
                    logger.warning(msg)
                    return
                    
            # 获取任务信息
            with self.task_lock:
                task_info = self.build_map_tasks.get(task_id)
                if not task_info:
                    logger.error(f'未找到任务信息: {task_id}')
                    self.report_build_map_progress(400, f'未找到任务信息: {task_id}')
                    return

            # 检查机器人类型
            if ROBOT_TYPE == 'ffr':
                # FFR机器人，调用HTTP接口获取地图信息
                try:
                    logger.info('FFR机器人类型，调用HTTP接口获取地图信息')
                    
                    # 调用FFR机器人的地图信息接口
                    url = f'{FFR_SERVER_URL}/fireRobot/getMapInfo?isQueryPos=true'
                    response = requests.get(url, timeout=5)
                    response.raise_for_status()  # 检查请求是否成功
                    
                    result = response.json()
                    #logger.info(f'获取地图信息成功: {result}')
                    
                    # 检查响应状态
                    if result['code'] != 200:
                        logger.error(f'获取地图信息失败: {result["message"]}')
                        with self.task_lock:
                            task_info['status'] = 'failed'
                            task_info['error'] = result['message']
                            self.report_build_map_progress(400, result['message'])
                            return
                    
                    map_info = result['data']

                    # 上传图像到文件服务器
                    file_id = ''
                    if map_info.get('base64'):
                        file_id = self.upload_image_to_file_server(map_info['base64'])
                        if not file_id or file_id == '':
                            logger.error('图像上传失败，跳过地图构建')
                            with self.task_lock:
                                task_info['status'] = 'failed'
                                task_info['error'] = '图像上传失败'
                                self.report_build_map_progress(400, '图像上传失败')
                                return  
                    
                    # 构建进度消息
                    progress_message = {
                        'taskId': task_id,
                        'estimatedTime': 0,  # 目前不支持自动，此值无效
                        'message': '地图构建完成',
                        'progress': 100,  # 完成进度，百分比
                        'fileId': file_id,
                        'picBase64': map_info['base64'],
                        'marks': [],  # 标注信息列表
                        'mapWidth': map_info['mapWidth'],
                        'mapHeight': map_info['mapHeight'],
                        'originPoint': {
                            'x': map_info['originPoint']['x'],
                            'y': map_info['originPoint']['y'],
                            'z': map_info['originPoint']['z'],
                            'w': map_info['originPoint']['w']
                        }
                    }
                    
                    # 处理地图点位信息，转换为标注格式
                    if map_info.get('positions'):
                        for pos in map_info['positions']:
                            # 转换点位类型
                            pos_type = 'point'  # 默认点位类型为point
                            
                            # 构建标注信息
                            mark = {
                                'name': pos['name'],
                                'type': pos_type,
                                'locations': [{
                                    'x': pos['position']['x'],
                                    'y': pos['position']['y'],
                                    'z': pos['position']['z'],
                                    'w': pos['position']['w']
                                }]
                            }
                            progress_message['marks'].append(mark)
                          
                    # 发布地图构建完成消息
                    self.report_build_map_progress(200, 'ok', progress_message)
                    print_data = progress_message.copy()
                    print_data.pop('picBase64', None)  # 移除base64数据，避免日志过长
                    logger.info(f'已发布地图构建完成消息: {json.dumps(print_data)}')
                    
                except requests.RequestException as e:
                    logger.error(f'调用FFR地图接口失败: {str(e)}')
                    self.report_build_map_progress(400, f'调用地图接口失败: {str(e)}')
                    with self.task_lock:
                        task_info['status'] = 'failed'
                        task_info['error'] = f'调用地图接口失败: {str(e)}'
                    return
                except Exception as e:
                    logger.error(f'处理FFR地图信息失败: {str(e)}')
                    self.report_build_map_progress(400, f'处理地图信息失败: {str(e)}')
                    with self.task_lock:
                        task_info['status'] = 'failed'
                        task_info['error'] = f'处理地图信息失败: {str(e)}'
                    return
            elif ROBOT_TYPE == 'uav':

                self.task_id = task_id

                pcd_file_id, col_radar, row_radar = self.point_cloud_manager._processing_uav_task(task_id)
                
                self.save_map(pcd_file_id, col_radar, row_radar)
            

            else:
                # 非FFR机器人，通过systemd控制服务
                logger.info('非FFR机器人类型，通过systemd停止lio_sam_nav2服务并启动auto_buildmap.service服务')
                result = self.perform_action_manager.perform_action_stand(argv={"mode": 1})
                
                # 停止lio_sam_nav2服务（不检查错误）
                logger.info('停止lio_sam_nav2服务...')
                subprocess.run(['sudo', 'systemctl', 'stop', 'lio_sam_nav2.service'], timeout=10)
                logger.info('lio_sam_nav2服务操作完成')
                
                # 启动auto_buildmap.service服务（不检查错误）
                logger.info('启动auto_buildmap.service服务...')
                subprocess.run(['sudo', 'systemctl', 'start', 'auto_buildmap.service'], timeout=10)
                logger.info('auto_buildmap.service服务操作完成')
                
                self.task_id = task_id
                # 等待服务启动完成
                time.sleep(2)
            
            # 构建完成，更新任务状态
            with self.task_lock:
                task_info['status'] = 'completed'
                task_info['end_time'] = time.time()
            
            logger.info(f'地图构建任务完成: {task_id}')
            
        except Exception as e:
            logger.error(f'异步构建地图失败: {str(e)}')
            self.report_build_map_progress(400, f'异步构建地图失败: {str(e)}')
            
            # 更新任务状态并发布错误
            with self.task_lock:
                task_info = self.build_map_tasks.get(task_id)
                if task_info:
                    task_info['status'] = 'failed'
                    task_info['error'] = str(e)
    
    def stop_build_map(self, message_data: Dict[str, Any]) -> None:
        """
        停止构建地图
        
        Args:
            message_data: 包含停止构建请求的消息数据
        """
        try:
            logger.info(f'开始停止构建地图请求: {message_data}')
            
            # 解析请求参数
            body = message_data.get('body', {})
            task_id = body.get('taskId')
            is_upload = body.get('isUpload', False)
            
            if not task_id:
                logger.error('任务ID不能为空')
                self.send_ack_response(message_data, 404, '任务不存在')
                return
            
            # 查找任务
            with self.task_lock:
                task_info = self.build_map_tasks.get(task_id)
                if not task_info:
                    logger.error(f'未找到地图构建任务: {task_id}')
                    self.send_ack_response(message_data, 404, '任务不存在')
                    return
                
                # 检查任务状态
                if task_info['status'] == 'completed':
                    self.send_ack_response(message_data, 200, 'ok')
                    return
                elif task_info['status'] == 'stopped':
                    self.send_ack_response(message_data, 200, 'ok')
                    return
                
                # 停止任务
                task_info['status'] = 'stopped'
                task_info['stop_time'] = time.time()
                task_info['is_upload'] = is_upload
            
            logger.info(f'地图构建任务已停止: {task_id}')
            
            # 计算耗时
            cost_time = (task_info['stop_time'] - task_info['start_time']) * 1000  # 转换为毫秒
            
            # 如果需要上传地图
            map_file_url = ''
            map_file_size = 0.0
            
            if is_upload:
                # 这里只是示例，实际项目中需要实现地图上传逻辑
                map_file_url = f'{self.map_folder}/{task_info["map_name"]}.yaml'
                if os.path.exists(map_file_url):
                    map_file_size = os.path.getsize(map_file_url) / (1024 * 1024)  # 转换为MB
            
            self.send_ack_response(
                message_data, 
                200, 
                'ok',
                {
                    'mapFileUrl': map_file_url,
                    'mapFileSize': round(map_file_size, 2),
                    'costTime': round(cost_time, 2),
                    'message': '地图构建已停止'
                }
            )
            
        except Exception as e:
            logger.error(f'停止地图构建失败: {str(e)}')
            self.send_ack_response(message_data, 500, '服务器错误')

    def send_ack_response(self, original_message, status, error_msg, body=None):
        """
        发送Ack响应到MQTT
        
        Args:
            original_message: 原始消息
            status: 状态码
            error_msg: 错误消息
            body: 响应体（可选）
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
            
            # 如果有响应体，添加到响应中
            if body:
                response['body'] = body
            
            # 发布到响应主题
            self.mqtt_client.publish(self.mqtt_response_topic, json.dumps(response))
            logger.info(f'已发送Ack响应: status={status}, error={error_msg}')
            
        except Exception as e:
            logger.error(f'发送Ack响应失败: {str(e)}')

    def report_build_map_progress(self, status, error_msg, body=None):
        """
        发送BuildMap响应到MQTT
        
        Args:
            status: 状态码
            error_msg: 错误消息
            body: 响应体（可选）
        """
        try:
            response = {
                'sn': self.robot_sn,
                'ver': self.version,
                'seq': str(uuid.uuid4()).replace('-', ''),
                'type': 1,  # 请求类型
                'ts': int(time.time() * 1000),
                'cmd': 'BuildMap',
                'status': status,
                'error': error_msg
            }
            
            # 如果有响应体，添加到响应中
            if body:
                response['body'] = body
            
            # 发布到响应主题
            self.mqtt_client.publish(self.mqtt_response_topic, json.dumps(response))
            logger.info(f'已发送BuildMap响应: status={status}, error={error_msg}')
            
        except Exception as e:
            logger.error(f'发送BuildMap响应失败: {str(e)}')

   
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
            unique_filename = f'map_image_{timestamp}.jpg'
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

    def save_map(self, pcd_file_id, col_radar = 0.0, row_radar = 0.0):
        """
        保存地图到指定路径
        """
        if pcd_file_id == '':
            logger.warning(f"pcd_file_id为空,跳过二维地图保存步骤.")
            return
        
        logger.info('开始保存地图...')
        
        try:
            # 切换到dog_slam目录并执行地图保存命令
            # map_save_cmd = '''
            #     cd /home/cat/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/ && \\ 
            #     source install/setup.bash && \\ 
            #     ros2 run nav2_map_server map_saver_cli -t /projected_map -f /home/cat/slam_data/map_10 --fmt png
            # '''
            
            # result = subprocess.run(map_save_cmd, shell=True, check=True, capture_output=True, text=True, timeout=30)
            # logger.info(f'地图保存成功: {result.stdout}')
            # 保存地图成功后，将地图上传到文件服务器
            # 从/home/cat/slam_data/map_10.png读取base64编码的图像数据
            # 读取图像文件并获取base64编码
            with open('/home/cat/slam_data/map.png', 'rb') as f:
                map_image_base64 = base64.b64encode(f.read()).decode('utf-8')
            
            # 获取图像宽高
            with Image.open('/home/cat/slam_data/map.png') as img:
                map_width, map_height = img.size
            logger.info(f'获取地图图像宽高成功: {map_width}x{map_height}')

            file_id = self.upload_image_to_file_server(map_image_base64)
            if not file_id or file_id == '':
                logger.error('图像上传失败，跳过地图构建')
                self.report_build_map_progress(400, '图像上传失败')
                return  
            
            # 构建进度消息
            progress_message = {
                'taskId': self.task_id,
                'estimatedTime': 0,  # 目前不支持自动，此值无效
                'message': '地图构建完成',
                'progress': 100,  # 完成进度，百分比
                'fileId': file_id,
                'pcdFileId': pcd_file_id,
                'picBase64': map_image_base64,
                'marks': [],  # 标注信息列表
                'mapWidth': map_width,
                'mapHeight': map_height,
                'originPoint': {
                    'x': col_radar,
                    'y': row_radar,
                    'z': 0.0,
                    'w': 0.0
                }
            }
                    
            # 发布地图构建完成消息
            self.report_build_map_progress(200, 'ok', progress_message)
            print_data = progress_message.copy()
            print_data.pop('picBase64', None)  # 移除base64数据，避免日志过长
            logger.info(f'已发布地图构建完成消息: {json.dumps(print_data)}')
            
        except subprocess.CalledProcessError as e:
            logger.error(f'地图保存失败，命令返回错误: {e.stderr}')
        except subprocess.TimeoutExpired:
            logger.error(f'地图保存命令执行超时')
        except Exception as e:
            logger.error(f'地图保存异常: {str(e)}')

    def end_buildmap_action(self):
        """
        结束地图构建动作
        """
        if ROBOT_TYPE == 'uav':
            # fast_lio.service服务（不检查错误）
            logger.info('fast_lio.service服务...')
            subprocess.run(['sudo', 'systemctl', 'stop', 'fast_lio.service'], timeout=10)
            logger.info('fast_lio.service服务操作完成')
        else:
            # 停止auto_buildmap.service服务（不检查错误）
            logger.info('停止auto_buildmap.service服务...')
            subprocess.run(['sudo', 'systemctl', 'stop', 'auto_buildmap.service'], timeout=10)
            logger.info('auto_buildmap.service服务操作完成')

        # 启动lio_sam_nav2服务（不检查错误）
        logger.info('启动lio_sam_nav2服务...')
        subprocess.run(['sudo', 'systemctl', 'start', 'lio_sam_nav2.service'], timeout=10)
        logger.info('lio_sam_nav2服务操作完成')
        # 停止动作
        result = self.perform_action_manager.perform_action_stop()
        time.sleep(1)
        # 设置机器人蹲站
        result = self.perform_action_manager.perform_action_stand(argv={"mode": 2})
        time.sleep(1)
        # 设置机器人趴下
        result = self.perform_action_manager.perform_action_stand(argv={"mode": 0})

    # def handle_map_data(self, map_data: ExploreStatus):
    #     """
    #     处理地图数据，将其上传到文件服务器
        
    #     Args:
    #         map_data: 包含地图数据的ExploreStatus消息
    #     """

    #     if map_data.state == ExploreStatus.STATE_STARTING:
    #         logger.info(f'开始探索地图，状态描述: {map_data.state_description}')
        
    #     elif map_data.state == ExploreStatus.STATE_EXPLORING:
    #         logger.info(f'正在探索地图，已探索前沿数量: {map_data.frontiers_explored}，剩余前沿数量: {map_data.frontiers_remaining}，进度: {map_data.progress_percentage:.2f}%')
        
    #     elif map_data.state == ExploreStatus.STATE_COMPLETED:
    #         logger.info(f'地图探索完成，已探索前沿数量: {map_data.frontiers_explored}，剩余前沿数量: {map_data.frontiers_remaining}，进度: {map_data.progress_percentage:.2f}%')
    #         # 运行地图保存命令
    #         self.save_map()
    #         # 结束地图构建动作
    #         self.end_buildmap_action()
            
    #     elif map_data.state == ExploreStatus.STATE_STOPPED:
    #         logger.info(f'地图探索已停止，状态描述: {map_data.state_description}')
    #         # 结束地图构建动作
    #         self.end_buildmap_action()
