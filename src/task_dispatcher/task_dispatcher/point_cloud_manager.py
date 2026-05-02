#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import logging
import time
import threading
import subprocess
import os
import base64
import requests
import rclpy
from rclpy.executors import SingleThreadedExecutor
from typing import Dict, List, Optional, Any
from task_dispatcher.ros2_topic_subscriber import Ros2TopicSubscriber
import uuid
from task_dispatcher.message_cache import message_cache_manager
import task_dispatcher.ptz_control as PtzControl
from task_dispatcher.perform_action_manager import PerformActionManager
from task_dispatcher.point_cloud_persistor import PointCloudPersistor
from enum import Enum
from std_srvs.srv import Trigger
from rclpy.client import Client
from requests.auth import HTTPBasicAuth
from .config import ROBOT_TYPE, PCD_FILE_PATH, PCD_FILE_COMPRESS_PATH  # 引入配置文件中的URL和点云文件路径
import matplotlib.cm as cm
import open3d as o3d
import numpy as np
from PIL import Image, ImageOps
from task_dispatcher.point_cloud_crop import PointCloudCrop
from auth_utils import AuthManager 

# 配置日志
logger = logging.getLogger('task_dispatcher.point_cloud_manager')
    
class PointCloudManager:
    """
    点云文件处理器
    支持：
    """
    
    def __init__(self, node, mqtt_client, response_topic, ros_topic_subscriber, version='1.0', robot_sn='', 
        server_url=None, perform_action_manager=None):
        """
        初始化巡检任务管理器
        
        Args:
            node: ROS2节点实例
            mqtt_client: MQTT客户端实例
            response_topic: MQTT响应主题
            ros_topic_subscriber: 机器人状态订阅器实例
            version: 版本号
            robot_sn: 机器人序列号
            ptz_controller: PTZ控制器实例
            server_url: 地图服务器URL
            perform_action_manager: 执行动作管理器实例
        """
        self.node = node
        self.mqtt_client = mqtt_client
        self.mqtt_response_topic = response_topic
        self.ros_topic_subscriber: Ros2TopicSubscriber = ros_topic_subscriber
        self.perform_action_manager: PerformActionManager = perform_action_manager
        self.server_url = server_url
        
        self.auth_manager = AuthManager(server_url)
        
        # 点云持久化管理器
        self.point_cloud_persistor = PointCloudPersistor(node, server_url)
        # 设置上传回调函数
        self.point_cloud_persistor.set_upload_callback(self.upload_point_cloud_to_file_server)
        
        self.point_cloud_crop = PointCloudCrop()
        
        # 存储巡检任务的字典，key为taskId
        self.tasks: Dict[str, Dict] = {}
        # 当前正在执行的任务ID
        self.current_task_id: Optional[str] = None
        # 任务执行状态标志
        self.is_executing: bool = False
        # 线程锁，用于保护共享变量
        self._lock = threading.Lock()
        
        logger.info('点云任务管理器初始化完成')
    
    def _processing_uav_task(self, task_id):
        """
        处理无人机巡检任务
        
        Args:
            task_id: 任务ID
        """
        #self._report_task_status(task_id, TaskStatus.STARTED.value, '任务已开始')

        # 标记为正在执行（已经在线程外部设置）
        with self._lock:
            self.cruises = [{'taskId': task_id, 'status': True}]
        self.ros_topic_subscriber.set_cruises(self.cruises)
        # 判断是否已经飞过了一次
        has_flown = False
        # 判断是否飞行完成
        flight_finished = False

        # 启动点云服务（不检查错误）
        logger.info('启动fast_lio.service服务...')
        subprocess.run(['sudo', 'systemctl', 'start', 'build_map_drone.service'], timeout=30)
        logger.info('启动fast_lio.service服务操作完成')
        time.sleep(5)
        
        self.is_executing = True
        self.current_task_id = task_id
        
        logger.info(f"等待无人机解锁并起飞...")
        while self.is_executing:
            takeoff_time, arming_state = self.ros_topic_subscriber.get_takeoff_time_and_arming_state()
            logger.info(f"当前无人机解锁状态: {arming_state}, 起飞时间: {takeoff_time}")
            # 无人机已经解锁并起飞
            if takeoff_time > 0 and arming_state == 2:
                has_flown = True
                break
            time.sleep(1)

        logger.info(f"等待无人机降落...")
        while self.is_executing:
            takeoff_time, arming_state = self.ros_topic_subscriber.get_takeoff_time_and_arming_state()
            logger.info(f"当前无人机解锁状态: {arming_state}, 起飞时间: {takeoff_time}")
            # 无人机已经降落
            if takeoff_time == 0 and arming_state == 1:
                flight_finished = True
                break
            time.sleep(1)

        logger.info(f"无人机已经降落，任务已完成")

        # 处理点云数据
        file_id, col_radar, row_radar = self.process_point_cloud_data()

        # 停止点云服务（不检查错误）
        logger.info('停止fast_lio.service服务...')
        subprocess.Popen(['sudo', 'systemctl', 'stop', 'build_map_drone.service'])
        logger.info('停止fast_lio.service服务操作完成')

        logger.info(f"执行巡检任务task_id={task_id}, 任务已完成")
        # self._report_task_status(task_id, TaskStatus.COMPLETED.value, '任务已完成', file_id, 'pcd')
        # 确保重置状态
        self.reset_status()
        
        return file_id, col_radar, row_radar

    def reset_status(self):
        """
        重置任务执行状态

        """
        # 标记为未执行
        with self._lock:
            self.is_executing = False
            self.current_task_id = None
            self.cruises = []
        self.ros_topic_subscriber.set_cruises(self.cruises)
        # self.perform_action_manager.set_stop_action(False)

    def fast_downsample(self, pcd, voxel_size=0.25, filter_params={'nb_neighbors':20, 'std_ratio':2.0}):
        # 体素下采样
        down_pcd = pcd.voxel_down_sample(voxel_size)
        # 滤波（可选）
        # if filter_params:
        #     nb_neighbors = filter_params.get('nb_neighbors', 20)
        #     std_ratio = filter_params.get('std_ratio', 2.0)
        #     down_pcd, _ = down_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        #     logger.info(f"滤波后点数: {len(pcd.points)}")
        logger.info(f"下采样后点数: {len(down_pcd.points)}")

        return down_pcd
    
    def pcd_to_raster(self, pcd, output_img, cell_size=0.05,
                      x_range=None, y_range=None,
                      radar_point=(0,0,0), use_color=False,
                      contrast_percentile=None, equalize=True, gamma=None):
        """
        将 PCD 点云栅格化并输出图像（支持对比度增强，已修复内存溢出问题）
        """
        # 1. 读取点云
        points = np.asarray(pcd.points)
        if points.shape[0] == 0:
            logger.info("点云为空")
            return False, 0.0, 0.0

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # ==========================================
        # [新增] 1.1 过滤异常坐标（防止噪点导致范围爆炸）
        # ==========================================
        # 设置一个合理的最大范围阈值（例如 10km），超出此范围的点通常是由于传感器故障或计算错误产生的无穷大/噪点
        MAX_COORDINATE_VALUE = 10000.0 
        valid_mask_coords = (np.abs(x) < MAX_COORDINATE_VALUE) & \
                            (np.abs(y) < MAX_COORDINATE_VALUE) & \
                            (np.abs(z) < MAX_COORDINATE_VALUE)
        
        if not np.all(valid_mask_coords):
            invalid_count = np.sum(~valid_mask_coords)
            logger.warning(f"检测到 {invalid_count} 个异常坐标点（超出 {MAX_COORDINATE_VALUE}m），已自动剔除。")
            x = x[valid_mask_coords]
            y = y[valid_mask_coords]
            z = z[valid_mask_coords]
            
        if len(x) == 0:
            logger.error("所有点均为异常点，过滤后点云为空")
            return False, 0.0, 0.0

        # 2. 确定投影范围
        if x_range is None:
            x_min, x_max = x.min(), x.max()
            # 避免范围过小导致除以零
            if x_max - x_min < 1e-6: 
                x_min -= 1.0
                x_max += 1.0
            else:
                margin_x = (x_max - x_min) * 0.01
                x_min -= margin_x
                x_max += margin_x
        else:
            x_min, x_max = x_range
            
        if y_range is None:
            y_min, y_max = y.min(), y.max()
            if y_max - y_min < 1e-6:
                y_min -= 1.0
                y_max += 1.0
            else:
                margin_y = (y_max - y_min) * 0.01
                y_min -= margin_y
                y_max += margin_y
        else:
            y_min, y_max = y_range

        logger.info(f"投影范围：X [{x_min:.3f}, {x_max:.3f}], Y [{y_min:.3f}, {y_max:.3f}]")

        # 初步计算行列数
        n_cols = int(np.ceil((x_max - x_min) / cell_size))
        n_rows = int(np.ceil((y_max - y_min) / cell_size))
        
        # ==========================================
        # [新增] 2.1 栅格尺寸安全检查（防止内存溢出）
        # ==========================================
        # 限制最大像素总数，例如 5000x5000 = 2500万像素。
        # 63GB 的报错意味着之前的计算结果可能产生了数亿甚至数十亿的像素。
        MAX_TOTAL_PIXELS = 25_000_000 
        
        if n_cols * n_rows > MAX_TOTAL_PIXELS:
            logger.warning(f"预估栅格过大 ({n_cols}x{n_rows} = {n_cols*n_rows:,} 像素)，可能导致内存溢出。")
            
            # 自动计算缩放因子，强制将分辨率降低到安全范围内
            scale_factor = np.sqrt((n_cols * n_rows) / MAX_TOTAL_PIXELS)
            new_cell_size = cell_size * scale_factor
            
            logger.warning(f"自动调整 cell_size: {cell_size:.4f} -> {new_cell_size:.4f}")
            cell_size = new_cell_size
            
            # 重新计算行列数
            n_cols = int(np.ceil((x_max - x_min) / cell_size))
            n_rows = int(np.ceil((y_max - y_min) / cell_size))
            
        logger.info(f"最终栅格大小：{n_cols} 列 x {n_rows} 行 (cell_size={cell_size})")

        # 4. 计算栅格索引
        col_indices = np.floor((x - x_min) / cell_size).astype(int)
        row_indices = np.floor((y - y_min) / cell_size).astype(int)

        valid_mask = (col_indices >= 0) & (col_indices < n_cols) & \
                    (row_indices >= 0) & (row_indices < n_rows)
        col_indices = col_indices[valid_mask]
        row_indices = row_indices[valid_mask]
        z_valid = z[valid_mask]
        logger.info(f"有效点数：{len(z_valid)} / {len(x)}")

        if len(z_valid) == 0:
            logger.info("没有点在投影范围内")
            return False, 0.0, 0.0

        # 5. 聚合计算平均高度
        # 注意：此时 n_cols*n_rows 已经被上面的安全检查限制在安全范围内
        flat_indices = row_indices * n_cols + col_indices
        z_sum_flat = np.bincount(flat_indices, weights=z_valid, minlength=n_rows*n_cols)
        count_flat = np.bincount(flat_indices, minlength=n_rows*n_cols)

        z_sum_bottom = z_sum_flat.reshape(n_rows, n_cols)
        count_bottom = count_flat.reshape(n_rows, n_cols)

        # 翻转行，使数组索引 0 对应图像顶部
        z_sum = np.flipud(z_sum_bottom)
        count = np.flipud(count_bottom)

        avg_z = np.zeros_like(z_sum, dtype=float)
        mask = count > 0
        avg_z[mask] = z_sum[mask] / count[mask]

        # 6. 灰度映射
        z_vals = avg_z[mask]
        if len(z_vals) == 0:
            logger.info("没有有效栅格")
            return False, 0.0, 0.0
            
        z_min_global = z_vals.min()
        z_max_global = z_vals.max()

        if contrast_percentile is not None and not equalize:
            p_low = contrast_percentile
            p_high = 100 - contrast_percentile
            z_low = np.percentile(z_vals, p_low)
            z_high = np.percentile(z_vals, p_high)
            logger.info(f"百分比裁剪：使用 [{p_low}%, {p_high}%] 范围 [{z_low:.3f}, {z_high:.3f}]")
            gray_float = np.zeros_like(avg_z)
            gray_float[mask] = (255 * (avg_z[mask] - z_low) / (z_high - z_low))
            gray_float = np.clip(gray_float, 0, 255)
            gray = gray_float.astype(np.uint8)
        else:
            if z_max_global - z_min_global < 1e-6:
                gray = np.full_like(avg_z, 128, dtype=np.uint8)
            else:
                gray_float = np.zeros_like(avg_z)
                gray_float[mask] = (255 * (avg_z[mask] - z_min_global) / (z_max_global - z_min_global))
                gray = gray_float.astype(np.uint8)

        # 7. 直方图均衡化
        if equalize:
            img_gray = Image.fromarray(gray, 'L')
            img_eq = ImageOps.equalize(img_gray)
            gray = np.array(img_eq)
            logger.info("已应用直方图均衡化")

        # 8. 伽马校正
        if gamma is not None:
            gray_float = gray.astype(np.float32) / 255.0
            gray_float = np.power(gray_float, 1.0 / gamma)
            gray = (gray_float * 255).astype(np.uint8)
            logger.info(f"已应用伽马校正，gamma={gamma}")

        # 9. 生成彩色图像
        if use_color:
            try:
                import matplotlib.pyplot as plt
                cmap = plt.get_cmap('jet')
                colored = cmap(gray / 255.0)
                colored = (colored[:, :, :3] * 255).astype(np.uint8)
                img = Image.fromarray(colored, 'RGB')
            except ImportError:
                logger.info("警告：未安装 matplotlib，将输出灰度图像。")
                img = Image.fromarray(gray, 'L')
        else:
            img = Image.fromarray(gray, 'L')

        img.save(output_img)
        logger.info(f"图像已保存至：{output_img}")

        # 10. 计算雷达点图像坐标
        radar_x, radar_y, radar_z = radar_point
        col_radar = (radar_x - x_min) / cell_size
        row_radar = (radar_y - y_min) / cell_size
        grid_x = col_radar * cell_size
        grid_y = row_radar * cell_size
        logger.info(f"雷达点 ({radar_x}, {radar_y}, {radar_z}) 的图像坐标：")
        logger.info(f"  列 (col) = {col_radar:.2f}  (从左到右)")
        logger.info(f"  行 (row) = {row_radar:.2f}  (从下到上)")
        logger.info(f"  grid_x = {grid_x:.2f}  (从左到右)")
        logger.info(f"  grid_y = {grid_y:.2f}  (从下到上)")

        if 0 <= col_radar < n_cols and 0 <= row_radar < n_rows:
            logger.info("  该点位于图像内部。")
        else:
            logger.info("  该点位于图像外部。")

        return True, col_radar, row_radar
    def process_point_cloud_data(self):
        '''
        处理点云数据，上传到文件服务器

            
        Returns:
            str or None: 上传成功返回文件ID，失败返回None
        '''

        try:
            logger.info("开始处理点云数据保存和上传")
            
            # 1. 创建ROS2服务客户端
            map_save_client = self.node.create_client(Trigger, '/map_save')
            
            logger.info("点云保存等待服务")
            
            # 2. 等待服务可用
            if not map_save_client.wait_for_service(timeout_sec=5.0):
                logger.error("点云保存服务 /map_save 不可用")
                return '', 0.0, 0.0
            
            # 3. 创建请求并调用服务
            request = Trigger.Request()
            future = map_save_client.call_async(request)
            
            # 4. 使用 SingleThreadedExecutor 避免 "wait set index too big" 错误
            service_timeout = 60.0  # 服务调用超时时间（秒）
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(self.node)
            result = executor.spin_until_future_complete(future, timeout_sec=service_timeout)
            executor.remove_node(self.node)

            # 检查是否超时
            if result != rclpy.executors.FutureState.COMPLETED:
                logger.error(f"点云保存服务调用超时（{service_timeout}秒）")
                return '', 0.0, 0.0
            
            # 5. 检查服务调用结果
            if future.result():
                if future.result().success:
                    logger.info("点云保存成功")
                else:
                    logger.error(f"点云保存失败: {future.result().message}")
                    return '', 0.0, 0.0
            else:
                logger.error("点云保存服务调用失败")
                return '', 0.0, 0.0
            
            # 6. 检查点云文件是否存在
            pcd_file_path = PCD_FILE_PATH
            if not os.path.exists(pcd_file_path):
                logger.error(f"点云文件不存在: {pcd_file_path}")
                return '', 0.0, 0.0
            pcd = o3d.io.read_point_cloud(pcd_file_path)
            if pcd.is_empty():
                logger.info("点云读取后为空")
                return '', 0.0, 0.0
            
            logger.info("点云数据压缩(采样-滤波)")
            pcd = self.fast_downsample(pcd, voxel_size=0.25, filter_params={'nb_neighbors':20, 'std_ratio':2.0})
            
            logger.info("裁减点云数据")
            #pcd = self.point_cloud_crop.preprocess(pcd)
            
            logger.info("开始执行点云文件截图")
            png_res, col_radar, row_radar = self.pcd_to_raster(pcd, '/home/cat/slam_data/map.png')
            if png_res == False:
                logger.error(f"点云文件截图失败")
                return '', 0.0, 0.0
            
            result_pcd = pcd
            
            logger.info("保存压缩后的点云文件")
            result_pcd_file_path = PCD_FILE_COMPRESS_PATH
            o3d.io.write_point_cloud(result_pcd_file_path, result_pcd)

            # 持久化点云文件和记录
            task_id = self.current_task_id if hasattr(self, 'current_task_id') and self.current_task_id else 'unknown_task'
            success, record = self.point_cloud_persistor.persist_point_cloud(result_pcd_file_path, task_id)
            if success and record.get('id'):
                logger.info(f"点云文件已持久化，记录ID: {record['id']}")
                # 立即更新记录状态为上传中，避免定时器重复处理
                self.point_cloud_persistor._update_record_status(record['id'], 'uploading')
                logger.info(f"点云记录状态已更新为上传中，记录ID: {record['id']}")
            else:
                logger.error(f"点云文件持久化失败")
            
            # 7. 上传点云文件到文件服务器（带重试机制）
            file_id = self.upload_point_cloud_with_retry(result_pcd_file_path)
            if file_id:
                logger.info(f"点云文件上传成功，文件ID: {file_id}")
                # 更新记录状态为已上传
                self.point_cloud_persistor._update_record_status(record.get('id'), 'uploaded')
                # 删除本地文件
                os.remove(record.get('file_path'))
                logger.info(f'已删除本地点云文件: {record.get("file_path")}')
                # 删除记录
                self.point_cloud_persistor._remove_record(record.get('id'))
                logger.info(f'已删除点云记录: {record.get("id")}')
                return file_id, col_radar, row_radar
            else:
                logger.error("点云文件上传失败，已达到最大重试次数")  
                # 上传失败，将记录状态更新回pending，以便定时器在网络恢复后重新尝试
                self.point_cloud_persistor._update_record_status(record.get('id'), 'pending')
                logger.info(f"点云记录状态已更新为待上传，记录ID: {record.get('id')}")
                return '', 0.0, 0.0
                
        except Exception as e:
            logger.error(f"处理点云数据时出错: {str(e)}")
            return '', 0.0, 0.0

    
    def _check_network_connection(self):
        """
        检查网络连接状态
        
        Returns:
            bool: 网络连接正常返回True，否则返回False
        """
        try:
            # 尝试连接到文件服务器
            requests.get(self.server_url, timeout=5, verify=False)
            return True
        except requests.ConnectionError:
            return False
        except Exception as e:
            logger.warning(f"网络连接检查异常: {str(e)}")
            return False
    
    def upload_point_cloud_to_file_server(self, pcd_file_path, is_retry=False, task_id=None):
        """
        直接上传点云文件到文件服务器，需要先获取token
        
        Args:
            pcd_file_path: 点云文件路径    
            is_retry: 是否为重试上传，默认False
            task_id: 任务ID，用于重试上传时报告任务状态
            
        Returns:
            str: 文件ID，失败返回空字符串
        """
        try:
            # 获取上传token
            token = self.auth_manager.get_upload_token()
            if not token or token == '':
                logger.error('Failed to get upload token')
                return ''
            
            # 构建带token的上传URL
            upload_url = f'{self.server_url}/file/file/upload/{token}'
            
            # 准备上传数据，使用时间戳确保文件名唯一性
            timestamp = int(time.time() * 1000)
            unique_filename = f'point_cloud_{timestamp}.pcd'
            
            # 直接打开文件进行上传
            with open(pcd_file_path, 'rb') as f:
                files = {
                    'file': (unique_filename, f, 'application/octet-stream')
                }
                logger.info(f'准备上传点云到文件服务器: {upload_url}, 文件名={unique_filename}, 文件路径={pcd_file_path}')
                # 发送请求到文件服务器，添加verify参数控制SSL证书验证
                response = requests.post(upload_url, files=files, data={}, timeout=300, verify=False)  # 生产环境建议设置为True并提供证书
            
            if response.status_code == 200:
                result = response.json()
                # 从data对象中获取id字段
                if result.get('data') and 'id' in result['data']:
                    if is_retry:
                        logger.info(f'重试上传点云成功，文件ID: {result["data"]["id"]}')
                        # self._report_task_status(task_id, TaskStatus.COMPLETED.value, '任务已完成', result['data']['id'], 'pcd')
                    return result['data']['id']
                else:
                    logger.warning(f'响应中未找到预期的data.id字段: {result}')
                    return ''
            else:
                logger.error(f'点云上传失败: 状态码={response.status_code}, 响应={response.text}')
                return ''
                
        except requests.ConnectionError as e:
            logger.error(f'点云上传网络连接错误，请检查网络状态: {str(e)}')
            return ''
        except requests.Timeout as e:
            logger.error(f'点云上传请求超时: {str(e)}')
            return ''
        except Exception as e:
            logger.error(f'点云上传异常: {str(e)}')
            return ''
    
    def upload_point_cloud_with_retry(self, pcd_file_path):
        """
        带重试机制的点云文件上传函数
        检查网络状态，在网络恢复后重新上传，直到上传成功
        
        Args:
            pcd_file_path: 点云文件路径
            
        Returns:
            str: 文件ID，成功返回文件ID，失败返回空字符串
        """
        retry_interval = 10  # 初始重试间隔（秒）
        max_retry_interval = 60  # 最大重试间隔（秒）
        retry_count = 0
        max_retries = 1  # 最大重试次数，防止无限重试
        
        while rclpy.ok() and retry_count < max_retries:
            # 检查点云文件是否仍然存在
            if not os.path.exists(pcd_file_path):
                logger.error(f"点云文件不存在: {pcd_file_path}")
                return ''
            
            # 检查网络连接
            if not self._check_network_connection():
                logger.warning(f"网络连接不可用，将在{retry_interval}秒后重试点云上传")
                time.sleep(retry_interval)
                # 指数退避策略
                retry_interval = min(retry_interval * 2, max_retry_interval)
                retry_count += 1
                continue
            
            # 尝试上传点云文件
            file_id = self.upload_point_cloud_to_file_server(pcd_file_path)
            if file_id:
                return file_id
            
            # 上传失败，等待后重试
            logger.warning(f"点云上传失败，将在{retry_interval}秒后重试")
            time.sleep(retry_interval)
            # 指数退避策略
            retry_interval = min(retry_interval * 2, max_retry_interval)
            retry_count += 1
        
        logger.error(f"点云上传已达到最大重试次数（{max_retries}次），请检查网络和服务器状态")
        return ''
    
    def shutdown(self):
        """
        关闭巡检任务管理器，清理资源
        """
        # 停止定时任务
        if hasattr(self, 'point_cloud_persistor'):
            self.point_cloud_persistor.stop_timer()
        logger.info("点云任务管理器已关闭")
