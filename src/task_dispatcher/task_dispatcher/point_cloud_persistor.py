#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import time
import threading
import logging
import shutil
import requests
from typing import Dict, List, Optional

# 配置日志
logger = logging.getLogger('task_dispatcher.point_cloud_persistor')

class PointCloudPersistor:
    """
    点云数据持久化和定时上传管理器
    
    功能：
    1. 当点云上传失败时，将点云文件保存到本地持久化目录
    2. 记录任务ID和点云路径到持久化存储
    3. 定时检查未上传的点云数据并重新上传
    4. 上传成功后，删除本地文件和记录
    """
    
    def __init__(self, node, server_url):
        """
        初始化点云持久化管理器
        
        Args:
            node: ROS2节点实例
            server_url: 文件服务器URL
        """
        self.node = node
        self.server_url = server_url
        
        # 点云持久化目录
        self.persist_dir = os.path.join(os.path.dirname(__file__), 'persisted_point_clouds')
        # 持久化记录文件
        self.records_file = os.path.join(self.persist_dir, 'upload_records.json')
        
        # 创建持久化目录
        self._ensure_persist_dir_exists()
        
        # 线程锁，用于保护共享资源
        self._lock = threading.Lock()
        
        # 定时任务状态
        self._is_running = False
        self._timer_thread = None
        
        # 启动定时任务
        self.start_timer()
    
    def _ensure_persist_dir_exists(self):
        """
        确保持久化目录存在
        """
        try:
            if not os.path.exists(self.persist_dir):
                os.makedirs(self.persist_dir)
                logger.info(f'创建点云持久化目录: {self.persist_dir}')
            
            # 确保记录文件存在
            if not os.path.exists(self.records_file):
                with open(self.records_file, 'w') as f:
                    json.dump([], f)
        except Exception as e:
            logger.error(f'创建点云持久化目录失败: {str(e)}')
    
    def persist_point_cloud(self, pcd_file_path: str, task_id: str) -> bool:
        """
        持久化点云文件和记录
        
        Args:
            pcd_file_path: 原始点云文件路径
            task_id: 任务ID
            
        Returns:
            bool: 持久化成功返回True，否则返回False
        """
        try:
            # 检查原始文件是否存在
            if not os.path.exists(pcd_file_path):
                logger.error(f'原始点云文件不存在: {pcd_file_path}')
                return False
            
            # 生成持久化文件名
            timestamp = int(time.time() * 1000)
            filename = os.path.basename(pcd_file_path)
            persist_filename = f'{task_id}_{timestamp}_{filename}'
            persist_path = os.path.join(self.persist_dir, persist_filename)
            
            # 拷贝文件到持久化目录
            shutil.copy2(pcd_file_path, persist_path)
            logger.info(f'点云文件已持久化: {persist_path}')
            
            # 记录任务ID和点云路径
            record = {
                'id': str(timestamp),
                'task_id': task_id,
                'file_path': persist_path,
                'created_time': timestamp,
                'status': 'pending'  # pending, uploading, uploaded, failed
            }
            
            self._add_record(record)
            logger.info(f'点云记录已持久化: 任务ID={task_id}, 文件路径={persist_path}')
            
            return True, record
        except Exception as e:
            logger.error(f'持久化点云文件失败: {str(e)}')
            return False, {}
    
    def _add_record(self, record: Dict):
        """
        添加记录到持久化存储
        
        Args:
            record: 记录字典
        """
        with self._lock:
            try:
                # 读取现有记录
                with open(self.records_file, 'r') as f:
                    records = json.load(f)
                
                # 添加新记录
                records.append(record)
                
                # 保存更新后的记录
                with open(self.records_file, 'w') as f:
                    json.dump(records, f, indent=2)
            except Exception as e:
                logger.error(f'添加点云记录失败: {str(e)}')
    
    def _get_pending_records(self) -> List[Dict]:
        """
        获取所有待上传的记录
        
        Returns:
            List[Dict]: 待上传记录列表
        """
        with self._lock:
            try:
                with open(self.records_file, 'r') as f:
                    records = json.load(f)
                
                # 过滤出待上传的记录
                pending_records = [r for r in records if r.get('status') == 'pending']
                return pending_records
            except Exception as e:
                logger.error(f'获取待上传记录失败: {str(e)}')
                return []
    
    def _update_record_status(self, record_id: str, status: str):
        """
        更新记录状态
        
        Args:
            record_id: 记录ID
            status: 新状态
        """
        with self._lock:
            try:
                with open(self.records_file, 'r') as f:
                    records = json.load(f)
                
                # 更新记录状态
                for record in records:
                    if record.get('id') == record_id:
                        record['status'] = status
                        record['updated_time'] = int(time.time() * 1000)
                        break
                
                # 保存更新后的记录
                with open(self.records_file, 'w') as f:
                    json.dump(records, f, indent=2)
            except Exception as e:
                logger.error(f'更新记录状态失败: {str(e)}')
    
    def _remove_record(self, record_id: str):
        """
        删除记录
        
        Args:
            record_id: 记录ID
        """
        with self._lock:
            try:
                with open(self.records_file, 'r') as f:
                    records = json.load(f)
                
                # 过滤掉要删除的记录
                updated_records = [r for r in records if r.get('id') != record_id]
                
                # 保存更新后的记录
                with open(self.records_file, 'w') as f:
                    json.dump(updated_records, f, indent=2)
            except Exception as e:
                logger.error(f'删除记录失败: {str(e)}')
    
    def _upload_pending_records(self):
        """
        上传所有待上传的点云文件
        """
        logger.info('开始检查待上传点云数据')
        
        # 获取待上传记录
        pending_records = self._get_pending_records()
        
        if not pending_records:
            logger.info('没有待上传的点云数据')
            return
        
        logger.info(f'发现{len(pending_records)}条待上传的点云数据')
        
        for record in pending_records:
            record_id = record.get('id')
            file_path = record.get('file_path')
            task_id = record.get('task_id')
            
            if not file_path or not os.path.exists(file_path):
                logger.error(f'点云文件不存在: {file_path}，记录ID: {record_id}')
                self._remove_record(record_id)
                continue
            
            try:
                # 检查网络连接
                if not self._check_network_connection():
                    logger.warning(f'网络连接不可用，跳过上传点云文件: {file_path}')
                    continue
                
                # 更新记录状态为上传中
                self._update_record_status(record_id, 'uploading')
                
                # 尝试上传文件
                logger.info(f'尝试上传点云文件: {file_path}，记录ID: {record_id}')
                file_id = self._upload_point_cloud(file_path, is_retry=True, task_id=task_id)
                
                if file_id and file_id != '':
                    logger.info(f'点云文件上传成功: {file_path}，文件ID: {file_id}')
                    
                    # 更新记录状态为已上传
                    self._update_record_status(record_id, 'uploaded')
                    
                    # 删除本地文件
                    os.remove(file_path)
                    logger.info(f'已删除本地点云文件: {file_path}')
                    
                    # 删除记录
                    self._remove_record(record_id)
                    logger.info(f'已删除点云记录: {record_id}')
                else:
                    logger.error(f'点云文件上传失败: {file_path}')
                    self._update_record_status(record_id, 'pending')
            except Exception as e:
                logger.error(f'上传点云文件异常: {str(e)}，记录ID: {record_id}')
                self._update_record_status(record_id, 'pending')
    
    def _upload_point_cloud(self, pcd_file_path: str, is_retry: bool = False) -> Optional[str]:
        """
        上传点云文件到文件服务器
        
        Args:
            pcd_file_path: 点云文件路径
            is_retry: 是否为重试上传，默认False
            
        Returns:
            Optional[str]: 文件ID，上传失败返回None
        """
        try:
            # 这里需要从外部获取token
            # 由于这个类是在CruiseTaskManager中使用的，我们可以考虑将token获取方法作为参数传入
            # 或者在CruiseTaskManager中调用这个方法时提供token
            logger.warning('当前实现中缺少token获取逻辑，请在实际使用时完善')
            
            # 临时返回None，表示上传失败
            return None
        except Exception as e:
            logger.error(f'上传点云文件失败: {str(e)}')
            return None
    
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

    def set_upload_callback(self, upload_callback):
        """
        设置上传回调函数
        
        Args:
            upload_callback: 上传回调函数，接收点云文件路径，返回文件ID
        """
        self._upload_point_cloud = upload_callback
    
    def start_timer(self, interval: int = 60):
        """
        启动定时任务
        
        Args:
            interval: 检查间隔（秒），默认60秒
        """
        if self._is_running:
            return
        
        self._is_running = True
        self._check_interval = interval
        
        def timer_func():
            while self._is_running:
                try:
                    self._upload_pending_records()
                except Exception as e:
                    logger.error(f'定时任务执行异常: {str(e)}')
                
                # 等待下一次检查
                for _ in range(self._check_interval):
                    if not self._is_running:
                        break
                    time.sleep(1)
        
        # 启动定时任务线程
        self._timer_thread = threading.Thread(target=timer_func, daemon=True)
        self._timer_thread.start()
        logger.info(f'点云定时上传任务已启动，检查间隔: {interval}秒')
    
    def stop_timer(self):
        """
        停止定时任务
        """
        if not self._is_running:
            return
        
        self._is_running = False
        if self._timer_thread:
            self._timer_thread.join(timeout=5)
        logger.info('点云定时上传任务已停止')
    
    def shutdown(self):
        """
        关闭持久化管理器
        """
        self.stop_timer()
        logger.info('点云持久化管理器已关闭')

# 测试代码
if __name__ == '__main__':
    # 配置日志
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    
    # 创建测试实例
    persistor = PointCloudPersistor(None, 'https://112.48.30.69:19000')
    
    # 测试持久化功能
    # 注意：需要确保有一个测试点云文件存在
    test_file = '/public/lgl/00292.pcd'
    test_task_id = 'test_task_123'
    
    if os.path.exists(test_file):
        print(f'测试文件存在，开始持久化: {test_file}')
        persistor.persist_point_cloud(test_file, test_task_id)
        print('持久化完成')
    
    # 等待定时任务执行
    

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        persistor.shutdown()