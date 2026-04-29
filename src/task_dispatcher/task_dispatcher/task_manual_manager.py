#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
手动任务管理器 - 独立脚本版本
用于在不依赖ROS2节点的情况下执行任务状态上报和点云上传功能

使用方法:
    python3 task_manual.manager.py --task-id <task_id> --action <action> [options]
    
示例:
    # 上报任务状态
    python3 task_manual.manager.py --task-id task_001 --action report-status --status 1 --message "任务进行中"
    
    # 上传点云文件
    python3 task_manual.manager.py --task-id task_001 --action upload-pcd --pcd-file /path/to/file.pcd
"""

import json
import logging
import time
import os
import sys
import uuid
import requests
from typing import Optional
from requests.auth import HTTPBasicAuth

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('task_manual_manager')


class TaskManualManager:
    """
    手动任务管理器
    提供独立的任务状态上报和点云上传功能
    """
    
    def __init__(self, server_url: str, robot_sn: str = 'UAV00002', version: str = '1.0'):
        """
        初始化手动任务管理器
        
        Args:
            server_url: 服务器URL
            robot_sn: 机器人序列号
            version: 版本号
        """
        self.server_url = server_url.rstrip('/')
        self.robot_sn = robot_sn
        self.version = version
        
        # 认证配置
        self.auth_config = {
            'USERNAME': 'robot-manage',
            'PASSWORD': '123',
            'TOKEN_ENDPOINT': '/file/provider/v1/file/getUploadTaskToken'
        }
        
        logger.info(f'手动任务管理器初始化完成，服务器地址: {self.server_url}')
    
    def _get_upload_token(self, file_properties: int = 0) -> str:
        """
        获取文件上传Token
        
        Args:
            file_properties: 文件属性参数，默认为0
            
        Returns:
            str: 成功返回Token字符串，失败返回空字符串
        """
        try:
            url = f"{self.server_url}{self.auth_config['TOKEN_ENDPOINT']}?fileProperties={file_properties}"
            username = self.auth_config['USERNAME']
            password = self.auth_config['PASSWORD']
            
            logger.info(f"正在请求获取上传Token，URL: {url}")
            
            response = requests.get(
                url,
                auth=HTTPBasicAuth(username, password),
                timeout=5,
                verify=False
            )
            
            data = response.json()
            logger.info(f'获取Token响应数据: {data}')
            
            if data.get('code') == 200:
                token = data.get('data', {}).get('uploadToken', '')
                if token:
                    logger.info("成功获取上传Token")
                    return token
                else:
                    logger.warning("响应Code为200，但Token字段为空")
            else:
                logger.error(f"获取Token失败，业务错误码: {data.get('code')}, 信息: {data.get('message')}")
                
            return ''
            
        except requests.exceptions.Timeout:
            logger.error("获取上传Token请求超时 (Timeout)")
            return ''
        except requests.exceptions.RequestException as e:
            logger.error(f"获取上传Token网络请求异常: {str(e)}")
            return ''
        except json.JSONDecodeError as e:
            logger.error(f"响应数据JSON解析失败: {str(e)}")
            return ''
        except Exception as e:
            logger.error(f"获取上传Token发生未知错误: {str(e)}")
            return ''
    
    def report_task_status(self, task_id: str, task_progress: int, 
                          message: str = '', file_id: str = '', 
                          file_type: str = '', mqtt_client=None, 
                          mqtt_response_topic: str = ''):
        """
        上报任务执行状态
        
        Args:
            task_id: 任务ID
            task_progress: 任务执行进度：0-启动、1-进行中、2-完成、3-中断、4-取消
            message: 消息内容
            file_id: 文件ID（如果有）
            file_type: 文件类型（如果有）
            mqtt_client: MQTT客户端实例（可选，如果提供则通过MQTT上报）
            mqtt_response_topic: MQTT响应主题（可选）
            
        Returns:
            dict: 上报的状态消息
        """
        try:
            # 构建状态上报消息
            status_msg = {
                'sn': self.robot_sn,
                'ver': self.version,
                'seq': str(uuid.uuid4()).replace('-', ''),
                'type': 1,
                'ts': int(time.time() * 1000),
                'cmd': 'ActionReport',
                'body': {
                    'taskId': task_id,
                    'pos': {},  # 位置信息，可根据需要填充
                    'message': message if message else 'ok',
                    'taskProgress': task_progress,
                    'tsReport': int(time.time() * 1000),
                    'fileId': file_id,
                    'fileType': file_type,
                }
            }
            
            # 如果提供了MQTT客户端，则发布到MQTT
            if mqtt_client and mqtt_response_topic:
                mqtt_client.publish(mqtt_response_topic, json.dumps(status_msg))
                logger.info(f'已通过MQTT上报任务状态: taskId={task_id}, taskProgress={task_progress}')
            else:
                # 否则打印到控制台
                logger.info(f'任务状态上报信息:')
                logger.info(f'  任务ID: {task_id}')
                logger.info(f'  进度: {task_progress}')
                logger.info(f'  消息: {message}')
                logger.info(f'  文件ID: {file_id}')
                logger.info(f'  文件类型: {file_type}')
                logger.info(f'  完整消息: {json.dumps(status_msg, indent=2)}')
            
            return status_msg
            
        except Exception as e:
            logger.error(f'上报任务状态失败: {str(e)}')
            return None
    
    def _check_network_connection(self) -> bool:
        """
        检查网络连接状态
        
        Returns:
            bool: 网络连接正常返回True，否则返回False
        """
        try:
            requests.get(self.server_url, timeout=5, verify=False)
            return True
        except requests.ConnectionError:
            return False
        except Exception as e:
            logger.warning(f"网络连接检查异常: {str(e)}")
            return False
    
    def upload_point_cloud_to_file_server(self, pcd_file_path: str, 
                                         is_retry: bool = False, 
                                         task_id: str = None) -> str:
        """
        直接上传点云文件到文件服务器
        
        Args:
            pcd_file_path: 点云文件路径
            is_retry: 是否为重试上传，默认False
            task_id: 任务ID，用于重试上传时报告任务状态
            
        Returns:
            str: 文件ID，失败返回空字符串
        """
        try:
            # 获取上传token
            token = self._get_upload_token()
            
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
                # 发送请求到文件服务器
                response = requests.post(upload_url, files=files, data={}, timeout=300, verify=False)
            
            if response.status_code == 200:
                result = response.json()
                # 从data对象中获取id字段
                if result.get('data') and 'id' in result['data']:
                    file_id = result['data']['id']
                    if is_retry:
                        logger.info(f'重试上传点云成功，文件ID: {file_id}')
                        # 如果是重试，可以再次上报任务状态
                        if task_id:
                            self.report_task_status(
                                task_id, 
                                2,  # COMPLETED
                                '任务已完成', 
                                file_id, 
                                'pcd'
                            )
                    return file_id
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
            logger.error(f'点云上传超时: {str(e)}')
            return ''
        except Exception as e:
            logger.error(f'点云上传异常: {str(e)}')
            return ''
    
    def upload_point_cloud_with_retry(self, pcd_file_path: str, 
                                     task_id: str = '',
                                     max_retries: int = 1) -> str:
        """
        带重试机制的点云文件上传函数
        检查网络状态，在网络恢复后重新上传，直到上传成功或达到最大重试次数
        
        Args:
            pcd_file_path: 点云文件路径
            task_id: 任务ID
            max_retries: 最大重试次数，默认1次
            
        Returns:
            str: 文件ID，成功返回文件ID，失败返回空字符串
        """
        retry_interval = 10  # 初始重试间隔（秒）
        max_retry_interval = 60  # 最大重试间隔（秒）
        retry_count = 0
        
        logger.info(f'开始上传点云文件: {pcd_file_path}, 任务ID: {task_id}')
        
        while retry_count <= max_retries:
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
            file_id = self.upload_point_cloud_to_file_server(
                pcd_file_path, 
                is_retry=(retry_count > 0),
                task_id=task_id
            )
            
            if file_id:
                logger.info(f'点云文件上传成功，文件ID: {file_id}')
                return file_id
            
            # 上传失败，等待后重试
            if retry_count < max_retries:
                logger.warning(f"点云上传失败，将在{retry_interval}秒后重试 (第{retry_count + 1}/{max_retries}次)")
                time.sleep(retry_interval)
                # 指数退避策略
                retry_interval = min(retry_interval * 2, max_retry_interval)
            
            retry_count += 1
        
        logger.error(f"点云上传已达到最大重试次数（{max_retries}次），请检查网络和服务器状态")
        return ''


def parse_arguments():
    """
    解析命令行参数
    
    Returns:
        argparse.Namespace: 解析后的参数
    """
    import argparse
    
    parser = argparse.ArgumentParser(
        description='手动任务管理器 - 用于任务状态上报和点云上传',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 上报任务状态（启动）
  python3 task_manual.manager.py --server-url https://example.com --robot-sn ROBOT001 --task-id task_001 --action report-status --status 0
  
  # 上报任务状态（进行中）
  python3 task_manual.manager.py --server-url https://example.com --robot-sn ROBOT001 --task-id task_001 --action report-status --status 1 --message "任务进行中"
  
  # 上报任务状态（完成）并关联文件
  python3 task_manual.manager.py --server-url https://example.com --robot-sn ROBOT001 --task-id task_001 --action report-status --status 2 --file-id file123 --file-type pcd
  
  # 上传点云文件
  python3 task_manual.manager.py --server-url https://example.com --robot-sn ROBOT001 --task-id task_001 --action upload-pcd --pcd-file /path/to/file.pcd
  
  # 上传点云文件（带重试）
  python3 task_manual.manager.py --server-url https://example.com --robot-sn ROBOT001 --task-id task_001 --action upload-pcd --pcd-file /path/to/file.pcd --max-retries 3
        """
    )
    
    # 通用参数
    parser.add_argument('--server-url', type=str, default='https://127.0.0.181',
                       help='服务器URL，默认: https://127.0.0.1:81')
    parser.add_argument('--robot-sn', type=str, default='UAV00002',
                       help='机器人序列号，默认: UAV00002')
    parser.add_argument('--version', type=str, default='1.0',
                       help='版本号，默认: 1.0')
    
    # 动作类型
    parser.add_argument('--action', type=str, required=True,
                       choices=['report-status', 'upload-pcd'],
                       help='执行的动作: report-status(上报状态), upload-pcd(上传点云)')
    
    # 任务ID（两个动作都需要）
    parser.add_argument('--task-id', type=str, required=True,
                       help='任务ID')
    
    # report-status 专用参数
    parser.add_argument('--status', type=int, choices=[0, 1, 2, 3, 4],
                       help='任务状态: 0-启动, 1-进行中, 2-完成, 3-中断, 4-取消')
    parser.add_argument('--message', type=str, default='',
                       help='状态消息')
    parser.add_argument('--file-id', type=str, default='',
                       help='文件ID')
    parser.add_argument('--file-type', type=str, default='',
                       help='文件类型，如: pcd, jpg, mp4')
    
    # upload-pcd 专用参数
    parser.add_argument('--pcd-file', type=str, default='/home/cat/slam_data/pcd/test.pcd',
                       help='点云文件路径，默认: /home/cat/slam_data/pcd/test.pcd')
    parser.add_argument('--max-retries', type=int, default=1,
                       help='最大重试次数，默认: 1')
    
    args = parser.parse_args()
    
    # 验证参数
    if args.action == 'report-status' and args.status is None:
        parser.error('--status 参数在 report-status 动作中是必需的')
    
    if args.action == 'upload-pcd' and not os.path.exists(args.pcd_file):
        parser.error(f'点云文件不存在: {args.pcd_file}')
    
    return args


def main():
    """
    主函数
    """
    args = parse_arguments()
    
    # 创建管理器实例
    manager = TaskManualManager(
        server_url=args.server_url,
        robot_sn=args.robot_sn,
        version=args.version
    )
    
    try:
        if args.action == 'report-status':
            # 上报任务状态
            logger.info(f'执行动作: 上报任务状态')
            result = manager.report_task_status(
                task_id=args.task_id,
                task_progress=args.status,
                message=args.message,
                file_id=args.file_id,
                file_type=args.file_type
            )
            
            if result:
                logger.info('任务状态上报成功')
                print(json.dumps(result, indent=2, ensure_ascii=False))
                sys.exit(0)
            else:
                logger.error('任务状态上报失败')
                sys.exit(1)
                
        elif args.action == 'upload-pcd':
            # 上传点云文件
            logger.info(f'执行动作: 上传点云文件')
            file_id = manager.upload_point_cloud_with_retry(
                pcd_file_path=args.pcd_file,
                task_id=args.task_id,
                max_retries=args.max_retries
            )
            
            if file_id:
                logger.info(f'点云文件上传成功，文件ID: {file_id}')
                print(json.dumps({'success': True, 'file_id': file_id}, indent=2))
                sys.exit(0)
            else:
                logger.error('点云文件上传失败')
                print(json.dumps({'success': False, 'error': '上传失败'}, indent=2))
                sys.exit(1)
    
    except KeyboardInterrupt:
        logger.info('用户中断操作')
        sys.exit(130)
    except Exception as e:
        logger.error(f'执行失败: {str(e)}', exc_info=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
