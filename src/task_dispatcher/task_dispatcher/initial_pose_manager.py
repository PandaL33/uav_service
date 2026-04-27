#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
初始化定位信息管理模块
负责处理机器人定位信息的初始化命令
"""

import math
import logging
from typing import Dict, Any
import tf_transformations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
import time
import json

logger = logging.getLogger(__name__)

class InitialPoseManager:
    """
    初始化定位信息管理器
    处理机器人定位信息的初始化命令
    """
    
    def __init__(self, node, mqtt_client, response_topic):
        """
        初始化定位管理器
        
        Args:
            node: ROS2节点实例
            
        """
        self.node = node
        self.mqtt_client = mqtt_client
        self.mqtt_response_topic = response_topic

        # 创建目标位置发布者
        self._goal_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        logger.info("初始化定位管理器已启动")
    
    def process_initial_pose(self, message_data) -> Dict[str, Any]:
        """
        处理初始化定位信息命令
        
        Args:
            message_data: 包含初始化定位信息的命令数据
            
        Returns:
            Dict[str, Any]: 响应数据
        """
        try:
            # 获取请求体中的位置信息
            body = message_data.get('body', {})
            pos = body.get('pos', {})
            
            if not pos:
                logger.error("初始化定位失败：未提供位置信息")
                return self.send_ack_response(message_data, 400, "未提供位置信息")
            
            # 构建并发布PoseStamped消息
            self._publish_initial_pose(pos)
            
            logger.info(f"成功初始化机器人定位，位置: {pos}")
            # 返回成功响应
            return self.send_ack_response(message_data, 200, "初始化定位成功")
            
        except Exception as e:
            logger.error(f"处理初始化定位命令时出错: {str(e)}")
            return self.send_ack_response(message_data, 500, "处理初始化定位命令时出错")
    
    def _publish_initial_pose(self, pos: Dict[str, float]):
        """
        发布初始化位置信息
        
        Args:
            pos: 包含x, y, z, d的位置字典
        """
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = pos.get("x", 0.0)
        pose_msg.pose.pose.position.y = pos.get("y", 0.0)
        pose_msg.pose.pose.position.z = 0.0 # pos.get("z", 0.0)

        # 处理角度信息，转换为四元数
        yaw = pos.get("d", 0.0)
        x, y, z, w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

        pose_msg.pose.pose.orientation.x = x
        pose_msg.pose.pose.orientation.y = y
        pose_msg.pose.pose.orientation.z = z
        pose_msg.pose.pose.orientation.w = w
        
        covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.25, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.25
        ]
        pose_msg.pose.covariance = covariance
        
        # 记录日志
        self.node.get_logger().info(f"publish initialpose: {pose_msg} angle={math.degrees(yaw):.2f}")
        # 发布消息
        self._goal_pose_pub.publish(pose_msg)
    
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
    
    def shutdown(self):
        """
        关闭资源
        """
        if hasattr(self, '_goal_pose_pub'):
            self.node.destroy_publisher(self._goal_pose_pub)
        logger.info("初始化定位管理器已关闭")