#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import base64
import time

def image_to_base64(image_path):
    """
    从文件读取图片并转换为base64编码字符串
    """
    try:
        with open(image_path, 'rb') as img_file:
            return base64.b64encode(img_file.read()).decode('utf-8')
    except Exception as e:
        print(f"读取图片失败: {e}")
        # 返回一个简单的1x1像素黑色图片的base64编码作为备选
        return "/9j/4AAQSkZJRgABAQAAJACAAAD/2wBDAAMCAgMCAgMDAwMEAwMEBQgFBQQEBQoHBwYIDAoMDAsKCwsNDhIQDQ4RDgsLEBYQERMUFRUVDA8XGBYUGBIUFRT/2wBDAQMEBAUEBQkFBQkUDQsNFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBQUFBT/wAARCAAQABADASIAAhEBAxEB/8QAFgABAQEAAAAAAAAAAAAAAAAABgcF/8QAJRAAAgEDAwIEAwQCBQAAAAAAAgMRBAUhEjESMQcUIjFBUWEyYnGBkaGxwdHwJDNygtHxI//EABQBAQAAAAAAAAAAAAAAAAAAAAD/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIRAxEAPwD1PXbsO4PwVbKZ7GxW8u1k0cV57yJ7W7V4r1iP4d2m3xw//Z"

class AlarmPublisher(Node):
    """
    告警消息发布器，用于模拟向'/rkbot/alarm'主题发送告警数据
    """
    def __init__(self):
        super().__init__('alarm_publisher')
        self.publisher_ = self.create_publisher(String, '/rkbot/alarm', 10)
        timer_period = 5.0  # 2秒发送一次
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        self.get_logger().info('告警模拟发布器已启动，将每2秒发送一次告警数据')
    
    def timer_callback(self):
        # 生成模拟告警数据
        self.count += 1
        # 使用示例图片路径，用户可以根据实际情况修改
        image_path = '/public/lgl/Finetune-Qwen2.5-VL/test1.jpg'
        image_base64 = image_to_base64(image_path)
        self.get_logger().info(f'已加载图片并转换为base64编码')
        
        alarm_data = {
            'taskId': f'17605',
            'algo_name': 'trash_detect',  # 算法名称
            'timestamp': int(time.time() * 1000),  # 毫秒级时间戳
            'pos': {
                'x': 1.0,
                'y': 1.0,
                'z': 1.0,
                'd': 1.0
            },
            'image_base64': image_base64
        }
        
        # 转换为JSON字符串
        msg = String()
        msg.data = json.dumps(alarm_data)
        
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发送告警 #{self.count}: {alarm_data["algo_name"]}')

def main(args=None):
    rclpy.init(args=args)
    alarm_publisher = AlarmPublisher()
    
    try:
        rclpy.spin(alarm_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        alarm_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()