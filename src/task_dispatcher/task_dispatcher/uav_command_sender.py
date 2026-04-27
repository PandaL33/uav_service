import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import logging


# 配置日志
logger = logging.getLogger('task_dispatcher')

class UavCommandSender:
    """
    轻量级命令发送器。
    """
    
    def __init__(self, node: Node, topic_name: str = '/uav_command'):
        self.node = node
        self.topic_name = topic_name
        
        # 定义 QoS 策略 (可靠传输)
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.publisher = self.node.create_publisher(String, self.topic_name, self.qos_profile)

    def send_command(self, command_data: str, wait_for_subscribers: bool = True, timeout: float = 2.0) -> bool:
        """
        发送命令。
        使用 time.sleep 轮询等待订阅者，完全避开 ROS 执行器。
        """
        try:

            logger.info(f"准备发送 '{command_data}' 到话题 '{self.topic_name}'")

            # 2. 等待订阅者 
            if wait_for_subscribers:
                start_time = time.time()
                while self.publisher.get_subscription_count() == 0:
                    if time.time() - start_time > timeout:
                        logger.warning(f"等待订阅者超时 ({timeout}s)。将尝试发送。")
                        break

                    time.sleep(0.05) 
                
                if self.publisher.get_subscription_count() > 0:
                    logger.info("订阅者已就绪。")
                else:
                    logger.warning("未检测到订阅者。请确认对方节点已启动。")

            # 3. 发布消息
            msg = String()
            msg.data = command_data
            self.publisher.publish(msg)
            
            # 4. 短暂休眠，确保数据发出
            time.sleep(0.05)
            
            logger.info("命令发送成功。")
            return True

        except Exception as e:
            logger.error(f"发送失败: {e}", exc_info=True)
            return False
            
            