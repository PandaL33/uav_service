# 首先配置日志，确保在导入其他模块前设置好
import logging
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(name)s] [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

# 然后导入其他模块
import rclpy
import time
from rclpy.node import Node
#from rclpy.logging import LoggingSeverity
from std_msgs.msg import String
from multi_algo_interfaces.srv import AlgoControl
from multi_algo_manager.algos.trash_detect import TrashDetect
from multi_algo_manager.algos.device_check import DeviceCheck
from multi_algo_manager.algos.channel_monitor import ChannelMonitor
from multi_algo_manager.algos.line_integrity import LineIntegrity
from multi_algo_manager.algos.gpio_control import GpioControl

logger = logging.getLogger(__name__)

class AlgoManagerNode(Node):
    def __init__(self):
        super().__init__('algo_manager_node')
        
        # 正确设置ROS2日志级别.系统默认INFO
        #self.get_logger().set_level(LoggingSeverity.INFO)
        logger.setLevel(logging.INFO)

        # 算法实例，默认不启动
        self.algos = {
            'TrashDetect': TrashDetect(self),
            'DeviceCheck': DeviceCheck(self),
            'ChannelMonitor': ChannelMonitor(self),
            'LineIntegrity': LineIntegrity(self),
            'GpioControl': GpioControl(self),
        }
        
        logger.info(f"Initialized {len(self.algos)} algorithms")

        # Service 控制
        self.control_service = self.create_service(
            AlgoControl,
            'algo_control',
            self.handle_algo_control
        )
        logger.info("started with service: algo_control")

        # 统一结果发布 Topic
        self.result_pub = self.create_publisher(String, 'algo_result', 10)
        self.timer = self.create_timer(1.0, self.publish_results)

    def handle_algo_control(self, request, response):
        algo_name = request.algo_name
        action = request.action.lower()
        algo = self.algos.get(algo_name)

        if not algo:
            response.success = False
            response.message = f"No such algorithm: {algo_name}"
            logger.warning(response.message)
            return response

        if action == 'start':
            algo.start()
            response.success = True
            response.message = f"{algo_name} started"
            logger.info(response.message)
        elif action == 'stop':
            algo.stop()
            response.success = True
            response.message = f"{algo_name} stopped"
            logger.info(response.message)
        else:
            response.success = False
            response.message = f"Unknown action: {action}"
            logger.warning(response.message)
        return response

    def publish_results(self):
        # 获取所有算法的运行状态
        # running_algos = 0
        # for algo_name, algo in self.algos.items():
        #     result = algo.process()
        #     if result:
        #         msg = String()
        #         msg.data = result
        #         self.result_pub.publish(msg)
        #         running_algos += 1
        
        # 每分钟打印一次运行状态摘要
        if int(time.time()) % 60 == 0:
            logger.info(f"Current status: algorithms service running")

def main(args=None):
    rclpy.init(args=args)
    node = AlgoManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
