import time
import threading
from std_msgs.msg import String
import platform
import logging

# 获取模块logger，继承root logger的配置
logger = logging.getLogger(__name__)

class AlgoBase():
    """
    算法基类，提供线程安全的启动/停止机制和通用功能
    """
    def __init__(self, node, node_name, topic_name=None):
        """
        初始化算法基类
        
        Args:
            node (Node): ROS2节点实例
            node_name (str): 节点名称
            topic_name (str): 结果发布的话题名称，如果为None则使用默认格式
        """
        #super().__init__(node_name)
        logger.info(f"Initialized {node_name}")
           
        # 获取参数值
        self._polling_interval = 1.0
        self._thread_timeout = 3.0
        self.stats_interval = 100
        
        # 线程安全控制变量
        self._lock = threading.RLock()  # 使用可重入锁
        self._running = False
        self._thread = None
        self._last_exception = None
        self._last_heartbeat = 0
        # 是否使用RTSP流
        self._use_rtsp = False
        #self._rtsp_url = "rtsp://192.168.168.151:8554/test"
        self._rtsp_url = "rtsp://admin:Lyq%402025@192.168.168.153:554/Streaming/Channels/101"
        # 根据系统架构选择不同的模型
        system_architecture = platform.machine().lower()
        if system_architecture == 'aarch64':
            self._use_rtsp = True
        else:
            self._use_rtsp = False
        
        # 设置结果发布话题
        topic_name = topic_name or f'/algo_result/{node_name.lower()}'
        logger.info(f"Result topic: {topic_name}")
        self.pub = node.create_publisher(String, topic_name, 10)
        
        # 创建心跳定时器
        self._heartbeat_timer = node.create_timer(5.0, self._update_heartbeat)
        
        # 性能统计相关属性
        self.reset_performance_stats()

    def start(self):
        """
        线程安全地启动算法
        立即返回，不等待算法初始化完成
        """
        with self._lock:
            if not self._running:
                self._running = True
                self._last_exception = None
                self._thread = threading.Thread(target=self._run_wrapper, daemon=True)
                self._thread.start()
                logger.info("start signal sent, algorithm starting in background")
                return True
            else:
                logger.info("is already running")
                return False

    def stop(self):
        """
        线程安全地停止算法
        立即返回，不等待线程实际结束
        """
        with self._lock:
            if self._running:
                self._running = False
                logger.info("stop signal sent, will stop gracefully in background")
                return True
            else:
                logger.info("is not running")
                return False
    
    def _publish_result(self, msg):
        """
        发布分析结果到ROS2话题
        
        Args:
            msg (String): 要发布的消息
        """
        if self.pub.get_subscription_count() == 0:
            logger.warning(f"Topic {self.pub.topic_name} no subscribers, message not published")
        else:
            logger.info(f"Topic {self.pub.topic_name} published successfully, {self.pub.get_subscription_count()} subscribers")
        self.pub.publish(msg)


    def _should_stop(self):
        """
        检查算法是否应该停止运行
        
        注意：此处不使用锁保护，因为：
        1. 在Python中布尔值的读取是原子操作
        2. 移除锁可以避免stop()方法调用join()时可能发生的死锁
        3. _running标志仅从stop()方法中设置为False，这里只是读取操作
        """
        return not self._running

    def _run_wrapper(self):
        """
        运行包装器，用于捕获和记录异常
        """
        try:
            self.run_analysis()
        except Exception as e:
            logger.error(f"encountered an error: {str(e)}")
            with self._lock:
                self._running = False
                self._last_exception = e

    def run_analysis(self):
        """
        算法分析主循环，子类需要实现此方法
        """
        while not self._should_stop():
            try:
                # 子类应该重写此方法以实现具体的算法逻辑
                self.process()
                time.sleep(self._polling_interval)
            except Exception as e:
                logger.error(f"Error in analysis: {str(e)}")
                # 短暂暂停后继续，避免快速失败
                time.sleep(0.1)

    def process(self):
        """
        算法处理逻辑，返回处理结果
        子类应该重写此方法
        """
        logger.warning("Child should implement this method")
        return None

    def get_state(self):
        """
        获取算法当前状态
        
        Returns:
            dict: 包含算法状态的字典
        """
        with self._lock:
            return {
                'running': self._running,
                'has_exception': self._last_exception is not None,
                'last_exception': str(self._last_exception) if self._last_exception else None,
                'last_heartbeat': self._last_heartbeat,
                'thread_alive': self._thread.is_alive() if self._thread else False
            }

    def _update_heartbeat(self):
        """
        更新心跳时间戳，用于监控算法是否正常运行
        """
        with self._lock:
            if self._running:
                self._last_heartbeat = time.time()
                # 如果线程已死但状态仍为running，自动重置状态
                if self._thread and not self._thread.is_alive():
                    logger.warning("thread is dead but state is running, resetting state")
                    self._running = False
                    self._thread = None

    def __del__(self):
        """
        析构函数，确保线程正确停止
        """
        self.stop()

    def shutdown(self):
        """
        关闭算法，释放资源
        """
        self.stop()
        self.destroy_node()
        
    def reset_performance_stats(self):
        """
        重置性能统计数据
        """
        self.process_count = 0  # 处理次数计数器
        self.total_inference_time = 0.0  # 总推理时间
        self.total_tracking_time = 0.0  # 总跟踪时间
        self.total_process_time = 0.0  # 总处理时间
        
    def start_timer(self):
        """
        通用的开始计时方法，可用于任何需要计时的操作
        
        Returns:
            float: 开始时间戳
        """
        return time.time()
    
    def end_timer(self, start_time, timer_type):
        """
        通用的结束计时方法，根据类型累加相应的时间
        
        Args:
            start_time (float): 开始时间戳
            timer_type (str): 计时器类型，可以是 'process', 'inference' 或 'tracking'
            
        Returns:
            float: 本次操作花费的时间（秒）
        """
        time_taken = time.time() - start_time
        
        # 根据类型累加时间
        if timer_type == 'inference':
            self.total_inference_time += time_taken
        elif timer_type == 'tracking':
            self.total_tracking_time += time_taken
        elif timer_type == 'process':
            self.total_process_time += time_taken
            self.process_count += 1
            # 处理时间结束时，检查是否需要打印统计信息
            if self.process_count % self.stats_interval == 0:
                self.log_performance_stats()
                self.reset_performance_stats()
        
        return time_taken
    
    def log_performance_stats(self):
        """
        打印性能统计信息
        """
        if self.process_count > 0:
            avg_inference = self.total_inference_time / self.process_count * 1000
            avg_tracking = self.total_tracking_time / self.process_count * 1000
            avg_process = self.total_process_time / self.process_count * 1000
            logger.info(
                f"{self.__class__.__name__} Performance Stats: "
                f"Avg Inference: {avg_inference:.2f}ms, "
                f"Avg Tracking: {avg_tracking:.2f}ms, "
                f"Avg Total: {avg_process:.2f}ms"
            )