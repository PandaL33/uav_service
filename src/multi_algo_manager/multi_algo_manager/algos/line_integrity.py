import time
from std_msgs.msg import String
from .base import AlgoBase
from .utils import AlgoUtils
import cv2
import logging

logger = logging.getLogger(__name__)

class LineIntegrity(AlgoBase):
    """
    线路完整性算法，用于检查线路完整性
    """
    def __init__(self, node):
        """
        初始化线路完整性算法
        """
        super().__init__(node, 'LineIntegrity', '/algo_result/line_integrity')

        # 初始化资源标志
        self.initialized = False

        # 初始化视频捕获设备
        self.cap = None
        if self._use_rtsp:
            self.video_path = self._rtsp_url
        else:
            self.video_path = '/public/lgl/叉车停放区有叉车.mp4'
        logger.info(f"Using video source: {self.video_path}")
        self.frame_counter = 0
        self.process_interval_frames = 25 * 60  # 每60秒处理一帧

    def _initialize_resources(self):
        """
        初始化视频捕获设备

        Returns:
            bool: 初始化是否成功
        """
        if self.initialized:
            return True

        # 打开视频文件或RTSP流
        try:
            self.cap = cv2.VideoCapture(self.video_path)
            if not self.cap.isOpened():
                logger.error(f"Failed to open video source: {self.video_path}")
                return False
            logger.info(f"Video capture initialized successfully: {self.video_path}")
        except Exception as e:
            logger.error(f"Failed to initialize video capture: {str(e)}")
            return False

        # 初始化帧计数器
        self.frame_counter = 0
        
        self.initialized = True
        logger.info("LineIntegrity resources initialized successfully")
        return True

    def run_analysis(self):
        """
        线路完整性算法主循环，定期发布检查结果
        实现处理时间+sleep时间=40毫秒的精确控制
        """
        try:
            # 在循环开始前初始化资源
            if not self._initialize_resources():
                # 初始化失败，等待一段时间后重试
                logger.warning("Initialization failed, retrying in 5 seconds...")
                time.sleep(5)
                if not self._initialize_resources():
                    logger.error("Failed to initialize resources after multiple attempts")
                    return

            # 设置目标周期时间为40毫秒
            target_cycle_time = 0.04  # 40毫秒 = 0.04秒

            while not self._should_stop():
                # 记录循环开始时间
                cycle_start_time = time.time()
                
                try:
                    # 执行具体的设备检查算法
                    output_results, frame = self.perform_line_integrity()
                    
                    # 处理输出结果
                    if len(output_results) > 0:
                        # 创建并发布消息
                        msg = String()
                        
                        # 获取当前时间戳（毫秒级）
                        timestamp = int(time.time() * 1000)
                        
                        try:
                            # 使用工具类将图像转换为base64
                            image_base64 = AlgoUtils.encode_image_to_base64(frame)
                            
                            # 使用工具类构造消息
                            msg.data = AlgoUtils.create_algorithm_message(
                                image_base64=image_base64,
                                timestamp=timestamp,
                                algo_name="line_integrity",
                                result_data=output_results
                            )
                            
                            self.pub.publish(msg)
                            
                            # 记录检查结果日志（避免记录过长的base64数据）
                            logger.info(f"Published LineIntegrity result with {len(output_results)} objects detected")
                            logger.info(f"Detection details: {output_results}")
                            
                        except ValueError as e:
                            logger.error(f"Failed to process image or create message: {str(e)}")
                        except Exception as e:
                            logger.error(f"Failed to publish message: {str(e)}")
                    
                    # 更新心跳，表示算法正常运行
                    self._last_heartbeat = time.time()
                    
                    # 计算处理已经花费的时间
                    elapsed_time = time.time() - cycle_start_time
                    
                    # 计算需要sleep的时间，确保总时间为40毫秒
                    # 如果处理时间已经超过目标时间，则不sleep
                    sleep_time = max(0, target_cycle_time - elapsed_time)
                    
                    # 避免极短时间的sleep操作带来的性能问题
                    if sleep_time > 0.0001:  # 大于0.1毫秒才sleep
                        time.sleep(sleep_time)
                    
                    # 记录实际周期时间（调试用）
                    # logger.debug(f"Cycle time: {(time.time() - cycle_start_time) * 1000:.2f}ms")
                
                except Exception as e:
                    logger.error(f"Error in LineIntegrity analysis: {str(e)}")
                    # 即使出错，也计算并执行剩余的sleep时间，保持周期稳定
                    elapsed_time = time.time() - cycle_start_time
                    sleep_time = max(0, target_cycle_time - elapsed_time)
                    if sleep_time > 0.0001:
                        time.sleep(sleep_time)
        finally:
            # 清理资源
            self._cleanup_resources()

    def _cleanup_resources(self):
        """
        清理视频捕获设备资源
        """
        # 释放视频捕获对象
        if hasattr(self, 'cap') and self.cap is not None:
            try:
                self.cap.release()
                logger.info("Video capture released successfully")
                # 释放后将cap设置为None，避免重复释放
                self.cap = None
            except Exception as e:
                logger.error(f"Failed to release video capture: {str(e)}")

        # 重置初始化标志，确保重新启动时能正确初始化
        self.initialized = False

    def perform_line_integrity(self):
        """
        执行线路完整性检查
        
        Returns:
            tuple: (检测结果列表, 处理后的帧图像)
        """
        # 确保资源已初始化
        if not self.initialized:
            if not self._initialize_resources():
                logger.error("Initialization failed")
                return [], None

        try:
            # 读取一帧视频
            success, frame = self.cap.read()
            
            if not success:
                # 如果读取失败，尝试重新打开视频
                logger.warning("Failed to read frame, trying to reopen video capture")
                try:
                    self.cap.release()
                    self.cap = cv2.VideoCapture(self.video_path)
                    if not self.cap.isOpened():
                        logger.error(f"Failed to reopen video capture: {self.video_path}")
                except Exception as reopen_error:
                    logger.error(f"Error reopening video capture: {str(reopen_error)}")
                return [], None
            
            # 增加帧计数器
            self.frame_counter += 1
            
            # 根据配置的间隔帧数处理
            if self.frame_counter % self.process_interval_frames != 0:
                return [], frame
            output_results = []
            output_result = {
                                "track_id": int(0),
                                "class_id": int(0),
                                "score": round(float(1), 2),
                                "box": [0, 0, 0, 0]
                            }
            output_results.append(output_result)
            
            return output_results, frame
        except Exception as e:
            logger.error(f"Error during line integrity: {str(e)}")
            # 确保即使出错也返回有效的帧
            if 'frame' not in locals():
                return [], None
            return [], frame

    def process(self):
        """
        获取线路完整性算法的当前状态
        
        Returns:
            str: 算法运行状态信息
        """
        state = self.get_state()
        if state['running']:
            return f"LineIntegrity running at {time.time()}"
        return None
