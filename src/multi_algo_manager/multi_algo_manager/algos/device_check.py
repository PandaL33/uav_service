import time
import cv2
from std_msgs.msg import String
from .base import AlgoBase
from .tracker_manager import TrackerManager
from .utils import AlgoUtils
from .gpio_controller import get_gpio_controller, DEVICE_CHECK_ALARM_PIN
from typing import Dict, Any
import logging
import os
from .device_check_result_process import process_detection_results

logger = logging.getLogger(__name__)

class DeviceCheck(AlgoBase):
    """
    设备检查算法，用于检查设备状态
    """
    def __init__(self, node, tracker_args: Dict[str, Any] = None):
        """
        初始化设备检查算法
        """
        super().__init__(node, 'DeviceCheck', '/algo_result/device_check')
        
        # 初始化资源标志
        self.initialized = False

        # 创建全局跟踪器管理器，并传递当前节点实例
        self.tracker_manager = TrackerManager(tracker_args)
        self.tracker = None
        
        # 跟踪目标检测次数和输出状态
        self.object_detection_count = {}  # 记录每个目标被检测到的次数
        self.outputted_objects = set()     # 记录已经输出过的目标ID
        
        # 初始化GPIO控制器
        self.gpio_controller = get_gpio_controller(logger)
        
        # 加载算法参数
        self._load_parameters()
        
        # 初始化视频捕获设备
        self.cap = None
        self.model = None
        self.frame_counter = 0
    
    def _load_parameters(self):
        """
        加载算法参数
        """
        if self._use_rtsp:
            self.video_path = self._rtsp_url
        else:
            self.video_path = '/public/lgl/device_check.avi'
        
        self.debug_mode = False
        self.process_interval_frames = 20
        self.detection_threshold = 0.5
        self.required_detection_count = 5
        self.default_image_height = 1080
        self.default_image_width = 1920
        
        # 记录参数加载信息
        logger.info(f"DeviceCheck parameters loaded: video_path={self.video_path}, "
                        f"debug_mode={self.debug_mode}, detection_threshold={self.detection_threshold}")

    def _initialize_resources(self):
        """
        初始化设备检查所需的资源和视频捕获设备
        
        Returns:
            bool: 初始化是否成功
        """
        if self.initialized:
            return True
            
        try:
            # 导入必要的库
            from ultralytics import YOLO
            import platform
            from ament_index_python.packages import get_package_share_directory
            
            # 获取package共享目录
            try:
                package_share_dir = get_package_share_directory('multi_algo_manager')
            except Exception as e:
                logger.error(f"Failed to get package share directory: {str(e)}")
                return False
            
            # 根据系统架构选择不同的模型
            system_architecture = platform.machine().lower()
            
            try:
                if 'x86' in system_architecture:
                    # x86架构，使用pytorch模型
                    model_path = os.path.join(package_share_dir, 'models', 'device_check', 'pytorch_model', 'best.pt')
                    logger.info(f"Using PyTorch model on x86 architecture: {model_path}")
                elif 'aarch64' in system_architecture:
                    # ARM架构，使用rknn模型
                    model_path = os.path.join(package_share_dir, 'models', 'device_check', 'best_rknn_model_device')
                    logger.info(f"Using RKNN model on ARM architecture: {model_path}")
                else:
                    # 未知架构，默认使用pytorch模型
                    model_path = os.path.join(package_share_dir, 'models', 'device_check', 'pytorch_model', 'best.pt')
                    logger.warning(f"Unknown architecture {system_architecture}, defaulting to PyTorch model: {model_path}")
                
                # 检查模型文件是否存在
                if not os.path.exists(model_path):
                    logger.error(f"Model file not found: {model_path}")
                    return False
                
                self.model = YOLO(model_path)
                logger.info(f"Model loaded successfully: {model_path}")
            except Exception as e:
                logger.error(f"Failed to load model: {str(e)}")
                return False

            # 获取跟踪器
            try:
                self.tracker = self.tracker_manager.get_tracker("DeviceCheck")
                logger.info("Tracker initialized successfully")
            except Exception as e:
                logger.error(f"Failed to initialize tracker: {str(e)}")
                return False

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
            logger.info("Device check resources initialized successfully")
            return True
        except ImportError as e:
            logger.error(f"Missing dependency: {str(e)}")
            self.initialized = False
            return False
        except Exception as e:
            logger.error(f"Failed to initialize device check resources: {str(e)}")
            # 清理已初始化的资源
            self._cleanup_resources()
            self.initialized = False
            return False

    def run_analysis(self):
        """
        设备检查算法主循环
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
                    
            while not self._should_stop():
                try:
                    # 执行具体的设备检查算法
                    output_results, frame = self.perform_device_check()
                    
                    # 算法无输出结果，跳过当前循环
                    if len(output_results) == 0:
                        continue

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
                            algo_name="device_check",
                            result_data=output_results
                        )
                        
                        self._publish_result(msg)
                        
                        # 记录检查结果日志（避免记录过长的base64数据）
                        logger.info(f"Published device check result with {len(output_results)} objects detected")
                        logger.info(f"Detection details: {output_results}")
                        
                        # 触发GPIO告警
                        try:
                            logger.info(f"Triggering GPIO alarm for device check")
                            self.gpio_controller.trigger_gpio(DEVICE_CHECK_ALARM_PIN)
                        except Exception as gpio_error:
                            logger.error(f"Error triggering GPIO alarm: {str(gpio_error)}")
                        
                    except ValueError as e:
                        logger.error(f"Failed to process image or create message: {str(e)}")
                    except Exception as e:
                        logger.error(f"Failed to publish message: {str(e)}")
                    
                    # 更新心跳，表示算法正常运行
                    self._last_heartbeat = time.time()
                except Exception as e:
                    logger.error(f"Error in DeviceCheck analysis: {str(e)}")
                    # 短暂暂停后继续，避免快速失败
                    time.sleep(0.1)
        finally:
            # 清理资源
            self._cleanup_resources()
    
    def _cleanup_resources(self):
        """
        清理算法使用的所有资源
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
        
        # 释放模型资源
        if hasattr(self, 'model') and self.model is not None:
            try:
                # 对于设备检查模型，通常不需要显式释放
                # 但为了彻底清理，我们可以将其设置为None
                self.model = None
                logger.info("Device check resources released successfully")
            except Exception as e:
                logger.error(f"Failed to release device check resources: {str(e)}")
        
        if hasattr(self, 'tracker') and self.tracker is not None:
            try:
                self.tracker_manager.remove_tracker("DeviceCheck")
                logger.info("Tracker released successfully")
                self.tracker = None
            except Exception as e:
                logger.error(f"Failed to release tracker: {str(e)}")

        # 释放OpenCV窗口
        if hasattr(self, 'debug_mode') and self.debug_mode:
            try:
                cv2.destroyAllWindows()
                logger.info("OpenCV windows closed successfully")
            except Exception as e:
                logger.error(f"Failed to close OpenCV windows: {str(e)}")
        
        # 重置初始化标志，确保重新启动时能正确初始化
        self.initialized = False
        self.outputted_objects.clear()
        self.object_detection_count.clear()
                
    def perform_device_check(self):
        """
        具体的设备检查算法实现，与trash_detect.py保持一致
        
        Returns:
            tuple: (设备检查结果, 处理后的图像)
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
            
            # 开始记录整体处理时间
            process_start_time = self.start_timer()
            
            # 运行YOLOv8推理，使用配置的检测阈值，禁用默认日志输出
            try:
                # 开始记录推理时间
                inference_start_time = self.start_timer()
                results = self.model(frame, conf=self.detection_threshold, verbose=False, iou=0.5)
                # 结束记录推理时间
                self.end_timer(inference_start_time, 'inference')
            except Exception as inference_error:
                logger.error(f"Model inference failed: {str(inference_error)}")
                # 确保记录处理完成
                self.end_timer(process_start_time, 'process')
                return [], frame

            try:
                if results is not None and len(results) > 0:
                    # 开始记录处理时间
                    process_start_time = self.start_timer()
                    # 处理检测结果
                    processed_images, detections_info, alerts = process_detection_results(results, display=self.debug_mode)
                    logger.info(f"Processed {len(processed_images)} detections")
                    if len(processed_images) != 1:
                        logger.error(f"Detected {len(processed_images)} objects")
                        return [], frame
                    
                    if len(alerts) > 0:
                        alert_types = ['device_invalid']
                        for alert in alerts:
                            if alert['alert_type'] in alert_types:
                                return [], frame
                            else:
                                output_result = {
                                "track_id": int(0),
                                "class_id": int(0),
                                "score": round(float(1), 2),
                                "box": [0, 0, 0, 0]
                                    }
                                output_results = []
                                output_results.append(output_result)
                                logger.warning(f"Detected alert type: {alert['alert_type']}")
                                return output_result, processed_images[0]
                    else:
                        return [], frame
                    
                    # 结束记录处理时间
                    self.end_timer(process_start_time, 'process')
                else:
                    return [], frame
            except Exception as process_error:
                logger.error(f"Error processing detection results: {str(process_error)}")
                # 确保记录处理完成
                self.end_timer(process_start_time, 'process')
                return [], frame
            
        except Exception as e:
            logger.error(f"Error during device check: {str(e)}")
            # 确保即使出错也记录处理完成
            if 'process_start_time' in locals():
                self.end_timer(process_start_time, 'process')
            # 确保即使出错也返回有效的帧
            if 'frame' not in locals():
                return [], None
            return [], frame

    def process(self):
        """
        获取设备检查算法的当前状态
        
        Returns:
            str: 算法运行状态信息
        """
        state = self.get_state()
        if state['running']:
            return f"DeviceCheck running at {time.time()}"
        return None