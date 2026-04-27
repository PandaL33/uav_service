import time
from std_msgs.msg import String
import cv2
from .base import AlgoBase
from .tracker_manager import TrackerManager
from .utils import AlgoUtils
from .gpio_controller import get_gpio_controller, TRASH_DETECT_ALARM_PIN
from typing import Dict, Any
import os
import logging

logger = logging.getLogger(__name__)

class TrashDetect(AlgoBase):
    """
    垃圾检测算法，用于检测环境中的垃圾
    """
    def __init__(self, node, tracker_args: Dict[str, Any] = None):
        """
        初始化垃圾检测算法
        """
        super().__init__(node, 'TrashDetect', '/algo_result/trash_detect')

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
        
        # 初始化资源属性
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
            self.video_path = '/public/lgl/20250916_152719.mp4'

        self.debug_mode = False
        self.process_interval_frames = 20
        self.detection_threshold = 0.5
        self.required_detection_count = 1
        self.default_image_height = 1080
        self.default_image_width = 1920
        
        # 记录参数加载信息
        logger.info(f"TrashDetect parameters loaded: video_path={self.video_path}, "
                        f"debug_mode={self.debug_mode}, detection_threshold={self.detection_threshold}")
    
    def _initialize_resources(self):
        """
        初始化YOLOv8模型和视频捕获设备
        
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
                    model_path = os.path.join(package_share_dir, 'models', 'trash_detect', 'pytorch_model', 'best.pt')
                    logger.info(f"Using PyTorch model on x86 architecture: {model_path}")
                elif 'aarch64' in system_architecture:
                    # ARM架构，使用rknn模型
                    model_path = os.path.join(package_share_dir, 'models', 'trash_detect', 'best_rknn_model_trash')
                    logger.info(f"Using RKNN model on ARM architecture: {model_path}")
                else:
                    # 未知架构，默认使用pytorch模型
                    model_path = os.path.join(package_share_dir, 'models', 'trash_detect', 'pytorch_model', 'best.pt')
                    logger.warning(f"Unknown architecture {system_architecture}, defaulting to PyTorch model: {model_path}")
                
                # 检查模型文件是否存在
                if not os.path.exists(model_path):
                    logger.error(f"Model file not found: {model_path}")
                    return False
                
                self.model = YOLO(model_path)
                logger.info(f"Model loaded successfully: {model_path}")
            except Exception as path_error:
                logger.error(f"Error determining model path: {str(path_error)}")
                return False

            # 初始化跟踪器
            try:
                self.tracker = self.tracker_manager.get_tracker("TrashDetect")
                logger.info("Tracker initialized successfully")
            except Exception as tracker_error:
                logger.error(f"Failed to initialize tracker: {str(tracker_error)}")
                return False
            
            # 打开视频文件或RTSP流
            try:
                self.cap = cv2.VideoCapture(self.video_path)
                if not self.cap.isOpened():
                    logger.error(f"Failed to open video capture: {self.video_path}")
                    return False
                logger.info(f"Successfully opened video capture: {self.video_path}")
            except Exception as cap_error:
                logger.error(f"Error opening video capture {self.video_path}: {str(cap_error)}")
                return False
            
            # 初始化帧计数器
            self.frame_counter = 0
            
            self.initialized = True
            logger.info("YOLOv8 model and video capture initialized successfully")
            return True
        except ImportError as e:
            logger.error(f"Missing dependency: {str(e)}")
            self.initialized = False
            return False
        except Exception as e:
            logger.error(f"Failed to initialize TrashDetect resources: {str(e)}")
            # 清理已初始化的资源
            self._cleanup_resources()
            self.initialized = False
            return False

    def run_analysis(self):
        """
        垃圾检测算法主循环，定期发布检测结果
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
                    # 执行具体的垃圾检测算法
                    output_results, frame = self.perform_trash_detection()
                    
                    # 算法无输出结果时，更新心跳并休眠
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
                            algo_name="trash_detect",
                            result_data=output_results
                        )
                        
                        self._publish_result(msg)
                        
                        # 记录检测结果日志，避免记录大量二进制数据
                        logger.info(f"Published trash detection result with {len(output_results)} objects detected")
                        logger.info(f"Detection details: {output_results}")
                        
                        # 触发GPIO告警
                        try:
                            logger.info(f"Triggering GPIO alarm for trash detection")
                            self.gpio_controller.trigger_gpio(TRASH_DETECT_ALARM_PIN)
                        except Exception as gpio_error:
                            logger.error(f"Error triggering GPIO alarm: {str(gpio_error)}")
                    except ValueError as e:
                        logger.error(f"Failed to process image or create message: {str(e)}")
                    except Exception as e:
                        logger.error(f"Failed to publish message: {str(e)}")
                    
                    # 更新心跳，表示算法正常运行
                    self._last_heartbeat = time.time()
                except Exception as e:
                    logger.error(f"Error in TrashDetect analysis: {str(e)}")
                    # 短暂暂停后继续，避免快速失败
                    time.sleep(0.1)
        finally:
            # 清理资源
            self._cleanup_resources()
    
    def _cleanup_resources(self):
        """
        清理算法使用的所有资源
        """
        try:
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
                    # 对于YOLO模型，通常不需要显式释放，Python的垃圾回收会处理
                    # 但为了彻底清理，我们可以将其设置为None
                    self.model = None
                    logger.info("Model resources released successfully")
                except Exception as e:
                    logger.error(f"Failed to release model resources: {str(e)}")

            if hasattr(self, 'tracker') and self.tracker is not None:
                try:
                    self.tracker_manager.remove_tracker("TrashDetect")
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
            
            logger.info("Resources cleaned up successfully")
        except Exception as e:
            logger.error(f"Error during resource cleanup: {str(e)}")
                
    def perform_trash_detection(self):
        """
        具体的垃圾检测算法实现，使用YOLOv8进行目标检测
        
        Returns:
            tuple: (检测结果描述, 处理后的图像)
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
                results = self.model(frame, conf=self.detection_threshold, verbose=False)
                # 结束记录推理时间
                self.end_timer(inference_start_time, 'inference')
            except Exception as model_error:
                logger.error(f"Error during model inference: {str(model_error)}")
                # 确保记录处理完成
                self.end_timer(process_start_time, 'process')
                return [], frame

            # 使用跟踪器更新结果
            tracked_results = []
            try:
                if results is not None and len(results) > 0:
                    # 开始记录跟踪时间
                    tracking_start_time = self.start_timer()
                    # 先获取原始检测框并更新跟踪器
                    tracked_results = self.tracker.update(results[0].boxes.cpu())
                    # 结束记录跟踪时间
                    self.end_timer(tracking_start_time, 'tracking')
                else:
                    tracked_results = []
            except Exception as tracker_error:
                logger.error(f"Error updating tracker: {str(tracker_error)}")
                # 确保记录处理完成
                self.end_timer(process_start_time, 'process')
                return [], frame
            
            # 清理不再出现在当前帧中的目标计数
            # 对于未输出过且当前帧未检测到的目标，重置其计数
            # 这确保了如果目标消失后又出现，会重新开始计数
            current_track_ids = set()
            output_results = []
            if len(tracked_results) > 0:
                # 记录和处理目标跟踪信息
                for track in tracked_results:
                    try:
                        x1, y1, x2, y2, track_id, score, class_id, idx = track
                        
                        # 记录当前帧中检测到的目标ID
                        current_track_ids.add(track_id)
                        
                        # 更新检测次数 - 如果是新目标或连续检测中
                        if track_id in self.object_detection_count:
                            # 连续检测，增加计数
                            self.object_detection_count[track_id] += 1
                        else:
                            # 新目标，初始化为1
                            self.object_detection_count[track_id] = 1
                        
                        # 获取当前计数
                        detection_count = self.object_detection_count[track_id]
                        
                        # 判断是否需要输出
                        if track_id not in self.outputted_objects and detection_count >= self.required_detection_count:
                            # 使用工具类确保坐标值在图像范围内
                            # 获取图像尺寸，如果frame为None则使用默认尺寸
                            frame_shape = frame.shape[:2] if frame is not None else \
                                         (self.default_image_height, self.default_image_width)
                            
                            # 使用工具类进行坐标边界检查
                            bbox = [x1, y1, x2, y2]
                            clamped_box = AlgoUtils.ensure_coordinates_in_bounds(bbox, frame_shape)
                            
                            output_result = {
                                "track_id": int(track_id),
                                "class_id": int(class_id),
                                "score": round(float(score), 2),
                                "box": clamped_box
                            }
                            output_results.append(output_result)
                            # 标记为已输出
                            self.outputted_objects.add(track_id)
                            logger.debug(f"Output object detected: Track {int(track_id)}, "
                                            f"Class {class_id}, Score {score:.2f}")
                    except ValueError as track_error:
                        logger.error(f"Error processing track data: {str(track_error)}")
                        continue
                    except Exception as track_error:
                        logger.error(f"Unexpected error processing track: {str(track_error)}")
                        continue

            # 清理计数（与调试/部署模式无关）
            for track_id in list(self.object_detection_count.keys()):
                if track_id not in current_track_ids and track_id not in self.outputted_objects:
                    # 目标消失，重置计数
                    del self.object_detection_count[track_id]
            
            # 仅在调试模式下进行图像绘制和显示
            if self.debug_mode:
                try:
                    # 复制YOLO绘制结果创建可写副本
                    if results is not None and len(results) > 0:
                        annotated_frame = results[0].plot().copy()
                        
                        # 如果有跟踪结果，在图像上额外绘制跟踪信息
                        if len(tracked_results) > 0:
                            for track in tracked_results:
                                try:
                                    x1, y1, x2, y2, track_id, score, class_id, idx = track
                                    
                                    # 获取当前计数
                                    detection_count = self.object_detection_count.get(track_id, 0)
                                    
                                    # 颜色选择：已输出过的目标用蓝色，未输出但检测次数达到要求的用红色，其他用绿色
                                    if track_id in self.outputted_objects:
                                        color = (255, 0, 0)  # 蓝色
                                    elif detection_count >= self.required_detection_count:
                                        color = (0, 0, 255)  # 红色
                                    else:
                                        color = (0, 255, 0)  # 绿色
                                    
                                    # 使用OpenCV绘制跟踪框
                                    cv2.rectangle(annotated_frame, (int(x1), int(y1)), 
                                                 (int(x2), int(y2)), color, 2)
                                    
                                    # 在框上方显示跟踪ID和检测次数
                                    display_text = f'Track {int(track_id)} (Count: {detection_count})'
                                    if track_id in self.outputted_objects:
                                        display_text += ' [OUTPUTTED]'
                                    
                                    cv2.putText(
                                        annotated_frame,
                                        display_text,
                                        (int(x1), int(y1) - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5,
                                        color,
                                        2
                                    )
                                except Exception as draw_error:
                                    logger.error(f"Error drawing track: {str(draw_error)}")
                        
                        # 显示处理后的图像
                        cv2.imshow("TrashDetect", annotated_frame)  # 修改窗口名称为算法名称
                        cv2.waitKey(1)
                except Exception as debug_error:
                    logger.error(f"Error in debug visualization: {str(debug_error)}")
            
            # 结束记录整体处理时间
            self.end_timer(process_start_time, 'process')
            return output_results, frame
        except Exception as e:
            logger.error(f"Error during TrashDetect: {str(e)}")
            # 确保即使出错也记录处理完成
            if 'process_start_time' in locals():
                self.end_timer(process_start_time, 'process')
            # 确保即使出错也返回有效的帧
            if 'frame' not in locals():
                return [], None
            return [], frame

    def process(self):
        """
        获取垃圾检测算法的当前状态
        
        Returns:
            str: 算法运行状态信息
        """
        state = self.get_state()
        if state['running']:
            return f"TrashDetect running at {time.time()}"
        return None