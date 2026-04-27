import cv2
import numpy as np
import base64
import json
from typing import Dict, Any, List, Tuple

class AlgoUtils:
    """
    算法通用工具类，提供坐标处理、图像编码等通用功能
    """
    
    @staticmethod
    def ensure_coordinates_in_bounds(bbox: List[float], frame_shape: Tuple[int, int]) -> List[int]:
        """
        确保坐标值在图像范围内
        
        Args:
            bbox: 边界框坐标 [x1, y1, x2, y2]
            frame_shape: 图像形状 (h, w)
            
        Returns:
            调整后的边界框坐标 [x1, y1, x2, y2]
        """
        if frame_shape is None or len(frame_shape) < 2:
            return [0, 0, 0, 0]
        
        h, w = frame_shape[:2]
        
        # 确保坐标在有效范围内
        x1 = max(0, min(w, int(bbox[0])))
        y1 = max(0, min(h, int(bbox[1])))
        x2 = max(0, min(w, int(bbox[2])))
        y2 = max(0, min(h, int(bbox[3])))
        
        return [x1, y1, x2, y2]
    
    @staticmethod
    def encode_image_to_base64(frame: np.ndarray) -> str:
        """
        将图像编码为base64字符串
        
        Args:
            frame: 图像数据
            
        Returns:
            base64编码的图像字符串
        """
        if frame is None:
            return ""
        
        try:
            # 确保图像是RGB格式
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                # 已经是RGB格式
                pass
            elif len(frame.shape) == 2:
                # 灰度图转换为RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
            
            # 编码为JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            # 转换为base64
            return base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            raise ValueError(f"Failed to encode image: {str(e)}")
    
    @staticmethod
    def create_algorithm_message(image_base64: str, timestamp: float, algo_name: str, result_data: Any) -> str:
        """
        创建算法结果消息
        
        Args:
            image_base64: base64编码的图像
            timestamp: 时间戳
            algo_name: 算法名称
            result_data: 算法结果数据
            
        Returns:
            JSON格式的消息字符串
        """
        message_data = {
            "image_base64": image_base64,
            "timestamp": timestamp,
            "algo_name": algo_name,
            "result_data": result_data
        }
        return json.dumps(message_data)
    
    @staticmethod
    def parse_configuration(config_path: str) -> Dict[str, Any]:
        """
        从配置文件解析配置
        
        Args:
            config_path: 配置文件路径
            
        Returns:
            配置字典
        """
        import os
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        try:
            import yaml
            with open(config_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f)
        except ImportError:
            # 如果没有安装yaml，尝试使用json
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                raise ValueError(f"Failed to parse configuration file: {str(e)}")
    
    @staticmethod
    def get_adaptive_default_size():
        """
        获取自适应默认图像尺寸
        
        Returns:
            默认图像尺寸 (h, w)
        """
        # 可以根据系统配置或其他因素动态调整默认尺寸
        return (1080, 1920)  # 默认HD分辨率