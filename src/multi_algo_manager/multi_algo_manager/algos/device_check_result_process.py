from ultralytics import YOLO
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import logging

# 获取模块logger，继承root logger的配置
logger = logging.getLogger(__name__)

def process_detection_results(results, display=True):
    """
    处理YOLO检测结果，绘制边界框和类别信息，并进行设备位置验证
    
    参数:
    results: YOLO模型的检测结果对象
    display: 是否显示结果图像
    
    返回:
    processed_images: 处理后的图像列表
    detections_info: 检测信息列表
    alerts: 告警信息列表
    """
    processed_images = []
    detections_info = []
    alerts = []
    
    # 设备判断参数配置
    config = {
        'device1_thresholds': {
            'min_area': 100 * 50,  # 设备1最小面积
            'max_area': 1300 * 720,  # 设备1最大面积
            'aspect_ratio_min': 0.1,  # 设备1最小宽高比
            'aspect_ratio_max': 10.0,  # 设备1最大宽高比
            'edge_threshold': 50,  # 边缘阈值
            'top_threshold': 50,  # 顶部阈值
            'bottom_threshold': 50,  # 底部阈值
            'confidence_threshold': 0.5,  # 置信度阈值
            'line_distance_coef': 1.0,  # 设备长度与框线角距离比较系数
        },
        'device2_thresholds': {
            'min_area': 200 * 200,  # 设备2最小面积
            'max_area': 1600 * 960,  # 设备2最大面积
            'aspect_ratio_min': 0.1,  # 设备2最小宽高比
            'aspect_ratio_max': 10.0,  # 设备2最大宽高比
            'edge_threshold': 50,  # 边缘阈值
            'top_threshold': 50,  # 顶部阈值
            'bottom_threshold': 50,  # 底部阈值
            'confidence_threshold': 0.5,  # 置信度阈值
            'line_distance_coef': 1.0,  # 设备长度与框线角距离比较系数
        },
        'line_corner_threshold': 100,  # 框线角与设备边缘最大距离阈值
        "device_length_threshold": 30,  # 设备长度与框线角距离最大阈值
    }
    
    # 遍历所有结果
    for i, result in enumerate(results):
        
        # 获取原始图像和尺寸
        img = result.orig_img.copy()
        img_height, img_width = img.shape[:2]
        img_center = (img_width // 2, img_height // 2)
        
        # 当前图像的检测信息
        image_detections = {
            'image_index': i+1,
            'total_boxes': len(result.boxes),
            'devices': [],
            'line_corners': [],
            'selected_device': None,
            'alert': None
        }
        
        # 1. 分类检测框：设备和框线角
        devices = []
        line_corners = []
        
        for j, box in enumerate(result.boxes):
            cls_idx = int(box.cls.item())
            cls_name = result.names[cls_idx]
            conf = box.conf.item()
            xyxy = box.xyxy.tolist()[0]
            x1, y1, x2, y2 = map(int, xyxy)
            
            # 计算中心点
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            box_info = {
                'box_index': j+1,
                'class_name': cls_name,
                'class_id': cls_idx,
                'confidence': conf,
                'bbox': [x1, y1, x2, y2],
                'center': (center_x, center_y),
                'area': (x2 - x1) * (y2 - y1),
                'width': x2 - x1,
                'height': y2 - y1,
                'aspect_ratio': (x2 - x1) / (y2 - y1) if (y2 - y1) > 0 else 0
            }
            
            # 分类为设备或框线角
            if cls_idx in [1, 2]:  # 设备类别
                devices.append(box_info)
                image_detections['devices'].append(box_info)
            elif cls_idx == 0:  # 框线角类别
                line_corners.append(box_info)
                image_detections['line_corners'].append(box_info)
        
        logger.info(f"  检测到 {len(devices)} 个设备，{len(line_corners)} 个框线角")
        
        # 2. 如果有多个设备，选择中心点最靠近图像中心的设备
        selected_device = None
        if devices:
            if len(devices) > 1:
                # 计算每个设备到图像中心的距离
                for device in devices:
                    distance = np.sqrt((device['center'][0] - img_center[0])**2 + 
                                      (device['center'][1] - img_center[1])** 2)
                    device['distance_to_center'] = distance
                
                # 选择距离中心最近的设备
                selected_device = min(devices, key=lambda d: d['distance_to_center'])
                #print(f"  选择了最靠近中心的设备: 类别ID={selected_device['class_id']}, "
                #      f"距离中心={selected_device['distance_to_center']:.2f}像素")
                if logger:
                    logger.info(f"  选择了最靠近中心的设备: 类别ID={selected_device['class_id']}, "
                                f"距离中心={selected_device['distance_to_center']:.2f}像素")
            else:
                selected_device = devices[0]
                logger.info(f"  选择了唯一设备: 类别ID={selected_device['class_id']}")
            
            image_detections['selected_device'] = selected_device
        
        # 3. 判断设备大小和位置是否符合要求
        valid_device = False
        if selected_device:
            device_id = selected_device['class_id']
            thresholds = config['device1_thresholds'] if device_id == 1 else config['device2_thresholds']
            
            x1, y1, x2, y2 = selected_device['bbox']
            
            # 3.1 检查面积和宽高比
            area_check = thresholds['min_area'] <= selected_device['area'] <= thresholds['max_area']
            aspect_check = thresholds['aspect_ratio_min'] <= selected_device['aspect_ratio'] <= thresholds['aspect_ratio_max']
            
            # 3.2 检查位置（是否太靠近边缘或顶部）
            left_edge_check = x1 > thresholds['edge_threshold']
            right_edge_check = img_width - x2 > thresholds['edge_threshold']
            top_edge_check = y1 > thresholds['top_threshold']
            bottom_edge_check = img_height - y2 > thresholds['bottom_threshold']

            # 3.3 检查置信度
            conf_check = selected_device['confidence'] >= thresholds['confidence_threshold']
            
            # 综合判断
            valid_device = (area_check and aspect_check and conf_check and
                        left_edge_check and right_edge_check and top_edge_check and bottom_edge_check)

            # 记录过滤原因
            if not valid_device:
                reasons = []
                if not area_check:
                    reasons.append(f"面积不在有效范围内 ({selected_device['area']})")
                if not aspect_check:
                    reasons.append(f"宽高比不在有效范围内 ({selected_device['aspect_ratio']:.2f})")
                if not conf_check:
                    reasons.append(f"置信度低于阈值 ({selected_device['confidence']:.2f})")
                if not left_edge_check:
                    reasons.append("设备太靠近左侧边缘")
                if not right_edge_check:
                    reasons.append("设备太靠近右侧边缘")
                if not top_edge_check:
                    reasons.append("设备太靠近顶部边缘")
                if not bottom_edge_check:
                    reasons.append("设备太靠近底部边缘")
                
                alert_info = {
                    'image_index': i+1,
                    'device_id': device_id,
                    'alert_type': 'device_invalid',
                    'reason': '; '.join(reasons),
                    'device_info': selected_device
                }
                alerts.append(alert_info)
                image_detections['alert'] = alert_info
                logger.warning(f"  设备不符合要求: {'; '.join(reasons)}")
            else:
                logger.info("  设备大小和位置检查通过")
        
        # 4. 如果设备有效，检查设备与框线角的位置关系
        if valid_device and selected_device:
            line_corners_count = len(line_corners)
            alert_info = None
            device_id = selected_device['class_id']
            thresholds = config['device1_thresholds'] if device_id == 1 else config['device2_thresholds']
            
            # 4.1 如果没有框线角，告警, 且置信度大于等于0.75，防止设备误检
            if line_corners_count == 0 and selected_device['confidence'] >= 0.4:
                alert_info = {
                    'image_index': i+1,
                    'device_id': selected_device['class_id'],
                    'alert_type': 'no_line_corners',
                    'reason': '未检测到框线角，设备可能未按规定放置',
                    'device_info': selected_device
                }
                logger.warning("  告警: 未检测到框线角，设备可能未按规定放置")
            
            # 4.2 如果只有一个框线角
            elif line_corners_count == 1:
                corner = line_corners[0]
                device_bbox = selected_device['bbox']
                x1, y1, x2, y2 = device_bbox
                corner_center = corner['center']
                
                # 检查框线角中心点是否在设备框内
                is_corner_inside_device = x1 <= corner_center[0] <= x2 and y1 <= corner_center[1] <= y2

                # 计算框线角到设备各边和角的距离
                distances = calculate_distances_to_device(corner['center'], device_bbox)
                # 获取框线角相对于设备的方位
                position = get_point_position_relative_to_device(corner['center'], device_bbox)
                threshold = 20
                # 调用函数根据方位应用不同的距离阈值判断规则
                is_too_far, alert_reason = check_corner_distance_by_position(distances, position, threshold)
                
                if is_corner_inside_device and is_too_far:
                    # 如果框线角在设备框内，生成告警
                    alert_info = {
                        'image_index': i+1,
                        'device_id': selected_device['class_id'],
                        'alert_type': 'corner_inside_device',
                        'reason': f'框线角中心点({corner_center[0]}, {corner_center[1]})位于设备框内，可能存在误检',
                        'device_info': selected_device,
                        'corner_info': corner
                    }
                    logger.warning(f"  告警: 框线角中心点位于设备框内，可能存在误检")
                else:
                    # 计算框线角到设备各边和角的距离
                    distances = calculate_distances_to_device(corner['center'], device_bbox)
                    # 获取框线角相对于设备的方位
                    position = get_point_position_relative_to_device(corner['center'], device_bbox)
                    threshold = config['line_corner_threshold']
                    
                    # 调用函数根据方位应用不同的距离阈值判断规则
                    is_too_far, alert_reason = check_corner_distance_by_position(distances, position, threshold)
                    
                    # 根据判断结果生成告警或打印信息
                    if is_too_far:
                        alert_info = {
                            'image_index': i+1,
                            'device_id': selected_device['class_id'],
                            'alert_type': 'single_corner_too_far',
                            'reason': f'单个框线角距离设备太远: {alert_reason}',
                            'device_info': selected_device,
                            'corner_info': corner,
                            'position': position
                        }
                        logger.warning(f"  告警: 单个框线角距离设备太远，{alert_reason}")
                    else:
                        logger.info(f"  单个框线角距离设备较近，{alert_reason}")
            
            # 4.3 如果有两个框线角
            elif line_corners_count == 2:
                # 计算两个框线角之间的距离
                corner1, corner2 = line_corners
                corner_distance = np.sqrt((corner1['center'][0] - corner2['center'][0])**2 + 
                                         (corner1['center'][1] - corner2['center'][1])** 2)
                
                # 设备长度（假设为宽度）
                device_length = selected_device['width']
                
                logger.info(f"  两个框线角距离: {corner_distance:.2f}像素, 设备长度: {device_length:.2f}像素")
                
                # 4.3.1 如果两个框线角距离小于设备长度*系数，认为是同一边的两个角
                if corner_distance < device_length * thresholds['line_distance_coef']:
                    # 检查设备底边是否紧挨着框线角，以及左侧或右侧是否紧挨着另一个框线角
                    device_bbox = selected_device['bbox']
                    
                    # 计算两个框线角到设备各边的距离
                    distances_to_device1 = calculate_distances_to_device(corner1['center'], device_bbox)
                    distances_to_device2 = calculate_distances_to_device(corner2['center'], device_bbox)
                    
                    # 检查是否有一个框线角靠近底边，并且这个角相对另一个角在更下方
                    # 获取两个框线角的y坐标（中心）
                    corner1_y = corner1['center'][1]
                    corner2_y = corner2['center'][1]
                    
                    # 确定哪个框线角在更下方
                    lower_corner_idx = 0 if corner1_y > corner2_y else 1
                    lower_corner_distances = distances_to_device1 if lower_corner_idx == 0 else distances_to_device2
                    #higher_corner_idx = 1 - lower_corner_idx
                    higher_corner_distances = distances_to_device2 if lower_corner_idx == 0 else distances_to_device1
                    
                    
                    # 检查下方的框线角是否靠近设备底边
                    bottom_corner_check = min(lower_corner_distances['bottom_edge'], 
                                             lower_corner_distances['bottom_left_corner'], 
                                             lower_corner_distances['bottom_right_corner']) <= config['line_corner_threshold']
                    
                    # 检查是否有一个框线角靠近左边或右边
                    side_corner_check = any(min(d['left_edge'], d['bottom_left_corner'], d['top_left_corner']) <= config['line_corner_threshold'] 
                                          or min(d['right_edge'], d['bottom_right_corner'], d['top_right_corner']) <= config['line_corner_threshold']
                                          for d in [higher_corner_distances])
                    
                    # 只有当同时满足底边和侧边都有框线角紧挨着才算正确放置
                    if not (bottom_corner_check and side_corner_check):
                        reasons = []
                        if not bottom_corner_check:
                            reasons.append(f"设备底边未紧挨着任何框线角")
                        if not side_corner_check:
                            reasons.append(f"设备左侧或右侧未紧挨着任何框线角")
                        
                        alert_info = {
                            'image_index': i+1,
                            'device_id': selected_device['class_id'],
                            'alert_type': 'two_corners_not_aligned',
                            'reason': '; '.join(reasons),
                            'device_info': selected_device,
                            'corner_distance': corner_distance
                        }
                        logger.warning(f"  告警: {'; '.join(reasons)}")
                    else:
                        logger.info(f"  设备放置正确: 设备底边和左侧/右侧同时紧挨着框线角")
                
                # TODO CHECK
                # 4.3.2 如果两个框线角距离大于设备长度，认为是底部两个角，将设备包裹在内
                else:
                    # 检查设备是否在两个框线角之间
                    device_center_x = selected_device['center'][0]
                    left_corner = min(line_corners, key=lambda c: c['center'][0])
                    right_corner = max(line_corners, key=lambda c: c['center'][0])
                    corners_distance = abs(right_corner['center'][0] - left_corner['center'][0])
                    device_length = selected_device['width']
                    corner_center_x = (left_corner['center'][0] + right_corner['center'][0]) / 2

                    if left_corner['center'][0] < device_center_x < right_corner['center'][0] and (corners_distance > device_length or abs(device_length - corners_distance) < config['device_length_threshold']) and abs(device_center_x - corner_center_x) < config['line_corner_threshold']:
                        logger.info("  设备放置正确: 两个框线角将设备包裹在内")
                    else:
                        alert_info = {
                            'image_index': i+1,
                            'device_id': selected_device['class_id'],
                            'alert_type': 'device_outside_corners',
                            'reason': '两个框线角距离较大，但设备不在框线角范围内',
                            'device_info': selected_device,
                            'left_corner_x': left_corner['center'][0],
                            'right_corner_x': right_corner['center'][0],
                            'device_center_x': device_center_x
                        }
                        logger.warning("  告警: 两个框线角距离较大，但设备不在框线角范围内")
            
            # 4.4 如果框线角数量大于3
            elif line_corners_count >= 3:
                # 找到最左和最右的框线角
                leftmost_corner = min(line_corners, key=lambda c: c['center'][0])
                rightmost_corner = max(line_corners, key=lambda c: c['center'][0])
                corners_distance = abs(rightmost_corner['center'][0] - leftmost_corner['center'][0])
                corner_center_x = (leftmost_corner['center'][0] + rightmost_corner['center'][0]) / 2
                
                device_length = selected_device['width']
                
                #print(f"  最左框线角X: {leftmost_corner['center'][0]}, 最右框线角X: {rightmost_corner['center'][0]}")
                #print(f"  框线角跨度: {corners_distance:.2f}像素, 设备长度: {device_length:.2f}像素")
                
                # 检查设备是否在框线角范围内
                device_center_x = selected_device['center'][0]
                
                if corners_distance > device_length and abs(device_center_x - corner_center_x) < config['line_corner_threshold']:
                    if leftmost_corner['center'][0] < device_center_x < rightmost_corner['center'][0]:
                        # 如果line_corners_count=3且最左边、最右边、最下边分别是不同三个框线角，则分别计算距离
                        if line_corners_count == 3:
                            # 找到最下边框线角
                            lowermost_corner = max(line_corners, key=lambda c: c['center'][1])
                            
                            # 检查三个角是否是不同的框线角
                            if (leftmost_corner != rightmost_corner and 
                                leftmost_corner != lowermost_corner and 
                                rightmost_corner != lowermost_corner):
                                # 计算各角到设备对应边缘的距离
                                device_bbox = selected_device['bbox']
                                left_distance = calculate_distances_to_device(leftmost_corner['center'], device_bbox)
                                right_distance = calculate_distances_to_device(rightmost_corner['center'], device_bbox)
                                bottom_distance = calculate_distances_to_device(lowermost_corner['center'], device_bbox)
                                
                                # 获取最小距离
                                min_left_dist = min(left_distance['left_edge'], left_distance['bottom_left_corner'], left_distance['top_left_corner'])
                                min_right_dist = min(right_distance['right_edge'], right_distance['bottom_right_corner'], right_distance['top_right_corner'])
                                min_bottom_dist = min(bottom_distance['bottom_edge'], bottom_distance['bottom_left_corner'], bottom_distance['bottom_right_corner'])
                                
                                # 检查距离是否都小于等于阈值
                                if (min_left_dist <= config['line_corner_threshold'] and 
                                    min_right_dist <= config['line_corner_threshold'] and 
                                    min_bottom_dist <= config['line_corner_threshold']):
                                    logger.info(f"  设备放置正确: 三个不同框线角分别靠近设备左、右、下边缘")
                                else:
                                    alert_info = {
                                        'image_index': i+1,
                                        'device_id': selected_device['class_id'],
                                        'alert_type': 'device_outside_corners',
                                        'reason': '三个框线角距离较大，但设备不在框线角范围内',
                                        'device_info': selected_device,
                                        'left_corner_x': leftmost_corner['center'][0],
                                        'right_corner_x': rightmost_corner['center'][0],
                                        'bottom_corner_y': lowermost_corner['center'][1],
                                        'device_center_x': device_center_x
                                    }
                                    logger.warning("  告警: 三个框线角距离较大，但设备不在框线角范围内")
                            else:
                                logger.info("  设备放置正确: 框线角跨度大于设备长度，且设备在范围内")
                        else:
                            logger.info("  设备放置正确: 框线角跨度大于设备长度，且设备在范围内")
                    else:
                        alert_info = {
                            'image_index': i+1,
                            'device_id': selected_device['class_id'],
                            'alert_type': 'device_outside_multiple_corners',
                            'reason': '框线角数量较多，但设备不在框线角范围内',
                            'device_info': selected_device,
                            'corners_distance': corners_distance,
                            'device_length': device_length
                        }
                        logger.warning("  告警: 框线角数量较多，但设备不在框线角范围内")
                else:
                    alert_info = {
                        'image_index': i+1,
                        'device_id': selected_device['class_id'],
                        'alert_type': 'corners_span_too_small',
                        'reason': f'框线角跨度小于设备长度 ({corners_distance:.2f} < {device_length:.2f})',
                        'device_info': selected_device,
                        'corners_distance': corners_distance,
                        'device_length': device_length
                    }
                    logger.warning("  告警: 框线角跨度小于设备长度")
            
            # 添加告警信息
            if alert_info:
                alerts.append(alert_info)
                image_detections['alert'] = alert_info
        
        # 绘制检测结果
        image = draw_detection_results(img, devices, line_corners, selected_device, alerts, display)
        
        # 保存处理后的图像和检测信息
        processed_images.append(image)
        detections_info.append(image_detections)
    
    return processed_images, detections_info, alerts

def calculate_distances_to_device(point, device_bbox):
    """
    计算点到设备各边和角的距离
    
    参数:
    point: 点坐标 (x, y)
    device_bbox: 设备边界框 [x1, y1, x2, y2]
    
    返回:
    包含各边和角距离的字典
    """
    x, y = point
    x1, y1, x2, y2 = device_bbox
    
    # 计算到各边的距离
    distance_to_left = abs(x - x1)
    distance_to_right = abs(x - x2)
    distance_to_top = abs(y - y1)
    distance_to_bottom = abs(y - y2)
    
    # 计算到各角的距离
    distance_to_tl = np.sqrt((x - x1)**2 + (y - y1)** 2)
    distance_to_tr = np.sqrt((x - x2)**2 + (y - y1)** 2)
    distance_to_bl = np.sqrt((x - x1)**2 + (y - y2)** 2)
    distance_to_br = np.sqrt((x - x2)**2 + (y - y2)** 2)
    
    return {
        'left_edge': distance_to_left,
        'right_edge': distance_to_right,
        'top_edge': distance_to_top,
        'bottom_edge': distance_to_bottom,
        'top_left_corner': distance_to_tl,
        'top_right_corner': distance_to_tr,
        'bottom_left_corner': distance_to_bl,
        'bottom_right_corner': distance_to_br
    }

def get_point_position_relative_to_device(point, device_bbox):
    """
    判断点相对于设备的方位
    
    参数:
    point: 点坐标 (x, y)
    device_bbox: 设备边界框 [x1, y1, x2, y2]
    
    返回:
    点相对于设备的方位，可能的值有：
    'left_top': 左上方
    'top': 上方
    'right_top': 右上方
    'right': 右方
    'right_bottom': 右下方
    'bottom': 下方
    'left_bottom': 左下方
    'left': 左方
    'inside': 内部
    """
    x, y = point
    x1, y1, x2, y2 = device_bbox
    
    # 检查点是否在设备内部
    if x1 <= x <= x2 and y1 <= y <= y2:
        return 'inside'
    
    # 确定点相对于设备的方位
    # 上方区域
    if y < y1:
        # 左上方
        if x < x1:
            return 'left_top'
        # 正上方
        elif x1 <= x <= x2:
            return 'top'
        # 右上方
        else:
            return 'right_top'
    # 同一水平区域
    elif y1 <= y <= y2:
        # 左方
        if x < x1:
            return 'left'
        # 右方
        else:
            return 'right'
    # 下方区域
    else:  # y > y2
        # 左下方
        if x < x1:
            return 'left_bottom'
        # 正下方
        elif x1 <= x <= x2:
            return 'bottom'
        # 右下方
        else:
            return 'right_bottom'


def check_corner_distance_by_position(distances, position, threshold):
    """
    根据点相对于设备的方位应用不同的距离阈值判断规则
    
    参数:
    distances: 包含点到设备各边距离的字典
    position: 点相对于设备的方位
    threshold: 距离阈值
    
    返回:
    is_too_far: 布尔值，表示距离是否超过阈值
    alert_reason: 字符串，包含详细的告警原因
    """
    is_too_far = True
    alert_reason = ""
    
    if position == 'inside':
        # 内部已经在前面处理过了
        is_too_far = False
    elif position == 'left_top':
        # 左上方：检查左边缘和上边缘距离
        left_distance = distances['left_edge']
        top_distance = distances['top_edge']
        is_too_far = left_distance > threshold or top_distance > threshold
        alert_reason = f"左上方位，左边缘距离{left_distance:.2f}像素，上边缘距离{top_distance:.2f}像素"
    elif position == 'top':
        # 上方：检查上边缘距离
        top_distance = distances['top_edge']
        is_too_far = top_distance > threshold
        alert_reason = f"上方位置，上边缘距离{top_distance:.2f}像素"
    elif position == 'right_top':
        # 右上方：检查右边缘和上边缘距离
        right_distance = distances['right_edge']
        top_distance = distances['top_edge']
        is_too_far = right_distance > threshold or top_distance > threshold
        alert_reason = f"右上方位，右边缘距离{right_distance:.2f}像素，上边缘距离{top_distance:.2f}像素"
    elif position == 'right':
        # 右方：检查右边缘距离
        right_distance = distances['right_edge']
        is_too_far = right_distance > threshold
        alert_reason = f"右方位置，右边缘距离{right_distance:.2f}像素"
    elif position == 'right_bottom':
        # 右下方：检查右边缘和下边缘距离
        right_distance = distances['right_edge']
        bottom_distance = distances['bottom_edge']
        is_too_far = right_distance > threshold or bottom_distance > threshold
        alert_reason = f"右下方位，右边缘距离{right_distance:.2f}像素，下边缘距离{bottom_distance:.2f}像素"
    elif position == 'bottom':
        # 下方：检查下边缘距离
        bottom_distance = distances['bottom_edge']
        is_too_far = bottom_distance > threshold
        alert_reason = f"下方位置，下边缘距离{bottom_distance:.2f}像素"
    elif position == 'left_bottom':
        # 左下方：检查左边缘和下边缘距离
        left_distance = distances['left_edge']
        bottom_distance = distances['bottom_edge']
        is_too_far = left_distance > threshold or bottom_distance > threshold
        alert_reason = f"左下方位，左边缘距离{left_distance:.2f}像素，下边缘距离{bottom_distance:.2f}像素"
    elif position == 'left':
        # 左方：检查左边缘距离
        left_distance = distances['left_edge']
        is_too_far = left_distance > threshold
        alert_reason = f"左方位置，左边缘距离{left_distance:.2f}像素"
    
    return is_too_far, alert_reason

def draw_detection_results(img, devices, line_corners, selected_device, alerts, display):
    """
    绘制检测结果，包括设备、框线角和告警信息
    
    参数:
    img: 原始图像
    devices: 设备列表
    line_corners: 框线角列表
    selected_device: 选中的设备
    alerts: 告警信息列表
    display: 是否显示图像
    """
    # 生成随机颜色
    np.random.seed(42)
    colors = np.random.randint(0, 255, size=(10, 3), dtype=np.uint8)
    
    # 获取中文字体（用于所有文本绘制）
    def get_font(size=16):
        font_paths = ['/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', 
                     '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                     '/usr/share/fonts/truetype/simhei/simhei.ttf']
        for font_path in font_paths:
            try:
                return ImageFont.truetype(font_path, size)
            except (IOError, OSError):
                continue
        return ImageFont.load_default()
    
    # 绘制文本的辅助函数（支持中文）
    def draw_text_with_background(image, text, position, font_size=16, text_color=(255,255,255), bg_color=(0,0,0)):
        try:
            # 使用PIL绘制中文
            img_pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(img_pil)
            font = get_font(font_size)
            
            # 获取文本边界框
            text_bbox = draw.textbbox(position, text, font=font)
            
            # 绘制背景
            draw.rectangle([text_bbox[0]-3, text_bbox[1]-3, text_bbox[2]+3, text_bbox[3]+3], fill=bg_color)
            
            # 绘制文本
            draw.text(position, text, font=font, fill=text_color)
            
            # 转换回OpenCV格式
            return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        except Exception:
            # 如果PIL方法失败，使用OpenCV默认方式
            try:
                font_scale = font_size / 20  # 近似缩放比例
                thickness = 1
                (w, h), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
                x, y = position
                
                # 绘制背景
                cv2.rectangle(image, (x, y - h - baseline), (x + w, y), bg_color, -1)
                
                # 绘制文本
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness)
                return image
            except:
                return image
    
    # 绘制未选中的设备（灰色）
    for device in devices:
        if selected_device and device['box_index'] != selected_device['box_index']:
            x1, y1, x2, y2 = device['bbox']
            cv2.rectangle(img, (x1, y1), (x2, y2), (128, 128, 128), 2)
            img = draw_text_with_background(img, "已过滤", (x1, max(10, y1 - 20)), 
                                           font_size=12, text_color=(128, 128, 128), bg_color=(220, 220, 220))
    
    # 绘制框线角（蓝色）
    for corner in line_corners:
        x1, y1, x2, y2 = corner['bbox']
        center_x, center_y = corner['center']
        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.circle(img, (center_x, center_y), 5, (255, 0, 0), -1)
        img = draw_text_with_background(img, "框线角", (x1, max(10, y1 - 20)), 
                                       font_size=12, text_color=(255, 255, 255), bg_color=(255, 0, 0))
    
    # 绘制选中的设备（绿色）
    if selected_device:
        x1, y1, x2, y2 = selected_device['bbox']
        device_color = (0, 255, 0)
        
        # 如果设备有告警，使用红色
        for alert in alerts:
            if ('device_info' in alert and 
                alert['device_info']['box_index'] == selected_device['box_index']):
                device_color = (0, 0, 255)
                break
        
        cv2.rectangle(img, (x1, y1), (x2, y2), device_color, 3)
        
        # 使用中文友好的标签
        device_name = f"设备{selected_device['class_id']}"
        label = f"{device_name}: 置信度{selected_device['confidence']:.2f}"
        img = draw_text_with_background(img, label, (x1, max(10, y1 - 25)), 
                                       font_size=14, text_color=(255, 255, 255), bg_color=device_color)
    
    # 显示告警信息
    alert_y = 30
    for alert in alerts:
        if 'reason' in alert:
            # 使用中文友好的字体显示告警信息
            alert_text = f"告警: {alert['reason'][:50]}..."
            
            try:
                # 使用PIL绘制中文文本（更可靠的中文显示方法）
                # 将OpenCV图像转换为PIL图像
                img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
                draw = ImageDraw.Draw(img_pil)
                
                # 尝试使用几种可能的中文字体
                font_paths = ['/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc', 
                             '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                             '/usr/share/fonts/truetype/simhei/simhei.ttf']
                font = None
                
                for font_path in font_paths:
                    try:
                        font = ImageFont.truetype(font_path, 16)
                        break
                    except (IOError, OSError):
                        continue
                
                if font is None:
                    # 如果找不到指定字体，使用默认字体
                    font = ImageFont.load_default()
                
                # 绘制背景
                text_bbox = draw.textbbox((20, alert_y-15), alert_text, font=font)
                draw.rectangle([text_bbox[0]-5, text_bbox[1]-5, text_bbox[2]+5, text_bbox[3]+5], fill=(0, 0, 200))
                
                # 绘制文本
                draw.text((20, alert_y-15), alert_text, font=font, fill=(255, 255, 255))
                
                # 将PIL图像转换回OpenCV图像
                img = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
            except Exception:
                # 如果PIL方法失败，使用OpenCV默认方式作为备选
                try:
                    # 绘制半透明背景以提高可读性
                    font_face = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.6
                    thickness = 2
                    (text_width, text_height), baseline = cv2.getTextSize(alert_text, font_face, font_scale, thickness)
                    cv2.rectangle(img, (20, alert_y - text_height - baseline), 
                                 (20 + text_width, alert_y + baseline), (0, 0, 200), -1)
                    cv2.putText(img, alert_text, (20, alert_y), 
                               font_face, font_scale, (255, 255, 255), thickness)
                except:
                    # 最后的备选方案
                    cv2.putText(img, alert_text, (20, alert_y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            alert_y += 40  # 增加行间距，更好地显示中文
            if alert_y > img.shape[0] - 40:
                break
    
    # 显示图像
    if display:
        img = cv2.resize(img, (1280, 720))
        cv2.imshow("result", img)
        cv2.waitKey(1)

    return img

def main():
    """
    主函数，加载模型、读取文件夹中的所有图像并进行检测
    支持按键交互，按空格键继续下一张，按ESC键退出
    """
    import os
    import glob
    
    # Load the YOLOv8 model
    model = YOLO('runs/detect/train7/weights/best.pt')
    
    # 指定图像文件夹路径
    image_folder = "/public/dataset/机器人数据/poc/不合规码垛1_out_error"
    
    # 获取文件夹中的所有图像文件
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
    image_files = []
    for ext in image_extensions:
        image_files.extend(glob.glob(os.path.join(image_folder, ext)))
    
    # 按文件名排序
    image_files.sort()
    
    if not image_files:
        print(f"错误：在文件夹 '{image_folder}' 中未找到图像文件")
        return
    
    print(f"找到 {len(image_files)} 张图像文件，开始处理...")
    
    total_images = 0
    total_boxes = 0
    total_alerts = 0

    # 等待按键
    # 默认不跳过
    process_all = False
    
    # 遍历每张图像
    for idx, image_path in enumerate(image_files):
        print(f"\n处理图像 {idx+1}/{len(image_files)}: {os.path.basename(image_path)}")
        
        # 读取图像
        image = cv2.imread(image_path)
        
        if image is None:
            print(f"错误：无法读取图像文件 '{image_path}'，跳过")
            continue
        
        print(f"成功读取图像，尺寸: {image.shape}")
        
        # 进行预测
        results = model(image, imgsz=640, conf=0.2, half=True, device="4", stream_buffer=False, agnostic_nms=False,
                                    save_txt=True, iou=0.5)
        
        # 处理检测结果
        processed_images, detections_info, alerts = process_detection_results(results, display=True)
        
        # 更新统计信息
        total_images += len(processed_images)
        for info in detections_info:
            total_boxes += info['total_boxes']
        total_alerts += len(alerts)
        
        # 打印当前图像的检测信息
        print(f"图像 {os.path.basename(image_path)} 处理完成")
        print(f"  检测到 {sum(info['total_boxes'] for info in detections_info)} 个物体")
        print(f"  产生了 {len(alerts)} 个告警")
        
        predict = ''
        if alerts:
            predict = '不合规'
            for i, alert in enumerate(alerts):
                print(f"    告警 {i+1}: {alert['reason']}")
        else:
            predict = '合规'
            print("  未产生任何告警，设备放置符合要求")

        groundtruth = '不合规'
        if groundtruth == predict:
            print("预测正确")
        else:
            output_dir = image_folder.replace('out', 'out_error')
            os.makedirs(output_dir, exist_ok=True)
            import shutil
            shutil.copy2(image_path, output_dir)
            print("预测错误")
        
        while not process_all:
            key = cv2.waitKey(1) & 0xFF
            if key == 32:  # 空格键
                print("继续下一张图像...")
                break
            elif key == 13:  # Enter键
                print("用户选择自动处理所有剩余图像...")
                process_all = True
                break
            elif key == 27:  # ESC键
                print("用户退出")
                cv2.destroyAllWindows()
                return
    
    # 打印总体统计信息
    print("\n===== 处理完成 =====")
    print(f"总共处理了 {total_images} 张图像")
    print(f"总共检测到 {total_boxes} 个物体")
    print(f"总共产生了 {total_alerts} 个告警")
    
    # 关闭所有窗口
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()