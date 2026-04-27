#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import logging
import time
import random
import requests
import xml.etree.ElementTree as ET
from xml.dom import minidom
import hashlib
from typing import Dict, Any, Optional
import rclpy
from rclpy.node import Node
from config import PTZ_URL, PTZ_USERNAME, PTZ_PASSWORD
from geometry_msgs.msg import Vector3
from config import ROBOT_TYPE

# 配置日志
logger = logging.getLogger('task_dispatcher.ptz_control')

class ISAPIUtil:
   
    @staticmethod
    def sha256_hex(data):
        """计算SHA256哈希值"""
        return hashlib.sha256(data.encode('utf-8')).hexdigest()
    
    @staticmethod
    def parse_xml(xml_str):
        """解析XML数据为字典，处理命名空间问题"""
        # 移除XML中的命名空间声明
        clean_xml = xml_str
        # 移除默认命名空间声明
        clean_xml = clean_xml.replace('xmlns=', 'xmlns:temp=')
        # 移除带前缀的命名空间声明
        import re
        clean_xml = re.sub(r'xmlns:[a-zA-Z0-9]+="[^"]*"', '', clean_xml)
        
        # 替换标签中的命名空间前缀
        clean_xml = re.sub(r'</?[a-zA-Z0-9]+:[a-zA-Z0-9]+', lambda m: m.group(0).split(':')[-1] if ':' in m.group(0) else m.group(0), clean_xml)
        
        # 处理可能剩余的命名空间URI格式 {namespace}tag
        root = ET.fromstring(clean_xml)
        result = {}
        
        def element_to_dict(element):
            d = {}
            for child in element:
                # 获取不带命名空间的标签名
                tag_name = child.tag.split('}')[-1] if '}' in child.tag else child.tag
                
                if len(child) == 0:
                    d[tag_name] = child.text
                else:
                    d[tag_name] = element_to_dict(child)
            return d
            
        # 处理根元素属性
        if root.attrib:
            for key, value in root.attrib.items():
                result[key] = value
        
        # 处理子元素
        result.update(element_to_dict(root))
        return result
    
    @staticmethod
    def dict_to_xml(tag, d):
        """将字典转换为XML字符串"""
        elem = ET.Element(tag)
        def _dict_to_xml(parent, d):
            for key, val in d.items():
                if isinstance(val, dict):
                    subelem = ET.SubElement(parent, key)
                    _dict_to_xml(subelem, val)
                else:
                    subelem = ET.SubElement(parent, key)
                    subelem.text = str(val) if val is not None else ""
        _dict_to_xml(elem, d)
        rough_string = ET.tostring(elem, encoding='unicode')
        # 格式化XML
        reparsed = minidom.parseString(rough_string)
        return reparsed.toprettyxml(indent="  ").strip()
    
    @staticmethod
    def get_capabilities(username, url):
        """获取设备登录信息"""
        req_url = f"{url}/ISAPI/Security/sessionLogin/capabilities?username={username}&random={random.randint(1000, 9999)}"
        try:
            response = requests.get(req_url, timeout=2)
            response.raise_for_status()  # 检查HTTP错误
            result = ISAPIUtil.parse_xml(response.text)
            # 如果响应头中有Set-Cookie，也添加到结果中
            if 'Set-Cookie' in response.headers:
                result['Cookie'] = response.headers['Set-Cookie']
            return result
        except requests.RequestException as e:
            logger.error(f"获取设备能力失败: {e}")
            return {}
    
    @staticmethod
    def session_login(username, password, capabilities_json, url):
        """登录设备"""
        # 安全获取值，提供默认值以防None
        challenge = capabilities_json.get("challenge", "")
        salt = capabilities_json.get("salt", "")
        iterations_str = capabilities_json.get("iterations", "1")
        
        # 安全转换为整数，默认为1
        try:
            iterations = int(iterations_str) if iterations_str else 1
        except (ValueError, TypeError):
            iterations = 1
        
        # 计算密码哈希
        password_hash = ISAPIUtil.sha256_hex(username + salt + password)
        password_hash = ISAPIUtil.sha256_hex(password_hash + challenge)
        for i in range(2, iterations):
            password_hash = ISAPIUtil.sha256_hex(password_hash)
        
        # 安全获取其他值
        session_id = capabilities_json.get("sessionID", "")
        session_version = capabilities_json.get("sessionIDVersion", "1")
        
        # 构建XML参数
        param_str = (
            "<SessionLogin>"
            f"<userName>{username}</userName>"
            f"<password>{password_hash}</password>"
            f"<sessionID>{session_id}</sessionID>"
            "<isSessionIDValidLongTerm>false</isSessionIDValidLongTerm>"
            f"<sessionIDVersion>{session_version}</sessionIDVersion>"
            "<isNeedSessionTag>true</isNeedSessionTag>"
            "</SessionLogin>"
        )
        
        logger.debug(f"登录参数: {param_str}")
        
        # 发送POST请求
        req_url = f"{url}/ISAPI/Security/sessionLogin?timeStamp={int(time.time() * 1000)}"
        try:
            response = requests.post(
                req_url, 
                data=param_str, 
                headers={'Content-Type': 'application/xml'}, 
                timeout=5
            )
            response.raise_for_status()
            result = ISAPIUtil.parse_xml(response.text)
            # 获取并设置Cookie
            if 'Set-Cookie' in response.headers:
                result['Cookie'] = response.headers['Set-Cookie']
            return result
        except requests.RequestException as e:
            logger.error(f"登录请求失败: {e}")
            return {}
    
    @staticmethod
    def ptz_ctrl(param_str, session_tag, cookie=None, url=""):
        """控制PTZ"""
        req_url = f"{url}/ISAPI/PTZCtrl/channels/1/continuous"
        headers = {
            "sessiontag": session_tag,
            "Content-Type": "application/xml"
        }
        # 如果提供了cookie，则添加到请求头中
        if cookie:
            headers["Cookie"] = cookie
            
        try:
            response = requests.put(req_url, data=param_str, headers=headers)
            response.raise_for_status()
            logger.debug(f"PTZ控制响应: {response.text}")
            return True
        except requests.RequestException as e:
            logger.error(f"PTZ控制失败: {e}")
            return False
    
    @staticmethod
    def lens_initialization(enabled, session_tag, cookie=None, url=""):
        """执行镜头初始化操作
        
        Args:
            enabled: 是否启用初始化，True表示开始初始化，False表示结束初始化
            session_tag: 会话标签
            cookie: Cookie信息（可选）
            url: 设备URL
            
        Returns:
            bool: 操作是否成功
        """
        req_url = f"{url}/ISAPI/Image/channels/1/lensInitialization"
        headers = {
            "sessiontag": session_tag,
            "Content-Type": "application/xml"
        }
        # 如果提供了cookie，则添加到请求头中
        if cookie:
            headers["Cookie"] = cookie
        
        # 构建XML参数
        xml_param = f'''<?xml version="1.0" encoding="UTF-8"?><LensInitialization><enabled>{"true" if enabled else "false"}</enabled></LensInitialization>'''.strip()
        
        try:
            response = requests.put(req_url, data=xml_param, headers=headers)
            response.raise_for_status()
            logger.info(f"镜头初始化{'开始' if enabled else '结束'}操作成功")
            logger.debug(f"镜头初始化响应: {response.text}")
            return True
        except requests.RequestException as e:
            logger.error(f"镜头初始化{'开始' if enabled else '结束'}操作失败: {e}")
            return False
            
    @staticmethod
    def get_ptz_status(session_tag, cookie=None, url=""):
        """获取PTZ状态信息
        
        Args:
            session_tag: 会话标签
            cookie: Cookie信息（可选）
            url: 设备URL
            
        Returns:
            dict: 包含PTZ状态信息的字典，如果失败返回空字典
        """
        req_url = f"{url}/ISAPI/PTZCtrl/channels/1/status"
        headers = {
            "sessiontag": session_tag,
            "Content-Type": "application/xml"
        }
        if cookie:
            headers["Cookie"] = cookie
            
        try:
            response = requests.get(req_url, headers=headers, timeout=5)
            response.raise_for_status()
            logger.debug(f"获取PTZ状态响应: {response.text}")
            
            # 解析XML响应
            status_data = ISAPIUtil.parse_xml(response.text)
            return status_data
        except requests.RequestException as e:
            logger.error(f"获取PTZ状态失败: {e}")
            return {}
    
    @staticmethod
    def ptz_absolute_control(pan, tile, zoom, session_tag, cookie=None, url=""):
        """PTZ绝对位置控制
        
        Args:
            pan: 水平角度 (0-360)
            tile: 垂直角度 (0-90)
            zoom: 变焦 (10-40)
            session_tag: 会话标签
            cookie: Cookie信息（可选）
            url: 设备URL
            
        Returns:
            bool: 操作是否成功
        """
        # 参数换算
        # pan: 0-360 → 0-3500
        azimuth = int(pan / 360 * 3500)
        # tile: 0-90 → 0-900
        elevation = int(tile / 90 * 900)
        # zoom
        absolute_zoom = zoom
        
        # 限制范围
        azimuth = max(0, min(3500, azimuth))
        elevation = max(0, min(900, elevation))
        absolute_zoom = max(10, min(40, absolute_zoom))
        
        # 构建XML参数
        xml_param = f"""<?xml version="1.0" encoding="UTF-8"?>
<PTZData>
<AbsoluteHigh>
  <elevation>{elevation}</elevation>
  <azimuth>{azimuth}</azimuth>
  <absoluteZoom>{absolute_zoom}</absoluteZoom>
</AbsoluteHigh>
</PTZData>""".strip()
        
        logger.debug(f"PTZ绝对位置控制参数: pan={pan}→{azimuth}, tile={tile}→{elevation}, zoom={zoom}→{absolute_zoom}")
        
        # 发送PUT请求
        req_url = f"{url}/ISAPI/PTZCtrl/channels/1/absolute"
        headers = {
            "sessiontag": session_tag,
            "Content-Type": "application/xml"
        }
        if cookie:
            headers["Cookie"] = cookie
        
        try:
            response = requests.put(req_url, data=xml_param, headers=headers)
            response.raise_for_status()
            logger.info(f"PTZ绝对位置控制成功: pan={pan}→{azimuth}, tile={tile}→{elevation}, zoom={zoom}→{absolute_zoom}")
            #logger.debug(f"PTZ绝对位置控制响应: {response.text}")
            return True
        except requests.RequestException as e:
            logger.error(f"PTZ绝对位置控制失败: {e}")
            return False

class PtzControl:
    """
    PTZ控制类，负责处理云台控制相关功能
    支持精准控制、方向控制和预置位控制
    """
    
    def __init__(self, node: Node):
        """
        初始化PTZ控制类
        
        Args:
            node: ROS2节点实例
        """
        self.node = node
        self.session_tag = ""
        self.cookie = ""
        self.last_login_time = 0
        self.login_timeout = 300  # 登录超时时间，5分钟
        
        # 初始化URL（移到构造函数中）
        self.url = PTZ_URL
        
        # 初始化用户名和密码（实际应用中应从配置文件读取）
        self.username = PTZ_USERNAME
        self.password = PTZ_PASSWORD

        self.publisher = self.node.create_publisher(Vector3, '/cam/speed', 10)
        
        # 尝试初始登录
        self._login_if_needed()
        
        logger.info('PTZ控制类初始化完成')
    
    def validate_ptz_params(self, ctrl_type: str, ctrl_params: Dict[str, Any]) -> bool:
        """
        验证PTZ控制参数
        
        Args:
            ctrl_type: 控制类型 (precise/ptz/preset)
            ctrl_params: 控制参数
            
        Returns:
            bool: 参数是否有效
        """
        try:
            if not isinstance(ctrl_params, dict):
                logger.error('PTZ控制参数不是有效的JSON对象')
                return False
                
            if ctrl_type == 'precise':
                # 精准控制参数验证
                if 'pan' not in ctrl_params or 'tile' not in ctrl_params or 'zoom' not in ctrl_params:
                    logger.error('精准控制缺少必要参数: pan, tile, zoom')
                    return False
                
                pan = ctrl_params['pan']
                tile = ctrl_params['tile']
                zoom = ctrl_params['zoom']
                
                # 验证参数范围
                if not isinstance(pan, (int, float)) or pan < 0 or pan > 360:
                    logger.error(f'pan参数无效: {pan}, 应为0-360之间的数值')
                    return False
                
                if not isinstance(tile, (int, float)) or tile < -30 or tile > 90:
                    logger.error(f'tile参数无效: {tile}, 应为-30到90之间的数值')
                    return False
                
                if not isinstance(zoom, (int, float)) or zoom < 1.0:
                    logger.error(f'zoom参数无效: {zoom}, 应大于等于1.0')
                    return False
                
            elif ctrl_type == 'ptz':
                # PTZ方向控制参数验证
                if 'direction' not in ctrl_params:
                    logger.error('PTZ方向控制缺少必要参数: direction')
                    return False
                
                direction = ctrl_params['direction']
                valid_directions = ['left', 'right', 'up', 'down', 'leftUp', 'leftDown', 
                                   'rightUp', 'rightDown', 'zoomIn', 'zoomOut', 'stop']
                
                if direction not in valid_directions:
                    logger.error(f'无效的方向参数: {direction}')
                    return False
                
                # speed是可选参数，验证类型
                if 'speed' in ctrl_params and (not isinstance(ctrl_params['speed'], int) or 
                                             ctrl_params['speed'] < 0 or ctrl_params['speed'] > 15):
                    logger.error(f'speed参数无效: {ctrl_params["speed"]}, 应为0-15之间的整数')
                    return False
                    
            elif ctrl_type == 'preset':
                # 预置位控制参数验证
                if 'presetCtrl' not in ctrl_params or 'presetNo' not in ctrl_params:
                    logger.error('预置位控制缺少必要参数: presetCtrl, presetNo')
                    return False
                
                preset_ctrl = ctrl_params['presetCtrl']
                if preset_ctrl not in ['call']:
                    logger.warning(f'预置位控制类型: {preset_ctrl} 可能不受支持，仅推荐使用call类型')
                    # 注意：根据文档，不建议对外开放set类型
                
                preset_no = ctrl_params['presetNo']
                if not isinstance(preset_no, int) or preset_no < 1 or preset_no > 32:
                    logger.error(f'presetNo参数无效: {preset_no}, 应为1-32之间的整数')
                    return False

            elif ctrl_type == 'lensInit':
                pass
            
            else:
                logger.error(f'无效的控制类型: {ctrl_type}')
                return False
                
            return True
            
        except Exception as e:
            logger.error(f'验证PTZ参数时出错: {str(e)}')
            return False
    
    def execute_ptz_control(self, camera_id: str, ctrl_type: str, ctrl_params: Dict[str, Any]) -> bool:
        """
        执行PTZ控制
        
        Args:
            camera_id: 摄像机ID
            ctrl_type: 控制类型
            ctrl_params: 控制参数
            
        Returns:
            bool: 控制是否成功
        """
        try:
            if not ctrl_params:
                logger.warning(f'PTZ控制参数为空: cameraId={camera_id}, ctrlType={ctrl_type}, ctrlParams={ctrl_params}')
                return False
            
            logger.info(f'执行PTZ控制: cameraId={camera_id}, ctrlType={ctrl_type}, ctrlParams={ctrl_params}')
            
            # 调用PTZ服务
            return self._call_ptz_service(camera_id, ctrl_type, ctrl_params)
            
        except Exception as e:
            logger.error(f'执行PTZ控制失败: {str(e)}')
            return False
    
    def _login_if_needed(self) -> bool:
        """
        检查登录状态，如果需要则重新登录
        
        Returns:
            bool: 登录是否成功
        """

        if ROBOT_TYPE == "uav":
            logger.info("UAV免登录")
            return True 

        current_time = time.time()
        # 如果没有会话标签或者已经超时，则重新登录
        if not self.session_tag or (current_time - self.last_login_time) > self.login_timeout:
            logger.info("开始PTZ控制器登录...")
            
            # 获取设备能力，传入URL
            capabilities = ISAPIUtil.get_capabilities(self.username, self.url)
            if not capabilities:
                logger.error("获取设备能力失败，无法登录")
                return False
            
            # 登录设备，传入URL
            session_info = ISAPIUtil.session_login(self.username, self.password, capabilities, self.url)
            if not session_info:
                logger.error("设备登录失败")
                return False
            
            # 获取会话信息
            self.session_tag = session_info.get("sessionTag", "")
            self.cookie = session_info.get("Cookie", "")
            self.last_login_time = current_time
            
            if self.session_tag:
                logger.info("PTZ控制器登录成功")
                return True
            else:
                logger.error("未获取到有效的sessionTag")
                return False
        return True
    
    def get_ptz_info(self, message_data: Dict[str, Any]) -> Dict[str, Any]:
        """获取云台信息
        
        Args:
            message_data: 包含cameraId的请求数据
            
        Returns:
            dict: 包含precise和presetList的字典，格式符合接口文档
        """
        try:
            # 检查登录状态
            if not self._login_if_needed():
                logger.error("获取云台信息失败：未成功登录")
                return {}
            
            # 获取PTZ状态信息
            status_data = ISAPIUtil.get_ptz_status(self.session_tag, self.cookie, self.url)
            
            # 构建返回结果
            result = {
                "precise": {},
                "presetList": []
            }
            
            logger.info(f"原始PTZ状态数据: {status_data}")
            # 处理精准位信息
            if "AbsoluteHigh" in status_data:
                absolute_high = status_data["AbsoluteHigh"]
                
                # 参数反向换算
                # azimuth (0-3500) → pan (0-360)
                pan = float(absolute_high.get("azimuth", 0)) / 3500 * 360
                # elevation (0-900) → tile (0-90)
                tile = float(absolute_high.get("elevation", 0)) / 900 * 90
                # absoluteZoom (10-40) → zoom (0-10) 或保留原始值
                zoom = float(absolute_high.get("absoluteZoom", 10))
                
                result["precise"] = {
                    "pan": round(pan, 2),
                    "tile": round(tile, 2),
                    "zoom": round(zoom, 2)
                }
            
            # 这里可以添加获取预置位列表的代码
            # 暂时返回空列表
            
            logger.info(f"获取云台信息成功: message_data={message_data}, result={result}")
            return result
            
        except Exception as e:
            logger.error(f"获取云台信息异常: {str(e)}")
            return {}
    
    def _call_ptz_service(self, camera_id: str, ctrl_type: str, params: Dict[str, Any]) -> bool:
        """
        使用ISAPI协议调用PTZ控制
        
        Args:
            camera_id: 摄像机ID
            ctrl_type: 控制类型
            params: 控制参数
            
        Returns:
            bool: 调用是否成功
        """
        # 检查登录状态
        if not self._login_if_needed():
            logger.error("PTZ控制失败：未成功登录")
            return False
        
        try:
            # 根据控制类型构建XML参数
            if ctrl_type == 'precise':
                # 精准控制
                pan = float(params.get('pan', 0.0))
                tile = float(params.get('tile', 0.0))
                zoom = float(params.get('zoom', 10.0))
                
                # 使用工具类的ptz_absolute_control方法进行绝对位置控制
                success = ISAPIUtil.ptz_absolute_control(pan, tile, zoom, self.session_tag, self.cookie, self.url)
                if success:
                    return True
                return False
                
            elif ctrl_type == 'ptz':
                # PTZ方向控制
                direction = params.get('direction', 'stop')
                #speed = int(params.get('speed', 7))
                speed = 60
                
                # 将方向映射到ISAPI协议的pan/tilt/zoom值
                pan = 0
                tilt = 0
                zoom = 0
                
                if direction == 'left':
                    pan = -speed
                elif direction == 'right':
                    pan = speed
                elif direction == 'up':
                    tilt = speed
                elif direction == 'down':
                    tilt = -speed
                elif direction == 'leftUp':
                    pan = -speed
                    tilt = speed
                elif direction == 'leftDown':
                    pan = -speed
                    tilt = -speed
                elif direction == 'rightUp':
                    pan = speed
                    tilt = speed
                elif direction == 'rightDown':
                    pan = speed
                    tilt = -speed
                elif direction == 'zoomIn':
                    zoom = speed
                elif direction == 'zoomOut':
                    zoom = -speed
                # stop状态保持0值
                
                xml_param = f'<?xml version="1.0" encoding="UTF-8"?><PTZData><pan>{pan}</pan><tilt>{tilt}</tilt><zoom>{zoom}</zoom></PTZData>'
                
            elif ctrl_type == 'preset':
                # 预置位控制
                preset_ctrl = params.get('presetCtrl', 'call')
                preset_no = params.get('presetNo', 1)
                
                if preset_ctrl == 'call':
                    # 调用预置位
                    req_url = f"{self.url}/ISAPI/PTZCtrl/channels/1/presets/{preset_no}/goto"
                    headers = {
                        "sessiontag": self.session_tag,
                        "Content-Type": "application/xml"
                    }
                    if self.cookie:
                        headers["Cookie"] = self.cookie
                        
                    try:
                        response = requests.put(req_url, headers=headers, timeout=5)
                        response.raise_for_status()
                        logger.info(f'预置位调用成功: 摄像机={camera_id}, 预置位={preset_no}')
                        return True
                    except requests.RequestException as e:
                        logger.error(f'预置位调用失败: {e}')
                        return False
                
                elif preset_ctrl == 'set':
                    logger.warning(f'预置位设置类型: {preset_ctrl} 可能不受支持，仅推荐使用call类型')
                    
            elif ctrl_type == 'lensInit':          
                # 使用ISAPIUtil的lens_initialization方法
                success = ISAPIUtil.lens_initialization(
                    True, 
                    self.session_tag, 
                    self.cookie, 
                    self.url
                )

                if success:
                    logger.info(f'镜头初始化开始成功: 摄像机={camera_id}')
                else:
                    logger.error(f'镜头初始化开始失败: 摄像机={camera_id}')

                time.sleep(0.5)

                success = ISAPIUtil.lens_initialization(
                    False, 
                    self.session_tag, 
                    self.cookie, 
                    self.url
                )

                if success:
                    logger.info(f'镜头初始化结束成功: 摄像机={camera_id}')
                else:
                    logger.error(f'镜头初始化结束失败: 摄像机={camera_id}')

                return success
            
            if ROBOT_TYPE == "uav":
                vec = Vector3()
                vec.x = float(pan)
                vec.y = float(tilt)
                logger.info(f'PTZ控制: vec={vec}')
                self.publisher.publish(vec)
                return True

            # 执行PTZ控制，传入URL
            success = ISAPIUtil.ptz_ctrl(xml_param, self.session_tag, self.cookie, self.url)
            
            if success:
                logger.info(f'PTZ控制成功: camera_id={camera_id}, ctrl_type={ctrl_type}')
            else:
                logger.error(f'PTZ控制失败: camera_id={camera_id}, ctrl_type={ctrl_type}')
            
            return success
            
        except Exception as e:
            logger.error(f'PTZ控制异常: {str(e)}')
            return False