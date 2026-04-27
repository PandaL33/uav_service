import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32
import json
import asyncio
import logging
from collections import defaultdict
import time

# 配置日志
logger = logging.getLogger('dock_control_client')

class DockControlClient:
    """
    无人机机库控制客户端
    """

    def __init__(self, node):
        self.node = node
        
        # --- 服务客户端 ---
        # 充电和放电
        self.charge_open_client = self.node.create_client(Trigger, '/dock/charge_open')
        self.charge_close_client = self.node.create_client(Trigger, '/dock/charge_close')
        # 机舱打开和关闭
        self.cover_open_client = self.node.create_client(Trigger, '/dock/cover_open')
        self.cover_close_client = self.node.create_client(Trigger, '/dock/cover_close')
        # 推合杆
        self.putter_open_client = self.node.create_client(Trigger, '/dock/putter_open')
        self.putter_close_client = self.node.create_client(Trigger, '/dock/putter_close')
        
        # 激活电池服务
        self.battery_activation_client = self.node.create_client(Trigger, '/dock/battery_activation')
        # 开始放电服务
        self.drone_discharge_start_client = self.node.create_client(Trigger, '/dock/drone_discharge_start')
        # 停止放电服务
        self.drone_discharge_stop_client = self.node.create_client(Trigger, '/dock/drone_discharge_stop')
        # 无人机开机
        self.drone_open_client = self.node.create_client(Trigger, '/dock/drone_open')
        # 无人机关机
        self.drone_close_client = self.node.create_client(Trigger, '/dock/drone_close')
        
        # --- 订阅者 ---
        self.progress_sub = self.node.create_subscription(
            String, '/dock/operation_progress', self._progress_callback, 10
        )
        
        self.cover_progress = 0
        self.cover_status = 'idle'
        
        self.putter_progress = 0
        self.putter_status = 'idle'
        
        self.charge_progress = 0
        self.charge_status = 'idle'
        
        # self.battery_activation_progress = 0
        # self.battery_activation_status = 'idle'
        
        # self.drone_discharge_progress = 0
        # self.drone_discharge_status = 'idle'
        
        # 存储多个设备的状态
        self.device_data = {
            'timestamp': 0,
            'temperature': 0,
            'humidity': 0,
            'rainfall': 0,
            'wind_speed': 0,
            'online': False
        }
        
        self.osd_sub = self.node.create_subscription(
            String,
            '/dock/osd_data',
            self.osd_callback,
            10
        )
        self.dock_alarm_pub = self.node.create_publisher(Int32,'/dock/play_alarm',10)
        
    def osd_callback(self, msg: String):
        """处理OSD数据"""
        payload = json.loads(msg.data)
        data = payload.get('data', {})
        
        # 更新设备状态
        self.device_data = {
            'timestamp': payload.get('timestamp'),
            'temperature': data.get('temperature'),
            'humidity': data.get('humidity'),
            'rainfall': data.get('rainfall'),
            'wind_speed': data.get('wind_speed'),
            'online': True
        }
        
    def _get_device_data(self):
        return self.device_data.copy()
    
    def _progress_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            op_type = data.get('type', '')
            if op_type == 'cover':
                self.cover_progress = data.get('progress', 0)
                self.cover_status = data.get('status', 'idle')
            elif op_type == 'putter':
                self.putter_progress = data.get('progress', 0)
                self.putter_status = data.get('status', 'idle')
            elif op_type == 'charge':
                self.charge_progress = data.get('progress', 0)
                self.charge_status = data.get('status', 'idle')
        except json.JSONDecodeError:
            logger.error(f"获取进度异常")
            pass

    def call_service(self, client, service_name: str, timeout_sec: float = 30.0):
        """
        同步调用服务
        返回: (success: bool, message: str)
        """
        logger.info(f"调用服务: {service_name}")
        
        # 等待服务可用
        if not client.wait_for_service(timeout_sec=5.0):
            msg = f"服务 {service_name} 不可用"
            logger.error(msg)
            return False, msg
        
        # 发送请求
        request = Trigger.Request()
        future = client.call_async(request)
        
        # 等待结果（使用专用Executor避免 "wait set index too big" 错误）
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        executor.remove_node(self.node)

        if not future.done():
            msg = f"服务 {service_name} 超时 ({timeout_sec}s)"
            logger.error(msg)
            return False, msg

        # 获取结果
        try:
            result = future.result()
            if result.success:
                logger.info(f"{service_name} 成功: {result.message}")
                return True, result.message or "操作成功"
            else:
                msg = f"{service_name} 失败: {result.message}"
                logger.error(msg)
                return False, msg
        except Exception as e:
            msg = f"{service_name} 异常: {str(e)}"
            logger.error(msg)
            return False, msg
        
    
    def wait_for_operation(self, operation_type: str, timeout: float = 60.0):
        """
        等待操作完成
        operation_type: 'cover' 或 'putter'
        """
        logger.info(f"等待{operation_type}操作完成...")
        start_time = time.time()
        
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                msg = f"等待{operation_type}操作超时"
                logger.error(msg)
                executor.remove_node(self.node)
                return False, msg

            # 检查进度
            if operation_type == 'cover':
                if self.cover_status == 'ok' and self.cover_progress == 100:
                    logger.info(f"{operation_type}操作完成")
                    executor.remove_node(self.node)
                    return True, f"{operation_type}操作完成"
            elif operation_type == 'putter':
                if self.putter_status == 'ok' and self.putter_progress == 100:
                    logger.info(f"{operation_type}操作完成")
                    executor.remove_node(self.node)
                    return True, f"{operation_type}操作完成"
            elif operation_type == 'charge':
                if self.charge_status == 'ok' and self.charge_progress == 100:
                    logger.info(f"{operation_type}操作完成")
                    executor.remove_node(self.node)
                    return True, f"{operation_type}操作完成"
            # 短暂等待，使用专用executor避免 "wait set index too big"
            executor.spin_once(timeout_sec=0.2)
        executor.remove_node(self.node)
        
        return False, "节点关闭"

    def takeoff_sequence(self) -> tuple[bool, str]:
        """
        执行起飞序列
        返回: (bool: 是否成功, str: 状态消息)
        """
        logger.info(">>> 开始起飞序列")
        
        # 1. 关闭充电
        # logger.info("步骤 1: 关闭充电")
        # success, msg = self.call_service(self.charge_close_client, 'charge_close')
        # if not success:
        #     return False, f"关闭充电失败: {msg}"
        
        # logger.info("步骤 2: 等待关闭充电完成")
        # success, msg = self.wait_for_operation('charge')
        # if not success:
        #     return False, f"舱盖未到位: {msg}"
        
        # 2. 打开舱盖
        logger.info("步骤 2: 打开舱盖")
        success, msg = self.call_service(self.cover_open_client, 'cover_open')
        if not success:
            return False, f"打开舱盖失败: {msg}"
        
        # 3. 等待舱盖完全打开
        logger.info("步骤 3: 等待舱盖打开完成")
        success, msg = self.wait_for_operation('cover')
        if not success:
            return False, f"舱盖未到位: {msg}"
            
        # 4. 展开推杆
        logger.info("步骤 4: 展开推杆")
        success, msg = self.call_service(self.putter_open_client, 'putter_open')
        if not success:
            # 这里就是你要求的错误返回
            return False, f"展开推杆失败: {msg}"
        
        # 3. 等待推杆完全展开
        logger.info("步骤 5: 等待推杆展开完成")
        success, msg = self.wait_for_operation('putter')
        if not success:
            return False, f"推杆展开未到位: {msg}"
            
        alarm_msg = Int32() 
        alarm_msg.data = 3
        self.dock_alarm_pub.publish(alarm_msg)

        logger.info(">>> 起飞序列完成！")
        return True, "起飞序列执行成功"

    def land_sequence(self) -> tuple[bool, str]:
        """
        执行降落序列
        返回: (bool: 是否成功, str: 状态消息)
        """
        logger.info(">>> 开始降落序列")
        
        # 1. 闭合推杆
        logger.info("步骤 1: 闭合推杆")
        success, msg = self.call_service(self.putter_close_client, 'putter_close')
        if not success:
            return False, f"闭合推杆失败: {msg}"
        
        # 2. 等待推杆闭合完成
        logger.info("步骤 2: 等待推杆闭合完成")
        success, msg = self.wait_for_operation('putter')
        if not success:
            return False, f"推杆闭合未到位: {msg}"
        
        # 3. 关闭舱盖
        logger.info("步骤 3: 关闭舱盖")
        success, msg = self.call_service(self.cover_close_client, 'cover_close')
        if not success:
            return False, f"关闭舱盖失败: {msg}"
            
        # 4. 等待舱盖关闭
        logger.info("步骤 4: 等待舱盖关闭")
        success, msg = self.wait_for_operation('cover')
        if not success:
            return False, f"舱盖关闭未到位: {msg}"
        
        # 5. 打开充电
        # logger.info("步骤 5: 打开充电")
        # success, msg = self.call_service(self.charge_open_client, 'charge_open')
        # if not success:
        #     return False, f"打开充电失败: {msg}"
        
        # logger.info("步骤 6: 等待充电打开")
        # success, msg = self.wait_for_operation('charge')
        # if not success:
        #     return False, f"打开充电未到位: {msg}"
            
        logger.info(">>> 降落序列完成！")
        return True, "降落序列执行成功"

    def land_alarm(self): 
        alarm_msg = Int32() 
        alarm_msg.data = 4
        self.dock_alarm_pub.publish(alarm_msg)

    def call_uav_hangar_control_service(self, act_name):
        logger.info(f"收到{act_name}指令")
        
        if(act_name == "battery_activation"):
            success, msg = self.call_service(self.battery_activation_client, 'battery_activation')
            if not success:
                return False, f"激活电池失败失败: {msg}"
        elif act_name == "cover_open":
            logger.info("打开舱盖")
            success, msg = self.call_service(self.cover_open_client, 'cover_open')
            if not success:
                return False, f"打开舱盖失败: {msg}"
            
            logger.info("等待舱盖打开完成")
            success, msg = self.wait_for_operation('cover')
            if not success:
                return False, f"舱盖未到位: {msg}"
        elif act_name == "cover_close":
            logger.info("关闭舱盖")
            success, msg = self.call_service(self.cover_close_client, 'cover_close')
            if not success:
                return False, f"关闭舱盖失败: {msg}"
            
            logger.info("等待舱盖关闭完成")
            success, msg = self.wait_for_operation('cover')
            if not success:
                return False, f"舱盖未到位: {msg}"    
        elif act_name == "putter_close":
            logger.info("推杆闭合")
            success, msg = self.call_service(self.putter_close_client, 'putter_close')
            if not success:
                return False, f"推杆闭合失败: {msg}"
            
            logger.info("等待推杆闭合完成")
            success, msg = self.wait_for_operation('putter')
            if not success:
                return False, f"推杆闭合未到位: {msg}" 
        elif act_name == "putter_open":
            logger.info("推杆展开")
            success, msg = self.call_service(self.putter_open_client, 'putter_open')
            if not success:
                return False, f"推杆展开失败: {msg}"
            
            logger.info("等待推杆展开完成")
            success, msg = self.wait_for_operation('putter')
            if not success:
                return False, f"推杆展开未到位: {msg}" 
        elif act_name == "drone_open":
            success, msg = self.call_service(self.drone_open_client, 'drone_open')
            if not success:
                return False, f"无人机开机失败: {msg}"
        elif act_name == "drone_close":
            success, msg = self.call_service(self.drone_close_client, 'drone_close')
            if not success:
                return False, f"无人机关机失败: {msg}"
        elif act_name == "charge_open":
            success, msg = self.call_service(self.charge_open_client, 'charge_open')
            if not success:
                return False, f"打开充电失败: {msg}"
        elif act_name == "charge_close":
            success, msg = self.call_service(self.charge_close_client, 'charge_close')
            if not success:
                return False, f"关闭充电失败: {msg}"    
            
        return True, f"操作{act_name}成功"
           
