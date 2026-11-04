import rclpy
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32
import json
import asyncio
import logging
from collections import defaultdict
import time

# 配置日志
logger = logging.getLogger('dock_control_client')

def _lazy_client(service_name):
    """
    按需创建服务客户端。

    rclpy 单线程 executor 每处理一条消息都要用节点的全部实体重建一次 WaitSet，
    空闲的服务客户端虽不产生消息，却会永久计入每轮重建开销。机舱服务只在起飞/降落
    时用，改成首次调用才创建（call_service 里的 wait_for_service 会覆盖发现耗时）。
    """
    attr = f'_client_{service_name}'

    def _getter(self):
        client = getattr(self, attr, None)
        if client is None:
            client = self.node.create_client(Trigger, f'/dock/{service_name}')
            setattr(self, attr, client)
        return client

    return property(_getter)


class DockControlClient:
    """
    无人机机库控制客户端
    """

    # 同一节点共用一个实例：多处实例化会重复订阅 /dock/* 并成倍增加节点实体数
    _instances = {}

    def __new__(cls, node):
        instance = cls._instances.get(id(node))
        # id 可能被回收后的新节点复用，因此还要确认是同一个 node
        if instance is None or getattr(instance, 'node', None) is not node:
            instance = super().__new__(cls)
            cls._instances[id(node)] = instance
        return instance

    # --- 服务客户端（首次使用时才创建） ---
    charge_open_client = _lazy_client('charge_open')
    charge_close_client = _lazy_client('charge_close')
    cover_open_client = _lazy_client('cover_open')
    cover_close_client = _lazy_client('cover_close')
    putter_open_client = _lazy_client('putter_open')
    putter_close_client = _lazy_client('putter_close')
    battery_activation_client = _lazy_client('battery_activation')
    drone_discharge_start_client = _lazy_client('drone_discharge_start')
    drone_discharge_stop_client = _lazy_client('drone_discharge_stop')
    drone_open_client = _lazy_client('drone_open')
    drone_close_client = _lazy_client('drone_close')

    def __init__(self, node):
        # 复用实例时跳过重复初始化（否则会重复创建订阅者并清空进度状态）
        if getattr(self, '_initialized', False):
            return
        self._initialized = True
        self.node = node
        
        
        # --- 订阅者 ---
        self.progress_sub = self.node.create_subscription(
            String, '/dock/operation_progress', self._progress_callback, 10
        )
        
        self.cover_progress = 0
        self.cover_status = 'idle'
        
        self.putter_progress = 0
        self.putter_status = 'idle'
        # 推杆闭合状态：由最近一次开/合指令的执行结果维护，None 表示状态未知
        self.putter_closed = None
        
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
        
        # 等待结果
        start_time = time.time()
        while not future.done():
            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                msg = f"服务 {service_name} 超时 ({timeout_sec}s)"
                logger.error(msg)
                return False, msg
            
            # 让出控制权；节点由主线程的 executor spin，此处不能再次 spin（并发 spin 同一节点
            # 会触发 rclpy "wait set index too big"）
            time.sleep(0.1)
        
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
          
    def _reset_progress(self, operation_type: str):
        """下发指令前清空上一次操作的残留状态，避免 wait_for_operation 因旧状态立即返回"""
        if operation_type == 'cover':
            self.cover_status, self.cover_progress = 'idle', 0
        elif operation_type == 'putter':
            self.putter_status, self.putter_progress = 'idle', 0
        elif operation_type == 'charge':
            self.charge_status, self.charge_progress = 'idle', 0

    def wait_for_operation(self, operation_type: str, timeout: float = 60.0):
        """
        等待操作完成
        operation_type: 'cover' 或 'putter'
        """
        logger.info(f"等待{operation_type}操作完成...")
        start_time = time.time()
        
        while rclpy.ok():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                msg = f"等待{operation_type}操作超时"
                logger.error(msg)
                return False, msg
            
            # 检查进度
            if operation_type == 'cover':
                if self.cover_status == 'ok' and self.cover_progress == 100:
                    logger.info(f"{operation_type}操作完成")
                    return True, f"{operation_type}操作完成"
            elif operation_type == 'putter':
                if self.putter_status == 'ok' and self.putter_progress == 100:
                    logger.info(f"{operation_type}操作完成")
                    return True, f"{operation_type}操作完成"
            elif operation_type == 'charge':
                if self.charge_status == 'ok' and self.charge_progress == 100:
                    logger.info(f"{operation_type}操作完成")
                    return True, f"{operation_type}操作完成"  
            # 短暂等待；进度由主线程 executor 的回调更新，此处不能再次 spin 同一节点
            time.sleep(0.2)
        
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
        self._reset_progress('cover')
        success, msg = self.call_service(self.cover_open_client, 'cover_open')
        if not success:
            return False, f"打开舱盖失败: {msg}"
        
        # 3. 等待舱盖完全打开（1 分钟兜底）
        #    已确认机库能正常上报“cover 操作完成”，因此 60s 仍未完成即判定机库异常：
        #    舱盖未开完不允许展开推杆，直接失败，由上层回滚机库并上报任务失败
        logger.info("步骤 3: 等待舱盖打开完成（最长 60s）")
        success, msg = self.wait_for_operation('cover', timeout=60.0)
        if not success:
            return False, f"舱盖未在 60s 内完全打开: {msg}"
            
        # 4. 展开推杆
        logger.info("步骤 4: 展开推杆")
        self._reset_progress('putter')
        success, msg = self.call_service(self.putter_open_client, 'putter_open')
        if not success:
            # 这里就是你要求的错误返回
            return False, f"展开推杆失败: {msg}"
        
        # 3. 等待推杆完全展开
        logger.info("步骤 5: 等待推杆展开完成")
        success, msg = self.wait_for_operation('putter')
        if not success:
            return False, f"推杆展开未到位: {msg}" 
        self.putter_closed = False
        
        alarm_msg = Int32() 
        alarm_msg.data = 3 #无人机起飞告警
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
        self.putter_closed = True
        
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
        logger.info("步骤 5: 打开充电")
        success, msg = self.call_service(self.charge_open_client, 'charge_open')
        if not success:
            return False, f"打开充电失败: {msg}"
        
        logger.info("步骤 6: 等待充电打开")
        success, msg = self.wait_for_operation('charge')
        if not success:
            return False, f"打开充电未到位: {msg}"
            
        logger.info(">>> 降落序列完成！")
        return True, "降落序列执行成功"

    def land_alarm(self): 
        alarm_msg = Int32() 
        alarm_msg.data = 4 #无人机返航告警
        self.dock_alarm_pub.publish(alarm_msg)
        
    def land_charge_sequence(self) -> tuple[bool, str]:
        """
        执行降落后充电：闭合推杆，检测到闭合后发送一次打开充电（不等待充电到位反馈）
        返回: (bool: 是否成功, str: 状态消息)
        """
        # 1. 闭合推杆
        logger.info("步骤 1: 闭合推杆")
        self._reset_progress('putter')
        success, msg = self.call_service(self.putter_close_client, 'putter_close')
        if not success:
            return False, f"闭合推杆失败: {msg}"
        
        # 2. 等待推杆闭合完成
        logger.info("步骤 2: 等待推杆闭合完成")
        success, msg = self.wait_for_operation('putter')
        if not success:
            return False, f"推杆闭合未到位: {msg}"
        self.putter_closed = True
        
        # 3. 打开充电（仅下发一次，不做充电到位闭环检测；未充上由每分钟充电巡检兜底）
        logger.info("步骤 3: 打开充电（不等待充电到位反馈）")
        self._reset_progress('charge')
        success, msg = self.call_service(self.charge_open_client, 'charge_open')
        if not success:
            return False, f"打开充电失败: {msg}"
            
        logger.info(">>> 降落后闭合推杆并发送充电完成！")
        return True, "推杆已闭合，充电指令已发送"

    def query_putter_closed(self) -> bool:
        """
        查询推杆是否已闭合
        说明：机库仅上报操作进度，不提供独立的状态查询服务，故以最近一次开/合指令的
        执行结果为准；状态未知（进程刚启动）时按未闭合处理。
        """
        return self.putter_closed is True

    def open_charge_once(self) -> tuple[bool, str]:
        """发送一次打开充电指令（不等待充电到位反馈）"""
        logger.info("发送一次打开充电")
        return self.call_service(self.charge_open_client, 'charge_open')
    
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
            self.putter_closed = True
        elif act_name == "putter_open":
            logger.info("推杆展开")
            success, msg = self.call_service(self.putter_open_client, 'putter_open')
            if not success:
                return False, f"推杆展开失败: {msg}"
            
            logger.info("等待推杆展开完成")
            success, msg = self.wait_for_operation('putter')
            if not success:
                return False, f"推杆展开未到位: {msg}" 
            self.putter_closed = False
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
        