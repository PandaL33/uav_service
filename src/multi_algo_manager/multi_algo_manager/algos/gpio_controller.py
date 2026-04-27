import time
import threading
from typing import Optional
import logging

logger = logging.getLogger(__name__)

# 注意：
# 1. gpiod使用pip安装，版本为2.3.0
# 2. gpiod库的使用方法与1.6.3版本有显著差异，需要更新代码
# 3. 使用sudo apt install gpiod安装gpiod库默认版本为1.6.3，需要使用2.3.0版本

# 尝试导入gpiod库，如果不可用则设置为None
try:
    # 新版gpiod导入方式
    import gpiod
    from gpiod.line import Direction, Value
except ImportError:
    gpiod = None
    Direction = None
    Value = None
    logging.warning("gpiod library not available, GPIO functionality will be disabled")

class GPIOController:
    """
    GPIO控制类，用于异步控制GPIO引脚
    特点：
    1. 异步操作，不阻塞主线程
    2. 支持防抖动，避免连续触发
    3. 线程安全的GPIO控制
    """
    
    def __init__(self, logger=None):
        """
        初始化GPIO控制器
        
        Args:
            logger: 日志记录器，如果为None则使用默认日志记录器
        """
        self.logger = logger or logging.getLogger('GPIOController')
        self._lock = threading.RLock()     # 使用可重入锁保证线程安全
        self._active_gpio_operations = {}  # 记录正在进行的GPIO操作
        self._last_trigger_time = {}       # 记录每个GPIO的最后触发时间
        self._min_interval = 10.0          # 默认最小触发间隔（秒）
        self._default_high_time = 2      # 默认高电平持续时间（秒）
        
        # 控制模式管理
        self._pin_control_modes = {}  # 记录每个引脚的控制模式: 'continuous'
        self._pin_owners = {}  # 记录每个引脚的所有者信息
        self._pin_states = {}  # 记录持续控制模式下引脚的当前状态
        
        # 检查gpiod库是否可用
        self.gpio_available = gpiod is not None and Direction is not None and Value is not None
        
        # GPIO配置
        self._gpio_chip = "/dev/gpiochip1"
        self._requests = {}  # 存储已请求的GPIO线配置
        
        if self.gpio_available:
            self.logger.info(f"GPIO controller initialized, chip path: {self._gpio_chip}")
        else:
            self.logger.warning("GPIO functionality disabled: gpiod library not available or incompatible")
    
    def _setup_gpio_line(self, pin_number: int) -> Optional[object]:
        """
        设置GPIO线
        
        Args:
            pin_number: GPIO引脚号
            
        Returns:
            请求对象或None
        """
        if not self.gpio_available:
            self.logger.warning("GPIO setup skipped: GPIO not available")
            return None
        
        with self._lock:
            if pin_number not in self._requests:
                try:
                    # 使用新版API的request_lines方法
                    request = gpiod.request_lines(
                        self._gpio_chip,
                        consumer="algo_alarm",
                        config={
                            pin_number: gpiod.LineSettings(
                                direction=Direction.OUTPUT,
                                output_value=Value.INACTIVE
                            )
                        }
                    )
                    self._requests[pin_number] = request
                    self.logger.info(f"GPIO line {pin_number} setup successfully with new API")
                except Exception as e:
                    self.logger.error(f"Failed to setup GPIO line {pin_number}: {str(e)}")
                    return None
            
            return self._requests[pin_number]
    
    def _gpio_operation_worker(self, pin_number: int, high_time: float):
        """
        GPIO操作的工作线程函数
        
        Args:
            pin_number: GPIO引脚号
            high_time: 高电平持续时间
        """
        # 如果GPIO不可用，只模拟操作并更新时间戳
        if not self.gpio_available:
            self.logger.info(f"Simulating GPIO operation for pin {pin_number} (GPIO not available)")
            time.sleep(high_time)
            with self._lock:
                self._last_trigger_time[pin_number] = time.time()
                if pin_number in self._active_gpio_operations:
                    del self._active_gpio_operations[pin_number]
            return
            
        request = self._setup_gpio_line(pin_number)
        if not request:
            with self._lock:
                if pin_number in self._active_gpio_operations:
                    del self._active_gpio_operations[pin_number]
            return
        
        try:
            # 设置为高电平（使用新版API的值枚举）
            request.set_value(pin_number, Value.ACTIVE)
            self.logger.info(f"GPIO pin {pin_number} set to ACTIVE")
            
            # 等待指定时间
            time.sleep(high_time)
            
            # 设置为低电平
            request.set_value(pin_number, Value.INACTIVE)
            self.logger.info(f"GPIO pin {pin_number} set to INACTIVE")
            
        except Exception as e:
            self.logger.error(f"Error in GPIO operation for pin {pin_number}: {str(e)}")
        finally:
            # 更新最后触发时间
            current_time = time.time()
            with self._lock:
                self._last_trigger_time[pin_number] = current_time
                if pin_number in self._active_gpio_operations:
                    del self._active_gpio_operations[pin_number]
    
    def trigger_gpio(self, pin_number: int, high_time: float = None, owner_info: str = "unknown") -> bool:
        """
        异步触发GPIO引脚（高电平后低电平）
        
        Args:
            pin_number: GPIO引脚号
            high_time: 高电平持续时间，如果为None则使用默认值
            owner_info: 操作所有者信息，用于日志记录
            
        Returns:
            bool: 是否成功触发（在GPIO不可用时也返回True，模拟操作）
        """
        high_time = high_time or self._default_high_time
        
        # 即使GPIO不可用，也允许模拟操作，不抛出错误
        if not self.gpio_available:
            self.logger.info(f"GPIO not available, simulating trigger for pin {pin_number} by {owner_info}")
        elif not hasattr(self, '_gpio_chip') or not self._gpio_chip:
            self.logger.warning("GPIO chip not initialized, simulating trigger")
        
        with self._lock:
            # 检查引脚是否被持续控制
            if pin_number in self._pin_control_modes:
                owner = self._pin_owners.get(pin_number, 'unknown')
                self.logger.warning(f"Cannot trigger GPIO pin {pin_number}: pin is under continuous control by {owner}")
                return False
            
            # 检查是否有正在进行的操作
            if pin_number in self._active_gpio_operations:
                self.logger.warning(f"GPIO pin {pin_number} already has an active operation, skipping")
                return False
            
            # 检查触发间隔（即使GPIO不可用也保持防抖动逻辑）
            current_time = time.time()
            last_time = self._last_trigger_time.get(pin_number, 0)
            if current_time - last_time < self._min_interval:
                self.logger.warning(f"GPIO pin {pin_number} triggered too recently, skipping")
                return False
            
            # 标记为正在操作
            self._active_gpio_operations[pin_number] = True
        
        # 在新线程中执行GPIO操作（在GPIO不可用时会模拟执行）
        thread = threading.Thread(
            target=self._gpio_operation_worker,
            args=(pin_number, high_time),
            daemon=True
        )
        thread.start()
        
        # 即使GPIO不可用也返回True，表示触发请求已处理
        return True
    
    def set_min_interval(self, interval: float):
        """
        设置最小触发间隔
        
        Args:
            interval: 最小触发间隔（秒）
        """
        if interval > 0:
            self._min_interval = interval
            self.logger.info(f"GPIO min trigger interval set to {interval}s")
    
    def set_default_high_time(self, time_seconds: float):
        """
        设置默认高电平持续时间
        
        Args:
            time_seconds: 高电平持续时间（秒）
        """
        if time_seconds > 0:
            self._default_high_time = time_seconds
            self.logger.info(f"GPIO default high time set to {time_seconds}s")
    
    def set_gpio_state(self, pin_number: int, state: bool, owner_info: str = "unknown") -> bool:
        """
        持续控制GPIO引脚状态
        
        Args:
            pin_number: GPIO引脚号
            state: True表示高电平，False表示低电平
            owner_info: 操作所有者信息，用于日志记录
            
        Returns:
            bool: 操作是否成功
        """
        # 检查GPIO是否可用
        if not self.gpio_available:
            self.logger.info(f"GPIO not available, simulating setting pin {pin_number} to {state} by {owner_info}")
            return True
        
        with self._lock:
            # 检查引脚是否有正在进行的操作
            if pin_number in self._active_gpio_operations:
                self.logger.warning(f"Cannot set GPIO pin {pin_number} state: pin has an active operation")
                return False
            
            # 设置为持续控制模式
            self._pin_control_modes[pin_number] = 'continuous'
            self._pin_owners[pin_number] = owner_info
            
            # 获取或创建GPIO请求
            request = self._setup_gpio_line(pin_number)
            if not request:
                return False
            
            try:
                # 设置GPIO状态
                gpio_value = Value.ACTIVE if state else Value.INACTIVE
                request.set_value(pin_number, gpio_value)
                self._pin_states[pin_number] = state
                self.logger.info(f"GPIO pin {pin_number} set to {'ACTIVE' if state else 'INACTIVE'} in continuous mode by {owner_info}")
                return True
            except Exception as e:
                self.logger.error(f"Error setting GPIO pin {pin_number} state: {str(e)}")
                return False
    
    def release_gpio(self, pin_number: int, owner_info: str = "unknown") -> bool:
        """
        释放GPIO引脚的持续控制
        
        Args:
            pin_number: GPIO引脚号
            owner_info: 操作所有者信息，用于日志记录
            
        Returns:
            bool: 操作是否成功
        """
        with self._lock:
            # 检查引脚是否被持续控制
            if pin_number not in self._pin_control_modes or self._pin_control_modes[pin_number] != 'continuous':
                self.logger.warning(f"GPIO pin {pin_number} is not under continuous control, nothing to release, _pin_control_modes: {self._pin_control_modes}")
                return False
            
            # 检查所有者是否匹配（简单验证）
            current_owner = self._pin_owners.get(pin_number, 'unknown')
            if owner_info != current_owner and owner_info != "unknown":
                self.logger.warning(f"Owner mismatch: GPIO pin {pin_number} is controlled by {current_owner}, not {owner_info}")
                return False
            
            # 设置引脚为低电平
            if pin_number in self._requests:
                try:
                    self._requests[pin_number].set_value(pin_number, Value.INACTIVE)
                    self.logger.info(f"GPIO pin {pin_number} released and set to INACTIVE by {owner_info}")
                except Exception as e:
                    self.logger.error(f"Error releasing GPIO pin {pin_number}: {str(e)}")
            
            # 清除控制模式和状态信息
            if pin_number in self._pin_control_modes:
                del self._pin_control_modes[pin_number]
            if pin_number in self._pin_owners:
                del self._pin_owners[pin_number]
            if pin_number in self._pin_states:
                del self._pin_states[pin_number]
            
            return True
    
    def get_gpio_control_mode(self, pin_number: int) -> Optional[str]:
        """
        获取GPIO引脚的当前控制模式
        
        Args:
            pin_number: GPIO引脚号
            
        Returns:
            str: 'continuous' 或 None
        """
        with self._lock:
            return self._pin_control_modes.get(pin_number)
    
    def get_gpio_owner(self, pin_number: int) -> Optional[str]:
        """
        获取GPIO引脚的当前所有者
        
        Args:
            pin_number: GPIO引脚号
            
        Returns:
            str: 所有者信息或None
        """
        with self._lock:
            return self._pin_owners.get(pin_number)
    
    def cleanup(self):
        """
        清理GPIO资源
        确保在关闭GPIO请求前将所有引脚设置为低电平，防止程序退出时GPIO保持在高电平状态
        """
        # 如果GPIO不可用，跳过清理
        if not self.gpio_available:
            self.logger.info("GPIO cleanup skipped: GPIO not available")
            return
            
        with self._lock:
            # 先将所有GPIO引脚设置为低电平，再关闭请求
            for pin_number, request in list(self._requests.items()):
                try:
                    # 显式设置为低电平
                    request.set_value(pin_number, Value.INACTIVE)
                    self.logger.info(f"GPIO pin {pin_number} set to INACTIVE during cleanup")
                except Exception as e:
                    self.logger.error(f"Failed to set GPIO pin {pin_number} to INACTIVE during cleanup: {str(e)}")
            
            # 然后关闭所有GPIO请求
            for pin_number, request in self._requests.items():
                try:
                    request.close()
                    self.logger.info(f"GPIO request for pin {pin_number} closed")
                except Exception as e:
                    self.logger.error(f"Failed to close GPIO request for pin {pin_number}: {str(e)}")
            
            # 清理状态信息
            self._requests.clear()
            self._pin_control_modes.clear()
            self._pin_owners.clear()
            self._pin_states.clear()
            self._active_gpio_operations.clear()
    
    def __del__(self):
        """
        析构函数，确保资源被正确释放
        """
        self.cleanup()

# 创建全局GPIO控制器实例
gpio_controller = None

def get_gpio_controller(logger=None) -> GPIOController:
    """
    获取GPIO控制器实例（单例模式）
    
    Args:
        logger: 日志记录器
        
    Returns:
        GPIOController实例
    """
    global gpio_controller
    if gpio_controller is None:
        gpio_controller = GPIOController(logger)
    return gpio_controller

# 告警GPIO引脚定义（可以根据实际硬件配置修改）
TRASH_DETECT_ALARM_PIN = 30  # 对应 GPIO1_B2\
DEVICE_CHECK_ALARM_PIN = 30  # 可以使用相同或不同的引脚