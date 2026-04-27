from .base import AlgoBase
from .gpio_controller import get_gpio_controller

# GPIO引脚定义
DEFAULT_GPIO_PIN = 30  # 默认控制引脚
import logging

logger = logging.getLogger(__name__)

class GpioControl(AlgoBase):
    """
    简化版GPIO控制算法，仅提供GPIO开关功能
    """
    def __init__(self, node):
        """
        初始化GPIO控制
        """
        super().__init__(node, 'GpioControl', '/algo_result/gpio_control')
        
        # 初始化GPIO控制器
        self.gpio_controller = get_gpio_controller(logger)
        
        # 状态标志
        self.is_gpio_on = False
        self.active_pin = DEFAULT_GPIO_PIN  # 使用默认引脚
        
        logger.info(f"GPIO Control initialized, default pin: {self.active_pin}")
    
    def start(self):
        """
        启动GPIO（设置为高电平）
        """
        with self._lock:
            if not self._running:
                # 直接设置运行状态
                self._running = True
                # 执行GPIO开启操作
                self._turn_gpio_on()
                logger.info("GPIO Control started")
                return True
            logger.info("GPIO Control is already running")
            return False
    
    def stop(self):
        """
        停止GPIO（设置为低电平）
        """
        with self._lock:
            if self._running:
                # 执行GPIO关闭操作
                self._turn_gpio_off()
                # 直接设置停止状态
                self._running = False
                logger.info("GPIO Control stopped")
                return True
            logger.info("GPIO Control is not running")
            return False
    
    def _turn_gpio_on(self):
        """
        开启GPIO
        """
        # 使用公共API设置GPIO为高电平（持续控制模式）
        result = self.gpio_controller.set_gpio_state(self.active_pin, True, owner_info='GpioControl')
        if result:
            self.is_gpio_on = True
            # 注意：不需要重复记录日志，因为set_gpio_state方法已经记录了
            return True
        else:
            logger.error(f"Failed to turn ON GPIO pin {self.active_pin}")
            return False
    
    def _turn_gpio_off(self):
        """
        关闭GPIO
        """
        # 使用公共API释放GPIO的持续控制（会自动设置为低电平）
        result = self.gpio_controller.release_gpio(self.active_pin, owner_info='GpioControl')
        if result:
            self.is_gpio_on = False
            # 注意：不需要重复记录日志，因为release_gpio方法已经记录了
            return True
        else:
            # 如果释放失败，尝试直接设置为低电平
            logger.warning(f"Failed to release GPIO pin {self.active_pin}, trying to set to low directly")
            result = self.gpio_controller.set_gpio_state(self.active_pin, False, owner_info='GpioControl')
            if result:
                self.is_gpio_on = False
                return True
            logger.error(f"Failed to turn OFF GPIO pin {self.active_pin}")
            return False