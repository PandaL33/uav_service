import cv2
import time
import os
import glob
import logging
from datetime import datetime

# 导入配置文件
import config

# 配置日志
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        # 可选：同时保存到文件 log.txt
        # logging.FileHandler("recorder.log", encoding='utf-8')
    ]
)
logger = logging.getLogger(__name__)

class LoopRecorder:
    def __init__(self):
        self.rtsp_url = config.RTSP_URL
        self.output_dir = config.SAVE_DIR
        self.segment_duration = config.SEGMENT_DURATION
        self.max_files = config.MAX_FILES
        self.fps_override = config.FORCE_FPS
        
        # 重连配置
        self.reconnect_delay = 5  # 基础重连间隔 (秒)
        self.max_reconnect_delay = 30 # 最大重连间隔 (秒)
        
        self.current_out = None
        self.file_list = []
        self.start_time = 0
        self.frame_count = 0
        self.current_fps = 30 
        self.width = 0
        self.height = 0
        
        # 确保输出目录存在
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            logger.info(f"创建保存目录: {self.output_dir}")
            
        # 扫描并清理旧文件
        self._scan_existing_files()

    def _scan_existing_files(self):
        """扫描目录下已有的mp4文件并按时间排序"""
        pattern = os.path.join(self.output_dir, "*.mp4")
        files = glob.glob(pattern)
        files.sort(key=os.path.getmtime)
        self.file_list = files
        
        if len(self.file_list) > self.max_files:
            logger.warning(f"检测到历史文件过多 ({len(self.file_list)} > {self.max_files})，正在清理...")
            
        while len(self.file_list) > self.max_files:
            oldest = self.file_list.pop(0)
            try:
                os.remove(oldest)
                logger.info(f"初始化清理: 删除旧文件 {os.path.basename(oldest)}")
            except Exception as e:
                logger.error(f"删除文件失败: {e}")

    def _rotate_file(self):
        """切换新文件并清理旧文件"""
        if self.current_out:
            self.current_out.release()
            self.current_out = None
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        new_filename = f"rec_{timestamp}.mp4"
        new_path = os.path.join(self.output_dir, new_filename)
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        # 注意：如果宽高分辨率为0，则无法创建，需在调用前检查
        if self.width <= 0 or self.height <= 0:
            logger.error("分辨率无效，无法创建新文件")
            return False

        self.current_out = cv2.VideoWriter(new_path, fourcc, self.current_fps, (self.width, self.height))
        
        if not self.current_out.isOpened():
            logger.error(f"错误: 无法创建新文件 {new_path}")
            return False
            
        logger.info(f"--- 开始新片段: {new_filename} ---")
        
        self.file_list.append(new_path)
        
        while len(self.file_list) > self.max_files:
            oldest_file = self.file_list.pop(0)
            try:
                time.sleep(0.1) 
                os.remove(oldest_file)
                logger.info(f"循环覆盖: 删除 {os.path.basename(oldest_file)} (剩余 {len(self.file_list)} 个)")
            except Exception as e:
                logger.error(f"删除旧文件出错: {e}")
                
        self.start_time = time.time()
        self.frame_count = 0
        return True

    def _connect_camera(self):
        """
        专门用于连接摄像子的函数，包含无限重试逻辑
        返回: (cap对象, 是否成功)
        """
        attempt = 0
        current_delay = self.reconnect_delay
        
        while True:
            attempt += 1
            logger.info(f"正在尝试连接摄像头 (第 {attempt} 次)... URL: {self.rtsp_url}")
            
            cap = cv2.VideoCapture(self.rtsp_url)
            
            # 给摄像头一点启动时间，轮询检查是否打开
            # 有些摄像头需要几秒才能建立握手
            for _ in range(20): # 最多等待 20 * 0.5 = 10秒
                if cap.isOpened():
                    logger.info(f"摄像头连接成功！ (尝试次数: {attempt})")
                    return cap, True
                time.sleep(0.5)
            
            # 如果上面循环结束还没打开，说明本次尝试失败
            cap.release()
            logger.warning(f"连接超时，等待 {current_delay} 秒后重试...")
            time.sleep(current_delay)
            
            # 指数退避策略：每次失败增加等待时间，但不超过最大值
            current_delay = min(current_delay * 1.2, self.max_reconnect_delay)

    def start(self):
        """开始录制主循环"""
        logger.info("="*30)
        logger.info("循环录制服务已启动")
        logger.info(f"目标地址: {self.rtsp_url}")
        logger.info(f"策略: {self.segment_duration}s/段, 保留{self.max_files}个文件")
        logger.info("="*30)

        # 1. 初始连接
        cap, success = self._connect_camera()
        if not success:
            # 理论上 _connect_camera 是死循环直到成功，除非手动中断
            return

        # 获取流参数
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        
        if fps <= 0:
            if self.fps_override:
                self.current_fps = self.fps_override
                logger.warning(f"无法获取FPS，使用强制设定值: {self.current_fps}")
            else:
                self.current_fps = 25
                logger.warning(f"无法获取FPS，使用默认值: {self.current_fps}")
        else:
            self.current_fps = fps
            
        logger.info(f"流参数确认: {self.width}x{self.height}, FPS: {self.current_fps}")
        
        # 初始化第一个文件
        self._rotate_file()
        
        # 重置重连延迟，下次断开时从基础时间开始
        current_reconnect_delay = self.reconnect_delay 

        try:
            while True:
                ret, frame = cap.read()
                
                if not ret:
                    # === 检测到断流 ===
                    logger.warning("检测到视频流断开 (读取失败)，进入重连模式...")
                    cap.release() # 彻底释放旧连接
                    
                    # 调用无限重试连接函数
                    # 注意：这里会阻塞，直到重新连接成功
                    new_cap, success = self._connect_camera()
                    
                    if success:
                        cap = new_cap
                        # 重新确认分辨率 (防止摄像头重启后分辨率变化)
                        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        if w > 0 and h > 0:
                            self.width, self.height = w, h
                            logger.info(f"分辨率更新为: {self.width}x{self.height}")
                        
                        # 连接成功后，如果当前写入器还在，可以选择关闭它重新开始一段
                        # 或者继续写。为了文件完整性，通常建议切断当前文件，开始新的一段
                        if self.current_out:
                            logger.info("断线重连成功，强制结束当前片段，开始新片段。")
                            self._rotate_file()
                        
                        logger.info("恢复录制。")
                    continue
                
                # === 正常写入帧 ===
                if self.current_out:
                    self.current_out.write(frame)
                
                self.frame_count += 1
                
                # 检查是否需要切换文件
                current_duration = time.time() - self.start_time
                if current_duration >= self.segment_duration:
                    self._rotate_file()
                
                # 可选：定期打印心跳
                if self.frame_count % (self.current_fps * 60) == 0:
                    logger.debug(f"心跳正常 | 片段时长: {int(current_duration)}s | 文件数: {len(self.file_list)}")

        except KeyboardInterrupt:
            logger.info("\n用户手动停止录制。")
        except Exception as e:
            logger.error(f"发生未捕获的异常: {e}", exc_info=True)
        finally:
            logger.info("正在清理资源...")
            if self.current_out:
                self.current_out.release()
            if 'cap' in locals() and cap is not None:
                cap.release()
            cv2.destroyAllWindows()
            logger.info("程序已安全退出。")

if __name__ == "__main__":
    # 可选：设置环境变量以优化 RTSP 传输 (TCP 模式更稳定)
    import os
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
    
    recorder = LoopRecorder()
    recorder.start()