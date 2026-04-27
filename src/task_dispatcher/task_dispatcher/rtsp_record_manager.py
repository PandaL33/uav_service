import cv2
import time
import os
import json
import logging
import threading
import queue
from datetime import datetime
import requests
from requests.auth import HTTPBasicAuth
from auth_utils import AuthManager
from .config import CAMERA01_URL, SEGMENT_DURATION, FORCE_FPS, RECORD_FILE_PATH

# 配置日志
logger = logging.getLogger('task_dispatcher.rtsp_record_manager')

class RtspRecordManager:
    def __init__(self, server_url):
        self.rtsp_url = CAMERA01_URL
        self.segment_duration = SEGMENT_DURATION
        self.fps_override = FORCE_FPS
        self.output_dir = os.path.abspath(RECORD_FILE_PATH)
        self.server_url = server_url
        
        # 重连配置
        self.reconnect_delay = 5
        self.max_reconnect_delay = 30
        
        self.current_out = None
        self.start_time = 0
        self.frame_count = 0
        self.current_fps = 30 
        self.width = 0
        self.height = 0
        self.is_recording = False
        
        # 用于记录当前正在录制的文件路径
        self._last_file_path = None
        
        # 线程控制标志
        self._stop_event = threading.Event()
        self._record_thread = None
        self._upload_thread = None
        
        # 使用队列进行线程间通信
        self._upload_queue = queue.Queue()
        
        # 任务相关上下文
        self._current_task_id = None
        self._save_record_flag = False
        self._upload_callback = None
        
        self.auth_manager = AuthManager(server_url)
        
        # 确保输出目录存在
        if not os.path.exists(self.output_dir):
            try:
                os.makedirs(self.output_dir, mode=0o755)
                logger.info("创建保存目录: %s", self.output_dir)
            except Exception as e:
                logger.error("无法创建保存目录 %s: %s", self.output_dir, str(e))
                raise
    
    def capture_rtsp_snapshot(self, timeout=10):
        """
        从 RTSP 流中截取一张图片并保存。

        参数:
        rtsp_url (str): RTSP 流的地址 (例如: rtsp://admin:password@192.168.1.100:554/stream1)
        output_filename (str): 输出图片的文件名 (例如: snapshot.jpg)
        timeout (int): 连接超时时间（秒）
        """
        
        # 1. 创建 VideoCapture 对象
        # 注意：对于某些网络摄像头，可能需要在 URL 后面添加参数以保持连接稳定
        # 例如: rtsp://...?tcp=1 (强制使用 TCP 传输，避免 UDP 丢包导致的绿屏或花屏)
        cap = cv2.VideoCapture(self.rtsp_url )

        # 设置缓冲区大小（可选，有时能提高读取速度）
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 检查摄像头是否成功打开
        if not cap.isOpened():
            logger.error(f"错误: 无法打开 RTSP 流地址: {self.rtsp_url}")
            return False

        logger.info(f"正在连接到 RTSP 流: {self.rtsp_url} ...")

        # 2. 预读几帧
        # 网络流通常需要读取前几帧才能获取到清晰的图像，否则第一帧可能是黑的或模糊的
        frame_count = 0
        while frame_count < 20: 
            ret, frame = cap.read()
            if not ret:
                logger.warning("警告: 读取帧失败，正在重试...")
                # 如果读取失败，稍微等待一下继续尝试
                continue
            frame_count += 1

        # 3. 正式读取一帧用于截图
        ret, frame = cap.read()

        if ret:
            # 4. 保存图片
            # cv2.IMWRITE_JPEG_QUALITY 设置 JPEG 质量 (0-100)，默认是 95
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_filename = f"{RECORD_FILE_PATH}/snap_{timestamp}.jpg"
            cv2.imwrite(output_filename, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            logger.info(f"成功! 截图已保存为: {output_filename}")
            
            # 获取图片尺寸
            height, width, _ = frame.shape
            logger.info(f"图片分辨率: {width}x{height}")
            
            file_id = self.upload_jpeg_to_file_server(output_filename)
            
            return file_id
        else:
            logger.error("错误: 无法读取视频帧，请检查网络或摄像头状态。")
            return ''
    def upload_jpeg_to_file_server(self, file_path):
        if not self.server_url:
            logger.error("服务器地址未配置")
            return ''

        try:
            token = self.auth_manager.get_upload_token()
            if not token:
                return ''
            
            upload_url = '{}/file/file/upload/{}'.format(self.server_url, token)
            timestamp = int(time.time() * 1000)
            unique_filename = 'snap_{}.jpg'.format(timestamp)
            
            with open(file_path, 'rb') as f:
                files = {'file': (unique_filename, f, 'image/jpg')}
                response = requests.post(upload_url, files=files, timeout=300, verify=False)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('data') and 'id' in result['data']:
                    return result['data']['id']
            else:
                logger.error('上传HTTP错误: %d', response.status_code)
            return ''
                
        except Exception as e:
            logger.error('上传异常: %s', str(e))
            return ''
    def _rotate_file(self):
        """
        切换新文件。
        逻辑：关闭旧文件 -> 将旧文件加入上传队列 -> 创建新文件
        """
        old_file_path = self._last_file_path
        
        # 1. 关闭当前的写入器
        if self.current_out:
            self.current_out.release()
            self.current_out = None
            
        # 2. 如果存在旧文件，说明它刚刚被完整关闭，加入上传队列
        if old_file_path:
            logger.info("片段完成: %s，已加入上传队列。", os.path.basename(old_file_path))
            self._upload_queue.put(old_file_path)
            
        # 3. 准备新文件
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        new_filename = "rec_{}.mp4".format(timestamp)
        new_path = os.path.join(self.output_dir, new_filename)
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        if self.width <= 0 or self.height <= 0:
            logger.error("分辨率无效，无法创建新文件")
            return False

        self.current_out = cv2.VideoWriter(new_path, fourcc, self.current_fps, (self.width, self.height))
        
        if not self.current_out.isOpened():
            logger.error("错误: 无法创建新文件 %s (编码器可能不支持)", new_path)
            return False
            
        logger.info("--- 开始新片段: %s ---", new_filename)
        
        # 更新状态
        self._last_file_path = new_path
        self.start_time = time.time()
        self.frame_count = 0
        return True

    def _connect_camera(self):
        """连接摄像头 (带重试机制)"""
        attempt = 0
        current_delay = self.reconnect_delay
        max_attempts = 3
        
        while True:
            attempt += 1
            if attempt > max_attempts:
                logger.error("摄像头连接失败：已达到最大尝试次数 (%d)。", max_attempts)
                return None, False
            
            logger.info("正在尝试连接摄像头 (第 %d 次)...", attempt)
            cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
            
            connected = False
            for _ in range(20): 
                if cap.isOpened():
                    connected = True
                    break
                time.sleep(0.5)
            
            if connected:
                logger.info("摄像头连接成功！")
                return cap, True
            
            cap.release()
            
            if attempt >= max_attempts:
                return None, False
            
            logger.warning("连接超时，等待 %d 秒后重试...", current_delay)
            time.sleep(current_delay)
            current_delay = min(current_delay * 1.2, self.max_reconnect_delay)

    def _recording_loop(self):
        """录制主循环"""
        logger.info("录制线程已启动")
        self._last_file_path = None
        cap = None
        
        try:
            # 1. 初始连接
            cap, success = self._connect_camera()
            if not success:
                logger.error("初始连接失败，录制线程退出")
                return

            # 获取流参数
            self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            fps = int(cap.get(cv2.CAP_PROP_FPS))
            
            if fps <= 0:
                self.current_fps = self.fps_override if self.fps_override else 25
                logger.warning("无法获取FPS，使用设定值: %d", self.current_fps)
            else:
                self.current_fps = fps
                
            logger.info("流参数确认: %dx%d, FPS: %d", self.width, self.height, self.current_fps)
            
            if not self._rotate_file():
                logger.error("无法初始化第一个文件，录制线程退出")
                return
            
            while not self._stop_event.is_set():
                ret, frame = cap.read()
                
                if not ret:
                    logger.warning("视频流断开，进入重连模式...")
                    cap.release()
                    
                    # 重连循环
                    while not self._stop_event.is_set():
                        new_cap, success = self._connect_camera()
                        if success:
                            cap = new_cap
                            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            if w > 0 and h > 0:
                                self.width, self.height = w, h
                            
                            if self.current_out:
                                logger.info("重连成功，强制结束当前片段。")
                                self._rotate_file()
                            logger.info("恢复录制。")
                            break 
                        
                        logger.warning("重连失败，10秒后重试...")
                        time.sleep(10)
                    continue
                
                if self.current_out:
                    self.current_out.write(frame)
                
                self.frame_count += 1
                
                if time.time() - self.start_time >= self.segment_duration:
                    self._rotate_file()
                
                if self.frame_count % (self.current_fps * 60) == 0:
                    logger.debug("录制正常 | 队列等待数: %d", self._upload_queue.qsize())

        except Exception as e:
            logger.error("录制线程异常: %s", str(e), exc_info=True)
        finally:
            logger.info("录制线程正在清理资源...")
            
            # 【关键】确保最后一个文件被正确处理
            if self.current_out:
                self.current_out.release()
                self.current_out = None
            
            # 将最后一个文件加入队列
            if self._last_file_path:
                if os.path.exists(self._last_file_path) and os.path.getsize(self._last_file_path) > 0:
                    logger.info("录制结束，将最后片段 %s 加入上传队列。", os.path.basename(self._last_file_path))
                    self._upload_queue.put(self._last_file_path)
                else:
                    logger.warning("最后片段无效或为空，跳过入队: %s", self._last_file_path)
            
            if cap:
                cap.release()
            
            self.is_recording = False
            logger.info("录制线程已安全退出。")

    def start_recording(self, task_id=None, save_record=True, upload_callback=None):
        if self.is_recording:
            logger.warning("录制服务已在运行，忽略启动请求。")
            return

        logger.info("启动录像服务 | TaskID: %s", str(task_id))
        
        self._current_task_id = task_id
        self._save_record_flag = save_record
        self._upload_callback = upload_callback
        
        self._stop_event.clear()
        
        # 清空旧队列
        while not self._upload_queue.empty():
            try:
                self._upload_queue.get_nowait()
            except queue.Empty:
                break
        
        self._record_thread = threading.Thread(target=self._recording_loop, name="RecordThread", daemon=True)
        self._record_thread.start()
        
        self._upload_thread = threading.Thread(target=self._upload_monitor_loop, name="UploadMonitorThread", daemon=True)
        self._upload_thread.start()
        
        self.is_recording = True
        logger.info("录像服务启动完成。")

    def _upload_monitor_loop(self):
        """上传监控循环 (消费队列)"""
        logger.info("上传监控线程已启动")
        
        while not self._stop_event.is_set() or not self._upload_queue.empty():
            try:
                # 阻塞获取，超时1秒以便检查停止信号
                file_path = self._upload_queue.get(timeout=1)
                
                logger.info("检测到完整文件，准备上传: %s", os.path.basename(file_path))
                
                # 安全缓冲：等待1秒，确保磁盘IO完全结束
                time.sleep(1.0)
                
                if not os.path.exists(file_path):
                    logger.error("文件不存在，跳过: %s", file_path)
                    self._upload_queue.task_done()
                    continue
                
                if os.path.getsize(file_path) == 0:
                    logger.warning("文件为空，跳过: %s", file_path)
                    self._upload_queue.task_done()
                    continue

                # 执行上传
                result_id = self.process_record_data(
                    task_id=self._current_task_id,
                    record_file_path=file_path,
                    save_record=self._save_record_flag
                )
                
                if result_id:
                    logger.info("上传成功: %s (ID: %s)", os.path.basename(file_path), str(result_id))
                    
                    if self._upload_callback:
                        try:
                            self._upload_callback(self._current_task_id, file_path, result_id)
                        except Exception as e:
                            logger.error("回调执行失败: %s", str(e), exc_info=True)
                    
                    # 上传成功后删除本地文件
                    try:
                        os.remove(file_path)
                        logger.info("本地文件已删除: %s", os.path.basename(file_path))
                    except Exception as e:
                        logger.error("删除本地文件失败: %s", str(e))
                        
                else:
                    logger.error("上传失败: %s。文件将保留在磁盘上以便排查。", os.path.basename(file_path))
                
                # 【关键】标记任务完成，这样 stop_recording 中的 queue.join() 才能继续
                self._upload_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                logger.error("上传线程异常: %s", str(e), exc_info=True)
        
        logger.info("上传监控线程已退出。")

    def wait_for_queue_empty(self, q, timeout):
        """
        等待队列变空，支持超时。
        注意：这主要依赖 q.empty()，在生产者已停止生产的情况下是安全的。
        """
        start_time = time.time()
        while True:
            if q.empty():
                return True
            if time.time() - start_time > timeout:
                return False
            time.sleep(1)
        
    def stop_recording(self):
        """
        停止录制。
        保证：
        1. 等待录制线程结束（确保最后一个文件 release 并入队）。
        2. 等待上传队列完全清空（确保最后一个文件上传完成并删除）。
        """
        if not self.is_recording:
            return
            
        logger.info("正在停止录像服务...")
        self._stop_event.set()
        
        # 第一步：等待录制线程完全退出
        # 这保证了 current_out.release() 已调用，且最后一个文件已 put 到队列中
        if self._record_thread:
            self._record_thread.join(timeout=15)
            if self._record_thread.is_alive():
                logger.error("录制线程未能及时停止，可能导致最后文件未入队。")
            else:
                logger.info("录制线程已退出，所有文件已加入上传队列。")
        
        # 第二步：等待上传队列处理完毕
        # queue.join() 会阻塞，直到队列中所有元素都被 task_done()
        # 这保证了即使最后一个文件刚入队，也会在这里被等待上传完成
        if not self._upload_queue.empty():
            logger.info("等待上传队列处理完毕 (包含最后一段录像)...")
            if self.wait_for_queue_empty(self._upload_queue, timeout=300):
                logger.info("上传队列已清空，所有文件处理完毕。")
            else:
                logger.error("等待上传队列超时 (300秒)。部分文件可能未完成上传。")

        # 第三步：等待上传线程退出
        # 因为队列空了，且 stop_event 已设置，上传线程应该会很快退出
        if self._upload_thread:
            self._upload_thread.join(timeout=10)
            if self._upload_thread.is_alive():
                logger.warning("上传线程未能及时退出。")
        
        self.is_recording = False
        logger.info("录像服务已完全停止。")

    def process_record_data(self, task_id, record_file_path, save_record):
        if not save_record:
            logger.debug("配置为不保存，跳过: %s", record_file_path)
            return ''

        try:
            if not os.path.exists(record_file_path):
                return ''

            logger.info("开始上传: %s", record_file_path)
            file_id = self.upload_record_with_retry(record_file_path)
            
            if file_id:
                return file_id
            else:
                logger.error("上传最终失败。")
                return ''
                
        except Exception as e:
            logger.error("处理数据异常: %s", str(e), exc_info=True)
            return ''

    def _check_network_connection(self):
        if not self.server_url:
            return False
        try:
            requests.get(self.server_url, timeout=5, verify=False)
            return True
        except:
            return False
    
    def upload_point_cloud_to_file_server(self, record_file_path, **kwargs):
        if not self.server_url:
            logger.error("服务器地址未配置")
            return ''

        try:
            token = self.auth_manager.get_upload_token()
            if not token:
                return ''
            
            upload_url = '{}/file/file/upload/{}'.format(self.server_url, token)
            timestamp = int(time.time() * 1000)
            unique_filename = 'rec_{}.mp4'.format(timestamp)
            
            with open(record_file_path, 'rb') as f:
                files = {'file': (unique_filename, f, 'video/mp4')}
                response = requests.post(upload_url, files=files, timeout=300, verify=False)
            
            if response.status_code == 200:
                result = response.json()
                if result.get('data') and 'id' in result['data']:
                    return result['data']['id']
            else:
                logger.error('上传HTTP错误: %d', response.status_code)
            return ''
                
        except Exception as e:
            logger.error('上传异常: %s', str(e))
            return ''
    
    def upload_record_with_retry(self, record_file_path):
        retry_interval = 10
        max_retry_interval = 60
        retry_count = 0
        max_retries = 3
        
        while retry_count < max_retries:
            if not os.path.exists(record_file_path):
                logger.error("文件丢失")
                return ''
            
            if not self._check_network_connection():
                logger.warning("网络不可用，重试...")
                time.sleep(retry_interval)
                retry_interval = min(retry_interval * 2, max_retry_interval)
                retry_count += 1
                continue
            
            file_id = self.upload_point_cloud_to_file_server(record_file_path)
            if file_id:
                return file_id
            
            logger.warning("上传失败，%d秒后重试 (%d/%d)", retry_interval, retry_count+1, max_retries)
            time.sleep(retry_interval)
            retry_interval = min(retry_interval * 2, max_retry_interval)
            retry_count += 1
        
        return ''
    
