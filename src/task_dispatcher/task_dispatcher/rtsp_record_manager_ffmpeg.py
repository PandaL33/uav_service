import subprocess
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

logger = logging.getLogger('task_dispatcher.rtsp_record_manager')

class RtspRecordManager:
    def __init__(self, server_url):
        self.rtsp_url = CAMERA01_URL
        self.segment_duration = SEGMENT_DURATION
        self.fps_override = FORCE_FPS
        self.output_dir = os.path.abspath(RECORD_FILE_PATH)
        self.server_url = server_url
        
        self.reconnect_delay = 5
        self.max_reconnect_delay = 30
        
        self.is_recording = False
        self.current_ffmpeg_process = None
        
        self._last_file_path = None
        
        self._stop_event = threading.Event()
        self._record_thread = None
        self._upload_thread = None
        
        self._upload_queue = queue.Queue()
        
        self._current_task_id = None
        self._save_record_flag = False
        self._upload_callback = None
        
        self.auth_manager = AuthManager(server_url)
        
        if not os.path.exists(self.output_dir):
            try:
                os.makedirs(self.output_dir, mode=0o755)
                logger.info("创建保存目录: %s", self.output_dir)
            except Exception as e:
                logger.error("无法创建保存目录 %s: %s", self.output_dir, str(e))
                raise

    def _recording_loop(self):
        logger.info("FFmpeg 录制线程已启动")
        logger.info("目标 RTSP 地址: %s", self.rtsp_url)
        self._last_file_path = None
        
        # 极简命令：只保留最老版本 FFmpeg 也支持的参数
        # 移除了 -reconnect, -reconnect_streamed, -reconnect_delay_max
        # 移除了 -preset, -crf
        # 使用 -b:v 控制码率
        base_cmd = [
            'ffmpeg',
            '-y',
            '-rtsp_transport', 'tcp',  # 强制 TCP，这个参数通常老版本也有
            '-i', self.rtsp_url,
            '-c:v', 'libx264',
            '-b:v', '2000k',
            '-g', '60',
            '-c:a', 'aac',
            '-movflags', '+faststart',
            '-loglevel', 'error'
        ]

        current_delay = self.reconnect_delay

        while not self._stop_event.is_set():
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = "rec_{}.mp4".format(timestamp)
            output_path = os.path.join(self.output_dir, filename)
            
            # 添加时长限制和输出路径
            cmd = base_cmd + [
                '-t', str(self.segment_duration),
                output_path
            ]
            
            logger.info("启动 FFmpeg: {} (时长:{}s)".format(filename, self.segment_duration))
            self._last_file_path = output_path
            
            try:
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    stdin=subprocess.PIPE
                )
                self.current_ffmpeg_process = process
                
                stdout, stderr = process.communicate()
                self.current_ffmpeg_process = None
                
                err_text = stderr.decode('utf-8', errors='ignore').strip()
                
                if process.returncode == 0:
                    logger.info("片段录制成功: {}".format(filename))
                    if os.path.exists(output_path) and os.path.getsize(output_path) > 0:
                        self._upload_queue.put(output_path)
                    else:
                        logger.warning("片段文件为空: {}".format(output_path))
                    
                    # 成功录制后，重置重连延迟
                    current_delay = self.reconnect_delay
                    continue 
                else:
                    # 录制失败（流断开或错误）
                    logger.warning("FFmpeg 异常退出 (代码 {}): {}".format(process.returncode, filename))
                    
                    error_found = False
                    error_lines = err_text.split('\n')
                    for line in error_lines:
                        line = line.strip()
                        if not line:
                            continue
                        
                        # 常见错误匹配
                        if "Authentication failed" in line or "401" in line:
                            logger.error("认证失败: 用户名或密码错误。请检查 CAMERA01_URL")
                            logger.error("详情: {}".format(line))
                            error_found = True
                            current_delay = self.max_reconnect_delay
                            break
                        elif "Connection timed out" in line or "timed out" in line:
                            logger.error("连接超时: 无法连接到摄像头 IP")
                            logger.error("详情: {}".format(line))
                            error_found = True
                            break
                        elif "Invalid data found" in line:
                            logger.error("数据无效: 摄像头返回了无法解析的流")
                            logger.error("详情: {}".format(line))
                            error_found = True
                            break
                        elif "No route to host" in line or "Connection refused" in line:
                            logger.error("网络不可达: 主机拒绝连接")
                            logger.error("详情: {}".format(line))
                            error_found = True
                            break
                        elif "Option" in line and "not found" in line:
                            logger.error("命令参数错误: {}".format(line))
                            logger.error("提示: 当前 FFmpeg 版本过老，不支持某些高级参数。")
                            error_found = True
                            break
                    
                    if not error_found and err_text:
                        logger.error("FFmpeg 错误输出: {}".format(err_text[-500:]))
                    elif not error_found and not err_text:
                        logger.error("FFmpeg 未输出具体错误信息，可能是流突然中断。")

                    # 如果有部分文件生成，尝试入队
                    if os.path.exists(output_path) and os.path.getsize(output_path) > 1024:
                        logger.warning("检测到残留文件，尝试入队: {}".format(filename))
                        self._upload_queue.put(output_path)
                    elif os.path.exists(output_path):
                        try:
                            os.remove(output_path)
                        except:
                            pass

                    # Python 层重连逻辑：等待一段时间后，while 循环会再次启动新的 FFmpeg 进程
                    logger.warning("流中断，等待 {} 秒后重试连接...".format(current_delay))
                    time.sleep(current_delay)
                    current_delay = min(current_delay * 1.5, self.max_reconnect_delay)
                    
            except FileNotFoundError:
                logger.error("致命错误: 系统未安装 ffmpeg。请执行: sudo apt install ffmpeg")
                break
            except Exception as e:
                logger.error("录制循环异常: {}".format(str(e)))
                time.sleep(current_delay)
                current_delay = min(current_delay * 1.5, self.max_reconnect_delay)

        logger.info("录制线程已安全退出。")
        self.is_recording = False

    def start_recording(self, task_id=None, save_record=True, upload_callback=None):
        if self.is_recording:
            logger.warning("录制服务已在运行，忽略启动请求。")
            return
        logger.info("启动录像服务 (FFmpeg 模式) | TaskID: {}".format(str(task_id)))
        self._current_task_id = task_id
        self._save_record_flag = save_record
        self._upload_callback = upload_callback
        self._stop_event.clear()
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
        logger.info("上传监控线程已启动")
        while not self._stop_event.is_set() or not self._upload_queue.empty():
            try:
                file_path = self._upload_queue.get(timeout=1)
                logger.info("检测到完整文件，准备上传: {}".format(os.path.basename(file_path)))
                time.sleep(0.5)
                if not os.path.exists(file_path):
                    logger.error("文件不存在，跳过: {}".format(file_path))
                    self._upload_queue.task_done()
                    continue
                if os.path.getsize(file_path) == 0:
                    logger.warning("文件为空，跳过: {}".format(file_path))
                    self._upload_queue.task_done()
                    continue

                result_id = self.process_record_data(
                    task_id=self._current_task_id,
                    record_file_path=file_path,
                    save_record=self._save_record_flag
                )
                
                if result_id:
                    logger.info("上传成功: {} (ID: {})".format(os.path.basename(file_path), str(result_id)))
                    if self._upload_callback:
                        try:
                            self._upload_callback(self._current_task_id, file_path, result_id)
                        except Exception as e:
                            logger.error("回调执行失败: {}".format(str(e)))
                    try:
                        os.remove(file_path)
                        logger.info("本地文件已删除: {}".format(os.path.basename(file_path)))
                    except Exception as e:
                        logger.error("删除本地文件失败: {}".format(str(e)))
                else:
                    logger.error("上传失败: {}。文件将保留在磁盘上以便排查。".format(os.path.basename(file_path)))
                
                self._upload_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                logger.error("上传线程异常: {}".format(str(e)))
        logger.info("上传监控线程已退出。")

    def wait_for_queue_empty(self, q, timeout):
        start_time = time.time()
        while True:
            if q.empty():
                return True
            if time.time() - start_time > timeout:
                return False
            time.sleep(1)
        
    def stop_recording(self):
        if not self.is_recording:
            return
        logger.info("正在停止录像服务...")
        self._stop_event.set()
        
        if self.current_ffmpeg_process:
            try:
                logger.info("正在终止 FFmpeg 进程...")
                self.current_ffmpeg_process.stdin.write(b'q')
                self.current_ffmpeg_process.stdin.flush()
                try:
                    self.current_ffmpeg_process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    logger.warning("FFmpeg 未及时响应，强制终止。")
                    self.current_ffmpeg_process.terminate()
                    self.current_ffmpeg_process.wait(timeout=5)
            except Exception as e:
                logger.error("终止 FFmpeg 进程时出错: {}".format(str(e)))
                if self.current_ffmpeg_process.poll() is None:
                    self.current_ffmpeg_process.kill()
        
        if self._record_thread:
            self._record_thread.join(timeout=15)
            if self._record_thread.is_alive():
                logger.error("录制线程未能及时停止。")
            else:
                logger.info("录制线程已退出。")
        
        if not self._upload_queue.empty():
            logger.info("等待上传队列处理完毕...")
            if self.wait_for_queue_empty(self._upload_queue, timeout=300):
                logger.info("上传队列已清空。")
            else:
                logger.error("等待上传队列超时。")

        if self._upload_thread:
            self._upload_thread.join(timeout=10)
            if self._upload_thread.is_alive():
                logger.warning("上传线程未能及时退出。")
        
        self.is_recording = False
        logger.info("录像服务已完全停止。")

    def process_record_data(self, task_id, record_file_path, save_record):
        if not save_record:
            return ''
        try:
            if not os.path.exists(record_file_path):
                return ''
            logger.info("开始上传: {}".format(record_file_path))
            file_id = self.upload_record_with_retry(record_file_path)
            return file_id if file_id else ''
        except Exception as e:
            logger.error("处理数据异常: {}".format(str(e)))
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
                logger.error('上传 HTTP 错误: %d', response.status_code)
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
            logger.warning("上传失败，%d 秒后重试 (%d/%d)", retry_interval, retry_count+1, max_retries)
            time.sleep(retry_interval)
            retry_interval = min(retry_interval * 2, max_retry_interval)
            retry_count += 1
        return ''
    