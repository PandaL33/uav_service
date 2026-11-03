#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import subprocess
import time
import json
from threading import Thread, Lock, Event
import signal
import os
import platform
from config import (CAMERAID, CAMERA01_URL, CAMERA02_URL, CODEC, ENABLE_VIDEO, PREVIEW_FPS,
                    PREVIEW_RETRY_MAX, PREVIEW_RETRY_DELAY, PREVIEW_RETRY_RESET_SEC)
import cv2

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(name)s] [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger('video_preview_manager')

class VideoPreviewManager:
    """
    视频预览管理器，负责处理视频预览相关的命令
    实现PlayVideo和StopVideo功能
    """
    
    def __init__(self, node, mqtt_client, response_topic):
        """
        初始化视频预览管理器
        
        Args:
            node: ROS2节点实例
            mqtt_client: MQTT客户端实例
            response_topic: MQTT响应主题
        """
        self.node = node
        self.mqtt_client = mqtt_client
        self.response_topic = response_topic
        
        # 存储当前运行的视频流进程
        self.running_streams = {}
        self.streams_lock = Lock()
        # 记录被主动停止的摄像头，避免把正常停止的退出码当成故障
        self._expected_stop = set()
        # 断流自动重连：推流参数、连续失败次数、每个摄像头独立的停止事件
        self._stream_params = {}
        self._restart_counts = {}
        self._stop_events = {}
        
        # 摄像头RTSP地址配置
        self.camera_rtsp_addresses = {CAMERAID: CAMERA01_URL}
        # if 'x86' in platform.machine():
        #     self.camera_rtsp_addresses = {
        #         'camera01': 'rtsp://192.168.200.131:8554/D1Ultra00001/camera03',  # 第一个摄像头的RTSP地址
        #         'camera02': 'rtsp://192.168.200.131:8554/D1Ultra00001/camera03'   # 第二个摄像头的RTSP地址
        #     }
        # else:
        #     self.camera_rtsp_addresses = {
        #         'camera01': CAMERA01_URL,  # 第一个摄像头的RTSP地址
        #         'camera02': CAMERA02_URL   # 第二个摄像头的RTSP地址
        #     }
        
        logger.info('视频预览管理器已初始化')
        logger.info(f'摄像头RTSP配置: {self.camera_rtsp_addresses}')
    
    def process_play_video(self, message_data):
        """
        处理播放视频命令
        
        Args:
            message_data: 包含播放视频命令的消息数据
        """
        try:
            # 仿真环境视频已禁用：直接返回，不启动 ffmpeg/RTSP
            if not ENABLE_VIDEO:
                self.send_ack_response(message_data, 200, '视频功能已禁用（仿真环境）', {})
                return

            # 提取请求参数
            body = message_data.get('body', {})
            camera_id = body.get('cameraId', '')
            trans_type = body.get('transType', 'RTMP').upper()
            push_url = body.get('pushUrl', '')
            username = body.get('username', '')
            password = body.get('password', '')
            
            # 参数验证
            if not camera_id:
                logger.error('播放视频命令缺少必要参数: cameraId')
                self.send_ack_response(message_data, 400, '缺少必要参数: cameraId', {})
                return
            
            if not push_url:
                logger.error('播放视频命令缺少必要参数: pushUrl')
                self.send_ack_response(message_data, 400, '缺少必要参数: pushUrl', {})
                return
            
            # 检查传输类型是否支持
            if trans_type not in ['RTMP', 'RTSP']:
                logger.warning(f'不支持的传输类型: {trans_type}，将使用默认值: RTMP')
                trans_type = 'RTMP'
            
            # 停止该摄像头当前可能正在运行的流
            self._stop_stream(camera_id)
            
            # 启动新的视频流
            success = self._start_stream(camera_id, trans_type, push_url, username, password)
            
            if success:
                logger.info(f'成功启动视频流: cameraId={camera_id}, type={trans_type}, url={push_url}')
                self.send_ack_response(message_data, 200, 'ok', {})
            else:
                logger.error(f'启动视频流失败: cameraId={camera_id}')
                self.send_ack_response(message_data, 500, '启动视频流失败', {})
                
        except Exception as e:
            logger.error(f'处理播放视频命令时出错: {str(e)}')
            self.send_ack_response(message_data, 500, f'处理错误: {str(e)}', {})
    
    def process_stop_video(self, message_data):
        """
        处理停止视频命令
        
        Args:
            message_data: 包含停止视频命令的消息数据
        """
        try:
            # 提取请求参数
            body = message_data.get('body', {})
            camera_id = body.get('cameraId', '')
            
            # 参数验证
            if not camera_id:
                logger.error('停止视频命令缺少必要参数: cameraId')
                self.send_ack_response(message_data, 400, '缺少必要参数: cameraId', {})
                return
            
            # 停止视频流
            success = self._stop_stream(camera_id)
            
            if success:
                logger.info(f'成功停止视频流: cameraId={camera_id}')
                self.send_ack_response(message_data, 200, 'ok', {})
            else:
                logger.warning(f'视频流不存在或已停止: cameraId={camera_id}')
                self.send_ack_response(message_data, 200, '视频流不存在或已停止', {})
                
        except Exception as e:
            logger.error(f'处理停止视频命令时出错: {str(e)}')
            self.send_ack_response(message_data, 500, f'处理错误: {str(e)}', {})
    
    def _start_stream(self, camera_id, trans_type, push_url, username, password, is_restart=False):
        """
        启动视频流
        
        Args:
            camera_id: 摄像机ID
            trans_type: 传输类型 (RTMP/RTSP)
            push_url: 推流URL
            username: 用户名
            password: 密码
            
        Returns:
            bool: 是否成功启动
        """
        try:
            # 检查camera_id是否在配置中
            if camera_id not in self.camera_rtsp_addresses:
                logger.error(f'未找到cameraId: {camera_id}的RTSP配置')
                return False
            
            rtsp_url = self.camera_rtsp_addresses[camera_id]

            # 记录推流参数供断流后自动重连复用；非重连（新播放命令）时重置计数与停止事件
            with self.streams_lock:
                self._stream_params[camera_id] = {
                    'trans_type': trans_type,
                    'push_url': push_url,
                    'username': username,
                    'password': password,
                }
                if not is_restart:
                    self._stop_events[camera_id] = Event()
                    self._restart_counts[camera_id] = 0

            cmd = []
            if CODEC == "h264":
                # 构建ffmpeg命令，参考用户提供的C++实现
                cmd = [
                    'ffmpeg',
                    '-rtsp_transport', 'tcp',  # 使用TCP传输，提高稳定性
                    '-timeout', '5000000',    # 设置超时时间，单位微秒
                    '-i', rtsp_url,            # 输入RTSP地址
                    '-c:v', 'copy',            # 视频使用copy模式，不重新编码
                    '-r', str(PREVIEW_FPS),    # 输出限帧（copy 模式下按时间戳丢帧）
                    '-c:a', 'aac',             # 音频使用AAC编码
                    '-f', 'flv',               # 输出格式为FLV
                    '-y'                       # 覆盖已存在的文件
                ]
            elif CODEC == "h265":
                cmd = [
                    'ffmpeg',
                    '-rtsp_transport', 'tcp',  # 使用TCP传输，提高稳定性
                    '-timeout', '5000000',    # 设置超时时间，单位微秒
                    '-i', rtsp_url,            # 输入RTSP地址
                    '-c:v', 'h264_rkmpp',            # 视频使用copy模式，不重新编码
                    '-pix_fmt', 'yuv420p',
                    '-profile:v', 'baseline',
                    '-r', str(PREVIEW_FPS),    # 输出限帧
                    '-g', str(PREVIEW_FPS * 2),        # 关键帧间隔约 2s（跟随限帧后的帧率）
                    '-keyint_min', str(PREVIEW_FPS * 2),
                    '-an',
                    '-f', 'flv',               # 输出格式为FLV
                    '-y'                       # 覆盖已存在的文件
                ]
            
            # 如果有用户名和密码，添加认证信息到推流URL
            if username and password:
                auth_url = push_url.replace('://', f'://{username}:{password}@')
                cmd.append(auth_url)
            else:
                cmd.append(push_url)
            
            # 记录命令
            logger.info(f'准备启动视频流命令: {" ".join(cmd)}')
            
            # ffmpeg 的 stderr 落地到文件：devnull 会让异常退出（如 returncode 187）
            # 完全失去原因，异常退出时打印日志尾部便于定位
            log_path = os.path.join('/tmp', f'ffmpeg_{camera_id}.log')
            devnull = open(os.devnull, 'w')
            try:
                # 重连时追加写，保留上一次异常退出的上下文
                log_file = open(log_path, 'a' if is_restart else 'w')
            except OSError as e:
                logger.warning(f'无法创建 ffmpeg 日志 {log_path}: {e}，回退到 devnull')
                log_path, log_file = '', devnull
            # 启动ffmpeg子进程，stdout 丢弃、stderr 写日志文件，避免输出阻塞
            process = subprocess.Popen(
                cmd,
                stdout=devnull,
                stderr=log_file,
                shell=False,
                preexec_fn=os.setsid  # 设置进程组，方便后续kill整个进程组
            )
            
            # 存储进程信息
            with self.streams_lock:
                self.running_streams[camera_id] = {
                    'process': process,
                    'trans_type': trans_type,
                    'push_url': push_url,
                    'start_time': time.time()
                }
            
            # 启动一个线程来监控进程状态
            monitor_thread = Thread(target=self._monitor_stream,
                                    args=(camera_id, process, log_path, log_file), daemon=True)
            monitor_thread.start()
            
            logger.info(f'启动预览[{camera_id}] 推送至[{push_url}] 成功')
            return True
            
        except Exception as e:
            logger.error(f'启动视频流时出错: {str(e)}')
            return False
    
    def _stop_stream(self, camera_id):
        """
        停止视频流
        
        Args:
            camera_id: 摄像机ID
            
        Returns:
            bool: 是否成功停止
        """
        try:
            with self.streams_lock:
                # 无论进程是否在运行都置位停止事件，取消可能仍在等待的重连
                stop_event = self._stop_events.get(camera_id)
                if stop_event is not None:
                    stop_event.set()
                if camera_id not in self.running_streams:
                    return False
                
                stream_info = self.running_streams.pop(camera_id)
                process = stream_info['process']
                self._expected_stop.add(camera_id)
            
            # 尝试优雅地终止进程
            try:
                # 先尝试terminate
                process.terminate()
                # 等待一段时间让进程正常终止
                time.sleep(1)
                
                # 如果进程还在运行，强制杀死整个进程组
                if process.poll() is None:
                    logger.warning(f'进程仍在运行，尝试强制终止: {process.pid}')
                    # 使用os.killpg杀死整个进程组，确保所有子进程都被终止
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    time.sleep(0.5)
            except Exception as e:
                logger.error(f'停止视频流进程时出错: {str(e)}')
            
            logger.info(f'成功停止视频流: cameraId={camera_id}')
            return True
            
        except Exception as e:
            logger.error(f'停止视频流时出错: {str(e)}')
            return False
    def _monitor_stream(self, camera_id, process, log_path='', log_file=None):
        """
        监控视频流进程状态
        
        Args:
            camera_id: 摄像机ID
            process: 进程对象
        """
        try:
            # 等待进程结束
            process.wait()
            
            # 进程结束后，清理资源
            with self.streams_lock:
                info = self.running_streams.pop(camera_id, None)
                expected_stop = camera_id in self._expected_stop
                self._expected_stop.discard(camera_id)
                # 稳定运行超过阈值则清零失败计数，避免长跑后偶发断流被旧计数拖死
                if info and time.time() - info.get('start_time', 0) >= PREVIEW_RETRY_RESET_SEC:
                    self._restart_counts[camera_id] = 0
            
            # 检查返回码
            if process.returncode != 0:
                if expected_stop:
                    # 由 _stop_stream 主动终止（如切换推流地址），不是故障
                    logger.info(f'视频流已按请求停止: cameraId={camera_id}, returncode={process.returncode}')
                else:
                    logger.error(f'视频流进程异常退出: cameraId={camera_id}, returncode={process.returncode}')
                    self._log_ffmpeg_tail(camera_id, log_path)
                    self._schedule_restart(camera_id)
            else:
                logger.info(f'视频流进程正常退出: cameraId={camera_id}')
                
        except Exception as e:
            logger.error(f'监控视频流进程时出错: {str(e)}')
            # 确保资源被清理
            with self.streams_lock:
                if camera_id in self.running_streams:
                    del self.running_streams[camera_id]
        finally:
            if log_file:
                try:
                    log_file.close()
                except Exception:
                    pass

    def _schedule_restart(self, camera_id):
        """断流后按指数退避重连；由停止事件取消，连续失败超过上限则放弃"""
        with self.streams_lock:
            params = self._stream_params.get(camera_id)
            stop_event = self._stop_events.get(camera_id)
            attempt = self._restart_counts.get(camera_id, 0) + 1
            self._restart_counts[camera_id] = attempt

        if not params or stop_event is None or stop_event.is_set():
            return
        # 上限为 0 表示不限次数：外网中断期间持续重试，直到收到停止命令
        if PREVIEW_RETRY_MAX > 0 and attempt > PREVIEW_RETRY_MAX:
            logger.error(f'视频流连续 {PREVIEW_RETRY_MAX} 次重连失败，放弃重连: cameraId={camera_id}')
            return

        delay = min(PREVIEW_RETRY_DELAY * (2 ** (attempt - 1)), 30)
        limit = f'/{PREVIEW_RETRY_MAX}' if PREVIEW_RETRY_MAX > 0 else ''
        logger.warning(f'视频流断开，{delay}s 后第 {attempt}{limit} 次重连: cameraId={camera_id}')
        # 等待期间可能收到停止请求，或已下发新的播放命令（停止事件被替换）
        cancelled = stop_event.wait(delay) or self._stop_events.get(camera_id) is not stop_event
        if cancelled:
            logger.info(f'重连已取消（收到停止请求或新指令）: cameraId={camera_id}')
            return
        logger.info(f'开始第 {attempt} 次重连: cameraId={camera_id}')

        ok = self._start_stream(camera_id, params['trans_type'], params['push_url'],
                                params['username'], params['password'], is_restart=True)
        if not ok:
            # 启动阶段就直接失败（如服务不可用），继续按退避重试
            self._schedule_restart(camera_id)

    def _log_ffmpeg_tail(self, camera_id, log_path, lines=20):
        """异常退出时打印 ffmpeg 日志尾部，用于定位退出码原因"""
        if not log_path or not os.path.isfile(log_path):
            logger.error(f'视频流可能需要重新启动: cameraId={camera_id}（无 ffmpeg 日志）')
            return
        try:
            with open(log_path, 'r', errors='ignore') as f:
                tail = f.read().splitlines()[-lines:]
        except OSError as e:
            logger.error(f'读取 ffmpeg 日志失败: {e}')
            return
        logger.error(f'ffmpeg 日志尾部（{log_path}）:{os.linesep}' + os.linesep.join(tail))
        logger.error(f'视频流可能需要重新启动: cameraId={camera_id}')

    def send_ack_response(self, original_message, status, error, body):
        """
        发送ACK响应消息
        
        Args:
            original_message: 原始请求消息
            status: 状态码 (200=成功, 其他=失败)
            error: 错误描述
            body: 响应体数据
        """
        try:
            # 构建响应消息
            response = {
                'sn': original_message.get('sn', ''),
                'ver': original_message.get('ver', '1.0'),
                'seq': original_message.get('seq', 0),
                'type': 2,
                'ts': int(time.time() * 1000),
                'cmd': 'Ack',
                'status': status,
                'error': error,
                'body': body
            }
            
            # 发送响应
            response_json = json.dumps(response, ensure_ascii=False)
            self.mqtt_client.publish(self.response_topic, response_json)
            logger.info(f'已发送视频预览响应: {response_json}')
            
        except Exception as e:
            logger.error(f'发送视频预览响应失败: {str(e)}')
    
    def get_camera_status(self):
        """
        获取所有摄像机的状态信息
        
        Returns:
            List[dict]: 摄像机状态列表，每个元素包含cameraId和status字段
            status字段是一个位标志，格式为：
            - bit0（最低位）: 在线状态（1表示在线，0表示离线）
            - bit1: 故障状态（1表示故障，0表示正常）
            - bit2: 预览状态（1表示正在预览，0表示未预览）
            - bit3: 录像状态（1表示正在录像，0表示未录像）
        """
        camera_status_list = []
        
        # 仿真环境视频已禁用：不探测 RTSP，直接全部上报为离线
        if not ENABLE_VIDEO:
            for camera_id in self.camera_rtsp_addresses.keys():
                camera_status_list.append({'cameraId': camera_id, 'status': 0})
            return camera_status_list
        
        try:
            # 遍历所有配置的摄像头ID
            for camera_id in self.camera_rtsp_addresses.keys():
                status = 0  # 默认状态：离线、其他状态暂不设置
                # 检查在线状态：通过ping摄像头IP地址来判断
                rtsp_url = self.camera_rtsp_addresses[camera_id]
                # 使用OpenCV尝试打开RTSP流来检测摄像头是否在线
                try:
                    # 设置视频捕获超时时间
                    success = False
                    success = self.is_rtsp_online(rtsp_url)
                    
                    if success:
                        status |= 1  # 设置bit0为1，表示在线
                        logger.debug(f'摄像头[{camera_id}] RTSP流[{rtsp_url}]可以正常打开，状态为在线')
                    else:
                        logger.debug(f'摄像头[{camera_id}] RTSP流[{rtsp_url}]无法打开，状态为离线')
                    
                except Exception as e:
                    logger.error(f'使用OpenCV检测摄像头[{camera_id}]状态时出错: {str(e)}')
                    # 出错时保持默认的离线状态
                
                # 添加到状态列表
                camera_status_list.append({
                    'cameraId': camera_id,
                    'status': status
                })
                
            logger.debug(f'获取摄像机状态: {camera_status_list}')
            return camera_status_list
            
        except Exception as e:
            logger.error(f'获取摄像机状态时出错: {str(e)}')
            return []

    def is_rtsp_online(self, rtsp_url, timeout_sec=2):
        """
        检查RTSP流是否在线
        
        Args:
            rtsp_url: RTSP流地址
            timeout_sec: 超时时间（秒），默认2秒
            
        Returns:
            bool: 如果流可以正常打开且返回数据，则返回True；否则返回False
        """
        cmd = [
            "ffmpeg",
            "-rtsp_transport", "tcp",
            "-timeout", str(timeout_sec * 1_000_000),
            "-i", rtsp_url,
            "-t", "1",
            "-f", "null",
            "-"
        ]

        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=timeout_sec + 1
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            return False
 
    def shutdown(self):
        """
        关闭视频预览管理器，清理所有资源
        """
        # 停止所有正在运行的视频流
        camera_ids = list(self.running_streams.keys())
        for camera_id in camera_ids:
            self._stop_stream(camera_id)
        
        logger.info('视频预览管理器已关闭')