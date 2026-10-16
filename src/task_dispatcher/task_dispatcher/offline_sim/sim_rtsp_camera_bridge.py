#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真 RTSP 相机源：Gazebo 机载相机图像 -> ffmpeg 推流 -> MediaMTX。

背景：Gazebo 不输出 RTSP，而在线链路的视频功能（RtspRecordManager 录像/抓图、
VideoPreviewManager 预览推流）都以 RTSP URL 为输入。本节点把 ros_gz_bridge 转发
出来的 OakD-Lite 可见光图像（默认 /<ns>/IMX214/image_raw）用 ffmpeg 编成 H.264
推到本机 MediaMTX（默认 rtsp://127.0.0.1:8554/main.264），与真机云台相机的
MediaMTX 出流拓扑一致，task_dispatcher 侧代码零改动。

仅 online_sim 套件使用（offline 无视频需求，不启动本节点）。

实现要点：
    - 不依赖 cv_bridge：手动按 Image 消息布局取原始字节，支持 rgb8/bgr8/rgba8/mono8
      （venv 里没有 ROS 的 cv_bridge，也避免编码转换损耗——rgb8 直接零拷贝透传）。
    - ffmpeg 查找顺序：--ffmpeg-bin > FFMPEG_BIN 环境变量 > PATH > imageio-ffmpeg 自带。
    - 限帧（默认 15fps）：仿真相机 30Hz，抽帧降低 CPU/带宽。
    - 推流进程异常退出（MediaMTX 未就绪/被断开）自动退避重启（2s 起、上限 30s）。
    - lazy bridge：本节点订阅后 gz 才开始发布图像，无需额外操作。

用法：
    python3 sim_rtsp_camera_bridge.py \
        --image-topic /x500_depth_0/IMX214/image_raw \
        --rtsp-url rtsp://127.0.0.1:8554/main.264 [--fps 15] [--ffmpeg-bin /path/ffmpeg]
"""

import argparse
import os
import queue
import shutil
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# Image.encoding -> ffmpeg rawvideo 输入像素格式
_PIX_FMT = {
    'rgb8': 'rgb24',
    'bgr8': 'bgr24',
    'rgba8': 'rgba',
    'mono8': 'gray',
}


def find_ffmpeg(explicit: str = '') -> str:
    """定位可用的 ffmpeg 可执行文件。

    顺序：--ffmpeg-bin > FFMPEG_BIN > imageio-ffmpeg 自带 > PATH。
    imageio-ffmpeg 排在 PATH 之前：IDE 终端（如 Trae）会往 PATH 注入精简版
    ffmpeg（缺 rtsp muxer/libx264，推流必败），venv 内的静态构建功能已验证完整。
    """
    for cand in (explicit, os.environ.get('FFMPEG_BIN')):
        if cand and os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        pass
    return shutil.which('ffmpeg') or ''


class RtspPusher:
    """把原始视频帧写进 ffmpeg stdin。

    设计要点：
      - 专属写线程 + 有界队列（maxsize=3）：1080p 一帧约 6MB，直接同步写 stdin 时
        若 ffmpeg 卡住（服务端断连后写 socket 阻塞），会拖死 rclpy 回调线程，
        表现为“进程都活着但整条链僵尸”。现在队列满即丢帧，写失败才判死。
      - -stimeout/-timeout: ffmpeg 侧 socket I/O 超时，断连后 ffmpeg 自行退出，
        poll() 能查到，触发重建。
      - stderr 透传：推流错误（403/断开等）直接上 launch 屏幕，便于排查。
    """

    def __init__(self, ffmpeg_bin: str, url: str, width: int, height: int,
                 pix_fmt: str, fps: int):
        self._cmd = [
            ffmpeg_bin, '-hide_banner', '-loglevel', 'warning', '-nostdin',
            '-f', 'rawvideo', '-pix_fmt', pix_fmt, '-s', f'{width}x{height}',
            '-framerate', str(fps), '-i', 'pipe:0',
            '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-tune', 'zerolatency',
            '-g', str(fps * 2), '-pix_fmt', 'yuv420p',
            # rtsp 输出侧 socket I/O 超时（微秒）：断连后 ffmpeg 自行退出可被重建；
            # 注意 imageio-ffmpeg 7.x 不支持输入侧 -stimeout，别挪到 -i 前面
            '-f', 'rtsp', '-rtsp_transport', 'tcp', '-timeout', '5000000', url,
        ]
        self._proc = subprocess.Popen(self._cmd, stdin=subprocess.PIPE,
                                      stdout=subprocess.DEVNULL)
        self._q: 'queue.Queue[bytes]' = queue.Queue(maxsize=3)
        self._dead = threading.Event()
        self._writer = threading.Thread(target=self._pump, daemon=True)
        self._writer.start()

    def _pump(self) -> None:
        while not self._dead.is_set():
            try:
                data = self._q.get(timeout=0.5)
            except queue.Empty:
                if self._proc.poll() is not None:
                    self._dead.set()
                    return
                continue
            try:
                self._proc.stdin.write(data)
                self._proc.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                self._dead.set()
                return

    @property
    def alive(self) -> bool:
        return not self._dead.is_set() and self._proc.poll() is None

    def write(self, data: bytes) -> bool:
        """入队一帧；返回 False 表示推流已死需重建（队列满丢帧返回 True）。"""
        if not self.alive:
            return False
        try:
            self._q.put_nowait(data)
        except queue.Full:
            pass
        return True

    def stop(self) -> None:
        self._dead.set()
        try:
            if self._proc.stdin:
                self._proc.stdin.close()
        except OSError:
            pass
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()


class SimRtspCameraBridge(Node):
    def __init__(self, args):
        super().__init__('sim_rtsp_camera_bridge')
        self.image_topic = args.image_topic
        self.rtsp_url = args.rtsp_url
        self.fps = max(1, int(args.fps))
        self.ffmpeg_bin = find_ffmpeg(args.ffmpeg_bin)
        if not self.ffmpeg_bin:
            self.get_logger().fatal('找不到 ffmpeg（安装 ffmpeg 或 pip install imageio-ffmpeg）')
            raise SystemExit(2)

        self._pusher = None
        self._size = None
        self._pix_fmt = None
        self._last_frame_t = 0.0
        self._restart_delay = 2.0
        self._restart_at = 0.0
        self._frames = 0

        self.create_subscription(Image, self.image_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f'RTSP 相机桥: {self.image_topic} -> {self.rtsp_url} '
            f'(ffmpeg={self.ffmpeg_bin}, fps={self.fps})')

    def _on_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last_frame_t < 1.0 / self.fps:
            return
        pix_fmt = _PIX_FMT.get(msg.encoding)
        need = msg.width * msg.height * self._bpp(pix_fmt) if pix_fmt else -1
        if pix_fmt is None or len(msg.data) < need:
            # 不认识的编码或长度异常：低频告警后丢帧
            self.get_logger().warn(f'跳过帧：encoding={msg.encoding} len={len(msg.data)}',
                                   every=200)
            return
        self._last_frame_t = now

        if self._pusher is None or not self._pusher.alive:
            if now < self._restart_at:
                return
            failed = self._pusher is not None
            if failed:
                self._pusher.stop()
                self._restart_delay = min(self._restart_delay * 2, 30.0)
            else:
                self._restart_delay = 2.0
            self._pusher = self._start_pusher(msg.width, msg.height, pix_fmt)
            if self._pusher is None:
                self._restart_at = now + self._restart_delay
                return
            self._restart_delay = 2.0
            self._size = (msg.width, msg.height, pix_fmt)
            self.get_logger().info(
                f'推流已启动: {msg.width}x{msg.height} {pix_fmt}'
                + ('（重启恢复）' if failed else ''))

        if (msg.width, msg.height, pix_fmt) != self._size:
            # 分辨率/格式变了：重建推流进程
            self.get_logger().info('图像参数变化，重建推流')
            self._pusher.stop()
            self._pusher = self._start_pusher(msg.width, msg.height, pix_fmt)
            self._size = (msg.width, msg.height, pix_fmt)
            if self._pusher is None:
                self._restart_at = now + self._restart_delay
                return

        if not self._pusher.write(bytes(msg.data)):
            self.get_logger().warn('写帧失败，推流进程将重建')
            self._pusher.stop()
            self._pusher = None
            self._restart_at = time.monotonic() + self._restart_delay
            return
        self._frames += 1
        if self._frames == 1 or self._frames % (self.fps * 30) == 0:
            self.get_logger().info(f'已推 {self._frames} 帧 -> {self.rtsp_url}')

    @staticmethod
    def _bpp(pix_fmt: str) -> int:
        return {'rgb24': 3, 'bgr24': 3, 'rgba': 4, 'gray': 1}[pix_fmt]

    def _start_pusher(self, w: int, h: int, pix_fmt: str):
        try:
            return RtspPusher(self.ffmpeg_bin, self.rtsp_url, w, h, pix_fmt, self.fps)
        except OSError as exc:
            self.get_logger().error(f'启动 ffmpeg 失败: {exc}')
            return None

    def shutdown(self):
        if self._pusher:
            self._pusher.stop()


def main():
    parser = argparse.ArgumentParser(description='gz 相机图像 -> RTSP 推流')
    parser.add_argument('--image-topic', default='/x500_depth_0/IMX214/image_raw')
    parser.add_argument('--rtsp-url', default='rtsp://127.0.0.1:8554/main.264')
    parser.add_argument('--fps', type=int, default=15)
    parser.add_argument('--ffmpeg-bin', default='')
    # launch 可能透传 ros-args，忽略之
    args, _unknown = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = SimRtspCameraBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
