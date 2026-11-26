#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PX4 时间同步模块 —— 开机自动把机载板 UTC 时间通过 MAVLink 传给飞控。

PX4 内部时钟优先级：GPS UTC > RTC 备份电池 > 开机时间。
飞控无 RTC 电池且 GPS 未锁星时，日志时间戳会是 2000 年附近的错误值。
本模块在后台线程中周期性发送 SYSTEM_TIME 消息，直到飞控时间被正确设置。
"""

import glob
import logging
import os
import time
import threading

logger = logging.getLogger('task_dispatcher')


class Px4TimeSync:
    """通过 MAVLink 向 PX4 飞控同步时间的后台线程。

    启动后在后台重试连接并发送 SYSTEM_TIME，成功一次后即停止。
    如果飞控有 GPS 且已锁星，GPS 时间会自动覆盖，本模块不影响。
    """

    def __init__(self, device=None, baudrate=57600, udp_port=None, udp_out=None,
                 max_retries=30, retry_interval=5.0):
        """
        Args:
            device: 串口设备路径，如 /dev/ttyACM0。None 则自动探测。
            baudrate: 串口波特率，默认 57600（TELEM 口默认值）。
            udp_port: UDP 监听端口，与 device 互斥。
            udp_out: UDP 目标地址，与 device 互斥。
            max_retries: 最大重试次数，默认 30（约 2.5 分钟）。
            retry_interval: 重试间隔秒数，默认 5。
        """
        self.device = device
        self.baudrate = baudrate
        self.udp_port = udp_port
        self.udp_out = udp_out
        self.max_retries = max_retries
        self.retry_interval = retry_interval

        self._thread = None
        self._stop_event = threading.Event()
        self._synced = False

    def start(self):
        """启动后台同步线程"""
        self._stop_event.clear()
        self._synced = False
        self._thread = threading.Thread(target=self._run, daemon=True, name='px4_time_sync')
        self._thread.start()
        logger.info('PX4 时间同步线程已启动')

    def stop(self):
        """停止同步线程"""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=5)
        logger.info('PX4 时间同步线程已停止')

    @property
    def synced(self):
        return self._synced

    def _run(self):
        """后台线程主循环：尝试连接飞控并发送时间"""
        try:
            from pymavlink import mavutil
        except ImportError:
            logger.error('pymavlink 未安装，跳过 PX4 时间同步')
            return

        for attempt in range(1, self.max_retries + 1):
            if self._stop_event.is_set():
                return

            master = None
            try:
                master = self._open_link(mavutil)
                if master is None:
                    logger.debug(f'PX4 时间同步：第 {attempt}/{self.max_retries} 次未找到链路，'
                                 f'{self.retry_interval}s 后重试')
                    self._stop_event.wait(self.retry_interval)
                    continue

                # 等心跳
                hb = self._wait_heartbeat(master, timeout=5)
                if hb is None:
                    logger.debug(f'PX4 时间同步：第 {attempt}/{self.max_retries} 次等心跳超时，'
                                 f'{self.retry_interval}s 后重试')
                    master.close()
                    self._stop_event.wait(self.retry_interval)
                    continue

                # 发送 SYSTEM_TIME（发 3 次确保收到）
                for i in range(3):
                    if self._stop_event.is_set():
                        master.close()
                        return
                    utc_usec = int(time.time() * 1e6)
                    master.mav.system_time_send(utc_usec, 0)
                    if i < 2:
                        time.sleep(0.5)

                utc_str = time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime(utc_usec / 1e6))
                logger.info(f'PX4 时间同步成功: {utc_str}（第 {attempt} 次尝试）')
                self._synced = True
                master.close()
                return

            except Exception as e:
                logger.warning(f'PX4 时间同步异常（第 {attempt} 次）: {e}')
                if master:
                    try:
                        master.close()
                    except Exception:
                        pass
                self._stop_event.wait(self.retry_interval)

        logger.warning(f'PX4 时间同步：已重试 {self.max_retries} 次仍未成功，放弃')

    def _open_link(self, mavutil):
        """打开 MAVLink 连接"""
        if self.device:
            if not os.path.exists(self.device):
                return None
            try:
                return mavutil.mavlink_connection(self.device, baud=self.baudrate)
            except Exception:
                return None
        elif self.udp_port is not None:
            try:
                return mavutil.mavlink_connection(f'udpin:0.0.0.0:{self.udp_port}')
            except Exception:
                return None
        elif self.udp_out:
            try:
                return mavutil.mavlink_connection(f'udpout:{self.udp_out}')
            except Exception:
                return None
        else:
            # 自动探测串口
            for pat in ('/dev/ttyACM*', '/dev/ttyUSB*'):
                for dev in sorted(glob.glob(pat)):
                    try:
                        cand = mavutil.mavlink_connection(dev, baud=self.baudrate)
                    except Exception:
                        continue
                    if self._wait_heartbeat(cand, timeout=2) is not None:
                        return cand
                    cand.close()
            return None

    def _wait_heartbeat(self, master, timeout=5):
        """等待飞控心跳"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                master.mav.heartbeat_send(
                    master.mavlink.MAV_TYPE_GCS,
                    master.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            hb = master.wait_heartbeat(timeout=0.5)
            if hb is not None:
                return hb
        return None
