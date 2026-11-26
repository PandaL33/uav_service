#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PX4 时间同步模块 —— 开机自动把机载板 UTC 时间通过 MAVLink 传给飞控。

PX4 内部时钟优先级：GPS UTC > RTC 备份电池 > 开机时间。
飞控无 RTC 电池且 GPS 未锁星时，日志时间戳会是板载 RTC 里存着的旧值。

本模块只在**节点启动后的一个短窗口内**（默认 30s）尝试同步，成功或窗口耗尽即
退出线程，之后运行期不再碰飞控时钟 —— 避免飞行途中时钟跳变污染日志与估计器。
"""

import glob
import logging
import os
import time
import threading

logger = logging.getLogger('task_dispatcher')


class Px4TimeSync:
    """通过 MAVLink 向 PX4 飞控同步时间的后台线程。

    安全边界：只在节点启动后 `window` 秒内尝试，成功或窗口耗尽即退出线程。
    运行期不再碰飞控时钟，避免飞行途中 SYSTEM_TIME 导致日志时间戳跳变或
    估计器状态异常。

    每次同步都是「开链路 -> 发几条 SYSTEM_TIME -> 立刻关链路」，不长期占用串口，
    以免挡住 px4_log_pull.py 用同一个口拉日志。
    """

    def __init__(self, device=None, baudrate=57600, udp_port=None, udp_out=None,
                 retry_interval=2.0, window=30.0):
        """
        Args:
            device: 串口设备路径，如 /dev/ttyACM0。None 则自动探测。
            baudrate: 串口波特率，默认 57600（TELEM 口默认值）。
            udp_port: UDP 监听端口，与 device 互斥。
            udp_out: UDP 目标地址，与 device 互斥。
            retry_interval: 两次尝试之间的最小间隔秒数（实际间隔 = 本次尝试
                            耗时 + retry_interval，且不超过 window）。
            window: 自线程启动起，最多尝试多少秒。超时即放弃，线程退出。
                    默认 30；设 0 表示完全禁用同步。
        """
        self.device = device
        self.baudrate = baudrate
        self.udp_port = udp_port
        self.udp_out = udp_out
        self.retry_interval = retry_interval
        self.window = window

        self._thread = None
        self._stop_event = threading.Event()
        self._synced = False

    def start(self):
        """启动后台同步线程。window<=0 时直接禁用。"""
        if self.window <= 0:
            logger.info('PX4 时间同步已禁用（window=0）')
            return
        self._stop_event.clear()
        self._synced = False
        self._thread = threading.Thread(target=self._run, daemon=True, name='px4_time_sync')
        self._thread.start()
        logger.info(f'PX4 时间同步线程已启动（窗口 {self.window:.0f}s）')

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
        """后台线程主循环：在 window 内反复尝试，超时即退出。"""
        try:
            from pymavlink import mavutil
        except ImportError:
            logger.error('pymavlink 未安装，跳过 PX4 时间同步')
            return

        deadline = time.time() + self.window
        attempt = 0

        while not self._stop_event.is_set() and time.time() < deadline:
            attempt += 1
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            # 单次尝试的心跳等待不超过剩余窗口，避免越界
            hb_timeout = min(5.0, max(remaining - 0.5, 0.1))
            ok, utc_usec = self._sync_once(mavutil, attempt, hb_timeout)

            if ok:
                self._synced = True
                utc_str = time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime(utc_usec / 1e6))
                logger.info(f'PX4 时间同步成功: {utc_str}（第 {attempt} 次尝试，'
                            f'耗时 {self.window - remaining:.1f}s）')
                return

            # 失败：等 retry_interval 或剩余窗口（取小者）再试
            wait = min(self.retry_interval, deadline - time.time())
            if wait > 0:
                self._stop_event.wait(wait)

        if not self._synced:
            logger.warning(f'PX4 时间同步：{self.window:.0f}s 窗口内 {attempt} 次尝试均未成功，放弃')

    def _sync_once(self, mavutil, attempt, hb_timeout=5.0):
        """做一次同步尝试。成功返回 (True, utc_usec)，失败返回 (False, 0)。"""
        master = None
        try:
            master = self._open_link(mavutil)
            if master is None:
                return False, 0

            hb = self._wait_heartbeat(master, timeout=hb_timeout)
            if hb is None:
                master.close()
                return False, 0

            # 发 3 次确保收到（串口丢包常见）
            utc_usec = 0
            for i in range(3):
                if self._stop_event.is_set():
                    master.close()
                    return False, 0
                utc_usec = int(time.time() * 1e6)
                master.mav.system_time_send(utc_usec, 0)
                if i < 2:
                    time.sleep(0.5)

            if attempt > 1:
                logger.info(f'PX4 时间同步：第 {attempt} 次尝试成功')
            master.close()
            return True, utc_usec

        except Exception as e:
            logger.warning(f'PX4 时间同步异常（第 {attempt} 次）: {e}')
            if master:
                try:
                    master.close()
                except Exception:
                    pass
            return False, 0

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
