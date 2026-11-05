#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
性能统计日志（默认关闭，见 config.ENABLE_PERF_LOG）。

排查现场性能时，真正决定开销的两个量都在节点自己手里：
  1) 每个话题实际到达的消息频率（回调被调用的次数）
  2) 节点常驻的 DDS 实体数（rclpy 每处理一条消息都要用全部实体重建一次 WaitSet）
以前只能靠 py-spy / ros2 node info 外部手数，这里直接读自身与 /proc 打出来。

只读 /proc，不引入 psutil 等依赖；采样动作只在打印那一刻发生。
"""

import logging
import os
import time

logger = logging.getLogger('perf_monitor')

_CLK_TCK = os.sysconf('SC_CLK_TCK') if hasattr(os, 'sysconf') else 100
_PAGE_SIZE = os.sysconf('SC_PAGE_SIZE') if hasattr(os, 'sysconf') else 4096
_SELF_STAT = '/proc/self/stat'
_SELF_STATM = '/proc/self/statm'
_SELF_TASKS = '/proc/self/task'


def _proc_times_ticks(pid=None):
    """返回进程累计 CPU 时间（tick）。失败返回 None"""
    path = _SELF_STAT if pid is None else f'/proc/{pid}/stat'
    try:
        with open(path, 'r') as f:
            fields = f.read().rsplit(') ', 1)[1].split()
        # 去掉 "pid (comm) " 前缀后：utime 是第 11 个、stime 第 12 个
        return (int(fields[11]) + int(fields[12])) / float(_CLK_TCK)
    except (OSError, IndexError, ValueError):
        return None


def _proc_rss_mb(pid=None):
    path = _SELF_STATM if pid is None else f'/proc/{pid}/statm'
    try:
        with open(path, 'r') as f:
            return int(f.read().split()[1]) * _PAGE_SIZE / 1048576.0
    except (OSError, IndexError, ValueError):
        return 0.0


def _thread_count():
    try:
        return len(os.listdir(_SELF_TASKS))
    except OSError:
        return 0


class PerfMonitor:
    """
    订阅回调计数 + 周期性性能摘要。

    安装方式（仅当开关打开时才包装 create_subscription，关闭时零改动零开销）：

        monitor = PerfMonitor(node, None, interval)
        monitor.install()                 # 必须早于任何 create_subscription
        ...创建各管理器与订阅...
        monitor.streams_owner = video_preview_manager
        monitor.start()
    """

    def __init__(self, node, streams_owner=None, interval=60.0, top_n=3):
        self.node = node
        self.streams_owner = streams_owner
        self.interval = max(5.0, float(interval))
        self.top_n = top_n
        self._stats = {}
        self._prev_at = time.monotonic()
        self._prev_cpu = _proc_times_ticks() or 0.0
        self._prev_entities = None
        self._child_cpu = {}
        self._timer = None

    # ---------- 订阅计数 ----------
    def install(self):
        """包装 node.create_subscription，为每个话题统计调用次数与累计回调耗时"""
        original = self.node.create_subscription

        def create_subscription(msg_type, topic, callback, qos_profile=10, *args, **kwargs):
            entry = self._stats.setdefault(topic, {'count': 0, 'ms': 0.0})

            def counted(msg, _cb=callback, _entry=entry):
                started = time.perf_counter()
                try:
                    return _cb(msg)
                finally:
                    _entry['count'] += 1
                    _entry['ms'] += (time.perf_counter() - started) * 1000.0

            return original(msg_type, topic, counted, qos_profile, *args, **kwargs)

        self.node.create_subscription = create_subscription
        logger.info(f'性能统计已开启，周期 {self.interval:g}s（回调包装已安装）')

    # ---------- 实体数 ----------
    def _count(self, attr):
        """
        统计 node 上某一组实体数量。

        rclpy 的 Node.subscriptions/timers/clients/services/guards/waitables 都是
        generator 属性（yield from 内部列表），不能直接 len()；这里用一次遍历计数，
        同时兼容列表型实现。属性缺失（不同版本）返回 0 而不是抛异常。
        """
        entities = getattr(self.node, attr, None)
        if entities is None:
            return 0
        try:
            return sum(1 for _ in entities)
        except TypeError:
            return 0

    def _entity_counts(self):
        """
        统计 executor 每轮 WaitSet 重建要遍历的实体。

        waitables 里包含每个订阅/发布默认挂上的 QoS 事件处理器以及 action 服务器等，
        它们同样进入 WaitSet，所以单独计数。
        """
        counts = {
            'sub': self._count('subscriptions'),
            'timer': self._count('timers'),
            'client': self._count('clients'),
            'srv': self._count('services'),
            'guard': self._count('guards'),
            'waitable': self._count('waitables'),
        }
        counts['total'] = sum(counts.values())
        return counts

    # ---------- 推流子进程 ----------
    def _stream_lines(self):
        owner = self.streams_owner
        if owner is None:
            return ''
        try:
            with owner.streams_lock:
                items = list(owner.running_streams.items())
        except (AttributeError, OSError):
            return ''
        if not items:
            return ' | ffmpeg: 无'
        now = time.monotonic()
        window = max(1e-6, now - self._prev_at)
        alive = set()
        parts = []
        for camera_id, info in items:
            process = info.get('process')
            pid = getattr(process, 'pid', None)
            ticks = _proc_times_ticks(pid) if pid else None
            if ticks is None:
                parts.append(f'{camera_id}(已退出)')
                continue
            alive.add(camera_id)
            prev = self._child_cpu.get(camera_id)
            # 进程换了新 pid 时旧的累计值不可比，本窗口不报百分比
            cpu = (ticks - prev[0]) / window * 100.0 if prev and prev[1] == pid else 0.0
            self._child_cpu[camera_id] = (ticks, pid)
            parts.append(f'{camera_id} pid={pid} cpu={cpu:.1f}%')
        for stale in [key for key in self._child_cpu if key not in alive]:
            self._child_cpu.pop(stale)
        hw = getattr(owner, '_used_hw_decode', False)
        return f' | ffmpeg: {", ".join(parts)} ({"硬解" if hw else "软解"})'

    # ---------- 摘要 ----------
    def report(self):
        now = time.monotonic()
        window = max(1e-6, now - self._prev_at)

        cpu_ticks = _proc_times_ticks() or 0.0
        cpu_pct = (cpu_ticks - self._prev_cpu) / window * 100.0
        self._prev_cpu = cpu_ticks

        entities = self._entity_counts()
        if self._prev_entities is not None and entities['total'] > self._prev_entities:
            logger.warning(f'节点实体数增长（可能存在客户端/订阅泄漏）: '
                           f'{self._prev_entities} -> {entities["total"]} {entities}')
        self._prev_entities = entities['total']

        total_count = 0
        total_ms = 0.0
        rows = []
        for topic, entry in self._stats.items():
            total_count += entry['count']
            total_ms += entry['ms']
            rows.append((entry['count'] / window, entry['ms'] / window, topic))
            entry['count'] = 0
            entry['ms'] = 0.0
        rows.sort(reverse=True)
        top = '; '.join(f'{topic} {hz:.1f}Hz {ms:.2f}ms/s' for hz, ms, topic in rows[:self.top_n])

        logger.info(
            f'[PERF] cpu={cpu_pct:.1f}%单核 rss={_proc_rss_mb():.0f}MB 线程={_thread_count()} '
            f'| 实体 sub={entities["sub"]} waitable={entities["waitable"]} '
            f'timer={entities["timer"]} client={entities["client"]} '
            f'srv={entities["srv"]} guard={entities["guard"]} 重建集={entities["total"]} '
            f'| 消息={total_count / window:.1f}/s 回调={total_ms / window:.2f}ms/s '
            f'| Top: {top or "无"}{self._stream_lines()}'
        )
        self._prev_at = now

    def start(self):
        """启动周期性摘要；install() 需在此之前单独调用（要早于任何订阅创建）"""
        if self._timer is not None:
            return
        self._timer = self.node.create_timer(self.interval, self.report)
