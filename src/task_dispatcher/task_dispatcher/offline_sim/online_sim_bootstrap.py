#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线节点 + 仿真机的引导入口：只做“路径/IO 边界”覆盖，业务代码原样。

背景：config.py 写的是机载路径（/home/cat/slam_data/...），在线节点构造
RtspRecordManager 时无条件 makedirs，x86 仿真机没有该目录会直接崩；相机/云台
URL 也是机载内网地址。本入口与离线套件同一注入套路：
    sim_support.install_sim_config(SIM_CONFIG_OVERRIDES)
把录像/点云路径改到 /tmp、相机与云台置空，之后原样调用在线 main()——
task_dispatcher 包代码零改动，除边界路径外所有业务开关仍是 config.py 原值。

用法（通常由 launch/online_sim_control.launch.py 在 sim_paths:=true 时调用）：
    python3 online_sim_bootstrap.py --ros-args -p robot_sn:=UAV_TEST_01 ...
"""

import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import sim_support  # noqa: E402

sim_support.bootstrap_sys_path()


def _ensure_ffmpeg_on_path() -> str:
    """让在线代码（探测/预览推流都写死调用 `ffmpeg`）用上可用的 ffmpeg。

    仿真机常见两种情况：PATH 里根本没有 ffmpeg（依赖只在 venv 的 imageio-ffmpeg 里），
    或 PATH 里是精简构建（缺 rtsp muxer/编码器）。与 sim_rtsp_camera_bridge 同一优先级
    选一个，再在 /tmp 做 `ffmpeg` 软链并前置 PATH，在线代码零改动。
    """
    candidates = []
    try:
        import imageio_ffmpeg
        candidates.append(imageio_ffmpeg.get_ffmpeg_exe())
    except Exception:
        pass
    candidates.append(shutil.which('ffmpeg') or '')
    exe = next((c for c in candidates if c and os.path.isfile(c) and os.access(c, os.X_OK)), '')
    if not exe:
        print('[online_sim_bootstrap] 警告：找不到可用的 ffmpeg，相机在线探测与平台推流会失败', flush=True)
        return ''

    shim_dir = '/tmp/uav_online_sim/bin'
    os.makedirs(shim_dir, exist_ok=True)
    link = os.path.join(shim_dir, 'ffmpeg')
    try:
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(exe, link)
    except OSError as exc:
        print(f'[online_sim_bootstrap] 警告：创建 ffmpeg 软链失败({exc})，沿用 PATH', flush=True)
        return exe
    os.environ['PATH'] = shim_dir + os.pathsep + os.environ.get('PATH', '')
    print(f'[online_sim_bootstrap] 在线视频链路使用 ffmpeg: {exe}', flush=True)
    return exe


# 视频源注入：online_sim 套件会拉起 MediaMTX + sim_rtsp_camera_bridge（gz 相机推流），
# 由 launch 通过环境变量把本机 RTSP 地址传给在线配置覆盖；未设置则维持无相机默认。
overrides = dict(sim_support.SIM_CONFIG_OVERRIDES)
cam_url = os.environ.get('SIM_CAMERA01_URL', '').strip()
if cam_url:
    overrides['CAMERA01_URL'] = cam_url
    overrides['ENABLE_VIDEO'] = os.environ.get('SIM_ENABLE_VIDEO', 'true').lower() == 'true'
    # 仿真相机桥输出的已是 H.264；真机配置的 h265 分支用 Rockchip 硬件编码器
    # (h264_rkmpp)，x86 仿真机上不存在，会让平台推流即时失败（节点仍回 200）。
    # 这里改走 h264 透传（-c:v copy）。
    overrides['CODEC'] = os.environ.get('SIM_CODEC', 'h264')
    _ensure_ffmpeg_on_path()
sim_support.install_sim_config(overrides)

# 相机在线探测超时适配：在线实现默认 timeout_sec=2（subprocess 硬超时 3s），而仿真
# RTSP 首帧要等 GOP 关键帧，实测整条探测命令约 4.2s，会在 3s 被杀并误判离线，
# 平台端一直显示“设备不在线”。仅本入口放宽到 8s，在线代码不改。
from task_dispatcher import video_preview_manager as _video_preview_module  # noqa: E402

_orig_is_rtsp_online = _video_preview_module.VideoPreviewManager.is_rtsp_online


def _is_rtsp_online(self, rtsp_url, timeout_sec=8):
    return _orig_is_rtsp_online(self, rtsp_url, timeout_sec)


_video_preview_module.VideoPreviewManager.is_rtsp_online = _is_rtsp_online

# 旁路 systemd 边界：在线 _execute_task_thread 在 cruise 任务前会
# `sudo systemctl start ego_planner.service`（见 cruise_uav_task_manager L1198），
# 仿真机无此 unit——sudo 挂起直至 timeout=120 抛 TimeoutExpired，被外层 except
# 捕获后整个任务直接 ABORTED，永远走不到起飞指令。与离线套件同一桩：
# 仿真环境的 ego_planner 由 demo launch 常驻拉起，无需 systemd 管理。
import task_dispatcher.cruise_uav_task_manager as cruise_module  # noqa: E402

cruise_module.subprocess = sim_support.SimSubprocess()

from task_dispatcher.task_dispatcher_node import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main() or 0)
