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
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import sim_support  # noqa: E402

sim_support.bootstrap_sys_path()
sim_support.install_sim_config()

from task_dispatcher.task_dispatcher_node import main  # noqa: E402

if __name__ == '__main__':
    sys.exit(main() or 0)
