#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真命令桥：/uav_command(String) -> multirotor_communication 的 XTD2Cmd 服务。

在线链路里 `CruiseUavTaskManager._emergency_land()`（返航受阻->迫降分支）会向 /uav_command
下发 'LAND'，真机上由 uav_service 的 UavMissionManager 消费。仿真里不能直接跑那个节点：
它会以 10Hz 持续发 offboard 心跳与设定点，和 ego_planner 抢控制权。
本桥把命令转成 multirotor_communication 自带的 XTD2Cmd 服务（LAND/RTL/HOVER/TAKEOFF/...），
由通信节点自己走 PX4 原生模式切换，规划侧会检测到 nav_state=18 并停止 offboard 输出，
两者不冲突，因此可在仿真里验证“迫降”分支。

用法：
    python3 sim_uav_command_bridge.py --cmd-service /xtdrone2/x500_depth_0/cmd
"""

import argparse
import logging

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from xtd2_msgs.srv import XTD2Cmd

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S',
)
logger = logging.getLogger('sim_uav_command_bridge')

# /uav_command 里在线会用的命令 -> XTD2Cmd 服务命令
KNOWN_COMMANDS = {'LAND', 'RTL', 'HOVER', 'TAKEOFF', 'ARM', 'DISARM', 'OFFBOARD'}


class SimUavCommandBridge(Node):
    def __init__(self, command_topic: str, cmd_service: str, use_sim_time: bool):
        super().__init__('sim_uav_command_bridge', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)
        ])
        self._service_name = cmd_service
        self._client = self.create_client(XTD2Cmd, cmd_service)
        self._sub = self.create_subscription(String, command_topic, self._on_command, 10)
        self.get_logger().info(
            f'命令桥启动: {command_topic} -> {cmd_service} (可用命令: {sorted(KNOWN_COMMANDS)})')

    def _on_command(self, msg: String) -> None:
        command = (msg.data or '').strip().upper()
        if command not in KNOWN_COMMANDS:
            # 例如在线的 'GPIO:1' 之类，仿真无对应设备，忽略即可
            self.get_logger().warn(f'忽略非飞行命令: {msg.data}')
            return

        if not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f'服务不可用: {self._service_name}，命令 {command} 未执行')
            return

        req = XTD2Cmd.Request()
        req.command = command
        future = self._client.call_async(req)
        future.add_done_callback(lambda f, c=command: self._on_result(f, c))
        self.get_logger().info(f'已下发命令: {command}')

    def _on_result(self, future, command: str) -> None:
        try:
            resp = future.result()
            self.get_logger().info(f'命令 {command} 结果: success={resp.success}')
        except Exception as exc:
            self.get_logger().error(f'命令 {command} 调用失败: {exc}')


def main() -> int:
    parser = argparse.ArgumentParser(description='仿真 /uav_command -> XTD2Cmd 命令桥')
    parser.add_argument('--command-topic', default='/uav_command')
    parser.add_argument('--cmd-service', default='/xtdrone2/x500_depth_0/cmd',
                        help='multirotor_communication 的 XTD2Cmd 服务名（由载具命名空间决定）')
    parser.add_argument('--use-sim-time', default='true', choices=['true', 'false'])
    args = parser.parse_args()

    rclpy.init()
    node = SimUavCommandBridge(args.command_topic, args.cmd_service,
                               args.use_sim_time.lower() == 'true')
    node._service_name = args.cmd_service
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
