#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在线节点 + 仿真环境的位姿桥：Gazebo odom 真值 -> /pcl_pose。

在线 task_dispatcher 的“航向来源”是激光定位节点发布的 /pcl_pose
（Ros2TopicSubscriber 以 amcl_qos 订阅：RELIABLE + TRANSIENT_LOCAL + depth=1），
仿真里没有这个节点。为不改任何在线代码，这里补一个转换节点：
    <namespace>/odometry (nav_msgs/Odometry, world->base_footprint 真值)
      -> /pcl_pose (geometry_msgs/PoseWithCovarianceStamped, frame_id=map)

消息组装与 QoS 全部复用 sim_support（与离线节点直喂回调的换算完全同源），
保证“离线测试验证过的到达判定/yaw/停滞 watchdog”与在线仿真跑的是同一份逻辑。

用法（通常由 launch/online_sim_control.launch.py 拉起）：
    python3 sim_pcl_pose_bridge.py --odom-topic /x500_depth_0/odometry --use-sim-time true
"""

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import sim_support  # noqa: E402  （话题/QoS/换算的单一事实来源）

logger = logging.getLogger('sim_pcl_pose_bridge')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Gazebo odom 真值 -> /pcl_pose 位姿桥（在线仿真用）')
    parser.add_argument('--odom-topic', default='/x500_depth_0/odometry',
                        help='Gazebo 真值里程计话题（world -> base_footprint）')
    parser.add_argument('--pose-topic', default='/pcl_pose',
                        help='在线链路的位置话题名（绝对名，与 Ros2TopicSubscriber 订阅一致）')
    parser.add_argument('--frame-id', default='map',
                        help='位姿 header.frame_id（与在线激光定位一致用 map）')
    parser.add_argument('--use-sim-time', default='true', choices=['true', 'false'],
                        help='是否跟随 Gazebo 仿真时钟（仿真必须 true）')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format='[%(asctime)s] [%(name)s] [%(levelname)s] [%(filename)s:%(lineno)d] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
    )

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry

    rclpy.init()
    use_sim_time = args.use_sim_time.lower() == 'true'
    node = Node('sim_pcl_pose_bridge',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])

    # QoS 与在线订阅端成对（sim_support.pcl_pose_qos 是单一出处，勿在此另写一份）
    pub = node.create_publisher(PoseWithCovarianceStamped, args.pose_topic, sim_support.pcl_pose_qos())
    count = 0
    last_log = 0.0

    def on_odom(msg):
        nonlocal count, last_log
        pub.publish(sim_support.wrap_odom_to_pcl_pose(msg, args.frame_id))
        count += 1
        # 每分钟输出一次累计帧数，避免频繁刷屏
        now = time.time()
        if count == 1 or now - last_log >= 60:
            p = msg.pose.pose.position
            logger.info('已发布 %d 帧 %s: x=%.2f y=%.2f z=%.2f', count, args.pose_topic, p.x, p.y, p.z)
            last_log = now

    node.create_subscription(Odometry, args.odom_topic, on_odom, 10)
    logger.info('位姿桥: %s -> %s (frame=%s, use_sim_time=%s)',
                args.odom_topic, args.pose_topic, args.frame_id, use_sim_time)
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
    sys.exit(main())
