#!/usr/bin/env python3
# Copyright 2026 uav_planning contributors
# SPDX-License-Identifier: Apache-2.0

"""
Record the ground reference for the current X500 demonstration.

先启动 takeoff_hold.launch.py start_controller:=false。
本脚本只订阅消息：对地面静止状态采样2仿真秒，打印地图参考位置和本次 NED 对应值。
不解锁、不发送目标、不自动写 YAML；后续飞行节点仍使用 PX4 估计位置闭环控制。
"""

import math
import time

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data


SAMPLE_SECONDS = 2.0
MAX_SPEED = 0.05  # m/s；只为判断标定时是否静止，不是飞行控制参数。
MAX_SPREAD = 0.05  # m；采样窗口内任一坐标的最大变化。
WAIT_SECONDS = 90.0
BASE_LINK_OFFSET_Z = 0.24  # m；当前 x500_base 合并后，base_link 相对模型原点的高度。


class ReferenceRecorder(Node):
    """Collect corresponding map and PX4 positions without controlling the UAV."""

    def __init__(self):
        super().__init__(
            'record_reference',
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.position = None
        self.status = None
        self.samples = []
        self.reset_counts = None
        self.done = False
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.on_position, qos_profile_sensor_data)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.on_status, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/simulation/ground_truth/odometry',
            self.on_truth, qos_profile_sensor_data)
        self.get_logger().info('等待未解锁的 X500、有效定位和地面静止数据；不会发送飞行命令。')

    def on_position(self, message):
        """Keep the latest fused NED position."""
        self.position = message

    def on_status(self, message):
        """Keep the latest arming state."""
        self.status = message

    def on_truth(self, message):
        """Sample map and NED positions from the same simulation time window."""
        position, status = self.position, self.status
        if self.done or position is None or status is None:
            return
        if status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            raise RuntimeError('参考点只能在未解锁时记录，请先结束飞行。')
        if message.header.frame_id != 'map' or message.child_frame_id != 'x500_0/base_link':
            raise RuntimeError('真值坐标帧不是当前演示的 map / x500_0/base_link。')
        if not (position.xy_valid and position.z_valid and
                position.v_xy_valid and position.v_z_valid):
            return

        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        # launch 已统一仿真时间；不把过期 DDS 状态与新的真值配成参考点。
        if not (-0.1 <= stamp - position.timestamp_sample * 1e-6 <= 0.2 and
                -0.1 <= stamp - status.timestamp * 1e-6 <= 2.0):
            return
        point = message.pose.pose.position
        orientation = message.pose.pose.orientation
        linear = message.twist.twist.linear
        # 当前 OdometryPublisher 输出模型原点，child_frame_id 标签并不会移动测量点。
        # 将模型坐标下的 (0,0,0.24) 按姿态旋转，再平移到世界，得到 base_link 真值。
        x, y, z, w = (orientation.x, orientation.y, orientation.z, orientation.w)
        base_x = point.x + BASE_LINK_OFFSET_Z * 2.0 * (x * z + w * y)
        base_y = point.y + BASE_LINK_OFFSET_Z * 2.0 * (y * z - w * x)
        base_z = point.z + BASE_LINK_OFFSET_Z * (1.0 - 2.0 * (x * x + y * y))
        values = (base_x, base_y, base_z, position.x, position.y, position.z,
                  linear.x, linear.y, linear.z, position.vx, position.vy, position.vz)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError('参考数据包含非有限值，不能记录标定结果。')
        if max(math.hypot(linear.x, linear.y, linear.z),
               math.hypot(position.vx, position.vy, position.vz)) > MAX_SPEED:
            if self.samples:
                raise RuntimeError('采样期间飞机发生移动，请重新在地面静止时运行。')
            return
        # 当前固定起飞区；低速但悬停在空中也不能当成地面标定。
        if math.hypot(base_x + 24.0, base_y + 18.0) > 0.1 or not 0.0 < base_z < 0.5:
            raise RuntimeError('飞机不在当前固定地面起飞区，请重启原始场景。')
        counts = (position.xy_reset_counter, position.z_reset_counter)
        if self.reset_counts is not None and counts != self.reset_counts:
            raise RuntimeError('采样期间 PX4 位置发生重置，请定位稳定后重新记录。')
        self.reset_counts = counts
        self.samples.append((stamp, *values[:6]))
        self.done = stamp - self.samples[0][0] >= SAMPLE_SECONDS

    def report(self):
        """Print a YAML reference and the temporary NED pair for inspection."""
        columns = list(zip(*(sample[1:] for sample in self.samples)))
        spread = [max(column) - min(column) for column in columns]
        if max(spread) > MAX_SPREAD:
            raise RuntimeError('采样窗口内位置变化超过5厘米，不能把它当作静止参考。')
        means = [sum(column) / len(column) for column in columns]
        print('\n采样数：{}；时长：{:.2f} 仿真秒'.format(
            len(self.samples), self.samples[-1][0] - self.samples[0][0]))
        print('map 参考位置 [东, 北, 上] (m)：', [round(x, 6) for x in means[:3]])
        print('参考点为 base_link，已补偿模型原点到机体原点的 0.24 m 偏移。')
        print('本次 NED 对应值 [北, 东, 下] (m)：', [round(x, 6) for x in means[3:]])
        print('map / NED 各轴采样范围 (m)：', [round(x, 6) for x in spread])
        print('\n核对后可写入 goto_point.yaml 的地图参考值：')
        print('    map_reference_position: [{:.6f}, {:.6f}, {:.6f}]'.format(*means[:3]))
        print('NED 对应值只用于核对；下次启动重新读取，不能作为固定配置保存。')
        print('本次仅验证地面参考采样，未验证单轴飞行或坐标转换模块。')


def main():
    """Observe a running simulation and release ROS resources on exit."""
    rclpy.init()
    node = ReferenceRecorder()
    deadline = time.monotonic() + WAIT_SECONDS
    try:
        while rclpy.ok() and not node.done:
            if time.monotonic() >= deadline:
                raise RuntimeError('90秒内未取得有效参考，请确认仿真运行且未启动控制节点。')
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.done:
            node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
