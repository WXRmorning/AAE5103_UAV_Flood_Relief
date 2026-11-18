#!/usr/bin/env python3
# Copyright 2026 uav_planning contributors
# SPDX-License-Identifier: Apache-2.0

"""
Validate the installed PX4/Harmonic takeoff launch with real flight feedback.

使用独立 ROS domain、Gazebo partition、DDS 端口和临时 SITL 参数目录。
观测 PX4 状态、估计位置及 Gazebo 真值，验证起飞、保持和降落后不自动重启。
仅在明确运行本脚本时启动仿真，不随 colcon test 自动执行。
"""

import argparse
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def versioned(name, message_type):
    """Use the PX4 v1.16 DDS topic-version convention."""
    version = message_type.MESSAGE_VERSION
    return name if version == 0 else name + '_v' + str(version)


def main():
    """Exercise the installed launch using an independent feedback observer."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--altitude', type=float, default=2.0)
    parser.add_argument('--hold-seconds', type=float, default=10.0)
    parser.add_argument('--extended', action='store_true', help='Also test pause/resume')
    parser.add_argument('--px4-dir', default='/home/ruan/PX4-Autopilot')
    args = parser.parse_args()
    if not (0.5 <= args.altitude <= 10.0 and args.hold_seconds > 0):
        parser.error('altitude must be [0.5,10], hold-seconds > 0')

    os.environ['GZ_PARTITION'] = 'uav_px4_validation_' + str(os.getpid())
    os.environ['ROS_DOMAIN_ID'] = str(190 + os.getpid() % 20)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    # 参数与日志留在临时目录，便于失败排查，不更改用户原 SITL 参数。
    runtime = tempfile.mkdtemp(prefix='uav_px4_validation_')
    print('SITL runtime: ' + runtime, flush=True)
    executable = Path(get_package_prefix('uav_planning')) / 'lib/uav_planning/takeoff_hold_node'
    invalid = subprocess.run(
        [str(executable), '--ros-args', '-p', 'control_rate:=0.0'],
        capture_output=True, text=True, timeout=10)
    assert invalid.returncode != 0 and 'control_rate' in invalid.stderr
    print('PASS: invalid control rate rejected before flight', flush=True)

    rclpy.init()
    observer = Node('px4_flight_validation_' + str(os.getpid()))
    state = {'clock': 0.0, 'position': None, 'status': None, 'truth': None,
             'target': None, 'ground_z': None, 'stable_since': None}
    beats = []
    samples = []
    commands = []

    def on_clock(message):
        state['clock'] = message.clock.sec + message.clock.nanosec * 1e-9

    def on_position(message):
        state['position'] = message

    def on_status(message):
        state['status'] = message

    def on_truth(message):
        state['truth'] = message
        samples.append((state['clock'], message))

    def on_target(message):
        if state['target'] is None:
            state['target'] = tuple(message.position)
            if state['truth'] is not None:
                state['ground_z'] = state['truth'].pose.pose.position.z
        assert all(math.isnan(value) for value in message.velocity)
        assert all(math.isnan(value) for value in message.acceleration)

    subscriptions = [
        observer.create_subscription(Clock, '/clock', on_clock, 10),
        observer.create_subscription(
            VehicleLocalPosition, versioned('/fmu/out/vehicle_local_position',
                                            VehicleLocalPosition),
            on_position, qos_profile_sensor_data),
        observer.create_subscription(
            VehicleStatus, versioned('/fmu/out/vehicle_status', VehicleStatus),
            on_status, qos_profile_sensor_data),
        observer.create_subscription(
            Odometry, '/simulation/ground_truth/odometry', on_truth, qos_profile_sensor_data),
        observer.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', on_target, 100),
        observer.create_subscription(
            OffboardControlMode, '/fmu/in/offboard_control_mode',
            lambda message: beats.append(message.timestamp * 1e-6), 100),
        observer.create_subscription(
            VehicleCommand, '/fmu/in/vehicle_command', commands.append, 100),
    ]
    land_pub = observer.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
    log = []
    process = subprocess.Popen(
        ['ros2', 'launch', 'uav_planning', 'takeoff_hold.launch.py', 'gui:=false',
         'target_altitude:=' + str(args.altitude), 'px4_dir:=' + args.px4_dir,
         'dds_port:=' + str(port), 'runtime_dir:=' + runtime],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)

    def drain():
        for line in process.stdout:
            log.append(line.rstrip())
    threading.Thread(target=drain, daemon=True).start()

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.03)

    def wait_until(condition, timeout, label):
        deadline = time.monotonic() + timeout
        report = time.monotonic() + 10
        while not condition():
            if process.poll() is not None:
                raise AssertionError('Launch exited: ' + label)
            if time.monotonic() >= deadline:
                raise AssertionError('Timed out: ' + label)
            rclpy.spin_once(observer, timeout_sec=0.03)
            if time.monotonic() > report:
                status = state['status']
                position = state['position']
                print('{}: sim={:.1f}, nav={}, arm={}, z={}'.format(
                    label, state['clock'], status.nav_state if status else None,
                    status.arming_state if status else None,
                    round(position.z, 2) if position else None), flush=True)
                report = time.monotonic() + 10

    def arrived():
        position, status, target = state['position'], state['status'], state['target']
        good = (position is not None and status is not None and target is not None
                and position.xy_valid and position.z_valid
                and status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and math.hypot(position.x - target[0], position.y - target[1]) < 0.35
                and abs(position.z - target[2]) < 0.25
                and math.hypot(position.vx, position.vy, position.vz) < 0.25)
        if not good:
            state['stable_since'] = None
        elif state['stable_since'] is None:
            state['stable_since'] = state['clock']
        return (good and state['clock'] - state['stable_since'] >= 2.0)

    def gazebo_pause(paused):
        response = subprocess.run(
            ['gz', 'service', '-s', '/world/flood_demo/control',
             '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', 'pause: ' + str(paused).lower()],
            capture_output=True, text=True, timeout=6)
        assert response.returncode == 0 and 'data: true' in response.stdout

    try:
        wait_until(arrived, 150, 'PX4 takeoff and stable position')
        assert state['ground_z'] is not None
        print('PASS: PX4 confirmed armed + Offboard; relative ascent {:.1f} m'.format(
            args.altitude), flush=True)
        start = state['clock']
        first = len(samples)
        wait_until(lambda: state['clock'] - start >= args.hold_seconds,
                   args.hold_seconds * 4 + 20, 'position hold')
        held = [message.pose.pose.position for _, message in samples[first:]]
        assert held
        max_xy = max(math.hypot(point.x + 24, point.y + 18) for point in held)
        target_z = state['ground_z'] + args.altitude
        max_z = max(abs(point.z - target_z) for point in held)
        assert max_xy < 0.5 and max_z < 0.4, (max_xy, max_z)
        assert len(beats) > 20
        assert '/takeoff_hold_node/state' not in dict(observer.get_topic_names_and_types())
        print('PASS: {:.1f} sim seconds hold; truth max XY={:.3f} m, Z={:.3f} m'.format(
            args.hold_seconds, max_xy, max_z), flush=True)
        assert not any('time jump detected' in line for line in log)
        assert not any('libGstCameraSystem.so' in line for line in log)
        print('PASS: no DDS time-jump or missing camera-stream plugin errors', flush=True)

        if args.extended:
            gazebo_pause(True)
            pump(0.5)
            paused_clock, paused_beats = state['clock'], len(beats)
            pump(1.5)
            assert abs(state['clock'] - paused_clock) < 0.02
            assert len(beats) == paused_beats
            gazebo_pause(False)
            wait_until(arrived, 30, 'pause/resume')
            print('PASS: pause/resume uses the common simulation clock', flush=True)

        land = VehicleCommand()
        land.timestamp = int(state['clock'] * 1e6)
        land.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        land.target_system = 1
        land.target_component = 1
        land.source_system = 1
        land.source_component = 10
        land.from_external = True
        land.param5 = float('nan')
        land.param6 = float('nan')
        land.param7 = float('nan')
        land_pub.publish(land)
        wait_until(lambda: state['status'].nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                   10, 'land mode transition')
        wait_until(lambda: state['status'].arming_state == VehicleStatus.ARMING_STATE_DISARMED,
                   90, 'landing and disarm')
        pump(1.0)
        count = len(beats)
        pump(1.0)
        assert len(beats) == count, 'Controller continued publishing after external landing'
        assert state['status'].arming_state == VehicleStatus.ARMING_STATE_DISARMED
        print('PASS: land/disarm; controller yields and never automatically rearms', flush=True)
        print('ALL CHECKS PASSED (headless; GUI not tested)', flush=True)
    except Exception:
        print('\nRecent launch output:\n' + '\n'.join(log[-55:]), flush=True)
        raise
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for subscription in subscriptions:
            observer.destroy_subscription(subscription)
        observer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
