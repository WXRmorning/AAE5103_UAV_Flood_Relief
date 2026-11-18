# Copyright 2026 uav_planning contributors
# SPDX-License-Identifier: Apache-2.0

"""
Launch Harmonic, PX4 X500 SITL, DDS Agent, clock bridge and takeoff/hold.

中文阅读顺序：检查路径 -> 设置本次进程环境 -> 创建各模块 -> 注册退出清理。
Gazebo 服务端与 PX4 分开启动；PX4 连接地图中已存在的 x500_0。
所有环境和 SITL 参数仅作用于本次启动，不改用户 .bashrc 或 PX4 源码。
"""

import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _shutdown_on_exit(label):
    def handle_exit(event, context):
        if context.is_shutdown:
            return []
        reason = '{} exited with code {}'.format(label, event.returncode)
        return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
    return handle_exit


def _path(context, argument, directory=False):
    path = Path(LaunchConfiguration(argument).perform(context)).expanduser().resolve()
    exists = path.is_dir() if directory else path.is_file()
    if not exists:
        raise RuntimeError('{} 不存在：{}'.format(argument, path))
    return path


def _launch_setup(context):
    share = Path(get_package_share_directory('uav_planning'))
    px4 = _path(context, 'px4_dir', directory=True)
    build = px4 / 'build' / 'px4_sitl_default'
    binary = build / 'bin' / 'px4'
    if not binary.is_file() or not (build / 'etc' / 'init.d-posix' / 'rcS').is_file():
        raise RuntimeError('请先在 {} 执行 make px4_sitl。'.format(px4))
    for tool in ('gz', 'MicroXRCEAgent'):
        if shutil.which(tool) is None:
            raise RuntimeError('找不到 {}，请先安装对应组件。'.format(tool))
    # 单机任务固定使用 instance 0，避免误接入另一场已经运行的 PX4。
    if subprocess.run(['pgrep', '-x', 'px4'], capture_output=True).returncode == 0:
        raise RuntimeError('已有 PX4 进程，请先结束原测试，再启动本任务。')

    port = int(LaunchConfiguration('dds_port').perform(context))
    if not 1024 <= port <= 65535:
        raise RuntimeError('dds_port 必须在 [1024,65535]。')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(('127.0.0.1', port))
        except OSError as error:
            raise RuntimeError('DDS UDP 端口 {} 已占用，请先关闭原 Agent。'.format(port)) from error

    world = _path(context, 'world')
    params = _path(context, 'params_file')
    bridge_config = _path(context, 'bridge_config')
    world_element = ET.parse(world).getroot().find('world')
    if world_element is None or not any(
            item.findtext('name') == 'x500_0' for item in world_element.findall('include')):
        raise RuntimeError('地图必须包含名为 x500_0 的官方 X500 include。')
    world_name = world_element.attrib['name']

    runtime = Path(LaunchConfiguration('runtime_dir').perform(context)).expanduser().resolve()
    runtime.mkdir(parents=True, exist_ok=True)
    models = px4 / 'Tools' / 'simulation' / 'gz' / 'models'
    environment = {
        'GZ_IP': '127.0.0.1',
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join(
            [str(models), str(world.parent), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]),
        'GZ_SIM_SERVER_CONFIG_PATH': str(share / 'config' / 'gazebo_server.config'),
    }
    # 用于启动软件在环仿真，而不是烧录实体飞控。参数文件保存在 runtime 目录。
    px4_environment = dict(environment, **{
        'PX4_SIM_MODEL': 'gz_x500',
        'PX4_SYS_AUTOSTART': '4001',
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_WORLD': world_name,
        'PX4_GZ_MODEL_NAME': 'x500_0',
        'PX4_GZ_MODELS': str(models),
        # 三者配套：PX4 不做系统时钟偏移补偿、ROS 用仿真时间、桥接 /clock。
        'PX4_PARAM_UXRCE_DDS_SYNCT': '0',
        'PX4_PARAM_UXRCE_DDS_DOM_ID': os.environ.get('ROS_DOMAIN_ID', '0'),
        'PX4_PARAM_UXRCE_DDS_PTCFG': (
            '1' if os.environ.get('ROS_LOCALHOST_ONLY') == '1' else '0'),
        'PX4_PARAM_UXRCE_DDS_PRT': str(port),
        'PX4_UXRCE_DDS_PORT': str(port),
        # 本地无地面站/遥控器仿真。Offboard 失联仍由 PX4 自动降落。
        'PX4_PARAM_NAV_DLL_ACT': '0',
        'PX4_PARAM_COM_RC_IN_MODE': '4',
        'PX4_PARAM_COM_OBL_RC_ACT': '4',
        'PX4_PARAM_COM_OF_LOSS_T': '1.0',
    })
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--render-engine', 'ogre', str(world)],
        additional_env=environment, output='screen')
    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', str(port)], output='screen')
    # -d 关闭交互 shell，但仍是由 launch 管理的前台进程；-w 隔离本项目参数与日志。
    autopilot = ExecuteProcess(
        cmd=[str(binary), '-d', '-w', str(runtime),
             '-s', str(share / 'config' / 'px4_startup.sh'), str(build / 'etc')],
        cwd=str(runtime), additional_env=px4_environment, output='screen')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='uav_bridge',
        parameters=[{'config_file': str(bridge_config)}],
        additional_env=environment, output='screen')
    processes = [
        (gazebo, 'Gazebo server'), (agent, 'DDS Agent'),
        (autopilot, 'PX4 SITL'), (bridge, 'Gazebo bridge'),
    ]

    if LaunchConfiguration('start_controller').perform(context) == 'true':
        parameters = [str(params), {'use_sim_time': True}]
        altitude_text = LaunchConfiguration('target_altitude').perform(context).strip()
        if altitude_text:
            altitude = float(altitude_text)
            if not math.isfinite(altitude) or not 0.5 <= altitude <= 30.0:
                raise RuntimeError('target_altitude 必须在 [0.5,30] 米。')
            parameters.append({'target_altitude': altitude})
        controller = Node(
            package='uav_planning', executable='takeoff_hold_node', name='takeoff_hold_node',
            parameters=parameters, output='screen')
        processes.append((controller, 'Offboard controller'))

    actions = [LogInfo(msg='Harmonic + PX4 X500: world={}, runtime={}'.format(
        world_name, runtime))]
    for process, label in processes:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=process, on_exit=_shutdown_on_exit(label))))
    actions.extend(process for process, _ in processes)

    if LaunchConfiguration('gui').perform(context) == 'true':
        # 界面关闭不影响服务端；所有子进程仍由本次 launch 统一清理。
        actions.append(ExecuteProcess(
            cmd=['gz', 'sim', '-g', '--render-engine', 'ogre'],
            additional_env=environment, output='screen'))
    return actions


def generate_launch_description():
    """Declare arguments and construct the launch description."""
    share = Path(get_package_share_directory('uav_planning'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                              description='Show the Ogre1 Gazebo GUI.'),
        DeclareLaunchArgument('start_controller', default_value='true',
                              choices=['true', 'false'],
                              description='Automatically start the takeoff/hold node.'),
        DeclareLaunchArgument('target_altitude', default_value='',
                              description='Relative ascent in metres; empty uses YAML.'),
        DeclareLaunchArgument('px4_dir', default_value=str(Path.home() / 'PX4-Autopilot'),
                              description='Built PX4 v1.16 repository.'),
        DeclareLaunchArgument('runtime_dir', default_value=str(
            Path.home() / 'uav_ws' / 'run' / 'px4_sitl'),
            description='Writable SITL parameters and logs directory.'),
        DeclareLaunchArgument('dds_port', default_value='8888',
                              description='UDP port for this DDS Agent and PX4 client.'),
        DeclareLaunchArgument('world', default_value=str(share / 'worlds' / 'flood_demo.sdf'),
                              description='Harmonic world containing x500_0.'),
        DeclareLaunchArgument('params_file', default_value=str(
            share / 'config' / 'takeoff_hold.yaml')),
        DeclareLaunchArgument('bridge_config', default_value=str(
            share / 'config' / 'bridge.yaml')),
        OpaqueFunction(function=_launch_setup),
    ])
