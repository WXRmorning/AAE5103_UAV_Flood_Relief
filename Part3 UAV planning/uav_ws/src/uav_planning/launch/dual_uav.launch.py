# Copyright 2026 uav_planning contributors
# SPDX-License-Identifier: Apache-2.0

"""
Launch two PX4 X500s with separate planners and controllers.

在同一洪水场景启动两架飞机，各自接收任务、规划和执行。
"""

import os
from pathlib import Path
import shutil
import socket
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _shutdown_on_exit(label):
    def handle_exit(event, context):
        if context.is_shutdown:
            return []
        reason = f'{label} exited with code {event.returncode}'
        return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
    return handle_exit


def _launch_setup(context):
    share = Path(get_package_share_directory('uav_planning'))
    px4 = Path(LaunchConfiguration('px4_dir').perform(context)).expanduser().resolve()
    build = px4 / 'build' / 'px4_sitl_default'
    binary = build / 'bin' / 'px4'
    if not binary.is_file() or not (build / 'etc' / 'init.d-posix' / 'rcS').is_file():
        raise RuntimeError(f'请先在 {px4} 执行 make px4_sitl。')
    for tool in ('gz', 'MicroXRCEAgent'):
        if shutil.which(tool) is None:
            raise RuntimeError(f'找不到 {tool}，请先安装对应组件。')
    if subprocess.run(['pgrep', '-x', 'px4'], capture_output=True).returncode == 0:
        raise RuntimeError('已有 PX4 进程，请先结束原测试，再启动本任务。')

    port = int(LaunchConfiguration('dds_port').perform(context))
    if not 1024 <= port <= 65535:
        raise RuntimeError('dds_port 必须在 [1024,65535]。')
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        try:
            probe.bind(('127.0.0.1', port))
        except OSError as error:
            raise RuntimeError(f'DDS UDP 端口 {port} 已占用，请先关闭原 Agent。') from error

    world = Path(LaunchConfiguration('world').perform(context)).expanduser().resolve()
    world_element = ET.parse(world).getroot().find('world')
    models_in_world = {item.findtext('name') for item in world_element.findall('include')} if (
        world_element is not None) else set()
    if not {'x500_0', 'x500_1'} <= models_in_world:
        raise RuntimeError('地图必须包含名为 x500_0 和 x500_1 的官方 X500 include。')
    world_name = world_element.attrib['name']
    runtime = Path(LaunchConfiguration('runtime_dir').perform(context)).expanduser().resolve()

    models = px4 / 'Tools' / 'simulation' / 'gz' / 'models'
    environment = {
        'GZ_IP': '127.0.0.1',
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join(
            [str(models), str(world.parent), os.environ.get('GZ_SIM_RESOURCE_PATH', '')]),
        'GZ_SIM_SERVER_CONFIG_PATH': str(share / 'config' / 'gazebo_server.config'),
    }
    px4_environment = dict(environment, **{
        'PX4_SIM_MODEL': 'gz_x500',
        'PX4_SYS_AUTOSTART': '4001',
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_WORLD': world_name,
        'PX4_GZ_MODELS': str(models),
        'PX4_PARAM_UXRCE_DDS_SYNCT': '0',
        'PX4_PARAM_UXRCE_DDS_DOM_ID': os.environ.get('ROS_DOMAIN_ID', '0'),
        'PX4_PARAM_UXRCE_DDS_PTCFG': (
            '1' if os.environ.get('ROS_LOCALHOST_ONLY') == '1' else '0'),
        'PX4_UXRCE_DDS_PORT': str(port),
        # 仿真不需要地面站或遥控器，Offboard 心跳丢失时仍降落。
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
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='dual_uav_bridge',
        parameters=[{'config_file': str(share / 'config' / 'bridge_dual.yaml')}],
        additional_env=environment, output='screen')
    processes = [(gazebo, 'Gazebo server'), (agent, 'DDS Agent'), (bridge, 'Gazebo bridge')]

    dispatcher = Node(
        package='uav_planning', executable='mission_dispatcher_node',
        name='mission_dispatcher_node',
        parameters=[str(share / 'config' / 'mission_points.yaml'), {'use_sim_time': True}],
        output='screen')
    processes.append((dispatcher, 'mission dispatcher'))

    return_hover = LaunchConfiguration('return_hover').perform(context) == 'true'
    controller_executable = 'goto_point_hover_node' if return_hover else 'goto_point_node'
    params_file = str(share / 'config' / 'goto_point.yaml')
    fmu_topics = (
        '/fmu/in/offboard_control_mode',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/vehicle_command',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_status_v1',
        '/fmu/out/vehicle_land_detected',
        '/fmu/out/vehicle_command_ack',
    )
    for instance, namespace in enumerate(('uav1', 'uav2')):
        vehicle_runtime = runtime / namespace
        vehicle_runtime.mkdir(parents=True, exist_ok=True)
        # PX4 用实例号 +1 设置 DDS client key 和 MAV_SYS_ID；共享同一个 Agent。
        autopilot = ExecuteProcess(
            cmd=[str(binary), '-d', '-i', str(instance), '-w', str(vehicle_runtime),
                 '-s', str(share / 'config' / 'px4_startup.sh'), str(build / 'etc')],
            name=f'px4_{namespace}', cwd=str(vehicle_runtime),
            additional_env=dict(px4_environment, **{
                'PX4_GZ_MODEL_NAME': f'x500_{instance}',
                'PX4_UXRCE_DDS_NS': namespace,
            }),
            output='screen')

        def mission_remaps(*names):
            return [(f'/uav/{name}', f'/{namespace}/{name}') for name in names]

        controller_params = {'target_system': instance + 1}
        if instance == 1:
            # S2 的地图参考点是静置机体 base_link 原点，坐标为 ENU（米）。
            controller_params['map_reference_position'] = [-22.0, -18.0, 0.227]
        # 显式节点名让 launch 按完整节点路径写入参数，S2 才能覆盖共用 YAML 的参考点。
        controller = Node(
            package='uav_planning', executable=controller_executable,
            name='goto_point_node', namespace=namespace,
            parameters=[params_file, controller_params],
            remappings=mission_remaps(
                'goal', 'path', 'estimated_pose', 'sortie_path', 'execution_state',
                'planning_start') + [
                (topic, f'/{namespace}{topic}') for topic in fmu_topics],
            output='screen')
        planner = Node(
            package='uav_planning', executable='planner_node',
            name='planner_node', namespace=namespace,
            parameters=[params_file, {
                'world_file': str(world), 'use_planning_start': return_hover}],
            remappings=mission_remaps(
                'mission', 'path', 'estimated_pose', 'sortie', 'sortie_path',
                'planning_state', 'planning_start', 'planned_path'),
            output='screen')
        processes.extend([
            (autopilot, f'{namespace} PX4 SITL'),
            (controller, f'{namespace} controller'),
            (planner, f'{namespace} planner'),
        ])

    actions = [LogInfo(msg=f'Dual X500: world={world_name}, runtime={runtime}')]
    for process, label in processes:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=process, on_exit=_shutdown_on_exit(label))))
    actions.extend(process for process, _ in processes)
    if LaunchConfiguration('gui').perform(context) == 'true':
        actions.append(ExecuteProcess(
            cmd=['gz', 'sim', '-g', '--render-engine', 'ogre'],
            additional_env=environment, output='screen'))
    if LaunchConfiguration('show_paths').perform(context) == 'true':
        actions.append(Node(
            package='uav_planning', executable='flight_path_overlay',
            additional_env=environment, output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('uav_planning'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'show_paths', default_value='false', choices=['true', 'false'],
            description='Draw planned routes and ground-truth tracks in the Gazebo GUI.'),
        DeclareLaunchArgument(
            'return_hover', default_value='false', choices=['true', 'false'],
            description='Hold above the home pad between dispatched sorties.'),
        DeclareLaunchArgument('px4_dir', default_value=str(Path.home() / 'PX4-Autopilot')),
        DeclareLaunchArgument('runtime_dir', default_value=str(
            Path.home() / 'uav_ws' / 'run' / 'px4_dual')),
        DeclareLaunchArgument('dds_port', default_value='8888'),
        DeclareLaunchArgument('world', default_value=str(share / 'worlds' / 'flood_demo.sdf')),
        OpaqueFunction(function=_launch_setup),
    ])
