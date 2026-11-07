# E0509 + RH-P12-RN(A) in RViz.  Based on dsr_bringup2/launch/dsr_bringup2_rviz.launch.py
# (Doosan, Apache-2.0): only the robot description (xacro) and the gripper node differ.
#
#   ros2 launch rhp12_gripper dsr_rviz_gripper.launch.py mode:=virtual
#   ros2 launch rhp12_gripper dsr_rviz_gripper.launch.py mode:=real host:=<robot ip>
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from dsr_bringup2.utils import read_update_rate


def generate_launch_description():
    args = [
        DeclareLaunchArgument('name', default_value='dsr01'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        DeclareLaunchArgument('port', default_value='12345'),
        DeclareLaunchArgument('mode', default_value='virtual'),
        DeclareLaunchArgument('model', default_value='e0509'),
        DeclareLaunchArgument('color', default_value='white'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rt_host', default_value=''),
        DeclareLaunchArgument('mount_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('mount_rpy', default_value='0 0 0'),
        # gripper bridge (see rhp12_gripper/gripper_node.py)
        DeclareLaunchArgument('real_backend', default_value='off',
                              description='off | drl  (real mode only; UNVERIFIED)'),
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='true: only log Modbus frames'),
    ]

    mode = LaunchConfiguration('mode')
    update_rate = int(read_update_rate())

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('rhp12_gripper'), 'urdf', 'e0509_rh_p12_rn_a.urdf.xacro'])

    def description(with_color):
        cmd = [PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file]
        if with_color:
            cmd += [' color:=', LaunchConfiguration('color')]
        cmd += [
            ' name:=', LaunchConfiguration('name'),
            ' host:=', LaunchConfiguration('host'),
            ' rt_host:=', LaunchConfiguration('rt_host'),
            ' port:=', LaunchConfiguration('port'),
            ' mode:=', LaunchConfiguration('mode'),
            ' model:=', LaunchConfiguration('model'),
            ' update_rate:=', str(update_rate),
            ' mount_xyz:="', LaunchConfiguration('mount_xyz'), '"',
            ' mount_rpy:="', LaunchConfiguration('mount_rpy'), '"',
        ]
        return Command(cmd)

    robot_controllers = [PathJoinSubstitution(
        [FindPackageShare('dsr_controller2'), 'config', 'dsr_controller2.yaml'])]
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('dsr_description2'), 'rviz', 'default.rviz'])

    run_emulator = Node(
        package='dsr_bringup2', executable='run_emulator',
        namespace=LaunchConfiguration('name'),
        parameters=[
            {'name': LaunchConfiguration('name')}, {'rate': 100}, {'standby': 5000},
            {'command': True}, {'host': LaunchConfiguration('host')},
            {'port': LaunchConfiguration('port')}, {'mode': mode},
            {'model': LaunchConfiguration('model')}, {'gripper': 'none'},
            {'mobile': 'none'}, {'rt_host': LaunchConfiguration('rt_host')},
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output='screen')

    control_node = Node(
        package='controller_manager', executable='ros2_control_node',
        namespace=LaunchConfiguration('name'),
        parameters=[{'robot_description': ParameterValue(description(False), value_type=str)}] + robot_controllers,
        output='both')

    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace=LaunchConfiguration('name'),
        output='both',
        parameters=[{'robot_description': ParameterValue(description(True), value_type=str)}])

    rviz = Node(
        package='rviz2', executable='rviz2', namespace=LaunchConfiguration('name'),
        name='rviz2', output='log', arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('gui')))

    jsb_spawner = Node(
        package='controller_manager', executable='spawner',
        namespace=LaunchConfiguration('name'),
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'])

    robot_ctrl_spawner = Node(
        package='controller_manager', executable='spawner',
        namespace=LaunchConfiguration('name'),
        arguments=['dsr_controller2', '-c', 'controller_manager'])

    after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=jsb_spawner, on_exit=[robot_ctrl_spawner]))

    gripper = Node(
        package='rhp12_gripper', executable='gripper_node',
        namespace=LaunchConfiguration('name'), output='screen',
        parameters=[{
            'mode': ParameterValue(mode, value_type=str),
            # 'off' would be parsed as YAML bool -> force string
            'real_backend': ParameterValue(LaunchConfiguration('real_backend'), value_type=str),
            'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool),
            # official RH-P12-RN-A model: single driving joint rh_r1 (others mimic it)
            'joint_names': ['rh_r1'],
            'joint_open': 0.0,
            'joint_closed': 1.1,
        }])

    return LaunchDescription(args + [
        run_emulator, robot_state_pub, rviz, after_jsb, jsb_spawner, control_node, gripper])
