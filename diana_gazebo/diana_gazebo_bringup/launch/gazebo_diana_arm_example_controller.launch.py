# Copyright (c) 2024 Franka Robotics GmbH
# Copyright (c) 2026 Agile Robots SE (modifications: adapted for Diana robots)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import xacro
import xml.dom.minidom

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_controller(context: LaunchContext, robot_type, controller_name):
    robot_type_str = context.perform_substitution(robot_type)
    controller_name_str = context.perform_substitution(controller_name)
    return [Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            controller_name_str,
            '--controller-manager-timeout', '30',
        ],
        parameters=[PathJoinSubstitution([
            FindPackageShare('diana_gazebo_bringup'),
            'config',
            robot_type_str+'_gazebo_controllers.yaml'
        ])],
        output='screen',
    )]


def get_robot_description(context: LaunchContext, robot_type, load_gripper, diana_hand):
    robot_type_str = context.perform_substitution(robot_type)
    load_gripper_str = context.perform_substitution(load_gripper)
    diana_hand_str = context.perform_substitution(diana_hand)

    diana_xacro_file = os.path.join(
        get_package_share_directory('diana_gazebo_bringup'),
        'urdf', 'diana_arm.gazebo.xacro'
    )

    robot_description_config = xacro.process_file(
        diana_xacro_file,
        mappings={
            'robot_type': robot_type_str,
            'hand': load_gripper_str,
            'gazebo': 'true',
            'ee_id': diana_hand_str,
            'gazebo_effort': 'true',
        }
    )

    if not isinstance(robot_description_config, xml.dom.minidom.Document):
        raise RuntimeError(
            f'The given xacro file {diana_xacro_file} is not a valid xml format.')

    robot_description = {'robot_description': robot_description_config.toxml()}

    return [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )]


def generate_launch_description():
    # Configure ROS nodes for launch
    load_gripper_name = 'load_gripper'
    diana_hand_name = 'diana_hand'
    robot_type_name = 'robot_type'
    namespace_name = 'namespace'
    controller_name = 'controller'
    rviz_name = 'rviz'
    gz_args_name = 'gz_args'

    load_gripper = LaunchConfiguration(load_gripper_name)
    diana_hand = LaunchConfiguration(diana_hand_name)
    robot_type = LaunchConfiguration(robot_type_name)
    namespace = LaunchConfiguration(namespace_name)
    controller = LaunchConfiguration(controller_name)
    rviz = LaunchConfiguration(rviz_name)
    gz_args = LaunchConfiguration(gz_args_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='false',
        description='true/false for activating the gripper')
    diana_hand_launch_argument = DeclareLaunchArgument(
        diana_hand_name,
        default_value='diana_hand',
        description='Default value: diana_hand')
    robot_type_launch_argument = DeclareLaunchArgument(
        robot_type_name,
        default_value='diana3',
        description='Available values: diana3, diana7 and diana7g2')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')
    controller_launch_argument = DeclareLaunchArgument(
        controller_name,
        default_value='gravity_compensation_example_controller',
        description='The controller name to be used. You can choose one from the diana_example_controllers.')
    gz_args_launch_argument = DeclareLaunchArgument(
        gz_args_name,
        default_value='empty.sdf -r',
        description='Extra args to be forwared to gazebo')
    rviz_launch_argument = DeclareLaunchArgument(
        rviz_name,
        default_value='true',
        description='true/false for visualizing the robot in rviz')

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type, load_gripper, diana_hand])

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join([
        os.path.dirname(get_package_share_directory('diana_description')),
        get_package_share_directory('diana_gazebo_bringup')])
    gazebo_empty_world = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    rviz_file = os.path.join(get_package_share_directory('diana_description'),
                             'rviz', 'visualize_diana.rviz')

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     namespace=namespace,
                     arguments=['--display-config', rviz_file, '-f', 'world'],
                     condition=IfCondition(rviz))

    launch_controller = OpaqueFunction(
        function=load_controller,
        args=[robot_type, controller]
    )

    return LaunchDescription([
        load_gripper_launch_argument,
        diana_hand_launch_argument,
        robot_type_launch_argument,
        namespace_launch_argument,
        controller_launch_argument,
        gz_args_launch_argument,
        rviz_launch_argument,
        gazebo_empty_world,
        robot_state_publisher,
        rviz_node,
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[launch_controller],
            )
        ),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=['pkill', '-SIGINT', 'ruby'],
                        name='gz_sim_graceful_shutdown',
                    )
                ]
            )
        )
    ])
