# Copyright (c) 2024 Franka Robotics GmbH
# Copyright (c) 2026 Agile Robots SE (modifications: adapted for Diana robots)
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# This file is an adapted version of
# https://github.com/ros-planning/moveit_resources/blob/ca3f7930c630581b5504f3b22c40b4f82ee6369d/panda_moveit_config/launch/demo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml

PARAM_NAMES={
    'ROBOT_IP':'robot_ip',
    'USE_FAKE_HARDWARE':'use_fake_hardware',
    'FAKE_SENSOR_COMMANDS':'fake_sensor_commands',
    'NAMESPACE':'namespace',
    'LOAD_GRIPPER':'load_gripper',
    'EE_ID':'ee_id',
    'ROBOT_TYPE':'robot_type',
}


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_robot_nodes(context, *args, **kwargs):
    robot_ip = LaunchConfiguration(PARAM_NAMES['ROBOT_IP'])
    use_fake_hardware = LaunchConfiguration(PARAM_NAMES['USE_FAKE_HARDWARE'])
    fake_sensor_commands = LaunchConfiguration(PARAM_NAMES['FAKE_SENSOR_COMMANDS'])
    namespace = LaunchConfiguration(PARAM_NAMES['NAMESPACE'])
    load_gripper = LaunchConfiguration(PARAM_NAMES['LOAD_GRIPPER'])
    ee_id = LaunchConfiguration(PARAM_NAMES['EE_ID'])
    robot_type = LaunchConfiguration(PARAM_NAMES['ROBOT_TYPE'])

    robot_type_str = robot_type.perform(context)

    # planning_context
    diana_xacro_file = os.path.join(
        get_package_share_directory('diana_bringup'),
        'urdf', 'diana_arm.urdf.xacro'
    )

    robot_description_config = Command(
            [FindExecutable(name='xacro'), ' ', diana_xacro_file,
             ' hand:=', load_gripper, ' robot_type:=', robot_type,
             ' robot_ip:=', robot_ip, ' ee_id:=', ee_id,
             ' use_fake_hardware:=', use_fake_hardware,
             ' fake_sensor_commands:=', fake_sensor_commands])

    robot_description = {'robot_description': ParameterValue(
        robot_description_config, value_type=str)}

    diana_semantic_xacro_file = os.path.join(
        get_package_share_directory('diana_description'),
        'robots', robot_type_str, robot_type_str+'.srdf.xacro'
    )

    robot_description_semantic_config = Command(
            [FindExecutable(name='xacro'), ' ',
             diana_semantic_xacro_file, ' hand:=', load_gripper, ' ee_id:=', ee_id]
        )
    
    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    kinematics_yaml = load_yaml(
        'diana_moveit_config', 'config/'+robot_type_str+'/kinematics.yaml'
    )

    # Planning Functionality
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'diana_moveit_config', 'config/'+robot_type_str+'/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'diana_moveit_config', 'config/'+robot_type_str+'/controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager'
                                     '/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz  
    rviz_base = os.path.join(get_package_share_directory(
        'diana_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit_' + robot_type_str + '.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[robot_description],
    )

    # mock_components/GenericSystem (use_fake_hardware:=true) mirrors the
    # controller command into the matching state interface and cannot simulate
    # dynamics from effort commands, so fake mode needs a position-based
    # controller configuration.
    controllers_config_name = (
        'ros_controllers_fake.yaml'
        if use_fake_hardware.perform(context).lower() == 'true'
        else 'ros_controllers.yaml'
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory('diana_moveit_config'),
        'config',
        robot_type_str + '/' + controllers_config_name,
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'diana/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        on_exit=Shutdown(),
    )

    # Load controllers
    load_controllers = []
    for controller in [robot_type_str+'_arm_controller', 'joint_state_broadcaster']:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', controller,
                    '--controller-manager-timeout', '60',
                    '--controller-manager',
                    PathJoinSubstitution([namespace, 'controller_manager'])
                ],
                output='screen'
            )
        )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'source_list': ['diana/joint_states', 'diana_gripper/joint_states'], 'rate': 30}],
    )

    diana_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['diana_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    return [
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_publisher,
        diana_robot_state_broadcaster,
    ] + load_controllers

def generate_launch_description():
    
    robot_arg = DeclareLaunchArgument(
        PARAM_NAMES['ROBOT_IP'],
        description='Hostname or IP address of the robot.')

    namespace_arg = DeclareLaunchArgument(
        PARAM_NAMES['NAMESPACE'],
        default_value='',
        description='Namespace for the robot.'
    )
    load_gripper_arg = DeclareLaunchArgument(
        PARAM_NAMES['LOAD_GRIPPER'],
        default_value='false',
        description='Whether to load the gripper or not (true or false)'
    )
    ee_id_arg = DeclareLaunchArgument(
        PARAM_NAMES['EE_ID'],
        default_value='diana_hand',
        description='The end-effector id to use. Available options: none, diana_hand'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        PARAM_NAMES['USE_FAKE_HARDWARE'],
        default_value='false',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        PARAM_NAMES['FAKE_SENSOR_COMMANDS'],
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            PARAM_NAMES['FAKE_SENSOR_COMMANDS']))
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz'
    )
    robot_type_arg = DeclareLaunchArgument(
        PARAM_NAMES['ROBOT_TYPE'], description='robot type'
    )
    db_arg = DeclareLaunchArgument(
        'db', default_value='False', description='Database flag'
    )

    robot_ip = LaunchConfiguration(PARAM_NAMES['ROBOT_IP'])
    use_fake_hardware = LaunchConfiguration(PARAM_NAMES['USE_FAKE_HARDWARE'])
    namespace = LaunchConfiguration(PARAM_NAMES['NAMESPACE'])

    return LaunchDescription(
        [robot_arg,
         namespace_arg,
         load_gripper_arg,
         ee_id_arg,
         use_fake_hardware_arg,
         fake_sensor_commands_arg,
         db_arg,
         use_rviz_arg,
         robot_type_arg,
         OpaqueFunction(function=generate_robot_nodes),
         ]
    )
