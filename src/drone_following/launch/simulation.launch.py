#!/usr/bin/env python3

# *******************************************************************************
# Script Name  : simulation.launch.py
# Author       : Daniel Sotelo Aguirre
# Date         : 08/11/2024
# Version      : v1.0
# *******************************************************************************

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):

    # Initialize arguments
    ur_type = LaunchConfiguration("ur_type")
    wind_enabled = LaunchConfiguration("wind_enabled")

    # General arguments
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # Declare the path to the URDF file and controllers
    package_dir = get_package_share_directory('drone_following')

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "resource/config", controllers_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # Robot control related packages
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # Spawn UR5e robot
    gz_spawn_entity = TimerAction(
        period=18.0,  # Delay to ensure world is initialized
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-world",
                "follow_drone_wind" if wind_enabled == 'true' else "follow_drone",
                "-string",
                robot_description_content,
                "-name",
                "ur",
                "-allow_renaming",
                "true",
            ],
        )]
    )

    # Bridge for sharing topics and visualization in RViz2
    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/processed_image@sensor_msgs/msg/Image@gz.msgs.Image"
        ],
        output="screen",
    )

    # Open terminals for initialization of PX4
    px4_sitl_process = ExecuteProcess(
        cmd=[
            'gnome-terminal', '--', 'bash', '-c',
            [
                'cd ~/PX4-Autopilot && ',
                'make px4_sitl gz_x500_tag_wind' if wind_enabled == 'true' else 'make px4_sitl gz_x500_tag',
                '; exec bash'
            ]
        ],
        output='screen'
    )

    # Initialize the MicroXRCE agent
    micro_dds_agent_process = TimerAction(
        period = 8.0,
        actions = [ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'MicroXRCEAgent udp4 -p 8888; exec bash'],
            output='screen'
        )]
    )

    # Control of the drone (4 random positions)
    drone_control_node = TimerAction(
        period=25.0,  # Delay to ensure PX4 and DDS agent are initialized
        actions=[Node(
            package='drone_following',
            namespace='drone_following',
            executable='drone_control',
            name='drone_control',
            output='screen'
        )]
    )

    # Visualization node for plotting in RViz
    visualizer_node = Node(
        package='drone_following',
        namespace='drone_following',
        executable='visualizer',
        name='visualizer',
        output='screen'
    )

    # Initialize RViz2
    rviz_node = TimerAction(
        period=25.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(package_dir, 'resource/rviz/visualize.rviz')],
            name='rviz2',
            output='log'
        )]
    )

    # Node for detecting the ArUco
    aruco_detector_node = Node(
        package='drone_following',
        namespace='drone_following',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[{'publish_image': True}]
    )

    # Node for controlling the robotic arm (pan-and-tilt PID control)
    arm_control = TimerAction(
        period=25.0,
        actions=[Node(
            package='drone_following',
            namespace='drone_following',
            executable='arm_control',
            name='arm_control',
            output='screen'
        )]
    )

    # List of nodes to initialize
    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        rviz_node,
        gz_spawn_entity,
        gz_sim_bridge,
        px4_sitl_process,
        micro_dds_agent_process,
        drone_control_node,
        visualizer_node,
        aruco_detector_node,
        arm_control
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    # Enable wind
    declared_arguments.append(
        DeclareLaunchArgument(
            "wind_enabled",
            default_value="true",
            description="Enable or disable wind in the simulation",
        )
    )

    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="drone_following",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur5e_gripper_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_joint_controller",
            default_value="true",
            description="Enable headless mode for robot control",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])