import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_content",
            default_value=Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("arduino_arm_hardware"), "urdf", "my_arm.urdf.xacro"]
                    ),
                ]
            ),
            description="XML string of the robot description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("arduino_arm_hardware"), "config", "controllers.yaml"]
            ),
            description="Path to the controllers YAML file.",
        )
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": LaunchConfiguration("robot_description_content")}],
        output="screen",
    )

    # Controller Manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("controllers_file"),
            {"robot_description": LaunchConfiguration("robot_description_content")}
        ],
        output="screen",
    )

    # Load the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Load the custom arm controller
    arduino_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arduino_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        declared_arguments +
        [
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            arduino_arm_controller_spawner,
        ]
    )
