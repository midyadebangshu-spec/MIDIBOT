"""Publish MIDIBOT's robot_description and TF (no simulator). Add rviz:=true to open RViz."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("midibot_description")
    vendor = LaunchConfiguration("vendor_meshes")
    description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([pkg, "urdf", "midibot.urdf.xacro"]),
        " sim:=false vendor_meshes:=", vendor]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("vendor_meshes", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher"),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "midibot.rviz"])]),
    ])
