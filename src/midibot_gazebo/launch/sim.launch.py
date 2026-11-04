"""Spawn MIDIBOT in Gazebo Harmonic with ros2_control.

Args:
  headless:=true|false   run the Gazebo server only (default true)
  sensors:=true|false    enable rendering sensors (RGB-D camera, lidar)
  vendor_meshes:=true|false  use vendor CAD meshes (run midibot_cad/scripts/fetch_vendor_cad.py first)
  dxl_meshes:=true|false     use Dynamixel CAD meshes (downloaded by hand, see COMPONENTS.md)
  world:=<path.sdf>
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("midibot_description")
    gz_pkg = FindPackageShare("midibot_gazebo")
    controllers = PathJoinSubstitution([desc_pkg, "config", "ros2_controllers.yaml"])
    headless = LaunchConfiguration("headless")

    description = ParameterValue(Command([
        "xacro ", PathJoinSubstitution([desc_pkg, "urdf", "midibot.urdf.xacro"]),
        " sim:=true controllers_file:=", controllers,
        " sensors:=", LaunchConfiguration("sensors"),
        " vendor_meshes:=", LaunchConfiguration("vendor_meshes"),
        " dxl_meshes:=", LaunchConfiguration("dxl_meshes")]), value_type=str)

    gz_args = PythonExpression([
        "'-r -v 2 ' + ('-s --headless-rendering ' if '", headless, "' == 'true' else '') + '",
        LaunchConfiguration("world"), "'"])

    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items())

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               parameters=[{"robot_description": description, "use_sim_time": True}])

    spawn = Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-topic", "robot_description", "-name", "midibot",
                            "-z", LaunchConfiguration("spawn_z")])

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  parameters=[{"config_file": PathJoinSubstitution([gz_pkg, "config", "bridge.yaml"]),
                               "use_sim_time": True}])

    def spawner(name):
        return Node(package="controller_manager", executable="spawner", output="screen",
                    arguments=[name, "--controller-manager-timeout", "120"])

    controllers_after_spawn = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[
        spawner("joint_state_broadcaster"), spawner("imu_sensor_broadcaster"), spawner("body_controller"),
        spawner("left_hand_controller"), spawner("right_hand_controller")]))

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("sensors", default_value="true"),
        DeclareLaunchArgument("vendor_meshes", default_value="false"),
        DeclareLaunchArgument("dxl_meshes", default_value="false"),
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution([gz_pkg, "worlds", "flat.sdf"])),
        DeclareLaunchArgument("spawn_z", default_value="0.775"),
        gazebo, rsp, spawn, bridge, controllers_after_spawn,
        Node(package="midibot_control", executable="hand_coupler", parameters=[{"use_sim_time": True}]),
    ])
