from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory("puzzlebot_description")

    # Declare robot model path
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(package_dir, "urdf", "puzzlebot.urdf.xacro"),
        description="Path to robot URDF file"
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), 
        value_type=str
    )

    # Load robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}]
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v 4 -r empty.sdf"}.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "puzzlebot"]
    )

    # Controller config path
    controller_config = os.path.join(package_dir, "config", "puzzlebot_controller.yaml")

    # Start ros2_control_node (the controller manager)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controller_config],
        output="screen"
    )

    # Spawn diff_drive_controller (after a short delay)
    spawn_diff_drive = TimerAction(
        period=5.0,  # wait a few seconds to ensure controller manager is up
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        ros2_control_node,
        spawn_diff_drive
    ])
