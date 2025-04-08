from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory("puzzlebot_description")

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(package_dir, "urdf", "puzzlebot.urdf.xacro"),
        description="Path to robot URDF file"
    )

    use_ignition_arg = DeclareLaunchArgument(
        name="use_ignition",
        default_value="true",
        description="Use Ignition Gazebo (Fortress) simulation"
    )

    # Prepare robot_description from xacro
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), 
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description, 
            "use_sim_time": True,
            "publish_frequency": 50.0
        }],
        remappings=[("/joint_states", "/joint_states")]
    )

    # Launch Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-v 4 -r empty.sdf",
            "use_sim_time": "true"
        }.items()
    )

    # Spawn robot entity
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "puzzlebot",
            "-allow_renaming", "false"
        ],
        output="screen"
    )
    
    # ros2_control_node: only launch if not using ignition
    controller_config = os.path.join(package_dir, "config", "puzzlebot_controller.yaml")

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controller_config,
            {"use_sim_time": True},
            {"diff_drive_controller": {
                "ros__parameters": {
                    "use_stamped_vel": False
                }}
            }
        ],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_ignition"))
    )

    # Spawners for controllers (these can remain always active)
    spawn_joint_state = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen"
            )
        ]
    )

    spawn_diff_drive = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller"],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        model_arg,
        use_ignition_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        ros2_control_node,
        spawn_joint_state,
        spawn_diff_drive
    ])
