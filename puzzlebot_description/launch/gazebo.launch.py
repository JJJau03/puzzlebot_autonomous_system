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

    # Load robot state publisher with explicit remapping
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description, 
            "use_sim_time": True,
            "publish_frequency": 50.0  # Higher rate for smoother TF
        }],
        remappings=[  # Explicit remapping ensures connectivity
            ("/joint_states", "/joint_states")
        ]
    )

    # Launch Gazebo with system clock
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": "-v 4 -r empty.sdf",
            "use_sim_time": "true"  # Critical for time synchronization
        }.items()
    )

    # Spawn robot in Gazebo
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
    
    # Controller config path
    controller_config = os.path.join(package_dir, "config", "puzzlebot_controller.yaml")

    # Start ros2_control_node with proper parameters
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controller_config,
            {"use_sim_time": True},
            {"diff_drive_controller": {
                "ros__parameters": {
                    "use_stamped_vel": False }}}
        ],
        output="screen",
       
    )

   
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
                # remappings=[
                #     ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel")
                # ],
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
        spawn_joint_state,
        spawn_diff_drive
    ])