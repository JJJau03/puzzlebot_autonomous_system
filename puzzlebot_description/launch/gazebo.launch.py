from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Model argument
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=PathJoinSubstitution([
            FindPackageShare('puzzlebot_description'),
            'urdf',
            'puzzlebot.urdf.xacro'
        ]),
        description='Path to robot URDF file'
    )

    # Robot description with Gazebo plugins
    robot_description = Command([
        'xacro ', LaunchConfiguration('model'), ' use_gazebo:=true'
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Gazebo Sim (Fortress uses 'ign gazebo')
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-v', '4', '-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn entity (using ign service)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'puzzlebot',
            '-z', '0.0254',
           
        ],
        output='screen'
    )

    # ROS-Ignition Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            #'/model/puzzlebot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"
        ],
        output='screen'
    )

    test = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'puzzlebot/odom'],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        test
    ])