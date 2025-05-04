import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Gazebo simulation
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("puzzlebot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )
        
    # Launch RViz
    rviz = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("puzzlebot_description"),
            "launch",
            "simple.launch.py"
        ),
        launch_arguments={"is_sim": "True"}.items()
    )
    
    # Return the launch description with Gazebo and RViz
    return LaunchDescription([
        gazebo,
        rviz
    ])  