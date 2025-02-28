from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('multi_drone_system')
    
    # Start Gazebo with the custom world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', os.path.join(pkg_dir, 'worlds', 'drone_world.world')],
        output='screen'
    )
    
    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        rviz
    ])