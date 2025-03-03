from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('multi_drone_system')
    
    # Path to RViz config file
    rviz_config = os.path.join(pkg_dir, 'rviz', 'drone_swarm.rviz')
    
    # Start the TF visualizer
    tf_visualizer = Node(
        package='multi_drone_system',
        executable='tf_visualizer',
        name='tf_visualizer',
        output='screen'
    )
    
    # Start RViz with our saved configuration
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        tf_visualizer,
        rviz
    ])