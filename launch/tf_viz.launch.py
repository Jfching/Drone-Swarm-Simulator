from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Start the TF visualizer
    tf_visualizer = Node(
        package='multi_drone_system',
        executable='tf_visualizer',
        name='tf_visualizer',
        output='screen'
    )
    
    # Start RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/opt/ros/humble/share/rviz2/default.rviz'],  # Use default config
        output='screen'
    )
    
    return LaunchDescription([
        tf_visualizer,
        rviz
    ])