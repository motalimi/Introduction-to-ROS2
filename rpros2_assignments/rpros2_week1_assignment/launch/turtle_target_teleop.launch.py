from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),

        Node(
            package='rpros2_week1_assignment',
            executable='turtle_target_plot',
            name='turtle_target_plot'
        ),

        ExecuteProcess(
            cmd=['xterm', '-e', 'ros2 run rpros2_week1_assignment turtle_keyboard_control'],
            output='screen'
        )
    ])
