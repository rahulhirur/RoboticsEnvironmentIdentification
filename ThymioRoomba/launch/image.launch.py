from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'thymio_name',
            default_value='thymio0'
        ),
        
        Node(
            package='thymiroomba',
            executable='image_controller',
            namespace=LaunchConfiguration('thymio_name'),
            output='screen',
            
            # Uncomment to enable DEBUG logging messages
            # arguments=['--ros-args', '--log-level', 'debug']
        )
    ])
