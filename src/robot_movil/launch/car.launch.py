from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([        
        Node(
            package='robot_movil',
            executable='cuadrado',
            name='nodo_cuadrado',
            output='screen'
        ),

        Node(
            package='robot_movil',
            executable='guardar',
            name='nodo_guardar',
            output='screen'
        )
    ])
