from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # Robot 1 con namespace
        GroupAction([
            PushRosNamespace('robot1'),
            Node(
                package='robot_movil',
                executable='trayectoria',
                name='path_visualizer',  # Nombre base igual para todos
                exec_name='robot1_path',  # Nombre único de proceso
                output='screen',
                parameters=[{'robot_namespace': 'robot1'}]
            )
        ]),
        
        # Robot 2 con namespace
        GroupAction([
            PushRosNamespace('robot2'),
            Node(
                package='robot_movil',
                executable='trayectoria',
                name='path_visualizer',  # Mismo nombre base
                exec_name='robot2_path',  # Nombre único de proceso
                output='screen',
                parameters=[{'robot_namespace': 'robot2'}]
            )
        ])
    ])