from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_robot_movil = get_package_share_directory('robot_movil')
    
    return LaunchDescription([
        # 1. Micro-ROS Agent
        ExecuteProcess(
            cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', '8888'],
            output='screen',
            shell=True
        ),

        # 2. EKF Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot_movil, 'launch', 'multi_ekf.launch.py')
            )
        ),

        # 3. Static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'robot1/odom'],
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '1', '0', '0', '0', '0', 'odom', 'robot2/odom'],
            output='screen'
        ),

        # 4. Path visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot_movil, 'launch', 'multi_robot_path.launch.py')
            )
        ),

        # 5. RViz visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_robot_movil, 'launch', 'multi_robot_display.launch.py')
            )
        )
    ])