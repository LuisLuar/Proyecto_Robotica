from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_nodes_with_context(context, *args, **kwargs):
    nodes = []
    robot_count = int(context.launch_configurations['robot_count'])

    for i in range(1, robot_count + 1):
        namespace = f"robot{i}"
        frame_prefix = f"{namespace}/"

        # Cargar parámetros YAML
        yaml_file = PathJoinSubstitution([
            get_package_share_directory('robot_localization'),
            'params',
            'ekf_new.yaml'
        ]).perform(context)

        with open(yaml_file, 'r') as f:
            params = yaml.safe_load(f)['ekf_filter_node']['ros__parameters']

        # Actualizar parámetros dinámicos 
        params.update({
            'odom_frame': f"{frame_prefix}odom",
            'base_link_frame': f"{frame_prefix}base_footprint",
            'world_frame': f"{frame_prefix}odom",
            'odom0': 'odom/unfiltered',  # <- Cambiado (sin namespace)
            'imu0': 'imu/unfiltered'     # <- Cambiado (sin namespace)
        })

        nodes.append(
            GroupAction([
                PushRosNamespace(namespace),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    parameters=[params],
                    output='screen'
                )
            ])
        )
    
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_count',
            default_value='2',
            description='Número de robots'
        ),
        OpaqueFunction(function=generate_nodes_with_context)
    ])