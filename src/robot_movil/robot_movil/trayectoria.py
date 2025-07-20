import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathVisualizer(Node):
    def __init__(self, robot_namespace=''):
        # Nombre simple del nodo (sin incluir el namespace en el nombre)
        super().__init__('path_visualizer', namespace=robot_namespace)
        
        self.robot_namespace = robot_namespace
        
        # Publicador en el namespace del robot
        self.path_pub = self.create_publisher(
            Path, 
            'path',  # Topic relativo al namespace
            10
        )

        # Suscriptor en el namespace del robot
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/filtered',
            self.odom_callback,
            10
        )

        self.path_msg = Path()
        self.path_msg.header.frame_id = f'{self.robot_namespace}/odom' if self.robot_namespace else 'odom'

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        
        if len(self.path_msg.poses) > 1000:
            self.path_msg.poses.pop(0)
            
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Nodo temporal solo para leer parámetros
    temp_node = Node('temp_node')
    temp_node.declare_parameter('robot_namespace', '')
    robot_namespace = temp_node.get_parameter('robot_namespace').value
    temp_node.destroy_node()
    
    # Crear nodo principal con el namespace adecuado
    path_visualizer = PathVisualizer(robot_namespace=robot_namespace)
    
    try:
        rclpy.spin(path_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        path_visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()