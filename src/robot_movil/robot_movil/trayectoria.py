#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        # Publicador del mensaje Path
        self.path_pub = self.create_publisher(Path, '/path', 10)

        # Suscriptor a la odometría filtrada
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Inicializar el mensaje Path
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'  # Puede ser 'map' si usas SLAM

    def odom_callback(self, msg):
        # Crear un nuevo pose stamped con la posición del robot
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Agregar la pose a la trayectoria
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        # Publicar el path
        self.path_pub.publish(self.path_msg)

        # Opcional: limitar longitud
        if len(self.path_msg.poses) > 1000:
            self.path_msg.poses.pop(0)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
