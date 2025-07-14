#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math

from tf_transformations import euler_from_quaternion  # pip install tf-transformations

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Parámetro para la ruta del archivo
        self.declare_parameter('csv_path', '/home/raynel/Documents/odometry_data.csv')
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value

        # Abrir el archivo CSV
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'x', 'y', 'yaw', 'linear_vel', 'angular_vel'])

        # Suscripción a la odometría
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.get_logger().info(f'DataLogger escribiendo en {csv_path}')

    def odom_callback(self, msg):
        # Posición
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Cuaternión a Yaw
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Velocidades
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Escribir en CSV
        self.csv_writer.writerow([t, x, y, yaw, linear_vel, angular_vel])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
