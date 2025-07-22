import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np
import py_trees


class GoToPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name="GoToPosition", node=None):
        super().__init__(name)
        # Declarar puertos de entrada
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(
            key="robot_ns",
            access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="goal_coords",
            access=py_trees.common.Access.READ
        )

        self.node = node
        self.namespace = self.blackboard.robot_ns
        self.goal = self.blackboard.goal_coords # [x, y]
        self.current_pose = np.zeros(3)  # x, y, theta
        self.reached_goal = False
        self.angular_speed = 0
        self.prev_angular_z = 0

        # Parámetros de control
        self.k_linear = 0.4
        self.k_angular = 1.2
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        self.goal_tolerance = 0.05
        self.angle_tolerance = 0.08
        self.slowdown_rad = 0.3

    def initialise(self):
        """Solo se ejecuta cuando el nodo se activa por primera vez"""
        self.namespace = self.blackboard.robot_ns
        self.goal = self.blackboard.goal_coords # [x, y]
        self.node.get_logger().info(f"Robot seleccionado: {self.namespace}")
        self.node.get_logger().info(f"Cordenadas: {self.goal}")

            # Suscripción a odometría
        self.node.create_subscription(
            Odometry,
            f'/{self.namespace}/odometry/filtered',
            self.odom_callback,
            10
        )

        # Publicador de velocidad
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10
        )
        self.reached_goal = False

    
    def odom_callback(self, msg):
        """Actualiza posición y orientación actual del robot."""
        #self.node.get_logger().info(f"Odometría recibida: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        _, _, self.current_pose[2] = euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )

    def update(self):
        #self.node.get_logger().info("ejecutando update")
        
        if self.goal is None:
            self.node.get_logger().warn("⚠️ No se ha definido una meta.")
            return py_trees.common.Status.FAILURE

        # Cálculo de errores
        dx = self.goal[0] - self.current_pose[0]  # Error en x
        dy = self.goal[1] - self.current_pose[1]  # Error en y
        distance = np.hypot(dx, dy)  # Distancia euclidiana al objetivo
        target_angle = np.arctan2(dy, dx)  # Ángulo hacia el objetivo
        angle_error = self.normalize_angle(target_angle - self.current_pose[2])  # Error angular normalizado
        self.node.get_logger().info(f"Distancia error: d={distance}, Angulo error: ang={angle_error}")

        if abs(distance) < self.goal_tolerance:
            self.cmd_vel_pub.publish(Twist())  # Detener robot
            self.reached_goal = True
            self.node.get_logger().info("Completado")
            return py_trees.common.Status.SUCCESS

        cmd = Twist()
        if abs(angle_error) > self.angle_tolerance:
            # Alineación
            cmd.angular.z = np.clip(self.k_angular * angle_error, -self.max_angular_speed, self.max_angular_speed)
        else:
            # Movimiento hacia objetivo
            cmd.linear.x = min(self.k_linear * distance, self.max_linear_speed* min(1.0, distance / self.slowdown_rad))
            self.angular_speed = np.clip(self.k_angular * angle_error, -self.max_angular_speed, self.max_angular_speed)
            
            cmd.angular.z = 0.8*self.angular_speed + 0.2 * self.prev_angular_z
            self.prev_angular_z = cmd.angular.z

        self.cmd_vel_pub.publish(cmd)
        """self.node.get_logger().info(
            f"Avanzando: vel_lineal={cmd.linear.x:.3f} m/s, "
            f"vel_angular={cmd.angular.z:.3f} rad/s"
        )"""
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.cmd_vel_pub.publish(Twist())  # Parada de seguridad

    @staticmethod
    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle
