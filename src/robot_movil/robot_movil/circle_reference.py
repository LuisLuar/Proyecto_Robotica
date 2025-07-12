#!/usr/bin/env python3
# Shebang para indicar que el script debe ejecutarse con Python 3

# Importación de librerías
import rclpy  # Librería principal de ROS 2 para Python
from rclpy.node import Node  # Para crear nodos ROS
import numpy as np  # Para cálculos numéricos
from nav_msgs.msg import Odometry  # Mensaje para la odometría del robot
from geometry_msgs.msg import Twist  # Mensaje para comandos de velocidad
from tf_transformations import euler_from_quaternion  # Para convertir orientación
from time import time  # Para medición de tiempos

class SquareTrajectoryController(Node):
    """Nodo ROS que controla un robot para seguir una trayectoria cuadrada"""
    
    def __init__(self):
        """Inicializa el nodo con parámetros, variables de estado y comunicaciones ROS"""
        super().__init__('controlador_trayectoria_cuadrada')  # Inicializa el nodo ROS

        # ========== PARÁMETROS CONFIGURABLES ==========
        self.declare_parameters(
            namespace='',
            parameters=[
                ('k_linear', 0.2),  # Ganancia para control lineal
                ('k_angular', 0.5),  # Ganancia para control angular
                ('max_linear_speed', 0.2),  # Velocidad lineal máxima (m/s)
                ('max_angular_speed', 0.8),  # Velocidad angular máxima (rad/s)
                ('slowdown_radius', 0.3),  # Radio para comenzar a frenar (m)
                ('goal_tolerance', 0.05),  # Tolerancia para considerar objetivo alcanzado (m)
                #('min_distance', 0.05),  # Distancia mínima para considerar obstáculo
                ('initial_angle_tolerance', 0.05),  # ~22.5°
                ('final_angle_tolerance', np.pi/8),        # ~2.86°
                ('wait_time_between_goals', 0.5),  # Tiempo de espera entre objetivos (s)
                ('wait_after_orientation', 0.5),  # Tiempo de espera después de alineación (s)
                ('mission_repetitions', 1)  # Veces que se repetirá la trayectoria
            ]
        )

        # ========== ESTADO INTERNO ==========
        self.current_pose = np.zeros(3)  # [x, y, theta] posición y orientación actual
        self.current_goal_idx = 0  # Índice del objetivo actual en la lista de goals
        self.mission_complete = False  # Bandera de misión completada
        self.last_goal_time = time()  # Último tiempo que se alcanzó un objetivo
        self.ready_to_move_time = None  # Tiempo en que el robot está listo para moverse
        self.waiting_to_move = False  # Bandera de espera para moverse
        self.moving_toward_goal = False  # Bandera de movimiento hacia objetivo
        self.prev_angular_z = 0.0  # Valor anterior de velocidad angular para suavizado
        self.completed_repetitions = 0  # Contador de repeticiones completadas

        # ========== TRAYECTORIA ==========
        # Lista de objetivos [x, y, theta] que forman un cuadrado
        self.goals = [
            [1.0, 0.0, 0.0],        # Vértice 1: frente, orientación 0
            [1.0, 1.0, np.pi / 2],  # Vértice 2: derecha, orientación 90°
            [0.0, 1.0, np.pi],      # Vértice 3: atrás, orientación 180°
            [0.0, 0.0, 3 * np.pi / 2] # Vértice 4: izquierda, orientación 270°
        ]

        # ========== COMUNICACIONES ROS ==========
        # Publicador para comandos de velocidad
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscriptor para odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        
        # Temporizador para control periódico (20 Hz)
        self.control_timer = self.create_timer(0.05, self.execute_control)

        # Parada inicial de seguridad
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("🚗 Controlador de trayectoria cuadrada iniciado.")

    def odom_callback(self, msg):
        """Callback para actualizar la pose actual del robot"""
        if self.mission_complete:
            return  # Ignora odometría si la misión está completa

        # Actualiza posición (x,y)
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y

        # Convierte orientación (quaternion) a ángulo de Euler (solo yaw)
        quat = msg.pose.pose.orientation
        _, _, self.current_pose[2] = euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w]
        )

    def calculate_control(self):
        """Calcula los comandos de velocidad para alcanzar el objetivo actual"""
        cmd_vel = Twist()  # Comando de velocidad inicializado a cero
        
        if self.mission_complete:
            return cmd_vel  # Retorna cero si misión completada

        goal = self.goals[self.current_goal_idx]  # Objetivo actual

        # Cálculo de errores
        dx = goal[0] - self.current_pose[0]  # Error en x
        dy = goal[1] - self.current_pose[1]  # Error en y
        distance = np.hypot(dx, dy)  # Distancia euclidiana al objetivo
        target_angle = np.arctan2(dy, dx)  # Ángulo hacia el objetivo
        angle_error = self.normalize_angle(target_angle - self.current_pose[2])  # Error angular normalizado

        # Obtención de parámetros configurables
        k_lin = self.get_parameter('k_linear').value  # Ganancia lineal
        k_ang = self.get_parameter('k_angular').value  # Ganancia angular
        max_lin = self.get_parameter('max_linear_speed').value  # Vel. lineal máxima
        max_ang = self.get_parameter('max_angular_speed').value  # Vel. angular máxima
        slowdown_rad = self.get_parameter('slowdown_radius').value  # Radio de frenado
        goal_tol = self.get_parameter('goal_tolerance').value  # Tolerancia de objetivo
        wait_time = self.get_parameter('wait_time_between_goals').value  # Tiempo de espera
        wait_after_orientation = self.get_parameter('wait_after_orientation').value  # Espera post-alineación
        mission_repetitions = self.get_parameter('mission_repetitions').value  # Repeticiones
        initial_tol = self.get_parameter('initial_angle_tolerance').value # Para alineación inicial        
        final_tol = self.get_parameter('final_angle_tolerance').value # Para orientación final

        # Lógica de control principal
        if distance > goal_tol:  # Si no hemos alcanzado el objetivo
            if not self.moving_toward_goal:  # Fase de alineación angular
                if abs(angle_error) > initial_tol:  # Si el error angular es grande
                    self.waiting_to_move = False
                    self.ready_to_move_time = None
                    # Control solo angular (giro en el lugar)
                    cmd_vel.angular.z = np.clip(k_ang * angle_error, -max_ang, max_ang)
                    return cmd_vel
                else:  # Error angular pequeño, esperar antes de moverse
                    if not self.waiting_to_move:
                        self.ready_to_move_time = time()
                        self.waiting_to_move = True

                    if time() - self.ready_to_move_time < wait_after_orientation:
                        return cmd_vel  # Espera sin movimiento
                    else:
                        self.moving_toward_goal = True  # Listo para movimiento lineal

            # Control combinado lineal + angular
            linear_speed = min(k_lin * distance, max_lin * min(1.0, distance / slowdown_rad))
            angular_speed = k_ang * angle_error
            angular_speed = np.clip(angular_speed, -max_ang, max_ang)

            # Suavizado de la velocidad angular (80% nuevo + 20% anterior)
            cmd_vel.linear.x = linear_speed
            cmd_vel.angular.z = 0.8 * angular_speed + 0.2 * self.prev_angular_z
            self.prev_angular_z = cmd_vel.angular.z

        else:  # Hemos alcanzado la posición del objetivo
            # Detenemos el robot entre objetivos
            self.cmd_vel_pub.publish(Twist())

            # Verificamos la orientación final deseada
            final_angle_error = self.normalize_angle(goal[2] - self.current_pose[2])

            if abs(final_angle_error) > final_tol:
                # Ajuste fino de orientación
                cmd_vel.angular.z = np.clip(k_ang * final_angle_error, -max_ang, max_ang)
                self.prev_angular_z = cmd_vel.angular.z
            else:
                # Espera antes de pasar al siguiente objetivo
                if time() - self.last_goal_time > wait_time:
                    if self.current_goal_idx < len(self.goals) - 1:  # Si hay más objetivos
                        self.current_goal_idx += 1  # Siguiente objetivo
                        self.last_goal_time = time()
                        self.ready_to_move_time = None
                        self.waiting_to_move = False
                        self.moving_toward_goal = False
                        self.get_logger().info(f"➡️ Objetivo {self.current_goal_idx} alcanzado. Avanzando al siguiente.")
                    else:  # Último objetivo alcanzado
                        self.completed_repetitions += 1
                        if self.completed_repetitions < mission_repetitions:
                            # Reinicia para otra repetición
                            self.get_logger().info(f"🔁 Repetición {self.completed_repetitions}/{mission_repetitions}. Reiniciando.")
                            self.current_goal_idx = 0
                            self.last_goal_time = time()
                            self.ready_to_move_time = None
                            self.waiting_to_move = False
                            self.moving_toward_goal = False
                        else:  # Misión completamente terminada
                            self.get_logger().info("✅ ¡Misión completada totalmente!")
                            self.mission_complete = True
                            self.control_timer.cancel()
                            self.cmd_vel_pub.publish(Twist())  # Parada final

        return cmd_vel

    def normalize_angle(self, angle):
        """Normaliza ángulos al rango [-π, π]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def execute_control(self):
        """Ejecuta el control periódicamente y publica los comandos"""
        cmd_vel = self.calculate_control()
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    """Función principal para iniciar el nodo"""
    rclpy.init(args=args)  # Inicializa ROS 2
    controller = SquareTrajectoryController()  # Crea el controlador

    try:
        rclpy.spin(controller)  # Mantiene el nodo activo
    except KeyboardInterrupt:  # Maneja interrupción por teclado
        pass
    finally:
        # Limpieza final
        controller.cmd_vel_pub.publish(Twist())  # Parada de seguridad
        controller.destroy_node()  # Destruye el nodo
        rclpy.shutdown()  # Cierra ROS 2

if __name__ == '__main__':
    main()  # Ejecuta la función principal