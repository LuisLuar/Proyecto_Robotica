#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import tf2_ros
import cv2
import numpy as np
from collections import deque, defaultdict
from scipy.spatial.transform import Rotation as R
from pupil_apriltags import Detector
import threading

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        
        # Configuración de reflejo (1=horizontal, 0=vertical, -1=ambos, None=sin reflejo)
        self.flip_mode = 1
        
        # Configuración inicial
        self.setup_parameters()
        self.setup_camera()
        self.setup_detector()
        self.setup_publishers()
        self.setup_tf_broadcaster()
        
        # Variables de estado protegidas con lock
        self.lock = threading.Lock()
        self.latest_detections = {}
        self.reference_pose = None
        self.reference_rotation = None
        
        # Iniciar procesamiento
        self.start_processing_timers()
        self.get_logger().info("Nodo AprilTag Detector iniciado correctamente")

    def setup_parameters(self):
        """Configura todos los parámetros del nodo"""
        # Parámetros básicos
        self.declare_parameter('camera_id', 2)
        self.declare_parameter('tag_size', 0.075)  # metros
        self.declare_parameter('reference_tag_id', 1)
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter('detection_frequency', 30.0)  # Hz
        self.declare_parameter('publish_frequency', 20.0)   # Hz
        self.declare_parameter('tf_publish_frequency', 10.0)  # Hz
        
        # Configuración de robots
        self.declare_parameter('robot1_tag_id', 3)
        self.declare_parameter('robot2_tag_id', 4)
        
        # Obtener parámetros
        self.camera_id = self.get_parameter('camera_id').value
        self.tag_size = self.get_parameter('tag_size').value
        self.reference_tag_id = self.get_parameter('reference_tag_id').value
        self.show_debug = self.get_parameter('show_debug_window').value
        self.detection_freq = self.get_parameter('detection_frequency').value
        self.publish_freq = self.get_parameter('publish_frequency').value
        self.tf_publish_freq = self.get_parameter('tf_publish_frequency').value
        
        # Configurar diccionario de robots
        self.robot_tags = {
            self.get_parameter('robot1_tag_id').value: 'robot1',
            self.get_parameter('robot2_tag_id').value: 'robot2'
        }

    def setup_camera(self):
        """Configura la cámara y parámetros de captura"""
        self.cap = cv2.VideoCapture(self.camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"No se pudo abrir la cámara con ID {self.camera_id}")
            raise RuntimeError("No se pudo inicializar la cámara")
        
        # Configuración óptima de la cámara
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, self.detection_freq)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        
        if self.show_debug:
            cv2.namedWindow("AprilTag Detection", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("AprilTag Detection", 800, 600)

    def setup_detector(self):
        """Configura el detector de AprilTags"""
        self.detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25
        )
        
        # Matriz de corrección para cámara (ajustar según necesidad)
        self.camera_correction = np.array([
            [1,  0,  0],
            [0, -1,  0],  # Inversión en Y (opcional)
            [0,  0, -1]   # Inversión en Z (opcional)
        ])

    def setup_publishers(self):
        """Configura los publishers de ROS2"""
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.robot_publishers = {}
        for tag_id, robot_name in self.robot_tags.items():
            self.robot_publishers[tag_id] = self.create_publisher(
                PoseWithCovarianceStamped,
                f'/{robot_name}/vision_pose',
                qos_profile
            )

    def setup_tf_broadcaster(self):
        """Configura el TF broadcaster"""
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def start_processing_timers(self):
        """Inicia los temporizadores para procesamiento"""
        # Temporizador para detección
        self.create_timer(1.0/self.detection_freq, self.detection_loop)
        
        # Temporizador para publicación
        self.create_timer(1.0/self.publish_freq, self.publish_loop)
        
        # Temporizador para TF
        self.create_timer(1.0/self.tf_publish_freq, self.tf_publish_loop)

    def detection_loop(self):
        """Procesamiento de detección de AprilTags con imagen reflejada"""
        try:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Error al capturar frame", throttle_duration_sec=5)
                return

            # Reflejar la imagen según el modo configurado
            if self.flip_mode in [1, 0, -1]:
                frame = cv2.flip(frame, self.flip_mode)
            
            # Obtener dimensiones de la imagen
            height, width = frame.shape[:2]
            
            # Detección en escala de grises
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self.get_camera_params(width),
                tag_size=self.tag_size
            )

            # Procesar detecciones
            processed_data = {}
            for detection in detections:
                tag_id = detection.tag_id
                processed_data[tag_id] = self.process_single_detection(detection, width)

            # Actualizar datos compartidos
            with self.lock:
                self.latest_detections = processed_data
                if self.reference_tag_id in processed_data:
                    ref_data = processed_data[self.reference_tag_id]
                    self.reference_pose = ref_data['position']
                    self.reference_rotation = ref_data['rotation']

            # Visualización
            if self.show_debug:
                self.draw_detections(frame, detections, processed_data)
                cv2.imshow("AprilTag Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.cleanup()
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error en detección: {str(e)}", throttle_duration_sec=5)

    def process_single_detection(self, detection, image_width):
        """Procesa una detección individual considerando el reflejo"""
        tag_id = detection.tag_id
        
        # Reflejar las coordenadas si es necesario
        if self.flip_mode == 1:  # Reflejo horizontal
            corners = detection.corners.copy()
            corners[:, 0] = image_width - corners[:, 0]
            # Reordenar esquinas para mantener orientación correcta
            corners = corners[[1, 0, 3, 2], :]
        else:
            corners = detection.corners
        
        # Aplicar corrección a la pose
        t_corr = self.camera_correction @ detection.pose_t.ravel()
        R_corr = self.camera_correction @ detection.pose_R
        
        # Calcular pose relativa si hay referencia
        t_rel, R_rel, quat_rel = None, None, None
        if (self.reference_pose is not None and 
            self.reference_rotation is not None and
            (tag_id in self.robot_tags or tag_id == self.reference_tag_id)):
            
            t_rel = self.reference_rotation.T @ (t_corr - self.reference_pose)
            R_rel = self.reference_rotation.T @ R_corr
            
            # Forzar 2D
            t_rel[2] = 0.0
            yaw = np.arctan2(R_rel[1, 0], R_rel[0, 0])
            
            # Ajustar ángulo si la imagen está reflejada
            if self.flip_mode == 1:
                yaw = -yaw
                
            quat_rel = R.from_euler('z', yaw).as_quat()
        
        return {
            'position': t_corr,
            'rotation': R_corr,
            'relative_position': t_rel,
            'relative_quaternion': quat_rel,
            'corners': corners,
            'center': np.mean(corners, axis=0)
        }

    def draw_detections(self, frame, detections, processed_data):
        """Dibuja las detecciones en el frame con diferentes colores"""
        for detection in detections:
            tag_id = detection.tag_id
            if tag_id not in processed_data:
                continue
                
            data = processed_data[tag_id]
            corners = data['corners'].astype(int)
            center = data['center'].astype(int)
            
            # Determinar color según tipo de tag
            if tag_id == self.reference_tag_id:
                color = (0, 255, 255)  # Amarillo para referencia
                label = f"REF {tag_id}"
            elif tag_id in self.robot_tags:
                color = (0, 255, 0)  # Verde para robots conocidos
                label = f"{self.robot_tags[tag_id]} (ID:{tag_id})"
            else:
                color = (0, 0, 255)  # Rojo para tags desconocidos
                label = f"UNKNOWN {tag_id}"
            
            # Dibujar contorno y texto
            cv2.polylines(frame, [corners], True, color, 2)
            cv2.putText(frame, label, tuple(center), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Mostrar posición relativa para tags configurados
            if tag_id in self.robot_tags and data['relative_position'] is not None:
                dist = np.linalg.norm(data['relative_position'][:2])
                cv2.putText(frame, f"Dist: {dist:.2f}m", 
                           (center[0], center[1] + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    def publish_loop(self):
        """Publicación periódica a frecuencia fija"""
        with self.lock:
            current_data = self.latest_detections.copy()
            ref_available = self.reference_pose is not None
        
        if not ref_available:
            return
        
        current_time = self.get_clock().now().to_msg()
        
        # Publicar solo los tags configurados como robots
        for tag_id in self.robot_tags:
            if tag_id in current_data and current_data[tag_id]['relative_position'] is not None:
                data = current_data[tag_id]
                self.publish_robot_pose(
                    tag_id,
                    current_time,
                    data['relative_position'],
                    data['relative_quaternion']
                )

    def tf_publish_loop(self):
        """Publicación TF periódica a frecuencia fija"""
        with self.lock:
            current_data = self.latest_detections.copy()
            ref_available = self.reference_pose is not None
        
        if not ref_available:
            return
        
        current_time = self.get_clock().now().to_msg()
        
        for tag_id in self.robot_tags:
            if tag_id in current_data and current_data[tag_id]['relative_position'] is not None:
                data = current_data[tag_id]
                self.publish_tf(
                    tag_id,
                    current_time,
                    data['relative_position'],
                    data['relative_quaternion']
                )

    def publish_robot_pose(self, tag_id, time, translation, quaternion):
        """Publica la pose del robot"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = time
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.pose.position.x = float(translation[0])
        pose_msg.pose.pose.position.y = float(translation[1])
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]
        
        # Covarianza estimada
        pose_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]
        
        self.robot_publishers[tag_id].publish(pose_msg)

    def publish_tf(self, tag_id, time, translation, quaternion):
        """Publica la transformación TF"""
        tf = TransformStamped()
        tf.header.stamp = time
        tf.header.frame_id = 'world'
        tf.child_frame_id = f'{self.robot_tags[tag_id]}/base_link'
        tf.transform.translation.x = float(translation[0])
        tf.transform.translation.y = float(translation[1])
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = quaternion[0]
        tf.transform.rotation.y = quaternion[1]
        tf.transform.rotation.z = quaternion[2]
        tf.transform.rotation.w = quaternion[3]
        self.tf_broadcaster.sendTransform(tf)

    def get_camera_params(self, image_width):
        """Obtiene parámetros de la cámara ajustados para el reflejo"""
        fx, fy = 800.0, 800.0  # Distancias focales
        cx, cy = image_width/2, 360.0  # Puntos principales
        
        # Ajustar punto principal para reflejo horizontal
        if self.flip_mode == 1:
            cx = image_width - cx
        
        return (fx, fy, cx, cy)

    def cleanup(self):
        """Limpia recursos"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        if self.show_debug:
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado")
    except Exception as e:
        node.get_logger().error(f"Error: {str(e)}")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()