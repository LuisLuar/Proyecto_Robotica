import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from pupil_apriltags import Detector
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from msg_nuevos.msg import AprilTagWorld, AprilTagWorldArray, PosRelativa

class AprilTagStaticTFPublisher(Node):
    def __init__(self):
        super().__init__('april_tag_static_tf_publisher')
        
        # Configuración de tags para robots y cubos
        self.tag_config = {
            # Robots - Colores distintivos y contrastantes
            9: ("robot1", (0, 120, 255)),    # Azul brillante
            460: ("robot2", (255, 50, 50)),    # Rojo vivo
            
            # Cubos - Tonos verdes variados
            459: ("cubo1", (0, 200, 0)),       # Verde intenso
            4: ("cubo2", (100, 255, 50)),    # Verde lima
            456: ("cubo3", (0, 180, 120)),   # Verde esmeralda
            457: ("cubo4", (50, 150, 50)),   # Verde bosque
            458: ("cubo5", (150, 255, 150)), # Verde claro
            
            # Depósito - Color distintivo
            1: ("deposito", (255, 165, 0)), # Naranja
            
            # Zonas de origen - Tonos morados/púrpura
            3: ("robot1_origen", (180, 0, 180)), # Púrpura
            461: ("robot2_origen", (130, 0, 130))  # Púrpura oscuro
        }

        # Calibración de cámara
        calibration_file = 'src/robot_movil/Camara_robot/parametros_calibracion.npz'
        with np.load(calibration_file) as X:
            self.camera_matrix, self.dist_coeffs = X['mtx'], X['dist']
        
        self.tag_size = 0.075
        self.detector = Detector(families='tag36h11')
        self.cap = cv2.VideoCapture(0)
        
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.published_transforms = {}
        
        self.tag_pub = self.create_publisher(AprilTagWorldArray, 'apriltag_world_array', 10)
        self.tag_pub_rel = self.create_publisher(PosRelativa, 'pos_relativa', 10)

        #self.window_name = 'Detección de Tags'
        self.image_pub = self.create_publisher(Image, 'tag_image', 10)
        self.bridge = CvBridge()

        #cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        #cv2.resizeWindow(self.window_name, 800, 600)
        
        self.timer = self.create_timer(0.1, self.detect_tags)
        self.timer_rel = self.create_timer(0.1, self.posicion_relativa)
        self.get_logger().info("Inicializando detección de AprilTags...")

        # Agregar a mensaje de salida
        self.pos_rel_msg = PosRelativa()

    def posicion_relativa(self):
        self.tag_pub_rel.publish(self.pos_rel_msg)

    def detect_tags(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No se pudo leer el frame de la cámara.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self.camera_matrix[0,0], self.camera_matrix[1,1],
                           self.camera_matrix[0,2], self.camera_matrix[1,2]),
            tag_size=self.tag_size
        )

        msg_array = AprilTagWorldArray()

        for det in detections:
            tag_id = det.tag_id
            if tag_id not in self.tag_config:
                continue

            #===============================================================
             # 1. Invertir coordenada Y en la traslación
            det.pose_t[1][0] = -det.pose_t[1][0]  # Inversión del eje Y

            # 2. Calcular rotación corregida
            # Extraer ángulos de Euler (roll, pitch, yaw) desde la matriz de rotación
            euler = tf_transformations.euler_from_matrix(det.pose_R, 'sxyz')
            roll, pitch, yaw = euler
            corrected_yaw = -yaw

             # Convertir a cuaternión corregido
            quat_corrected = tf_transformations.quaternion_from_euler(
                roll,
                pitch,
                corrected_yaw,
                'sxyz'
            )

            # Actualizar la matriz de rotación (opcional, solo si otros componentes la necesitan)
            det.pose_R = tf_transformations.quaternion_matrix(quat_corrected)[:3, :3]
            #=========================================================================

            nombre, color = self.tag_config[tag_id]
            b, g, r = color

            corners = det.corners.astype(int)
            cv2.polylines(frame, [corners], isClosed=True, color=color, thickness=2)
            center = (int(det.center[0]), int(det.center[1]))
            cv2.circle(frame, center, 5, color, -1)
            text_pos = (corners[0][0], corners[0][1] - 10)
            cv2.putText(frame, nombre, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Agregar a mensaje de salida
            tag_msg = AprilTagWorld()
            tag_msg.id = tag_id
            tag_msg.nombre = nombre
            tag_msg.posx = float(det.pose_t[0][0])
            tag_msg.posy = float(det.pose_t[1][0])            
            tag_msg.r = r
            tag_msg.g = g
            tag_msg.b = b

            msg_array.tags.append(tag_msg)

            # Solo para robots: publicar transformada estática una vez
            if "robot" in nombre and nombre not in self.published_transforms:
                T = np.eye(4)
                T[:3, :3] = det.pose_R
                T[:3, 3] = det.pose_t.ravel()

                x = float(T[0, 3])
                y = float(T[1, 3])
                z = 0.0
                yaw = math.atan2(T[1, 0], T[0, 0])
                quat = tf_transformations.quaternion_from_euler(0, 0, yaw)

                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'world'
                tf_msg.child_frame_id = f'{nombre}/camara_link'

                tf_msg.transform.translation.x = x
                tf_msg.transform.translation.y = y
                tf_msg.transform.translation.z = z
                tf_msg.transform.rotation.x = quat[0]
                tf_msg.transform.rotation.y = quat[1]
                tf_msg.transform.rotation.z = quat[2]
                tf_msg.transform.rotation.w = quat[3]

                self.static_broadcaster.sendTransform(tf_msg)
                self.published_transforms[nombre] = True

                if nombre == "robot1":
                    self.pos_rel_msg.posx1 = x
                    self.pos_rel_msg.posy1 = y
                    self.pos_rel_msg.yaw1 = yaw
                
                if nombre == "robot2":
                    self.pos_rel_msg.posx2 = x
                    self.pos_rel_msg.posy2 = y
                    self.pos_rel_msg.yaw2 = yaw

                self.get_logger().info(f"TF estática publicada para {nombre}")
                self.get_logger().info(f"X: {x}")
                self.get_logger().info(f"Y: {y}")

        self.tag_pub.publish(msg_array)

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(image_msg)

        """cv2.imshow(self.window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()
            self.cap.release()
            cv2.destroyAllWindows()"""

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagStaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
