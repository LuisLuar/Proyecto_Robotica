#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Range
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import serial
import time
from threading import Lock

class BluetoothBridge(Node):
    def __init__(self):
        super().__init__('bluetooth_bridge')
        
        # Parámetros configurables
        self.declare_parameter('device', '/dev/rfcomm0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('reconnect_interval', 5.0)
        
        # Variables de estado
        self.serial_lock = Lock()
        self.last_connect_time = 0.0
        self.connected = False
        self.serial_buffer = ""
        
        # Configuración QoS
        self.qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Inicialización del puerto serial
        self.init_serial()
        
        # Publishers
        self.pub_odom = self.create_publisher(Odometry, 'odom/unfiltered', self.qos_profile)
        self.pub_imu = self.create_publisher(Imu, 'imu/unfiltered', self.qos_profile)
        self.pub_range = self.create_publisher(Range, 'range/unfiltered', self.qos_profile)
        
        # Subscriber para cmd_vel
        self.sub_cmd = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_callback, 
            self.qos_profile
        )
        
        # Timers
        self.read_timer = self.create_timer(0.01, self.read_serial)  # 100Hz
        self.check_timer = self.create_timer(1.0, self.check_connection)
        
        self.get_logger().info("Nodo Bluetooth Bridge inicializado")

    def init_serial(self):
        """Inicializa o reinicia la conexión serial"""
        try:
            with self.serial_lock:
                if hasattr(self, 'ser') and self.ser:
                    self.ser.close()
                
                device = self.get_parameter('device').value
                baudrate = self.get_parameter('baudrate').value
                
                self.ser = serial.Serial(
                    port=device,
                    baudrate=baudrate,
                    timeout=0.5,
                    write_timeout=1.0
                )
                
                time.sleep(0.5)  # Pausa para estabilización
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                
                self.connected = True
                self.last_connect_time = self.get_clock().now().seconds_nanoseconds()[0]
                self.get_logger().info(f"Conexión establecida con {device} a {baudrate} baudios")
                
        except Exception as e:
            self.connected = False
            self.get_logger().error(f"Error al conectar: {str(e)}", throttle_duration_sec=10)

    def check_connection(self):
        """Verifica y mantiene la conexión"""
        if not self.connected:
            reconnect_interval = self.get_parameter('reconnect_interval').value
            current_time = self.get_clock().now().seconds_nanoseconds()[0]
            
            if current_time - self.last_connect_time > reconnect_interval:
                self.get_logger().info("Intentando reconectar...")
                self.init_serial()

    def read_serial(self):
        """Lee datos seriales y procesa mensajes"""
        if not self.connected:
            return
            
        try:
            with self.serial_lock:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line:
                        self.process_message(line)
                
        except serial.SerialException as e:
            self.get_logger().error(f"Error serial: {str(e)}")
            self.connected = False
        except Exception as e:
            self.get_logger().error(f"Error inesperado: {str(e)}", throttle_duration_sec=5)

    def process_message(self, message):
        """Procesa un mensaje completo recibido por serial"""
        try:
            if message.startswith("type:odom"):
                self.process_odometry(message)
            elif message.startswith("type:imu"):
                self.process_imu(message)
            elif message.startswith("type:range"):
                self.process_range(message)
            else:
                self.get_logger().warn(f"Mensaje no reconocido: {message[:60]}...")
                
        except Exception as e:
            self.get_logger().warn(f"Error procesando mensaje: {str(e)}")

    def process_odometry(self, data):
        """Procesa datos de odometría en formato type:odom,x:1.23,y:2.34,..."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        
        try:
            # Extraer valores del string
            parts = data.split(',')
            x = float(parts[1].split(':')[1])
            y = float(parts[2].split(':')[1])
            yaw = float(parts[3].split(':')[1])
            v = float(parts[4].split(':')[1])
            w = float(parts[5].split(':')[1])
            
            # Posición
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            
            # Orientación (yaw)
            q = quaternion_from_euler(0, 0, yaw)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]
            
            # Velocidades
            msg.twist.twist.linear.x = v
            msg.twist.twist.angular.z = w
            
            # Covarianzas (valores estimados, ajustar según robot)
            msg.pose.covariance[0] = 0.1  # x
            msg.pose.covariance[7] = 0.1  # y
            msg.pose.covariance[35] = 0.1  # yaw
            
            msg.twist.covariance[0] = 0.1  # vx
            msg.twist.covariance[35] = 0.1  # wz
            
            self.pub_odom.publish(msg)
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Error odometría: {str(e)}")

    def process_imu(self, data):
        """Procesa datos del IMU en formato type:imu,ax:0.12,ay:0.05,..."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        try:
            # Extraer valores del string
            parts = data.split(',')
            ax = float(parts[1].split(':')[1])
            ay = float(parts[2].split(':')[1])
            az = float(parts[3].split(':')[1])
            gx = float(parts[4].split(':')[1])
            gy = float(parts[5].split(':')[1])
            gz = float(parts[6].split(':')[1])
            roll = float(parts[7].split(':')[1])
            pitch = float(parts[8].split(':')[1])
            yaw = float(parts[9].split(':')[1])
            
            # Orientación
            q = quaternion_from_euler(roll, pitch, yaw)
            msg.orientation.x = q[0]
            msg.orientation.y = q[1]
            msg.orientation.z = q[2]
            msg.orientation.w = q[3]
            
            # Velocidad angular
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
            
            # Aceleración lineal
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
            
            # Covarianzas (valores estimados, ajustar según IMU)
            msg.orientation_covariance[0] = 0.01
            msg.orientation_covariance[4] = 0.01
            msg.orientation_covariance[8] = 0.01
            
            msg.angular_velocity_covariance[0] = 0.02
            msg.angular_velocity_covariance[4] = 0.02
            msg.angular_velocity_covariance[8] = 0.02
            
            msg.linear_acceleration_covariance[0] = 0.04
            msg.linear_acceleration_covariance[4] = 0.04
            msg.linear_acceleration_covariance[8] = 0.04
            
            self.pub_imu.publish(msg)
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Error IMU: {str(e)}")

    def process_range(self, data):
        """Procesa datos de sensor de distancia en formato type:range,range:1.23"""
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "range_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 grados
        msg.min_range = 0.02  # 2cm
        msg.max_range = 4.0   # 4m
        
        try:
            msg.range = float(data.split(':')[2])
            self.pub_range.publish(msg)
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Error rango: {str(e)}")

    def cmd_callback(self, msg):
        """Envía comandos de velocidad al robot en formato v:0.5,w:0.1"""
        if not self.connected:
            return
            
        cmd = f"v:{msg.linear.x:.2f},w:{msg.angular.z:.2f}\n"
        
        try:
            with self.serial_lock:
                self.ser.write(cmd.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Error enviando comando: {str(e)}")
            self.connected = False

    def __del__(self):
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()