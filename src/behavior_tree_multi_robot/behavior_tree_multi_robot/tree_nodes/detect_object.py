# detect_object.py
import py_trees
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class DetectObject(py_trees.behaviour.Behaviour):
    def __init__(self, name="DetectObject", node=None):
        super().__init__(name)
        # Declarar puertos de entrada
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(
            key="robot_ns",
            access=py_trees.common.Access.READ
        )
        self.node = node
        self.namespace = self.blackboard.robot_ns
        self.distance_threshold = 0.05
        self.detected = False
        self.latest_range = None

        


    def range_callback(self, msg):
        self.latest_range = msg.range
        if msg.range < self.distance_threshold and msg.range >= 0.02:
            self.detected = True
        else:
            self.detected = False

    def initialise(self):
        self.detected = False
        self.latest_range = None

        self.namespace = self.blackboard.robot_ns
        self.node.get_logger().info(f"Robot seleccionado: {self.namespace}")

        self.sub = self.node.create_subscription(
            Range,
            f'/{self.namespace}/range/unfiltered',
            self.range_callback,
            qos_profile_sensor_data
        )

        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.namespace}/cmd_vel',
            10
        )

    def update(self):
        if self.detected:
            self.cmd_vel_pub.publish(Twist())  # Detener
            self.node.get_logger().info("✅ Objeto detectado. Deteniendo.")
            return py_trees.common.Status.SUCCESS
        else:
            # Girar para buscar objeto
            cmd = Twist()
            cmd.angular.z = 0.4  # velocidad angular constante
            self.cmd_vel_pub.publish(cmd)
            self.node.get_logger().info("🔍 Buscando objeto...")
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Detener al terminar la ejecución (ya sea éxito o no)
        self.cmd_vel_pub.publish(Twist())
