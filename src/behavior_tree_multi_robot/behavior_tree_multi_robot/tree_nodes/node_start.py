import py_trees
import rclpy
from std_msgs.msg import Bool

class WaitForStart(py_trees.behaviour.Behaviour):
    def __init__(self, name="Start", node=None):
        """
        Nodo que espera señal de inicio desde GUI
        @param node: Nodo ROS2 para crear suscripción
        """
        super().__init__(name)
        self.node = node
        self.start_received = False
        
        # Suscriptor al tópico del botón
        self.subscription = self.node.create_subscription(
            Bool,
            '/boton_start',
            self.start_callback,
            10
        )
        self.node.get_logger().info(f"WaitForStart: Suscrito a /boton_start")

    def start_callback(self, msg):
        """Callback para mensajes del botón"""
        self.start_received = msg.data
        if self.start_received:
            self.node.get_logger().info("WaitForStart: Señal START recibida")

    def initialise(self):
        """Reinicia estado al comenzar"""
        self.start_received = False
        self.node.get_logger().debug("WaitForStart: Reiniciando...")

    def update(self):
        """
        Lógica principal - Retorna RUNNING/SUCCESS
        """
        if self.start_received:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Limpieza al finalizar"""
        self.node.get_logger().debug(f"WaitForStart: Terminando con estado {new_status}")