# bt_nodes/move_to_safe_position.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
import py_trees

class MoveArmToSafePosition(py_trees.behaviour.Behaviour):
    def __init__(self, name, node: Node):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(
            key="robot_ns",
            access=py_trees.common.Access.READ
        )
        self.node = node
        self.namespace = self.blackboard.robot_ns
        self.done = False

    def setup(self, **kwargs):
        return True

    def initialise(self):
        self.done = False
        msg = Int32MultiArray()
        msg.data = [0, 80, 40, 1]  # Ejemplo para agarrar

        self.namespace = self.blackboard.robot_ns

        self.publisher = self.node.create_publisher(Int32MultiArray, f"/{self.namespace}/request_srv", 10)
        self.subscription = self.node.create_subscription(
            Bool,
            f"/{self.namespace}/response_srv",
            self.listener_callback,
            10)

        self.publisher.publish(msg)
        self.node.get_logger().info(f"[{self.namespace}] Enviando comando: agarrar objeto")

    def update(self):
        if self.done:
            self.node.get_logger().info("Objeto agarrado")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def listener_callback(self, msg: Bool):
        if msg.data:
            self.done = True

    def terminate(self, new_status):
        # Resetear estado interno para permitir reutilización
        self._done = False
        #return super().terminate(new_status)

