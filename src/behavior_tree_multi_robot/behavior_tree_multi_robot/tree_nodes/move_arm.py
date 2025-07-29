# bt_nodes/grasp_object.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
import py_trees

class MoveArmNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, node: Node, q1: int,q2: int,q3: int ,efector: int):
        super().__init__(name)
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(
            key="order_info",
            access=py_trees.common.Access.READ
        )
        self.node = node
        self.namespace = None
        self.cubo_ns = None
        self.done = False
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.efector = efector

        self.msg = Int32MultiArray()
        self.msg.data = [self.q1, self.q2, self.q3, self.efector]  # Ejemplo para agarrar
        #self.node.get_logger().info(f"MSG: {self.msg}")

    def setup(self, **kwargs):
        return True

    def initialise(self):
        self.done = False    

        self.namespace,self.cubo_ns  = self.blackboard.order_info

        self.publisher = self.node.create_publisher(Int32MultiArray, f"/{self.namespace}/request_srv", 10)
        self.subscription = self.node.create_subscription(
            Bool,
            f"/{self.namespace}/response_srv",
            self.listener_callback,
            10)

        self.publisher.publish(self.msg)
        self.node.get_logger().info(f"[{self.namespace}] Enviando comando: agarrar objeto")

    def update(self):
        if self.done:
            self.node.get_logger().info("MoveArm COMPLETE")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def listener_callback(self, msg: Bool):
        if msg.data:
            self.done = True

    def terminate(self, new_status):
        # Resetear estado interno para permitir reutilización
        self._done = False
        return super().terminate(new_status)
