# wait_for_order.py

import py_trees
from rclpy.task import Future
from msg_nuevos.srv import ElegirRobot

class WaitForOrder(py_trees.behaviour.Behaviour):
    def __init__(self, name="WaitForOrder", node=None):
        super().__init__(name)
        self.node = node  # Nodo rclpy que maneja ROS
        self.robot_selected = None
        self.future = None
        self.client = None
        self.sent_request = False

        # Declarar puerto de salida
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(
            key="robot_ns",
            access=py_trees.common.Access.WRITE
        )


    def initialise(self):
        # Solo se llama una vez al empezar
        self.client = self.node.create_client(ElegirRobot, 'elegir_robot')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Esperando servicio elegir_robot...')
        self.sent_request = False
        self.future = None

    def update(self):
        if not self.sent_request:
            request = ElegirRobot.Request()
            self.future = self.client.call_async(request)
            self.sent_request = True
            return py_trees.common.Status.RUNNING

        if self.future.done():
            try:
                response = self.future.result()
                self.robot_selected = response.robot_namespace
                self.node.get_logger().info(f"Robot seleccionado: {self.robot_selected}")
                self.blackboard.robot_ns = self.robot_selected
                return py_trees.common.Status.SUCCESS
            except Exception as e:
                self.node.get_logger().error(f"Error al obtener respuesta: {e}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING
