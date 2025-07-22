# wait_for_order.py (Versión simplificada)
import py_trees

class WaitForOrder(py_trees.behaviour.Behaviour):
    def __init__(self, name="WaitForOrder", node=None):
        super().__init__(name)
        self.node = node

        # Declarar puerto de salida
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(
            key="robot_ns",
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        if not self.blackboard.robot_ns:
            self.blackboard.robot_ns = input("Seleccione robot (robot1/robot2): ").strip()
            self.node.get_logger().info(f"Robot seleccionado: {self.blackboard.robot_ns }")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING