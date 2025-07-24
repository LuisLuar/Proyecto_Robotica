# input_coordinates.py
import py_trees

class InputCoordinates(py_trees.behaviour.Behaviour):
    def __init__(self, name="InputCoordinates", node=None, input_method="terminal"):  # Añade parámetro
        super().__init__(name)
        self.node = node
        self.input_method = input_method  # Nuevo atributo
        #self._done = False
 
        # Declarar puerto de salida
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(
            key="goal_coords",
            access=py_trees.common.Access.WRITE
        )

    def initialise(self):
        # Se llama automáticamente antes de cada ejecución
        pass

    def update(self):
        try:
            if self.input_method == "terminal":
                x = float(input("Ingrese coordenada X: "))
                y = float(input("Ingrese coordenada Y: "))
                self.blackboard.goal_coords = [x, y]
                self.node.get_logger().info("Completo")
                return py_trees.common.Status.SUCCESS
            
        except ValueError:
            self.node.get_logger().error("Coordenadas inválidas!")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self._done = False
        # Se llama cuando el estado está por cambiar
        if new_status == py_trees.common.Status.INVALID:
            self.node.get_logger().error("Estado invalido")