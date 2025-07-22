# input_coordinates.py
import py_trees

class InputCoordinates(py_trees.behaviour.Behaviour):
    def __init__(self, name="InputCoordinates", node=None, input_method="terminal"):  # Añade parámetro
        super().__init__(name)
        self.node = node
        self.input_method = input_method  # Nuevo atributo
        self._done = False
 
        # Declarar puerto de salida
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(
            key="goal_coords",
            access=py_trees.common.Access.WRITE
        )

    def update(self):
        if not self._done:
            try:
                if self.input_method == "terminal":
                    x = float(input("Ingrese coordenada X: "))
                    y = float(input("Ingrese coordenada Y: "))
                    self.blackboard.goal_coords = [x,y]
                # elif self.input_method == "camera":  # Para implementación futura
                #     self.node.goal_coords = self._get_coords_from_camera()
                
                self._done = True
                #self.node.get_logger().info("Completo")
                return py_trees.common.Status.SUCCESS
                
            except ValueError:
                self.node.get_logger().error("Coordenadas inválidas!")
                return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self._done = False