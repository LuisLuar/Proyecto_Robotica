# input_coordinates.py
import py_trees
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from msg_nuevos.msg import AprilTagWorldArray, PosRelativa  # Ajusta el import según tu paquete
import tf_transformations
import math

class InputCoordinates(py_trees.behaviour.Behaviour):
    def __init__(self, name="InputCoordinates", node=None,ubication=None):  # Añade parámetro
        super().__init__(name)
        self.node = node
 
        # Declarar puerto de salida
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key="order_info",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_coords",access=py_trees.common.Access.WRITE)
        self.namespace = None
        self.cubo_ns = None
        self.ubication = ubication

        self.tx = None
        self.ty = None
        self.yaw = None
        self.done_init = False
        self.done_callback = False

        #Parametros de corrección
        self.x1 = 0.05
        self.y1 = -0.05

         # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

        # Variable para almacenar datos del callback
        self.tags_dict = {}  # {tag_id: (x, y)}

        self.node.create_subscription(
            AprilTagWorldArray,
            '/apriltag_world_array',
            self.callback_tags,
            10
        )

        self.node.create_subscription(
            PosRelativa,
            '/pos_relativa',
            self.pos_rel_callaback,
            10
        )
        

    def initialise(self):
        # Se llama automáticamente antes de cada ejecución
        self.namespace,self.cubo_ns = self.blackboard.order_info

        self.done_init = True
        pass

    def pos_rel_callaback(self, msg):
        if(self.namespace=='robot1' and self.done_init):
            self.tx = msg.posx1
            self.ty = msg.posy1
            self.yaw = msg.yaw1
            self.done_callback = True

        if(self.namespace=='robot2' and self.done_init):
            self.tx = msg.posx2
            self.ty = msg.posy2
            self.yaw = msg.yaw2
            self.done_callback = True

    def callback_tags(self, msg):
        # Almacena las coordenadas absolutas de cada tag
        for tag in msg.tags:
            self.tags_dict[tag.nombre] = (tag.posx, tag.posy)

    def update(self):        
        try:            
             # Verificar condiciones necesarias (todas deben ser True)
            if not (self.done_init and 
                    self.done_callback and 
                    self.namespace in self.tags_dict and 
                    self.cubo_ns in self.tags_dict and
                    self.tx is not None and
                    self.ty is not None and
                    self.yaw is not None):
                self.node.get_logger().warn("Esperando datos iniciales...", throttle_duration_sec=1)
                return py_trees.common.Status.RUNNING

            
            if(self.ubication=='deposito'):
                self.cubo_ns = self.ubication
            elif(self.ubication=='origen'):
                self.cubo_ns = f'{self.namespace}_origen'
            #pos_robot = self.tags_dict[self.namespace]  # (x, y)
            pos_cubo = self.tags_dict[self.cubo_ns]     # (x, y)

            cubo_x, cubo_y = pos_cubo

            # Obtener la transformada de world -> robotX/odom
            try:
                #self.node.get_logger().info(f"Posición tx:{self.tx}, ty:{self.ty}")

                # Transformar punto del cubo a frame del robot (odom)
                dx = cubo_x - self.tx
                dy = cubo_y - self.ty

                rel_x = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy - self.x1
                rel_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy - self.y1

                # Guardar coordenadas relativas
                self.blackboard.goal_coords = [rel_x, rel_y]
                self.node.get_logger().info(f"Destino relativo al robot: X={rel_x:.2f}, Y={rel_y:.2f}")
                return py_trees.common.Status.SUCCESS
        
            except Exception as e:
                self.node.get_logger().error(f"Error al obtener transformada TF: {e}")
                return py_trees.common.Status.RUNNING
            
        except ValueError:
            self.node.get_logger().error("Coordenadas inválidas!")
            return py_trees.common.Status.FAILURE


    def terminate(self, new_status):
        # Se llama cuando el estado está por cambiar
        self.done_init = False
        self.done_callback = False
        if new_status == py_trees.common.Status.INVALID:
            self.node.get_logger().error("Estado invalido")