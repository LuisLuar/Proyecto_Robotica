#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
from py_trees.decorators import Repeat  # Import corregido
from behavior_tree_multi_robot.tree_nodes.wait_for_order import WaitForOrder
from behavior_tree_multi_robot.tree_nodes.input_coordinates import InputCoordinates
from behavior_tree_multi_robot.tree_nodes.go_to_position import GoToPosition
from behavior_tree_multi_robot.tree_nodes.detect_object import DetectObject
from behavior_tree_multi_robot.tree_nodes.grasp_object import GraspObjectNode
from behavior_tree_multi_robot.tree_nodes.move_to_safe_position import MoveArmToSafePosition
from behavior_tree_multi_robot.tree_nodes.release_object import ReleaseObjectNode

class BTMainNode(Node):
    def __init__(self):
        super().__init__('bt_main_node')
        
def main(args=None):
    rclpy.init(args=args)
    node = BTMainNode()

    # No crees Blackboard, solo configura claves globales una vez
    py_trees.blackboard.Blackboard.set("robot_ns", None)
    py_trees.blackboard.Blackboard.set("goal_coords", None)

    # 2. Construir árbol
        # Construcción exacta del árbol solicitado
    main_sequence = py_trees.composites.Sequence("MainSequence", memory=True)
    main_sequence.add_children([
        WaitForOrder(name="SeleccionarRobot", node=node),                          # SeleccionarRobot
        InputCoordinates(name="ObtenerCoordenadas",node=node,input_method="terminal"),                         # ObtenerCoordenadas 1
        GoToPosition(name="NavegarHaciaDestino",node=node),                          # Navegar 1
        DetectObject(name="DetectarObjeto",node=node),                        # DetectarObjeto
        GraspObjectNode("Grasp", node=node),                         # Agarrar
        MoveArmToSafePosition("SafePos", node=node),                          # PosiciónSegura 1
        InputCoordinates(name="ObtenerCoordenadas",node=node,input_method="terminal") ,         # ObtenerCoordenadas 2 
        GoToPosition(name="NavegarHaciaDestino",node=node),                                     # Navegar 2
        ReleaseObjectNode("Release", node=node),                                                                           # Soltar
        MoveArmToSafePosition("SafePos", node=node),                                            # PosiciónSegura 2
        InputCoordinates(name="ObtenerCoordenadas",node=node,input_method="terminal") ,         # ObtenerCoordenadas 3
        GoToPosition(name="NavegarHaciaDestino",node=node)                                      # Navegar 3
    ])

    
     # Decorador para repetir 2 veces el ciclo
    root = Repeat(name="RepeatRoot", child= main_sequence , num_success=-1)

    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.setup(timeout=15)

    # Visualizar estructura del árbol (solo una vez)
    py_trees.display.render_dot_tree(root, name="bt")

    # 3. Bucle principal
    try:
        while rclpy.ok():

            #node.get_logger().info(f"Tick: {behaviour_tree.root.status}")
            behaviour_tree.tick() # Ejecutar un paso
                        
            """if behaviour_tree.root.status == py_trees.common.Status.RUNNING:
                
                # Resetear blackboard y estados
                py_trees.blackboard.Blackboard.set("robot_ns", None)
                py_trees.blackboard.Blackboard.set("goal_coords", None)
                # Resetear la secuencia principal
                main_sequence.stop(py_trees.common.Status.INVALID)"""
            
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        node.get_logger().info("Ejecución finalizada")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()