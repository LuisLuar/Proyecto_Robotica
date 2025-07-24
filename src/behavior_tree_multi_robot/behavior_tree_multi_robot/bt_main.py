#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
from py_trees.decorators import Repeat  # Import corregido
from behavior_tree_multi_robot.tree_nodes.wait_for_order import WaitForOrder
from behavior_tree_multi_robot.tree_nodes.input_coordinates import InputCoordinates
from behavior_tree_multi_robot.tree_nodes.go_to_position import GoToPosition
from behavior_tree_multi_robot.tree_nodes.detect_object import DetectObject
from behavior_tree_multi_robot.tree_nodes.move_arm import MoveArmNode

class BTMainNode(Node):
    def __init__(self):
        super().__init__('bt_main_node')

def build_tree(node):
     #===========================CONSTRUCCIÓN DE SUBRAMAS===========================
    nav_seq1 = py_trees.composites.Sequence("NavigationToObject", memory=True)
    nav_seq1.add_children([
        InputCoordinates(name="ObtenerCoordenadas1",node=node,input_method="terminal"),
        GoToPosition(name="NavegarHaciaDestino1",node=node),
        DetectObject(name="DetectarObjeto",node=node)
    ])

    nav_seq2 = py_trees.composites.Sequence("NavigationToDeposit", memory=True)
    nav_seq2.add_children([
        InputCoordinates(name="ObtenerCoordenadas2",node=node,input_method="terminal"),
        GoToPosition(name="NavegarHaciaDestino2",node=node)
    ])

    nav_seq3 = py_trees.composites.Sequence("NavigationOrigin", memory=True)
    nav_seq3.add_children([
        InputCoordinates(name="ObtenerCoordenadas3",node=node,input_method="terminal"),
        GoToPosition(name="NavegarHaciaDestino3",node=node) 
    ])
    #==================================================================================

    #===========================CONSTRUCCIÓN DEL ARBOL PRINCIPAL===========================
    # 2. Construir árbol
        # Construcción exacta del árbol solicitado
    main_sequence = py_trees.composites.Sequence("MainSequence", memory=True)
    main_sequence.add_children([
        WaitForOrder(name="SeleccionarRobot", node=node),                          
        nav_seq1,
        #MoveArmNode("Grasp", node=node,q1=95,q2=0,q3=80,efector=1),              
        #MoveArmNode("SafePos", node=node,q1=95,q2=70,q3=40,efector=1),                          
        nav_seq2,                                    
        #MoveArmNode("RealeasePos", node=node,q1=95,q2=0,q3=80,efector=0),
        #MoveArmNode("SafePos2", node=node,q1=180,q2=70,q3=80,efector=0),                                                                                                       # PosiciónSegura 2
        nav_seq3
    ])

    
     # Decorador para repetir 2 veces el ciclo
    #root = Repeat(name="RepeatRoot", child= main_sequence , num_success=-1)
 
    behaviour_tree = py_trees.trees.BehaviourTree(main_sequence)
    behaviour_tree.setup(timeout=15)

    # Visualizar estructura del árbol (solo una vez)
    #py_trees.display.render_dot_tree(main_sequence, name="bt")

    return behaviour_tree

def main(args=None):
    rclpy.init(args=args)
    node = BTMainNode()

    # No crees Blackboard, solo configura claves globales una vez
    py_trees.blackboard.Blackboard.set("robot_ns", None)
    py_trees.blackboard.Blackboard.set("goal_coords", None)

    
    # 3. Bucle principal
    try:
        while rclpy.ok():
             # Construir el árbol
            behaviour_tree = build_tree(node)
            behaviour_tree.setup(timeout=15)
            
            # Ejecutar el árbol hasta completar un ciclo
            while rclpy.ok():
                behaviour_tree.tick()
                rclpy.spin_once(node, timeout_sec=0.1)
                
                # Verificar si el ciclo se completó
                if behaviour_tree.root.status == py_trees.common.Status.SUCCESS:
                    node.get_logger().info("Ciclo completado con éxito. Reiniciando...")
                    break
                elif behaviour_tree.root.status == py_trees.common.Status.INVALID:
                    node.get_logger().error("Estado inválido detectado. Reiniciando árbol...")
                    break
            
            # Limpiar antes de reiniciar
            behaviour_tree.interrupt()
            
            # Resetear el Blackboard
            py_trees.blackboard.Blackboard.set("robot_ns", None)
            py_trees.blackboard.Blackboard.set("goal_coords", None)
            
    except KeyboardInterrupt:
        node.get_logger().info("Ejecución finalizada")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()


"""if final_node.status == py_trees.common.Status.SUCCESS:
        
        # Resetear blackboard y estados
        py_trees.blackboard.Blackboard.set("robot_ns", None)
        py_trees.blackboard.Blackboard.set("goal_coords", None)
        # Resetear la secuencia principal
        #main_sequence.stop(py_trees.common.Status.INVALID)
        final_node.stop(py_trees.common.Status.SUCCESS)"""
