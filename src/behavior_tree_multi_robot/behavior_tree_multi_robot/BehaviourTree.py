from behavior_tree_multi_robot.tree_nodes import (
    WaitForStart,
    WaitForOrder,
    InputCoordinates,
    GoToPosition, 
    DetectObject,
    MoveArmNode
)
import py_trees

#===========================secuencias de navegación===========================
def create_navigation_sequences(node):    
    nav_seq1 = py_trees.composites.Sequence("NavigateToObject", memory=True)
    nav_seq1.add_children([
        InputCoordinates(name="GetCoordinatesObject",node=node,ubication='cubo'),
        GoToPosition(name="NavegateObj",node=node, goal_tolerance=0.15)
        #DetectObject(name="DetectObject",node=node)
    ])

    nav_seq2 = py_trees.composites.Sequence("NavigateToDeposit", memory=True)
    nav_seq2.add_children([
        InputCoordinates(name="GetCoordinatesDeposit",node=node,ubication='deposito'),
        GoToPosition(name="NavegateDep",node=node, goal_tolerance=0.1)
    ])

    nav_seq3 = py_trees.composites.Sequence("NavigateToOrigin", memory=True)
    nav_seq3.add_children([
        InputCoordinates(name="GetCoordinatesOrigin",node=node,ubication='origen'),
        GoToPosition(name="NavegateOrg",node=node, goal_tolerance=0.1) 
    ])
    return nav_seq1, nav_seq2, nav_seq3
#==================================================================================

#===================secuencias de manipulación del brazo======================
def create_arm_sequences(node):
    arm_seq1 = py_trees.composites.Sequence("HoldObject", memory=True)
    arm_seq1.add_children([
        MoveArmNode("Grasp", node=node,q1=95,q2=0,q3=80,efector=1),              
        MoveArmNode("SafePosObj", node=node,q1=95,q2=70,q3=40,efector=1)
    ]) 

    arm_seq2 = py_trees.composites.Sequence("ReleaseObject", memory=True)
    arm_seq2.add_children([
        MoveArmNode("RealeasePos1", node=node,q1=95,q2=70,q3=80,efector=1),
        MoveArmNode("RealeasePos2", node=node,q1=95,q2=0,q3=80,efector=0),
        MoveArmNode("SafePosHome", node=node,q1=180,q2=70,q3=80,efector=0)
    ]) 
    return arm_seq1, arm_seq2
#==================================================================================

#===========================CONSTRUCCIÓN DEL ARBOL PRINCIPAL===========================
def build_tree(node):    
    """Construye y retorna el árbol de comportamiento completo"""
    nav_seq1, nav_seq2, nav_seq3 = create_navigation_sequences(node)
    arm_seq1, arm_seq2 = create_arm_sequences(node)

    main_sequence = py_trees.composites.Sequence("MainSequence", memory=True)
    main_sequence.add_children([
        WaitForStart(name="Start", node=node),
        WaitForOrder(name="SelectRobot", node=node),                      
        nav_seq1,
        arm_seq1,                          
        nav_seq2,                                    
        arm_seq2,                                                                                                       # PosiciónSegura 2
        nav_seq3
    ])
    # Visualizar estructura del árbol (solo una vez)
    py_trees.display.render_dot_tree(main_sequence, name="bt")

    return py_trees.trees.BehaviourTree(main_sequence)
#==================================================================================
