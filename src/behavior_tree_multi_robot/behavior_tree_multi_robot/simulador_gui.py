#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msg_nuevos.srv import ElegirRobot

class SimuladorGUI(Node):
    def __init__(self):
        super().__init__('simulador_gui')
        self.srv = self.create_service(ElegirRobot, 'elegir_robot', self.callback)

    def callback(self, request, response):
        ns = input("Selecciona el robot [robot1 / robot2]: ")
        response.robot_namespace = ns
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SimuladorGUI()
    rclpy.spin(node)
    rclpy.shutdown()
