#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msg_nuevos.srv import ElegirRobot
import threading
import tkinter as tk

class SimuladorGUI(Node):
    def __init__(self):
        super().__init__('simulador_gui')

        # Crear servicio
        self.srv = self.create_service(ElegirRobot, 'elegir_robot', self.callback)

        # Variables de sincronización
        self.robot_seleccionado = None
        self.esperando_respuesta = threading.Event()

        # Lanzar GUI en hilo separado
        gui_thread = threading.Thread(target=self.crear_gui)
        gui_thread.daemon = True
        gui_thread.start()

    def crear_gui(self):
        self.root = tk.Tk()
        self.root.title("Selector de Robot")

        self.label = tk.Label(self.root, text="Esperando solicitud...", font=("Arial", 14))
        self.label.pack(pady=10)

        self.btn_robot1 = tk.Button(self.root, text="Robot 1", width=20, command=lambda: self.seleccionar_robot("robot1"), state=tk.DISABLED)
        self.btn_robot1.pack(pady=5)

        self.btn_robot2 = tk.Button(self.root, text="Robot 2", width=20, command=lambda: self.seleccionar_robot("robot2"), state=tk.DISABLED)
        self.btn_robot2.pack(pady=5)

        self.root.mainloop()

    def seleccionar_robot(self, robot_ns):
        self.robot_seleccionado = robot_ns
        self.label.config(text=f"Robot seleccionado: {robot_ns}")
        self.btn_robot1.config(state=tk.DISABLED)
        self.btn_robot2.config(state=tk.DISABLED)
        self.esperando_respuesta.set()

    def callback(self, request, response):
        self.get_logger().info("Solicitud recibida: elegir robot")
        
        # Habilitar botones
        self.robot_seleccionado = None
        self.esperando_respuesta.clear()
        self.root.after(0, lambda: self.label.config(text="Selecciona el robot:"))
        self.root.after(0, lambda: self.btn_robot1.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.btn_robot2.config(state=tk.NORMAL))

        # Esperar hasta que se seleccione un robot
        self.esperando_respuesta.wait()

        response.robot_namespace = self.robot_seleccionado
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SimuladorGUI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
