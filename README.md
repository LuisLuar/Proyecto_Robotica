Proyecto basado en:
https://github.com/amalshaji4540/diffdrive_ws.git

Comandos importantes:

--------para coneccion wifi---------------------
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

------------para conección bluetooth--------------------
ros2 run robot_movil puentebt

Ejecución simulación:
ros2 launch diffdrive_description display.launch.xml

Ejecucion trayectoria y registro de datos:
ros2 launch robot_movil car.launch.py

