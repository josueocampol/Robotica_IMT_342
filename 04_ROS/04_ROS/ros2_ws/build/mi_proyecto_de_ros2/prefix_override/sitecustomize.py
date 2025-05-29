import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/josue/Documentos/JOSUE/04_ROS/ros2_ws/install/mi_proyecto_de_ros2'
