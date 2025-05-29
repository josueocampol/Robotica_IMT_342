import rclpy
from rclpy.node import Node

class MyMinimalNode(Node):

    def __init__(self):
        super().__init__('my_first_node') # Nombre de tu nodo
        self.get_logger().info('Nodo MyMinimalNode iniciado!')

def main(args=None):
    rclpy.init(args=args) # Inicializa la comunicación ROS2

    my_node = MyMinimalNode() # Crea una instancia de tu nodo

    rclpy.spin(my_node) # Mantén el nodo en ejecución hasta que sea detenido (Ctrl+C)

    my_node.destroy_node() # Limpia el nodo
    rclpy.shutdown() # Apaga la comunicación ROS2

if __name__ == '__main__':
    main()
