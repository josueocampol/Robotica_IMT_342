import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# --- Configuración de Factores de Movimiento ---
# Estos son solo factores de dirección y rotación,
# se multiplicarán por las velocidades actuales (self.linear_velocity, self.angular_velocity)
key_bindings = {
    'w': (1.0, 0.0),   # Adelante (factor 1.0 en lineal, 0 en angular)
    's': (-1.0, 0.0),  # Atrás (factor -1.0 en lineal)
    'a': (0.0, 1.0),   # Girar Izquierda (factor 1.0 en angular)
    'd': (0.0, -1.0),  # Girar Derecha (factor -1.0 en angular)
}
# --------------------------------------------

# --- Función para leer el teclado sin bloquear (Configuración de terminal) ---
# Esta función auxiliar permanece fuera de la clase principal del nodo
def get_key(settings, timeout=0.1):
    # Guarda la configuración actual de la terminal
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        # Configura la terminal en modo "raw" (lee cada tecla al instante) y sin "echo" (no muestra la tecla en la terminal)
        tty.setraw(sys.stdin.fileno())
        # Usa select para esperar entrada por un corto tiempo (timeout)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1) # Lee un solo carácter
        else:
            key = '' # No se presionó ninguna tecla en el timeout
    finally:
        # Restaura la configuración original de la terminal al salir o si hay un error
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key
# ----------------------------------------------------------------------------


class MyTeleopNode(Node):

    def __init__(self):
        super().__init__('my_teleop_node') # Nombre único de tu nodo

        # Crea un publicador para enviar mensajes Twist al tópico de comando de velocidad de la tortuga
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.get_logger().info('Nodo MyTeleopNode iniciado.')
        self.get_logger().info('Controla la tortuga con WASD.')
        self.get_logger().info('Presiona Q para salir.')
        self.get_logger().info('Usa los números 1-9 para ajustar la velocidad base.')
        self.get_logger().info('---------------------------')

        # --- Velocidades ---
        # Definimos las velocidades máximas posibles para escalar los números 1-9
        # Puedes ajustar estos máximos si la tortuga es muy rápida o lenta en el nivel 9
        self.MAX_LIN_VEL = 2.5 # Velocidad lineal máxima al presionar '9'
        self.MAX_ANG_VEL = 3.5 # Velocidad angular máxima al presionar '9'

        # Velocidades actuales (se ajustan con los números 1-9).
        # Iniciamos con una velocidad base media, por ejemplo, equivalente a presionar '5'
        initial_speed_factor = 5 / 9.0
        self.linear_velocity = initial_speed_factor * self.MAX_LIN_VEL
        self.angular_velocity = initial_speed_factor * self.MAX_ANG_VEL

        self.get_logger().info(f'Velocidad base actual (Escala 5/9): Lineal={self.linear_velocity:.2f}, Angular={self.angular_velocity:.2f}')
        # -------------------


        # Guarda la configuración original de la terminal para poder restaurarla
        # Esto es importante para que tu terminal vuelva a funcionar normalmente al cerrar el nodo
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Presiona las teclas de control (WASD, Q, 1-9)...')

    # --- MÉTODO DE LA CLASE para enviar el comando Twist ---
    def send_command(self, linear_x, angular_z):
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.publisher_.publish(twist_msg)
        # self.get_logger().info(f'Sent: Linear={linear_x:.2f}, Angular={angular_z:.2f}') # Opcional: imprimir comando enviado


    def run(self):
        """
        Función principal para ejecutar el bucle de teleoperación.
        """
        # Mantenemos un registro del último comando de movimiento enviado
        # para saber si necesitamos enviar un comando de parada (0,0)
        last_sent_linear = 0.0
        last_sent_angular = 0.0


        try:
            # Envía un comando de parada inicial para asegurar que la tortuga no se mueva al iniciar
            self.send_command(0.0, 0.0)
            last_sent_linear = 0.0
            last_sent_angular = 0.0

            # rclpy.ok() verifica si ROS2 está activo (no se ha recibido Ctrl+C global)
            while rclpy.ok():
                # Lee una tecla sin bloquear la ejecución del nodo por mucho tiempo
                key = get_key(self.settings, timeout=0.05) # Ajusta timeout si la respuesta no es fluida

                # --- Lógica de control ---
                # Inicializamos el comando target para esta iteración
                target_linear = 0.0
                target_angular = 0.0
                publish_command = False # Bandera para saber si necesitamos publicar un comando al final del loop

                if key == 'q':
                    # Si se presiona 'q', salimos del bucle principal
                    self.get_logger().info('Comando de salida recibido (Q).')
                    break # Exit loop

                elif '1' <= key <= '9':
                    # Si se presiona un número (entre '1' y '9'): actualizamos la velocidad base
                    speed_factor = int(key) / 9.0 # Escala el número (ej. 1/9, 5/9, 9/9)
                    self.linear_velocity = speed_factor * self.MAX_LIN_VEL
                    self.angular_velocity = speed_factor * self.MAX_ANG_VEL
                    self.get_logger().info(f'Velocidad base actualizada (Escala {key}/9): Lineal={self.linear_velocity:.2f}, Angular={self.angular_velocity:.2f}')
                    # NOTA: Presionar un número SOLO cambia la velocidad base para futuros movimientos WASD.
                    # NO causa que la tortuga se mueva por sí solo ni la detiene inmediatamente.
                    # El movimiento activo se controla con WASD o soltando teclas (key == '').
                    publish_command = False # No publicamos un comando de movimiento debido a un número
                    # Continuamos al final del bucle para procesar ROS2 y leer otra tecla
                    # Si WASD está presionado, la tortuga continuará moviéndose, y si se suelta,
                    # el bloque key == '' enviará la parada usando las *nuevas* velocidades (0,0).
                    # No hacemos 'continue' aquí para que spin_once se ejecute.

                elif key in key_bindings:
                    # Si se presiona una tecla de movimiento (WASD): calculamos el comando target
                    # usando los factores de la tecla y las velocidades base actuales
                    vel_factor_lin, vel_factor_ang = key_bindings[key]
                    target_linear = vel_factor_lin * self.linear_velocity
                    target_angular = vel_factor_ang * self.angular_velocity
                    # Marcamos para publicar solo si el comando target es diferente al último enviado
                    if target_linear != last_sent_linear or target_angular != last_sent_angular:
                         publish_command = True

                elif key == '':
                    # Si no se presionó ninguna tecla en el timeout:
                    # El comando target es (0,0) - parada.
                    target_linear = 0.0
                    target_angular = 0.0
                    # Marcamos para publicar solo si el último comando enviado no era ya (0,0)
                    # (es decir, si la tortuga se estaba moviendo y ahora debe detenerse)
                    if last_sent_linear != 0.0 or last_sent_angular != 0.0:
                         publish_command = True

                # --- Publicar el comando si es necesario ---
                if publish_command:
                    self.send_command(target_linear, target_angular)
                    last_sent_linear = target_linear # Actualizamos el último comando enviado
                    last_sent_angular = target_angular

                # --- Fin Lógica de control ---


                # Permite que ROS2 procese callbacks pendientes y mantenga el nodo vivo
                rclpy.spin_once(self, timeout_sec=0.001) # Un timeout muy corto para no bloquear

        except Exception as e:
            self.get_logger().error(f'Ocurrió un error: {e}')

        finally:
            # Asegúrate de que la tortuga se detenga y la terminal se restaure
            # Esta sección se ejecuta cuando sales del bucle (por 'q' o Ctrl+C)
            self.send_command(0.0, 0.0) # Envía un último comando de detener
            self.get_logger().info("Deteniendo tortuga y restaurando terminal...")
            # La función get_key ya restaura la configuración, pero la volvemos a llamar aquí
            # para estar seguros en caso de una salida abrupta del bucle antes de get_key
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except termios.error as e:
                 self.get_logger().warn(f"Error al restaurar la terminal: {e}. Intenta reiniciar tu terminal.")


def main(args=None):
    # Inicializa la librería ROS2
    rclpy.init(args=args)

    # Crea una instancia de tu nodo
    teleop_node = MyTeleopNode()

    try:
        # Ejecuta la lógica principal del nodo
        teleop_node.run()
    except KeyboardInterrupt:
        # Maneja la interrupción por teclado (Ctrl+C) de forma limpia
        teleop_node.get_logger().info('Interrupción por teclado (Ctrl+C) detectada.')
        # El bloque finally en teleop_node.run() se encargará de la limpieza

    finally:
        # Limpieza final después de que run() termina
        # Si run() sale por 'q', el finally interno ya detuvo la tortuga y restauró la terminal.
        # Si sale por Ctrl+C, el except KeyboardInterrupt se activa y luego este finally
        # se asegura de que rclpy.shutdown() se llame.
        # Importante: el método run() ya llama send_command(0,0) en su finally,
        # así que esta llamada aquí es redundante pero segura.
        teleop_node.destroy_node() # Destruye el nodo ROS2
        rclpy.shutdown() # Cierra la comunicación ROS2


if __name__ == '__main__':
    main()
