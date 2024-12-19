import rclpy
from rclpy.node import Node
from custom_interfaces.msg import SetPosition
from dynamixel_sdk import PortHandler, PacketHandler

# Configuración de la tabla de control de Dynamixel
ADDR_OPERATING_MODE = 11  # 1: Control de velocidad | 3: Control de posición
ADDR_TORQUE_ENABLE = 64  # 0: Torque apagado | 1: Torque encendido
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEFAULT_DEVICE_NAME = "/dev/ttyUSB0"

class DynamixelPositionPublisher(Node):
    def __init__(self):
        super().__init__('dynamixel_position_publisher')

        # Configuración del puerto y manejadores Dynamixel
        self.port_handler = PortHandler(DEFAULT_DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error('¡Error al abrir el puerto!')
            return
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('¡Error al configurar el baudrate!')
            return

        self.get_logger().info('Puerto Dynamixel configurado correctamente')

        # Habilita el torque y configura el modo de operación para los motores
        self.setup_motor(0, 3)  # Motor 0 en modo posición
        self.setup_motor(1, 3)  # Motor 1 en modo posición

        # Publicadores
        self.publisher_motor_0 = self.create_publisher(SetPosition, '/set_position', 10)
        self.publisher_motor_1 = self.create_publisher(SetPosition, '/set_position', 10)

        # Publica las posiciones cada 1 segundo
        self.timer = self.create_timer(1.0, self.publish_positions)

        # Inicializa posiciones
        self.motor_0_position = 512
        self.motor_1_position = 512

        self.get_logger().info('Nodo de publicación de posiciones de Dynamixel inicializado')

    def setup_motor(self, motor_id, mode):
        # Configura el modo de operación
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_OPERATING_MODE, mode
        )
        if dxl_comm_result != 0:
            self.get_logger().error(f'Error al configurar el modo para el motor {motor_id}')
        else:
            self.get_logger().info(f'Modo de operación configurado para motor {motor_id}')

        # Habilita el torque
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 1
        )
        if dxl_comm_result != 0:
            self.get_logger().error(f'Error al habilitar torque para el motor {motor_id}')
        else:
            self.get_logger().info(f'Torque habilitado para motor {motor_id}')

    def publish_positions(self):
        msg_motor_0 = SetPosition()
        msg_motor_1 = SetPosition()

        msg_motor_0.id = 0
        msg_motor_0.position = self.motor_0_position

        msg_motor_1.id = 1
        msg_motor_1.position = self.motor_1_position

        self.publisher_motor_0.publish(msg_motor_0)
        self.publisher_motor_1.publish(msg_motor_1)

        self.get_logger().info(f'Publicando posiciones: Motor 0: {self.motor_0_position}, Motor 1: {self.motor_1_position}')

    def destroy_node(self):
        self.get_logger().info('Apagando motores y liberando recursos')
        for motor_id in [0, 1]:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, 0)
        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelPositionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido manualmente.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
