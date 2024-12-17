import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class BooleanSubscriberPublisher(Node):

    def __init__(self):
        super().__init__('boolean_subscriber_publisher')

        # Suscripción al tópico 'avanzar'
        self.subscription_avanzar = self.create_subscription(
            Bool,
            'avanzar',
            self.listener_callback_avanzar,
            10)

        # Suscripción al tópico 'vueltas'
        self.subscription_vueltas = self.create_subscription(
            Bool,
            'vueltas',
            self.listener_callback_vueltas,
            10)

        # Publicación en el tópico 'topic'
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback_avanzar(self, msg):
        if msg.data:
            self.get_logger().info(' True en /avanzar')
            vel_x = 0.5
            vel_y = 0.0
        else:
            self.get_logger().info('False en /avanzar')
            vel_x = 0.0
            vel_y = 0.0
        
        self.publish_message(vel_x, vel_y)

    def listener_callback_vueltas(self, msg):
        if msg.data:
            self.get_logger().info(' True en /vueltas')
            vel_x = 0.0
            vel_y = 0.5
        else:
            self.get_logger().info('False en /vueltas')
            vel_x = 0.0
            vel_y = 0.0

        self.publish_message(vel_x, vel_y)

    def publish_message(self, vel_x, vel_y):
        msg = Twist()
        msg.linear.x = vel_x
        msg.angular.z = vel_y
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = BooleanSubscriberPublisher()

    rclpy.spin(node)

    # Destruimos el nodo explícitamente
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
