import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CameraPublisher(Node):
    """
    Este nodo toma imágenes de una cámara conectada al sistema 
    y las publica comprimidas en el topic /camara_logitech_1/image_raw/compressed.
    Así otros nodos pueden suscribirse y obtener el stream de video.
    """
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/camara_logitech_1/image_raw/compressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            ret_jpg, jpg_buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(jpg_buffer).tobytes()
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
