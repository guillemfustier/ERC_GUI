import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class MultiCameraSubscriber(Node):
    def __init__(self):
        super().__init__('multi_camera_subscriber')
        print("[INFO] Inicializando el nodo 'multi_camera_subscriber'")

        # Suscripciones a los tópicos
        self.subscription_cam1 = self.create_subscription(
            CompressedImage,
            '/camara_logitech_1/image_raw/compressed',
            self.listener_callback_cam1,
            10)
        print("[INFO] Suscripción al tópico '/camara_logitech_1/image_raw/compressed' creada")

        self.subscription_cam2 = self.create_subscription(
            CompressedImage,
            '/camara_logitech_2/image_raw/compressed',
            self.listener_callback_cam2,
            10)
        print("[INFO] Suscripción al tópico '/camara_logitech_2/image_raw/compressed' creada")

        self.subscription_color = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.listener_callback_color,
            10)
        print("[INFO] Suscripción al tópico '/camera/color/image_raw/compressed' creada")

        self.subscription_depth = self.create_subscription(
            CompressedImage,
            '/camera/depth/image_rect_raw/compressed',
            self.listener_callback_depth,
            10)
        print("[INFO] Suscripción al tópico '/camera/depth/image_rect_raw/compressed' creada")

    def listener_callback_cam1(self, msg):
        print("[INFO] Imagen comprimida recibida de la cámara Logitech 1")
        self.process_image(msg, "Cámara Logitech 1")

    def listener_callback_cam2(self, msg):
        print("[INFO] Imagen comprimida recibida de la cámara Logitech 2")
        self.process_image(msg, "Cámara Logitech 2")

    def listener_callback_color(self, msg):
        print("[INFO] Imagen comprimida recibida de la cámara de Color")
        self.process_image(msg, "Cámara de Color")

    def listener_callback_depth(self, msg):
        print("[INFO] Imagen comprimida recibida de la cámara de Profundidad")
        self.process_image(msg, "Cámara de Profundidad", depth=True)

    def process_image(self, msg, window_name, depth=False):
        try:
            # Convertir los datos comprimidos a una imagen de OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)

            if depth:
                # Para profundidad, verificar el contenido
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # Cambiar a UNCHANGED para datos de profundidad
            else:
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Estándar para imágenes de color

            if cv_image is None:
                raise ValueError("Los datos no pudieron ser decodificados. Posible formato no compatible.")

            print(f"[DEBUG] Imagen descomprimida y convertida a formato OpenCV ({window_name})")

            # Mostrar la imagen usando OpenCV
            cv2.imshow(window_name, cv_image)
            print(f"[DEBUG] Mostrando imagen en ventana: {window_name}")
            cv2.waitKey(1)  # Actualiza la ventana de OpenCV
        except Exception as e:
            print(f"[ERROR] Error al procesar la imagen ({window_name}): {e}")

def main(args=None):
    print("[INFO] Iniciando el nodo")
    rclpy.init(args=args)
    multi_camera_subscriber = MultiCameraSubscriber()

    try:
        rclpy.spin(multi_camera_subscriber)
    except KeyboardInterrupt:
        print("[INFO] Nodo detenido por el usuario (KeyboardInterrupt)")
    except Exception as e:
        print(f"[ERROR] Excepción inesperada: {e}")
    finally:
        # Destruir el nodo y cerrar ventanas de OpenCV
        print("[INFO] Cerrando nodo y ventanas")
        multi_camera_subscriber.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
