import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import pyzbar.pyzbar as pyzbar

class TestNode(Node):
    def __init__(self):
        super().__init__('search_qr')

        # Подписываемся на топик камеры
        self.camera_sub = self.create_subscription(Image, '/uav1/camera_down', self.camera_cb, 10)
        self.bridge = CvBridge()

    def state_cb(self, msg):
        self.current_state = msg

    def camera_cb(self, msg):
        # Конвертируем ROS Image сообщение в OpenCV изображение
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Сканируем изображение на наличие QR-кодов
        decoded_objects = pyzbar.decode(cv_image)
        for obj in decoded_objects:
            self.get_logger().info(f'QR-код считан: {obj.data.decode("utf-8")}')

        # Отображаем изображение в отдельном окне
        cv2.imshow("Drone Camera Down", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
