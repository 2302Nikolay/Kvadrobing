import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DepthCameraVisualizer(Node):
    def __init__(self):
        super().__init__('depth_camera_visualizer')
        self.subscription = self.create_subscription(
            Image,
            '/uav1/depth_camera',
            self.depth_camera_callback,
            10)
        self.bridge = CvBridge()  # Конвертер ROS -> OpenCV
        self.min_depth = 0.5  # Минимальное расстояние в метрах (регулируется)
        self.max_depth = 10.0  # Максимальное расстояние в метрах (регулируется)

    def depth_camera_callback(self, msg):
        try:
            # Преобразуем изображение глубины из ROS-сообщения в формат OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except CvBridgeError as e:
            self.get_logger().error(f"Ошибка конвертации изображения глубины: {e}")
            return

        # Установка значений за пределами диапазона глубины в NaN для дальнейшей фильтрации
        depth_image = np.where((depth_image >= self.min_depth) & (depth_image <= self.max_depth), 
                            depth_image, np.nan)

        # Игнорируем NaN значения для получения min и max глубины в нужном диапазоне
        finite_depth_image = depth_image[np.isfinite(depth_image)]
        if finite_depth_image.size > 0:
            depth_min, depth_max = finite_depth_image.min(), finite_depth_image.max()
        else:
            self.get_logger().warn("Нет данных в диапазоне глубины.")
            return

        # Нормализуем изображение глубины вручную
        depth_normalized = np.uint8(255 * (depth_image - depth_min) / (depth_max - depth_min))
        depth_normalized[np.isnan(depth_image)] = 0  # Преобразуем NaN в 0 для отображения

        # Визуализация с цветовой картой
        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        cv2.imshow("Depth Image", depth_colormap)
        cv2.waitKey(1)  # Обновляем кадр


def main(args=None):
    rclpy.init(args=args)
    depth_camera_visualizer = DepthCameraVisualizer()
    rclpy.spin(depth_camera_visualizer)
    depth_camera_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
