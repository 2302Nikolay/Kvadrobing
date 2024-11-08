import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollowingNode(Node):
    def __init__(self):
        super().__init__('line_following_node')
        self.bridge = CvBridge()
        
        # Подписчик на изображение с нижней камеры
        self.camera_sub = self.create_subscription(Image, '/uav1/camera_down', self.camera_callback, 10)
        
        # Публикация команд движения
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/uav1/mavros/setpoint_velocity/cmd_vel', 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)

        # Диапазон для черного цвета в пространстве HSV (можно настраивать)
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 255, 50])  # Поправлен верхний диапазон для черного цвета

        self.get_logger().info('Node initialized, waiting for camera images...')

    def camera_callback(self, image_msg):
        # Преобразование изображения из ROS-сообщения в OpenCV формат
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        # Выделяем только центральную область кадра
        h, w = frame.shape[:2]
        top_offset = 0               # Оставляем кадр без обрезки сверху
        bottom_offset = h            # Оставляем кадр без обрезки снизу
        left_offset = int(w * 0.3)   # Обрезаем 10% слева
        right_offset = int(w * 0.7)  # Обрезаем 10% справа
        cropped_frame = frame[top_offset:bottom_offset, left_offset:right_offset]

        # Обнаружение черной линии на обрезанном изображении
        found_line, line_center_x, angle_to_line = self.detect_black_line(cropped_frame)
        
        if found_line:
            # Корректируем положение дрона, чтобы двигаться вдоль линии
            self.follow_line(line_center_x, angle_to_line, cropped_frame.shape[1] // 2)

        # Визуализация для отладки
        cv2.imshow("Downward Camera - Line Following", cropped_frame)
        cv2.waitKey(1)

    def detect_black_line(self, frame):
        # Преобразуем изображение в пространство HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Применяем фильтр для выделения черного цвета
        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)

        # Поиск контуров на маске
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        line_center_x = 0
        angle_to_line = 0
        if contours:
            # Находим самый большой контур, вероятно это черная линия
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            line_center_x = x + w // 2
            
            # Рассчитываем угол между дроном и линией
            rect = cv2.minAreaRect(largest_contour)
            angle_to_line = rect[2]

            # Рисуем контур для визуализации
            cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
        
            return True, line_center_x, angle_to_line
        
        return False, 0, 0

    def follow_line(self, line_center_x, angle_to_line, frame_center_x):
        # Вычисляем отклонение линии от центра кадра
        deviation = line_center_x - frame_center_x
        tolerance = 20  # Допустимое отклонение от центра

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        # Корректируем движение вдоль линии
        if abs(deviation) > tolerance:
            cmd.twist.linear.y = -0.01 * deviation  # Поправка по оси Y для центрирования
            self.get_logger().info(f'Корректировка по Y: {cmd.twist.linear.y}')

        # Корректируем угол поворота
        cmd.twist.angular.z = -0.008 * angle_to_line  # Поправка угла для следования за линией
        self.get_logger().info(f'Корректировка угла: {cmd.twist.angular.z}')
        
        # Продвигаемся вперед вдоль линии
        cmd.twist.linear.x = 0.4  # Уменьшена скорость для более плавного прохождения

        # Публикуем команду движения
        self.cmd_vel_pub.publish(cmd)

    def take_off(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 5.0

        for _ in range(0,100):
            self.local_pos_pub.publish(pose)
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.05))
            self.get_logger().info('Команда на взлет отправлена...')

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingNode()
    node.take_off()
    time.sleep(2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
