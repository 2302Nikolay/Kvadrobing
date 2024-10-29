import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        self.lidar_data = {"strht": None, "left": None, "right": None}
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')
        
        # Инициализация кадрового счетчика и подписка на камеру
        self.frame_counter = 0
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(Image, '/uav1/camera', self.camera_callback, 200)

        # Инициализация издателя управления дроном
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/uav1/mavros/setpoint_velocity/cmd_vel', 10)

        self.take_off()

        # Создание и запуск потока обработки кадров
        self.frame_queue = None
        self.processing_thread = threading.Thread(target=self.process_frame_thread)
        self.processing_thread.daemon = True
        self.processing_thread.start()

    def take_off(self):
        # Код для взлета
        self.get_logger().info('\x1b[33m Попытка взлета... \x1b[0m ')
        cmd = TwistStamped()
        cmd.twist.linear.z = 0.5
        cmd.header.stamp = self.get_clock().now().to_msg()

        # Публикуем целевую позицию для взлета
        for _ in range(0,100):
            self.cmd_vel_pub.publish(cmd)
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.05))
        self.get_logger().info('\x1b[33m Команда на взлет отправлена... \x1b[0m ')

    def camera_callback(self, image_msg):
        # Счетчик кадров
        self.frame_counter = (self.frame_counter + 1) % 3
        if self.frame_counter != 0:
            return

        # Конвертируем сообщение в формат OpenCV
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        # Отправляем кадр в очередь для обработки
        self.frame_queue = frame

    def process_frame_thread(self):
        cv2.setUseOptimized(True)
        while True:
            # Проверяем наличие кадра для обработки
            if self.frame_queue is None:
                continue

            frame = self.frame_queue
            self.frame_queue = None  # Очищаем очередь после захвата кадра

            # Создаем объект для распознавания QR-кода
            qr_detector = cv2.QRCodeDetector()
            data, vertices, _ = qr_detector.detectAndDecode(frame)

            # Если QR-код распознан, рисуем рамку и управляем дроном
            if vertices is not None:
                vertices = vertices[0]
                center_x = int(np.mean(vertices[:, 0]))
                center_y = int(np.mean(vertices[:, 1]))

                # Рассчитываем площадь QR-кода на изображении
                width = int(np.max(vertices[:, 0]) - np.min(vertices[:, 0]))
                height = int(np.max(vertices[:, 1]) - np.min(vertices[:, 1]))
                qr_area = width * height

                # Условие остановки, если дрон подлетел на нужное расстояние
                stop_area = 500  # Подберите экспериментально, чтобы настроить нужное расстояние
                if qr_area >= stop_area:
                    # Останавливаем дрон, если QR-код достаточно велик (то есть близко)
                    stop_cmd = TwistStamped()
                    stop_cmd.header.stamp = self.get_clock().now().to_msg()
                    stop_cmd.twist.linear.x = 0.0
                    stop_cmd.twist.linear.y = 0.0
                    stop_cmd.twist.linear.z = 0.0
                    stop_cmd.twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(stop_cmd)
                    continue

                # Если QR-код еще не достиг нужного размера, продолжаем управление дроном
                self.control_drone(center_x, center_y, frame.shape[1] // 2, frame.shape[0] // 2)

    def control_drone(self, qr_x, qr_y, frame_center_x, frame_center_y):
        delta_x = qr_x - frame_center_x
        delta_y = qr_y - frame_center_y
        tolerance = 50  # допустимое отклонение

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        # Поворот и подъем в зависимости от смещения
        if abs(delta_x) > tolerance:
            cmd.twist.angular.z = -0.0005 * delta_x
        if abs(delta_y) > tolerance:
            cmd.twist.linear.z = -0.0005 * delta_y

        # Если QR в центре, двигаемся вперед
        if abs(delta_x) <= tolerance and abs(delta_y) <= tolerance:
            cmd.twist.linear.x = 0.5
        self.cmd_vel_pub.publish(cmd)

    def land(self):
        self.get_logger().info('\x1b[33m Посадка... \x1b[0m')
        # Установка режима посадки
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'AUTO.LAND'
            future = self.set_mode_client.call_async(set_mode_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('\x1b[33m Режим посадки установлен \x1b[0m')
            else:
                self.get_logger().error('\x1b[31m Не удалось установить режим посадки \x1b[0m')
        else:
            self.get_logger().error('\x1b[31m Сервис установки режима недоступен \x1b[0m')

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    node.take_off()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
