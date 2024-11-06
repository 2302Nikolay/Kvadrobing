import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import TwistStamped, PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import threading
import math

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # Создаем QoS профиль с режимом BEST_EFFORT
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        
        # Инициализация сервисов и подписчиков
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(Image, '/uav1/camera_down', self.camera_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/uav1/mavros/local_position/pose', self.pose_callback, qos_profile=qos_profile)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/uav1/mavros/setpoint_velocity/cmd_vel', 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)

        # Создание потока для обработки кадров
        self.frame_queue = None
        self.processing_thread = threading.Thread(target=self.process_frame_thread)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.drone_position = None  # Для хранения текущей позиции дрона
        self.cube_position = None   # Координаты куба 
        self.cube_distance = None   # Дистанция до куба
        self.searched_cube = False  # Флаг с сообщением о том, найден ли кубик: false - не найден, true - найден

    def take_off(self):
        # Функция для взлета дрона
        self.get_logger().info('Попытка взлета...')
        pose = PoseStamped()
        pose.pose.position.z = 4.0  # Набор высоты
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.header.stamp = self.get_clock().now().to_msg()

        # Публикуем команду взлета
        for _ in range(100):
            self.local_pos_pub.publish(pose)
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.05))
        self.get_logger().info('Команда на взлет отправлена...')
        
        time.sleep(3)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        if self.searched_cube == False:
            cmd.twist.linear.y = 0.5
            self.get_logger().info("Режим блуждание")
        else:
            self.get_logger().info("НЕ режим блуждания...")
            cmd.twist.linear.y = 0.0
        self.cmd_vel_pub.publish(cmd)


    def camera_callback(self, image_msg):
        # Обработка входящих изображений от камеры
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        self.frame_queue = frame  # Отправляем кадр в очередь для обработки

    def pose_callback(self, pose_msg):
        # Обновление позиции дрона
        self.drone_position = (pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)


    def process_frame_thread(self):
        while True:
            if self.frame_queue is None:
                continue

            frame = self.frame_queue
            self.frame_queue = None  # Очищаем очередь после обработки

            # Обнаружение цветной фигуры на изображении
            figure_detected, figure_center, figure_area = self.detect_figure(frame)

            if figure_detected:
                self.searched_cube = True
                self.get_logger().info('Обнаружена цветная фигура')
                self.get_logger().info(f'Площадь фигуры: {figure_area}')

                # Если фигура еще не достигла нужного размера, корректируем траекторию
                self.control_drone(figure_center, frame.shape[1] // 2, frame.shape[0] // 2)

            if self.drone_position is not None:
                x, y, z = self.drone_position
                coords_text = f"Drone Position: X={x:.2f}, Y={y:.2f}, Z={z:.2f}"
                if self.cube_distance != None:
                    distance_text = f"Distance: {self.cube_distance:.2f} m. Color: RED"
                    cv2.putText(frame, distance_text, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                cv2.putText(frame, coords_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            else:
                self.get_logger().warn("Drone position data not available yet.")


            # Показываем обновленное изображение
            cv2.imshow("Drone Camera Down", frame)
            cv2.waitKey(1)

    def detect_figure(self, frame):
        # Обнаружение фигуры определенного цвета на изображении
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Настройки для поиска красного цвета (измените под нужный цвет фигуры)
        lower_red = np.array([0, 74, 50], dtype=np.uint8)
        upper_red = np.array([3, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv_frame, lower_red, upper_red)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Игнорируем мелкие контуры
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                area = w * h

                # Рисуем контуры для визуализации
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
                
                return True, (center_x, center_y), area
            
        return False, (0, 0), 0

    def control_drone(self, figure_center, frame_center_x, frame_center_y):
        # Функция для управления дроном в направлении центра фигуры
        self.get_logger().info('Корректировка положения дрона...')
        delta_x = figure_center[0] - frame_center_x
        delta_y = figure_center[1] - frame_center_y
        tolerance = 10  # Допустимое отклонение для центрирования

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        
        x, y, z = self.drone_position

        if z > 4.0:
            cmd.twist.linear.z = -0.1
            self.get_logger().info(f"Корректировка дрона вниз... {z}")
        elif z < 4.0:
            cmd.twist.linear.z = 0.1
            self.get_logger().info(f"Корректировка дрона вверх... {z}")


        # Регулируем движение в зависимости от смещения фигуры
        if abs(delta_x) > tolerance:
            cmd.twist.linear.y = -0.001 * delta_x  # Поправка по оси Y
            self.get_logger().info(f'Смещение по Y: {delta_x}')
        if abs(delta_y) > tolerance:
            cmd.twist.linear.x = -0.001 * delta_y  # Поправка по оси X
            self.get_logger().info(f'Смещение по X: {delta_y}')

        # Если дрон находится над центром фигуры
        if abs(delta_x) <= tolerance and abs(delta_y) <= tolerance and self.cube_position == None:
            self.cube_position = self.drone_position 
            self.cube_distance = math.sqrt(pow(x, 2) + pow(y, 2))
            self.searched_cube = False
            self.get_logger().error(f'Дрон над фигурой, позиция дрона {self.cube_position}, расстояние до куба: {self.cube_distance}')
        
        self.cmd_vel_pub.publish(cmd)

    def hover_over_figure(self):
        # Остановка дрона над центром фигуры
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.linear.y = 0.0
        stop_cmd.twist.linear.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

    def land(self):
        # Функция для посадки дрона
        self.get_logger().info('Посадка...')
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'AUTO.LAND'
            future = self.set_mode_client.call_async(set_mode_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('Режим посадки установлен')
            else:
                self.get_logger().error('Не удалось установить режим посадки')
        else:
            self.get_logger().error('Сервис установки режима недоступен')

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    node.take_off()
    time.sleep(7) # чтобы немного подождал после взлета
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
