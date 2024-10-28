import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
import rclpy.time
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
import time
import math
import numpy as np

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        self.lidar_data = {"strht": None, "left": None, "right": None}

        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')

        
        # Подписываемся на топик лидара (для управление полетом)
        self.lidar_sub = self.create_subscription(LaserScan, '/uav1/scan', self.lidar_cb, 10)

        # Подписывваемся на паблишер для управления дроном 
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/uav1/mavros/setpoint_velocity/cmd_vel', 10)

        self.test_services()

    def test_services(self):

        # Взлет
        self.take_off() # взлетаем

        time.sleep(2) # ждем некоторое время на высоте (например, 2 секунд)

        # Посадка
        #self.land()

    def take_off(self):
        self.get_logger().info('\x1b[33m Попытка взлета... \x1b[0m ')
        # Создаем сообщение PoseStamped для взлета
        pose = TwistStamped()
        pose.header.stamp = self.get_clock().now().to_msg()

        # Устанавливаем целевую позицию
        pose.twist.linear.x = 0.0
        pose.twist.linear.y = 0.0
        pose.twist.linear.z = 1.0

        # Публикуем целевую позицию несколько раз для взлета
        for _ in range(100):
            self.cmd_vel_pub.publish(pose)
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.05))
        self.get_logger().info('\x1b[33m Команда на взлет отправлена... \x1b[0m ')


    def land(self):
        self.get_logger().info('\x1b[33m Посадка... \x1b[0m')

        # Устанавливаем режим на посадку
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
    
    def fly_forward(self):
        self.get_logger().info('\x1b[33m Полет вперёд... \x1b[0m ')

        # Создаем сообщение PoseStamped для полета
        pose = TwistStamped()
        pose.header.stamp = self.get_clock().now().to_msg()

        # Пролет вперед
        pose.twist.linear.x = 0.1
        pose.twist.linear.y = 0.0
        pose.twist.linear.z = 0.0
        self.get_logger().info('\x1b[33m Пролет вперед...\x1b[0m')


        # Публикуем целевую позицию несколько раз для движения вперед
        for _ in range(0, 100):
            self.cmd_vel_pub.publish(pose)
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.05))
        self.get_logger().info('\x1b[32m Дрон пролетел вперед. \x1b[0m ')

          

    def lidar_cb(self, msg):
        # Извлекаем расстояния в разных направлениях (например, спереди, справа и слева)
        front_distance = min(msg.ranges[158:202])       # Прямо перед дроном
        left_distance = min(msg.ranges[68:112])         # Лидар смотрит влево
        right_distance = min(msg.ranges[248:292])       # Лидар смотрит вправо
        #back_distance = min(msg.ranges[0:22][338:360]) # Лидар смотрит назад
        
        pos = TwistStamped()
        
        # Двигаемся прямо, пока впереди нет препятствий
        if front_distance > 1.0 and left_distance > 1.0 and right_distance > 1.0 and front_distance != float('inf'):  # Если впереди нет препятствия ближе 0.7 метра
            self.fly_forward() # Лети вперёд до препятствия
        else:
            # Останавливаемся, если впереди препятствие
            pos.twist.linear.x = 0.0
            pos.twist.linear.y = 0.0
            pos.twist.linear.z = 0.0

            for _ in range(0, 100):
                self.cmd_vel_pub.publish(pos)
                self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.05))
            self.land()





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


if __name__ == '__main__':
    main()
