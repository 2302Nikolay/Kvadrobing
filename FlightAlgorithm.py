import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')

        # Публишер для установки целевой позиции дрона (для взлета)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)

        # Подписываемся на топик состояния дрона (для проверки состояния)
        self.state_sub = self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)

        self.test_services()

    def state_cb(self, msg):
        self.current_state = msg

    def test_services(self):

        # Взлет
        self.take_off() # взлетаем

        time.sleep(2) # ждем некоторое время на высоте (например, 2 секунд)

        self.fly() # выполняем полет

        time.sleep(2) 

        # Посадка
        self.land()


    def take_off(self):
        self.get_logger().info('\x1b[33m Попытка взлета... \x1b[0m ')
        # Создаем сообщение PoseStamped для взлета
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()

        # Устанавливаем целевую позицию
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 3.0

        # Публикуем целевую позицию несколько раз для взлета
        for _ in range(100):
            self.local_pos_pub.publish(pose)
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

        
    def fly(self):
        self.get_logger().info('\x1b[33m Попытка взлета... \x1b[0m ')

        # Создаем сообщение PoseStamped для полета
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()

        # Пролет вперед на 2 метра
        self.get_logger().info('\x1b[33m Пролет вперед на 2 метра...\x1b[0m')
        pose.pose.position.x = 2.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 3.0


        # Публикуем целевую позицию несколько раз для движения вперед
        for _ in range(0, 100):
            self.local_pos_pub.publish(pose)
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.05))
        self.get_logger().info('\x1b[32m Дрон пролетел вперед на 2 метра. \x1b[0m ')

        # Пролет назад на 2 метра
        self.get_logger().info('\x1b[33m Пролет назад на 2 метра...\x1b[0m')
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 3.0

        # Публикуем целевую позицию несколько раз для движения назад
        for _ in range(0, 100):
            self.local_pos_pub.publish(pose)
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.05))
        self.get_logger().info('\x1b[32m Дрон пролетел вперед на 2 метра. \x1b[0m ')


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
