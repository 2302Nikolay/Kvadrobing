import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')

        # Создаем клиента для установки режима
        self.set_mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')

        # Публишер для установки целевой позиции дрона (для взлета)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/uav1/mavros/setpoint_position/local', 10)

        # Подписываемся на топик состояния дрона (для проверки состояния)
        self.state_sub = self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)

        # Текущее состояние дрона
        self.current_state = None

        self.test_services()

    def state_cb(self, msg):
        self.current_state = msg

    def test_services(self):
        # Ждем, пока дрон не подключится
        while not self.current_state or not self.current_state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            rclpy.spin_once(self)

        # Проверка доступности сервиса армирования
        if self.arming_client.wait_for_service(timeout_sec=5.0):
            arm_req = CommandBool.Request()
            arm_req.value = True
            future = self.arming_client.call_async(arm_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().success:
                self.get_logger().info('Drone armed')
            else:
                self.get_logger().error('Failed to arm drone')
        else:
            self.get_logger().error('Arming service not available')

        # Установка режима OFFBOARD
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'OFFBOARD'
            future = self.set_mode_client.call_async(set_mode_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('OFFBOARD mode set')
            else:
                self.get_logger().error('Failed to set OFFBOARD mode')
        else:
            self.get_logger().error('Set mode service not available')

        # Взлет
        self.takeoff()

        # Ждем некоторое время на высоте (например, 2 секунд)
        time.sleep(0.2)

        # Посадка
        self.land()

    def takeoff(self):
        self.get_logger().info('Taking off...')

        # Создаем сообщение для взлета (целевое положение)
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()

        # Устанавливаем высоту взлета (например, 10 метров)
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 10.0

        # Публикуем целевую позицию несколько раз (необходимо для стабильной работы OFFBOARD режима)
        for _ in range(0,10):
            self.local_pos_pub.publish(pose)
            rclpy.spin_once(self)
            time.sleep(0.05)

        self.get_logger().info('Drone should be at target altitude.')

    def land(self):
        self.get_logger().info('Landing...')

        # Устанавливаем режим на посадку
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'AUTO.LAND'
            future = self.set_mode_client.call_async(set_mode_req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() and future.result().mode_sent:
                self.get_logger().info('Landing mode set')
            else:
                self.get_logger().error('Failed to set Landing mode')
        else:
            self.get_logger().error('Set mode service not available')


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
