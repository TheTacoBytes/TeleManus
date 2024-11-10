import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from Rosmaster_Lib import Rosmaster
import time

bot = Rosmaster()
bot.create_receive_threading()


class ArmControlSubscriber(Node):
    def __init__(self):
        super().__init__('arm_control_subscriber')
        self.subscription = self.create_subscription(
            Int32MultiArray, 'arm_control_topic', self.listener_callback, 10)
        self.subscription  

    def listener_callback(self, msg):
        angles = msg.data
        if len(angles) == 5:
            s1, s2, s3, s4, s5 = angles
            bot.set_uart_servo_angle_array([s1, s2, s3, s4, 90, s5])
            self.get_logger().info(f'Received angles: {angles}')
        else:
            self.get_logger().error(
                f'Incorrect number of angles received: {angles}')


def main(args=None):
    rclpy.init(args=args)

    arm_control_subscriber = ArmControlSubscriber()

    try:
        rclpy.spin(arm_control_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        arm_control_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
