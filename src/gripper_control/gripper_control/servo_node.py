import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from gpiozero import Servo
from time import sleep

# GPIO pin
SERVO_PIN = 18

# Adjust these if needed
OPEN_POS = -0.8
CLOSED_POS = -0.1


class ServoNode(Node):

    def __init__(self):
        super().__init__('servo_node')

        self.get_logger().info("Servo node started")

        self.servo = Servo(SERVO_PIN)

        self.subscription = self.create_subscription(
            Bool,
            'gripper_command',
            self.callback,
            10
        )

    def callback(self, msg):

        if msg.data:
            self.get_logger().info("Closing gripper")
            self.servo.value = CLOSED_POS

        else:
            self.get_logger().info("Opening gripper")
            self.servo.value = OPEN_POS

        sleep(0.5)


def main(args=None):

    rclpy.init(args=args)

    node = ServoNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
