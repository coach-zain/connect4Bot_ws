#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import time


class UR3eGripperTest(Node):
    def __init__(self):
        super().__init__('ur3e_gripper_test')
        # Publisher for the OnRobot RG2 gripper
        self.gripper_pub = self.create_publisher(Float64MultiArray, '/finger_width_controller/commands', 10)
        time.sleep(0.5)  # allow publisher to initialize

    def move_gripper(self, width: float):
        """
        Move the gripper fingers to a given width in meters.
        Typical range: 0.0 (fully open) to ~0.08 (fully closed).
        """
        msg = Float64MultiArray()
        msg.data = [width]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Gripper command sent: {width:.3f} m")
        time.sleep(1.0)  # ensure the command is processed


def main(args=None):
    rclpy.init(args=args)

    # default width
    width = 0.08

    # check for command-line argument
    if len(sys.argv) > 1:
        try:
            width = float(sys.argv[1])
        except ValueError:
            print(f"Invalid width argument: {sys.argv[1]}. Using default {width}")

    node = UR3eGripperTest()
    node.move_gripper(width=width)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
