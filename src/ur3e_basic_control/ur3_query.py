import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class UR3eJointReader(Node):
    def __init__(self):
        super().__init__('ur3e_joint_reader')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',   # topic published by ur_robot_driver
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: JointState):
        # Convert radians to degrees
        joint_degrees = [math.degrees(pos) for pos in msg.position]
        joint_info = {name: round(deg, 2) for name, deg in zip(msg.name, joint_degrees)}
        print(f"Joint positions (deg): {joint_info}\n")  # extra newline for spacing
        time.sleep(1.0)  # 1-second delay

def main(args=None):
    rclpy.init(args=args)
    node = UR3eJointReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
