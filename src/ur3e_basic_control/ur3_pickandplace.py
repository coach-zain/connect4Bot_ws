#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
import time


class UR3ePickPlace(Node):
    def __init__(self):
        super().__init__('ur3e_pick_place')

        # ✅ Use the scaled joint trajectory controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        # Publisher for gripper
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/finger_width_controller/commands',
            10
        )

        # Store the pick position for reuse
        self.pick_pose = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]

    def move_arm(self, positions, duration=5.0, done_cb=None):
        """Send joint trajectory goal to move the arm."""
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        traj.points.append(point)
        goal_msg.trajectory = traj

        self._action_client.wait_for_server()
        send_future = self._action_client.send_goal_async(goal_msg)

        # Pass through the callback to chain motions
        def wrapped_goal_response(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error('Arm goal rejected!')
                return
            self.get_logger().info('Arm goal accepted!')
            result_future = goal_handle.get_result_async()
            if done_cb:
                result_future.add_done_callback(done_cb)

        send_future.add_done_callback(wrapped_goal_response)

    def close_gripper(self, width: float = 0.0):
        """Close gripper to target width (default 0.0 = fully closed)."""
        msg = Float64MultiArray()
        msg.data = [width]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Gripper command sent: {width:.3f} m")
        time.sleep(1.0)

    def open_gripper(self, width: float = 0.08):
        """Open gripper to target width (default 0.08 = fully open)."""
        msg = Float64MultiArray()
        msg.data = [width]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Gripper command sent: {width:.3f} m")
        time.sleep(1.0)

    # ---------------- Sequence ----------------

    def start_pick_place(self):
        # Step 1: Move to pick pose
        self.get_logger().info("Moving to pick pose...")
        self.move_arm(self.pick_pose, duration=7.0, done_cb=self.after_pick_pose)

    def after_pick_pose(self, future):
        self.get_logger().info("At pick pose. Closing gripper...")
        self.close_gripper()

        self.get_logger().info("Waiting for gripper to close...")
        time.sleep(2.0)  # tune this to match gripper speed

        # ✅ Step 2: Lift the object slightly upward before rotating
        self.lift_pose = self.pick_pose.copy()
        self.lift_pose[1] -= 0.65  # raise shoulder_lift_joint (~5 cm upward)
        self.get_logger().info("Lifting object upwards...")
        self.move_arm(self.lift_pose, duration=2.0, done_cb=self.after_lift_pose)

    def after_lift_pose(self, future):
        # ✅ Step 3: Rotate about base joint to move to place position
        self.place_pose = self.lift_pose.copy()
        self.place_pose[0] = 1.57   # rotate base joint 90 degrees
        self.get_logger().info("Moving to place pose...")
        self.move_arm(self.place_pose, duration=7.0, done_cb=self.after_place_pose)

    def after_place_pose(self, future):
        self.get_logger().info("At place pose. Tilting jug to pour...")

        # ✅ Step 4: Tilt the jug (rotate wrist_2_joint forward)
        tilt_pose = self.place_pose.copy()
        tilt_angle = 0.6545  # about 30 degrees forward tilt
        tilt_pose[4] -= tilt_angle  # ✅ subtract instead of add to tilt correctly
        self.move_arm(tilt_pose, duration=3.0, done_cb=self.after_tilt_pose)

    def after_tilt_pose(self, future):
        self.get_logger().info("Holding pour position...")
        time.sleep(4.0)  # simulate pouring duration

        # ✅ Step 5: Return to upright position
        self.get_logger().info("Returning jug to upright position...")
        upright_pose = self.place_pose.copy()
        self.move_arm(upright_pose, duration=3.0, done_cb=self.after_upright_pose)

    def after_upright_pose(self, future):
        self.get_logger().info("At upright position. Opening gripper...")
        # self.open_gripper()


def main(args=None):
    rclpy.init(args=args)
    node = UR3ePickPlace()

    # Start the pick and place sequence
    node.start_pick_place()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
