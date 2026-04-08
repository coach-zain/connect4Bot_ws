#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration


class UR3eMove(Node):
    def __init__(self):
        super().__init__('ur3e_move')
        # ✅ Use the scaled controller for the real robot
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

    def send_goal(self, positions, duration=3.0):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        traj.points.append(point)
        goal_msg.trajectory = traj

        # Wait until action server is ready
        self.get_logger().info('Waiting for scaled_joint_trajectory_controller...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending trajectory goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by server!')
            return
        self.get_logger().info('✅ Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('🏁 Goal finished!')


def main(args=None):
    rclpy.init(args=args)
    node = UR3eMove()
    # Example: Move back to upright/reset position
    node.send_goal([0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0], duration=7.0)
    #node.send_goal([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], duration=7.0)  # downward L position
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
