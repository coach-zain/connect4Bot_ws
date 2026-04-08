#include <memory>
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "print_current_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("print_current_pose");

  // Required executor spinner
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  using moveit::planning_interface::MoveGroupInterface;

  MoveGroupInterface mgi(node, "ur_manipulator");

  RCLCPP_INFO(logger, "Printing current pose every second...");

  rclcpp::Rate rate(1.0);  // 1 Hz

  while (rclcpp::ok())
  {
    auto pose = mgi.getCurrentPose();

    auto p = pose.pose.position;
    auto q = pose.pose.orientation;

    RCLCPP_INFO(logger,
      "Position: x=%.4f y=%.4f z=%.4f",
      p.x, p.y, p.z);

    RCLCPP_INFO(logger,
      "Orientation: qx=%.4f qy=%.4f qz=%.4f qw=%.4f",
      q.x, q.y, q.z, q.w);

    RCLCPP_INFO(logger, "--------------------------------");

    rate.sleep();
  }

  rclcpp::shutdown();
  spinner.join();

  return 0;
}