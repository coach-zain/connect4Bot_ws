#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>

// ======================== Helper Functions ================================

// Helper to build a Pose message cleanly
geometry_msgs::msg::Pose makePose(
  double x, double y, double z,
  double qx, double qy, double qz, double qw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = qx;
  p.orientation.y = qy;
  p.orientation.z = qz;
  p.orientation.w = qw;
  return p;
}

// Free Space Cartesian motion through a list of waypoints.
bool moveToPose(
  moveit::planning_interface::MoveGroupInterface& mgi,
  const geometry_msgs::msg::Pose& target,
  const rclcpp::Logger& logger)
{
  mgi.setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(mgi.plan(plan));

  if (success) {
    mgi.execute(plan);
    RCLCPP_INFO(logger, "Move succeeded.");
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  return success;
}

int main(int argc, char* argv[])
{
  // ============================= ROS2 Setup =====================================

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "ur3e_pick_and_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("ur3e_pick_and_place");

  rclcpp::executors::SingleThreadedExecutor executor; // spin the node so getCurrentState() / planning works
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto gripper_pub =
  node->create_publisher<std_msgs::msg::Bool>(
    "/gripper_command",
    10
  );

  // ======================== MoveGroupInterface Setup ================================

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mgi(node, "ur_manipulator");

  mgi.setMaxVelocityScalingFactor(0.3);    // 30% speed — safe for real hw
  mgi.setMaxAccelerationScalingFactor(0.3);
  mgi.setPlanningTime(30.0);               // seconds to allow for planning

  auto current = mgi.getCurrentPose();
  RCLCPP_INFO(logger, "Current EEF: x=%.3f y=%.3f z=%.3f",
    current.pose.position.x,
    current.pose.position.y,
    current.pose.position.z);

  // ============================= Define poses =======================================
  // auto wp0      = makePose(0.1737, 0.2076, 0.6410,  -0.7062, 0.3622, -0.0368, 0.6072); // test wp, delete after
  auto wp0      = makePose(0.3290, 0.2910, 0.3440,  0.8485, -0.4986, -0.0867, -0.1549); // between reset and prePick
  auto prePick  = makePose(0.3356, 0.0498, 0.1484,  -0.6502, 0.7595, 0.0058, 0.0162); // just above pick         
  auto pick     = makePose(0.3357, 0.0496, 0.0581,  -0.6501, 0.7597, 0.0056, 0.0159); // drop down to pick
  auto wp1      = makePose(0.0, -0.3, 0.10,  1.0, 0.0, 0.0, 0.0);
  auto wp2      = makePose(0.2851, 0.3432, 0.2571,  0.9863, -0.1577, -0.0143, -0.0470);
  auto prePlace = makePose(0.0932, 0.4568, 0.304,  0.7620, 0.6475, 0.0088, 0.0099);
  auto place    = makePose(0.0932, 0.4568, 0.2925,  0.7620, 0.6475, 0.0088, 0.0099);
  auto reset    = makePose(0.0000,  0.2232, 0.6939,  -0.7071, 0.0000, 0.0000, 0.7071);

  // ======================== Pick and Place Sequence ================================

  std_msgs::msg::Bool open_msg;
  open_msg.data = false;

  RCLCPP_INFO(logger, "Opening gripper");

  for (int i = 0; i < 10; i++)
  {
    gripper_pub->publish(open_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::sleep_for(std::chrono::seconds(1));
  
  RCLCPP_INFO(logger, "Moving to wp0...");
  moveToPose(mgi, wp0, logger);

  RCLCPP_INFO(logger, "Moving to prePick...");
  moveToPose(mgi, prePick, logger);

  RCLCPP_INFO(logger, "Descending to pick (Cartesian)...");
  moveToPose(mgi, pick, logger);

  std_msgs::msg::Bool close_msg;
  close_msg.data = true;

  RCLCPP_INFO(logger, "Closing gripper");

  // Publish multiple times
  for (int i = 0; i < 10; i++)
  {
    gripper_pub->publish(close_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(logger, "Moving to WP2...");
  moveToPose(mgi, wp2, logger);

  RCLCPP_INFO(logger, "Lifting...");
  moveToPose(mgi, prePlace, logger);

  RCLCPP_INFO(logger, "Opening gripper");

  for (int i = 0; i < 10; i++)
  {
    gripper_pub->publish(open_msg);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::sleep_for(std::chrono::seconds(1));

  // RCLCPP_INFO(logger, "Moving to place position...");
  // moveToPose(mgi, place, logger);

  // RCLCPP_INFO(logger, "Descending to drop...");
  // moveToPose(mgi, drop, logger);

  // // TODO: call your gripper open service/topic here
  // RCLCPP_INFO(logger, "[Hook] Open gripper here");
  // rclcpp::sleep_for(std::chrono::seconds(1));

  // RCLCPP_INFO(logger, "Returning to lift height...");
  // moveToPose(mgi, lift, logger);  // reuse lift pose as a safe retreat

  // ======================== Cleanup ================================
  rclcpp::shutdown();
  spinner.join();
  return 0;
}