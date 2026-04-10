#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

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

  // ======================== MoveGroupInterface Setup ================================

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mgi(node, "ur_manipulator");

  mgi.setMaxVelocityScalingFactor(0.3);    // 30% speed — safe for real hw
  mgi.setMaxAccelerationScalingFactor(0.3);
  mgi.setPlanningTime(10.0);               // seconds to allow for planning

  auto current = mgi.getCurrentPose();
  RCLCPP_INFO(logger, "Current EEF: x=%.3f y=%.3f z=%.3f",
    current.pose.position.x,
    current.pose.position.y,
    current.pose.position.z);

  // ============================= Define poses =======================================
  auto wp0      = makePose(0.3290, 0.2910, 0.3440,  0.8485, -0.4986, -0.0867, -0.1549); // between reset and prePick
  auto prePick  = makePose(0.3223, 0.1641, 0.113,  0.9087, -0.4170, -0.0084, 0.0150);  // just above pick
  auto pick     = makePose(0.3222, 0.1618, 0.0557,  0.9087, -0.4171, -0.0085, 0.0150);  // drop down to pick
  auto wp1      = makePose(0.0, -0.3, 0.10,  1.0, 0.0, 0.0, 0.0);
  auto prePlace = makePose(0.2,  0.0, 0.25,  1.0, 0.0, 0.0, 0.0);
  auto place    = makePose(0.0, -0.3, 0.25,  1.0, 0.0, 0.0, 0.0);
  auto reset    = makePose(0.0000,  0.2232, 0.6939,  -0.7071, 0.0000, 0.0000, 0.7071);

  // ======================== Pick and Place Sequence ================================

  RCLCPP_INFO(logger, "Moving to wp0...");
  moveToPose(mgi, wp0, logger);

  RCLCPP_INFO(logger, "Moving to prePick...");
  moveToPose(mgi, prePick, logger);

  RCLCPP_INFO(logger, "Descending to pick (Cartesian)...");
  moveToPose(mgi, pick, logger);

  // // TODO: call your gripper close service/topic here
  // // e.g. publish to /gripper_control or call your servo node
  // RCLCPP_INFO(logger, "[Hook] Close gripper here");
  // rclcpp::sleep_for(std::chrono::seconds(1));

  // RCLCPP_INFO(logger, "Lifting...");
  // moveToPose(mgi, lift, logger);

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