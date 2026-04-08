#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>

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
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "ur3e_pick_and_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto logger = rclcpp::get_logger("ur3e_pick_and_place");

  // ── Critical: spin the node so getCurrentState() / planning works ──
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // ── MoveGroup for the UR3e ──────────────────────────────────────────
  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mgi(node, "ur_manipulator");

  mgi.setMaxVelocityScalingFactor(0.3);    // 30% speed — safe for real hw
  mgi.setMaxAccelerationScalingFactor(0.3);
  mgi.setPlanningTime(10.0);               // seconds to allow for planning

  // ── Log where the robot currently is ───────────────────────────────
  auto current = mgi.getCurrentPose();
  RCLCPP_INFO(logger, "Current EEF: x=%.3f y=%.3f z=%.3f",
    current.pose.position.x,
    current.pose.position.y,
    current.pose.position.z);

  // ── Define your poses ───────────────────────────────────────────────
  // These are placeholders — replace with values from RViz/your setup.
  // Orientation here keeps the end effector pointing straight down.
  // (qx=1, qw=0 flips Z down — tune to match your TCP orientation)

  auto pre_grasp = makePose(0.1645,  0.0473, 0.6589,  -0.7071, 0.0005, 0.0006, 0.7071);
  auto grasp     = makePose(0.0000,  0.2232, 0.6939,  -0.7071, 0.0000, 0.0000, 0.7071);
  auto lift      = makePose(0.2,  0.0, 0.25,  1.0, 0.0, 0.0, 0.0);
  auto place     = makePose(0.0, -0.3, 0.25,  1.0, 0.0, 0.0, 0.0);
  auto drop      = makePose(0.0, -0.3, 0.10,  1.0, 0.0, 0.0, 0.0);

  // ── Pick and place sequence ─────────────────────────────────────────
  RCLCPP_INFO(logger, "Moving to pre-grasp...");
  moveToPose(mgi, pre_grasp, logger);

  RCLCPP_INFO(logger, "Descending to grasp...");
  moveToPose(mgi, grasp, logger);

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

  // ── Cleanup ─────────────────────────────────────────────────────────
  rclcpp::shutdown();
  spinner.join();
  return 0;
}