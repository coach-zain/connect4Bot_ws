#include <memory>
#include <thread>
#include <vector>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>

// ======================== Helper Functions ================================

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

// Joint-space move — no IK involved, always reaches the exact same robot config
bool moveToJoints(
  moveit::planning_interface::MoveGroupInterface& mgi,
  const std::map<std::string, double>& joint_targets,
  const rclcpp::Logger& logger)
{
  mgi.setStartStateToCurrentState();
  mgi.setJointValueTarget(joint_targets);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(mgi.plan(plan));
  if (success) {
    mgi.execute(plan);
    RCLCPP_INFO(logger, "Joint move succeeded.");
  } else {
    RCLCPP_ERROR(logger, "Joint move planning failed!");
  }
  return success;
}

// Cartesian pose move — used only for pick/place poses where precision matters
bool moveToPose(
  moveit::planning_interface::MoveGroupInterface& mgi,
  const geometry_msgs::msg::Pose& target,
  const rclcpp::Logger& logger)
{
  mgi.setStartStateToCurrentState();
  mgi.setPoseTarget(target);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(mgi.plan(plan));
  if (success) {
    mgi.execute(plan);
    RCLCPP_INFO(logger, "Pose move succeeded.");
  } else {
    RCLCPP_ERROR(logger, "Pose move planning failed!");
  }
  return success;
}

void setGripper(
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub,
  bool close,
  const rclcpp::Logger& logger)
{
  std_msgs::msg::Bool msg;
  msg.data = close;
  RCLCPP_INFO(logger, "%s gripper", close ? "Closing" : "Opening");
  for (int i = 0; i < 10; i++) {
    pub->publish(msg);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  rclcpp::sleep_for(std::chrono::seconds(1));
}

// ======================== Core Pick and Place Function ================================

void pickAndPlace(
  moveit::planning_interface::MoveGroupInterface& mgi,
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_pub,
  const geometry_msgs::msg::Pose& pickPose,
  const geometry_msgs::msg::Pose& placePose,
  const std::map<std::string, double>& wp0Joints,
  const std::map<std::string, double>& wp1Joints,
  const std::map<std::string, double>& wp2Joints,
  const std::map<std::string, double>& prePlaceJoints,
  const rclcpp::Logger& logger,
  double prePickZOffset = 0.09)
{
  geometry_msgs::msg::Pose prePick = pickPose;
  prePick.position.z += prePickZOffset;

  // --- Pick sequence ---
  RCLCPP_INFO(logger, "Moving to wp0 (joint)...");
  moveToJoints(mgi, wp0Joints, logger);

  RCLCPP_INFO(logger, "Moving to wp1 (joint)...");  
  moveToJoints(mgi, wp1Joints, logger);             

  RCLCPP_INFO(logger, "Moving to prePick...");
  moveToPose(mgi, prePick, logger);

  RCLCPP_INFO(logger, "Descending to pick...");
  moveToPose(mgi, pickPose, logger);

  setGripper(gripper_pub, true, logger);

  RCLCPP_INFO(logger, "Ascending after pick...");
  moveToPose(mgi, prePick, logger);

  // --- Place sequence ---
  RCLCPP_INFO(logger, "Moving to wp1 (joint)...");  
  moveToJoints(mgi, wp1Joints, logger); 

  RCLCPP_INFO(logger, "Moving to wp2 (joint)...");
  moveToJoints(mgi, wp2Joints, logger);

  RCLCPP_INFO(logger, "Moving to prePlace (joint)...");
  moveToJoints(mgi, prePlaceJoints, logger);

  RCLCPP_INFO(logger, "Moving to place column...");
  moveToPose(mgi, placePose, logger);

  setGripper(gripper_pub, false, logger);

  RCLCPP_INFO(logger, "Retreating to prePlace (joint)...");
  moveToJoints(mgi, prePlaceJoints, logger);

  RCLCPP_INFO(logger, "Pick and place complete.");
}

// ======================== Main ================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "ur3e_pick_and_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  auto logger = rclcpp::get_logger("ur3e_pick_and_place");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  auto gripper_pub = node->create_publisher<std_msgs::msg::Bool>("/gripper_command", 10);

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface mgi(node, "ur_manipulator");
  mgi.setMaxVelocityScalingFactor(0.3);
  mgi.setMaxAccelerationScalingFactor(0.3);
  mgi.setPlanningTime(10.0);
  mgi.setWorkspace(-1.0, -1.0, -0.1, 1.0, 1.0, 1.5);

  auto current = mgi.getCurrentPose();
  RCLCPP_INFO(logger, "Current EEF: x=%.3f y=%.3f z=%.3f",
    current.pose.position.x,
    current.pose.position.y,
    current.pose.position.z);

  // ======================== Transit Poses — Joint Space ================================
  // Raw recorded values are normalized to [-pi, pi] so the planner can reach them.
  // Normalization: if value > pi, subtract 2*pi. If value < -pi, add 2*pi.
  //
  // wp0 corrections:
  //   shoulder_pan:  4.1281 - 2pi = -2.1551
  //   shoulder_lift: 3.3676 - 2pi = -2.9156
  //   wrist_1:       3.8216 - 2pi = -2.4616
  //   wrist_2:      -4.5245 + 2pi =  1.7587
  //
  // reset corrections:
  //   shoulder_lift: 4.6900 - 2pi = -1.5932
  //
  // wp2 and prePlace were already in [-pi, pi] — no changes needed.

  std::map<std::string, double> resetJoints = {
    {"elbow_joint",         0.02404579374743503},
    {"shoulder_lift_joint", -1.5932},   // 4.6900 - 2pi
    {"shoulder_pan_joint",  -0.02976396116565512},
    {"wrist_1_joint",       -1.557438005806942},
    {"wrist_2_joint",       0.030023697029965067},
    {"wrist_3_joint",       -0.014615995228060947}
  };

  std::map<std::string, double> wp0Joints = {
    {"elbow_joint",         0.9688706176852158},
    {"shoulder_lift_joint", -2.9156},   // 3.3676 - 2pi
    {"shoulder_pan_joint",  -2.1551},   // 4.1281 - 2pi
    {"wrist_1_joint",       -2.4616},   // 3.8216 - 2pi
    {"wrist_2_joint",       1.7587},    // -4.5245 + 2pi
    {"wrist_3_joint",       0.5066508667981687}
  };

    std::map<std::string, double> wp1Joints = {
    {"elbow_joint",         -1.4082554397536577},
    {"shoulder_lift_joint", -1.9788557558037776},
    {"shoulder_pan_joint",  -2.5293148632113325},
    {"wrist_1_joint",       -1.2885034904065222},
    {"wrist_2_joint",       1.5855746760621088},
    {"wrist_3_joint",      0.4416724750908781 }
  };

  std::map<std::string, double> wp2Joints = {
    {"elbow_joint",         1.12389154566365},
    {"shoulder_lift_joint", -1.0441440195981195},
    {"shoulder_pan_joint",  0.5601556502901898},
    {"wrist_1_joint",       -1.6904117410028279},
    {"wrist_2_joint",       -1.4814113477976116},
    {"wrist_3_joint",       -0.69140612923975}
  };


  std::map<std::string, double> prePlaceJoints = {
    {"elbow_joint",         0.6021024453699536},
    {"shoulder_lift_joint", -0.8726952752587962},
    {"shoulder_pan_joint",  1.089711777740749},
    {"wrist_1_joint",       -1.3085565988704506},
    {"wrist_2_joint",       -1.596205272481035},
    {"wrist_3_joint",       -1.8889134135014995}
  };

  // ======================== Pick Poses — Cartesian ================================

  auto pick1 = makePose(0.3344,  0.0560, 0.0581, -0.6501, 0.7596, 0.0059, 0.0161); // 0,0
  auto pick2 = makePose(0.3361,  0.0138, 0.0581, -0.6501, 0.7596, 0.0057, 0.0161); // -1,0
  auto pick3 = makePose(0.3367, -0.0261, 0.0596, -0.6501, 0.7596, 0.0057, 0.0160); // -2,0
  auto pick4 = makePose(0.2956,  0.0138, 0.0581, -0.6502, 0.7595, 0.0056, 0.0159); // -1,1
  auto pick5 = makePose(0.3369, -0.1101, 0.0581, -0.6501, 0.7597, 0.0055, 0.0159); // -4,0

  // ======================== Place Poses — Cartesian ================================

  auto place1 = makePose(0.0932, 0.4568, 0.2925, 0.7620, 0.6475, 0.0088, 0.0099); 
  auto place2 = makePose(0.0590, 0.4583, 0.2991, 0.7620, 0.6475, 0.0088, 0.0099);
  auto place3 = makePose(0.0295, 0.4598, 0.3013, 0.7620, 0.6475, 0.0088, 0.0099);

  // ======================== Open gripper before starting ================================

  setGripper(gripper_pub, false, logger);

  // ======================== Pick and Place Sequence ================================

  pickAndPlace(mgi, gripper_pub, pick1, place1, wp0Joints, wp1Joints, wp2Joints, prePlaceJoints, logger);
  pickAndPlace(mgi, gripper_pub, pick2, place2, wp0Joints, wp1Joints, wp2Joints, prePlaceJoints, logger);
  pickAndPlace(mgi, gripper_pub, pick3, place3, wp0Joints, wp1Joints, wp2Joints, prePlaceJoints, logger);

  // ======================== Return to reset ================================

  RCLCPP_INFO(logger, "Returning to reset...");
  moveToJoints(mgi, resetJoints, logger);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}