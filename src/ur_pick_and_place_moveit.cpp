#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.141592

int main(int argc, char * argv[])
{

  // Create a ROS logger
  auto const LOGGER = rclcpp::get_logger("ur_pick_and_place_moveit");

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("ur_pick_and_place_moveit", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // Make plane to appropriate path planning
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm.getPlanningFrame();
  collision_object.id = "ground";

  shape_msgs::msg::Plane plane;
  plane.coef = {0, 0, 1, 0}; // Equation of plane z = 0

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.z = -0.01; // z position

  collision_object.planes.push_back(plane);
  collision_object.plane_poses.push_back(ground_pose);

  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  planning_scene_interface.addCollisionObjects(collision_objects);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  move_group_arm.setStartStateToCurrentState();

  // Set UR3e to Home State 
  RCLCPP_INFO(LOGGER, "Going Home");

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -PI/2; // Shoulder Lift
  joint_group_positions_arm[2] = 0.00;  // Elbow
  joint_group_positions_arm[3] = 0.00; // Wrist 1
  joint_group_positions_arm[4] = 0.00; // Wrist 2
  //joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm1;
  bool success = (move_group_arm.plan(my_plan_arm1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm1);

  rclcpp::sleep_for(std::chrono::seconds(1));

  // Pregrasp - Move to Picking Point
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  
  tf2::Quaternion orientation;
  orientation.setRPY(0, -PI, 0); // Set Rotation Roll Pitch Yaw
  geometry_msgs::msg::Quaternion ros_orientation;

  ros_orientation = tf2::toMsg(orientation);  // tf2 Quaternion -> ROS msg

  // Assign Quaternion
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation = ros_orientation;

  //target_pose1.orientation.x = 0.00;
  //target_pose1.orientation.y = 1.00;
  //target_pose1.orientation.z = 0.00;
  //target_pose1.orientation.w = 0.00;

  target_pose1.position.x = 0.010;
  target_pose1.position.y = 0.410;
  target_pose1.position.z = 0.264;
  move_group_arm.setPoseTarget(target_pose1);
  success = (move_group_arm.plan(my_plan_arm1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm1);

  rclcpp::sleep_for(std::chrono::seconds(3));


  // Approach - Get close to object along the Cartesian Path
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints1;
  target_pose1.position.z -= 0.04;
  approach_waypoints1.push_back(target_pose1);

  target_pose1.position.z -= 0.04;
  approach_waypoints1.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach1;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints1, eef_step, jump_threshold, trajectory_approach1);

  move_group_arm.execute(trajectory_approach1);

  rclcpp::sleep_for(std::chrono::seconds(1));

  // Grasp - Add Grasping action here


  // Retreat - Lift up the object along the Cartesian Path
  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints1;
  target_pose1.position.z += 0.04;
  retreat_waypoints1.push_back(target_pose1);

  target_pose1.position.z += 0.04;
  retreat_waypoints1.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat1;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints1, eef_step, jump_threshold, trajectory_retreat1);

  move_group_arm.execute(trajectory_retreat1);
  
  rclcpp::sleep_for(std::chrono::seconds(1));

  // Carry - Move to Placing Point
  RCLCPP_INFO(LOGGER, "Move to Placing Point");

  std::vector<geometry_msgs::msg::Pose> carry_waypoints;
  target_pose1.position.x -= 0.350;
  target_pose1.position.y -= 0.100;
  carry_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_carry;

  fraction = move_group_arm.computeCartesianPath(
      carry_waypoints, eef_step, jump_threshold, trajectory_carry);

  move_group_arm.execute(trajectory_carry);

  rclcpp::sleep_for(std::chrono::seconds(3));


  // Approach - Get close to object along the Cartesian Path
  RCLCPP_INFO(LOGGER, "Approach to object!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints2;
  target_pose1.position.z -= 0.04;
  approach_waypoints2.push_back(target_pose1);

  target_pose1.position.z -= 0.04;
  approach_waypoints2.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach2;

  fraction = move_group_arm.computeCartesianPath(
      approach_waypoints2, eef_step, jump_threshold, trajectory_approach2);

  move_group_arm.execute(trajectory_approach2);

  rclcpp::sleep_for(std::chrono::seconds(1));

  // Dropping - Add Dropping action here

  // Retreat - Lift up the object along the Cartesian Path
  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints2;
  target_pose1.position.z += 0.04;
  retreat_waypoints2.push_back(target_pose1);

  target_pose1.position.z += 0.04;
  retreat_waypoints2.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat2;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints2, eef_step, jump_threshold, trajectory_retreat2);

  move_group_arm.execute(trajectory_retreat2);

  rclcpp::sleep_for(std::chrono::seconds(1));
  

  // Get back UR3e to Home State 
  RCLCPP_INFO(LOGGER, "Going Home");

  current_state_arm = move_group_arm.getCurrentState(10);

  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  move_group_arm.setStartStateToCurrentState();

  joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
  joint_group_positions_arm[1] = -PI/2; // Shoulder Lift
  joint_group_positions_arm[2] = 0.00;  // Elbow
  joint_group_positions_arm[3] = 0.00; // Wrist 1
  joint_group_positions_arm[4] = 0.00; // Wrist 2
  //joint_group_positions_arm[5] = 0.00;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm2;
  success = (move_group_arm.plan(my_plan_arm2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_arm.execute(my_plan_arm2);

  rclcpp::sleep_for(std::chrono::seconds(1));

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
