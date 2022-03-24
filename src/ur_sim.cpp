#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;


int main(int argc, char** argv) {
  ros::init(argc, argv, "ur5e_sim");
  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  float x1, x2, x3, x4, x5, x6, y1, y2;
  node_handle.getParam("/robosoft/x1", x1);
  node_handle.getParam("/robosoft/x2", x2);
  node_handle.getParam("/robosoft/x3", x3);
  node_handle.getParam("/robosoft/x4", x4);
  node_handle.getParam("/robosoft/x5", x5);
  node_handle.getParam("/robosoft/x6", x6);

  node_handle.getParam("/robosoft/y1", y1);
  node_handle.getParam("/robosoft/y2", y2);

  static const std::string PLANNING_GROUP = "manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);


  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


  // ROS_INFO_NAMED("tutorial", "Pose1");
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x1;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose2");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x2;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose3");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x3;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose4");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x4;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose5");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x5;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose6");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x6;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  //  ROS_INFO_NAMED("tutorial", "Pose1");
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x1;
  // target_pose1.position.y = y2;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose2");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x2;
  // target_pose1.position.y = y2;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose3");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x3;
  // target_pose1.position.y = y2;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose4");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x4;
  // target_pose1.position.y = y2;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose5");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x5;
  // target_pose1.position.y = y2;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose6");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.w = 0.7071068;
  // target_pose1.position.x = x6;
  // target_pose1.position.y = y2;
  // target_pose1.position.z = 0.21;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();


// horizontal
  geometry_msgs::Pose target_pose1;

  // ROS_INFO_NAMED("tutorial", "Pose1");
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.x = 0.7071068;
  // target_pose1.position.x = x1-0.2;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.021;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose2");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.x = 0.7071068;
  // target_pose1.position.x = x2-0.2;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.021;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose3");
  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.x = 0.7071068;
  // target_pose1.position.x = x3-0.2;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.021;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  // ROS_INFO_NAMED("tutorial", "Pose4");

  // target_pose1.orientation.y = 0.7071068;
  // target_pose1.orientation.x = 0.7071068;
  // target_pose1.position.x = x4-0.2;
  // target_pose1.position.y = y1;
  // target_pose1.position.z = 0.021;
  // move_group_interface.setPoseTarget(target_pose1);
  // move_group_interface.move();
  // ros::Duration(2).sleep();

  ROS_INFO_NAMED("tutorial", "Pose5");
  target_pose1.orientation.y = 0.7071068;
  target_pose1.orientation.x = 0.7071068;
  target_pose1.position.x = x5-0.2;
  target_pose1.position.y = y1;
  target_pose1.position.z = 0.021;
  move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.move();
  ros::Duration(2).sleep();

  ROS_INFO_NAMED("tutorial", "Pose6");
  target_pose1.orientation.y = 0.7071068;
  target_pose1.orientation.x = 0.7071068;
  target_pose1.position.x = x6-0.2;
  target_pose1.position.y = y1;
  target_pose1.position.z = 0.021;
  move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.move();
  ros::Duration(2).sleep();

  ROS_INFO_NAMED("tutorial", "Pose1");
  target_pose1.orientation.y = 0.7071068;
  target_pose1.orientation.x = 0.7071068;
  target_pose1.position.x = x1-0.2;
  target_pose1.position.y = y2;
  target_pose1.position.z = 0.021;
  move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.move();
  ros::Duration(2).sleep();

  ROS_INFO_NAMED("tutorial", "Pose2");
  target_pose1.orientation.y = 0.7071068;
  target_pose1.orientation.x = 0.7071068;
  target_pose1.position.x = x2-0.2;
  target_pose1.position.y = y2;
  target_pose1.position.z = 0.021;
  move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.move();
  ros::Duration(2).sleep();

  ROS_INFO_NAMED("tutorial", "Pose3");
  target_pose1.orientation.y = 0.7071068;
  target_pose1.orientation.x = 0.7071068;
  target_pose1.position.x = x3-0.2;
  target_pose1.position.y = y2;
  target_pose1.position.z = 0.021;
  move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.move();
  ros::Duration(2).sleep();

  ROS_INFO_NAMED("tutorial", "Pose4");
  target_pose1.orientation.y = 0.7071068;
  target_pose1.orientation.x = 0.7071068;
  target_pose1.position.x = x4-0.2;
  target_pose1.position.y = y2;
  target_pose1.position.z = 0.02;
  move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.move();
  ros::Duration(2).sleep();

  ROS_INFO_NAMED("tutorial", "Pose5");
  target_pose1.orientation.y = 0.7071068;
  target_pose1.orientation.x = 0.7071068;
  target_pose1.position.x = x5-0.2;
  target_pose1.position.y = y2;
  target_pose1.position.z = 0.02;
  move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.move();
  ros::Duration(2).sleep();

  ROS_INFO_NAMED("tutorial", "Pose6");
  target_pose1.orientation.y = 0.7071068;
  target_pose1.orientation.x = 0.7071068;
  target_pose1.position.x = x6-0.2;
  target_pose1.position.y = y2;
  target_pose1.position.z = 0.02;
  move_group_interface.setPoseTarget(target_pose1);
  move_group_interface.move();
  ros::Duration(2).sleep();

  ros::shutdown();
  return 0;
}