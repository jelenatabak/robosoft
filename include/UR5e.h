#pragma once

#include <ros/package.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <stdexcept>
#include <math.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <robosoft/jointGoal.h>
#include <robosoft/poseGoal.h>
#include <robosoft/cartesianPath.h>
#include <robosoft/positionGoal.h>
#include <robosoft/grasp.h>

#include <dynamixel_workbench_msgs/DynamixelCommand.h>

class UR5e {
  public:
    UR5e(ros::NodeHandle& nodeHandle);
    bool goToJointGoalCallback(robosoft::jointGoal::Request &req, robosoft::jointGoal::Response &res);
    bool goToPoseGoalCallback(robosoft::poseGoal::Request &req, robosoft::poseGoal::Response &res);
    bool planCartesianPathCallback(robosoft::cartesianPath::Request &req, robosoft::cartesianPath::Response &res);
    bool goToPositionCallback(robosoft::positionGoal::Request &req, robosoft::positionGoal::Response &res);
    bool pourCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool openGripperCallback(robosoft::grasp::Request &req, robosoft::grasp::Response &res);
    bool closeGripperCallback(robosoft::grasp::Request &req, robosoft::grasp::Response &res);

    moveit_msgs::CollisionObject addBox(const char* name, float box_x, float box_y, float box_z, float x, float y, float z);
    void attachObject(float obj_r, float obj_z);

    void control();
    void goToJointGoal();
    void goToPoseGoal();
    void planCartesianPath();
    void goToPosition();
    void pour();
    void openGripper();
    void closeGripper();

  private:
    ros::NodeHandle nodeHandle;
    ros::ServiceServer goToJointGoalSrv_;
    ros::ServiceServer goToPoseGoalSrv_;
    ros::ServiceServer planCartesianPathSrv_;
    ros::ServiceServer goToPositionSrv_;
    ros::ServiceServer pourSrv_;
    ros::ServiceServer openGripperSrv_;
    ros::ServiceServer closeGripperSrv_;

    ros::ServiceClient dynamixelCommandClient_;
    ros::ServiceClient dynamixelCommandWaitClient_;
    ros::ServiceClient missionDoneClient_;
    std_srvs::Trigger trigger;

    int openPosition_;
    int closedPosition_;
    dynamixel_workbench_msgs::DynamixelCommand dynamixelCommand_;

    bool joint_, pose_, path_, position_, pour_, open_, close_;

    std::vector<double> jointPositionReference_;
    geometry_msgs::Pose poseReference_;
    std::vector<geometry_msgs::Pose> posesReference_;
    geometry_msgs::Point positionReference_;
    bool parallel_, perpendicular_, perpendicular_rotate_, constraint_;
    geometry_msgs::Point move_;

    std::string planning_group_;
    std::shared_ptr <moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    double velocity_scale_, acceleration_scale_;
};