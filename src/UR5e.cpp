#include "UR5e.h"

UR5e::UR5e (ros::NodeHandle& nodeHandle) {

    joint_ = false;
    pose_ = false;
    path_ = false;
    position_ = false;
    pour_ = false;
    open_ = false;
    close_ = false;

    velocity_scale_ = 1;
    acceleration_scale_ = 1;

    openPosition_ = 1300;
    closedPosition_ = 200;

    dynamixelCommand_.request.id = 1;
    dynamixelCommand_.request.addr_name = "Goal_Position";


    goToJointGoalSrv_ = nodeHandle.advertiseService("/go_to_joint_goal",
        &UR5e::goToJointGoalCallback, this);
    goToPoseGoalSrv_ = nodeHandle.advertiseService("/go_to_pose_goal",
        &UR5e::goToPoseGoalCallback, this);
    planCartesianPathSrv_ = nodeHandle.advertiseService("/plan_cartesian_path",
        &UR5e::planCartesianPathCallback, this);
    goToPositionSrv_ = nodeHandle.advertiseService("/go_to_position_goal",
        &UR5e::goToPositionCallback, this);
    pourSrv_ = nodeHandle.advertiseService("/pour",
        &UR5e::pourCallback, this);
    openGripperSrv_ = nodeHandle.advertiseService("/open_gripper",
        &UR5e::openGripperCallback, this);
    closeGripperSrv_ = nodeHandle.advertiseService("/close_gripper",
        &UR5e::closeGripperCallback, this);

    dynamixelCommandClient_ = nodeHandle.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    missionDoneClient_ = nodeHandle.serviceClient<std_srvs::Trigger>("/mission_done");

    nodeHandle.getParam("/robosoft/planning_group", planning_group_);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface> (planning_group_);

    std::cout << "Reference frame: " << move_group_->getPlanningFrame() << std::endl;
    std::cout << "End effector link: " << move_group_->getEndEffectorLink() << std::endl;

    // Add collisions
    ros::Duration(2).sleep();
    // float z_board;
    // nodeHandle.getParam("/robosoft/z", z_board);
    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // collision_objects.push_back(addBox("table", 1,2,0.35,0.3,0,0));
    // collision_objects.push_back(addBox("board", 0.53,1.27,z_board+0.01,0.55,0,0));
    // planning_scene_interface_.addCollisionObjects(collision_objects);
    // std::cout << "Added collision" << std::endl;
    // std::cout << "Robot is ready!" << std::endl;
}

void UR5e::control() {
    while (ros::ok()) {
        if (joint_) {
            goToJointGoal();
            missionDoneClient_.call(trigger);
        }
        else if (pose_){
            goToPoseGoal();
            missionDoneClient_.call(trigger);
        }
        else if (path_){
            planCartesianPath();
            missionDoneClient_.call(trigger);
        }
        else if (position_){
            goToPosition();
            missionDoneClient_.call(trigger);
        }
        else if (pour_){
            pour();
            missionDoneClient_.call(trigger);
        }
        else if (open_){
            openGripper();
            missionDoneClient_.call(trigger);
        }
        else if (close_){
            closeGripper();
            missionDoneClient_.call(trigger);
        }
    }
}

void UR5e::goToJointGoal() {
    std::cout << "Moving to joint state" << std::endl;
    try {
        move_group_->clearPoseTargets();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit_msgs::MoveItErrorCodes errorCode;

        move_group_->setJointValueTarget(jointPositionReference_);
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        errorCode = move_group_->move();
        std::cout << "Done" << std::endl;

        if (errorCode.val < 0) {
            throw std::runtime_error("Did not move!");
        }

        joint_ = false;


    } catch (const std::runtime_error& e) {
        std::cout << "Runtime error. Aborting trajectory." << std::endl;
    }
}

void UR5e::goToPoseGoal() {
    std::cout << "Moving to pose goal" << std::endl;
    try{
        move_group_->clearPoseTargets();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(poseReference_);
        move_group_->move();
        std::cout << "Done" << std::endl;

        pose_ = false;

    } catch (const std::runtime_error& e) {
        std::cout << "Runtime error. Aborting trajectory." << std::endl;
    }
}

void UR5e::planCartesianPath() {
    std::cout << "Planning cartesian path" << std::endl;
    try{
        move_group_->clearPoseTargets();
        moveit_msgs::RobotTrajectory trajectory;
        moveit_msgs::RobotTrajectory trajectory_slow;

        const double jump_threshold = 5;
        const double eef_step = 0.5;
        double fraction = move_group_->computeCartesianPath(posesReference_, eef_step, jump_threshold, trajectory);
        trajectory_processing::IterativeParabolicTimeParameterization iptp(100, 0.05);
        robot_trajectory::RobotTrajectory r_trajec(move_group_->getRobotModel(), planning_group_);
        r_trajec.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);
        iptp.computeTimeStamps(r_trajec, velocity_scale_, acceleration_scale_);
        r_trajec.getRobotTrajectoryMsg(trajectory_slow);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory_slow;
        move_group_->setStartStateToCurrentState();
        move_group_->execute(plan);
        std::cout << "Done" << std::endl;

        path_ = false;

    } catch (const std::runtime_error& e) {
        std::cout << "Runtime error. Aborting trajectory." << std::endl;
    }
}

void UR5e::goToPosition() {
    std::cout << "Moving to position goal" << std::endl;
    try{
        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();

        geometry_msgs::Pose poseRef;
        poseRef.position = positionReference_;

        if(parallel_) 
        {
            poseRef.orientation.x = -0.7071068;
            poseRef.orientation.y = 0;
            poseRef.orientation.z = 0;
            poseRef.orientation.w = 0.7071068;

        } 
        else if (perpendicular_) {
            poseRef.orientation.x = 1;
            poseRef.orientation.y = 0;
            poseRef.orientation.z = 0;
            poseRef.orientation.w = 0;
        }

        else if (perpendicular_rotate_) {
            poseRef.orientation.x = 0;
            poseRef.orientation.y = 1;
            poseRef.orientation.z = 0;
            poseRef.orientation.w = 0;  
        }
        move_group_->setPoseTarget(poseRef);
        move_group_->move();
        std::cout << "Done" << std::endl;

        position_ = false;

    } catch (const std::runtime_error& e) {
        std::cout << "Runtime error. Aborting trajectory." << std::endl;
    }
}

void UR5e::pour() {
    std::cout << "Pouring whiskey" << std::endl;
    std::cout << "Not implemented" << std::endl;

    pour_ = false;
}

void UR5e::openGripper() {
    std::cout << "Opening gripper" << std::endl;
    dynamixelCommand_.request.value = openPosition_;
    dynamixelCommandClient_.call(dynamixelCommand_);

    ros::Duration(2).sleep();
    open_ = false;
}

void UR5e::closeGripper() {
    std::cout << "Closing gripper" << std::endl;
    dynamixelCommand_.request.value = closedPosition_;
    dynamixelCommandClient_.call(dynamixelCommand_);

    if (graspMove_) {
        geometry_msgs::PoseStamped current = move_group_->getCurrentPose();
        current.pose.position.x += 0.05;
        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(current.pose);
        move_group_->move();
        std::cout << "Done" << std::endl;
    }

    ros::Duration(2).sleep();
    close_ = false;
}

bool UR5e::goToJointGoalCallback(robosoft::jointGoal::Request &req, robosoft::jointGoal::Response &res){
    joint_ = true;
    jointPositionReference_ = req.joint_states;
    return true;
}

bool UR5e::goToPoseGoalCallback(robosoft::poseGoal::Request &req, robosoft::poseGoal::Response &res){
    pose_ = true;
    poseReference_ = req.pose;
    return true;
}

bool UR5e::planCartesianPathCallback(robosoft::cartesianPath::Request &req, robosoft::cartesianPath::Response &res){
    path_ = true;
    posesReference_ = req.poses;
    return true;
}

bool UR5e::goToPositionCallback(robosoft::positionGoal::Request &req, robosoft::positionGoal::Response &res){
    position_ = true;
    positionReference_ = req.position;
    parallel_ = req.parallel;
    perpendicular_ = req.perpendicular;
    perpendicular_rotate_ = req.perpendicular_rotate;
    return true;
}

bool UR5e::pourCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    pour_ = true;
    return true;
}

bool UR5e::openGripperCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    open_ = true;
    return true;
}

bool UR5e::closeGripperCallback(robosoft::closeGripper::Request &req, robosoft::closeGripper::Response &res){
    close_ = true;
    graspMove_ = req.move;
    return true;
}

// args: collision object id, box size, position in world frame (min x, y, min z)
moveit_msgs::CollisionObject UR5e::addBox(const char* name, float box_x, float box_y, float box_z, float x, float y, float z) {
  moveit_msgs::CollisionObject collision_object;

  collision_object.header.frame_id = "world";
  collision_object.id = name;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = box_x;
  primitive.dimensions[1] = box_y;
  primitive.dimensions[2] = box_z;

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1;
  box_pose.position.x = x + box_x/2;
  box_pose.position.y = y;
  box_pose.position.z = z + box_z/2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "UR5e");
    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    UR5e robot = UR5e(nodeHandle);
    robot.control();
}