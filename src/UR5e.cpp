#include "UR5e.h"

UR5e::UR5e (ros::NodeHandle& nodeHandle) {

    joint_ = false;
    pose_ = false;
    path_ = false;
    position_ = false;
    pour_ = false;
    open_ = false;
    close_ = false;
    constraint_ = false;

    velocity_scale_ = 1;
    acceleration_scale_ = 1;

    openPosition_ = 1700;
    closedPosition_ = 700;

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
    dynamixelCommandWaitClient_ = nodeHandle.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command_wait");
    missionDoneClient_ = nodeHandle.serviceClient<std_srvs::Trigger>("/mission_done");

    nodeHandle.getParam("/robosoft/planning_group", planning_group_);
    nodeHandle.getParam("/robosoft/pour_angle", pour_angle_);
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface> (planning_group_);

    move_group_->setStartStateToCurrentState();
    // move_group_->setNumPlanningAttempts(10);
    // move_group_->setPlanningTime(20);
    move_group_->clearPathConstraints();

    std::cout << "Reference frame: " << move_group_->getPlanningFrame() << std::endl;
    std::cout << "End effector link: " << move_group_->getEndEffectorLink() << std::endl;
    std::cout << "Pose: " << move_group_->getCurrentPose() << std::endl;
    std::vector<double> states = move_group_->getCurrentJointValues();


    // Add collisions
    ros::Duration(2).sleep();
    float z_board;
    nodeHandle.getParam("/robosoft/z", z_board);
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(addBox("board", 2,2,0.01,-1,0,-0.02));      // TODO baord je ispod stvarnog da omoguci kontakt
    // collision_objects.push_back(addBox("board2", 2,0.7,0.01,-1,-0.82,0.095));
    // collision_objects.push_back(addBox("wall", 2,0.2,2,-1,0.75,0));
    planning_scene_interface_.addCollisionObjects(collision_objects);
    std::cout << "Added collision" << std::endl;
    std::cout << "Robot is ready!" << std::endl;
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
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
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
        tf2::Quaternion quat;
        tf2::fromMsg(poseReference_.orientation, quat);
        quat.normalize();
        geometry_msgs::Quaternion quat_msg;
        quat_msg = tf2::toMsg(quat);
        poseReference_.orientation = quat_msg;
        move_group_->clearPoseTargets();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
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
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
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
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);
    try{
        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();

        geometry_msgs::Pose poseRef;
        poseRef.position = positionReference_;

        if(parallel_) 
        {
            if(constraint_) {
                constraint_ = false;
                moveit_msgs::OrientationConstraint ocm;
                ocm.link_name = "tool0";
                ocm.header.frame_id = "base_link";
                ocm.orientation.x = 0.5927576;
                ocm.orientation.y = 0.3918398;
                ocm.orientation.z = -0.3928394;
                ocm.orientation.w = 0.5837613;
                ocm.absolute_x_axis_tolerance = 0.2;
                ocm.absolute_y_axis_tolerance = 0.2;
                ocm.absolute_z_axis_tolerance = 0.2;
                ocm.weight = 1.0;

                moveit_msgs::Constraints test_constraints;
                test_constraints.orientation_constraints.push_back(ocm);
                move_group_->setPathConstraints(test_constraints);

            }
            poseRef.orientation.x = 0.5927576;
            poseRef.orientation.y = 0.3918398;
            poseRef.orientation.z = -0.3928394;
            poseRef.orientation.w = 0.5837613;
        } 
        else if (perpendicular_) {
            poseRef.orientation.x = 0.823;
            poseRef.orientation.y = 0.567;
            poseRef.orientation.z = 0.023;
            poseRef.orientation.w = 0.001;
        }

        else if (perpendicular_rotate_) {
            poseRef.orientation.x = 0;
            poseRef.orientation.y = 1;
            poseRef.orientation.z = 0;
            poseRef.orientation.w = 0;  
        }

        tf2::Quaternion quat;
        tf2::fromMsg(poseRef.orientation, quat);
        quat.normalize();
        geometry_msgs::Quaternion quat_msg;
        quat_msg = tf2::toMsg(quat);
        poseRef.orientation = quat_msg;

        std::cout << poseRef << std::endl;

        move_group_->setPoseTarget(poseRef);
        // move_group_->setApproximateJointValueTarget(poseRef);
        move_group_->move();

        std::cout << "Done" << std::endl;
        move_group_->clearPathConstraints();

        position_ = false;

    } catch (const std::runtime_error& e) {
        std::cout << "Runtime error. Aborting trajectory." << std::endl;
    }
}

void UR5e::pour() {
    std::cout << "Pouring whiskey" << std::endl;

    std::vector<double> jointTarget = move_group_->getCurrentJointValues();
    jointTarget.back() += pour_angle_;

    try {
        move_group_->clearPoseTargets();
        move_group_->setMaxVelocityScalingFactor(velocity_scale_);
        move_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        moveit_msgs::MoveItErrorCodes errorCode;

        move_group_->setJointValueTarget(jointTarget);
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        errorCode = move_group_->move();
        std::cout << "Done" << std::endl;

        if (errorCode.val < 0) {
            throw std::runtime_error("Did not move!");
        }

    } catch (const std::runtime_error& e) {
        std::cout << "Runtime error. Aborting trajectory." << std::endl;
    }

    pour_ = false;
}

void UR5e::openGripper() {
    std::cout << "Opening gripper" << std::endl;
    dynamixelCommand_.request.value = openPosition_;

    if (move_.x != 0 or move_.y != 0 or move_.z != 0) {
        dynamixelCommandWaitClient_.call(dynamixelCommand_);
        geometry_msgs::PoseStamped current = move_group_->getCurrentPose();
        current.pose.position.x += move_.x;
        current.pose.position.y += move_.y;
        current.pose.position.z += move_.z;
        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(current.pose);
        move_group_->move();
        std::cout << "Done" << std::endl;
    }

    dynamixelCommandClient_.call(dynamixelCommand_);
    open_ = false;
    move_.x = 0;
    move_.y = 0;
    move_.z = 0;
}

void UR5e::closeGripper() {
    std::cout << "Closing gripper" << std::endl;
    dynamixelCommand_.request.value = closedPosition_;

    if (move_.x != 0 or move_.y != 0 or move_.z != 0) {
        dynamixelCommandWaitClient_.call(dynamixelCommand_);
        geometry_msgs::PoseStamped current = move_group_->getCurrentPose();
        current.pose.position.x += move_.x;
        current.pose.position.y += move_.y;
        current.pose.position.z += move_.z;
        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(current.pose);
        move_group_->move();
        std::cout << "Done" << std::endl;
    }

    dynamixelCommandClient_.call(dynamixelCommand_);
    close_ = false;
    move_.x = 0;
    move_.y = 0;
    move_.z = 0;
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
    constraint_ = req.constraint;
    perpendicular_rotate_ = req.perpendicular_rotate;
    return true;
}

bool UR5e::pourCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    pour_ = true;
    return true;
}

bool UR5e::openGripperCallback(robosoft::grasp::Request &req, robosoft::grasp::Response &res){
    open_ = true;
    move_ = req.move;
    return true;
}

bool UR5e::closeGripperCallback(robosoft::grasp::Request &req, robosoft::grasp::Response &res){
    close_ = true;
    move_ = req.move;
    return true;
}
// args: collision object id, box size, position in world frame (min x, y, min z)
moveit_msgs::CollisionObject UR5e::addBox(const char* name, float box_x, float box_y, float box_z, float x, float y, float z) {
  moveit_msgs::CollisionObject collision_object;

  collision_object.header.frame_id = "base_link";
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
    ros::AsyncSpinner spinner(0);
    spinner.start();
    UR5e robot = UR5e(nodeHandle);
    robot.control();
}