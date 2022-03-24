#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import yaml
from dynamixel_workbench_msgs.srv import DynamixelCommand
from dynamixel_workbench_msgs.msg import DynamixelStateList
import rospkg
import copy

class FrankaRobot(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospack = rospkg.RosPack()
        self.yaml_path = rospack.get_path('robosoft') + '/resources/'

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        self.eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        # self.group_names = self.robot.get_group_names()
        # print("============ Available Planning Groups:", self.robot.get_group_names())

        # print("============ Printing robot state")
        # print(self.robot.get_current_state())
        print("")

        self.box_name = ""

        self.id = 1
        self.addr_name = 'Goal_Position'
        self.grasp_position = 200
        self.open_position = 1300

        print("============ Waiting for dynamixel")
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        self.dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        print("============ Dynamixel srv available")

    def read_vector_from_yaml(self, file):
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)
        return(dict.get('positions')[0][0])

    def read_pose_from_yaml(self, file):
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)
        vector = dict.get('positions')[0][0]
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = vector[0]
        pose_goal.position.y = vector[1]
        pose_goal.position.z = vector[2]
        pose_goal.orientation.x = vector[3]
        pose_goal.orientation.y = vector[4]
        pose_goal.orientation.z = vector[5]
        pose_goal.orientation.w = vector[6]
        return pose_goal

    def go_to_joint_state(self, joint_goal, wait=True):
        self.move_group.go(joint_goal, wait)
        self.move_group.stop()

    def go_to_pose_goal(self, pose_goal, wait=True):
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def plan_cartesian_path(self, scale=1):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        wpose.position.z -= 0.02
        wpose.position.y += 0.02
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += 0.02
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= 0.02
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  
        )  

        self.move_group.execute(plan, wait=True)

    def go_to_position_perpendicular(self, position):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        pose_goal.orientation.x = 1
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0
        self.go_to_pose_goal(pose_goal)

    def go_to_position_parallel(self, position):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]
        pose_goal.orientation.x = -0.7071068
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0.7071068
        self.go_to_pose_goal(pose_goal)

    def pour(self):
        joint_goal = self.move_group.get_current_joint_values()

        # for i in range(3):
        joint_goal[-1] -= 1
        self.go_to_joint_state(joint_goal)

    def grasp(self, move=False):
        if move:
            pose_goal = self.move_group.get_current_pose().pose
            pose_goal.position.y += 0.05
            self.go_to_pose_goal(pose_goal, False)

        self.dynamixel_command(
            '', self.id, self.addr_name, self.grasp_position)

    def open(self):
        self.dynamixel_command('', self.id, self.addr_name, self.open_position)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = self.box_name in self.scene.get_known_object_names()
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def add_box(self, timeout=4):
        rospy.sleep(2)

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_link0"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0.64 
        box_pose.pose.position.z = 0.26 
        self.box_name = "box"
        self.scene.add_box(self.box_name, box_pose, size=(1.20, 0.60, 0.50))

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
