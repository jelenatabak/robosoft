#!/usr/bin/env python3

import copy
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import yaml
from geometry_msgs.msg import Pose, Point, PointStamped
from robosoft.srv import *
from std_srvs.srv import Trigger, TriggerResponse
import rospkg


class Robosoft(object):
    def __init__(self):
        self.z = rospy.get_param('/robosoft/z')
        self.r = rospy.get_param('/robosoft/r')

        self.y1 = rospy.get_param('/robosoft/y1')
        self.y2 = rospy.get_param('/robosoft/y2')

        self.x1 = rospy.get_param('/robosoft/x1')
        self.x2 = rospy.get_param('/robosoft/x2')
        self.x3 = rospy.get_param('/robosoft/x3')
        self.x4 = rospy.get_param('/robosoft/x4')
        self.x5 = rospy.get_param('/robosoft/x5')
        self.x6 = rospy.get_param('/robosoft/x6')

        self.base_frame = rospy.get_param('/robosoft/base_frame')
        self.camera_frame = rospy.get_param('/robosoft/camera_frame')

        self.pc_topic = '/camera/depth_registered/points'

        rospack = rospkg.RosPack()
        self.yaml_path = rospack.get_path('robosoft') + '/resources/'

        rospy.wait_for_service('go_to_joint_goal')
        rospy.wait_for_service('go_to_pose_goal')
        rospy.wait_for_service('plan_cartesian_path')
        rospy.wait_for_service('go_to_position_goal')
        rospy.wait_for_service('pour')
        rospy.wait_for_service('open_gripper')
        rospy.wait_for_service('close_gripper')

        self.go_to_joint_goal_client = rospy.ServiceProxy(
            'go_to_joint_goal', jointGoal)
        self.go_to_pose_goal_client = rospy.ServiceProxy(
            'go_to_pose_goal', poseGoal)
        self.plan_cartesian_path_client = rospy.ServiceProxy(
            'plan_cartesian_path', cartesianPath)
        self.go_to_position_goal_client = rospy.ServiceProxy(
            'go_to_position_goal', positionGoal)
        self.pour_client = rospy.ServiceProxy('pour', Trigger)
        self.open_gripper_client = rospy.ServiceProxy('open_gripper', grasp)
        self.close_gripper_client = rospy.ServiceProxy(
            'close_gripper', grasp)

        self.test_publisher = rospy.Publisher(
            'test_pc', PointCloud2, queue_size=10)
        self.point_publisher = rospy.Publisher(
            'test_point', PointStamped, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        print('Sleeping for 3s - tf subscriber initialized')
        rospy.sleep(3)

        rospy.Service('mission_done', Trigger, self.mission_done_callback)
        self.mission_done = False

        self.pts_A_count = np.zeros(6)
        self.pts_A = []
        self.pts_B_count = np.zeros(6)
        self.pts_B = []
        self.points_list = []

    def mission_done_callback(self, req):
        self.mission_done = True
        return TriggerResponse()

    def check_mission_done(self):
        while not rospy.is_shutdown():
            if self.mission_done:
                self.mission_done = False
                return

    def read_vector_from_yaml(self, file):
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)
        return(dict.get('positions')[0][0])

    def read_vectors_from_yaml(self, file):
        joint_goals = []
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)

        l = len(dict.get('positions'))
        for i in range(l):
            joint_goals.append(dict.get('positions')[i][0])

        return joint_goals

    def read_pose_from_yaml(self, file):
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)
        vector = dict.get('positions')[0][0]
        pose_goal = Pose()
        pose_goal.position.x = vector[0]
        pose_goal.position.y = vector[1]
        pose_goal.position.z = vector[2]
        pose_goal.orientation.x = vector[3]
        pose_goal.orientation.y = vector[4]
        pose_goal.orientation.z = vector[5]
        pose_goal.orientation.w = vector[6]
        return pose_goal

    def read_poses_from_yaml(self, file):
        poses_goal = []
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)

        l = len(dict.get('positions'))
        for i in range(l):
            vector = dict.get('positions')[i][0]
            pose_goal = Pose()
            pose_goal.position.x = vector[0]
            pose_goal.position.y = vector[1]
            pose_goal.position.z = vector[2]
            pose_goal.orientation.x = vector[3]
            pose_goal.orientation.y = vector[4]
            pose_goal.orientation.z = vector[5]
            pose_goal.orientation.w = vector[6]
            poses_goal.append(pose_goal)

        return poses_goal

    def ur_count_pts(self):
        z_values = self.points['z']

        mask = ~np.isnan(z_values) * (z_values > self.z)
        pts_z = self.points[mask]
        x_values = pts_z['x']
        y_values = pts_z['y']

        mask = (x_values > self.x4-self.r) * (x_values < self.x4+self.r) * \
            (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.pts_A_count[0] += (len(pts_z[mask]))
        self.pts_A.append(pts_z[mask])

        mask = (x_values > self.x5-self.r) * (x_values < self.x5+self.r) * \
            (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.pts_A_count[1] += (len(pts_z[mask]))
        self.pts_A.append(pts_z[mask])

        mask = (x_values > self.x6-self.r) * (x_values < self.x6+self.r) * \
            (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.pts_A_count[2] += (len(pts_z[mask]))
        self.pts_A.append(pts_z[mask])

        mask = (x_values > self.x4-self.r) * (x_values < self.x4+self.r) * \
            (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.pts_A_count[3] += (len(pts_z[mask]))
        self.pts_A.append(pts_z[mask])

        mask = (x_values > self.x5-self.r) * (x_values < self.x5+self.r) * \
            (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.pts_A_count[4] += (len(pts_z[mask]))
        self.pts_A.append(pts_z[mask])

        mask = (x_values > self.x6-self.r) * (x_values < self.x6+self.r) * \
            (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.pts_A_count[5] += (len(pts_z[mask]))
        self.pts_A.append(pts_z[mask])

        mask = (x_values > self.x1-self.r) * (x_values < self.x1+self.r) * \
            (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.pts_B_count[0] += (len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x2-self.r) * (x_values < self.x2+self.r) * \
            (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.pts_B_count[1] += (len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x3-self.r) * (x_values < self.x3+self.r) * \
            (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.pts_B_count[2] += (len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x1-self.r) * (x_values < self.x1+self.r) * \
            (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.pts_B_count[3] += (len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x2-self.r) * (x_values < self.x2+self.r) * \
            (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.pts_B_count[4] += (len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x3-self.r) * (x_values < self.x3+self.r) * \
            (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.pts_B_count[5] += (len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        print('Done with pc filtering')
        print('Pts A count: ', self.pts_A_count)
        print('Pts B count: ', self.pts_B_count)


    def record(self, joint_goals):
        for i in range(len(joint_goals)):
            self.go_to_joint_goal_client.call(joint_goals[i])
            self.check_mission_done()
            print("Recording from state ", i)

            print('Waiting for pc message')
            self.pc_msg = rospy.wait_for_message(self.pc_topic, PointCloud2)
            print('Recieved pc message')

            print('Waiting for tf')
            try:
                trans = self.tfBuffer.lookup_transform(
                    self.base_frame, self.camera_frame, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('Did not get transform')

            self.pc_glob = do_transform_cloud(self.pc_msg, trans)
            print('Transformed point cloud')

            points = ros_numpy.point_cloud2.pointcloud2_to_array(self.pc_glob)
            self.points_list.append(points)

        self.points = np.concatenate(self.points_list)


    def get_positions(self, A_num, B_num):
        A_ind = np.sort(np.argpartition(self.pts_A_count, -A_num)[-A_num:])
        B_ind = np.sort(np.argpartition(self.pts_B_count, -B_num)[-B_num:])

        print("Object positions in A: ", A_ind)
        print("Object positions in B: ", B_ind)

        return A_ind, B_ind

    def get_centroid(self, pc):
        x_values = pc['x']
        y_values = pc['y']
        z_values = pc['z']

        c = Point()
        c.x = np.sum(x_values)/len(x_values)
        c.y = np.sum(y_values)/len(y_values)
        c.z = np.sum(z_values)/len(z_values)

        print('Object centroid: ', c)
        return c

    def get_dimensions(self, pc):
        x_values = pc['x']
        y_values = pc['y']
        z_values = pc['z']

        x = abs(max(x_values) - min(x_values))
        y = abs(max(y_values) - min(y_values))
        z = abs(max(z_values) - min(z_values))
        return [x, y, z]

    def get_min(self, pc, axis):
        values = pc[axis]
        return min(values)

    def get_max(self, pc, axis):
        values = pc[axis]
        return max(values)

    def filter(self, pc, axis, min, max):
        values = pc[axis]
        mask = (values > min) * (values < max) 
        filtered = pc[mask]
        return filtered


    def grasp(self, pc):
        ret_value = 0

        centroid_pt = self.get_centroid(pc)
        dimensions = self.get_dimensions(pc)

        min_z = 0.02
        max_y = 0.2
        min_x = 0.03  # grasp small objects with fingertip

        req = positionGoalRequest()
        req_gripper = closeGripperRequest()

        if dimensions[2] > min_z and dimensions[1] < max_y:
            print("Grasping parallel")

            req.parallel = True
            req.perpendicular = False
            req.perpendicular_rotate = False

            approach_pt = copy.deepcopy(centroid_pt)
            approach_pt.x -= 0.4
            grasp_pt = copy.deepcopy(centroid_pt)
            if dimensions[0] < min_x:
                grasp_pt.x -= 0.20
            else:
                grasp_pt.x -= 0.19

            up_pt = copy.deepcopy(grasp_pt)
            up_pt.z += 0.1

            req_gripper.move = 0.03  

            ret_value = 1

        elif dimensions[2] < min_z and dimensions[1] < max_y:
            print("Grasping perpendicular")

            req.parallel = False
            req.perpendicular = True
            req.perpendicular_rotate = False

            approach_pt = copy.deepcopy(centroid_pt)
            approach_pt.z -= 0.4
            grasp_pt = copy.deepcopy(centroid_pt)
            grasp_pt.z -= 0.3

            up_pt = copy.deepcopy(grasp_pt)
            up_pt.z += 0.1

            req_gripper.move = 0

            ret_value = 2

        else:
            # y > max_y
            print("Grasping perpendicular with rotation")

            req.parallel = False
            req.perpendicular = False
            req.perpendicular_rotate = True

            approach_pt = copy.deepcopy(centroid_pt)
            approach_pt.z -= 0.4
            grasp_pt = copy.deepcopy(centroid_pt)
            grasp_pt.z -= 0.3

            up_pt = copy.deepcopy(grasp_pt)
            up_pt.z += 0.1

            req_gripper.move = 0

            ret_value = 3

        req.position = approach_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to approach pose")

        req.position = grasp_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to grasp pose")
  
        self.close_gripper_client.call(req_gripper)
        self.check_mission_done()
        print("Object grasped")

        req.position = up_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above object")

        return ret_value

    def task1(self):
        A_ind, B_ind = self.get_positions(3, 1)

        box_pc = self.pts_B[B_ind[0]]
        box_centroid = self.centroid(box_pc)

        print("Box centroid: ")
        print(box_centroid)

        up_pt = copy.deepcopy(box_centroid)
        up_pt.z = self.get_max(box_centroid, 'z') + 0.04
        place_pt = copy.deepcopy(box_centroid)
        place_pt.z = self.get_min(box_centroid, 'z') + 0.02

        req = positionGoalRequest()
        req.parallel = False
        req.perpendicular = True
        req.perpendicular_rotate = False

        req_gripper = closeGripperRequest()
        req_gripper.move = 0

        for i in A_ind:
            pc = self.pts_A[i]
            grasp = self.grasp(pc)
            if grasp == 3:
                req.perpendicular = False
                req.perpendicular_rotate = True

            req.position = up_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved above box")

            req.position = place_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved to place pose")

            self.open_gripper_client.call(req_gripper)
            self.check_mission_done()
            print("Object placed")

            req.position = up_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved above box")

            self.go_home()

        print("Done with task 1")

    def task2(self):
        pot_z = 0.1
        A_ind, B_ind = self.get_positions(2, 2)

        req = positionGoalRequest()
        req.parallel = True
        req.perpendicular = False
        req.perpendicular_rotate = False

        req_gripper = closeGripperRequest()

        for i in range(len(A_ind)):
            pot_i = A_ind[i]
            shelf_i = B_ind[i]

            plant_pc = self.pts_A[pot_i]
            pot_pc = self.filter(plant_pc, 'z', self.z, self.z+pot_z)

            centroid_pt = self.get_centroid(pot_pc)
            dimensions = self.get_dimensions(pot_pc)

            approach_pt = copy.deepcopy(centroid_pt)
            approach_pt.x -= 0.4
            req.position = approach_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved to approach pose")

            grasp_pt = copy.deepcopy(centroid_pt)
            grasp_pt.x -= 0.2
            req.position = grasp_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved to grasp pose")

            req_gripper = closeGripperRequest()
            req_gripper.move = 0.03
            self.close_gripper_client.call(req_gripper)
            self.check_mission_done()
            print("Object grasped")

            up_pt = copy.deepcopy(grasp_pt)
            up_pt.z += 0.1
            req.position = up_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved above object")

            shelf_pc = self.pts_B[shelf_i]
            shelf_centroid = self.get_centroid(shelf_pc)
            shelf_z = self.get_max(shelf_pc, 'z')
            pot_centroid_min_delta = centroid_pt.z - self.get_min(pot_pc, 'z') - 0.02  #finger delta

            shelf_up_pt = copy.deepcopy(shelf_centroid)
            shelf_up_pt.x -= 0.16
            shelf_up_pt.z = shelf_z + dimensions[2]
            req.position = shelf_up_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved above shelf")

            place_pt = copy.deepcopy(shelf_centroid)
            place_pt.x -= 0.14
            place_pt.z = shelf_z + pot_centroid_min_delta
            req.position = place_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved to place pose")

            req_gripper.move = -0.03
            self.open_gripper_client.call(req_gripper)
            self.check_mission_done()
            print("Object placed")

            back_pt = copy.deepcopy(place_pt)
            back_pt.x -= 0.16
            req.position = back_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved away from the shelf")

            self.go_home()

        print("Done with task 2")

    def task3(self):
        A_ind, B_ind = self.get_positions(1, 2)

        if self.get_max(self.pts_B[B_ind[0]], 'z') > self.get_max(self.pts_B[B_ind[1]], 'z'):
            glass_pc = self.pts_B[B_ind[0]]
            print("Glass: " + str(B_ind[0]))
            coaster_pc = self.pts_B[B_ind[1]]
        else:
            glass_pc = self.pts_B[B_ind[1]]
            print("Glass: " + str (B_ind[1]))
            coaster_pc = self.pts_B[B_ind[0]]
        
        req = positionGoalRequest()
        req.parallel = True
        req.perpendicular = False
        req.perpendicular_rotate = False
        req.constraint = False

        whiskey_pc = self.pts_A[A_ind[0]]
        centroid_pt = self.get_centroid(whiskey_pc)
        dimensions = self.get_dimensions(whiskey_pc)

        approach_pt = copy.deepcopy(centroid_pt)
        approach_pt.x -= 0.4
        req.position = approach_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to approach pose")

        grasp_pt = copy.deepcopy(centroid_pt)
        grasp_pt.x -= 0.2
        grasp_pt.z = self.z + 0.1
        req.position = grasp_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to grasp pose")

        req_gripper = graspRequest()
        req_gripper.move = 0.03   
        self.close_gripper_client.call(req_gripper)
        self.check_mission_done()
        print("Object grasped")

        up_pt = copy.deepcopy(grasp_pt)
        up_pt.z += 0.1
        req.position = up_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above bottle")

        glass_centroid = self.get_centroid(glass_pc)
        glass_up_pt = copy.deepcopy(glass_centroid)
        glass_up_pt.z = self.z + 0.08 + 0.1
        glass_up_pt.x -= 0.18
        glass_up_pt.y -= 0.18
        req.position = glass_up_pt
        req.constraint = True
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above glass")

        self.pour_client.call() # num deg?
        self.check_mission_done()
        print("Poured whiskey!")

        # # move to normal rotation
        # # self.pour_client.call() # num deg?
        # # self.check_mission_done()
        # # print("Poured whiskey!")

        req.position = up_pt
        req.constraint = False
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above bottle")

        req.position = grasp_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to release pose")

        req_gripper.move = -0.03 
        self.open_gripper_client.call(req_gripper)
        self.check_mission_done()
        print("Gripper open")

        req.position = approach_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to approach pose")




        # casu na coaster
        approach_pt = copy.deepcopy(glass_centroid)
        approach_pt.x -= 0.4
        req.position = approach_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to glass approach pose")

        glass_pt = copy.deepcopy(glass_centroid)
        req.position = glass_pt
        req.position.z = self.z + 0.04
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to glass")

        req_gripper = closeGripperRequest()
        req_gripper.move = True   
        self.close_gripper_client.call(req_gripper)
        self.check_mission_done()
        print("Object grasped")

        up_pt = copy.deepcopy(glass_pt)
        up_pt.z += 0.1
        req.position = up_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above glass")

        coaster_centroid = self.get_centroid(coaster_pc)
        up_pt = copy.deepcopy(coaster_centroid)
        up_pt.z += 0.1
        req.position = up_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above coaster")

        place_pt = copy.deepcopy(coaster_centroid)
        place_pt.z = self.z + 0.04
        req.position = place_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Ready to release glass")

        self.open_gripper_client.call()
        self.check_mission_done()
        print("Gripper open")

        req.position = place_pt
        place_pt.x -= 0.4
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to approach pose")

    def go_home(self):
        joint_goal = self.read_vector_from_yaml(
            'franka_home.yaml')
        self.go_to_joint_goal_client.call(joint_goal)
        self.check_mission_done()
        print("Moved home")


def main():
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) < 2:
        print("Select among: task1, task2, task3")
    else:
        task = myargv[1]
        rospy.init_node('robosoft_control')
        robosoft = Robosoft()
        req_gripper = graspRequest()
        req_gripper.move = 0
        robosoft.open_gripper_client.call(req_gripper)
        robosoft.check_mission_done()
        print("Gripper open")


        joint_goals = robosoft.read_vectors_from_yaml(
            'franka_record_low.yaml')
        robosoft.record(joint_goals)
        robosoft.franka_count_pts()
        print("Done with recording")

        robosoft.go_home()

        # if task == "task1":
        #     print("Starting task 1")
        #     robosoft.task1()
        # elif task == "task2":
        #     print("Starting task 2")
        #     robosoft.task2()
        # elif task == "task3":
        #     print("Starting task 3")
        #     robosoft.task3()
        # else:
        #     print("Select among: task1, task2, task3")

        # pt = PointStamped()
        # pt.header.frame_id = robosoft.base_frame
        # pt.point = centroid_pt

        # while not rospy.is_shutdown():
        #     pc_test = ros_numpy.point_cloud2.array_to_pointcloud2(
        #         robosoft.A2, frame_id=robosoft.base_frame)
        #     robosoft.test_publisher.publish(pc_test)
        #     robosoft.point_publisher.publish(pt)
        #     rospy.sleep(2)


if __name__ == "__main__":
    main()
