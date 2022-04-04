#!/usr/bin/env python3

import copy
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header 
import ros_numpy
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import yaml
from geometry_msgs.msg import Pose, Point, PointStamped
from robosoft.srv import *
from std_srvs.srv import Trigger, TriggerResponse
import rospkg
import pcl


class Robosoft(object):
    def __init__(self):
        self.z = rospy.get_param('/robosoft/z')
        self.r = rospy.get_param('/robosoft/r')

        self.y1 = rospy.get_param('/robosoft/y1')
        self.y2 = rospy.get_param('/robosoft/y2')

        self.x3 = rospy.get_param('/robosoft/x3')
        self.x4 = rospy.get_param('/robosoft/x4')
        self.x5 = rospy.get_param('/robosoft/x5')
        self.x6 = rospy.get_param('/robosoft/x6')

        self.x1 = rospy.get_param('/robosoft/x1')
        self.x2 = rospy.get_param('/robosoft/x2')

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
        self.points_list_segmented = []

        self.delta_z = 0.45

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
        
        vec = dict.get('positions')[0][0]
        temp0 = vec[0]
        temp2 = vec[2]
        vec[2] = temp0
        vec[0] = temp2
        print(vec)
        return vec

    def read_vectors_from_yaml(self, file):
        joint_goals = []
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)

        l = len(dict.get('positions'))
        for i in range(l):
            vec = dict.get('positions')[i][0]
            temp0 = vec[0]
            temp2 = vec[2]
            vec[2] = temp0
            vec[0] = temp2
            joint_goals.append(vec)

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

    def record_devel(self, joint_goals):
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

            points = ros_numpy.point_cloud2.pointcloud2_to_array(self.pc_msg)
            # z_values = points['z']
            # mask = ~np.isnan(z_values) 
            # points = points[mask]

            #self.points_list.append(points)
            # for row in points:
            #     self.points_list.append([row[0], row[1], row[2], row[3]])

        self.points = np.concatenate(self.points_list)

        #     self.points = np.array(self.points_list)
        #     self.points_list_segmented.append(self.segment_plane())

        # self.points_segmented = np.concatenate(self.points_list_segmented)

        # pc_pub_plane = rospy.Publisher("/segmented_plane", PointCloud2, queue_size=10)
        # pc = self.point_cloud(self.points_segmented, "panda_link0")

        # while not rospy.is_shutdown():
        #     pc_pub_plane.publish(pc)
        #     rospy.sleep(1)

        # self.pc = pc
        # self.points = ros_numpy.point_cloud2.pointcloud2_to_array(self.pc)


    def segment_plane(self):

        self.points_g = self.points[:,0:3]
        cloud = pcl.PointCloud()
        cloud.from_array(np.array(self.points_g))
        seg = cloud.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(pcl.SACMODEL_PLANE)
        # seg.set_model_type(pcl.SACMODEL_PERPENDICULAR_PLANE)
        # seg.set_axis(0., 0., 1.0)
        seg.set_eps_angle(0.2)
        seg.set_normal_distance_weight(0.05)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(200)
        seg.set_distance_threshold(0.02)
        inliers, model = seg.segment()

        if len(inliers)>1:
            print(len(inliers))
            inliers = np.array(inliers)

            ind = np.arange(len(self.points_g))
            outliers = []
            counter = 0

            for i in ind:
                if i not in inliers:
                    outliers.append(i)
                    counter += 1

            points2 = self.points[outliers,:]

            return points2



    def point_cloud(self, points, parent_frame):
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize

        data = points.astype(dtype).tobytes()

        offsets = [0,4,8,16]
        offsets = [0,4,8,12]

        fields = [PointField(
            name=n, offset=offsets[i], datatype=ros_dtype, count=1)
            for i, n in enumerate(['x','y','z','rgb'])]


        header = Header(frame_id=parent_frame, stamp=rospy.Time.now())

        return PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=itemsize*4,
            row_step=32*points.shape[0],
            data=data
        )




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

        # print('Object centroid: ', c)
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
        centroid_pt = self.get_centroid(pc)
        dimensions = self.get_dimensions(pc)

        req = positionGoalRequest()
        req_gripper = graspRequest()

        req.parallel = False
        req.perpendicular = True
        req.perpendicular_rotate = False
        req.constraint = False

        approach_pt = copy.deepcopy(centroid_pt)
        approach_pt.z = 0.45

        grasp_pt = copy.deepcopy(centroid_pt)
        grasp_pt.z = 0.31

        req_gripper.move.x = 0
        req_gripper.move.y = 0
        req_gripper.move.z = -0.02

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

        rospy.sleep(2)

        req.position = approach_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above object")


    def task1(self):

        self.go_home_pick()
        joint_goals = self.read_vectors_from_yaml(
            'ur_record_pick.yaml')
        self.record(joint_goals)
        self.ur_count_pts()
        print("Done with recording")

        self.go_home_pick()

        A_ind, B_ind = self.get_positions(3, 1)

        box_pc = self.pts_B[B_ind[0]]
        box_centroid = self.get_centroid(box_pc)

        print("Box centroid: ")
        print(box_centroid)

        up_pt = copy.deepcopy(box_centroid)
        up_pt.z = self.get_max(box_pc, 'z') + 0.45

        place_pt = copy.deepcopy(box_centroid)
        place_pt.z = self.get_max(box_pc, 'z') + 0.32

        req = positionGoalRequest()
        req.parallel = False
        req.perpendicular = True
        req.perpendicular_rotate = False
        req.constraint = False

        req_gripper = graspRequest()
        req_gripper.move.x = 0
        req_gripper.move.y = 0
        req_gripper.move.z = 0.03

        for i in A_ind:
            print(i)
            pc = self.pts_A[i]
            self.grasp(pc)

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

            self.go_home_pick()

        print("Done with task 1")

    def task2(self):
        self.go_home_parallel()
        joint_goals = self.read_vectors_from_yaml(
            'ur_record_far.yaml')
        self.record(joint_goals)
        self.ur_count_pts()
        print("Done with recording")

        self.go_home_parallel()

        pot_z = 0.1
        A_ind, B_ind = self.get_positions(1, 1)

        req = positionGoalRequest()
        req.parallel = True
        req.perpendicular = False
        req.perpendicular_rotate = False
        req.constraint = False

        req_gripper = graspRequest()

        for i in range(len(A_ind)):
            pot_i = A_ind[i]
            shelf_i = B_ind[i]

            plant_pc = self.pts_A[pot_i]
            pot_pc = self.filter(plant_pc, 'z', self.z, self.z+pot_z)

            centroid_pt = self.get_centroid(pot_pc)
            dimensions = self.get_dimensions(pot_pc)

            approach_pt = copy.deepcopy(centroid_pt)
            #approach_pt.x -= 0.4
            approach_pt.y += self.delta_z
            approach_pt.z = self.z + 0.05
            req.position = approach_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved to approach pose")

            print(approach_pt)

            grasp_pt = copy.deepcopy(centroid_pt)
            # grasp_pt.x -= 0.2
            grasp_pt.y += 0.25
            grasp_pt.z = self.z + 0.05
            req.position = grasp_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved to grasp pose")

            req_gripper = graspRequest()
            # req_gripper.move.x = 0.03
            req_gripper.move.y = -0.03
            self.close_gripper_client.call(req_gripper)
            self.check_mission_done()
            print("Object grasped")

            up_pt = copy.deepcopy(grasp_pt)
            up_pt.z += 0.1
            req.position = up_pt
            # req.constraint = True
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved above object")

            shelf_pc = self.pts_B[shelf_i]
            shelf_centroid = self.get_centroid(shelf_pc)
            shelf_z = self.get_max(shelf_pc, 'z')

            self.go_home_parallel()

            shelf_up_pt = copy.deepcopy(shelf_centroid)
            shelf_up_pt.y += 0.25
            shelf_up_pt.z = shelf_z + 0.1
            req.position = shelf_up_pt
            # req.constraint = True
            print(req)
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved above shelf")

            place_pt = copy.deepcopy(shelf_centroid)
            place_pt.y += 0.25
            place_pt.z = shelf_z + 0.05
            req.position = place_pt
            # req.constraint = True
            print(req)
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved to place pose")

            req_gripper.move.y = 0.03
            self.open_gripper_client.call(req_gripper)
            self.check_mission_done()
            print("Object placed")

            back_pt = copy.deepcopy(shelf_centroid)
            back_pt.y += self.delta_z
            back_pt.z = shelf_z + 0.05
            req.position = back_pt
            self.go_to_position_goal_client.call(req)
            self.check_mission_done()
            print("Moved away from the shelf")

            self.go_home_parallel()

        print("Done with task 2")

    def task3(self):
        self.go_home_parallel()
        joint_goals = self.read_vectors_from_yaml(
            'ur_record_far.yaml')
        self.record(joint_goals)
        self.ur_count_pts()
        print("Done with recording")

        self.go_home_parallel()

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
        approach_pt.y += self.delta_z
        req.position = approach_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to approach pose")

        grasp_pt = copy.deepcopy(centroid_pt)
        grasp_pt.y += 0.25
        grasp_pt.z = self.z + 0.1
        req.position = grasp_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to grasp pose")

        req_gripper = graspRequest()
        req_gripper.move.y = -0.03   
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
        glass_up_pt.x -= 0.2   # pouring distance
        glass_up_pt.y += 0.25
        req.position = glass_up_pt
        # req.constraint = True
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above glass")

        self.pour_client.call() # num deg?
        self.check_mission_done()
        print("Poured whiskey!")

        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above glass")

        req.position = up_pt
        req.constraint = False
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above bottle")

        req.position = grasp_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to release pose")

        req_gripper.move.y = 0.03 
        self.open_gripper_client.call(req_gripper)
        self.check_mission_done()
        print("Gripper open")

        req.position = approach_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to approach pose")

        self.go_home_parallel()


        whiskey_glass_grasp = 0.05
        approach_pt = copy.deepcopy(glass_centroid)
        approach_pt.y += self.delta_z
        approach_pt.z = self.z + whiskey_glass_grasp
        req.position = approach_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to glass approach pose")

        glass_pt = copy.deepcopy(glass_centroid)
        glass_pt.z = self.z + whiskey_glass_grasp
        glass_pt.y += 0.25
        req.position = glass_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to glass")

        req_gripper = graspRequest()
        req_gripper.move.y = -0.03 
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
        coaster_height = self.get_max(coaster_pc, 'z')
        up_pt = copy.deepcopy(coaster_centroid)
        up_pt.z = coaster_height + 0.1 
        up_pt.y += 0.25
        req.position = up_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved above coaster")

        place_pt = copy.deepcopy(coaster_centroid)
        place_pt.z = coaster_height + whiskey_glass_grasp
        place_pt.y += 0.25
        req.position = place_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Ready to release glass")

        req_gripper = graspRequest()
        req_gripper.move.y = 0.03 
        self.open_gripper_client.call(req_gripper)
        self.check_mission_done()
        print("Object grasped")

        last_pt = copy.deepcopy(coaster_centroid)
        last_pt.z = self.z + coaster_height + whiskey_glass_grasp
        last_pt.y += self.delta_z
        req.position = last_pt
        self.go_to_position_goal_client.call(req)
        self.check_mission_done()
        print("Moved to approach pose")

        self.go_home_parallel()

        print("Done with task 3!")

    def go_home_pick(self):
        joint_goal = self.read_vector_from_yaml(
            'ur_home_pick.yaml')
        self.go_to_joint_goal_client.call(joint_goal)
        self.check_mission_done()
        print("Moved home")

    def go_home_parallel(self):
        joint_goal = self.read_vector_from_yaml(
            'ur_home_far.yaml')
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
        robosoft.open_gripper_client.call(req_gripper)
        robosoft.check_mission_done()
        print("Gripper open")

        if task == "task1":
            print("Starting task 1")
            robosoft.task1()
        elif task == "task2":
            print("Starting task 2")
            robosoft.task2()
        elif task == "task3":
            print("Starting task 3")
            robosoft.task3()
        else:
            print("Select among: task1, task2, task3")


        # req_gripper = graspRequest()
        # # req_gripper.move.x = 0.03
        # req_gripper.move.x = 0
        # req_gripper.move.y = 0
        # req_gripper.move.z = -0.03
        # robosoft.close_gripper_client.call(req_gripper)
        # robosoft.check_mission_done()
        # print("Object grasped")


        # req = positionGoalRequest()
        # req.parallel = True
        # req.perpendicular = False
        # req.perpendicular_rotate = False
        # req.constraint = False
        # req.position.x = -0.108
        # req.position.y = -0.29
        # req.position.z = 0.45

        # req = poseGoalRequest()
        # pose = Pose()
        # pose.position.x = 0.275
        # pose.position.y = -0.427
        # pose.position.z = 0.096
        # # pose.orientation.x = 0.66537
        # # pose.orientation.y = 0.495849
        # # pose.orientation.z = 0.0254282
        # # pose.orientation.w = 0.5574675
        # pose.orientation.x = 0.3730793
        # pose.orientation.y = 0.6301339
        # pose.orientation.z = -0.0890189
        # pose.orientation.w = 0.6751435
        # robosoft.go_to_pose_goal_client(req)

        # pt = PointStamped()
        # pt.header.frame_id = robosoft.base_frame

        # print("Ploting")

        # while not rospy.is_shutdown():
        #     # pc_test = ros_numpy.point_cloud2.array_to_pointcloud2(robosoft.points, frame_id=robosoft.base_frame)
        #     # robosoft.test_publisher.publish(pc_test)
        #     # rospy.sleep(5)

        #     for i in range(6):
        #         print ("A" + str(i))
        #         pc = robosoft.pts_A[i]
        #         # centroid = robosoft.get_centroid(pc)
        #         # pt.point = centroid
        #         pc_test = ros_numpy.point_cloud2.array_to_pointcloud2(
        #             pc, frame_id=robosoft.base_frame)
        #         robosoft.test_publisher.publish(pc_test)
        #         # robosoft.point_publisher.publish(pt)
        #         rospy.sleep(5)

        #     for i in range(6):
        #         print ("B" + str(i))
        #         pc = robosoft.pts_B[i]
        #         # centroid = robosoft.get_centroid(pc)
        #         # pt.point = centroid
        #         pc_test = ros_numpy.point_cloud2.array_to_pointcloud2(
        #             pc, frame_id=robosoft.base_frame)
        #         robosoft.test_publisher.publish(pc_test)
        #         # robosoft.point_publisher.publish(pt)
        #         rospy.sleep(5)


if __name__ == "__main__":
    main()
