#!/usr/bin/env python3

import rospy
from robosoft.franka_moveit_robosoft import FrankaRobot
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import ros_numpy
import numpy as np
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

class Robosoft(object):
    def __init__(self):
        self.robot = FrankaRobot()
        box = self.robot.add_box()
        print('Box added: ', box)

        self.robot.open()
        
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

        self.test_publisher = rospy.Publisher('test_pc', PointCloud2, queue_size=10)
        self.point_publisher = rospy.Publisher('test_point', PointStamped, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        print('Sleeping for 3s - tf subscriber initialized')
        rospy.sleep(3)

    def count_pts_A(self):
        print('Waitong for pc message')
        self.pc_msg = rospy.wait_for_message(self.pc_topic, PointCloud2)
        print('Recieved pc message')

        print('Waiting for tf')
        try:
            trans = self.tfBuffer.lookup_transform(self.base_frame, self.camera_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Did not get transform')

        self.pc_glob = do_transform_cloud(self.pc_msg, trans)
        print('Transformed point cloud')

        self.pts_A_count = []
        self.pts_A = []

        # 1D np array
        self.points = ros_numpy.point_cloud2.pointcloud2_to_array(self.pc_glob)
        self.points = self.points.flatten()

        # filtering
        z_values = self.points['z']

        mask = ~np.isnan(z_values) * (z_values > self.z) 
        pts_z = self.points[mask]
        x_values = pts_z['x']
        y_values = pts_z['y']
        
        mask = (x_values > self.x1-self.r) * (x_values < self.x1+self.r) * (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.A1 = pts_z[mask]
        self.pts_A_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x2-self.r) * (x_values < self.x2+self.r) * (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.A2 = pts_z[mask]
        self.pts_A_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x3-self.r) * (x_values < self.x3+self.r) * (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.A3 = pts_z[mask]
        self.pts_A_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x1-self.r) * (x_values < self.x1+self.r) * (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.A4 = pts_z[mask]
        self.pts_A_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x2-self.r) * (x_values < self.x2+self.r) * (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.A5 = pts_z[mask]
        self.pts_A_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x3-self.r) * (x_values < self.x3+self.r) * (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.A6 = pts_z[mask]
        self.pts_A_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        print('Done with pc filtering')
        print('Pts count: ')
        print(self.pts_A_count)

    def count_pts_B(self):
        print('Waiting for pc message')
        self.pc_msg = rospy.wait_for_message(self.pc_topic, PointCloud2)
        print('Recieved pc message')

        print('Waiting for tf')
        try:
            trans = self.tfBuffer.lookup_transform(self.base_frame, self.camera_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Did not get transform')

        self.pc_glob = do_transform_cloud(self.pc_msg, trans)
        print('Transformed point cloud')

        self.pts_B_count = []
        self.pts_B = []

        # 1D np array
        self.points = ros_numpy.point_cloud2.pointcloud2_to_array(self.pc_glob)
        self.points = self.points.flatten()

        # filtering
        z_values = self.points['z']

        mask = ~np.isnan(z_values) * (z_values > self.z) 
        pts_z = self.points[mask]
        x_values = pts_z['x']
        y_values = pts_z['y']
        
        mask = (x_values > self.x4-self.r) * (x_values < self.x4+self.r) * (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.B1 = pts_z[mask]
        self.pts_B_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x5-self.r) * (x_values < self.x5+self.r) * (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.B2 = pts_z[mask]
        self.pts_B_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x6-self.r) * (x_values < self.x6+self.r) * (y_values > self.y1-self.r) * (y_values < self.y1+self.r)
        self.B3 = pts_z[mask]
        self.pts_B_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x4-self.r) * (x_values < self.x4+self.r) * (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.B4 = pts_z[mask]
        self.pts_B_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x5-self.r) * (x_values < self.x5+self.r) * (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.B5 = pts_z[mask]
        self.pts_B_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        mask = (x_values > self.x6-self.r) * (x_values < self.x6+self.r) * (y_values > self.y2-self.r) * (y_values < self.y2+self.r)
        self.B6 = pts_z[mask]
        self.pts_B_count.append(len(pts_z[mask]))
        self.pts_B.append(pts_z[mask])

        print('Done with pc filtering')
        print('Pts count: ', self.pts_B_count)

    def get_centroid(self, pc):
        x_values = pc['x']
        y_values = pc['y']
        z_values = pc['z']

        x = np.sum(x_values)/len(x_values)
        y = np.sum(y_values)/len(y_values)
        z = np.sum(z_values)/len(z_values)

        print('Object centroid: ', x, y, z)
        return [x, y, z]


def main():
    rospy.init_node('robosoft_control')
    robosoft = Robosoft()
    # robosoft.robot.grasp()
    #robosoft.robot.open()

    joint_goal = robosoft.robot.read_vector_from_yaml('record_B.yaml')
    robosoft.robot.go_to_joint_state(joint_goal)
    robosoft.count_pts_B()
    c = robosoft.get_centroid(robosoft.pts_B[0])

    joint_goal = robosoft.robot.read_vector_from_yaml('grasp_B.yaml')
    robosoft.robot.go_to_joint_state(joint_goal)

    robosoft.robot.go_to_position_parallel([c[0], c[1]-0.25, c[2]])
    robosoft.robot.go_to_position_parallel([c[0], c[1]-0.15, c[2]])
    robosoft.robot.grasp()

    joint_goal = robosoft.robot.read_vector_from_yaml('grasp_B.yaml')
    robosoft.robot.go_to_joint_state(joint_goal)

    # pt = PointStamped()
    # pt.header.frame_id = robosoft.base_frame
    # pt.point.x = c[0]
    # pt.point.y = c[1]
    # pt.point.z = c[2]
        
    # while not rospy.is_shutdown():
    #     pc_test = ros_numpy.point_cloud2.array_to_pointcloud2(robosoft.B1, frame_id=robosoft.base_frame)
    #     robosoft.test_publisher.publish(pc_test)
    #     robosoft.point_publisher.publish(pt)
    #     rospy.sleep(2)


if __name__ == "__main__":
    main()