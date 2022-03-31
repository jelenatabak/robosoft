#!/usr/bin/env python3

import sys
import rospy
import rospkg
from sensor_msgs.msg import JointState
from franka_interface.srv import *
import tf2_ros


class Waypoints(object):
    def __init__(self, filename):

        self.filename = filename

        # Recorded waypoints
        self._waypoints = []
        self._gripper_waypoints = []
        rospack = rospkg.RosPack()
        self.yaml_path = rospack.get_path('robosoft') + '/resources/'

        print(self.yaml_path)

        # Recording state
        self._is_recording = False

        self._joint_states = JointState()

        rospy.Subscriber("joint_states",
                         JointState, self.joint_states_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.world_frame = "world"
        # self.eef_frame = "panda_link8"
        self.world_frame = "base_link"
        self.eef_frame = "tool0"

        self.record_service = rospy.Service(
            "/waypoint_record", waypoint_record, self._waypoint_record_callback)

        print('Call /waypoint_record, 0 to delete, 1 for joint states, 2 for pose, 3 for saving to file.')

    def _add_point(self):

        dikt = {}

        for i in range(0, len(self._joint_states.name)):
            dikt[self._joint_states.name[i]] = self._joint_states.position[i]

        self._waypoints.append(dikt)

    def _waypoint_record_callback(self, msg):
        if (msg.command == 0):
            # delete list
            print("Deleting Waypoint List!")
            self._waypoints = []
        elif (msg.command == 1):
            # add waypoint (joint states)
            print("Add waypoint to list - joint states!")
            self._waypoints.append(self._joint_states.position)
        elif(msg.command == 2):
            # add waypoint (eef pose)
            print("Add waypoint to list - eef pose!")
            try:
                trans = self.tf_buffer.lookup_transform(self.world_frame, self.eef_frame, rospy.Time(), rospy.Duration(2))
                t = trans.transform.translation
                r = trans.transform.rotation
                pose = [t.x, t.y, t.z, r.x, r.y, r.z, r.w]
                self._waypoints.append(pose)
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Did not get transform")
                return False
        elif (msg.command == 3):
            # store waypoints 
            print("Store waypoints to file.")

            with open(self.yaml_path + self.filename, 'w') as dump_file:
                dump_file.write("positions:\n")
                dump_file.write("  [\n")
                counter = 0
                for item in self._waypoints:
                    dump_file.write('    [\n')
                    dump_file.write('      [')    
                    for (i, j1) in enumerate(item):
                        if i < 7:
                            dump_file.write(str(j1))
                            if i < 6:
                                dump_file.write(', ')
                    
                    dump_file.write('],\n')
                    dump_file.write('    ]')

                    counter = counter + 1
                    if (counter < len(self._waypoints)):
                        dump_file.write(',')

                    dump_file.write('\n')

                dump_file.write("  ]")

                # for item in self._waypoints:
                #     dump_file.write('{')
                #     for (i, j1) in enumerate(item):
                #         json.dump(j1, dump_file)
                #         if i < 6:
                #             dump_file.write(', ')
                #     dump_file.write('},\n')
                #json.dump(self._waypoints, dump_file, indent=1)

        return True

    def _stop_recording(self, value):
        pass
        """
        #Sets is_recording to false
        #Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        # if value:
        #    self._is_recording = False

        # with open(self.json_path + self._arm + '_waypoints.json', 'w') as dump_file:
        #    json.dump(self._waypoints, dump_file)

        # with open(self.json_path + self._arm + '_gripper.json', 'w') as dump_file:
        #    json.dump(self._gripper_waypoints, dump_file)

    def joint_states_callback(self, msg):
        self._joint_states = msg

    def _run(self):

        while not rospy.is_shutdown():
            try:

                # print "Running!"
                rospy.sleep(0.1)
            except KeyboardInterrupt:
                print("Interrupt")
                self._add_point()

                print(self.waypoints)


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) < 2:
        print("Please provide a filename")
    else:
        print("Initializing node... ")
        rospy.init_node("Franka_christmas_recorder")
        waypoints = Waypoints(myargv[1])
        waypoints._run()
