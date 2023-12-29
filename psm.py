#  Author(s):  Anton Deguet, Brendan Burkhart
#  Created on: 2022-08-06

#  (C) Copyright 2022 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import time
from dvrk.arm import *

from crtk_msgs.srv import QueryForwardKinematics, QueryForwardKinematicsRequest
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import numpy as np
import crtk


class PSM(arm):
    class Jaw:
        def __init__(self, ral, expected_interval, operating_state_instance):
            self._crtk_utils = crtk.utils(
                self, ral, expected_interval, operating_state_instance
            )
            self._crtk_utils.add_move_jp()
            self._crtk_utils.add_measured_js()

        def close(self):
            return self.move_jp(np.array(math.radians(-20.0)))

        def open(self, angle=math.radians(60.0)):
            return self.move_jp(np.array(angle))

    # initialize the robot
    def __init__(self, ral, arm_name, ros_namespace="", expected_interval=0.01):
        # self._arm__init_arm(arm_name, ros_namespace, expected_interval)
        super().__init__(ral, arm_name, expected_interval)
        jaw_ral = self.ral().create_child("/jaw")

        self.namespace = ros_namespace
        self.jaw = PSM.Jaw(jaw_ral, expected_interval, operating_state_instance=self)

        query_cp_name = "{}/local/query_cp".format(self.namespace)
        self.local_query_cp = rospy.ServiceProxy(query_cp_name, QueryForwardKinematics)

        base_frame_topic = "/{}/set_base_frame".format(self.namespace)
        self._set_base_frame_pub = rospy.Publisher(
            base_frame_topic, Pose, queue_size=1, latch=True
        )

        # Base class will unregister pub_list on shutdown
        # self._arm__pub_list.append(self._set_base_frame_pub)

        self.cartesian_insertion_minimum = 0.055
        self.name = arm_name

    # Sets speed ratio for move_cp/move_jp
    def set_speed(self, speed):
        self.trajectory_j_set_ratio(speed)

    def clear_base_frame(self):
        identity = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self._set_base_frame_pub.publish(identity)

    def set_base_frame(self, pose):
        self._set_base_frame_pub.publish(pose)

    # def forward_kinematics(self, joint_position):
    #     pad_length = max(0, 8 - len(joint_position))
    #     request = QueryForwardKinematicsRequest()
    #     request.jp.position = np.pad(joint_position, (0, pad_length)).tolist()
    #     response = self.local_query_cp(request)
    #     point = response.cp.pose.position
    #     return np.array([point.x, point.y, point.z])

    # Bring arm back to center
    def center(self):
        pose = np.copy(self.measured_jp())
        pose.fill(0.0)
        pose[2] = self.cartesian_insertion_minimum
        return self.move_jp(pose)

    # Make sure tool is inserted past cannula so move_cp works
    def enter_cartesian_space(self):
        pose = np.copy(self.measured_jp())
        if pose[2] >= self.cartesian_insertion_minimum:

            class NoWaitHandle:
                def wait(self):
                    pass

                def is_busy(self):
                    return False

            return NoWaitHandle()

        pose[2] = self.cartesian_insertion_minimum
        return self.move_jp(pose)


if __name__ == "__main__":
    ral = crtk.ral("dvrk_psm_test")
    # time.sleep(0.5)

    psm2 = PSM(ral, "PSM2", ros_namespace="", expected_interval=0.01)
    ral.check_connections()

    pose1 = np.array([0.0, -0.0, 0.132, -0.0, -0.0, 0.0])
    psm2.move_jp(pose1)
    print(f"measured_jp {psm2.measured_jp()}")
