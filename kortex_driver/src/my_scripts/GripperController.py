#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time

from kortex_driver.srv import *
from kortex_driver.msg import *

from std_msgs.msg import Float32, Header

class ExampleFullArmMovement:
    def __init__(self):
        
        rospy.init_node('example_full_arm_movement_python')

        # self.HOME_ACTION_IDENTIFIER = 2

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

        rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

        # # Init the action topic subscriber
        # self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)

        send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        self.GripperSubscriber = rospy.Subscriber("/KinovaAR/gripper_cmd/goal_mod", Float32, self.Gripper_callback)

        rospy.spin()

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event


    def Gripper_callback(self, data):
        gripper_pose = data.data

        if(gripper_pose >= 0 and gripper_pose <= 1):

            req = SendGripperCommandRequest()
            finger = Finger()
            finger.finger_identifier = 0
            finger.value = gripper_pose
            req.input.gripper.finger.append(finger)
            req.input.mode = GripperMode.GRIPPER_POSITION
            # rospy.loginfo(GripperMode.GRIPPER_POSITION)
            self.send_gripper_command(req)
            time.sleep(1)



    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.5)
            return True


if __name__ == "__main__":
    ex = ExampleFullArmMovement()

