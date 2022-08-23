#!/usr/bin/env python

import rospy

from control_msgs.msg import GripperCommandActionGoal
from std_msgs.msg import Float32, Header

class GripperManagerClass:

	def __init__(self):
		rospy.init_node('GripperManagerScript', anonymous = True)

		self.GripperSubscriber = rospy.Subscriber("/KinovaAR/gripper_cmd/goal_mod", Float32, self.Gripper_callback)
		self.GripperPublisher = rospy.Publisher("/my_gen3_lite/gen3_lite_2f_gripper_controller/gripper_cmd/goal", GripperCommandActionGoal, queue_size=1)
		rospy.spin()

	def Gripper_callback(self, data):
	
		gripper_pose = data.data

		if(gripper_pose > 0 and gripper_pose < 0.96):

			Gripper_Message = GripperCommandActionGoal()

			h = Header()
			h.stamp = rospy.Time.now()
			Gripper_Message.header = h

			Gripper_Message.goal.command.position = data.data

			self.GripperPublisher.publish(Gripper_Message)

if __name__ == '__main__':
    GripperManager = GripperManagerClass()