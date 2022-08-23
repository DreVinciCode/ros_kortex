#!/usr/bin/env python

from geometry_msgs.msg import Twist
from kortex_driver.msg import TwistCommand

import rospy

class messageConverter:

    def __init__(self):
    	rospy.init_node('TwistToTwistCommand', anonymous = True)
    	self.Twist_sub = rospy.Subscriber("/my_gen3_lite/in/cartesian_velocity_mod", Twist, self.twist_callback)
    	self.TwistCommand_pub = rospy.Publisher("/my_gen3_lite/in/cartesian_velocity", TwistCommand, queue_size=1)
    	rospy.spin()

    def twist_callback(self, data):
    	linearX = data.linear.x
    	linearY = data.linear.y
    	linearZ = data.linear.z 

    	TwistCommand_Message = TwistCommand()
    	TwistCommand_Message.twist.linear_x = linearX
    	TwistCommand_Message.twist.linear_y = linearY
    	TwistCommand_Message.twist.linear_z = linearZ

    	self.TwistCommand_pub.publish(TwistCommand_Message)

if __name__ == '__main__':
    converter = messageConverter()