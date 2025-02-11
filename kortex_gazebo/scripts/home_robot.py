#!/usr/bin/env python

import sys
import time
import rospy
from std_srvs.srv import Empty
from kortex_driver.msg import ActionNotification, ActionEvent
from kortex_driver.srv import ReadAction, ReadActionRequest, ExecuteAction, ExecuteActionRequest

class ExampleInitializeGazeboRobot(object):
  """Unpause Gazebo and home robot"""

  def __init__(self):
    # Initialize the node
    self.REST_ACTION_IDENTIFIER = 1
    self.HOME_ACTION_IDENTIFIER = 2
    self.last_action_notif_type = None

    try:
      # self.robot_name = rospy.get_param('~robot_name')
      self.robot_name = 'my_gen3_lite'
      self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)

      # Wait for the driver to be initialised
      while not rospy.has_param("/" + self.robot_name + "/is_initialized"):
        time.sleep(0.1)
      
      # Init the services
      read_action_full_name = '/' + self.robot_name + '/base/read_action'
      rospy.wait_for_service(read_action_full_name)
      self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

      execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
      rospy.wait_for_service(execute_action_full_name)
      self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)
    except rospy.ROSException as e:
      self.is_init_success = False
    else:
      self.is_init_success = True

  def cb_action_topic(self, notif):
    self.last_action_notif_type = notif.action_event



  def wait_for_action_end_or_abort(self):
    while not rospy.is_shutdown():
      if (self.last_action_notif_type == ActionEvent.ACTION_END):
        rospy.loginfo("Received ACTION_END notification")
        return True
      elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
        rospy.loginfo("Received ACTION_ABORT notification")
        return False
      else:
        time.sleep(0.01)

  def home_the_robot(self):
    # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
    self.last_action_notif_type = None
    req = ReadActionRequest()
    req.input.identifier = self.HOME_ACTION_IDENTIFIER

    try:
      res = self.read_action(req)
    except rospy.ServiceException:
      rospy.logerr("Failed to call ReadAction")
      return False
    # Execute the HOME action if we could read it
    else:
      # What we just read is the input of the ExecuteAction service
      req = ExecuteActionRequest()
      req.input = res.output
      rospy.loginfo("Sending the robot home...")
      try:
        self.execute_action(req)
      except rospy.ServiceException:
        rospy.logerr("Failed to call ExecuteAction")
        return False
      else:
        return self.wait_for_action_end_or_abort()

  def rest_the_robot(self):
    # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
    self.last_action_notif_type = None
    req = ReadActionRequest()
    req.input.identifier = self.REST_ACTION_IDENTIFIER

    try:
      res = self.read_action(req)
    except rospy.ServiceException:
      rospy.logerr("Failed to call ReadAction")
      return False
    # Execute the HOME action if we could read it
    else:
      # What we just read is the input of the ExecuteAction service
      req = ExecuteActionRequest()
      req.input = res.output
      rospy.loginfo("Sending the robot to rest...")
      try:
        self.execute_action(req)
      except rospy.ServiceException:
        rospy.logerr("Failed to call ExecuteAction")
        return False
      else:
        return self.wait_for_action_end_or_abort()

  def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
    waypoint = Waypoint()
    cartesianWaypoint = CartesianWaypoint()

    cartesianWaypoint.pose.x = new_x
    cartesianWaypoint.pose.y = new_y
    cartesianWaypoint.pose.z = new_z
    cartesianWaypoint.pose.theta_x = new_theta_x
    cartesianWaypoint.pose.theta_y = new_theta_y
    cartesianWaypoint.pose.theta_z = new_theta_z
    cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
    cartesianWaypoint.blending_radius = blending_radius
    waypoint.oneof_type_of_waypoint.cartesian_waypoint.append(cartesianWaypoint)

    return waypoint


def main():
  try:
    # For testing purposes
    try:
        rospy.delete_param("is_initialized")
    except:
        pass

    # Unpause the physics
    rospy.loginfo("Unpausing Gazebo...")
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()
    rospy.loginfo("Unpaused Gazebo.")

    example = ExampleInitializeGazeboRobot()
    success = example.is_init_success
    if success:
      success &= example.home_the_robot()
      # success &= example.rest_the_robot()

  except Exception as e:
    print (e)
    success = False
  
  # For testing purposes
  rospy.set_param("is_initialized", success)
  if not success:
    rospy.logerr("The Gazebo initialization encountered an error.")
  else:
    rospy.loginfo("The Gazebo initialization executed without fail.")

if __name__ == '__main__':
  rospy.init_node('init_robot')
  main()
