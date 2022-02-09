#!/usr/bin/env python

import rospy
from std_msgs.msg import String
# The type of message of cmd_vel
from geometry_msgs.msg import Twist 

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


def quaternion_to_yaw(quat):
  orientation_list = [quat.x, quat.y,  quat.z, quat.w]
  roll, pitch, yaw = euler_from_quaternion(orientation_list)
  return yaw

def callback(goal):
  
  # Note that the yaw recieved is simply the atan of 
  # delta_x and delta_y. It is required if the goal is
  # being broadcast to Move Base instead of this node
  delta_x = goal.target_pose.pose.position.x
  delta_y = goal.target_pose.pose.position.y
  delta_r = quaternion_to_yaw(goal.target_pose.pose.orientation)

  delta_r *= 0.5

  # rospy.loginfo("Mover node recieved goal vector r:{} x:{} y:{}".format(delta_x, delta_y, delta_r));

  message = Twist()
  message.linear.x = delta_x;
  message.angular.z = delta_r; 


  pub.publish(message)


def mover():
  try:
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
      
    rospy.Subscriber("amp_goal_position", MoveBaseGoal, callback, queue_size=1)
    rospy.spin()

  except rospy.ROSInterruptException:
    rospy.loginfo("Navigation Complete.")


if __name__ == '__main__':
  try:
    rospy.init_node('mover', anonymous=True)
    mover()	
  except rospy.ROSInterruptException:
    pass


