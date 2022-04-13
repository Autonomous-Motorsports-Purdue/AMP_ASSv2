#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray

obstacle_cloud = PointCloud2()
    
def obstacle_cloud_callback(data):
    global obstacle_cloud
    obstacle_cloud = data

def filter_loop():
    pub = rospy.Publisher('/obstacle_filter/obstacles', PoseArray, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        # filter obstacle_cloud
        # cluster obstacle_cloud into PoseArray called "cones"
        rospy.logwarn("TODO: obstacle_filter logic")
        
        #pub.publish(cones)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_filter')
        rospy.Subscriber('/rtabmap/cloud_obstacles', PointCloud2, obstacle_cloud_callback)
        filter_loop()
    except rospy.ROSInterruptException:
        pass
