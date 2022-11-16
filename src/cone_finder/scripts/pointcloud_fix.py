#!/usr/bin/env python
import struct
import colorsys
import rospy
from sensor_msgs.msg import PointCloud2

def obstacle_cloud_callback(pc):
    global pub
    pc.row_step = pc.point_step * pc.width
    pub.publish(pc);

if __name__ == "__main__":
    try:
        rospy.init_node("pointcloud_fixer")
        rospy.Subscriber("/input", PointCloud2, obstacle_cloud_callback)

        pub = rospy.Publisher("/output", PointCloud2, queue_size=1)
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
