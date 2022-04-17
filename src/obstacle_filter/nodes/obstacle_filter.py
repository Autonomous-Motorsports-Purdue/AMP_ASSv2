#!/usr/bin/env python
import rospy
import math
import struct
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray
        
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def sqr_distance(self, other):
        return (self.x - other.x) ** 2 + (self.y - other.y) ** 2

obstacle_cloud = PointCloud2()
obstacle_cloud_dirty = False
    
def obstacle_cloud_callback(data):
    global obstacle_cloud, obstacle_cloud_dirty
    obstacle_cloud = data
    obstacle_cloud_dirty = True

def filter_loop():
    global obstacle_cloud, obstacle_cloud_dirty
    
    pub = rospy.Publisher('/obstacle_filter/obstacles', PoseArray, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        if obstacle_cloud_dirty:
            points = []
            for i in range(obstacle_cloud.width):
                start_byte = i * obstacle_cloud.point_step
                x = struct.unpack("<f", obstacle_cloud.data[start_byte + 0:start_byte + 4])[0]
                y = struct.unpack("<f", obstacle_cloud.data[start_byte + 4:start_byte + 8])[0]
                #z = struct.unpack("<f", obstacle_cloud.data[start_byte + 8:start_byte + 12])[0]
                r = obstacle_cloud.data[start_byte + 16]
                g = obstacle_cloud.data[start_byte + 17]
                b = obstacle_cloud.data[start_byte + 18]
                
                if (r >= 0 and r <= 255 and
                    g >= 0 and g <= 255 and
                    b >= 0 and b <= 255):
                    points.append(Point(x, y))
            
            for point in points:
                rospy.logwarn((point.x, point.y))
            obstacle_cloud_dirty = False
        
        # cluster obstacle_cloud into PoseArray called "cones"
        # rospy.logwarn("TODO: obstacle_filter logic")
        #pub.publish(cones)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_filter')
        rospy.Subscriber('/rtabmap/cloud_obstacles', PointCloud2, obstacle_cloud_callback)
        filter_loop()
    except rospy.ROSInterruptException:
        pass
