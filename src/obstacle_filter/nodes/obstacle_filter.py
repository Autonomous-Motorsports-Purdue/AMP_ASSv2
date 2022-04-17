#!/usr/bin/env python
import rospy
import math
import struct
from sensor_msgs.msg import PointCloud2
from obstacle_filter_msgs.msg import ConeList
        
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def sqr_distance(self, other):
        return (self.x - other.x) ** 2 + (self.y - other.y) ** 2
        
class Cluster:
    def __init__(self):
        self.points = []
        
    @property
    def center(self):
        center_x = 0
        center_y = 0
        for point in self.points:
            center_x += point.x
            center_y += point.y
        return (center_x / len(self.points), center_y / len(self.points))

obstacle_cloud = PointCloud2()
obstacle_cloud_dirty = False
    
def obstacle_cloud_callback(data):
    global obstacle_cloud, obstacle_cloud_dirty
    obstacle_cloud = data
    obstacle_cloud_dirty = True

def filter_loop():
    global obstacle_cloud, obstacle_cloud_dirty
    
    min_r = rospy.get_param("/obstacle_filter/min_r")
    max_r = rospy.get_param("/obstacle_filter/max_r")
    min_g = rospy.get_param("/obstacle_filter/min_g")
    max_g = rospy.get_param("/obstacle_filter/max_g")
    min_b = rospy.get_param("/obstacle_filter/min_b")
    max_b = rospy.get_param("/obstacle_filter/max_b")
    
    cluster_sqr_radius = rospy.get_param("/obstacle_filter/cluster_radius") ** 2
    cluster_count_min = rospy.get_param("/obstacle_filter/cluster_count_min")
    
    pub = rospy.Publisher('/obstacle_filter/obstacles', ConeList, queue_size=10)
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
                
                if (r >= min_r and r <= max_r and
                    g >= min_g and g <= max_g and
                    b >= min_b and b <= max_b):
                    points.append(Point(x, y))
            
            clusters = []
            while len(points) > 0:
                cluster = Cluster()
                basis = points[0]
                cluster.points = [point for point in points if point.sqr_distance(basis) <= cluster_sqr_radius]
                for removed in cluster.points:
                    points.remove(removed)
                if len(cluster.points) >= cluster_count_min:
                    clusters.append(cluster)
            
            centers = [cluster.center for cluster in clusters]
            cones = ConeList()
            cones.x = [center[0] for center in centers]
            cones.y = [center[1] for center in centers]
            
            pub.publish(cones)
            obstacle_cloud_dirty = False
        
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_filter')
        rospy.Subscriber('/rtabmap/cloud_obstacles', PointCloud2, obstacle_cloud_callback)
        filter_loop()
    except rospy.ROSInterruptException:
        pass
