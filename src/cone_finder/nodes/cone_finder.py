#!/usr/bin/env python
import struct
import colorsys
import rospy
from sensor_msgs.msg import PointCloud2
from amp_msgs.msg import ConeList


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


obstacle_cloud = None
obstacle_cloud_dirty = False


def obstacle_cloud_callback(data):
    global obstacle_cloud, obstacle_cloud_dirty
    obstacle_cloud = data
    obstacle_cloud_dirty = True


def filter_loop():
    global obstacle_cloud, obstacle_cloud_dirty

    # Get params for color filtering
    min_h = rospy.get_param("cone_finder/min_h")
    max_h = rospy.get_param("cone_finder/max_h")
    min_s = rospy.get_param("cone_finder/min_s")
    max_s = rospy.get_param("cone_finder/max_s")
    min_v = rospy.get_param("cone_finder/min_v")
    max_v = rospy.get_param("cone_finder/max_v")

    # Get params for point clustering
    cluster_sqr_radius = rospy.get_param("cone_finder/cluster_radius") ** 2
    cluster_count_min = rospy.get_param("cone_finder/cluster_count_min")

    rate = rospy.Rate(rospy.get_param("cone_finder/update_rate"))
    pub = rospy.Publisher("cones_found", ConeList, queue_size=10)
    while not rospy.is_shutdown():
        if obstacle_cloud_dirty:
            # Parse PointCloud2 data and only accept points with correct color
            points = []
            for i in range(obstacle_cloud.width):
                start_byte = i * obstacle_cloud.point_step
                x = struct.unpack("<f", obstacle_cloud.data[start_byte + 0:start_byte + 4])[0]
                y = struct.unpack("<f", obstacle_cloud.data[start_byte + 4:start_byte + 8])[0]
                h, s, v = colorsys.rgb_to_hsv(
                    obstacle_cloud.data[start_byte + 18] / float(255),
                    obstacle_cloud.data[start_byte + 17] / float(255),
                    obstacle_cloud.data[start_byte + 16] / float(255),
                )

                if (min_h <= h <= max_h and min_s <= s <= max_s and min_v <= v <= max_v):
                    points.append(Point(x, y))

            # Perform very basic clustering to find cone positions
            cones = ConeList()
            while len(points) > 0:
                cluster = Cluster()
                basis = points[0]
                cluster.points = [
                    point for point in points if point.sqr_distance(basis) <= cluster_sqr_radius
                ]
                for removed in cluster.points:
                    points.remove(removed)
                if len(cluster.points) >= cluster_count_min:
                    center = cluster.center
                    cones.x.append(center[0])
                    cones.y.append(center[1])

            pub.publish(cones)
            obstacle_cloud_dirty = False

        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("cone_finder")
        rospy.Subscriber("/rtabmap/cloud_obstacles", PointCloud2, obstacle_cloud_callback)
        filter_loop()
    except rospy.ROSInterruptException:
        pass
