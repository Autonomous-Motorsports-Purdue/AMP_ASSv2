#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from amp_msgs.msg import ConeList


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def sqr_distance(self, other):
        return (self.x - other.x) ** 2 + (self.y - other.y) ** 2


cone_list = ConeList()
cone_list_dirty = False


def cone_list_callback(data):
    global cone_list, cone_list_dirty
    cone_list = data
    cone_list_dirty = True


def constructor_loop():
    global cone_list, cone_list_dirty

    # Get params for construction
    wall_thickness = rospy.get_param("occupancy_constructor/wall_thickness")

    rate = rospy.Rate(rospy.get_param("occupancy_constructor/update_rate"))
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    while not rospy.is_shutdown():
        rospy.logwarn("TODO: occupancy_constructor logic")
        if cone_list_dirty:
            #pub.publish(map)
            cone_list_dirty = False

        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("occupancy_constructor")
        rospy.Subscriber("/cone_finder/cones_found", ConeList, cone_list_callback)
        constructor_loop()
    except rospy.ROSInterruptException:
        pass
