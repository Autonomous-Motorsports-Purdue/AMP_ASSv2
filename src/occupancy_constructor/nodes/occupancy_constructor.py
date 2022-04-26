#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from amp_msgs.msg import ConeList


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.next = None
        self.prev = None

    def sqr_distance(self, other):
        return (self.x - other.x) ** 2 + (self.y - other.y) ** 2

    def find_nearest(self, points, exclude):
        nearest_sqr_distance = 1000000 ** 2
        nearest_point = None

        for other in points:
            if other == self or other == exclude:
                continue
            sqr_distance = self.sqr_distance(other)
            if sqr_distance < nearest_sqr_distance:
                nearest_point = other
                nearest_sqr_distance = sqr_distance

        return nearest_point


cone_list = None
cone_list_dirty = False


def cone_list_callback(data):
    global cone_list, cone_list_dirty
    cone_list = []
    for i in range(len(data.x)):
        cone_list.append(Point(data.x[i], data.y[i]))
    cone_list_dirty = True


def constructor_loop():
    global cone_list, cone_list_dirty

    # Get params for construction
    wall_thickness = rospy.get_param("occupancy_constructor/wall_thickness")

    rate = rospy.Rate(rospy.get_param("occupancy_constructor/update_rate"))
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    while not rospy.is_shutdown():
        if cone_list_dirty:
            # Traverse loop of cones, keeping track of which haven't been traversed
            cones_remaining = cone_list.copy()
            current_cone = cone_list[0]
            loop1_start = current_cone
            while current_cone.next == None:
                cones_remaining.remove(current_cone)
                current_cone.next = current_cone.find_nearest(cone_list, current_cone.prev)
                current_cone.next.prev = current_cone
                current_cone = current_cone.next
                rospy.logwarn(("loop1: ", current_cone.x, current_cone.y))

            # Traverse loop again, this time excluding cones included in the first loop
            current_cone = cones_remaining[0]
            loop2_start = current_cone
            while current_cone.next == None:
                current_cone.next = current_cone.find_nearest(cones_remaining, current_cone.prev)
                current_cone.next.prev = current_cone
                current_cone = current_cone.next
                rospy.logwarn(("loop2: ", current_cone.x, current_cone.y))

            # Create OccupancyGrid and draw both cone loops with loop1_start and loop2_start
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
