#!/usr/bin/env python
import math
import rospy
import numpy as np
from PIL import Image
from PIL import ImageDraw
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

    def find_nearest(self, points):
        nearest_sqr_distance = 1000000 ** 2
        nearest_point = None

        for other in points:
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


def traverse_cone_loop(map_min, map_max, cones, min_loop_cones):
    loop = []
    cones_remaining = cones.copy()
    first_cone = cones[0]
    current_cone = first_cone
    while current_cone.next == None:
        loop.append(current_cone)
        cones_remaining.remove(current_cone)

        map_min = (min(current_cone.x, map_min[0]), min(current_cone.y, map_min[1]))
        map_max = (max(current_cone.x, map_max[0]), max(current_cone.y, map_max[1]))

        current_cone.next = current_cone.find_nearest(cones_remaining)
        current_cone.next.prev = current_cone
        current_cone = current_cone.next

        # Minimum has been met, allow loop closure
        if len(loop) == min_loop_cones:
            cones_remaining.append(first_cone)

    return (loop, map_min, map_max, cones_remaining)


def transform_cone_loop(loop, map_min, grid_resolution):
    # Copy first point to the end, so it closes the loop
    loop.append(Point(loop[0].x, loop[0].y))
    loop_coords = []
    for point in loop:
        # Move all points to be relative to map_min
        point.x -= map_min[0]
        point.y -= map_min[1]
        # Transfer scaled coords to a single list
        loop_coords.append(point.x * grid_resolution)
        loop_coords.append(point.y * grid_resolution)
    return loop_coords


def constructor_loop():
    global cone_list, cone_list_dirty

    # Get params for construction
    min_loop_cones = rospy.get_param("occupancy_constructor/min_loop_cones")
    grid_resolution = rospy.get_param("occupancy_constructor/grid_resolution")
    grid_padding = rospy.get_param("occupancy_constructor/grid_padding")
    wall_thickness = int(rospy.get_param("occupancy_constructor/wall_thickness") * grid_resolution)

    rate = rospy.Rate(rospy.get_param("occupancy_constructor/update_rate"))
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    while not rospy.is_shutdown():
        if cone_list_dirty:
            map_min = (cone_list[0].x, cone_list[0].y)
            map_max = (cone_list[0].x, cone_list[0].y)
            loop1, map_min, map_max, cones_remaining = traverse_cone_loop(map_min, map_max, cone_list, min_loop_cones)
            loop2, map_min, map_max, cones_remaining = traverse_cone_loop(map_min, map_max, cones_remaining, min_loop_cones)

            # Pad the map bounds by 1 meter
            map_min = (map_min[0] - grid_padding, map_min[1] - grid_padding)
            map_max = (map_max[0] + grid_padding, map_max[1] + grid_padding)
            map_size = (map_max[0] - map_min[0], map_max[1] - map_min[0])

            loop1_coords = transform_cone_loop(loop1, map_min, grid_resolution)
            loop2_coords = transform_cone_loop(loop2, map_min, grid_resolution)

            # Initialize OccupancyGrid
            grid = OccupancyGrid()
            grid.header.seq = 0
            grid.header.frame_id = "map"
            grid.info.resolution = 1 / grid_resolution
            grid.info.width = math.ceil(map_size[0] * grid_resolution)
            grid.info.height = math.ceil(map_size[1] * grid_resolution)
            grid.info.origin.position.x = map_min[0]
            grid.info.origin.position.y = map_min[1]
            grid.info.origin.position.z = 0
            grid.info.origin.orientation.x = 0
            grid.info.origin.orientation.y = 0
            grid.info.origin.orientation.z = 0
            grid.info.origin.orientation.w = 1

            # Draw both loops onto an image, then transfer to OccupancyGrid
            img = Image.new("L", (grid.info.width, grid.info.height), 0)  
            draw = ImageDraw.Draw(img)
            draw.line(loop1_coords, fill=100, width=wall_thickness)
            draw.line(loop2_coords, fill=100, width=wall_thickness)
            grid.data = list(img.getdata())

            pub.publish(grid)
            cone_list_dirty = False

        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("occupancy_constructor")
        rospy.Subscriber("/cone_finder/cones_found", ConeList, cone_list_callback)
        constructor_loop()
    except rospy.ROSInterruptException:
        pass
