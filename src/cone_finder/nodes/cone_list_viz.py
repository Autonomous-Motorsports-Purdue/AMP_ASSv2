#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from amp_msgs.msg import ConeList

cones = ConeList()
cones_dirty = False

def conelist_callback(data):
    global cones, cones_dirty
    cones = data
    cones_dirty = True

def viz_loop():
    global cones, cones_dirty

    rate = rospy.Rate(60) # 60hz
    pub = rospy.Publisher('markers', MarkerArray, queue_size=10)
    while not rospy.is_shutdown():
        if cones_dirty:
            markers = MarkerArray()
            for i in range(len(cones.x)):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.id = i
                marker.type = marker.CYLINDER
                marker.action = marker.ADD
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15
                marker.color.r = 1.0
                marker.color.g = 0.35
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = cones.x[i]
                marker.pose.position.y = cones.y[i]
                marker.pose.position.z = 0.075
                markers.markers.append(marker)
            pub.publish(markers)

            cones_dirty = False

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('cone_list_viz')
        rospy.Subscriber('cones_found', ConeList, conelist_callback)
        viz_loop()
    except rospy.ROSInterruptException:
        pass
