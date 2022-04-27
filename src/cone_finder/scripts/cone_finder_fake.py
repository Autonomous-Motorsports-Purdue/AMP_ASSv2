#!/usr/bin/env python
import rospy
import math
from amp_msgs.msg import ConeList


def fake_loop():
    rate = rospy.Rate(rospy.get_param("cone_finder/update_rate"))
    pub = rospy.Publisher("cones_found", ConeList, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(fake_cones)
        rate.sleep()


if __name__ == "__main__":
    # Generates two concentric rings of cones, clumped in groups of 3
    """inner_radius = 1;
    inner_count = 8;
    outer_multiplier = 3;

    fake_cones = ConeList()
    fake_cones.x = []
    fake_cones.y = []
    for i in range(int(inner_count * outer_multiplier)):
        rad = (i / (inner_count * outer_multiplier)) * math.pi * 2 - (i % 3) * 0.15
        # outer ring
        fake_cones.x.append(math.cos(rad) * inner_radius * outer_multiplier)
        fake_cones.y.append(math.sin(rad) * inner_radius * outer_multiplier)

        # inner ring, less dense
        if i % outer_multiplier == 0:
            fake_cones.x.append(math.cos(rad) * inner_radius)
            fake_cones.y.append(math.sin(rad) * inner_radius)"""

    # Load premade track data
    fake_cones = ConeList()
    fake_cones.x = rospy.get_param("cone_finder/fake_cones_x")
    fake_cones.y = rospy.get_param("cone_finder/fake_cones_y")

    try:
        rospy.init_node("cone_finder")
        fake_loop()
    except rospy.ROSInterruptException:
        pass
