#!/usr/bin/env python
import math
import os
import rospy
from amp_msgs.msg import ConeList


def fake_loop():
    rate = rospy.Rate(rospy.get_param("cone_finder/update_rate"))
    pub = rospy.Publisher("cones_found", ConeList, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(fake_cones)
        rate.sleep()


if __name__ == "__main__":

    if os.getenv("DEBUG") is not None:
        # Testing branch
        # TODO: Find usecases for such a branch and better implement it.

        # Generates two concentric rings of cones, clumped in groups of 3
        radius = 1
        count = 8
        multiplier = 3

        fake_cones = ConeList()
        fake_cones.x = []
        fake_cones.y = []
        for i in range(int(count * multiplier)):
            rad = (i / (count * multiplier)) * math.pi * 2 - (i % 3) * 0.15
            # outer ring
            fake_cones.x.append(math.cos(rad) * radius * multiplier)
            fake_cones.y.append(math.sin(rad) * radius * multiplier)

            # inner ring, less dense
            if i % multiplier == 0:
                fake_cones.x.append(math.cos(rad) * radius)
                fake_cones.y.append(math.sin(rad) * radius)
    else:
        # Load premade track data
        fake_cones = ConeList()
        fake_cones.x = rospy.get_param("cone_finder/fake_cones_x")
        fake_cones.y = rospy.get_param("cone_finder/fake_cones_y")

    try:
        rospy.init_node("cone_finder")
        fake_loop()
    except rospy.ROSInterruptException:
        pass
