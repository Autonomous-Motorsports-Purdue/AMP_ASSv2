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
    fake_cones = ConeList()
    fake_cones.x = []
    fake_cones.y = []
    for i in range(48):
        rad = (i / 48) * math.pi * 2
        # outer ring
        fake_cones.x.append(math.cos(rad) * 3)
        fake_cones.y.append(math.sin(rad) * 3)

        # inner ring, less dense
        if i % 3 == 0:
            fake_cones.x.append(math.cos(rad))
            fake_cones.y.append(math.sin(rad))

    try:
        rospy.init_node("cone_finder")
        fake_loop()
    except rospy.ROSInterruptException:
        pass
