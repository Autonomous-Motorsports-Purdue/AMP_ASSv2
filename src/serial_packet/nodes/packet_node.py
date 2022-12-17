import rospy
from serial_packet import PacketClient

import sys
from time import sleep

if __name__ == "__main__":
    rospy.init_node("serial_node")
    rospy.loginfo("Autonomous Motorsports Purdue ROS Serial Node")

    twist_topic = rospy.get_param("~twist_topic", "/cmd_vel")
    packet_topic = rospy.get_param("~packet_topic", "/serial_packet")

    while not rospy.is_shutdown():
        rospy.loginfo("Running AMP ROS Serial Node")
        try:
            client = PacketClient(twist_topic=twist_topic,
                                  packet_topic=packet_topic)
            client.run()
        except KeyboardInterrupt:
            break
        except OSError:
            sleep(1.0)
            continue
        except:
            rospy.logwarn("Unexpected Error: %s", sys.exc_info()[0])
            sleep(1.0)
            continue
