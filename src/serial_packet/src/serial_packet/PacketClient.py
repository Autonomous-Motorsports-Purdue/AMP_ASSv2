import rospy
from geometry_msgs.msg import Twist
from amp_msgs.msg import SerialPacket

max_speed = 10


def get_throttle_params(linear):
    if linear >= 0:
        return int(linear * 100), 1
    else:
        return int(-linear * 100), -1


def get_steering_params(angular):
    return int(angular * 1000 + 3000)


class PacketClient(object):

    def __init__(self, twist_topic="/cmd_vel", packet_topic="/serial_packet"):
        self.twist_sub = rospy.Subscriber(twist_topic, Twist,
                                          self.update_packet)
        self.packet_pub = rospy.Publisher(packet_topic,
                                          SerialPacket,
                                          queue_size=1000)

    def update_packet(self, msg):
        packet = SerialPacket()
        packet.throttle, packet.direction = get_throttle_params(msg.linear.x)
        packet.steering = get_steering_params(msg.angular.z)

        self.packet_pub.publish(packet)

    def run(self):
        rospy.spin()
