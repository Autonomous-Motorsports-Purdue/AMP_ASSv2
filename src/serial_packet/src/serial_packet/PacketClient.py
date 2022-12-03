import rospy
from geometry_msgs.msg import Twist
from amp_msgs.msg import SerialPacket

max_speed = 10


def get_throttle_params(linear):
    if linear > 0:
        return int(linear), 0
    else:
        return 0, int(-linear)


def get_steering_params(angular):
    return int(angular)


class PacketClient(object):

    def __init__(self):
        self.twist_sub = rospy.Subscriber("/cmd_vel", Twist,
                                          self.update_packet)
        self.packet_pub = rospy.Publisher("/serial_packet",
                                          SerialPacket,
                                          queue_size=1000)

    def update_packet(self, msg):
        packet = SerialPacket()
        packet.throttle, _ = get_throttle_params(msg.linear.x)
        packet.steering = get_steering_params(msg.angular.z)

        self.packet_pub.publish(packet)

    def run(self):
        rospy.spin()
