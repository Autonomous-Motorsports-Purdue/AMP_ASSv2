#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped


def set_throttle_steer(data):
    throttle = data.drive.speed / 0.1
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(throttle)
    pub_vel_right_rear_wheel.publish(throttle)
    pub_vel_left_front_wheel.publish(throttle)
    pub_vel_right_front_wheel.publish(throttle)

    pub_pos_left_steering_hinge.publish(steer)
    pub_pos_right_steering_hinge.publish(steer)


def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    global pub_vel_left_rear_wheel
    global pub_vel_right_rear_wheel
    global pub_vel_left_front_wheel
    global pub_vel_right_front_wheel

    global pub_pos_left_steering_hinge
    global pub_pos_right_steering_hinge

    pub_vel_left_rear_wheel = \
        rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = \
        rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = \
        rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = \
        rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = \
        rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = \
        rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    rospy.Subscriber("/racecar_cmd", AckermannDriveStamped, set_throttle_steer)

    rospy.spin()


if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
