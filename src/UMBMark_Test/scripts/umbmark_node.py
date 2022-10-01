#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from differential_drive.msg import WheelAngularVelocityPair

import numpy as np

WHEEL_BASE =  0.212     # m
WHEEL_DIA =   0.065     # m
WHEEL_MAX_SPEED = 12.0  # rad/s
SQ_DISTANCE = 4.0          # m

# Commands Twist to a topic to conduct UMBmark test on the robot
class UMBmarkNode:
    def __init__(self, bot_name, wheel_speed):
        rospy.init_node("UMBmarkNode")
        rospy.loginfo("Starting UMBmarkNode as UMBmarkNode.")

        self.twist_pub = rospy.Publisher("/" + bot_name + "/cmd_vel", type, queue_size=10E6)
        self.state_checker = rospy.Subscriber("/ghost/wheel_current_angular_velocity", WheelAngularVelocityPair, self.wheel_vel_cb)
        self.wheel_speed = wheel_speed  # Wheel speed in cm/sec
        self.twist_msg = Twist()
        self.left_wheel_angle = None
        self.right_wheel_angle = None
        
    def run_test(self):
        pass

    def go_straight(self):
        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.twist_pub.publish(self.twist_msg)

        # Note the Angles
        starting_left_angle = self.left_wheel_angle
        starting_right_angle = self.right_wheel_angle
        
        # Give Command
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = self.wheel_speed*WHEEL_DIA/2
        self.twist_pub.publish(self.twist_msg)

        # Wait for Completion
        while(max((self.left_wheel_angle - starting_left_angle), (self.right_wheel_angle - starting_right_angle)) < 2*SQ_DISTANCE/WHEEL_DIA):
            pass

        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.twist_pub.publish(self.twist_msg)

    def turn_cw(self):
        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.twist_pub.publish(self.twist_msg)

        # Note the Angles
        starting_left_angle = self.left_wheel_angle
        starting_right_angle = self.right_wheel_angle
        
        # Give Command
        self.twist_msg.angular.z =  self.wheel_speed*WHEEL_DIA/WHEEL_BASE 
        self.twist_msg.linear.x = 0
        self.twist_pub.publish(self.twist_msg)

        # Wait for Completion
        while((self.right_wheel_angle - starting_right_angle) - (self.left_wheel_angle - starting_left_angle) > -(np.pi*WHEEL_BASE/WHEEL_DIA)):
            pass

        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.twist_pub.publish(self.twist_msg)

    def turn_ccw(self):
        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.twist_pub.publish(self.twist_msg)

        # Note the Angles
        starting_left_angle = self.left_wheel_angle
        starting_right_angle = self.right_wheel_angle
        
        # Give Command
        self.twist_msg.angular.z =  self.wheel_speed*WHEEL_DIA/WHEEL_BASE 
        self.twist_msg.linear.x = 0
        self.twist_pub.publish(self.twist_msg)

        # Wait for Completion
        while((self.right_wheel_angle - starting_right_angle) - (self.left_wheel_angle - starting_left_angle) < (np.pi*WHEEL_BASE/WHEEL_DIA)):
            pass

        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.twist_pub.publish(self.twist_msg)

    def wheel_vel_cb(self, wheel_vel_msg):
        self.left_wheel_angle = wheel_vel_msg.wheel_angle_left
        self.right_wheel_angle = wheel_vel_msg.wheel_angle_right
        
        
    


if __name__ == "__main__":
    name_node = UMBmarkNode("ghost")
    rospy.spin()