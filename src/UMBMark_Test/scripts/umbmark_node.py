#!/usr/bin/env python
from symbol import pass_stmt
import rospy
from geometry_msgs.msg import Twist
from differential_drive.msg import WheelAngularVelocityPair

import numpy as np
import time

WHEEL_BASE =  0.212     # m
WHEEL_DIA =   0.065     # m
WHEEL_MAX_SPEED = 12.0  # rad/s
SQ_DISTANCE = 1.0          # m

# Commands Twist to a topic to conduct UMBmark test on the robot
class UMBmarkNode:
    def __init__(self, bot_name, wheel_speed):
        rospy.init_node("UMBmarkNode")
        rospy.loginfo("Starting UMBmarkNode as UMBmarkNode.")
        rospy.logdebug("Compare Values: ")
        rospy.logdebug("Sraight Line: " + str(2*SQ_DISTANCE/WHEEL_DIA))
        rospy.logdebug("Rotate: " + str(np.pi*WHEEL_BASE/WHEEL_DIA))

        self.twist_pub = rospy.Publisher("/" + bot_name + "/cmd_vel", Twist, queue_size=10E6)
        self.state_checker = rospy.Subscriber("/ghost/wheel_current_angular_velocity", WheelAngularVelocityPair, self.wheel_vel_cb)
        self.wheel_speed = wheel_speed  # Wheel speed in cm/sec
        self.twist_msg = Twist()
        self.left_wheel_angle = None
        self.right_wheel_angle = None
        
    def run_test_ccw(self):
        pass

    def go_straight(self):
        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.pub_twist()

        # Note the Angles
        starting_left_angle = self.left_wheel_angle
        starting_right_angle = self.right_wheel_angle
        
        # Give Command
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = self.wheel_speed*WHEEL_DIA/2
        self.pub_twist()
        

        # Wait for Completion
        while(max((self.left_wheel_angle - starting_left_angle), (self.right_wheel_angle - starting_right_angle)) < 2*SQ_DISTANCE/WHEEL_DIA):
            print("RIGHT WHEEL: " + str(self.right_wheel_angle - starting_right_angle))
            print("LEFT WHEEL: " + str(self.left_wheel_angle - starting_left_angle))
            print()
            pass

        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.pub_twist()
        
        print("WENT STRAIGHT")

    def turn_cw(self):
        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.pub_twist()

        # Note the Angles
        starting_left_angle = self.left_wheel_angle
        starting_right_angle = self.right_wheel_angle
        
        # Give Command
        self.twist_msg.angular.z =  WHEEL_MAX_SPEED*WHEEL_DIA/WHEEL_BASE /12
        self.twist_msg.linear.x = 0
        self.pub_twist()

        # Wait for Completion
        while((self.right_wheel_angle - starting_right_angle) - (self.left_wheel_angle - starting_left_angle) > -(np.pi*WHEEL_BASE/WHEEL_DIA)):
            print("RIGHT WHEEL: " + str(self.right_wheel_angle - starting_right_angle))
            print("LEFT WHEEL: " + str(self.left_wheel_angle - starting_left_angle))
            print()
            pass

        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.pub_twist()

        print("TURNED CW")

    def turn_ccw(self):
        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.pub_twist()

        # Note the Angles
        starting_left_angle = self.left_wheel_angle
        starting_right_angle = self.right_wheel_angle
        
        # Give Command
        self.twist_msg.angular.z =  WHEEL_MAX_SPEED*WHEEL_DIA/WHEEL_BASE /12
        self.twist_msg.linear.x = 0
        self.pub_twist()

        # Wait for Completion
        while((self.right_wheel_angle - starting_right_angle) - (self.left_wheel_angle - starting_left_angle) < (np.pi*WHEEL_BASE/WHEEL_DIA)):
            print("RIGHT WHEEL: " + str(self.right_wheel_angle - starting_right_angle))
            print("LEFT WHEEL: " + str(self.left_wheel_angle - starting_left_angle))
            print()

        # Stop the Bot
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0
        self.pub_twist()

        print("TURNED CCW")

    def wheel_vel_cb(self, wheel_vel_msg):
        self.left_wheel_angle = wheel_vel_msg.wheel_angle_left
        self.right_wheel_angle = -wheel_vel_msg.wheel_angle_right
    
    def ccw_square(self):
        for i in range(4):
            self.go_straight()
            self.turn_ccw()
        return None

    def cw_square(self):
        for i in range(4):
            self.go_straight()
            self.turn_cw()
        return None

    def pub_twist(self):
        time.sleep(0.05)
        for i in range(4):
            self.twist_pub.publish(self.twist_msg)
            time.sleep(0.2)

if __name__ == "__main__":
    UMBMarktest_node = UMBmarkNode("ghost", WHEEL_MAX_SPEED/4)
    rospy.sleep(4)
    print("STARTING NOW")

    UMBMarktest_node.ccw_square()