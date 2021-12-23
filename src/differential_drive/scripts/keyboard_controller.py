#!/usr/bin/env python

# Create ROS node to publish on the ROS Topic "/turtle1/cmd_vel"
# to direct the turtle to travel in a circle

# Gives you access to python methods for creating
# ROS nodes and publisher and subscribers
import rospy

# Import the data type / message type for subscribing / publishing to ROS Topic
from geometry_msgs.msg import Twist, TwistStamped

# For detecting key presses
import curses

lin_vel_step = 0.02
ang_vel_step = 0.25

v = 0
w = 0

def current_velocity_callback(ros_msg) :

    global v, w

    v = ros_msg.twist.linear.x
    w = ros_msg.twist.angular.z
    
    pass

def clear(ros_msg) :

    ros_msg.linear.x = 0
    ros_msg.linear.y = 0
    ros_msg.linear.z = 0

    ros_msg.angular.x = 0
    ros_msg.angular.y = 0
    ros_msg.angular.z = 0

    pass    

# My main code will perform this method continuously
def talker(win):

    # Code for key presses is copied from 
    # https://stackoverflow.com/a/32386410
    win.nodelay(True)
    key=''
    win.clear()                
    win.addstr('Detected key:')
    win.addstr('\nCurrent Velocity:\n')
    win.addstr('\tv = ' + str(v) + ' m/s\n')
    win.addstr('\tw = ' + str(w) + ' rad/s')

    # Declaring a publisher object to publish to /turtle1/cmd_vel
    # The message type for the topic /turtle1/cmd_vel is Twist
    pub = rospy.Publisher('cmd_vel', Twist)

    # Creating a message template of Twist data class
    # for publishing to /turtle1/cmd_vel
    msg = Twist()
    
    # Initialize my ROS node
    rospy.init_node('keyboard_teleop')

    sub = rospy.Subscriber('vel', TwistStamped, current_velocity_callback)
    
    # Main while loop
    while not rospy.is_shutdown():
        
        try :
            
            key = win.getkey()         
            win.clear()                
            win.addstr('Detected key:')
            win.addstr(str(key)) 
            win.addstr('\nCurrent Velocity:\n')
            win.addstr('\tv = ' + str(v) + ' m/s\n')
            win.addstr('\tw = ' + str(w) + ' rad/s')
            
            # clear(msg)

            if key == 'w' :

                msg.linear.x += lin_vel_step
                pub.publish(msg)

            if key == 's' :

                msg.linear.x -= lin_vel_step
                pub.publish(msg)

            if key == 'a' :

                msg.angular.z += ang_vel_step
                pub.publish(msg)

            if key == 'd' :

                msg.angular.z -= ang_vel_step
                pub.publish(msg)

            if key == ' ' :

                clear(msg)
                pub.publish(msg)

        except Exception as e :
            # If no key is pressed
            # win.getkey() throws an exception
            pass
    pass

try:
    
    curses.wrapper(talker)

except rospy.ROSInterruptException:
    
    pass