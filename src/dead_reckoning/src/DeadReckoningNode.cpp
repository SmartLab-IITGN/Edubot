#include <ros/ros.h>
#include <ros/console.h>

#include "dead_reckoning/ROSDeadReckoning.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dead_reckoning_node");

    ros::NodeHandle node_handle;

    std::string odom_init_frame_name;
    node_handle.param<std::string> ("odom_init_frame", odom_init_frame_name, "odom");

    ROSDeadReckoning ros_dead_reckoning(node_handle, odom_init_frame_name);

    ROS_INFO("Dead Reckoning Node set up successfully.");
    
    ros::spin();

    return 0;
}