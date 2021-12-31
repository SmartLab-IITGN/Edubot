#include <ros/ros.h>
#include <ros/console.h>

#include "dead_reckoning/ROSDeadReckoning.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dead_reckoning_node");

    ros::NodeHandle node_handle;

    std::string map_frame_name;
    node_handle.param<std::string> ("map_frame", map_frame_name, "map");

    ROSDeadReckoning ros_dead_reckoning(node_handle, map_frame_name);

    ROS_INFO("Dead Reckoning Node set up successfully.");
    
    ros::spin();

    return 0;
}