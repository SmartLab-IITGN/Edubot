#include <ros/ros.h>
#include <ros/console.h>

#include "differential_drive/ROSDifferentialDrive.h"

#define WHEEL_BASE  0.212f // m
#define WHEEL_DIA   0.065f // m

#define WHEEL_MAX_SPEED 12.0f // rad/s

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_drive_node");

    ros::NodeHandle node_handle;

    ROSDifferentialDrive ros_differential_drive(node_handle, WHEEL_BASE, WHEEL_DIA / 2, WHEEL_MAX_SPEED);

    ROS_INFO("Differential Drive Node set up successfully.");
    
    ros::spin();

    return 0;
}