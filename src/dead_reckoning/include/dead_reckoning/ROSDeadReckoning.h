#ifndef __ROS_DEAD_RECKONING__
#define __ROS_DEAD_RECKONING__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

#include "dead_reckoning/DeadReckoning.h"
#include "dead_reckoning/SetStampedPoseParam.h"

class ROSDeadReckoning : public DeadReckoning
{
    private:

        long double last_vel_cb_time_;

        ros::Subscriber vel_sub_;
        void velCb_(const geometry_msgs::TwistStamped &msg);

        ros::Publisher odom_pub_;
        nav_msgs::Odometry odom_msg_;

        ros::ServiceServer set_pose_srv_server_;
        bool setPoseSrvCb_(
            dead_reckoning::SetStampedPoseParamRequest &,
            dead_reckoning::SetStampedPoseParamResponse &
        );

        void init_pubs_(ros::NodeHandle &node_handle);
        void init_subs_(ros::NodeHandle &node_handle);
        void init_srv_servers_(ros::NodeHandle &node_handle);

    public:
    
        ROSDeadReckoning(
            ros::NodeHandle &node_handle
        );
};

#endif