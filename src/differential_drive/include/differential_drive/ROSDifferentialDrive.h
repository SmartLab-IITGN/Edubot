#ifndef __ROS_DIFFERENTIAL_DRIVE__
#define __ROS_DIFFERENTIAL_DRIVE__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <differential_drive/SetFloatParam.h>
#include <differential_drive/WheelAngularVelocityPair.h>

#include "differential_drive/DifferentialDrive.h"

class ROSDifferentialDrive : public DifferentialDrive 
{
    private:

        ros::Publisher vel_pub_;
        geometry_msgs::TwistStamped vel_msg_;
        
        ros::Publisher wheel_targ_ang_vel_pub_;
        differential_drive::WheelAngularVelocityPair wheel_targ_ang_vel_msg_;
        
        ros::Subscriber wheel_curr_ang_vel_sub_;
        void wheelCurrAngVelCb_(const differential_drive::WheelAngularVelocityPair &wheel_curr_ang_vel_msg);

        ros::Subscriber cmdvel_sub_;
        void cmdvelCb_(const geometry_msgs::Twist &cmd_vel_msg);
        
        ros::ServiceServer set_max_wheel_speed_srv_server_;
        bool setMaxWheelSpeedSrvCb_(
            differential_drive::SetFloatParam::Request  &srv_rqst,
            differential_drive::SetFloatParam::Response &srv_resp
        );

        void init_pubs_(ros::NodeHandle &node_handle);
        void init_subs_(ros::NodeHandle &node_handle);
        void init_srv_servers_(ros::NodeHandle &node_handle);
    
    public:
    
        ROSDifferentialDrive(
            ros::NodeHandle &node_handle,
            float wheel_base,
            float wheel_radius,
            float max_wheel_speed = 1.0
        );
};

#endif