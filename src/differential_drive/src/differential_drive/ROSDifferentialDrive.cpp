#include "differential_drive/ROSDifferentialDrive.h"

ROSDifferentialDrive::ROSDifferentialDrive(
    ros::NodeHandle &node_handle,
    float wheel_base,
    float wheel_radius,
    float max_wheel_speed
) : DifferentialDrive(
        wheel_base,
        wheel_radius,
        max_wheel_speed
    )
{
    init_pubs_(node_handle);
    init_subs_(node_handle);
    init_srv_servers_(node_handle);
}

void ROSDifferentialDrive::init_subs_(
    ros::NodeHandle &node_handle
) {
    cmdvel_sub_ = node_handle.subscribe("cmd_vel", 1000, &ROSDifferentialDrive::cmdvelCb_, this);

    wheel_curr_ang_vel_sub_ = node_handle.subscribe(
        "wheel_current_angular_velocity", 1000,
        &ROSDifferentialDrive::wheelCurrAngVelCb_, this
    );
}

void ROSDifferentialDrive::init_pubs_(
    ros::NodeHandle &node_handle
) {
    wheel_targ_ang_vel_pub_ = node_handle.advertise<differential_drive::WheelAngularVelocityPair> ("wheel_target_angular_velocity",  1000);
    
    vel_pub_ = node_handle.advertise<geometry_msgs::TwistStamped> ("vel", 1000);
}

void ROSDifferentialDrive::init_srv_servers_(
    ros::NodeHandle &node_handle
) {
    set_max_wheel_speed_srv_server_ = node_handle.advertiseService(
        "set_max_wheel_speed",
        &ROSDifferentialDrive::setMaxWheelSpeedSrvCb_,
        this
    );
}

bool ROSDifferentialDrive::setMaxWheelSpeedSrvCb_(
    differential_drive::SetFloatParam::Request  &srv_rqst,
    differential_drive::SetFloatParam::Response &srv_resp
) {
    setMaxWheelSpeed(srv_rqst.val);
    srv_resp.success = true;
    return true;
}

void ROSDifferentialDrive::cmdvelCb_(const geometry_msgs::Twist &msg)
{
    std::pair<float, float> wheel_vel = getWheelVelocity(msg.linear.x, msg.angular.z);

    wheel_targ_ang_vel_msg_.wheel_angular_velocity_left  = wheel_vel.first;
    wheel_targ_ang_vel_msg_.wheel_angular_velocity_right = wheel_vel.second;

    wheel_targ_ang_vel_pub_.publish(wheel_targ_ang_vel_msg_);
}

void ROSDifferentialDrive::wheelCurrAngVelCb_(const differential_drive::WheelAngularVelocityPair &msg)
{
    std::pair<float, float> bot_vel = getBotVelocity(msg.wheel_angular_velocity_left, msg.wheel_angular_velocity_right);

    vel_msg_.header.frame_id = "base_link";
    // vel_msg_.header.seq is updated automatically
    vel_msg_.header.stamp = ros::Time::now();

    vel_msg_.twist.linear.x  = bot_vel.first;
    vel_msg_.twist.angular.z = bot_vel.second;

    vel_pub_.publish(vel_msg_);
}