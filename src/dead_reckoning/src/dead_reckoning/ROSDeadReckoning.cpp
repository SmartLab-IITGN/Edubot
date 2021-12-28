#include "dead_reckoning/ROSDeadReckoning.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ROSDeadReckoning::ROSDeadReckoning(
    ros::NodeHandle &nh
) : DeadReckoning(0, 0, 0, 0, 0, 0)   
{
    last_vel_cb_time_ = ros::Time::now().toSec();

    init_pubs_(nh);
    init_subs_(nh);
    init_srv_servers_(nh);
}

void ROSDeadReckoning::init_subs_(
    ros::NodeHandle &node_handle
) {
    vel_sub_ = node_handle.subscribe("vel", 1000, &ROSDeadReckoning::velCb_, this);
}

void ROSDeadReckoning::init_pubs_(
    ros::NodeHandle &node_handle
) {
    odom_pub_ = node_handle.advertise<nav_msgs::Odometry> ("posture",  1000);
}

void ROSDeadReckoning::init_srv_servers_(
    ros::NodeHandle &node_handle
) {
    set_pose_srv_server_ = node_handle.advertiseService(
        "set_posture",
        &ROSDeadReckoning::setPoseSrvCb_,
        this
    );
}

void ROSDeadReckoning::velCb_(const geometry_msgs::TwistStamped &msg)
{
    Eigen::Vector2f q;
    q << msg.twist.linear.x, msg.twist.angular.z;

    long double curr_time = msg.header.stamp.toSec();

    implicitUpdate(q, curr_time - last_vel_cb_time_);

    last_vel_cb_time_ = curr_time;

    odom_msg_.header.stamp = msg.header.stamp;

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id  = "odom";

    Eigen::Vector3f p, p_dot;
    p = getPosture();
    p_dot = getPostureTimeDerivative();

    odom_msg_.pose.pose.position.x = p(0);
    odom_msg_.pose.pose.position.y = p(1);

    tf2::Quaternion orientation;

    orientation.setRPY(0, 0, p(2));
    
    odom_msg_.pose.pose.orientation = tf2::toMsg(orientation);

    odom_msg_.twist.twist.linear.x = p_dot(0);
    odom_msg_.twist.twist.linear.y = p_dot(1);

    odom_msg_.twist.twist.angular.z = p_dot(2);

    odom_pub_.publish(odom_msg_);
}

bool ROSDeadReckoning::setPoseSrvCb_(
    dead_reckoning::SetStampedPoseParamRequest &srv_rqst,
    dead_reckoning::SetStampedPoseParamResponse &srv_resp
) {
    last_vel_cb_time_ = srv_rqst.pose.header.stamp.toSec();

    tf2::Quaternion orientation;

    tf2::fromMsg(srv_rqst.pose.pose.orientation, orientation);

    tf2Scalar roll, pitch, yaw;

    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    setPosture(
        srv_rqst.pose.pose.position.x,
        srv_rqst.pose.pose.position.y,
        yaw
    );

    srv_resp.success = true;

    return true;
}