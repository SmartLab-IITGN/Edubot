/**
 * @file ROSBotController.h
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief 
 * @version 0.1
 * @date 2021-08-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __ROS_BOT_CONTROLLER__
#define __ROS_BOT_CONTROLLER__

#include "ros.h"
#include <std_srvs/Trigger.h>
#include <control_toolbox/SetPidGains.h>
#include "differential_drive/WheelAngularVelocityPair.h"

#include "MotorController.h"

class ROSBotController
{
    private :

        MotorController &motor_controller_left_;
        MotorController &motor_controller_right_;

        ros::Subscriber<differential_drive::WheelAngularVelocityPair, ROSBotController> target_angular_velocity_subscriber_;
        void targetAngularVelocityCallback_(const differential_drive::WheelAngularVelocityPair &target_angular_velocity_message);
        
        differential_drive::WheelAngularVelocityPair current_angular_velocity_message_;
        ros::Publisher current_angular_velocity_publisher_;

        ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse, ROSBotController> stop_bot_service_server_;
        void stopBotServiceCallback_(const std_srvs::TriggerRequest&, std_srvs::TriggerResponse&);

        ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse, ROSBotController> move_bot_service_server_;
        void moveBotServiceCallback_(const std_srvs::TriggerRequest&, std_srvs::TriggerResponse&);

        ros::ServiceServer<control_toolbox::SetPidGainsRequest, control_toolbox::SetPidGainsResponse, ROSBotController> set_PID_gains_left_motor_service_server_;
        void setPIDGainsLeftMotorServiceCallback_(const control_toolbox::SetPidGainsRequest&, control_toolbox::SetPidGainsResponse&);

        ros::ServiceServer<control_toolbox::SetPidGainsRequest, control_toolbox::SetPidGainsResponse, ROSBotController> set_PID_gains_right_motor_service_server_;
        void setPIDGainsRightMotorServiceCallback_(const control_toolbox::SetPidGainsRequest&, control_toolbox::SetPidGainsResponse&);

    public :

        ROSBotController(
            MotorController &motor_left,
            MotorController &motor_right
        );

        void initialize(ros::NodeHandle &node_handle);

        void publish();
};

#endif