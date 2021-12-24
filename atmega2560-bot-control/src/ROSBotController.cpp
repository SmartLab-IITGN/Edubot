/**
 * @file ROSBotController.cpp
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief 
 * @version 0.1
 * @date 2021-07-27
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "ROSBotController.h"

// Constructor
ROSBotController::ROSBotController(
    MotorController &motor_left,
    MotorController &motor_right
) : // Mem Initialization list
    motor_controller_left_(motor_left),
    motor_controller_right_(motor_right),
    // Subscriber Constructor
    target_angular_velocity_subscriber_(
        "wheel_target_angular_velocity",
        &ROSBotController::targetAngularVelocityCallback_,
        this
    ),
    // Publisher Constructor
    current_angular_velocity_publisher_(
        "wheel_current_angular_velocity",
        &current_angular_velocity_message_
    ),
    // Service Servers' Constructors
    stop_bot_service_server_(
        "stop_bot",
        &ROSBotController::stopBotServiceCallback_,
        this
    ),
    move_bot_service_server_(
        "move_bot",
        &ROSBotController::moveBotServiceCallback_,
        this
    ),
    set_PID_gains_left_motor_service_server_(
        "left_motor_controller/set_PID_gains",
        &ROSBotController::setPIDGainsLeftMotorServiceCallback_,
        this
    ),
    set_PID_gains_right_motor_service_server_(
        "right_motor_controller/set_PID_gains",
        &ROSBotController::setPIDGainsRightMotorServiceCallback_,
        this
    )
{
    // Nothing to do 
    ;
}

void ROSBotController::targetAngularVelocityCallback_(const differential_drive::WheelAngularVelocityPair &target_velocity_message)
{
    // Set PID target to the desired target velocity
    motor_controller_left_.setMotorAngularVelocity(target_velocity_message.wheel_angular_velocity_left);
    motor_controller_right_.setMotorAngularVelocity(target_velocity_message.wheel_angular_velocity_right);
}

void ROSBotController::initialize(ros::NodeHandle &node_handle)
{
    node_handle.subscribe(target_angular_velocity_subscriber_);
    node_handle.advertise(current_angular_velocity_publisher_);
    
    node_handle.advertiseService(stop_bot_service_server_);
    node_handle.advertiseService(move_bot_service_server_);

    node_handle.advertiseService(set_PID_gains_left_motor_service_server_);
    node_handle.advertiseService(set_PID_gains_right_motor_service_server_);
}

void ROSBotController::publish()
{
    current_angular_velocity_message_.wheel_angular_velocity_left  = motor_controller_left_.getMotorAngularVelocity();
    current_angular_velocity_message_.wheel_angular_velocity_right = motor_controller_right_.getMotorAngularVelocity();
    
    current_angular_velocity_publisher_.publish(&current_angular_velocity_message_);
}

void ROSBotController::stopBotServiceCallback_(const std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response)
{
    motor_controller_left_.stopMotor();
    motor_controller_right_.stopMotor();

    response.success = true;
    response.message = "Bot stopped successfully";
}

void ROSBotController::moveBotServiceCallback_(const std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response)
{
    motor_controller_left_.enablePIDControl();
    motor_controller_right_.enablePIDControl();

    response.success = true;
    response.message = "Bot velocity PID control enabled";
}

void ROSBotController::setPIDGainsLeftMotorServiceCallback_(const control_toolbox::SetPidGainsRequest &request, control_toolbox::SetPidGainsResponse &response)
{
    motor_controller_left_.stopMotor();

    motor_controller_left_.setPIDGains(
        request.p,
        request.i,
        request.d
    );

    motor_controller_left_.enablePIDControl();
}

void ROSBotController::setPIDGainsRightMotorServiceCallback_(const control_toolbox::SetPidGainsRequest &request, control_toolbox::SetPidGainsResponse &response)
{
    motor_controller_right_.stopMotor();

    motor_controller_right_.setPIDGains(
        request.p,
        request.i,
        request.d
    );

    motor_controller_right_.enablePIDControl();
}