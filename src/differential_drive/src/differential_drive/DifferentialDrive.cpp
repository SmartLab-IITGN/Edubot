#include "differential_drive/DifferentialDrive.h"

DifferentialDrive::DifferentialDrive(
    float wheel_base,
    float wheel_radius,
    float max_wheel_speed
) : wheel_base_(wheel_base),
    wheel_radius_(wheel_radius)
{
    setMaxWheelSpeed(max_wheel_speed);
}

float DifferentialDrive::getClippedVeloctiy_(float velocity)
{
    return std::max(- max_wheel_speed_, std::min(max_wheel_speed_, velocity));
}

void DifferentialDrive::setMaxWheelSpeed(float max_wheel_speed)
{
    max_wheel_speed_ = max_wheel_speed;
}

std::pair<float, float> DifferentialDrive::getWheelVelocity(
    float linear_velocity,
    float angular_velocity
) {
    angular_velocity *= wheel_base_ / 2;

    return std::pair<float, float>(
        getClippedVeloctiy_((linear_velocity - angular_velocity) / wheel_radius_),
        getClippedVeloctiy_((linear_velocity + angular_velocity) / wheel_radius_)
    );
}

std::pair<float, float> DifferentialDrive::getBotVelocity(
    float left_wheel_velocity,
    float right_wheel_velocity
) {
    left_wheel_velocity  *= wheel_radius_;
    right_wheel_velocity *= wheel_radius_;

    return std::pair<float, float>(
        (right_wheel_velocity + left_wheel_velocity) / 2.0f,
        (right_wheel_velocity - left_wheel_velocity) / wheel_base_
    );
}