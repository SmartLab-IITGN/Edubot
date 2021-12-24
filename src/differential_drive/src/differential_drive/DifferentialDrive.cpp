#include "differential_drive/DifferentialDrive.h"

#include <math.h>

DifferentialDrive::DifferentialDrive(
    float wheel_base,
    float wheel_radius,
    float max_wheel_speed
) : wheel_base_(wheel_base),
    wheel_radius_(wheel_radius)
{
    setMaxWheelSpeed(max_wheel_speed);
}

void DifferentialDrive::reduceWheelVelocties_(std::pair<float, float> &wheel_velocities)
{
    if (abs(wheel_velocities.first) > max_wheel_speed_)
    {
        float factor = max_wheel_speed_ / abs(wheel_velocities.first);

        wheel_velocities.first *= factor;
        wheel_velocities.second *= factor;
    }

    if (abs(wheel_velocities.second) > max_wheel_speed_)
    {
        float factor = max_wheel_speed_ / abs(wheel_velocities.second);

        wheel_velocities.first *= factor;
        wheel_velocities.second *= factor;
    }
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

    std::pair<float, float> wheel_velocities(
        (linear_velocity - angular_velocity) / wheel_radius_,
        (linear_velocity + angular_velocity) / wheel_radius_
    );

    reduceWheelVelocties_(wheel_velocities);

    return wheel_velocities;
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