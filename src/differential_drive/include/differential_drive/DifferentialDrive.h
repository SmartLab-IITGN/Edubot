#ifndef __DIFFERENTIAL_DRIVE__
#define __DIFFERENTIAL_DRIVE__

#include <vector>

class DifferentialDrive
{
    private:
        
        const float wheel_base_;
        const float wheel_radius_;

        float max_wheel_speed_;

        float getClippedVeloctiy_(float velocity);

    public:

        DifferentialDrive(
            float wheel_base,
            float wheel_radius,
            float max_wheel_speed = 1.0
        );

        void setMaxWheelSpeed(float max_wheel_speed);

        std::pair<float, float> getWheelVelocity(
            float linear_velocity,
            float angular_velocity
        );

        std::pair<float, float> getBotVelocity(
            float left_wheel_velocity,
            float right_wheel_velocity
        );
};

#endif