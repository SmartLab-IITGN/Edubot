#ifndef __DEAD_RECKONING__
#define __DEAD_RECKONING__

#include <eigen3/Eigen/Dense>

class DeadReckoning
{
    private:
    
        Eigen::Vector3f p_;
        Eigen::Vector3f p_dot_;

    public:

        DeadReckoning(
            float x = 0,
            float y = 0,
            float theta = 0,
            float x_dot = 0,
            float y_dot = 0,
            float theta_dot = 0
        );

        Eigen::Vector3f getPosture();
        Eigen::Vector3f getPostureTimeDerivative();

        void setPosture(float x, float y, float theta);
        void setPostureTimeDerivative(float x_dot, float y_dot, float theta_dot);

        void implicitUpdate(Eigen::Vector2f q, float delta_t);
        void explicitUpdate(Eigen::Vector2f q, float delta_t);
};

#endif