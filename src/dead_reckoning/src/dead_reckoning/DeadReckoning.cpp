#include "dead_reckoning/DeadReckoning.h"

#include <math.h>

Eigen::Matrix<float, 3, 2> gradqp(Eigen::Vector3f p)
{
    Eigen::Matrix<float, 3, 2> J;

    float theta = p(2);
    
    J   <<  cos(theta),  0,
            sin(theta),  0,
                     0,  1;
    
    return J;
}

Eigen::Matrix<float, 3, 3> gradppdot(Eigen::Vector3f p, Eigen::Vector2f q)
{
    Eigen::Matrix<float, 3, 3> J;

    float theta = p(2);

    float v = q(0);
    float w = q(1);

    J   <<  0,  0,  - v * sin(theta),
            0,  0,    v * cos(theta),
            0,  0,                 0;

    return J;
}

DeadReckoning::DeadReckoning(
    float x,
    float y,
    float theta,
    float x_dot,
    float y_dot,
    float theta_dot
) {
    setPosture(x, y, theta);
    setPostureTimeDerivative(x_dot, y_dot, theta_dot);
}

void DeadReckoning::setPosture(
    float x,
    float y,
    float theta
) {
    p_ << x, y, theta;
}

void DeadReckoning::setPostureTimeDerivative(
    float x_dot,
    float y_dot,
    float theta_dot
) {
    p_dot_ << x_dot, y_dot, theta_dot;
}

void DeadReckoning::explicitUpdate(
    Eigen::Vector2f q,
    float Dt
) {
    Eigen::Vector3f p_dot = gradqp(p_) * q;
    
    p_ += 0.5 * (p_dot_ + p_dot) * Dt;
    
    p_dot_ = p_dot;
}

void DeadReckoning::implicitUpdate(
    Eigen::Vector2f q,
    float Dt
) {
    Eigen::Vector3f b, p_dot;

    p_dot = gradqp(p_) * q;

    Eigen::Matrix<float, 3, 3> A = Eigen::Matrix3f::Identity() - gradppdot(p_, q) * Dt;

    b = A * p_ + p_dot * Dt;

    p_ = A.householderQr().solve(b);

    p_dot_ = p_dot;
}

Eigen::Vector3f DeadReckoning::getPosture()
{
    return Eigen::Vector3f(p_);
}

Eigen::Vector3f DeadReckoning::getPostureTimeDerivative()
{
    return Eigen::Vector3f(p_dot_);
}