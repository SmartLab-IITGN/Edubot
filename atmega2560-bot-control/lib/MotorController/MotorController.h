/**
 * @file MotorController.h
 * @author Navneet Kaur (navneet.kaur@iitgn.ac.in)
 * @brief This header defines a class to control DC
 * motors with rotary encoders
 * @version 0.1
 * @date 2021-06-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __MOTOR_CONTROLLER__
#define __MOTOR_CONTROLLER__

// Required for digital Write function
#include <Arduino.h>

// Required for AngularState class
#include "AngularState.h"

// Required for PID class
#include "PID.h"

#include "FeedForward.h"

/**
 * @brief Class to control a DC motor with rotary encoder attached to its
 * shaft
 * 
 * It derives from the classes AngularState and PID
 */
class MotorController : public AngularState, public PID, public FeedForward
{
    private:
        
        /**
         * @brief Arduino Mega pin for direction output
         */
        const uint8_t direction_pin_;

        /**
         * @brief Truth value whether to reverse direction pin logic
         */
        const float reverse_;

        /**
         * @brief Variable for storing angular velocity
         */
        float angular_velocity_;
        
        /**
         * @brief Variable for storing PID controller output
         */
        float PID_output_;

        /**
         * @brief Variable to store truth value, whether to enable
         * PID control for the motor
         */
        bool PID_control_enable_;

		float max_controller_output_;

    public :

        /**
         * @brief Stops the motor and disables PID controller
         */
        void stopMotor();

        /**
         * @brief Enable PID control for the motor
         */
        void enablePIDControl();

        /**
         * @brief Updates the current angular velocity, uses PID
         * to calculate the motor PWM control output and sets the
         * PWM output
         * 
         * The post-fix __attribute__((always_inline)) ensures
         * the function is definitely inlined by compiler.
         */
        inline void spinMotor() __attribute__((always_inline));

        /**
         * @brief Get the latest calculated motor angular velocity
         * 
         * @return float Angular velocity of the motor
         */
        inline float getMotorAngularVelocity()
		{
			// Return the last computed copy of angular velocity
			return reverse_ * angular_velocity_;
		}

        /**
         * @brief Get the latest evaluated PID output
         * 
         * The post-fix __attribute__((always_inline)) ensures
         * the function is definitely inlined by compiler.
         * 
         * @return float PID output
         */
        inline float getControllerOutput() __attribute__((always_inline));

        /**
         * @brief Construct a new Motor Controller object
         * 
         * @param direction_pin Arduino Mega digital pin number for
         * direction output to motor driver
         * @param encoder_pin_1 Arduino Mega (interrupt enabled) pin number
         * to which rotary encoder pin A is connected
         * @param encoder_pin_2 Arduino Mega (interrupt enabled) pin number
         * to which rotary encoder pin B is connected
         * @param update_frequency Frequency at which control loop is 
         * updated
         * @param reverse Truth value whether to reverse direction pin logic
         */
        MotorController(
            uint8_t direction_pin,
            uint8_t encoder_pin_1,
            uint8_t encoder_pin_2,
            float update_frequency,
            float counts_per_rotation,
            bool reverse = false
        );

		inline void setMotorAngularVelocity(float target_angular_velocity)
		{
			setTargetStateValue(reverse_ * target_angular_velocity);
			setTargetStateValue_FF(reverse_ * target_angular_velocity);
		}

		void setMaxControllerOutput(float max_controller_ouput);

		inline float getPIDOutput()
		{
			return PID_output_;
		}
};

/*=====================================================================================================*/

// The inline functions need to be defined and declared in the same file.

// Update and extract the angular velocity and calculate the pid
// control values for the same. Write the PWM value and direction
void MotorController::spinMotor()
{
    // Calculate the angular velocity and store it in the variable angular_velocity_
    getAngularVelocity(angular_velocity_);

    // If PID control is enabled
    if (PID_control_enable_)
    {
        // Calculate the PID output and round it off to closest integer (for PWM output)
        PID_output_ =  getPIDControllerOutput(angular_velocity_);
        
        // Write a HIGH or LOW value to the direction pin on the motor driver 
        // depending on the sign of PID output
        digitalWrite(direction_pin_, signbit(PID_output_ + getFeedForwardControllerOutput()));
    }
}

float MotorController::getControllerOutput()
{
    // Set the duty cycle of PWM ouput to motor driver to the absolute value
    // of PID output. The value needs to be within 0xFFFF = (65535)_{10}
	return min(abs(round(PID_output_ + getFeedForwardControllerOutput())), max_controller_output_);
}

#endif