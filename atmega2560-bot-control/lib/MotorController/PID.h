/**
 * @file PID.h
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief This header defines a class to implement 
 * Proportional Integral Differential controller.
 * @version 0.1
 * @date 2021-07-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __PID__
#define __PID__

/**
 * @brief Class to implement PID control
 * 
 * It is assumed that the control ouput is calculated at
 * a fixed frequency
 */
class PID
{
    private:

        /**
         * @brief Proportional gain for PID control
         */
        float proportional_gain_;
        
        /**
         * @brief Integral gain for PID control
         */
        float integral_gain_;

        /**
         * @brief Differential gain for PID control
         */
        float differential_gain_;
        
        /**
         * @brief Variable for storing error in the last itereation
         */
        float last_error_;
        /**
         * @brief Variable for storing integral of error
         */
        float integral_error_;
        
        /**
         * @brief Variable for storing the set point value
         */
        float target_state_value_;

        /**
         * @brief Variable for storing the value of the state variable
         * in the last iteration
         * 
         * Value of the state variable in the last iteration is required 
         * to determine the derivative for differential control
         */
        float last_state_value_;

        /**
         * @brief Time period in which PID control is executed once
         */
        const float time_period_;

    public:

        /**
         * @brief Construct a new PID object
         * 
         * @param update_frequency Frequency at which PID control is excuted
         */
        PID(float update_frequency);

        /**
         * @brief Function to modify PID gains for the controller
         * 
         * @param proportional_gain gain for proportional control
         * @param integral_gain gain for integral control
         * @param differential_gain gain for differential control
         */
        void setPIDGains(
            float proportional_gain,
            float integral_gain,
            float differential_gain
        );

        /**
         * @brief Get the Controller Output
         * 
         * The post-fix __attribute__((always_inline)) ensures
         * the function is definitely inlined by compiler.
         * 
         * @param current_state_value current value of the state variable
         * @return float PID control output
         */
        inline float getPIDControllerOutput(float current_state_value) __attribute__((always_inline));

        /**
         * @brief Get the error calculated in the last iteration
         * 
         * @return float Error calculated in the last iteration
         */
        inline float getError()
        {
            return last_error_;
        }

        /**
         * @brief Set the target state value
         * 
         * @param target_state_value Desired target state
         */
        inline void setTargetStateValue(float target_state_value) __attribute__((always_inline))
        {
            target_state_value_ = target_state_value;
        }

        /**
         * @brief Reset last stored error and integral error to zero
         */
        void reset();
};

/*=====================================================================================================*/

// The inline functions need to be defined and declared in the same file.

// Evaluate and return the PID control output
// Also update the integral error, last error and last state value
float PID::getPIDControllerOutput(float current_state_value)
{
    // Evaluate the error at the current iteration
    float current_error = target_state_value_ - current_state_value;
    
    // Evaluate the integral of error from time t = 0
    // using trapezoidal rule
    integral_error_ += 0.5 * (last_error_ + current_error) * time_period_;

    // Evaluate the controller output by multiplying
    // gains with corresponding errors
    float controller_output = 
        proportional_gain_  * current_error +
        integral_gain_      * integral_error_ +
        differential_gain_  * (last_state_value_ - current_state_value) / time_period_;
    // derivative for differential control is determined
    // by taking backward difference of the state variable

    // Update last value of state variable and error
    last_state_value_ = current_state_value;
    last_error_ = current_error;

    // return controller output
    return controller_output;
}

#endif