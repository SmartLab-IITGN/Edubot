// Definitions of member functions for the class PID

#include "PID.h"

// Constructor
PID::PID(
    float update_frequency
) : // Initialize the constant variable time_period_
    // in the mem initialization list
    time_period_(1 / update_frequency)
{
    // Initialize the gains to zero
    proportional_gain_	= 0;
    integral_gain_		= 0;
    differential_gain_	= 0;

    // Initialize the value of the state variable
    // to zero
    target_state_value_ = 0;
    last_state_value_	= 0;

    // Initialize the variables last error and
    // integral error to zero
    reset();
}

// Reset error and integral error to zero
void PID::reset()
{
    target_state_value_ = 0;
    last_error_			= 0;
    integral_error_		= 0;
}

// Set the value of internal variables for
// PID gains to the requested values
void PID::setPIDGains(
    float proportional_gain,
    float integral_gain,
    float differential_gain
)
{
    proportional_gain_	= proportional_gain;
    integral_gain_		= integral_gain;
    differential_gain_ 	= differential_gain;
}


