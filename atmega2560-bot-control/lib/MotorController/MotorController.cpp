#include "MotorController.h"

// Constructor
MotorController::MotorController(
    uint8_t direction_pin,
    uint8_t encoder_pin_1,
    uint8_t encoder_pin_2,
    float update_frequency,
    float counts_per_rotation,
    bool reverse
) : // Call base class constructors
    AngularState(
        encoder_pin_1,
        encoder_pin_2,
        update_frequency,
        counts_per_rotation
    ), PID(update_frequency),
    // Initialize const variables
    direction_pin_(direction_pin),
    reverse_(reverse ? -1.0 : 1.0)
{
    // Initialize internal variables
	max_controller_output_ = 255; // By default most boards support 8 bit PWM
    angular_velocity_ = 0;
	
	PID_output_ = 0;

    // Initialise the data direction for direction pin to
    // output
    pinMode(direction_pin_, OUTPUT);

    // Initialise the motors in stopping position
    stopMotor();
}

void MotorController::stopMotor()
{
    // Disable PID control
    PID_control_enable_ = false;
    // Set duty cyle for PWM output to 0
    PID_output_ = 0;
}

void MotorController::enablePIDControl()
{
    // Reset PID
    reset();
    // Enable PID
    PID_control_enable_ = true;
}

void MotorController::setMaxControllerOutput(float max_controller_output)
{
	max_controller_output_ = max_controller_output;
}