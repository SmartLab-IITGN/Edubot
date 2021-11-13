/**
 * @file timing_motor_control_methods.cpp
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief This code finds the mean time required for running
 * the important control routines in the MotorController class
 * @version 0.1
 * @date 2021-10-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <Arduino.h>

#include "MotorController.h"

// This pin is attached to PWM input of
// motor driver to control speed
#define PWM_PIN D1
// This pin is attached to direction control
// input of motor driver
#define DIRECTION_PIN D3
// Encoder outputs attached to these
// interrupt pins
#define ENCODER_PIN_A D5
#define ENCODER_PIN_B D6

// Frequency at which control loop will be run
#define CONTROL_ROUTINE_CALL_FREQUENCY 50 // Hz

// Number of quadrature counts to complete
// one full rotation of the shaft
#define ENCODER_COUNTS_PER_ROTATION 4160

// Time interval after which things are
// printed to serial monitor
#define SERIAL_PRINT_TIME_PERIOD 1E6 // us

// Motor controller object to 
// control motor angular velocity
MotorController motor_controller(
  DIRECTION_PIN,
  ENCODER_PIN_A,
  ENCODER_PIN_B,
  CONTROL_ROUTINE_CALL_FREQUENCY,
  ENCODER_COUNTS_PER_ROTATION
);

// Variable to fetch and store the current time
float current_time;

// Variable to store when the last angular state
// update was performed in microseconds
float last_control_routine_call_time;

// Variable to store when the last serial print
// was performed in microsecond
float last_serial_print_time;

// Variable to store the sum of durations in 
// microseconds required to perform spinMotor call
float motor_control_routine_time_sum;
// Variable to store the sum of squares of durations in 
// microseconds required to perform spinMotor call
float motor_contorl_routine_time_square_sum;
// Variable to store the number of spinMotor calls 
// that are performed
float number_motor_control_routine_calls;

// Variable to determine the duration of a function call
// Time when function was called
double start_time;
// Time when returned from function
double duration;

void setup()
{
	// Initialize the Serial Monitor
	Serial.begin(115200);

	// The following functions are not available
	// in Arduino Uno or Mega chips, comment them
	// PWM Frequency input to motor driver
	// Should be between 5 ~ 15 kHz
	analogWriteFreq(10E3);
	// Bit-Resolution for representing PWM Duty CYCLE
	analogWriteResolution(8);
	// Set max controller output according to bit resolution of PWM Duty cycle
	motor_controller.setMaxControllerOutput((1 << 8) - 1);

	// Initialize motor controller
	motor_controller.setPIDGains(3500, 1250, 2);

	motor_controller.enablePIDControl();

	// Initialize global variables	
	number_motor_control_routine_calls = 0;
	motor_control_routine_time_sum = 0;
	motor_contorl_routine_time_square_sum = 0;

	current_time = micros();
	last_control_routine_call_time	= current_time;
	last_serial_print_time			= current_time;
	
	// Set motor controller target to 10 rad/s
	motor_controller.setMotorAngularVelocity(10);

	Serial.println("Initialized successfully");
}

void loop()
{
	current_time = micros();

	if (current_time - last_control_routine_call_time > 1E6/CONTROL_ROUTINE_CALL_FREQUENCY)
	{
		motor_controller.updateAngularState();

		start_time = micros();
		motor_controller.spinMotor();
		duration = micros() - start_time;

		motor_control_routine_time_sum += duration;
		motor_contorl_routine_time_square_sum += duration * duration;
		number_motor_control_routine_calls += 1;

		analogWrite(PWM_PIN, motor_controller.getControllerOutput());

		last_control_routine_call_time = current_time;
	}

	if (current_time - last_serial_print_time > SERIAL_PRINT_TIME_PERIOD)
	{
		float mean_motor_control_routine_duration = 
			motor_control_routine_time_sum /
			number_motor_control_routine_calls;

		float std_dev_motor_control_routine_duration = sqrt(
			motor_contorl_routine_time_square_sum /
			number_motor_control_routine_calls - 
			motor_control_routine_time_sum * motor_control_routine_time_sum
		);

		Serial.print("Time taken for Motor Control Routine :\tMean\t");
		Serial.print(mean_motor_control_routine_duration);
		Serial.print("\tStd Dev\t");
		Serial.println(std_dev_motor_control_routine_duration);

		last_serial_print_time = current_time;
	}
}