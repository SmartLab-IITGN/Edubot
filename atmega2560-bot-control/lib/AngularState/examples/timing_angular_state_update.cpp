/**
 * @file timing_angular_state_update.cpp
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief This is an example code to measure the time taken for the
 * updateAngularState and getAngularVelocity class methods of
 * AngularState class to execute.
 * 
 * This example can run on both NodeMCU ESP8266
 * and Arduino AVR / ATmega boards,
 * subject to the availability of interrupt pins.
 * 
 * @version 1.0
 * @date 2021-07-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "AngularState.h"

// Encoder outputs attached to these
// interrupt pins
#define ENCODER_PIN_A D5
#define ENCODER_PIN_B D6

// Time interval after which things are
// printed to serial monitor
#define SERIAL_PRINT_TIME_PERIOD 1E6 // us
// Time interval after which getAngularVelocity
// method of AngularState type onject is called
#define VELOCITY_GET_TIME_PERIOD 1E5 // us

// Frequency at which angular state will be sampled / updated
#define ANGULAR_STATE_UPDATE_FREQUENCY 50 // Hz

// Angular state object to track the angular velocity of encoder shaft
AngularState encoder_shaft(
	ENCODER_PIN_A,
	ENCODER_PIN_B,
	ANGULAR_STATE_UPDATE_FREQUENCY,
	ENCODER_COUNTS_PER_ROTATION
);

// Variable to fetch and store the current angular velocity
float angular_velocity;

// Variable to fetch and store the current time
float current_time;

// Variable to store when the last angular state
// update was performed in microseconds
float last_angular_state_update_time;

// Variable to store when the last serial print
// was performed in microsecond
float last_serial_print_time;

// Variable to store when the last getAngularVelocity method
// of AngularState class was called
long int last_velocity_get_time;

// Variable to store the number of getAngularVelocity calls 
// that are performed
double velocity_get_num_observations;
// Variable to store the sum of durations in 
// microseconds required to perform getAngularVelocity call
double velocity_get_time_period_sum;
// Variable to store the sum of squares of durations in 
// microseconds required to perform getAngularVelocity call
double velocity_get_time_period_sqr_sum;

// Variable to store the number of angular state updates 
// that are performed
double angular_state_update_num_observations;
// Variable to store the sum of durations in 
// microseconds required for performing angular state update
double angular_state_update_time_period_sum;
// Variable to store the sum of squares of durations in 
// microseconds required for performing angular state update
double angular_state_update_time_period_square_sum;

// Variable to determine the duration of a function call
// Time when function was called
double start_time;
// Time when returned from function
double duration;

void setup()
{
	// Initialize Serial Monitor
	Serial.begin(115200);

	// Initialize global variables
	angular_state_update_num_observations		= 0;
	angular_state_update_time_period_sum		= 0;
	angular_state_update_time_period_square_sum = 0;
	
	velocity_get_num_observations		= 0;
	velocity_get_time_period_sum		= 0;
	velocity_get_time_period_sqr_sum	= 0;
	
	current_time = micros();
	last_angular_state_update_time	= current_time;
	last_serial_print_time			= current_time;
	last_velocity_get_time			= current_time;

	Serial.println("Initialized successfully");
}

void loop()
{
	current_time = micros();

	if (current_time - last_angular_state_update_time >= 1E6 / ANGULAR_STATE_UPDATE_FREQUENCY)
	{
		start_time	= micros();
		encoder_shaft.updateAngularState();
		duration	= micros() - start_time;

		angular_state_update_time_period_sum		+= duration;
		angular_state_update_time_period_square_sum += duration * duration;
		angular_state_update_num_observations		+= 1;

		last_angular_state_update_time = current_time;
	}

	if (current_time - last_velocity_get_time >= VELOCITY_GET_TIME_PERIOD)
	{
		start_time	= micros();
		encoder_shaft.getAngularVelocity(angular_velocity);
		duration	= micros() - start_time;

		velocity_get_time_period_sum		+= duration;
		velocity_get_time_period_sqr_sum	+= duration * duration;
		velocity_get_num_observations		+= 1;

		last_velocity_get_time = current_time;
	}
  
	if (current_time - last_serial_print_time >= SERIAL_PRINT_TIME_PERIOD)
	{
		double mean_angular_state_update_time		= angular_state_update_time_period_sum / angular_state_update_num_observations;
		double std_dev_angular_state_update_time 	= sqrt(
			angular_state_update_time_period_square_sum / angular_state_update_num_observations - 
			mean_angular_state_update_time * mean_angular_state_update_time
		);

		double mean_velocity_get_time		= velocity_get_time_period_sum / velocity_get_num_observations;
		double std_dev_velocity_get_time	= sqrt(
			velocity_get_time_period_sqr_sum / velocity_get_num_observations -
			mean_velocity_get_time * mean_velocity_get_time
		);

		Serial.print("\nTime taken for Angular State Update :\tMean\t");
		Serial.print(mean_angular_state_update_time);
		Serial.print("\tStd Dev\t");
		Serial.println(std_dev_angular_state_update_time);

		Serial.print("Time taken for Velocity Get :\t\tMean\t");
		Serial.print(mean_velocity_get_time);
		Serial.print("\tStd Dev\t");
		Serial.println(std_dev_velocity_get_time);    

		last_serial_print_time = current_time;
	}
}