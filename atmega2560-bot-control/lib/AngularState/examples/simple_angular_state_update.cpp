/**
 * @file simple_angular_state_update.cpp
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief This is an example code to estimate angular velocity 
 * and acceleration of a rotating shaft with an encoder attached 
 * to it, using the AngularState class.
 * 
 * This example can run on both NodeMCU ESP8266
 * and Arduino AVR / ATmega boards,
 * subject to the availability of interrupt pins.
 * 
 * @version 1.0
 * @date 2021-06-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <Arduino.h>

#include "AngularState.h"

// Time interval after which things are
// printed to serial monitor
#define SERIAL_PRINT_TIME_PERIOD 1E6 // us

// Encoder outputs attached to these
// interrupt pins
#define ENCODER_PIN_A D5
#define ENCODER_PIN_B D6

// Frequency at which angular state will be sampled / updated
#define ANGULAR_STATE_UPDATE_FREQUENCY 50 // Hz

// Number of quadrature counts to complete
// one full rotation of the shaft
#define ENCODER_COUNTS_PER_ROTATION 4160

// Angular state object to track the angular velocity of encoder shaft
AngularState encoder_shaft(
	ENCODER_PIN_A,
	ENCODER_PIN_B,
	ANGULAR_STATE_UPDATE_FREQUENCY,
	ENCODER_COUNTS_PER_ROTATION
);

// Variable to fetch and store the current time
float current_time;

// Variable to store when the last angular state
// update was performed in microseconds
float last_angular_state_update_time;

// Variable to store when the last serial print
// was performed in microsecond
float last_serial_print_time;

// To verify the time period of angular state update 

// Variable to store the sum of durations in 
// microseconds after which angular state is updated
float angular_state_update_interval_sum;

// Variable to store the number of angular state updates 
// that are performed
float number_angular_state_updates;

void setup()
{
	// Initialize Serial Monitor
	Serial.begin(115200);

	// Initialize global variables
	angular_state_update_interval_sum	= 0;
	number_angular_state_updates		= 0;
	
	current_time = micros();
	last_angular_state_update_time	= current_time;
	last_serial_print_time		= current_time;

	Serial.println("Initialized successfully");
}

void loop()
{
	current_time = micros();

	if (current_time - last_angular_state_update_time >= 1E6 / ANGULAR_STATE_UPDATE_FREQUENCY)
	{
		encoder_shaft.updateAngularState();

		angular_state_update_interval_sum += current_time - last_angular_state_update_time;
		number_angular_state_updates += 1.0f;

		last_angular_state_update_time = current_time;
	}

	if (current_time - last_serial_print_time >= SERIAL_PRINT_TIME_PERIOD)
	{
		float angular_velocity;
		encoder_shaft.getAngularVelocity(angular_velocity);
		Serial.print("Velocity:\t");
		Serial.println(angular_velocity, 5);

		Serial.print("Encoder shaft position:\t");
		Serial.println(encoder_shaft.read());

		Serial.print("Average angular state update interval:\t");
		Serial.println(angular_state_update_interval_sum / number_angular_state_updates);

		last_serial_print_time = current_time;
	}
}
