/**
 * @file angular_state_update_timer.cpp
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief This is an example code to estimate angular velocity 
 * of a rotating shaft with an encoder attached to it,
 * using the AngularState class.
 * 
 * This example is specific to Arduino Uno (AVR128) 
 * and Arduino Mega (ATmega2560) boards. The angular velocity 
 * is updated at frequency of 50 Hz, using Timer 1 interrupt.
 * 
 * @version 1.0
 * @date 2021-06-30
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "AngularState.h"

// Time interval after which anything is
// printed to serial monitor
#define SERIAL_PRINT_TIME_PERIOD 1E6 // us

// Encoder outputs attached to these
// interrupt pins on the Arduino Mega board
#define ENCODER_PIN_A 21
#define ENCODER_PIN_B 19

// Frequency at which velocity will be updated
#define VELOCITY_UPDATE_FREQUENCY 50 // Hz

// Number of quadrature counts to complete
// one full rotation of the shaft
#define ENCODER_COUNTS_PER_ROTATION 840

// Our object to estimate the angular velocity of encoder shaft
AngularState encoder_shaft(
	ENCODER_PIN_A,
	ENCODER_PIN_B,
	VELOCITY_UPDATE_FREQUENCY,
	ENCODER_COUNTS_PER_ROTATION
);

// Variable to fetch and store the current time
float current_time;

// Variable to store when the last serial print
// was performed in microsecond
float last_serial_print_time;

// To verify the time period of angular state update 

// Variable to fetch and store the current time
float current_angular_state_update_time;

// Variable to store when the last angular state
// update was performed in microseconds
float last_angular_state_update_time;

// Variable to store the sum of durations in 
// microseconds after which angular state is updated
float angular_state_update_interval_sum;

// Variable to store the number of velocity updates 
// that are performed
float number_angular_state_updates;

/**
 * @brief Declaration of function for initializing Timer 2
 * interrupts
 */
void initializeTimer1();

void setup()
{
	Serial.begin(115200);

	// Initialize global variables

	current_time = micros();
	last_serial_print_time = current_time;

	current_angular_state_update_time 	= 0;
	last_angular_state_update_time		= current_time;

	angular_state_update_interval_sum 	= 0;
	number_angular_state_updates		= 0;

	// Initialize timer 1 for interrupts
	initializeTimer1();

	Serial.println("Arduino Initialized successfully");
}

void loop()
{
	current_time = micros();

	if (current_time - last_serial_print_time > SERIAL_PRINT_TIME_PERIOD)
	{
		float angular_velocity;
		encoder_shaft.getAngularVelocity(angular_velocity);
		Serial.print("Velocity:\t");
		Serial.println(angular_velocity, 5);

		Serial.print("Encoder shaft position:\t");
		Serial.println(encoder_shaft.read());

		Serial.print("Average Angular State Update Period:\t");
		Serial.println(angular_state_update_interval_sum / number_angular_state_updates);

		last_serial_print_time = current_time;
	}
}


void initializeTimer1()
{
	// stop interrupts
	cli();

	// Clear Timer/Counter Control Resgisters
	TCCR1A &= 0x00;
	TCCR1B &= 0x00;
	TCCR1C &= 0x00;
	// Clear Timer/Counter Register
	TCNT1 &= 0x0000;

	// Set Timer1 to interrupt at 8kHz

	// ATmega2560 clock frequency - 16MHz
	// Prescalers available for timers - 1, 8, 64, 256, 1024
	// Timer clock frequency = ATmega2560 clock frequency / Prescaler

	// To have interrupts with better frequency resolution timer
	// will be operated in Clear Timer on Compare Match (CTC) mode
	// TCNTx will count from 0x00 to the value in OCRnA register
	// Frequency of Interrupt = Timer clock frequency / Number of TCNTn Counts before it resets to 0x00

	// TCNTx will be compared to OCRnx in each clock cycle
	// Upon compare match, interrupt is fired
	// For 50Hz, TCNTx needs - 
	//   - 40000 counts at 8 prescaler
	//   - 5000 counts at 64 prescaler
	//   - 1250 counts at 256 prescaler
	//   - 312.5 counts at 1024 prescaler

	// Turn on CTC mode
	TCCR1B |= (0x01 << WGM12);
	// Set Prescaler to 256
	TCCR1B |= (0x01 << CS12);
	// Set compare match register (OCR2A) to 249
	OCR1A = 0x04E1;
	// Enable interrupt upon compare match of OCR2A
	TIMSK1 |= (0x01 << OCIE1A);

	// allow interrupts
	sei();
}

ISR(TIMER1_COMPA_vect)
{
	current_angular_state_update_time = micros();

	encoder_shaft.updateAngularState();

	angular_state_update_interval_sum	+= current_angular_state_update_time - last_angular_state_update_time;
	number_angular_state_updates		+= 1;

	last_angular_state_update_time = current_angular_state_update_time;
}