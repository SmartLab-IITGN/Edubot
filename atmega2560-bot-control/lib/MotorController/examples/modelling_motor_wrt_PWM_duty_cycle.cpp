/**
 * @file main.cpp
 * @author Souritra Garai (souritra.garai@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-22
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <Arduino.h>

#include "AngularState.h"
#include "PhaseCorrect16BitPWM.h"

// This pin is attached to direction control
// input of motor driver
#define DIRECTION_PIN 23
// Encoder outputs attached to these
// interrupt pins
#define ENCODER_PIN_A 21
#define ENCODER_PIN_B 19

// Frequency at which control loop will be run
#define CONTROL_ROUTINE_CALL_FREQUENCY 50 // Hz

// Number of quadrature counts to complete
// one full rotation of the shaft
#define ENCODER_COUNTS_PER_ROTATION 840

// Time interval after which things are
// printed to serial monitor
#define SERIAL_PRINT_TIME_PERIOD 5E5 // us

AngularState motor_shaft(
	ENCODER_PIN_A,
	ENCODER_PIN_B,
	CONTROL_ROUTINE_CALL_FREQUENCY,
	ENCODER_COUNTS_PER_ROTATION
);

// Variable to fetch and store the current time
float current_time;

// Variable to store when the last serial print
// was performed in microsecond
float last_serial_print_time;

float iteration_num;
float target_pwm_duty_cycle;

float angular_velocity[5] = {0, 0, 0, 0, 0};

float getMean(float array[5])
{
	return (array[0] + array[1] + array[2] + array[3] + array[4]) / 5.0;
}

float getStandardDeviation(float array[5])
{
	float mean = getMean(array);

	float mean_square = (
		array[0]*array[0] + 
		array[1]*array[1] + 
		array[2]*array[2] + 
		array[3]*array[3] + 
		array[4]*array[4]
	) / 5.0;

	return sqrt(mean_square - mean * mean);
}

void initializeTimer3();

void setup()
{
	// Initialize the Serial Monitor
	Serial.begin(115200);

	Timer1PhaseCorrectPWM::clearTimerSettings();
	Timer1PhaseCorrectPWM::setupTimer();
	Timer1PhaseCorrectPWM::setupChannelA();

	pinMode(DIRECTION_PIN, OUTPUT);

	initializeTimer3();

	// Initialize global variables	
	current_time = micros();
	last_serial_print_time	= current_time;

	iteration_num = 0;
	target_pwm_duty_cycle = 0;

	// Serial.println("Initialized successfully");
}

void loop()
{
	current_time = micros();

	if (current_time - last_serial_print_time > SERIAL_PRINT_TIME_PERIOD)
	{
		float mean_angular_velocity = getMean(angular_velocity);
		float std_dev_angular_velocity = getStandardDeviation(angular_velocity);

		if (std_dev_angular_velocity < 0.5) 
		{
			// Serial.print("PWM Duty Cycle Output :\tMean : ");
			// Serial.println(target_pwm_duty_cycle);

			// Serial.print("Velocity :\tMean : ");
			// Serial.print(mean_angular_velocity);
			// Serial.print(" rad/s\tStd Dev : ");
			// Serial.print(std_dev_angular_velocity);
			// Serial.println(" rad/s");

			Serial.println(
				String(target_pwm_duty_cycle) + "," +
				String(mean_angular_velocity) + "," +
				String(std_dev_angular_velocity)
			);

			iteration_num += 1;
			target_pwm_duty_cycle = MAX_PWM_DUTY_CYCLE_INPUT * sin(2.0 * PI * iteration_num / 2000.0);
			Timer1PhaseCorrectPWM::setDutyCyclePWMChannelA((int) abs(target_pwm_duty_cycle));
			digitalWrite(DIRECTION_PIN, signbitf(target_pwm_duty_cycle));

			// Serial.print("Next Target : ");
			// Serial.println(target_pwm_duty_cycle);
		}

		last_serial_print_time = current_time;
	}
}

void initializeTimer3()
{
	// stop interrupts
	cli();

	// Clear Timer/Counter Control Resgisters
	TCCR3A &= 0x00;
	TCCR3B &= 0x00;
	TCCR3C &= 0x00;
	// Clear Timer/Counter Register
	TCNT3 &= 0x0000;

	// Set Timer1 to interrupt at 50Hz

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
	TCCR3B |= (0x01 << WGM32);
	// Set Prescaler to 256
	TCCR3B |= (0x01 << CS32);
	// Set compare match register (OCR3A) to 1250-1
	OCR3A = 0x04E1;
	// Enable interrupt upon compare match of OCR3A
	TIMSK3 |= (0x01 << OCIE3A);

	// allow interrupts
	sei();
}

ISR(TIMER3_COMPA_vect)
{
	motor_shaft.updateAngularState();

	angular_velocity[4] = angular_velocity[3];
	angular_velocity[3] = angular_velocity[2];
	angular_velocity[2] = angular_velocity[1];
	angular_velocity[1] = angular_velocity[0];
	motor_shaft.getAngularVelocity(angular_velocity[0]);
}