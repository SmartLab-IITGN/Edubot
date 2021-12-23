/**
 * @file motor_example_timer_pwm.cpp
 * @author Navneet Kaur (navneet.kaur@iitgn.ac.in)
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief Example for running PID controls for a target velocity using timer PWM
 * The angular velocity is updated at a rate of 50Hz.  PID output
 * and angular velocity is printed at every 2000 ms rate.
 * @version 0.1
 * @date 2021-06-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "MotorController.h"
#include "PhaseCorrect16BitPWM.h"

#define DIRECTION_PIN 23
#define ENCODER_PIN_A 21
#define ENCODER_PIN_B 19

#define VELOCITY_UPDATE_FREQUENCY 50 // Hz

#define ENCODER_COUNTS_PER_ROTATION 840

// Time interval at which anything is printed to serial monitor.
#define PRINT_TIME_PERIOD 2000 // ms

MotorController motor_controller(
	DIRECTION_PIN,
	ENCODER_PIN_A,
	ENCODER_PIN_B,
	VELOCITY_UPDATE_FREQUENCY,
	ENCODER_COUNTS_PER_ROTATION,
	true
);

// Variable to store the last time anything was
// printed through the serial port in millisecond.
long int last_print_time;

// // Variable to store the last time the angular 
// // velocity was updated in millisecond.
// long int last_vel_update_time;

void initializeTimer3();

void setup()
{
	Serial.begin(115200);

	Timer1PhaseCorrectPWM::clearTimerSettings();
	Timer1PhaseCorrectPWM::setupTimer();
	Timer1PhaseCorrectPWM::setupChannelA();

	// last_vel_update_time = millis();
	last_print_time = millis();

	motor_controller.setMaxControllerOutput(65535);
	motor_controller.setPIDGains(2000, 2500, 0);

	motor_controller.set_polynomial_coefficients_positive_value_positive_acceleration(1827, 3844, 0, 0, 0);
	motor_controller.set_polynomial_coefficients_positive_value_negative_acceleration(1248, 3894, 0, 0, 0);
	motor_controller.set_polynomial_coefficients_negative_value_negative_acceleration(-1540, 4031, 0, 0, 0);
	motor_controller.set_polynomial_coefficients_negative_value_positive_acceleration(-1130, 4054, 0, 0, 0);

	initializeTimer3();

	Serial.println("Arduino Initialized successfully");

	motor_controller.enablePIDControl();

	motor_controller.setMotorAngularVelocity(10);
}

void loop()
{
	// if (millis() - last_vel_update_time > 1000/VELOCITY_UPDATE_FREQUENCY)
	// {
	//   motor_controller.spinMotor();

	//   last_vel_update_time = millis();
	// }

	if (millis() - last_print_time > PRINT_TIME_PERIOD)
	{
		Serial.print("Velocity:\t");
		Serial.println(motor_controller.getMotorAngularVelocity());

		Serial.print("PID output:\t");
		Serial.println(motor_controller.getControllerOutput());

		Serial.print("Error: ");
		Serial.println(motor_controller.getError());

		last_print_time = millis();
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
	motor_controller.updateAngularState();
	motor_controller.spinMotor();
	Timer1PhaseCorrectPWM::setDutyCyclePWMChannelA(motor_controller.getControllerOutput());
}