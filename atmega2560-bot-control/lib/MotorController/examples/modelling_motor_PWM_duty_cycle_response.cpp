#include <Arduino.h>

#include "MotorController.h"

// This pin is attached to PWM input of
// motor driver to control speed
#define PWM_PIN D2
// This pin is attached to direction control
// input of motor driver
#define DIRECTION_PIN D4
// Encoder outputs attached to these
// interrupt pins
#define ENCODER_PIN_A D6
#define ENCODER_PIN_B D5

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

float iteration_num;
float target_angular_velocity;

float angular_velocity[5] = {0, 0, 0, 0, 0};
float pwm_duty_cycle[5] = {0, 0, 0, 0, 0};

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

void setup()
{
	// Initialize the Serial Monitor
	Serial.begin(115200);

	// The following functions are not available
	// in Arduino Uno or Mega chips, comment them
	// PWM Frequency input to motor driver
	// Should be between 5 ~ 15 kHz
	analogWriteFreq(10E3);
	// Bit-Resolution for representing PWM Duty cycle
	analogWriteResolution(15);
	// Set max controller output according to bit resolution of PWM Duty cycle
	motor_controller.setMaxControllerOutput((1 << 15) - 1);

	// Initialize motor controller
	motor_controller.setPIDGains(3500, 1250, 2);

	motor_controller.enablePIDControl();

	// Initialize global variables	
	current_time = micros();
	last_control_routine_call_time	= current_time;
	last_serial_print_time			= current_time;

	iteration_num = 0;
	target_angular_velocity = 0;

	Serial.println("Initialized successfully");
}

void loop()
{
	current_time = micros();

	if (current_time - last_control_routine_call_time > 1E6/CONTROL_ROUTINE_CALL_FREQUENCY)
	{
		motor_controller.updateAngularState();
		motor_controller.spinMotor();

		analogWrite(PWM_PIN, motor_controller.getControllerOutput());

		angular_velocity[4] = angular_velocity[3];
		angular_velocity[3] = angular_velocity[2];
		angular_velocity[2] = angular_velocity[1];
		angular_velocity[1] = angular_velocity[0];
		angular_velocity[0] = motor_controller.getMotorAngularVelocity();
		
		pwm_duty_cycle[4] = pwm_duty_cycle[3];
		pwm_duty_cycle[3] = pwm_duty_cycle[2];
		pwm_duty_cycle[2] = pwm_duty_cycle[1];
		pwm_duty_cycle[1] = pwm_duty_cycle[0];
		pwm_duty_cycle[0] = motor_controller.getPIDOutput();

		last_control_routine_call_time = current_time;
	}

	if (current_time - last_serial_print_time > SERIAL_PRINT_TIME_PERIOD)
	{
		float mean_angular_velocity = getMean(angular_velocity);
		float std_dev_angular_velocity = getStandardDeviation(angular_velocity);

		if (	abs(mean_angular_velocity - target_angular_velocity) < 0.1 &&
				std_dev_angular_velocity < 0.1	) 
		{
			Serial.print("Velocity :\tMean : ");
			Serial.print(mean_angular_velocity);
			Serial.print(" rad/s\tStd Dev : ");
			Serial.print(std_dev_angular_velocity);
			Serial.println(" rad/s");

			Serial.print("PWM Duty Cycle Output :\tMean : ");
			Serial.print(getMean(pwm_duty_cycle));
			Serial.print("\tStd Dev : ");
			Serial.println(getStandardDeviation(pwm_duty_cycle));

			iteration_num += 1;
			target_angular_velocity = 16.0 * sin(2.0 * PI * iteration_num / 40.0);
			motor_controller.setMotorAngularVelocity(target_angular_velocity);
		}

		last_serial_print_time = current_time;
	}
}