#include <Arduino.h>

#include <ESP8266WiFi.h>

#include "MotorController.h"

const char* ssid     = "IITGN-GUEST";

const char* host = "10.1.149.213";
const uint16_t port = 3000;

// Use WiFiClient class to create TCP connections
WiFiClient client;

// This pin is attached to PWM input of
// motor driver to control speed
#define PWM_PIN D2
// This pin is attached to direction control
// input of motor driver
#define DIRECTION_PIN D4
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

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid);

	Serial.println();
	Serial.println();
	Serial.print("Wait for WiFi... ");

	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(500);
	}

	Serial.println("");
	Serial.println("WiFi connected");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());

	delay(500);
	
	// The following functions are not available
	// in Arduino Uno or Mega chips, comment them
	// PWM Frequency input to motor driver
	// Should be between 5 ~ 15 kHz
	analogWriteFreq(10E3);
	// Bit-Resolution for representing PWM Duty cycle
	analogWriteResolution(15);
	// Set max controller output according to bit resolution of PWM Duty cycle
	motor_controller.setMaxControllerOutput((1 << 15) - 1);

	motor_controller.set_polynomial_coefficients_positive_value_positive_acceleration(7548, 3739, -780, 93.8, -3.58);
	motor_controller.set_polynomial_coefficients_positive_value_negative_acceleration(10664, 977, -71, 23.8, -1.22);
	motor_controller.set_polynomial_coefficients_negative_value_negative_acceleration(-6889, 4173, 977, 119, 4.52);
	motor_controller.set_polynomial_coefficients_negative_value_positive_acceleration(-10622, 1025, 144, 34.5, 1.62);

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

	Serial.print("connecting to ");
	Serial.print(host);
	Serial.print(':');
	Serial.println(port);

	while (!client.connect(host, port))
	{
		Serial.print(".");
		delay(500);
	}

	Serial.println("Connected to Socket Server");
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

			float mean_pwm_duty_cycle = getMean(pwm_duty_cycle);
			float std_dev_pwm_duty_cycle = getStandardDeviation(pwm_duty_cycle);

			Serial.print("PWM Duty Cycle Output :\tMean : ");
			Serial.print(mean_pwm_duty_cycle);
			Serial.print("\tStd Dev : ");
			Serial.println(std_dev_pwm_duty_cycle);

			client.printf(
				"%f, %f, %f, %f\n",
				mean_angular_velocity,
				std_dev_angular_velocity,
				mean_pwm_duty_cycle,
				std_dev_pwm_duty_cycle
			);

			iteration_num += 1;
			target_angular_velocity = 12.0 * sin(2.0 * PI * iteration_num / 40.0);
			motor_controller.setMotorAngularVelocity(target_angular_velocity);
		}

		last_serial_print_time = current_time;
	}
}