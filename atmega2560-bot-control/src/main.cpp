#include <Arduino.h>

#include <ros.h>

#include "MotorController.h"
#include "PhaseCorrect16BitPWM.h"

#include "ROSBotController.h"

#define DIRECTION_PIN_L 22
#define ENCODER_PIN_A_L 20
#define ENCODER_PIN_B_L 18

#define DIRECTION_PIN_R 23
#define ENCODER_PIN_A_R 21
#define ENCODER_PIN_B_R 19

#define VELOCITY_UPDATE_FREQUENCY 50 // Hz

#define PID_Kp  2000
#define PID_Ki  2500
#define PID_Kd  0

#define ENCODER_COUNTS_PER_ROTATION 840

// Time interval at which anything is printed to serial monitor.
#define ROS_CYCLE_TIME_PERIOD 100 // ms

MotorController motor_controller_l(
    DIRECTION_PIN_L,
    ENCODER_PIN_A_L,
    ENCODER_PIN_B_L,
    VELOCITY_UPDATE_FREQUENCY,
    ENCODER_COUNTS_PER_ROTATION
);

MotorController motor_controller_r(
    DIRECTION_PIN_R,
    ENCODER_PIN_A_R,
    ENCODER_PIN_B_R,
    VELOCITY_UPDATE_FREQUENCY,
    ENCODER_COUNTS_PER_ROTATION,
	true
);

ROSBotController bot_controller(
    motor_controller_l,
    motor_controller_r
);

ros::NodeHandle node_handle;

// Variable to store the last time anything was
// printed through the serial port in millisecond.
long int prev_ros_cylce_time;

void initializeTimer3();

void setup()
{
    Timer1PhaseCorrectPWM::clearTimerSettings();
    Timer1PhaseCorrectPWM::setupTimer();

    Timer1PhaseCorrectPWM::setupChannelA(); // Right motor
    Timer1PhaseCorrectPWM::setupChannelB(); // Left motor

    node_handle.initNode();
    node_handle.loginfo("Arduino node initialised");

    motor_controller_l.setMaxControllerOutput(65535);
    motor_controller_r.setMaxControllerOutput(65535);

    motor_controller_l.setPIDGains(PID_Kp, PID_Ki, PID_Kd);
    motor_controller_r.setPIDGains(PID_Kp, PID_Ki, PID_Kd);

    motor_controller_l.set_polynomial_coefficients_positive_value_positive_acceleration(1827, 3844, 0, 0, 0);
	motor_controller_l.set_polynomial_coefficients_positive_value_negative_acceleration(1248, 3894, 0, 0, 0);
	motor_controller_l.set_polynomial_coefficients_negative_value_negative_acceleration(-1540, 4031, 0, 0, 0);
	motor_controller_l.set_polynomial_coefficients_negative_value_positive_acceleration(-1130, 4054, 0, 0, 0);

    motor_controller_r.set_polynomial_coefficients_positive_value_positive_acceleration(1827, 3844, 0, 0, 0);
	motor_controller_r.set_polynomial_coefficients_positive_value_negative_acceleration(1248, 3894, 0, 0, 0);
	motor_controller_r.set_polynomial_coefficients_negative_value_negative_acceleration(-1540, 4031, 0, 0, 0);
	motor_controller_r.set_polynomial_coefficients_negative_value_positive_acceleration(-1130, 4054, 0, 0, 0);

    bot_controller.initialize(node_handle);

    initializeTimer3();

    node_handle.loginfo("Arduino Initialized successfully");
    prev_ros_cylce_time = millis();
}

void loop()
{
    if (millis() - prev_ros_cylce_time > ROS_CYCLE_TIME_PERIOD)
    {
        bot_controller.publish(node_handle);

        node_handle.spinOnce();

        prev_ros_cylce_time = millis();
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
    motor_controller_l.updateAngularState();
    motor_controller_r.updateAngularState();

    motor_controller_l.spinMotor();
    motor_controller_r.spinMotor();

    Timer1PhaseCorrectPWM::setDutyCyclePWMChannelB(motor_controller_l.getControllerOutput());
    Timer1PhaseCorrectPWM::setDutyCyclePWMChannelA(motor_controller_r.getControllerOutput());
}