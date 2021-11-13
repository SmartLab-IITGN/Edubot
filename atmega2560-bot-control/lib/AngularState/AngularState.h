/**
 * @file AngularState.h
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief This library is intended to calculate the angular velocity of a rotating 
 * shaft using a rotary encoder. It derives from the Encoder class in the Encoder.h 
 * library by Paul Stoffregen at https://www.pjrc.com/teensy/td_libs_Encoder.html
 *  
 * The rotary encoder provides angular position of the shaft. The initial position 
 * of the shaft, when the system boots up, is assumed to be zero. The Encoder type
 * object increments / decrements the position, a 32-bit integer, in the range 
 * -2147483648 to 2147483647, upon receiving a quadrature pulse from the rotary
 * encoder.
 * 
 * This library defines AngularState class. It determines the angular
 * velocity at the current time step by using last 5 encoder position readings and
 * a 5th order scheme to estimate the first derivative with respect to time. 
 * 
 * @version 1.0
 * @date 2021-06-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __ANGULAR_STATE__
#define __ANGULAR_STATE__

// This library contains the definition of Encoder class
#include "Encoder.h"

#if !defined(ESP8266)
    
    // This library is required for blocking interrupts
    // while copying volatile variables in main execution
    // path. Link - https://www.nongnu.org/avr-libc/user-manual/group__util__atomic.html
    #include <util/atomic.h>

#endif

/**
 * @brief Definition of class angularVelocityCalculator.
 * It is derived from Encoder class.
 * 
 * It determines the angular velocity at the current time step
 * by using last 5 encoder position readings and a 5th order scheme
 * to estimate the first derivative with respect to time.
 * 
 * Angular Velocity is given by
 * \f{equation}{
 * \omega =
 * \f} 
 */
class AngularState : public Encoder
{
    private :
        
        /**
         * @brief This array of 5 elements stores last 5 positions
         * of the shaft read from the encoder at a certain time interval
         * 
         * Since the values will be updated at a very high frequency 
         * making the array volatile tells the compiler not to optimize 
         * the reading from array task. Usually a variable that is used 
         * frequently, is stored in a cache close to the processor. 
         * But here our variable is updated more frequently than it is read. 
         * Hence, it needs to be read from memory every time.
         */
        volatile float encoder_readings_array_[5];
        
        /**
         * @brief Time interval after which a new encoder position
         * value is inserted to
         */
        const float time_period_;

        /**
         * @brief Number of counts of change in encoder position to
         * complete one full rotation of the wheel
         */
        const float counts_per_rotation_;

    public :

        /**
         * @brief Construct a new Angular State object
         * 
         * @param encoder_pin_1 Pin no. to which Pin A of encoder is connected
         * @param encoder_pin_2 Pin no. to which Pin B of encoder is connected
         * @param update_frequency Frequency at which new position value is 
         * read from the encoder
         * @param counts_per_rotation No. of quadrature turn counts received 
         * from encoder upon completion of one shaft rotation
         */
        AngularState(
            uint8_t encoder_pin_1,
            uint8_t encoder_pin_2,
            float update_frequency,
            float counts_per_rotation
        );

        /**
         * @brief This function is called at the specified frequency
         * to capture a new position reading of the encoder
         * 
         * When inlined by compiler, it executes under 15 us
         * The post-fix __attribute__((always_inline)) ensures
         * the function is definitely inlined by compiler.
         */
        inline void updateAngularState() __attribute__((always_inline));

        /**
         * @brief Set the value of the variable passed by reference 
         * - angular_velocity to the current estimated angular velocity
         * 
         * When inlined by compiler, it executes under 15 us
         * The post-fix __attribute__((always_inline)) ensures
         * the function is definitely inlined by compiler.
         * 
         * @param angular_velocity Variable to which the value of current angular
         * velocity is set
         */
        inline void getAngularVelocity(float &angular_velocity)  __attribute__((always_inline));

		/**
         * @brief Set the value of the variable passed by reference 
         * - angular_velocity to the current estimated angular velocity
         * 
         * When inlined by compiler, it executes under 15 us
         * The post-fix __attribute__((always_inline)) ensures
         * the function is definitely inlined by compiler.
         * 
         * @param angular_velocity Variable to which the value of current angular
         * velocity is set
         */
        inline void getAngularAcceleration(float &angular_velocity)  __attribute__((always_inline));
};

/*============================================================================================================*/

// The definition of the member functions of this class is done in the header file itself
// since the compiler cannot inline functions that are defined and declared in separate files.
// Inlining the functions are absolutely necessary as funciton call overheads cost up to 100 us (ATmega2560)
// whereas executing the tasks inside the functions take up to 10 us

// Read the shaft position from the encoder
// and place it at the front of the encoder readings array
// while shifting other elements one place back
void AngularState::updateAngularState()
{
    // memmove(&encoderReadingsArray[1], &encoderReadingsArray[0], 4*sizeof(float));
    // As encoder_readings_array_ is declared as volatile,
    // memmove function cannot be used
    encoder_readings_array_[4] = encoder_readings_array_[3];
    encoder_readings_array_[3] = encoder_readings_array_[2];
    encoder_readings_array_[2] = encoder_readings_array_[1];
    encoder_readings_array_[1] = encoder_readings_array_[0];
    encoder_readings_array_[0] = (float) read();
}

// Calculate and set the argument passed by reference, angular velocity, from  
// five encoder readings using a fifth order scheme
void AngularState::getAngularVelocity(float &angular_velocity)
{
    #if !defined(ESP8266)
    
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            // Atomic block prevents any interrupts from
            // stopping the initializing task
    
    #endif
    
            // Estimate angular velocity using 5th order scheme
            angular_velocity = 2 * PI * (
                25.0f * encoder_readings_array_[0] -
                48.0f * encoder_readings_array_[1] +
                36.0f * encoder_readings_array_[2] -
                16.0f * encoder_readings_array_[3] +
                3.0f  * encoder_readings_array_[4]
            ) / (12.0f * time_period_ * counts_per_rotation_);

    #if !defined(ESP8266)
            
            // Atomic restorestate will restore the status
            // of interrupts to whatever it was before it stopped
            // all interrupts, i.e. enabled / disabled.
        }
        
    #endif
}

// Calculate and set the argument passed by reference, angular acceleration, from  
// five encoder readings using a fifth order scheme
void AngularState::getAngularAcceleration(float &angular_acceleration)
{
    #if !defined(ESP8266)
    
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            // Atomic block prevents any interrupts from
            // stopping the initializing task
    
    #endif
    
            // Estimate angular velocity using 5th order scheme
            angular_acceleration = 2 * PI * (
                35.0f	* encoder_readings_array_[0] -
                104.0f	* encoder_readings_array_[1] +
            	114.0f	* encoder_readings_array_[2] -
                56.0f	* encoder_readings_array_[3] +
                11.0f	* encoder_readings_array_[4]
            ) / (12.0f * time_period_ * time_period_ * counts_per_rotation_);

    #if !defined(ESP8266)
            
            // Atomic restorestate will restore the status
            // of interrupts to whatever it was before it stopped
            // all interrupts, i.e. enabled / disabled.
        }
        
    #endif
}

#endif