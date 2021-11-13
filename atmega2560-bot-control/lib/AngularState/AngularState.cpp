/**
 * @file AngularState.cpp
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief Implementation of non inline methods of AngularState class
 * @version 1.0
 * @date 2021-09-28
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "AngularState.h"

// Constructor
AngularState::AngularState(
    uint8_t encoder_pin_1,
    uint8_t encoder_pin_2,
    float update_frequency,
    float counts_per_rotation
) : // Initialize the base Encoder class
    // and constant variables in the mem initialization list
    Encoder(encoder_pin_1, encoder_pin_2),
    time_period_(1.0f / update_frequency),
    counts_per_rotation_(counts_per_rotation)
{
	#if !defined(ESP8266)
    
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            // Atomic block prevents any interrupts from
            // stopping the initializing task
    
	#endif
    
	encoder_readings_array_[0] = 0.0f;
    encoder_readings_array_[1] = 0.0f;
    encoder_readings_array_[2] = 0.0f;
    encoder_readings_array_[3] = 0.0f;
    encoder_readings_array_[4] = 0.0f;

	#if !defined(ESP8266)
            
            // Atomic restorestate will restore the status
            // of interrupts to whatever it was before it stopped
            // all interrupts, i.e. enabled / disabled.
        }
        
    #endif
}