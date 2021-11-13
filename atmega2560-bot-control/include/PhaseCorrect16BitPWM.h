/**
 * @file PhaseCorrect16bitPWM.h
 * @author Navneet Kaur (navneet.kaur@iitgn.ac.in), Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief This library defines a functions to set up 16 bit Phase Correct PWM
 * signals on Output Compare Pins of Timer1 in ATmega2560
 * @version 0.1
 * @date 2021-07-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __PHASE_CORRECT_16_BIT_PWM__
#define __PHASE_CORRECT_16_BIT_PWM__

#include "Arduino.h"

#define MAX_PWM_DUTY_CYCLE_INPUT    0xFFFF

namespace Timer1PhaseCorrectPWM
{
    /**
     * @brief Set the Duty Cycle of PWM Channel A of Timer 1
     * 
     * @param dutyCycle 16 bit integer from 0 to 0xFFFF = \f$ (65535)_{10} \f$
     */
    inline void setDutyCyclePWMChannelA(uint16_t dutyCycle) __attribute__((always_inline));

    /**
     * @brief Set the Duty Cycle of PWM Channel B of Timer 1
     * 
     * @param dutyCycle 16 bit integer from 0 to 0xFFFF = \f$ (65535)_{10} \f$
     */
    inline void setDutyCyclePWMChannelB(uint16_t dutyCycle) __attribute__((always_inline));

    /**
     * @brief Set the Duty Cycle of PWM Channel C of Timer 1
     * 
     * @param dutyCycle 16 bit integer from 0 to 0xFFFF = \f$ (65535)_{10} \f$
     */
    inline void setDutyCyclePWMChannelC(uint16_t dutyCycle) __attribute__((always_inline));

    /**
     * @brief Clears the control settings for Timer 1
     */
    void clearTimerSettings();

    /**
     * @brief Sets up Timer 1 for Phase Correct PWM mode with frequency of 15.625 kHz
     */
    void setupTimer();

    /**
     * @brief Sets up channel A of Timer 1 for PWM signal
     * 
     * PWM signal will be observed on OC1A pin (Pin 11 on Arduino Mega)
     */
    void setupChannelA();

    /**
     * @brief Sets up channel B of Timer 1 for PWM signal
     * 
     * PWM signal will be observed on OC1B pin (Pin 12 on Arduino Mega)
     */
    void setupChannelB();

    /**
     * @brief Sets up channel C of Timer 1 for PWM signal
     * 
     * PWM signal will be observed on OC1C pin (Pin 13 on Arduino Mega)
     */
    void setupChannelC();
}

/*============================================================================================================*/

// The definition of the member functions of this class is done in the header file itself
// since compiler could not inline functions that are defined and declared in separate files.

void Timer1PhaseCorrectPWM::setDutyCyclePWMChannelA(uint16_t dutyCycle)
{
    // Set the value of compare match register for channel A to val
    OCR1A = dutyCycle;
}

void Timer1PhaseCorrectPWM::setDutyCyclePWMChannelB(uint16_t dutyCycle)
{
    // Set the value of compare match register for channel A to val
    OCR1B = dutyCycle;
}

void Timer1PhaseCorrectPWM::setDutyCyclePWMChannelC(uint16_t dutyCycle)
{
    // Set the value of compare match register for channel A to val
    OCR1C = dutyCycle;
}

#endif