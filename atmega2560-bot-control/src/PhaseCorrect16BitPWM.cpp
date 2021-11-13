/**
 * @file PhaseCorrect16BitPWM.cpp
 * @author Souritra Garai (souritra.garai@iitgn.ac.in)
 * @brief Implementation details for Timer1PhaseCorrectPWM 
 * @version 0.1
 * @date 2021-07-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "PhaseCorrect16BitPWM.h"

// Required for Atomic Block function to block interrupts
#include <util/atomic.h>

void Timer1PhaseCorrectPWM::clearTimerSettings()
{
    // Blocks all interrupts and restores after clearing the registers
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Clear the Timer / Counter Control Registers
        TCCR1A &= 0x00;
        TCCR1B &= 0x00;
        TCCR1C &= 0x00;

        // Clear the Timer / Counter Register
        TCNT1 &= 0x0000;
    }
}

void Timer1PhaseCorrectPWM::setupTimer()
{
    // Blocks all interrupts and restores after clearing the registers
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Hotfix - Keep prescaler = 1
        // Don't know why but higher prescalers doesn't work
        // No prescaler
        TCCR1B |= (0x01 << CS10);

        // Setting Wave Generator mode to Phase and Frequency
        // correct PWM with ICR1 register as the TOP value
        TCCR1B |= 0x01 << WGM13;

        // Set the TOP value aka value of ICR1 register to 0xFFFF
        // for 16 bit control
        ICR1 = 0xFFFF;
    }
}

void Timer1PhaseCorrectPWM::setupChannelA()
{
    // Blocks all interrupts and restores after clearing the registers
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Initially set duty cycle to 0
        OCR1A = 0x0000;
        
        // OC1A is attached to Pin 5 of Port B on the ATmega chip
        // Configure Data direction of Port B Pin 5 to Output
        DDRB |= 0x01 << PB5;

        // Clear OC1A on compare match while up-counting and
        // Set OC1A on compare match while down-counting
        TCCR1A |= (0x01 << COM1A1);
    }
}

void Timer1PhaseCorrectPWM::setupChannelB()
{
    // Blocks all interrupts and restores after clearing the registers
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // OC1A is attached to Pin 6 of Port B on the ATmega chip
        // Configure Data direction of Port B Pin 6 to Output
        DDRB |= 0x01 << PB6;

        // Clear OC1B on compare match while up-counting and
        // Set OC1B on compare match while down-counting
        TCCR1A |= 0x01 << COM1B1;
    }
    // Initially set duty cycle to 0
    setDutyCyclePWMChannelB(0);
}

void Timer1PhaseCorrectPWM::setupChannelC()
{
    // Blocks all interrupts and restores after clearing the registers
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // OC1A is attached to Pin 7 of Port B on the ATmega chip
        // Configure Data direction of Port B Pin 7 to Output
        DDRB |= 0x01 << PB7;

        // Clear OC1C on compare match while up-counting and
        // Set OC1C on compare match while down-counting
        TCCR1A |= 0x01 << COM1C1;
    }
    // Initially set duty cycle to 0
    setDutyCyclePWMChannelC(0);
}
