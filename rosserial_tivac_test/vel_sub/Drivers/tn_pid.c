/*
 * tn_pid.c
 *
 *  Created on: Jun 14, 2020
 *      Author: Tuan Nguyen
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#define TARGET_IS_TM4C123_RA1

#include "tn_pid.h"

//*****************************************************************************
//
// This function calculate for the PID Controller, using for controlling
// Velocity.
// i32CurrentVel: RPM
// i32SetVel
// ui16DeltaTime: ms
// ResFlag: RES or NONRES
//  |-> Accumulated variables need reseting when setting new Velocity.
//  |-> Set ResFlag = RES, don't care about the others arguments.
//
//*****************************************************************************
int32_t CalculatePID(int32_t i32CurrentVel, int32_t i32SetVel,
                     float* floIn, int32_t* i32PreError,
                     uint16_t ui16SampTime)
{
    int32_t i32CurrentError = 0;

    float floPn = 0;

    float floDn = 0;

    int32_t i32PWMDuty = 0;

    //
    // Calculate the current Error.
    //
    i32CurrentError = i32SetVel - i32CurrentVel;

    //
    // Calculate the P part.
    //
    floPn = Kp * (float)i32CurrentError;

    //
    // Calculate the I part.
    //
    *floIn += Ki * ((float)ui16SampTime / 1000) * (float)i32CurrentError;

    //
    // Calculate the D part.
    //
    floDn = Kd * ((float)i32CurrentError - (float)(*i32PreError)) * 1000 / (float)ui16SampTime;
    *i32PreError = i32CurrentError;

    //
    // Calculate the output: PWM Duty.
    //
    i32PWMDuty = floPn + *floIn + floDn;

    //
    // Limit the range of PWM value.
    //
    if (i32PWMDuty > 0)
    {
        if (i32PWMDuty > PWM_MAX)
        {
            i32PWMDuty = PWM_MAX;
        }
        else if (i32PWMDuty < PWM_MIN)
        {
            i32PWMDuty = PWM_MIN;
        }
    }
    else if (i32PWMDuty < 0)
        {
            if (i32PWMDuty < -PWM_MAX)
        {
            i32PWMDuty = -PWM_MAX;
        }
            else if (i32PWMDuty > -PWM_MIN)
        {
            i32PWMDuty = -PWM_MIN;
         }
    }
    return i32PWMDuty;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
