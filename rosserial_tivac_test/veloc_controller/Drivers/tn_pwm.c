/*
 * tn_pwm.c
 *
 *  Created on: Mar 19, 2020
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
#include "driverlib/rom.h"
#include "driverlib/pwm.h"
#include "tn_pwm.h"

//*****************************************************************************
//
// Initialize the PWM Module before using. PWM Module is configured to which
// Module and mode are selected.
//
//*****************************************************************************
void PWMInit(void)
{
    //
    // Configure the rate of the clock provided to PWM module as
    // a ratio of processor clock. The parameter must be of:
    // \b SYSCTL_PWMDIV_1, \b SYSCTL_PWMDIV_2, \b SYSCTL_PWMDIV_4,
    // \b SYSCTL_PWMDIV_8, \b SYSCTL_PWMDIV_16, \b SYSCTL_PWMDIV_32, or
    // \b SYSCTL_PWMDIV_64.
    //
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //
    // Enable the PWM peripheral.
    //
    ROM_SysCtlPeripheralEnable(PWM_PERIPH);

    //
    // Enable the GPIO used by PWM.
    //
    ROM_SysCtlPeripheralEnable(PWM_GPIO_PERIPH);

    //
    // Configure the GPIO pin muxing to select M0PWM0 function for these pin.
    //
    ROM_GPIOPinConfigure(PWM_PIN0_CONFIG);
    ROM_GPIOPinConfigure(PWM_PIN1_CONFIG);

    //
    // Configure PWM function for this pin.
    //
    ROM_GPIOPinTypePWM(PWM_GPIO_BASE, PWM_PIN_0 | PWM_PIN_1);

    //
    // Configure the PWM's mode.
    //
    ROM_PWMGenConfigure(PWM_BASE, PWM_GEN, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
}

//*****************************************************************************
//
// This function configure the frequency of PWM signal.
// The maximum value passed to ui32Period parameter of PWMGenPeriodSet();
// function is 131000.
// This function used for configuring for both M0PWM0 and M0PWM1.
//
// \return True or False
//
//*****************************************************************************
bool PWMFreqConfig(uint32_t ui32Freq)
{
    uint32_t ui32Period;

    ui32Period = ROM_SysCtlClockGet() / ui32Freq;

    if (ui32Period > 131000)
        return false;
    else
    {
        ROM_PWMGenPeriodSet(PWM_BASE, PWM_GEN, ui32Period);
        return true;
    }
}

//*****************************************************************************
//
// This function configure the Duty circle of PWM signal. (%)
//
// \return True or False
//
//*****************************************************************************
bool PWMDutyConfig(pwmindex_t PWMIndex, uint8_t ui8Duty)
{
    if(ui8Duty > 100)
        return false;
    else
    {
        uint32_t ui32PulseWidth;

        ui32PulseWidth = ROM_PWMGenPeriodGet(PWM_BASE, PWM_GEN) * ui8Duty / 100;

        switch (PWMIndex)
        {
        case PWM0:
            ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, ui32PulseWidth);
            break;
        case PWM1:
            ROM_PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ui32PulseWidth);
            break;
        }

        return true;
    }
}

//*****************************************************************************
//
// This function start the PWM Generator Block at pin configured.
//
//*****************************************************************************
void PWMStart(pwmindex_t PWMIndex)
{
    //
    // Enable the Output Signal.
    //
    switch (PWMIndex)
    {
    case PWM0:
        ROM_PWMOutputState(PWM_BASE, PWM_OUT_0_BIT, true);
        break;
    case PWM1:
        ROM_PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, true);
        break;
    }

    //
    // Enable the PWM generator block.
    //
    ROM_PWMGenEnable(PWM_BASE, PWM_GEN);
}

//*****************************************************************************
//
// This function stop the PWM Generator Block at pin configured.
//
//*****************************************************************************
void PWMStop(pwmindex_t PWMIndex)
{
    //
    // Disable the PWM generator block.
    //
    ROM_PWMGenDisable(PWM_BASE, PWM_GEN);

    //
    // Disable the Output Signal.
    //
    switch (PWMIndex)
    {
    case PWM0:
        ROM_PWMOutputState(PWM_BASE, PWM_OUT_0_BIT, false);
        break;
    case PWM1:
        ROM_PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, false);
        break;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************















