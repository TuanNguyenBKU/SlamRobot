/*
 * tn_timer.c
 *
 *  Created on: Mar 18, 2020
 *      Author: Tuan Nguyen
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"
#include "tn_timer.h"

//*****************************************************************************
//
// Initialize the Timer before using. Timer is configured to which timer and
// mode is selected.
//
//*****************************************************************************
void TimerInit()
{
    SysCtlPeripheralEnable(TIMER_PERIPH);
    TimerConfigure(TIMER_BASE, TIMER_MODE);
}

//*****************************************************************************
//
// This function provide a delay. (t: ms)
//
//*****************************************************************************
void DelayMs(uint32_t ui32TimeMs)
{
    uint32_t ui32Value = 0;
    uint32_t ui32CurrentValue = 0;

    TimerDisable(TIMER_BASE, TIMER_A);

    ui32Value = (SysCtlClockGet()/1000) * ui32TimeMs - 1;

    TimerLoadSet(TIMER_BASE, TIMER_A, ui32Value);
    TimerEnable(TIMER_BASE, TIMER_A);

    do
    {
        ui32CurrentValue = TimerValueGet(TIMER0_BASE, TIMER_A);
    }
    while(ui32CurrentValue != ui32Value);

    TimerDisable(TIMER_BASE, TIMER_A);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
