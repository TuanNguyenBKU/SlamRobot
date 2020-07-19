/*
 * tn_qei.c
 *
 *  Created on: Apr 21, 2020
 *      Author: Tuan Nguyen
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#define TARGET_IS_TM4C123_RA1
#include "driverlib/rom.h"
#include "driverlib/qei.h"

#include "tn_qei.h"

//*****************************************************************************
//
// Initialize the QEI module before using.
// Configure the Velocity capture mode.
// \ui32SamplePeriod: ms
//
//*****************************************************************************
void QEIVelocInit(QEINumber qei, uint32_t ui32SamplePeriod)
{
    //
    // Calculate the Load value for the Timer of QEI0: SAMP_TIME (ms)
    //
    uint32_t ui32QEIPeriod = 0;
    ui32QEIPeriod = ROM_SysCtlClockGet()/1000 * ui32SamplePeriod - 1;

    switch (qei)
    {
    case QEI_0:
        //
        // Enable QEI0 and QEI0's GPIO Peripheral.
        //
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

        //
        // PIN PD7 (PhB0) is muxed with NMI.
        // Firstly, unlock the Commit Register of Port D. Set bit 7 of Port D
        // to change it to be alternative function PIN. Then, re-lock Commit
        // Register of Port D to prevent further changes.
        //
        HWREG(QEI0_GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;    // Unlock CR
        HWREG(QEI0_GPIO_BASE + GPIO_O_CR) |= GPIO_PIN_7;        // Set bit 7
        HWREG(QEI0_GPIO_BASE + GPIO_O_LOCK) = 0;                // Re-lock CR

        //
        // Configure QEI0 Interface PIN.
        //
        ROM_GPIOPinConfigure(QEI0_A_CONFIG);
        ROM_GPIOPinConfigure(QEI0_B_CONFIG);
        ROM_GPIOPinTypeQEI(QEI0_GPIO_BASE, QEI0_PIN_A | QEI0_PIN_B);

        //
        // Configure QEI0 to:
        //      - x4 Resolution
        //      - None Reset Index
        //      - Quadrature Mode
        //      - No Swap
        //      - Pulses per Round = 1319
        //
        ROM_QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET
                        | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1319);

        //
        // Configure Input Filter to eliminate noise from environment.
        //
        QEIFilterConfigure(QEI0_BASE, QEI_FILTCNT_4);
        QEIFilterEnable(QEI0_BASE);

        //
        // Configure the QEI Velocity capture mode.
        //
        ROM_QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, ui32QEIPeriod);

        break;

    case QEI_1:
        //
        // Enable QEI1 and QEI1's GPIO Peripheral.
        //
        ROM_SysCtlPeripheralEnable(QEI1_PERIPH);
        ROM_SysCtlPeripheralEnable(QEI1_GPIO_PERIPH);

        //
        // Configure QEI1 Interface PIN.
        //
        ROM_GPIOPinConfigure(QEI1_A_CONFIG);
        ROM_GPIOPinConfigure(QEI1_B_CONFIG);
        ROM_GPIOPinTypeQEI(QEI1_GPIO_BASE, QEI1_PIN_A | QEI1_PIN_B);

        //
        // Configure QEI1 to:
        //      - x4 Resolution
        //      - None Reset Index
        //      - Quadrature Mode
        //      - No Swap
        //      - Pulses per Round = 1319
        //
        ROM_QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_NO_RESET
                        | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1319);

        //
        // Configure Input Filter to eliminate noise from environment.
        //
        QEIFilterConfigure(QEI1_BASE, QEI_FILTCNT_4);
        QEIFilterEnable(QEI1_BASE);

        //
        // Configure the QEI Velocity capture mode.
        //
        ROM_QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, ui32QEIPeriod);

        break;
    }
}

//*****************************************************************************
//
// Enable QEI Interrupt and specifying the QEI Interrupt Source: Velocity Timer
//
//*****************************************************************************
void QEIVelocIntEnable(QEINumber qei)
{
    switch (qei)
    {
    case QEI_0:
        ROM_IntEnable(INT_QEI0);
        ROM_QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
        break;
    case QEI_1:
        ROM_IntEnable(INT_QEI1);
        ROM_QEIIntEnable(QEI1_BASE, QEI_INTTIMER);
        break;
    }
}

//*****************************************************************************
//
// Clear the QEI Velocity Interrupt Flag.
//
//*****************************************************************************
void QEIVelocIntClear(QEINumber qei)
{
    switch (qei)
    {
    case QEI_0:
        ROM_QEIIntClear(QEI0_BASE, QEI_INTTIMER);
        break;
    case QEI_1:
        ROM_QEIIntClear(QEI1_BASE, QEI_INTTIMER);
        break;
    }
}

//*****************************************************************************
//
// Enable the QEI Velocity module.
//
//*****************************************************************************
void QEIVelocStart(QEINumber qei)
{
    switch (qei)
    {
    case QEI_0:
        ROM_QEIEnable(QEI0_BASE);
        ROM_QEIVelocityEnable(QEI0_BASE);
        break;
    case QEI_1:
        ROM_QEIEnable(QEI1_BASE);
        ROM_QEIVelocityEnable(QEI1_BASE);
        break;
    }
}

//*****************************************************************************
//
// Disable the QEI Velocity module.
//
//*****************************************************************************
void QEIVelocStop(QEINumber qei)
{
    switch (qei)
    {
    case QEI_0:
        ROM_QEIVelocityDisable(QEI0_BASE);
        ROM_QEIDisable(QEI0_BASE);
        break;
    case QEI_1:
        ROM_QEIVelocityDisable(QEI0_BASE);
        ROM_QEIDisable(QEI1_BASE);
        break;
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
