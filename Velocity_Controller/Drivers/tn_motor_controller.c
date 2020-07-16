/*
 * tn_motor_controller.c
 *
 *  Created on: Jul 8, 2020
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
#include "tn_pid.h"
#include "tn_qei.h"
#include "tn_uart.h"
#include "tn_motor_controller.h"

//*****************************************************************************
//
// Declare for global variables.
//
//*****************************************************************************
volatile int32_t i32LeftSetVel = 0;
volatile int32_t i32RightSetVel = 0;

float floLeftIn = 0;
float floRightIn = 0;

int32_t i32LeftPreError = 0;
int32_t i32RightPreError = 0;

volatile brk_t LeftStopFlag = FLOAT;
volatile brk_t RightStopFlag = FLOAT;

//*****************************************************************************
//
// This function handle the QEI0 Interrupt. This interruption occur when the
// velocity timer expires.
// Used to control the velocity of Left Motor.
//
//*****************************************************************************
void QEI0TimerIntHandler()
{
    //
    // Clear QEI0 Velocity Interrupt Flag.
    //
    QEIVelocIntClear(QEI_0);

    //
    // Read the Velocity counter value of QEI module per period.
    //
    uint32_t ui32VelocPulse = 0;
    int32_t i32Dir = 0;

    ui32VelocPulse = ROM_QEIVelocityGet(QEI0_BASE);
    i32Dir = ROM_QEIDirectionGet(QEI0_BASE);

    //
    // Calculate Current Velocity: RPM.
    // CurrentVeloc = (VelocPulse * 60) / (PulsesPerRevolutions * Edges * Delta_t)
    //      VelocPulse: Counter value of QEI module per period.
    //      PulsesPerRevolutions = 11 * 30 = 330
    //      Edges: Resolution of encoder = x4
    //      Delta_t: SAMPLE_TIME (ms)
    //
    int32_t i32CurrentVel = 0;
    i32CurrentVel = (ui32VelocPulse * 60 * 1000) / (330 * 4 * SAMP_TIME);
    i32CurrentVel = i32CurrentVel * i32Dir;

    //
    // Display the Left Velocity: RPM.
    //
//    UARTPutInt(i32CurrentVel);
//    UARTPutStr("\n");

    //
    // Calculate PID: PWM Duty Circle value.
    //
    int32_t i32PWMValue = 0;
    i32PWMValue = CalculatePID(i32CurrentVel, i32LeftSetVel,
                               &floLeftIn, &i32LeftPreError, SAMP_TIME);

    //
    // Specify the Direction.
    //
    if ((i32PWMValue >= 0) && (LeftStopFlag == FLOAT))
    {
        DirCtrl(LEFT, FORWARD);
    }
    else if ((i32PWMValue < 0) && (LeftStopFlag == FLOAT))
    {
        DirCtrl(LEFT, BACKWARD);
        i32PWMValue = -i32PWMValue;
    }
    else if (LeftStopFlag == BREAK)
    {
        DirCtrl(LEFT, STOP);
    }

    //
    // Generate PWM signal.
    //
    PWMDutyConfig(PWM0, i32PWMValue);
}

//*****************************************************************************
//
// This function handle the QEI1 Interrupt. This interruption occur when the
// velocity timer expires.
// Used to control the velocity of Right Motor.
//
//*****************************************************************************
void QEI1TimerIntHandler()
{

    // Clear QEI0 Interrupt Flag.

    QEIVelocIntClear(QEI_1);

    //
    // Read the Velocity counter value of QEI module per period.
    //
    uint32_t ui32VelocPulse = 0;
    int32_t i32Dir = 0;

    ui32VelocPulse = ROM_QEIVelocityGet(QEI1_BASE);
    i32Dir = ROM_QEIDirectionGet(QEI1_BASE);

    //
    // Calculate Current Velocity: RPM.
    // CurrentVeloc = (VelocPulse * 60) / (PulsesPerRevolutions * Edges * Delta_t)
    //      VelocPulse: Counter value of QEI module per period.
    //      PulsesPerRevolutions = 11 * 30 = 330
    //      Edges: Resolution of encoder = x4
    //      Delta_t: SAMPLE_TIME (ms)
    //
    int32_t i32CurrentVel = 0;
    i32CurrentVel = (ui32VelocPulse * 60 * 1000) / (330 * 4 * SAMP_TIME);
    i32CurrentVel = i32CurrentVel * i32Dir;

    //
    // Display the Right Velocity: RPM.
    //
    UARTPutInt(i32CurrentVel);
    UARTPutStr("\n");

    //
    // Calculate PID: PWM Duty Circle value.
    //
    int32_t i32PWMValue = 0;
    i32PWMValue = CalculatePID(i32CurrentVel, i32RightSetVel,
                               &floRightIn, &i32RightPreError, SAMP_TIME);

    //
    // Specify the Direction.
    //
    if ((i32PWMValue >= 0) && (RightStopFlag == FLOAT))
    {
        DirCtrl(RIGHT, FORWARD);
    }
    else if ((i32PWMValue < 0) && (RightStopFlag == FLOAT))
    {
        DirCtrl(RIGHT, BACKWARD);
        i32PWMValue = -i32PWMValue;
    }
    else if (RightStopFlag == BREAK)
    {
        DirCtrl(RIGHT, STOP);
    }

    //
    // Generate PWM signal.
    //
    PWMDutyConfig(PWM1, i32PWMValue);
}

//*****************************************************************************
//
// This function control the direction of motor.
//
//*****************************************************************************
void DirCtrl(motor_t MotorIndex, dir_t Dir)
{
    switch (MotorIndex)
    {
    case LEFT:
        switch (Dir)
        {
        case STOP:
            GPIOPinWrite(DIR_BASE, PIN_IN2_L | PIN_IN1_L, 0);
            break;
        case BACKWARD:
            GPIOPinWrite(DIR_BASE, PIN_IN2_L | PIN_IN1_L, PIN_IN2_L);
            break;
        case FORWARD:
            GPIOPinWrite(DIR_BASE, PIN_IN2_L | PIN_IN1_L, PIN_IN1_L);
            break;
        }
        break;

    case RIGHT:
        switch (Dir)
        {
        case STOP:
            GPIOPinWrite(DIR_BASE, PIN_IN2_R | PIN_IN1_R, 0);
            break;
        case BACKWARD:
            GPIOPinWrite(DIR_BASE, PIN_IN2_R | PIN_IN1_R, PIN_IN2_R);
            break;
        case FORWARD:
            GPIOPinWrite(DIR_BASE, PIN_IN2_R | PIN_IN1_R, PIN_IN1_R);
            break;
        }
        break;
    }
}

//*****************************************************************************
//
// This function Initialize all of necessary modules used by this driver.
// Enable all of necessary ISR.
//
//*****************************************************************************
void MotorInit()
{
    //
    // Initialize the hardware resources used by \DirCtrl();
    //
    ROM_SysCtlPeripheralEnable(DIR_PERIPH);
    ROM_GPIOPinTypeGPIOOutput(DIR_BASE, PIN_IN1_L | PIN_IN2_L | PIN_IN1_R| PIN_IN2_R);

    //
    // Initialize PWM module
    //
    PWMInit();
    PWMFreqConfig(PWM_FREQ);

    PWMDutyConfig(PWM0, 0);
    PWMDutyConfig(PWM1, 0);

    DirCtrl(LEFT, STOP);
    DirCtrl(RIGHT, STOP);

    PWMStart(PWM0);
    PWMStart(PWM1);

    //
    // Initialize QEI module
    //
    QEIVelocInit(QEI_0, SAMP_TIME);
    QEIVelocInit(QEI_1, SAMP_TIME);

    //
    // Enable QEI Velocity Interrupt
    //
    QEIVelocIntEnable(QEI_0);
    QEIVelocIntEnable(QEI_1);
    ROM_IntMasterEnable();      // Enable Global Interrupt

    //
    // Enable both two QEI modules
    //
    QEIVelocStart(QEI_0);
    QEIVelocStart(QEI_1);
}

//*****************************************************************************
//
// This function control both 2 motors.
// User invoke this function, provide the velocity and which motor.
// Velocity = 0 mean that motor is in breaking mode.
//
//*****************************************************************************
void MotorControl(motor_t MotorIndex, int32_t i32Veloc)
{
    switch(MotorIndex)
    {
    case LEFT:
        if (i32Veloc == 0)
        {
            LeftStopFlag = BREAK;
        }
        else
        {
            LeftStopFlag = FLOAT;
        }

        i32LeftPreError = 0;
        floLeftIn = 0;
        i32LeftSetVel = i32Veloc;
        break;

    case RIGHT:
        if (i32Veloc == 0)
        {
            RightStopFlag = BREAK;
        }
        else
        {
            RightStopFlag = FLOAT;
        }
        i32RightPreError = 0;
        floRightIn = 0;
        i32RightSetVel = i32Veloc;
        break;
    }
}























