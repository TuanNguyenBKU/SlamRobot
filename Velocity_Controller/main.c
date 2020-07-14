#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "driverlib/qei.h"

#include "drivers/tn_pwm.h"
#include "drivers/tn_timer.h"
#include "drivers/tn_pid.h"
#include "drivers/tn_uart.h"
#include "drivers/tn_qei.h"
#include "drivers/tn_motor_controller.h"

//*****************************************************************************
//
// main() function.
//
//*****************************************************************************
int main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL
                   |SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    TimerInit();
    UARTConfig();

    UARTPutInt(0);
    UARTPutStr("\n");

    MotorInit();

    while(1)
    {
        MotorControl(LEFT, 70);
        MotorControl(RIGHT, 70);

        DelayMs(3000);

        MotorControl(LEFT, -70);
        MotorControl(RIGHT, -70);

        DelayMs(3000);
    }
}

