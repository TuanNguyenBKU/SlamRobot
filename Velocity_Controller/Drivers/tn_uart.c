/*
 * tn_uart.c
 *
 *  Created on: Mar 16, 2020
 *      Author: Tuan Nguyen
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#define  TARGET_IS_TM4C123_RA1
#include "driverlib/rom.h"
#include "tn_uart.h"

//*****************************************************************************
//
// Configure UART before using.
// This function must be called during the application initialization to
// configure the UART. User can choose which UART is used by changing the
// definitions in the tn_uart.h file.
//
//*****************************************************************************
void UARTConfig()
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(UART_GPIO_PERIPH);

    //
    // Enable UART.
    //
    ROM_SysCtlPeripheralEnable(UART_PERIPH);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(UART_CONFIG_RX);
    ROM_GPIOPinConfigure(UART_CONFIG_TX);
    ROM_GPIOPinTypeUART(UART_GPIO_BASE, UART_PIN_RX | UART_PIN_TX);

    //
    // Configure UART for:
    //
    // Baud rate: 115200
    // Length: 8
    // Stop bit: 1
    // Parity: None
    //
    ROM_UARTConfigSetExpClk(UART_BASE, SysCtlClockGet(), 115200,
            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

//*****************************************************************************
//
// This function allow to transmit a string through UART.
//
//*****************************************************************************
void UARTPutStr(char *string)
{
    char *ptr_str;

    ptr_str = string;

    while(*ptr_str != '\0')
    {
        ROM_UARTCharPut(UART_BASE, *ptr_str);
        ptr_str ++;
    }
}

//*****************************************************************************
//
// This function allow to display a signed integer number onto console through
// UART.
//
//*****************************************************************************
void UARTPutInt(int32_t u32Int)
{
    char letter;
    char u32Int_Str[10];
    uint8_t count = 0;
    uint8_t sign_flag = 0;

    //
    // Check u32Int < 0 ???
    //
    if (u32Int < 0)
    {
        u32Int = - u32Int;
        sign_flag = 1;
    }

    do
    {
        letter = u32Int % 10;
        u32Int = u32Int / 10;

        u32Int_Str[count] = letter + 48;
        count ++;
    } while (u32Int > 0);

    if (sign_flag)
    {
        u32Int_Str[count] = '-';
        count ++;
    }

    do
    {
        ROM_UARTCharPut(UART_BASE, u32Int_Str[count - 1]);
        count --;
    } while (count > 0);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
