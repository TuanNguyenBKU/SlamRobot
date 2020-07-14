/*
 * tn_uart.h
 *
 *  Created on: Mar 16, 2020
 *      Author: Tuan Nguyen
 */

#ifndef TN_UART_H_
#define TN_UART_H_

//*****************************************************************************
//
// Defines for the hardware resources used by UART.
// UART0 is used on the port A:
//
// PA0 - RX
// PA1 - TX
//
//*****************************************************************************
#define UART_PERIPH         SYSCTL_PERIPH_UART0
#define UART_BASE           UART0_BASE

#define UART_GPIO_PERIPH    SYSCTL_PERIPH_GPIOA
#define UART_GPIO_BASE      GPIO_PORTA_BASE
#define UART_PIN_RX         GPIO_PIN_0
#define UART_PIN_TX         GPIO_PIN_1

#define UART_CONFIG_RX      GPIO_PA0_U0RX
#define UART_CONFIG_TX      GPIO_PA1_U0TX

//*****************************************************************************
//
// Functions exported from from tn_uart.c
//
//*****************************************************************************
extern void UARTConfig();
extern void UARTPutStr(char *string);
extern void UARTPutInt(int32_t u32Int);

//*****************************************************************************
//
// Prototypes for the globals exported by this driver.
//
//*****************************************************************************

#endif /* TN_UART_H_ */
