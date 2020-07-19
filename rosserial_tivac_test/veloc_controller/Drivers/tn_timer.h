/*
 * tn_timer.h
 *
 *  Created on: Mar 18, 2020
 *      Author: Tuan Nguyen
 */

#ifndef TN_TIMER_H_
#define TN_TIMER_H_

//*****************************************************************************
//
// Defines for Timer Peripheral.
//
//*****************************************************************************
#define TIMER_PERIPH    SYSCTL_PERIPH_TIMER0
#define TIMER_BASE      TIMER0_BASE

//*****************************************************************************
//
// Defines for the Timer's mode.
//
//*****************************************************************************
#define TIMER_MODE      TIMER_CFG_ONE_SHOT

//*****************************************************************************
//
// Functions exported from from tn_uart.c
//
//*****************************************************************************
extern void TimerInit();
extern void DelayMs(uint32_t ui32TimeMs);

//*****************************************************************************
//
// Prototypes for the globals exported by this driver.
//
//*****************************************************************************

#endif /* TN_TIMER_H_ */
