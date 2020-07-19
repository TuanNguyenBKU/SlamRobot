/*
 * tn_qei.h
 *
 *  Created on: Apr 21, 2020
 *      Author: Tuan Nguyen
 */

#ifndef DRIVERS_TN_QEI_H_
#define DRIVERS_TN_QEI_H_

//*****************************************************************************
//
// Defines for QEI0 Peripheral and hardware resources used by QEI0.
//
//*****************************************************************************
#define QEI0_PERIPH         SYSCTL_PERIPH_QEI0

#define QEI0_GPIO_PERIPH    SYSCTL_PERIPH_GPIOD
#define QEI0_GPIO_BASE      GPIO_PORTD_BASE

#define QEI0_A_CONFIG       GPIO_PD6_PHA0
#define QEI0_B_CONFIG       GPIO_PD7_PHB0

#define QEI0_PIN_A          GPIO_PIN_6
#define QEI0_PIN_B          GPIO_PIN_7

//*****************************************************************************
//
// Defines for QEI1 Peripheral and hardware resources used by QEI1.
//
//*****************************************************************************
#define QEI1_PERIPH         SYSCTL_PERIPH_QEI1

#define QEI1_GPIO_PERIPH    SYSCTL_PERIPH_GPIOC
#define QEI1_GPIO_BASE      GPIO_PORTC_BASE

#define QEI1_A_CONFIG       GPIO_PC5_PHA1
#define QEI1_B_CONFIG       GPIO_PC6_PHB1

#define QEI1_PIN_A          GPIO_PIN_5
#define QEI1_PIN_B          GPIO_PIN_6

//*****************************************************************************
//
// Defines for type name of variable indicating which QEI module is used.
//
//*****************************************************************************
typedef enum {QEI_0 = 0, QEI_1} QEINumber;

//*****************************************************************************
//
// Functions exported from tn_qei.c
//
//*****************************************************************************
extern void QEIVelocInit(QEINumber qei, uint32_t ui32SamplePeriod);
extern void QEIVelocIntEnable(QEINumber qei);
extern void QEIVelocIntClear(QEINumber qei);
extern void QEIVelocStart(QEINumber qei);
extern void QEIVelocStop(QEINumber qei);

//*****************************************************************************
//
// Prototypes for the globals exported by this driver.
//
//*****************************************************************************

#endif /* DRIVERS_TN_QEI_H_ */
