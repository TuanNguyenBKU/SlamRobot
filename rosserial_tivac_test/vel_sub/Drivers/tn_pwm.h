/*
 * tn_pwm.h
 *
 *  Created on: Mar 19, 2020
 *      Author: Tuan Nguyen
 */

#ifndef TN_PWM_H_
#define TN_PWM_H_

//*****************************************************************************
//
// Defines for the common hardware resources used by PWM module:
// Consist of:
//      - PWM Module 0
//      - Generator 0
//      - M0PWM0 - PB6
//      - M0PWM1 - PB7
//
//*****************************************************************************
#define PWM_PERIPH          SYSCTL_PERIPH_PWM0  // PWM Module 0 Peripheral
#define PWM_BASE            PWM0_BASE           // PWM Module 0 Base
#define PWM_GEN             PWM_GEN_0           // Generator 0

#define PWM_GPIO_PERIPH     SYSCTL_PERIPH_GPIOB
#define PWM_GPIO_BASE       GPIO_PORTB_BASE

//*****************************************************************************
//
// Defines for the hardware resources used by M0PWM0:
//
//*****************************************************************************
#define PWM_PIN_0           GPIO_PIN_6

#define PWM_PIN0_CONFIG     GPIO_PB6_M0PWM0     // GPIO_PIN's function.

//*****************************************************************************
//
// Defines for the hardware resources used by M0PWM1:
//
//*****************************************************************************
#define PWM_PIN_1           GPIO_PIN_7

#define PWM_PIN1_CONFIG     GPIO_PB7_M0PWM1    // GPIO_PIN's function.

//*****************************************************************************
//
// Defines for type name of PWM Output Index.
//
//*****************************************************************************
typedef enum {PWM0 = 0, PWM1} pwmindex_t;

//*****************************************************************************
//
// Functions exported from from tn_pwm.c
//
//*****************************************************************************
extern void PWMInit(void);
extern bool PWMFreqConfig(uint32_t ui32Freq);
extern bool PWMDutyConfig(pwmindex_t PWMIndex, uint8_t ui8Duty);
extern void PWMStart(pwmindex_t PWMIndex);
extern void PWMStop(pwmindex_t PWMIndex);

#endif /* TN_PWM_H_ */
