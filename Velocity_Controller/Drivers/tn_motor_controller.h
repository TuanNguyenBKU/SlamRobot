/*
 * tn_motor_controller.h
 *
 *  Created on: Jul 8, 2020
 *      Author: Tuan Nguyen
 */

#ifndef TN_MOTOR_CONTROLLER_H_
#define TN_MOTOR_CONTROLLER_H_

//*****************************************************************************
//
// Define for hardware resources used by Motor Controller.
// Left Motor: M0PWM0
//      IN1_L - PA3
//      IN2_L - PA4
// Right Motor: M0PWM1
//      IN1_R - PA6
//      IN2_R - PA7
//
//*****************************************************************************
#define DIR_PERIPH    SYSCTL_PERIPH_GPIOA
#define DIR_BASE      GPIO_PORTA_BASE

#define PIN_IN1_L     GPIO_PIN_3
#define PIN_IN2_L     GPIO_PIN_4

#define PIN_IN1_R     GPIO_PIN_7
#define PIN_IN2_R     GPIO_PIN_6

//*****************************************************************************
//
// Defines for Sample Time: ms
//
//*****************************************************************************
#define SAMP_TIME   20

//*****************************************************************************
//
// Defines for PWM Frequency: Hz
//
//*****************************************************************************
#define PWM_FREQ    1000

//*****************************************************************************
//
// Define for type name of Direction variable.
//
//*****************************************************************************
typedef enum{STOP = 0, BACKWARD, FORWARD} dir_t;

//*****************************************************************************
//
// Define for type name of Motor variable.
//
//*****************************************************************************
typedef enum{LEFT = 0, RIGHT} motor_t;

//*****************************************************************************
//
// Functions exported from tn_qei.c
//
//*****************************************************************************
extern void QEI0TimerIntHandler();
extern void QEI1TimerIntHandler();
extern void DirCtrl(motor_t MotorIndex, dir_t Dir);
extern void MotorInit();
extern void MotorControl(motor_t MotorIndex, int32_t i32Veloc);

#endif /* TN_MOTOR_CONTROLLER_H_ */
