/*
 * tn_pid.h
 *
 *  Created on: Jun 14, 2020
 *      Author: Tuan Nguyen
 */

#ifndef DRIVERS_TN_PID_H_
#define DRIVERS_TN_PID_H_

//*****************************************************************************
//
// Defines for the range of valid PWM's value.
//
//*****************************************************************************
#define PWM_MIN     5
#define PWM_MAX     95

//*****************************************************************************
//
// Defines for PID's coefficient. At: 70 rpm
// Kp  0.5
// Ki  4
// Kd  0.002
//
//*****************************************************************************
#define Kp  0.5
#define Ki  4
#define Kd  0.002

//*****************************************************************************
//
// Functions exported from from tn_pwm.c
//
//*****************************************************************************
extern int32_t CalculatePID(int32_t i32CurrentVel, int32_t i32SetVel,
                            float* floIn, int32_t* i32PreError,
                            uint16_t ui16SampTime);

#endif /* DRIVERS_TN_PID_H_ */
