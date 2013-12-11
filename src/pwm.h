/*
 *  pwm.h
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 */

#ifndef PWM_H_
#define PWM_H_

#include "definitions.h"

// 1000 -> 18 kHz I believe. Prescale is 3, 72M/3/1000 = 18k
//define PWM_PERIODE 1000
#define PWM_PERIODE 1200

extern int MaxCnt[NUMAXES];
extern int MinCnt[NUMAXES];
extern int IrqCnt[NUMAXES];

extern int timer_4_5_deadtime_delay;
extern float testPhase;

void MaxCntClear(void);
void SetRollMotor(float phi, int power);
void SetPitchMotor(float phi, int power);
void SetYawMotor(float phi, int power);

void PWMOff(void);
void PWMConfig(void);

#endif /* PWM_H_ */
