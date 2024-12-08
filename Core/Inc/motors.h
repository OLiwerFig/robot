/*
 * motors.h
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */

// motors.h
#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"

#define MAX_PWM 1000
#define MIN_PWM 0

extern volatile uint32_t speed_L;
extern volatile uint32_t speed_R;
extern volatile uint32_t count;
extern volatile uint32_t count1;
extern float pwm_L, pwm_R;

void InitMotors(void);
void SetMotorDirection(int direction_L, int direction_R);
void SetMotorsPWM(uint32_t pwm_left, uint32_t pwm_right);
void Motors_Forward(uint32_t speed);
void Motors_Backward(uint32_t speed);
void Motors_TurnLeft(uint32_t speed);
void Motors_TurnRight(uint32_t speed);
void Motors_Stop(void);

#endif
