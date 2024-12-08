/*
 * pid.h
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */

#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float prev_error;
    float integral;
    float output;
    uint32_t prev_time;
} PID_TypeDef;

extern volatile int pid_iterations;
extern PID_TypeDef pid_L, pid_R;
extern float speed_L_target, speed_R_target;


void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float setpoint);
float PID_Compute(PID_TypeDef *pid, float current_value, float dt);
void SetSpeed(PID_TypeDef *pid, float setpoint);

#endif
