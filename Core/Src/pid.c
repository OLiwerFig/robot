#include "pid.h"

volatile int pid_iterations = 0;
PID_TypeDef pid_L, pid_R;
float speed_L_target, speed_R_target;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
    pid->prev_time = HAL_GetTick();
}

float PID_Compute(PID_TypeDef *pid, float current_value, float dt) {
    float error = pid->setpoint - current_value;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;

    pid->output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    pid->prev_error = error;

    // Limit output
    if (pid->output > 1000) pid->output = 1000;
    if (pid->output < 0) pid->output = 0;

    return pid->output;
}

void SetSpeed(PID_TypeDef *pid, float setpoint) {
    pid->setpoint = setpoint;
}
