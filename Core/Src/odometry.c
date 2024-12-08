/*
 * odometry.c
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */

#include "odometry.h"
#include <math.h>

Odometry_TypeDef odom;
Target_TypeDef target;
float dt;
volatile float distance;



#define WHEEL_RADIUS            0.035
#define WHEEL_BASE              0.15
#define ENCODER_1_RESOLUTION    14
#define MOTOR_1_GEAR            48
#define ENCODER_2_RESOLUTION    14
#define MOTOR_2_GEAR            48

void Odometry_Init(Odometry_TypeDef *odom) {
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
}

void Update_Odometry(Odometry_TypeDef *odom, float speed_L, float speed_R, float dt) {
    float v_L = speed_L * (2 * M_PI * WHEEL_RADIUS) / (ENCODER_1_RESOLUTION * MOTOR_1_GEAR);
    float v_R = speed_R * (2 * M_PI * WHEEL_RADIUS) / (ENCODER_2_RESOLUTION * MOTOR_2_GEAR);

    float v = (v_L + v_R) / 2.0;
    float omega = (v_R - v_L) / WHEEL_BASE;

    odom->theta += omega * dt;
    odom->x += v * cos(odom->theta) * dt;
    odom->y += v * sin(odom->theta) * dt;
}

void SetTarget(Target_TypeDef *target, float x, float y) {
    target->x = x;
    target->y = y;
}

void CalculateTargetSpeed(Odometry_TypeDef *odom, Target_TypeDef *target,
                         float *speed_L_target, float *speed_R_target) {
    float dx = target->x - odom->x;
    float dy = target->y - odom->y;
    float distance = sqrt(dx * dx + dy * dy);
    float angle_to_target = atan2(dy, dx);

    float angle_error = angle_to_target - odom->theta;
    if (angle_error > M_PI) angle_error -= 2 * M_PI;
    if (angle_error < -M_PI) angle_error += 2 * M_PI;

    float max_linear_speed = 120.0f;
    float max_angular_speed = 1000.0f;
    float linear_speed_kp = 60.0f;
    float angular_speed_kp = 10.0f;

    float linear_speed = linear_speed_kp * distance;
    if (linear_speed > max_linear_speed) {
        linear_speed = max_linear_speed;
    }

    float angular_speed = angular_speed_kp * angle_error;
    if (angular_speed > max_angular_speed) {
        angular_speed = max_angular_speed;
    } else if (angular_speed < -max_angular_speed) {
        angular_speed = -max_angular_speed;
    }

    *speed_L_target = linear_speed - (WHEEL_BASE / 2) * angular_speed;
    *speed_R_target = linear_speed + (WHEEL_BASE / 2) * angular_speed;
}
