/*
 * odometry.h
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "main.h"

// Type definitions must come before variable declarations
typedef struct {
    float x;
    float y;
    float theta;
    float v_linear;
    float v_angular;
} Odometry_TypeDef;


typedef struct {
    float x;
    float y;
    // Add other necessary fields
} Target_TypeDef;


extern Odometry_TypeDef odom;
extern Target_TypeDef target;


extern float dt;
extern volatile float distance;


void Odometry_Init(Odometry_TypeDef *odom);
void SetTarget(Target_TypeDef *target, float x, float y);
void Update_Odometry(Odometry_TypeDef *odom, float speed_L, float speed_R, float dt);

void CalculateTargetSpeed(Odometry_TypeDef *odom, Target_TypeDef *target,
                         float *speed_L_target, float *speed_R_target);

#endif
