/*
 * communication.h
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */

// communication.h
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "main.h"
#include "odometry.h"

#define CMD_FORWARD 'F'
#define CMD_BACKWARD 'B'
#define CMD_LEFT 'L'
#define CMD_RIGHT 'R'
#define CMD_STOP 'S'

extern volatile uint8_t Rx_data;
extern char flag;
extern uint8_t uartBuffer[UART_BUFFER_SIZE];

void ProcessCommand(char cmd);
void SendDataToQt(Odometry_TypeDef *odom, Target_TypeDef *target,
                 float pwm_L, float pwm_R, float speed_L, float speed_R);
void InitCommunication(void);

#endif



