/*
 * communication.c
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */



// communication.c
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "communication.h"
#include "motors.h"
#include "odometry.h"

volatile uint8_t Rx_data = 0;  // Definicja zmiennej z volatile
char flag = 0;
int speed = 400;
uint8_t uartBuffer[UART_BUFFER_SIZE];



void SendDataToQt(Odometry_TypeDef *odom, Target_TypeDef *target,
                 float pwm_L, float pwm_R, float speed_L, float speed_R) {
    char buffer[100];
    sprintf(buffer, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
            speed_L, speed_R, pwm_L, pwm_R, odom->x, odom->y,
            odom->theta, target->x, target->y);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}


void ProcessCommand(char cmd) {
    // Wysyłamy potwierdzenie otrzymania komendy
    char debug_msg[32];
    sprintf(debug_msg, "CMD: %c\r\n", cmd);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);

    switch(cmd) {
        case CMD_FORWARD:
            SetMotorDirection(1, 1);
            SetMotorsPWM(speed, speed);
            HAL_UART_Transmit(&huart2, (uint8_t*)"Moving Forward\r\n", 15, 100);
            break;

        case CMD_BACKWARD:
            SetMotorDirection(0, 0);
            SetMotorsPWM(speed, speed);
            HAL_UART_Transmit(&huart2, (uint8_t*)"Moving Backward\r\n", 16, 100);
            break;

        case CMD_LEFT:
            SetMotorDirection(1, 0);
            SetMotorsPWM(speed, speed);
            HAL_UART_Transmit(&huart2, (uint8_t*)"Turning Left\r\n", 13, 100);
            break;

        case CMD_RIGHT:
            SetMotorDirection(0, 1);
            SetMotorsPWM(speed, speed);
            HAL_UART_Transmit(&huart2, (uint8_t*)"Turning Right\r\n", 14, 100);
            break;

        case CMD_STOP:
            SetMotorsPWM(0, 0);
            HAL_UART_Transmit(&huart2, (uint8_t*)"Stopped\r\n", 9, 100);
            break;
    }
}

void InitCommunication(void) {
    // Włączamy UART w trybie przerwań
    HAL_UART_Transmit(&huart2, (uint8_t*)"UART Initialized\r\n", 17, 100);
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&Rx_data, 1);
}
