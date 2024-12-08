/*
 * communication.c
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */



// communication.c
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "communication.h"
#include "motors.h"
#include "odometry.h"



volatile uint8_t Rx_data;
char messageBuffer[MAX_MSG_LEN];
int bufferIndex = 0;
int isReceiving = 0;

void InitCommunication(void) {
    HAL_UART_Transmit(&huart2, (uint8_t*)"UART Initialized\r\n", 17, 100);
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&Rx_data, 1);
}

void ParseCommand(char* msg, char* cmd, int* speed) {
    char* token = strtok(msg, ",");
    if (token != NULL) {
        *cmd = token[0];  // Zapisz pierwszy znak jako komendę
        token = strtok(NULL, "#");  // Użyj # jako delimiter, nie kolejnego przecinka
        if (token != NULL) {
            *speed = atoi(token);
        }
    }
}

void ProcessCommand(char* msg) {
    char cmd;
    int speed = 0;
    ParseCommand(msg, &cmd, &speed);  // Przekaż adresy zmiennych

    // Debug - wyświetl otrzymaną komendę
    char debug_msg[64];
    sprintf(debug_msg, "Received: CMD=%c, Speed=%d\r\n", cmd, speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);

    // Przed wykonaniem komendy, wyświetl wartości PWM
    char debug_pwm[64];
    sprintf(debug_pwm, "Setting PWM to: %d\r\n", speed);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_pwm, strlen(debug_pwm), 100);

    switch(cmd) {
        case CMD_FORWARD:
            SetMotorDirection(1, 1);
            SetMotorsPWM(speed, speed);
            break;

        case CMD_BACKWARD:
            SetMotorDirection(0, 0);
            SetMotorsPWM(speed, speed);
            break;

        case CMD_LEFT:
            SetMotorDirection(1, 0);
            SetMotorsPWM(speed, speed);
            break;

        case CMD_RIGHT:
            SetMotorDirection(0, 1);
            SetMotorsPWM(speed, speed);
            break;

        case CMD_STOP:
            SetMotorsPWM(0, 0);
            break;
    }
}
void SendPWMFeedback(float pwmL, float pwmR) {
    char buffer[32];
    // Dodaj qDebug do sprawdzenia wartości przed wysłaniem
    sprintf(buffer, "$PWM,%.2f,%.2f#\r\n", pwm_L, pwm_R);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}
