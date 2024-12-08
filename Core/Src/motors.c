#include "motors.h"
#include "main.h"
#include <stdio.h>
#include <string.h>


volatile uint32_t speed_L = 0;
volatile uint32_t speed_R = 0;
volatile uint32_t count = 0;
volatile uint32_t count1 = 0;
float pwm_L = 0, pwm_R = 0;

void InitMotors(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void SetMotorDirection(int direction_L, int direction_R) {
    if (direction_L == 1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
    }

    if (direction_R == 1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
    }
}

void SetMotorsPWM(uint32_t pwm_left, uint32_t pwm_right) {
    // Aktualizacja globalnych zmiennych PWM
    pwm_L = pwm_left;
    pwm_R = pwm_right;

    // Ograniczenie wartości PWM do bezpiecznego zakresu
    if (pwm_left > MAX_PWM) pwm_left = MAX_PWM;
    if (pwm_right > MAX_PWM) pwm_right = MAX_PWM;

    if (pwm_left < MIN_PWM) pwm_left = MIN_PWM;
    if (pwm_right < MIN_PWM) pwm_right = MIN_PWM;

    // Debug
    char debug_msg[64];
    sprintf(debug_msg, "Setting PWM: L=%lu, R=%lu\r\n", pwm_left, pwm_right);
    HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, strlen(debug_msg), 100);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_left);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_right);
}



void Motors_Forward(uint32_t speed) {
    SetMotorDirection(1, 1);
    SetMotorsPWM(speed, speed);
}

void Motors_Backward(uint32_t speed) {
    SetMotorDirection(0, 0);
    SetMotorsPWM(speed, speed);
}

void Motors_TurnLeft(uint32_t speed) {
    SetMotorDirection(0, 1);
    SetMotorsPWM(speed, speed);
}

void Motors_TurnRight(uint32_t speed) {
    SetMotorDirection(1, 0);
    SetMotorsPWM(speed, speed);
}

void Motors_Stop(void) {
    SetMotorsPWM(0, 0);
}
