/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>



#define ENCODER_1_RESOLUTION	14
#define ENCODER_2_RESOLUTION	26

#define TIMER_CONF_BOTH_EDGE_T1T2	4

#define MOTOR_1_GEAR		48
#define MOTOR_2_GEAR		48

#define	TIMER_FREQENCY	10
#define	SECOND_IN_MINUTE	60


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t speed_L;
volatile uint32_t speed_R;
volatile uint32_t count = 0;
volatile uint32_t count1 = 0;


typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;
    float prev_error;
    float integral;
    float output;
} PID_TypeDef;

PID_TypeDef pid_L, pid_R;



void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->output = 0;
}


typedef struct {
    float x;
    float y;
    float theta;
} Odometry_TypeDef;

Odometry_TypeDef odom;

void Odometry_Init(Odometry_TypeDef *odom) {
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
}


void Update_Odometry(Odometry_TypeDef *odom, float speed_L, float speed_R, float dt) {
    float wheel_base = 0.2f;
    float wheel_radius = 0.05f;

    // Convert speed from RPM to meters per second
    float v_L = (speed_L / 60.0f) * (2 * M_PI * wheel_radius);
    float v_R = (speed_R / 60.0f) * (2 * M_PI * wheel_radius);

    // Calculate linear and angular velocities
    float v = (v_L + v_R) / 2.0f;
    float omega = (v_R - v_L) / 2 * wheel_base;

    // Update position and orientation
    odom->x += v * cos(odom->theta) * dt;
    odom->y += v * sin(odom->theta) * dt;
    odom->theta += omega * dt;
}



typedef struct {
    float x;
    float y;
} Target_TypeDef;

Target_TypeDef target;

void SetTarget(float x, float y) {
    target.x = x;
    target.y = y;
}


void CalculateTargetError(Odometry_TypeDef *odom, Target_TypeDef *target, float *distance, float *angle) {
    float dx = target->x - odom->x;
    float dy = target->y - odom->y;
    *distance = sqrt(dx * dx + dy * dy);
    *angle = atan2(dy, dx) - odom->theta;
    if (*angle > M_PI) *angle -= 2 * M_PI;
    if (*angle < -M_PI) *angle += 2 * M_PI;
}




void UART_Transmit(const char *data) {
    HAL_UART_Transmit(&huart2, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);
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
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
          } else {
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
          }

}


void ReadSensorOutput() {
    GPIO_PinState output1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    GPIO_PinState output2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

    if (output1 == 1) {
        UART_Transmit("Sensor 1: Object detected\r\n");
    } else {
        UART_Transmit("Sensor 1: No object detected\r\n");
    }

    if (output2 == 1) {
        UART_Transmit("Sensor 2: Object detected\r\n");
    } else {
        UART_Transmit("Sensor 2: No object detected\r\n");
    }
}

int _write(int file, char* ptr, int len){
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
	return len;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim6){

    	speed_L = ( TIMER_FREQENCY * __HAL_TIM_GET_COUNTER(&htim2) * SECOND_IN_MINUTE) / ( MOTOR_1_GEAR * ENCODER_1_RESOLUTION * TIMER_CONF_BOTH_EDGE_T1T2);
    	htim2.Instance->CNT = 0;

    	speed_R = ( TIMER_FREQENCY * __HAL_TIM_GET_COUNTER(&htim3) * SECOND_IN_MINUTE) / ( MOTOR_2_GEAR * ENCODER_2_RESOLUTION * TIMER_CONF_BOTH_EDGE_T1T2);
    	htim3.Instance->CNT = 0;

    }
}


float PID_Compute(PID_TypeDef *pid, float current_value) {

    float error = pid->setpoint - current_value;
    pid->integral += error;
    float derivative = error - pid->prev_error;

    pid->output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    pid->prev_error = error;


    return pid->output;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 100);
  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 100);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start_IT(&htim6);

  Odometry_Init(&odom);

  PID_Init(&pid_L, 2, 0.1, 0.2, 50);
  PID_Init(&pid_R, 2, 0.1, 0.2, 50);

  uint32_t prev_time = HAL_GetTick();


  SetMotorDirection(0,0);

  SetTarget(10.0f, 5.0f);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      	  count = __HAL_TIM_GET_COUNTER(&htim2);
      	  count1 = __HAL_TIM_GET_COUNTER(&htim3);

          uint32_t current_time = HAL_GetTick();
          float dt = (current_time - prev_time) / 1000.0f;
          prev_time = current_time;

          float distance, angle;
          CalculateTargetError(&odom, &target, &distance, &angle);

          // Update PID setpoints
          pid_L.setpoint = distance;
          pid_R.setpoint = angle;

          // Calculate speed using PID controllers
          float speed_L = PID_Compute(&pid_L, 0);
          float speed_R = PID_Compute(&pid_R, 0);

          // Update motor speeds
          //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed_L);
          //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed_R);

          // Update odometry
          Update_Odometry(&odom, speed_L, speed_R, dt);

          // Print odometry and target data for debugging
          printf("Position: x = %.2f, y = %.2f, theta = %.2f\n\r", odom.x, odom.y, odom.theta);
          printf("Target: x = %.2f, y = %.2f\n\r", target.x, target.y);
          printf("Distance to target: %.2f, Angle to target: %.2f\n\r", distance, angle);

          HAL_Delay(50);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
