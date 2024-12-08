/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Główny plik programu
  ******************************************************************************
  * @attention
  *
  * Prawa autorskie (c) 2024 STMicroelectronics.
  * Wszelkie prawa zastrzeżone.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"




#include "motors.h"
#include "sensors.h"
#include "odometry.h"
#include "pid.h"
#include "communication.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define LED_GPIO_PORT GPIOA
#define LED_PIN GPIO_PIN_5


volatile uint8_t isProcessing = 0;

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
// Deklaracje funkcji

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Funkcja do obsługi printf przez UART

void toggleLED(void) {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    HAL_Delay(100);  // Krótkie mignięcie
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
}


int _write(int file, char* ptr, int len){
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

// Funkcja wywoływana przy przepełnieniu timera
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {

        count = __HAL_TIM_GET_COUNTER(&htim2);
        count1 = __HAL_TIM_GET_COUNTER(&htim3);

        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim3, 0);

        speed_L = count;
        speed_R = count1;

    }
}

// Funkcja do przetwarzania danych odebranych przez UART


// Callback dla przerwania UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (Rx_data == MSG_HEADER) {
            isReceiving = 1;
            bufferIndex = 0;
        }
        else if (isReceiving) {
            if (Rx_data == MSG_FOOTER) {
                messageBuffer[bufferIndex] = '\0';
                ProcessCommand(messageBuffer);
                isReceiving = 0;
            }
            else if (bufferIndex < MAX_MSG_LEN - 1) {
                messageBuffer[bufferIndex++] = Rx_data;
            }
        }

        HAL_UART_Receive_IT(&huart2, (uint8_t*)&Rx_data, 1);
    }
}

// Inicjalizacja regulatora PID


// Ustawienie nowego setpointu dla regulatora PID


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Inicjalizacja zmiennych, struktur, PID itd.
  Odometry_Init(&odom);
  PID_Init(&pid_L, 6, 1.5, 0.1, 1);
  PID_Init(&pid_R, 6, 1.5, 0.3, 1);
  SetTarget(&target, 0.5f, 0);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
   GPIO_InitStruct.Pin = LED_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

  /* USER CODE BEGIN Init */
  // Twoja inicjalizacja
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Dodatkowa konfiguracja systemu, jeśli potrzebna
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  uint8_t* rx_ptr = (uint8_t*)&Rx_data;
  HAL_UART_Receive_IT(&huart2, rx_ptr, 1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  HAL_UART_Receive_IT(&huart2, (uint8_t*)&Rx_data, 1);

 // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
 // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);

  HAL_TIM_Base_Start_IT(&htim6);

  // Skanowanie urządzeń I2C
  Scan_I2C_Devices();

  // Inicjalizacja czujników
  init_sensors();

  // Przypisanie uchwytów I2C do sensorHandles
  // Jeśli nie jest to konieczne, można to pominąć
  /*
  sensorHandles[0].i2c_handle = &hi2c1;
  sensorHandles[1].i2c_handle = &hi2c2;
  sensorHandles[2].i2c_handle = &hi2c3;
  */

  InitCommunication();

  // Informacja o starcie systemu
  HAL_UART_Transmit(&huart2, (uint8_t*)"System Ready\r\n", 13, 100);


  //SetMotorDirection(0,0);

  uint32_t prev_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      read_sensors();

 //     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
 //     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);
      //printf("jaminkkkk \r\n");

      // Aktualizacja odometrii
      uint32_t current_time = HAL_GetTick();
      dt = (current_time - prev_time) / 1000.0f; // Konwersja ms na s
      prev_time = current_time;

 //     Update_Odometry(&odom, speed_L, speed_R, dt);


      SendPWMFeedback(pwm_L, pwm_R);
      // Obliczanie prędkości docelowych
      //CalculateTargetSpeed(&odom, &target, &speed_L_target, &speed_R_target);

      // Ustawienie nowego setpointu dla regulatorów PID
      //SetSpeed(&pid_L, speed_L_target);
      //SetSpeed(&pid_R, speed_R_target);

      // Aktualizacja sterowania PID
      //pwm_L = PID_Compute(&pid_L, speed_L, dt);
      //pwm_R = PID_Compute(&pid_R, speed_R, dt);

      // Sterowanie silnikami na podstawie wyjść z regulatorów PID
      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)pwm_L);
      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)pwm_R);

      // Wysyłanie danych do aplikacji Qt
      //SendDataToQt(&odom, &target, pwm_L, pwm_R, speed_L, speed_R);

      HAL_Delay(1000); // Odpowiedni delay, aby nie przeciążać magistrali I2C

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Twoje pozostałe funkcje, np. odometria, PID, sterowanie silnikami itd.
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
