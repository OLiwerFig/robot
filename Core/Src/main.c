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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "vl53l5cx_api.h"
#include "platform.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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

typedef struct {
    float x;
    float y;
    float theta;
} Odometry_TypeDef;

typedef struct {
    float x;
    float y;
} Target_TypeDef;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE              64
#define TIMER_CONF_BOTH_EDGE_T1T2     4
#define TIMER_FREQUENCY               10
#define SECOND_IN_MINUTE              60

#define ENCODER_1_RESOLUTION    14
#define ENCODER_2_RESOLUTION    14

#define MOTOR_1_GEAR            48
#define MOTOR_2_GEAR            48

#define WHEEL_RADIUS            0.035
#define WHEEL_BASE              0.15

#define DEFAULT_SENSOR_1_ADDR    0x29
#define DEFAULT_SENSOR_2_ADDR    0x54

uint8_t sensor1_addr = DEFAULT_SENSOR_1_ADDR;
uint8_t sensor2_addr = DEFAULT_SENSOR_2_ADDR;

#define I2C_RESET_PIN GPIO_PIN_3
#define I2C_RESET_PORT GPIOC




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Zmienne globalne
VL53L5CX_Configuration dev;
VL53L5CX_ResultsData results;
uint16_t sensor_address = VL53L5CX_DEFAULT_I2C_ADDRESS; // Adres czujnika

volatile uint32_t speed_L = 0;
volatile uint32_t speed_R = 0;
volatile uint32_t count = 0;
volatile uint32_t count1 = 0;

uint8_t uartBuffer[UART_BUFFER_SIZE];

float speed_L_target, speed_R_target, pwm_R, pwm_L, dt;

volatile int pid_iterations = 0;
volatile float distance;

PID_TypeDef pid_L, pid_R;

Odometry_TypeDef odom;
Target_TypeDef target;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// Deklaracje funkcji
void ProcessReceivedData(uint8_t* data, uint16_t length);
void SetTarget(float x, float y);
void Scan_I2C_Devices(void);
void Initialize_Two_Sensors(void);
void ProcessData(VL53L5CX_ResultsData *results);
void Update_Odometry(Odometry_TypeDef *odom, float speed_L, float speed_R, float dt);
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float setpoint);
float PID_Compute(PID_TypeDef *pid, float current_value, float dt);
void SetSpeed(PID_TypeDef *pid, float setpoint);
void SetMotorDirection(int direction_L, int direction_R);
void SendDataToQt(Odometry_TypeDef *odom, Target_TypeDef *target ,float pwm_L ,float pwm_R,  float speed_L, float speed_R);
void Odometry_Init(Odometry_TypeDef *odom);
void CalculateTargetSpeed(Odometry_TypeDef *odom, Target_TypeDef *target, float *speed_L_target, float *speed_R_target);
void ScanAndInitializeSensors(void);
void I2C_ResetBus(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Funkcja do obsługi printf przez UART
int _write(int file, char* ptr, int len){
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

// Funkcja wywoływana przy przepełnieniu timera
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {

        count = __HAL_TIM_GET_COUNTER(&htim2);
        count1 = __HAL_TIM_GET_COUNTER(&htim3);

        htim2.Instance->CNT = 0;
        htim3.Instance->CNT = 0;

        // Obliczanie prędkości na podstawie enkoderów
        speed_L = count;   // Dostosuj zgodnie z rozdzielczością enkodera
        speed_R = count1;

        // Tutaj możesz dodać dodatkowe obliczenia prędkości
    }
}

// Funkcja do przetwarzania danych odebranych przez UART
void ProcessReceivedData(uint8_t* data, uint16_t length)
{
    if (length >= sizeof(float) * 2) {
        float targetX, targetY;
        memcpy(&targetX, data, sizeof(float));
        memcpy(&targetY, data + sizeof(float), sizeof(float));
        SetTarget(targetX, targetY);
    }
}

// Callback dla przerwania UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        ProcessReceivedData(uartBuffer, UART_BUFFER_SIZE);
        HAL_UART_Receive_IT(&huart2, uartBuffer, UART_BUFFER_SIZE);
    }
}

// Inicjalizacja regulatora PID
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

// Ustawienie nowego setpointu dla regulatora PID
void SetSpeed(PID_TypeDef *pid, float setpoint) {
    pid->setpoint = setpoint;
}

// Inicjalizacja odometrii
void Odometry_Init(Odometry_TypeDef *odom) {
    odom->x = 0.0f;
    odom->y = 0.0f;
    odom->theta = 0.0f;
}

// Aktualizacja odometrii
void Update_Odometry(Odometry_TypeDef *odom, float speed_L, float speed_R, float dt) {
    float v_L = speed_L * (2 * M_PI * WHEEL_RADIUS) / (ENCODER_1_RESOLUTION * MOTOR_1_GEAR);
    float v_R = speed_R * (2 * M_PI * WHEEL_RADIUS) / (ENCODER_2_RESOLUTION * MOTOR_2_GEAR);

    float v = (v_L + v_R) / 2.0;
    float omega = (v_R - v_L) / WHEEL_BASE;

    // Aktualizuj pozycje
    odom->theta += omega * dt;
    odom->x += v * cos(odom->theta) * dt;
    odom->y += v * sin(odom->theta) * dt;
}

// Ustawienie nowego celu
void SetTarget(float x, float y) {
    target.x = x;
    target.y = y;
}

// Obliczanie prędkości docelowych dla silników
void CalculateTargetSpeed(Odometry_TypeDef *odom, Target_TypeDef *target, float *speed_L_target, float *speed_R_target) {
    float dx = target->x - odom->x;
    float dy = target->y - odom->y;
    distance = sqrt(dx * dx + dy * dy);
    float angle_to_target = atan2(dy, dx);

    float angle_error = angle_to_target - odom->theta;
    if (angle_error > M_PI) angle_error -= 2 * M_PI;
    if (angle_error < -M_PI) angle_error += 2 * M_PI;

    // Parametry do dostosowania
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

// Ustawienie kierunku obrotu silników
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

// Wysyłanie danych do aplikacji Qt
void SendDataToQt(Odometry_TypeDef *odom, Target_TypeDef *target ,float pwm_L ,float pwm_R,  float speed_L, float speed_R) {
    char buffer[100];
    sprintf(buffer, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", speed_L, speed_R, pwm_L, pwm_R, odom->x, odom->y, odom->theta, target->x, target->y);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

// Obliczanie wyjścia regulatora PID
float PID_Compute(PID_TypeDef *pid, float current_value, float dt) {
    float error = pid->setpoint - current_value;

    pid->integral += error * dt;

    float derivative = (error - pid->prev_error) / dt;

    pid->output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    pid->prev_error = error;

    if (pid_iterations < 4) {
        if (pid->output > 30) pid->output  = 30;
        if (pid->output < 0) pid->output  = 0;
    } else {
        if (pid->output > 1000) pid->output  = 1000;
        if (pid->output < 0) pid->output  = 0;
    }

    pid_iterations++;

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
  // Inicjalizacja zmiennych, struktur, PID itd.
  Odometry_Init(&odom);
  PID_Init(&pid_L, 6, 1.5, 0.1, 1);
  PID_Init(&pid_R, 6, 1.5, 0.3, 1);
  SetTarget(0.5f, 0);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

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
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, uartBuffer, UART_BUFFER_SIZE);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);

  HAL_TIM_Base_Start_IT(&htim6);

  // Skanowanie urządzeń I2C
  I2C_ResetBus();

  Scan_I2C_Devices();

  ScanAndInitializeSensors();



  // Inicjalizacja czujnika
  Initialize_Two_Sensors();

  // Start pomiarów z czujnika
  printf("Rozpoczynanie pomiarów...\r\n");
  int status = vl53l5cx_start_ranging(&dev);
  if (status == VL53L5CX_STATUS_OK) {
      printf("Pomiary rozpoczęte pomyślnie\r\n");
  } else {
      printf("Błąd rozpoczynania pomiarów, kod błędu: %d\r\n", status);
  }

  SetMotorDirection(0,0);

  uint32_t prev_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // Odczyt danych z pierwszego czujnika
	  dev.platform.address = sensor1_addr;
	    uint8_t isReady = 0;
	    status = vl53l5cx_check_data_ready(&dev, &isReady);
	    if (status == VL53L5CX_STATUS_OK && isReady) {
	        status = vl53l5cx_get_ranging_data(&dev, &results);
	        if (status == VL53L5CX_STATUS_OK) {
	            printf("Czujnik 1 wysyła dane:\r\n");
	            ProcessData(&results);
	        } else {
	            printf("Błąd odczytu danych z Czujnika 1, kod błędu: %d\r\n", status);
	        }
	    } else {
	        printf("Czujnik 1 nie ma nowych danych do odczytu\r\n");
	    }

	    // Odczyt danych z drugiego czujnika
	    dev.platform.address = sensor2_addr;
	    isReady = 0;
	    status = vl53l5cx_check_data_ready(&dev, &isReady);
	    if (status == VL53L5CX_STATUS_OK && isReady) {
	        status = vl53l5cx_get_ranging_data(&dev, &results);
	        if (status == VL53L5CX_STATUS_OK) {
	            printf("Czujnik 2 wysyła dane:\r\n");
	            ProcessData(&results);
	        } else {
	            printf("Błąd odczytu danych z Czujnika 2, kod błędu: %d\r\n", status);
	        }
	    } else {
	        printf("Czujnik 2 nie ma nowych danych do odczytu\r\n");
	    }

	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
	    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);

	    uint32_t current_time = HAL_GetTick();
	    dt = (current_time - prev_time) / 1000.0f;
	    prev_time = current_time;

      // Update_Odometry(&odom, speed_L, speed_R, dt);

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

      HAL_Delay(3000); // Odpowiedni delay, aby nie przeciążać magistrali I2C

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

// Funkcja do przetwarzania danych z czujnika
void ProcessData(VL53L5CX_ResultsData *results) {
    char buffer[1024];

    for (int i = 0; i < 64; i++) {
        sprintf(buffer, "%d ", results->distance_mm[i]);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    }

    // Dodanie nowej linii, aby oznaczyć koniec macierzy
    sprintf(buffer, "\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}


// Funkcja do inicjalizacji czujnika
void ScanAndInitializeSensors(void) {
    printf("Rozpoczynanie skanowania i inicjalizacji czujników I2C...\r\n");
    HAL_StatusTypeDef result;
    uint8_t is_alive;
    uint8_t status;

    // Skanowanie w poszukiwaniu urządzeń I2C
    for (uint8_t i = 1; i < 128; i++) {
        result = HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(i << 1), 1, 10);
        if (result == HAL_OK) {
            printf("Urządzenie znalezione pod adresem: 0x%02X\r\n", i);
        }
    }

    // Inicjalizacja pierwszego czujnika z rozdzielczością 8x8
    dev.platform.address = DEFAULT_SENSOR_1_ADDR;
    status = vl53l5cx_is_alive(&dev, &is_alive);
    if (status == VL53L5CX_STATUS_OK && is_alive) {
        vl53l5cx_init(&dev);
        vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_8X8);  // Ustawienie rozdzielczości 8x8
        printf("Pierwszy czujnik zainicjalizowany na adresie 0x%02X z rozdzielczością 8x8\r\n", DEFAULT_SENSOR_1_ADDR);
    } else {
        printf("Błąd inicjalizacji pierwszego czujnika na adresie 0x%02X\r\n", DEFAULT_SENSOR_1_ADDR);
    }

    // Inicjalizacja drugiego czujnika z rozdzielczością 8x8
    dev.platform.address = DEFAULT_SENSOR_2_ADDR;
    status = vl53l5cx_is_alive(&dev, &is_alive);
    if (status == VL53L5CX_STATUS_OK && is_alive) {
        vl53l5cx_init(&dev);
        vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_8X8);  // Ustawienie rozdzielczości 8x8
        printf("Drugi czujnik zainicjalizowany na adresie 0x%02X z rozdzielczością 8x8\r\n", DEFAULT_SENSOR_2_ADDR);
    } else {
        printf("Błąd inicjalizacji drugiego czujnika na adresie 0x%02X\r\n", DEFAULT_SENSOR_2_ADDR);
    }

    printf("Skanowanie i inicjalizacja czujników zakończona.\r\n");
}



// Funkcja do skanowania magistrali I2C
void Scan_I2C_Devices(void) {
    printf("Skanowanie urządzeń I2C...\r\n");
    HAL_StatusTypeDef result;
    uint8_t foundDevices = 0;

    // Przeskanuj wszystkie możliwe adresy (1-127)
    for (uint8_t i = 1; i < 128; i++) {
        result = HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(i << 1), 1, 10);
        if (result == HAL_OK) {
            printf("Urządzenie znalezione pod adresem: 0x%02X\r\n", i);
            foundDevices++;
        }
    }

    // Sprawdź, czy jakiekolwiek urządzenia zostały znalezione
    if (foundDevices == 0) {
        printf("Nie znaleziono żadnych urządzeń I2C na magistrali.\r\n");
    } else {
        printf("Skanowanie zakończone: znaleziono %d urządzeń.\r\n", foundDevices);
    }

    HAL_Delay(1000);  // Krótkie opóźnienie przed kontynuacją
}


void Initialize_Two_Sensors(void) {
    uint8_t status, is_alive;

    printf("Inicjalizacja pierwszego czujnika na adresie: 0x%02X\r\n", sensor1_addr);
    dev.platform.address = sensor1_addr;
    status = vl53l5cx_is_alive(&dev, &is_alive);
    if (status == VL53L5CX_STATUS_OK && is_alive) {
        vl53l5cx_init(&dev);
        printf("Pierwszy czujnik zainicjalizowany na 0x%02X\r\n", sensor1_addr);
    } else {
        printf("Błąd inicjalizacji pierwszego czujnika na 0x%02X\r\n", sensor1_addr);
    }

    printf("Inicjalizacja drugiego czujnika na adresie: 0x%02X\r\n", sensor2_addr);
    dev.platform.address = sensor2_addr;
    status = vl53l5cx_is_alive(&dev, &is_alive);
    if (status == VL53L5CX_STATUS_OK && is_alive) {
        vl53l5cx_init(&dev);
        printf("Drugi czujnik zainicjalizowany na 0x%02X\r\n", sensor2_addr);
    } else {
        printf("Błąd inicjalizacji drugiego czujnika na 0x%02X\r\n", sensor2_addr);
    }
}

void I2C_ResetBus(void) {
    // Set the reset pin low to disable devices
    HAL_GPIO_WritePin(I2C_RESET_PORT, I2C_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);  // Small delay to ensure reset is registered

    // Set the reset pin high to enable devices
    HAL_GPIO_WritePin(I2C_RESET_PORT, I2C_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);  // Give devices time to reinitialize
}






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
