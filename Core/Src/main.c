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



typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    uint16_t address;
    uint8_t found; // 0 - nie znaleziono, 1 - znaleziono
} SensorInfo;

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


#define MAX_SENSORS 3


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Zmienne globalne
VL53L5CX_Configuration dev1, dev2, dev3;
VL53L5CX_ResultsData results1, results2, results3;


SensorInfo g_sensors[MAX_SENSORS];
uint8_t g_num_sensors_found = 0;

uint8_t uartBuffer[UART_BUFFER_SIZE];



float speed_L_target, speed_R_target, pwm_R, pwm_L, dt;

volatile uint32_t speed_L = 0;
volatile uint32_t speed_R = 0;
volatile uint32_t count = 0;
volatile uint32_t count1 = 0;

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
void init_sensors(void);
void read_sensors(void);
void ProcessData(VL53L5CX_ResultsData *results);
void Update_Odometry(Odometry_TypeDef *odom, float speed_L, float speed_R, float dt);
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float setpoint);
float PID_Compute(PID_TypeDef *pid, float current_value, float dt);
void SetSpeed(PID_TypeDef *pid, float setpoint);
void SetMotorDirection(int direction_L, int direction_R);
void SendDataToQt(Odometry_TypeDef *odom, Target_TypeDef *target ,float pwm_L ,float pwm_R,  float speed_L, float speed_R);
void Odometry_Init(Odometry_TypeDef *odom);

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

        __HAL_TIM_SET_COUNTER(&htim2, 0);
        __HAL_TIM_SET_COUNTER(&htim3, 0);

        speed_L = count;
        speed_R = count1;

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
        flag = (char)Rx_data;
        // Ponownie włącz przerwanie na odbiór następnego znaku
        HAL_UART_Receive_IT(&huart2, &Rx_data, 1);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart2, &Rx_data, 1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);

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

  SetMotorDirection(0,0);

  uint32_t prev_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      read_sensors();

      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 200);
      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 200);
      printf("jaminkkkk \r\n");

      // Aktualizacja odometrii
      uint32_t current_time = HAL_GetTick();
      dt = (current_time - prev_time) / 1000.0f; // Konwersja ms na s
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

      HAL_Delay(10000); // Odpowiedni delay, aby nie przeciążać magistrali I2C

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

// Funkcja odczytu danych z czujników
void read_sensors(void) {
    uint8_t status;
    uint8_t isReady = 0;

    // Czujnik 1
    status = vl53l5cx_check_data_ready(&dev1, &isReady);
    printf("Czujnik 1 - status: %d, isReady: %d \r\n", status, isReady);
    if (isReady) {
        printf("Czujnik 1 jest gotowy, próbuję odczytać dane... \r\n");
        status = vl53l5cx_get_ranging_data(&dev1, &results1);
        if (status == VL53L5CX_STATUS_OK) {
            printf("Czujnik 1 - Odczytano dane pomyślnie.\r\n");
            ProcessData(&results1);
            vl53l5cx_start_ranging(&dev1); // Ponowne uruchomienie pomiaru
        } else {
            printf("Błąd odczytu danych z czujnika 1, status: %d\r\n", status);
        }
    } else {
        printf("Czujnik 1 nie ma gotowych danych.\r\n");
    }

    // Czujnik 2
    status = vl53l5cx_check_data_ready(&dev2, &isReady);
    printf("Czujnik 2 - status: %d, isReady: %d \r\n", status, isReady);
    if (isReady) {
        printf("Czujnik 2 jest gotowy, próbuję odczytać dane... \r\n");
        status = vl53l5cx_get_ranging_data(&dev2, &results2);
        if (status == VL53L5CX_STATUS_OK) {
            printf("Czujnik 2 - Odczytano dane pomyślnie.\r\n");
            ProcessData(&results2);
            vl53l5cx_start_ranging(&dev2); // Ponowne uruchomienie pomiaru
        } else {
            printf("Błąd odczytu danych z czujnika 2, status: %d\r\n", status);
        }
    } else {
        printf("Czujnik 2 nie ma gotowych danych.\r\n");
    }

    // Czujnik 3
    status = vl53l5cx_check_data_ready(&dev3, &isReady);
    printf("Czujnik 3 - status: %d, isReady: %d \r\n", status, isReady);
    if (isReady) {
        printf("Czujnik 3 jest gotowy, próbuję odczytać dane... \r\n");
        status = vl53l5cx_get_ranging_data(&dev3, &results3);
        if (status == VL53L5CX_STATUS_OK) {
            printf("Czujnik 3 - Odczytano dane pomyślnie.\r\n");
            ProcessData(&results3);
            vl53l5cx_start_ranging(&dev3); // Ponowne uruchomienie pomiaru
        } else {
            printf("Błąd odczytu danych z czujnika 3, status: %d\r\n", status);
        }
    } else {
        printf("Czujnik 3 nie ma gotowych danych.\r\n");
    }
}

// Funkcja do przetwarzania danych z czujnika
void ProcessData(VL53L5CX_ResultsData *results) {
    char buffer[256]; // Zmniejszony rozmiar bufora
    int offset = 0;

    for (int i = 0; i < 64; i++) {
        offset += sprintf(buffer + offset, "%d ", results->distance_mm[i]);
        // Jeśli bufor się zapełni, wyślij i zresetuj
        if (offset > sizeof(buffer) - 20) { // Pozostaw miejsce na końcowy znak
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, offset, HAL_MAX_DELAY);
            offset = 0;
        }
    }

    // Dodanie nowej linii, aby oznaczyć koniec macierzy
    offset += sprintf(buffer + offset, "\r \n");
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, offset, HAL_MAX_DELAY);
}


void init_sensors(void) {
    uint8_t status;

    if (g_num_sensors_found == 0) {
        printf("Brak wykrytych czujników do inicjalizacji.\r\n");
        return;
    }

    for (int i = 0; i < g_num_sensors_found && i < 3; i++) {
        if (g_sensors[i].found == 1) {
            VL53L5CX_Configuration* current_dev;

            // Przydział devX w zależności od indexu
            if (i == 0) {
                current_dev = &dev1;
            } else if (i == 1) {
                current_dev = &dev2;
            } else {
                current_dev = &dev3;
            }

            // Ustaw platformę
            current_dev->platform.i2c_handle = g_sensors[i].i2c_handle;
            current_dev->platform.address = g_sensors[i].address;

            // Dodatkowa weryfikacja przed inicjalizacją:
            if (HAL_I2C_IsDeviceReady(current_dev->platform.i2c_handle, current_dev->platform.address, 1, 100) != HAL_OK) {
                printf("Czujnik %d: urządzenie na adresie 0x%X nie odpowiada przed init.\r\n", i+1, current_dev->platform.address);
                g_sensors[i].found = 0; // Oznacz jako niewykryty, skoro nie odpowiada
                continue;
            }

            printf("Inicjalizacja czujnika %d na magistrali %p, adres 0x%X...\r\n", i+1, (void*)current_dev->platform.i2c_handle, current_dev->platform.address);

            // Inicjalizuj czujnik
            status = vl53l5cx_init(current_dev);
            printf("Inicjalizacja czujnika %d, status: %d\r\n", i+1, status);

            if (status == VL53L5CX_STATUS_OK) {
                printf("Czujnik %d zainicjalizowany poprawnie.\r\n", i+1);
                vl53l5cx_set_resolution(current_dev, VL53L5CX_RESOLUTION_8X8);
                vl53l5cx_set_ranging_frequency_hz(current_dev, 15);
                vl53l5cx_set_ranging_mode(current_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);
                vl53l5cx_start_ranging(current_dev);
            } else {
                printf("Błąd inicjalizacji czujnika %d, kod błędu: %d\r\n", i+1, status);
                g_sensors[i].found = 0; // Nie udało się zainicjalizować, oznacz jako niewykryty
            }
        }
    }
}





// Funkcja do skanowania magistrali I2C
void Scan_I2C_Devices() {
    printf("Rozpoczynanie skanowania urządzeń I2C...\r\n");

    I2C_HandleTypeDef* i2c_handles[] = {&hi2c1, &hi2c2, &hi2c3};
    const char* i2c_names[] = {"I2C1", "I2C2", "I2C3"};

    g_num_sensors_found = 0;
    for (int i = 0; i < MAX_SENSORS; i++) {
        g_sensors[i].found = 0;
    }

    for (int i = 0; i < 3; i++) {
        printf("Skanowanie magistrali %s...\r\n", i2c_names[i]);

        for (uint8_t addr = 1; addr < 128; addr++) {
            HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(i2c_handles[i], (uint16_t)(addr << 1), 1, 10);
            if (result == HAL_OK) {
                printf("Urządzenie znalezione na magistrali %s pod adresem: 0x%02X\r\n", i2c_names[i], addr);

                // Sprawdź, czy to adres czujnika VL53L5CX
                // Domyślny adres: 0x29 (7-bit) -> (0x29 << 1) = 0x52 (8-bit)
                if (addr == 0x29 && g_num_sensors_found < MAX_SENSORS) {
                    g_sensors[g_num_sensors_found].i2c_handle = i2c_handles[i];
                    g_sensors[g_num_sensors_found].address = (uint16_t)(0x52); // 8-bitowy adres
                    g_sensors[g_num_sensors_found].found = 1;
                    g_num_sensors_found++;
                }

                HAL_Delay(100); // mały delay
            }
        }
    }

    printf("Skanowanie zakończone na wszystkich magistralach.\r\n");

    // Debug: Wyświetl które czujniki znaleziono
    for (int i = 0; i < g_num_sensors_found; i++) {
        printf("Czujnik %d znaleziony na magistrali z handle: %p, adres: 0x%02X\r\n",
               i+1, (void*)g_sensors[i].i2c_handle, g_sensors[i].address);
    }
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
