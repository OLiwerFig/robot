/*
 * sensors.c
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */

#include "sensors.h"
#include <stdio.h>
#include <string.h>
#include "vl53l5cx_api.h"
#include "platform.h"
#include "main.h"


VL53L5CX_Configuration dev1, dev2, dev3;
VL53L5CX_ResultsData results1, results2, results3;
SensorInfo g_sensors[MAX_SENSORS];
uint8_t g_num_sensors_found = 0;
extern UART_HandleTypeDef huart2;



// Funkcja odczytu danych z czujników
void read_sensors(void) {
    uint8_t status;
    uint8_t isReady;
    char buffer[512];
    int offset;

    // Odczyt z czujnika 1 (oznaczenie "A")
    isReady = 0;
    status = vl53l5cx_check_data_ready(&dev1, &isReady);
    if (isReady) {
        status = vl53l5cx_get_ranging_data(&dev1, &results1);
        if (status == VL53L5CX_STATUS_OK) {
            // Wysyłamy literę 'A' i nową linię
            HAL_UART_Transmit(&huart2, (uint8_t *)"A\n", 2, HAL_MAX_DELAY);

            offset = 0;
            for (int i = 0; i < 64; i++) {
                offset += sprintf(buffer + offset, "%d ", results1.distance_mm[i]);
            }
            offset += sprintf(buffer + offset, "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, offset, HAL_MAX_DELAY);

            // Ponowne uruchomienie pomiaru
            vl53l5cx_start_ranging(&dev1);
        }
    }

    // Odczyt z czujnika 2 (oznaczenie "B")
    isReady = 0;
    status = vl53l5cx_check_data_ready(&dev2, &isReady);
    if (isReady) {
        status = vl53l5cx_get_ranging_data(&dev2, &results2);
        if (status == VL53L5CX_STATUS_OK) {
            // Wysyłamy literę 'B' i nową linię
            HAL_UART_Transmit(&huart2, (uint8_t *)"B\n", 2, HAL_MAX_DELAY);

            offset = 0;
            for (int i = 0; i < 64; i++) {
                offset += sprintf(buffer + offset, "%d ", results2.distance_mm[i]);
            }
            offset += sprintf(buffer + offset, "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, offset, HAL_MAX_DELAY);

            // Ponowne uruchomienie pomiaru
            vl53l5cx_start_ranging(&dev2);
        }
    }

    // Odczyt z czujnika 3 (oznaczenie "C")
    isReady = 0;
    status = vl53l5cx_check_data_ready(&dev3, &isReady);
    if (isReady) {
        status = vl53l5cx_get_ranging_data(&dev3, &results3);
        if (status == VL53L5CX_STATUS_OK) {
            // Wysyłamy literę 'C' i nową linię
            HAL_UART_Transmit(&huart2, (uint8_t *)"C\n", 2, HAL_MAX_DELAY);

            offset = 0;
            for (int i = 0; i < 64; i++) {
                offset += sprintf(buffer + offset, "%d ", results3.distance_mm[i]);
            }
            offset += sprintf(buffer + offset, "\r\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, offset, HAL_MAX_DELAY);

            // Ponowne uruchomienie pomiaru
            vl53l5cx_start_ranging(&dev3);
        }
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

