#include "platform.h"
#include "stm32l4xx_hal.h"
#include <string.h>

#define I2C_TIMEOUT 5000 // Zwiększony timeout dla bezpieczeństwa

uint8_t VL53L5CX_RdByte(
    VL53L5CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_value)
{
    // Upewnij się, że p_platform->i2c_handle i p_platform->address są ustawione
    if (p_platform->i2c_handle == NULL) return 1; // Błąd, brak uchwytu I2C

    uint8_t status;
    status = HAL_I2C_Mem_Read(p_platform->i2c_handle, p_platform->address, RegisterAddress,
                              I2C_MEMADD_SIZE_16BIT, p_value, 1, I2C_TIMEOUT);
    return (status == HAL_OK) ? 0 : 1;
}

uint8_t VL53L5CX_WrByte(
    VL53L5CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t value)
{
    if (p_platform->i2c_handle == NULL) return 1;

    uint8_t status;
    status = HAL_I2C_Mem_Write(p_platform->i2c_handle, p_platform->address, RegisterAddress,
                               I2C_MEMADD_SIZE_16BIT, &value, 1, I2C_TIMEOUT);
    return (status == HAL_OK) ? 0 : 1;
}

uint8_t VL53L5CX_WrMulti(
    VL53L5CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_values,
    uint32_t size)
{
    if (p_platform->i2c_handle == NULL) return 1;

    uint8_t status;
    status = HAL_I2C_Mem_Write(p_platform->i2c_handle, p_platform->address, RegisterAddress,
                               I2C_MEMADD_SIZE_16BIT, p_values, size, I2C_TIMEOUT);
    return (status == HAL_OK) ? 0 : 1;
}

uint8_t VL53L5CX_RdMulti(
    VL53L5CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_values,
    uint32_t size)
{
    if (p_platform->i2c_handle == NULL) return 1;

    uint8_t status;
    status = HAL_I2C_Mem_Read(p_platform->i2c_handle, p_platform->address, RegisterAddress,
                              I2C_MEMADD_SIZE_16BIT, p_values, size, I2C_TIMEOUT);
    return (status == HAL_OK) ? 0 : 1;
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform)
{
    // Załóżmy, że masz pin resetu czujnika podłączony do GPIO i chcesz go zresetować.
    // Jeśli nie masz, zostaw to jako prosty delay.

    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    // VL53L5CX_WaitMs(p_platform, 100);
    // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    // VL53L5CX_WaitMs(p_platform, 100);

    // Jeśli brak fizycznego resetu, zwróć po prostu 0:
    return 0;
}

void VL53L5CX_SwapBuffer(
    uint8_t         *buffer,
    uint16_t          size)
{
    uint32_t i, tmp;

    for(i = 0; i < size; i = i + 4)
    {
        tmp = (
          buffer[i]<<24)
        |(buffer[i+1]<<16)
        |(buffer[i+2]<<8)
        |(buffer[i+3]);

        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t VL53L5CX_WaitMs(
    VL53L5CX_Platform *p_platform,
    uint32_t TimeMs)
{
    HAL_Delay(TimeMs);
    return 0;
}
