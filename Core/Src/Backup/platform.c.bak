#include "platform.h"
#include "i2c.h"
#include "stm32l4xx_hal.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c3;

#define I2C_TIMEOUT 5000 // Zwiększony timeout

uint8_t VL53L5CX_RdByte(
    VL53L5CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_value)
{
    uint8_t status;
    status = HAL_I2C_Mem_Read(&hi2c3, p_platform->address, RegisterAddress, I2C_MEMADD_SIZE_16BIT, p_value, 1, I2C_TIMEOUT);
    return status;
}

uint8_t VL53L5CX_WrByte(
    VL53L5CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t value)
{
    uint8_t status;
    status = HAL_I2C_Mem_Write(&hi2c3, p_platform->address, RegisterAddress, I2C_MEMADD_SIZE_16BIT, &value, 1, I2C_TIMEOUT);
    return status;
}

uint8_t VL53L5CX_WrMulti(
    VL53L5CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_values,
    uint32_t size)
{
    uint8_t status;
    status = HAL_I2C_Mem_Write(&hi2c3, p_platform->address, RegisterAddress, I2C_MEMADD_SIZE_16BIT, p_values, size, I2C_TIMEOUT);
    return status;
}

uint8_t VL53L5CX_RdMulti(
    VL53L5CX_Platform *p_platform,
    uint16_t RegisterAddress,
    uint8_t *p_values,
    uint32_t size)
{
    uint8_t status;
    status = HAL_I2C_Mem_Read(&hi2c3, p_platform->address, RegisterAddress, I2C_MEMADD_SIZE_16BIT, p_values, size, I2C_TIMEOUT);
    return status;
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform)
{
    /* Ustawienie pinu resetu czujnika */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
    VL53L5CX_WaitMs(p_platform, 100);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    VL53L5CX_WaitMs(p_platform, 100);

    return 0;
}

void VL53L5CX_SwapBuffer(
    uint8_t         *buffer,
    uint16_t          size)
{
    uint32_t i, tmp;

    /* Przykład implementacji z użyciem <string.h> */
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
