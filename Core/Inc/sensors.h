/*
 * sensors.h
 *
 *  Created on: Dec 7, 2024
 *      Author: oliwerfigura
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "main.h"
#include "vl53l5cx_api.h"

#define MAX_SENSORS 3

typedef struct {
    I2C_HandleTypeDef *i2c_handle;
    uint16_t address;
    uint8_t found;
} SensorInfo;

extern VL53L5CX_Configuration dev1, dev2, dev3;
extern VL53L5CX_ResultsData results1, results2, results3;
extern SensorInfo g_sensors[MAX_SENSORS];
extern uint8_t g_num_sensors_found;



void Scan_I2C_Devices(void);
void init_sensors(void);
void read_sensors(void);
void ProcessData(VL53L5CX_ResultsData *results);

#endif
