#ifndef __BLUEBOOTH_H
#define __BLUEBOOTH_H


#include "stm32f4xx_hal.h"                  // Device header
#include "FreeRTOS.h" 
#include "queue.h"
#include "sensors.h"
void bluetooth_Init(void);
void Bluetooth_SendSensorData(const SensorData_t* sensorData);

#endif
