#ifndef __sensors_H
#define __sensors_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "openmv.h"
// sensors.h
typedef struct {
    float roll, pitch, yaw;    // IMU角度
    float wx, wy, wz;           // IMU角速度
    double temperature;         // 温度
    double pressure;            // 压力
    double depth;               // 深度
    uint32_t bus_voltage;       // 总线电压
    uint32_t current;           // 电流
    uint32_t power;             // 功率
	OpenMV_Data openmv;
} SensorData_t;
extern SensorData_t g_sensor_data;//定义全局传感器数据
void Sensors_Init(void);
void Sensor_ReadAll(SensorData_t* data); 
#endif
