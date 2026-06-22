#include "stm32f4xx_hal.h"                  // Device header
#include "bluebooth.h"
#include "stdio.h"
#include "usart.h"
#include <string.h>
#include "sensors.h"
#include "FreeRTOS.h" 
#include "semphr.h"
#include "task.h"
void bluetooth_Init(void)
{
	printf("bluetooth started!");
}
void Bluetooth_SendSensorData(const SensorData_t* sensorData)
{
    char buffer[512]; // 更大的缓冲区
    
    //所有传感器数据
    sprintf(buffer, 
        "IMU: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°, Wx=%.2f°/s, Wy=%.2f°/s, Wz=%.2f°/s\n"
        "Depth: Depth=%.2fm, Temp=%.2fC, Pressure=%.2fkPa\n"
        "Power: Voltage=%umV, Current=%umA, Power=%umW\n\n",
        sensorData->roll, sensorData->pitch, sensorData->yaw,
        sensorData->wx, sensorData->wy, sensorData->wz,
        sensorData->depth, sensorData->temperature, sensorData->pressure,
        sensorData->bus_voltage, sensorData->current, sensorData->power);
    
    // 通过蓝牙发送数据
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
