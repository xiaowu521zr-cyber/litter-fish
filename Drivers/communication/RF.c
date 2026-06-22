#include "stm32f4xx_hal.h"                  // Device header
#include "RF1.h"
#include "stdio.h"
#include "usart.h"
#include <string.h>
#include "sensors.h"
#include "FreeRTOS.h" 
#include "semphr.h"
#include "task.h"
// 初始化互斥量
SemaphoreHandle_t rfMutex = NULL;

void RF_Init(void) 
{
    MX_USART2_UART_Init();
    // 创建互斥量
    rfMutex = xSemaphoreCreateMutex();
    printf("RF模块初始化完成\n");
}
void RF_SendSensorData(RF_PacketType type, const void* sensorData)
{
    if(xSemaphoreTake(rfMutex, portMAX_DELAY) == pdTRUE) {
        RF_Packet packet;
        packet.header = 0xAA;
        packet.type = type;
        
        if (type == SENSOR_FULL_PACK) {
            const SensorData_t* fullData = (const SensorData_t*)sensorData;
            
            // 填充完整数据包
            packet.payload.full.roll = (int16_t)(fullData->roll * 100);
            packet.payload.full.pitch = (int16_t)(fullData->pitch * 100);
            packet.payload.full.yaw = (int16_t)(fullData->yaw * 100);
            packet.payload.full.wx = (int16_t)(fullData->wx * 100);
            packet.payload.full.wy = (int16_t)(fullData->wy * 100);
            packet.payload.full.wz = (int16_t)(fullData->wz * 100);
            packet.payload.full.depth = (int16_t)(fullData->depth * 100);
            packet.payload.full.temperature = (int16_t)(fullData->temperature * 100);
            packet.payload.full.pressure = (uint16_t)(fullData->pressure * 10);
            packet.payload.full.bus_voltage = (uint16_t)fullData->bus_voltage;
            packet.payload.full.current = (uint16_t)fullData->current;
            packet.payload.full.power = (uint16_t)fullData->power;
        }            
		else{
                xSemaphoreGive(rfMutex);
                return;
        }

        // 计算校验和
        packet.checksum = 0;
        uint8_t* p = (uint8_t*)&packet;
        for(int i = 0; i < sizeof(packet) - 1; i++) {
            packet.checksum ^= p[i];
        }

        // 发送数据包
        HAL_UART_Transmit(&huart2, (uint8_t*)&packet, sizeof(packet), 100);
        
        xSemaphoreGive(rfMutex);
    }
}
void RF_SendFullSensorData(const SensorData_t* fullData) 
{
    RF_SendSensorData(SENSOR_FULL_PACK, fullData);
}
