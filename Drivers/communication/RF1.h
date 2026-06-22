#ifndef __RF1_H
#define __RF1_H
#include "stm32f4xx_hal.h"                  // Device header
#include "usart.h"    
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sensors.h"  
#include "semphr.h"   
// RF配置参数
#define RF_TX_BUFFER_SIZE 128
#define RF_RX_BUFFER_SIZE 64
#define RF_PACKET_TIMEOUT 50 // ms
#define RF_BAUD_RATE 115200
// RF工作模式
typedef enum {
    RF_MODE_IDLE = 0,
    RF_MODE_TX,
    RF_MODE_RX
} RF_Mode;

// 传感器数据类型标识
typedef enum {
    SENSOR_IMU = 1,
    SENSOR_MS5837 = 2,
    SENSOR_INA226 = 3,
	SENSOR_OPENMV = 4,
	SENSOR_FULL_PACK = 5
} RF_PacketType;

typedef struct {
    float roll, pitch, yaw;    // 角度(度)
    float wx, wy, wz;          // 角速度(度/秒)
} IMU_Data;

typedef struct {
    double depth;              // 深度(m)
    double temperature;        // 温度(℃)
    double pressure;           // 压力(kPa)
} MS5837_Data;

typedef struct {
    uint32_t bus_voltage;      // 总线电压(mV)
    uint32_t current;          // 电流(mA)
    uint32_t power;            // 功率(mW)
} INA226_Data;

#pragma pack(push, 1)
typedef struct {
    int16_t roll;          // 单位0.01度
    int16_t pitch;
    int16_t yaw;
    int16_t wx;            // 单位0.01度/秒
    int16_t wy;
    int16_t wz;
    int16_t depth;         // 单位0.01米
    int16_t temperature;   // 单位0.01℃
    uint16_t pressure;     // 单位0.1 kPa
    uint16_t bus_voltage;  // 单位mV
    uint16_t current;      // 单位mA
    uint16_t power;        // 单位mW
    // 可以添加其他字段
} RF_FullSensorData;
#pragma pack(pop)
// 更新RF数据结构
#pragma pack(push, 1)
typedef struct {
    uint8_t header;        // 固定头 (0xAA)
    RF_PacketType type;    // 数据类型
    union {
//        uint16_t data[6];      // 单传感器数据
        RF_FullSensorData full; // 完整数据包（不能同时使用）
    } payload;
    uint8_t checksum;      // 校验和
} RF_Packet;
#pragma pack(pop)
void RF_Init(void); 
void RF_SendSensorData(RF_PacketType type, const void* sensorData);
void RF_SendFullSensorData(const SensorData_t* fullData); 
extern SemaphoreHandle_t rfMutex;
#endif
