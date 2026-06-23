#include "stdio.h"
#include "sensors.h"
#include "msp5837.h"
#include "INA226.h"
#include "imu.h"
#include "openmv.h"
#include "usart.h"
#include <string.h>
SensorData_t g_sensor_data;
void Sensors_Init(void) 
{ 
    IMUinit();
    MS5837_Init();
    INA226_Init();
    OPENMV_Init();
    memset(&g_sensor_data,0,sizeof(SensorData_t));//初始化全局传感器数据
	printf("Sensors_Init complete\r\n");
}
void Sensor_ReadAll(SensorData_t* data) 
{
    // 读取IMU
    USART6_Get_IMU();
// 使用新函数获取数据
    float r, p, y, wx, wy, wz;
    GetIMUData(&r, &p, &y, &wx, &wy, &wz);
    data->roll = r;
    data->pitch = p;
    data->yaw = y;
    data->wx = wx;
    data->wy = wy;
    data->wz = wz;  
	g_sensor_data.roll = r;
	g_sensor_data.pitch = p;
	g_sensor_data.yaw = y;
	g_sensor_data.wx = wx;
	g_sensor_data.wy = wy;
	g_sensor_data.wz = wz;
	double temp, pressure;
    MS5837_Getdata(&temp, &pressure); // 调用直接读取函数
    data->temperature = temp;
    data->pressure = pressure;
    data->depth = (pressure - 985.0) / 0.983615f; // 计算深度
	g_sensor_data.temperature = temp;
    g_sensor_data.pressure = pressure;
    g_sensor_data.depth = data->depth;	
    // 读取电源传感器
    data->bus_voltage = INA226_GetBusVoltage();
    data->current = INA226_GetCurrent();
    data->power = INA226_GetPower();
	OpenMV_GetData(&data->openmv);
}
