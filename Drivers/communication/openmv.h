#ifndef __OPENMV_H
#define __OPENMV_H

// OpenMV 三轴数据结构
typedef struct {
    int16_t x;  // X轴坐标
    int16_t y;  // Y轴坐标
    int16_t z;  // Z轴数据 (深度/置信度等)
} OpenMV_Data;
void OPENMV_Init(void);
void OpenMV_GetData(OpenMV_Data* data);
#endif
