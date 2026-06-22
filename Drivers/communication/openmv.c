#include "stm32f4xx_hal.h"                  // Device header
#include "openmv.h"
#include "stdio.h"
#include "usart.h"
#include <string.h>
#include "FreeRTOS.h" 
#include "semphr.h"
#include "task.h"

void OPENMV_Init(void)
{
	MX_USART1_UART_Init();
	printf("OpenMV initialized\n");
}
void OpenMV_GetData(OpenMV_Data* data)
{
    // 接收数据 (OpenMV 连续输出)
    char buffer[32] = {0};
    HAL_UART_Receive(&huart1, (uint8_t*)buffer, sizeof(buffer)-1, 500); // 500ms超时
    
    // 解析数据
    if (sscanf(buffer, "X:%hd,Y:%hd,Z:%hd", 
              &data->x, &data->y, &data->z) != 3) {
        // 解析失败，设置默认值
        data->x = -1;
        data->y = -1;
        data->z = -1;
    }
}
