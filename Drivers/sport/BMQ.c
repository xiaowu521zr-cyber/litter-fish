#include "stm32f4xx_hal.h"                  // Device header
#include "usart.h"
#include "BMQ.h"
#include "stdio.h"
#include "stdlib.h"

uint8_t Set_zero[8];//置0缓存区
uint8_t T_angle[8];//读取角度缓存区
uint8_t T_cnt[8];//读取圈数缓存区
uint8_t R_angle[8];//读取角度缓存区
uint8_t R_cnt[8];//读取圈数缓存区
uint8_t uart4_Rx_index = 0;               // 缓冲区索引
uint8_t BMQData[8];
//编码器代码
   
void ProcessEncoderData(void) 
    { void set_zero (void );
		HAL_Delay(100);
	  void re_angle (void );
		  if (uart4_Rx_index < 7) 
		  {
            uart4_Rx_index++;
            
            // 判断是否收到数据
            if (T_cnt[0] == 0x01&&T_cnt[2] != 0 && T_cnt[4]!= 0 )
			   {
				for (int i = 3; i < 7; i++)
				{
                  BMQData[i+1] = T_cnt[i];
				  T_cnt[i]=0;
                 }   
                
			   }
           } 
		   else
		  {
            // 重置
            uart4_Rx_index = 0;
          }
		  uart4_Rx_index = 0;
		HAL_Delay(300);
        void re_cnt (void);
			 if (uart4_Rx_index < 7) 
		  {
            uart4_Rx_index++;
            
            // 判断是否收到数据
            if (R_cnt[0] == 0x01&&R_cnt[2] != 0 && R_cnt[4]!= 0 )
			   {
				for (int i =3; i < 7; i++)
				{
                  BMQData[i-3] = R_cnt[i];
				  R_cnt[i]=0;
                 }   
                
			   }
           } 
		   else
		  {
            // 重置
            uart4_Rx_index = 0;
          }
    uint32_t Cnt[4];  // 前四个字节
    uint32_t _Angle[4];   // 后四个字节
    
    // 转换前四个元素
    for (int i = 0; i < 4; i++) {
        Cnt[i] = (uint32_t)BMQData[i];
    }
    
    // 转换后四个元素
    for (int i = 0; i < 4; i++) {
       _Angle[i] = (uint32_t)BMQData[i + 4];
    }
    
    // 输出结果
    printf("前四个元素转换结果：\n");
    for (int i = 0; i < 4; i++) {
        printf("Cnt[%d] = %u (十六进制: 0x%02X)\n", i, Cnt[i], BMQData[i]);
    }
    
    printf("\n后四个元素转换结果：\n");
    for (int i = 0; i < 4; i++) {
        printf("_Angle[%d] = %u (十六进制: 0x%02X)\n", i, _Angle[i], BMQData[i + 4]);
    
    
	}

}	
void set_zero (void)
    {uint8_t Set_zero[8]={0x01, 0x06, 0x00, 0x0B,0x00, 0x01, 0x39, 0xC8};
     HAL_UART_Transmit_IT(&huart4,  Set_zero, 8);
	 
	}
void re_angle (void)
    {uint8_t T_angle[8]={0x01, 0x03, 0x00, 0x0D,0x00, 0x01, 0x15, 0xC9};
     HAL_UART_Transmit_IT(&huart4,T_angle, 8);
	 HAL_UART_Receive_IT(&huart4, &T_angle[8], 8);
	}
void re_cnt (void)
    {uint8_t T_cnt[8]={0x01, 0x03, 0x00, 0x0E,0x00, 0x01, 0xE5, 0xC9};
	 HAL_UART_Transmit_IT(&huart4, T_cnt, 8);
	 HAL_UART_Receive_IT(&huart4, &T_cnt[8], 8);
	}
void BMQ_Init(void)
{
	MX_UART4_Init();
	printf("BMQ is started!\r\n");
}
