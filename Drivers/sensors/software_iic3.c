#include "stm32f4xx_hal.h"                  // Device header
#include "stdio.h"
#include "software_iic3.h"
#include "gpio.h"

void I2C3_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // SCL - PA8
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 开漏输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // SDA - PC9
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    
    // 初始状态拉高
    I2C_3_SCL_H;
    I2C_3_SDA_H;
}


/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C3_delay(void)
{
		
   uint8_t i=50; //这里可以优化速度	，经测试最低到5还能写入
   while(i) 
   { 
     i--; 
   }  
}

void I2C3_delay5ms(void)
{
		
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
int I2C3_Start(void)
{
	I2C_3_SDA_H;
	I2C_3_SCL_H;
	I2C3_delay();
	if(!HAL_GPIO_ReadPin(GPIOC,  GPIO_PIN_9))return 0;	//SDA线为低电平则总线忙,退出
	I2C_3_SDA_L;
	I2C3_delay();
	if(HAL_GPIO_ReadPin(GPIOC,  GPIO_PIN_9)) return 0;	//SDA线为高电平则总线出错,退出
	I2C_3_SDA_L;
	I2C3_delay();
	return 1;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C3_Stop(void)
{
	I2C_3_SCL_L;
	I2C3_delay();
	I2C_3_SDA_L;
	I2C3_delay();
	I2C_3_SCL_H;
	I2C3_delay();
	I2C_3_SDA_H;
	I2C3_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C3_Ack(void)
{	
	I2C_3_SCL_L;
	I2C3_delay();
	I2C_3_SDA_L;
	I2C3_delay();
	I2C_3_SCL_H;
	I2C3_delay();
	I2C_3_SCL_L;
	I2C3_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C3_NoAck(void)
{	
	I2C_3_SCL_L;
	I2C3_delay();
	I2C_3_SDA_H;
	I2C3_delay();
	I2C_3_SCL_H;
	I2C3_delay();
	I2C_3_SCL_L;
	I2C3_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
int I2C3_WaitAck(void) 	 //返回为:=0有ACK,=1无ACK
{
	I2C_3_SCL_L;
	I2C3_delay();
	I2C_3_SDA_H;			
	I2C3_delay();
	I2C_3_SCL_H;
	I2C3_delay();
	if(HAL_GPIO_ReadPin(GPIOC,  GPIO_PIN_9))
	{
      I2C_3_SCL_L;
	  I2C3_delay();
      return 1;
	}
	I2C_3_SCL_L;
	I2C3_delay();
	return 0;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
uint8_t I2C3_SendByte(uint8_t SendByte) //数据从高位到低位//
{
    uint8_t i=8;
    while(i--)
    {
        I2C_3_SCL_L;
        I2C3_delay();
      if(SendByte&0x80)
        I2C_3_SDA_H;  
      else 
        I2C_3_SDA_L;   
        
      SendByte<<=1;
        I2C3_delay();
		I2C_3_SCL_H;
        I2C3_delay();
    }
    I2C_3_SCL_L;
	return 0; // 发送成功
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C3_RadeByte(void)  //数据从高位到低位//
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    I2C_3_SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      I2C_3_SCL_L;
      I2C3_delay();
	  I2C_3_SCL_H;
      I2C3_delay();	
      if(HAL_GPIO_ReadPin(GPIOC,  GPIO_PIN_9))
      {
        ReceiveByte|=0x01;
      }
    }
    I2C_3_SCL_L;
    return ReceiveByte;
} 
