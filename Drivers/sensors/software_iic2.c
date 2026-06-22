#include "stm32f4xx_hal.h"                  // Device header
#include "stdio.h"
#include "software_iic2.h"
#include "gpio.h"

void I2C2_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // SCL - PA4
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 开漏输出
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // SDA - PA5
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 初始状态拉高
    I2C2_SCL_H;
    I2C2_SDA_H;
}


/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_delay(void)
{
		
   uint8_t i=50; //这里可以优化速度	，经测试最低到5还能写入
   while(i) 
   { 
     i--; 
   }  
}

void I2C2_delay5ms(void)
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
int I2C2_Start(void)
{
	I2C2_SDA_H;
	I2C2_SCL_H;
	I2C2_delay();
	if(!HAL_GPIO_ReadPin(GPIOA,  GPIO_PIN_5))return 0;	//SDA线为低电平则总线忙,退出
	I2C2_SDA_L;
	I2C2_delay();
	if(HAL_GPIO_ReadPin(GPIOA,  GPIO_PIN_5)) return 0;	//SDA线为高电平则总线出错,退出
	I2C2_SDA_L;
	I2C2_delay();
	return 1;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_Stop(void)
{
	I2C2_SCL_L;
	I2C2_delay();
	I2C2_SDA_L;
	I2C2_delay();
	I2C2_SCL_H;
	I2C2_delay();
	I2C2_SDA_H;
	I2C2_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_Ack(void)		//主机向从机发送应答信号
{	
	I2C2_SCL_L;
	I2C2_delay();
	I2C2_SDA_L;
	I2C2_delay();
	I2C2_SCL_H;
	I2C2_delay();
	I2C2_SCL_L;
	I2C2_delay();
	I2C2_SDA_H;
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_NoAck(void)		//主机在读取完一个字节后，发送一个NACK信号，告诉从机不需要再发送数据
{	
	I2C2_SCL_L;
	I2C2_delay();
	I2C2_SDA_H;
	I2C2_delay();
	I2C2_SCL_H;
	I2C2_delay();
	I2C2_SCL_L;
	I2C2_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
int I2C2_WaitAck(void) 	 //返回为:=0有ACK,=1无ACK
{
	I2C2_SCL_L;
	I2C2_delay();
	I2C2_SDA_H;			
	I2C2_delay();
	I2C2_SCL_H;
	I2C2_delay();
	if(HAL_GPIO_ReadPin(GPIOA,  GPIO_PIN_5))
	{
      I2C2_SCL_L;
	  I2C2_delay();
      return 1;
	}
	I2C2_SCL_L;
	I2C2_delay();
	return 0;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
uint8_t I2C2_SendByte(uint8_t SendByte) //数据从高位到低位//
{
    uint8_t i=8;
    while(i--)
    {
        I2C2_SCL_L;
        I2C2_delay();
      if(SendByte&0x80)
        I2C2_SDA_H;  
      else 
        I2C2_SDA_L;   
        
      SendByte<<=1;
        I2C2_delay();
		I2C2_SCL_H;
        I2C2_delay();
    }
    I2C2_SCL_L;
	return 0; // 发送成功
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C2_RadeByte(void)  //数据从高位到低位//
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    I2C2_SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      I2C2_SCL_L;
      I2C2_delay();
	  I2C2_SCL_H;
      I2C2_delay();	
      if(HAL_GPIO_ReadPin(GPIOA,  GPIO_PIN_5))
      {
        ReceiveByte|=0x01;
      }
    }
    I2C2_SCL_L;
    return ReceiveByte;
} 
uint8_t I2C2_WaitAck_Timeout(uint32_t timeout)
{
    uint32_t start = HAL_GetTick();
    
    // 临时配置SDA为输入模式
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5; // 您的SDA是PA5
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 产生时钟脉冲
    I2C2_SCL_H;
    I2C2_delay();
    
    // 等待ACK响应
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)) {
        if(HAL_GetTick() - start > timeout) {
            // 超时后恢复SDA为输出模式
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
            return 1; // 超时错误
        }
    }
    
    I2C2_SCL_L;
    
    // 恢复SDA为输出模式
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    return 0; // 成功
}
uint8_t I2C2_SendByte_Timeout(uint8_t data, uint32_t timeout)
{
    for(uint8_t i = 0; i < 8; i++) {
        // 设置数据位
        (data & 0x80) ? I2C2_SDA_H : I2C2_SDA_L;
        data <<= 1;
        
        // 时钟脉冲
        I2C2_SCL_H;
        I2C2_delay();
        I2C2_SCL_L;
        I2C2_delay();
    }
    
    // 检查ACK并设置超时
    return I2C2_WaitAck_Timeout(timeout);
}
