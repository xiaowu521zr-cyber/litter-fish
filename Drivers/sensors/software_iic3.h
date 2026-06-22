#ifndef __software_iic3_H
#define __software_iic3_H


//************************************++++++++++++++++++++++++++++++++
/*模拟IIC端口输出输入定义*/
#define I2C_3_SCL_H         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET) // 拉高
#define I2C_3_SCL_L         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET) // 拉低
   

#define I2C_3_SDA_H         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET) // 拉高
#define I2C_3_SDA_L         HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)

#define I2C_3_SCL_read      HAL_GPIO_ReadPin(GPIOA,  GPIO_PIN_8)
#define I2C_3_SDA_read      HAL_GPIO_ReadPin(GPIOC,  GPIO_PIN_9)

void I2C3_GPIO_Init(void);
int I2C3_Start(void);
void I2C3_Stop(void);
void I2C3_Ack(void);
void I2C3_NoAck(void);
int I2C3_WaitAck(void); 	 //返回为:=1有ACK,=0无ACK
uint8_t I2C3_SendByte(uint8_t SendByte); //数据从高位到低位//
unsigned char I2C3_RadeByte(void);  //数据从高位到低位//



#endif
