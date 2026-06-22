#ifndef __software_iic2_H
#define __software_iic2_H


//************************************++++++++++++++++++++++++++++++++
/*模拟IIC端口输出输入定义*/
#define I2C2_SCL_H         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET) // 拉高
#define I2C2_SCL_L         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET) // 拉低
   

#define I2C2_SDA_H         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET) // 拉高
#define I2C2_SDA_L         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

#define I2C2_SCL_read      HAL_GPIO_ReadPin(GPIOA,  GPIO_PIN_4)
#define I2C2_SDA_read      HAL_GPIO_ReadPin(GPIOA,  GPIO_PIN_5)

void I2C2_GPIO_Init(void);
int I2C2_Start(void);
void I2C2_Stop(void);
void I2C2_Ack(void);
void I2C2_NoAck(void);
int I2C2_WaitAck(void); 	 //返回为:=1有ACK,=0无ACK
uint8_t I2C2_SendByte(uint8_t SendByte); //数据从高位到低位//
unsigned char I2C2_RadeByte(void);  //数据从高位到低位//



#endif
