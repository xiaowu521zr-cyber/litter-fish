#include "stm32f4xx_hal.h"                  // Device header
#include "software_iic3.h"
#include "INA226.h"
#include "stdio.h"
void INA226_WriteReg(uint8_t Register, uint8_t Data_H, uint8_t Data_L)
{
	I2C3_Start();						  //I2C起始
	I2C3_SendByte(INA226_W);	  //发送从机地址，读写位为0，表示即将写入
	I2C3_WaitAck();					//接收应答
	I2C3_SendByte(Register);		//发送寄存器地址
	I2C3_WaitAck();					//接收应答
	I2C3_SendByte(Data_H);			//发送要写入寄存器的数据高位
	I2C3_WaitAck();					//接收应答
	I2C3_SendByte(Data_L);			//发送要写入寄存器的数据低位
	I2C3_WaitAck();					//接收应答
	I2C3_Stop();						    //I2C终止
}

uint32_t INA226_ReadReg(uint8_t RegAddress)
{
	uint32_t Data;
	
	I2C3_Start();						  //I2C起始
	I2C3_SendByte(INA226_W);	  //发送从机地址
	I2C3_WaitAck();				//接收应答
	I2C3_SendByte(RegAddress);	//发送寄存器地址
	I2C3_WaitAck();					//接收应答
	
	I2C3_Start();					   	//I2C重复起始
	I2C3_SendByte(INA226_R);	  //发送从机地址
	I2C3_WaitAck();					//接收应答
	Data = I2C3_RadeByte();	//接收指定寄存器的高位数据
	I2C3_Ack();			  	//发送应答
	
	uint8_t low_byte = I2C3_RadeByte(); // 接收低字节 
	I2C3_NoAck();
	I2C3_Stop();						    //I2C终止
	Data = (Data << 8)| low_byte;	//接收指定寄存器的低位数据
	return Data;
}

void INA226_Init(void)
{
	I2C3_GPIO_Init();
	INA226_WriteReg(INA226_Configuration,Configuration_H,Configuration_L);   //4次平均  1.1ms转换时间  连续检测
	INA226_WriteReg(INA226_Calibration,Calibration_H,Calibration_L);         //基准值 0x0200
}

uint32_t INA226_GetShuntVoltage(void)//分流电压值 =  寄存器值 * LSB(2.5uA)
{
	uint32_t ShuntVoltage;
	ShuntVoltage = (uint32_t)((INA226_ReadReg(INA226_Shuntvoltage)) * 2.5 / 1000);
	
	return ShuntVoltage;
}

uint32_t INA226_GetBusVoltage(void)//总线电压值 =  寄存器值 * LSB(1.25mV)
{
	uint32_t BusVoltage;
	BusVoltage = (uint32_t)((INA226_ReadReg(INA226_Busvoltage)) * 1.25);
	
	return BusVoltage;
}

uint32_t INA226_GetCurrent(void)//电流值 = 寄存器值 * Current_LSB(0.05mA)
{
	uint32_t Current;
	Current = (uint32_t)((INA226_ReadReg(INA226_Current)) * 0.05);
	
	return Current;
}

uint32_t INA226_GetPower(void)//功率 = 寄存器值 * Power_LSB(1.25mW)
{
	uint32_t Power;
	Power = (uint32_t)((INA226_ReadReg(INA226_Power)) * 1.25);
	
	return Power;
}
