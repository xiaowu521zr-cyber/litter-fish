#include "stm32f4xx_hal.h"                  // Device header
#include "software_iic2.h"
#include "msp5837.h"
#include "stdio.h"

//typedef enum 
//{
//   MS5837_TEMP_START,
//   MS5837_TEMP_CONVERSION,
//   MS5837_PRE_START,
//   MS5837_PRE_CONVERSION,
//   MS5837_PRE_FINISH
//}MS5837_State;

//MS5837_State init_state = MS5837_TEMP_START;

uint32_t Cal_C[7]; //用于存放PROM中的6组数据C1-C6
double OFF_;
float Aux;
/*
dT 实际和参考温度之间的差异
Temperature 实际温度	
*/
uint64_t dT, Temperature;

/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/
uint64_t SENS;
uint32_t D1_Pres, D2_Temp;		 // 数字压力值,数字温度值
uint32_t TEMP2, T2, OFF2, SENS2; //温度校验值

uint32_t Pressure;			  //气压
uint32_t Depth;
float Atmdsphere_Pressure; //大气压

void MS583702BA_RESET(void)
{
	I2C2_Start();
	I2C2_SendByte(0xEC); //CSB接地，主机地址：0XEE，否则 0X77
	I2C2_WaitAck();
	I2C2_SendByte(0x1E); //发送复位命令
	I2C2_WaitAck();
	I2C2_Stop();
}

void MS5837_Init(void)
{
	printf("MS5837_Init: Starting\r\n"); 
	I2C2_GPIO_Init();
	uint8_t inth, intl;
	uint8_t i;
	MS583702BA_RESET(); // Reset Device  复位MS5837
	HAL_Delay(40);		//复位后延时（注意这个延时是一定必要的，可以缩短但似乎不能少于20ms）
	printf("MS5837_Init: Reset done\r\n");
	for (i = 1; i <= 6; i++)
	{
		I2C2_Start();
		I2C2_SendByte(0xEC);
		I2C2_WaitAck();
		I2C2_SendByte(0xA0 + (i * 2));
		I2C2_WaitAck();
		I2C2_Stop();
		HAL_Delay(5);
		I2C2_Start();
		I2C2_SendByte(0xEC + 0x01); //进入接收模式
		HAL_Delay(1);
		I2C2_WaitAck();
		inth = I2C2_RadeByte(); //带ACK的读数据
		I2C2_Ack();
		HAL_Delay(1);
		intl = I2C2_RadeByte(); //最后一个字节NACK
		I2C2_NoAck();
		I2C2_Stop();
		Cal_C[i] = (((uint16_t)inth << 8) | intl);
   }  
	printf("MS5837_Init: Done\r\n");
}

/**
  * 函    数：ms5837温度数据转换开始
  * 参    数：无
  * 返 回 值：0:失败 1：成功
  */
uint8_t MS5837_Temp_Start(void)
{
   I2C2_Start();
	I2C2_SendByte(0xEC); //写地址
	I2C2_WaitAck();
	I2C2_SendByte(0x58); //写转换命令
	I2C2_WaitAck();
	I2C2_Stop();
   
   return 1;
}

/**
  * 函    数：ms5837压力数据转换开始
  * 参    数：无
  * 返 回 值：0:失败 1：成功
  */
uint8_t MS5837_Pre_Start(void)
{
	I2C2_Start();
	I2C2_SendByte(0xEC); //写地址
	I2C2_WaitAck();
	I2C2_SendByte(0x48); //写转换命令
	I2C2_WaitAck();
	I2C2_Stop();
   
   return 1;
}

/**
  * 函    数：温度原始数据读取
  * 参    数：无
  * 返 回 值：温度原始数据
  */
uint32_t MS5837_Temp_Conversion(void)
{
   uint32_t  conversion = 0;
   uint8_t temp[3];
   I2C2_Start();
	I2C2_SendByte(0xEC); //写地址
	I2C2_WaitAck();
	I2C2_SendByte(0); // start read sequence
	I2C2_WaitAck();
	I2C2_Stop();

	I2C2_Start();
	I2C2_SendByte(0xEC + 0x01); //进入接收模式
	I2C2_WaitAck();
	temp[0] = I2C2_RadeByte(); //带ACK的读数据  bit 23-16
   I2C2_Ack();
	temp[1] = I2C2_RadeByte(); //带ACK的读数据  bit 8-15
   I2C2_Ack();
	temp[2] = I2C2_RadeByte(); //带NACK的读数据 bit 0-7
   I2C2_NoAck();
	I2C2_Stop();
   
   conversion = ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1] << 8) | temp[2];

	return conversion;
}

/**
  * 函    数：压力原始数据读取
  * 参    数：无
  * 返 回 值：压力原始数据
  */
uint32_t MS5837_Pre_Conversion(void)
{
   uint32_t  conversion = 0;
   uint8_t pre[3];
   I2C2_Start();
	I2C2_SendByte(0xEC); //写地址
	I2C2_WaitAck();
	I2C2_SendByte(0); // start read sequence
	I2C2_WaitAck();
	I2C2_Stop();

	I2C2_Start();
	I2C2_SendByte(0xEC + 0x01); //进入接收模式
	I2C2_WaitAck();
	pre[0] = I2C2_RadeByte(); //带ACK的读数据  bit 23-16
   I2C2_Ack();
	pre[1] = I2C2_RadeByte(); //带ACK的读数据  bit 8-15
   I2C2_Ack();
	pre[2] = I2C2_RadeByte(); //带NACK的读数据 bit 0-7
   I2C2_NoAck();
	I2C2_Stop();
   
   conversion = ((uint32_t)pre[0] << 16) | ((uint32_t)pre[1] << 8) | pre[2];

	return conversion;
}

/**
  * 函    数：读取
  * 参    数：无
  * 返 回 值：无
  */
void MS5837_Temp_Getdata(double *outTemp1, double *outPress1)
{
   double outTemp = 1;
   double outPress = 1;
   outTemp1 = &outTemp;
   outPress1 = &outPress;
//   printf ("%d  %d  ", D1_Pres, D2_Temp);

	if (D2_Temp > (((uint32_t)Cal_C[5]) * 256))
	{
		dT = D2_Temp - (((uint32_t)Cal_C[5]) * 256);
		Temperature = 2000 + dT * ((uint32_t)Cal_C[6]) / 8388608;
		OFF_ = (uint32_t)Cal_C[2] * 65536 + ((uint32_t)Cal_C[4] * dT) / 128;
		SENS = (uint32_t)Cal_C[1] * 32768 + ((uint32_t)Cal_C[3] * dT) / 256;
	}
	else
	{
		dT = (((uint32_t)Cal_C[5]) * 256) - D2_Temp;
		Temperature = 2000 - dT * ((uint32_t)Cal_C[6]) / 8388608;
		OFF_ = (uint32_t)Cal_C[2] * 65536 - ((uint32_t)Cal_C[4] * dT) / 128;
		SENS = (uint32_t)Cal_C[1] * 32768 - ((uint32_t)Cal_C[3] * dT) / 256;
	}
//	printf ("%lld  %lld %.f %lld  ", dT, Temperature, OFF_, SENS);
	if (Temperature < 2000) // low temp
	{
		Aux = (2000 - Temperature) * (2000 - Temperature);
		T2 = 3 * (dT * dT) / 8589934592;
		OFF2 = 3 * Aux / 2;
		SENS2 = 5 * Aux / 8;
	}
	else
	{
		Aux = (Temperature - 2000) * (Temperature - 2000);
		T2 = 2 * (dT * dT) / 137438953472;
		OFF2 = 0;
		SENS2 = 0;
	}
//   printf ("%d  %d ", T2, OFF2);
	OFF_ = OFF_ - OFF2;
	SENS = SENS - SENS2;
	
	*outTemp1 = (double)(((double)(Temperature - T2))/100.0);
	*outPress1 =(double )((((D1_Pres * (double)SENS) / (double)2097152) - OFF_) / 16384) / 100.0;
   printf ("PRE:%f  %f\r\n", *outTemp1, *outPress1);

}  

/**
  * 函    数：数据读取状态机制
  * 参    数：无
  * 返 回 值：无
  */
//void MS5837_S_Get_Statistic(double *outTemp1, double *outPress1)
//{
//   switch (init_state)
//   {
//      case MS5837_TEMP_START:
//            if(MS5837_Temp_Start())
//            {
//               init_state = MS5837_TEMP_CONVERSION;
//            }
////            printf ("temp\r\n");
//            break ;
//            
//      case MS5837_TEMP_CONVERSION:
//            D2_Temp = MS5837_Temp_Conversion();
//            init_state = MS5837_PRE_START;
//      
//      case MS5837_PRE_START:
//            if(MS5837_Pre_Start())
//            {
//               init_state = MS5837_PRE_CONVERSION;
//            }
////            printf ("pre\r\n");
//            break ;
//            
//      case MS5837_PRE_CONVERSION:
//            D1_Pres = MS5837_Pre_Conversion();
//            init_state = MS5837_PRE_FINISH;
////            printf ("conver\r\n");
//      case MS5837_PRE_FINISH:
//            MS5837_Temp_Getdata(outTemp1,outPress1);
//            init_state = MS5837_TEMP_START;
////            printf ("finish\r\n");
//            break ;
//   }
//   
//}

/**************************实现函数********************************************
*函数原型:uint32_t MS583702BA_getConversion(uint8_t command)
*功　　能:    读取 MS5837 的转换结果 
*******************************************************************************/
uint32_t MS583702BA_getConversion(uint8_t command)
{
	
	uint32_t  conversion = 0;
	uint8_t temp[3];

	
	I2C2_Start();
	I2C2_SendByte(0xEC); //写地址
	I2C2_WaitAck();
	I2C2_SendByte(command); //写转换命令
	I2C2_WaitAck();
	I2C2_Stop();
	
	HAL_Delay(10);			// 等待AD转换完成
	
	I2C2_Start();
	I2C2_SendByte(0xEC); //写地址
	I2C2_WaitAck();
	I2C2_SendByte(0); // start read sequence
	I2C2_WaitAck();
	I2C2_Stop();

	I2C2_Start();
	I2C2_SendByte(0xEC + 0x01); //进入接收模式
	I2C2_WaitAck();
	temp[0] = I2C2_RadeByte(); //带ACK的读数据  bit 23-16
   I2C2_Ack();
	temp[1] = I2C2_RadeByte(); //带ACK的读数据  bit 8-15
   I2C2_Ack();
	temp[2] = I2C2_RadeByte(); //带NACK的读数据 bit 0-7
   I2C2_NoAck();
	I2C2_Stop();
	
	conversion = ((uint32_t)temp[0] << 16) | ((uint32_t)temp[1] << 8) | temp[2];

	return conversion;
}

///***********************************************
//  * @brief  读取气压
//  * @param  None
//  * @retval None
//************************************************/
void MS5837_Getdata(double *outTemp, double *outPress)
{
//   double outTemp = 1;
//   double outPress = 1;
//	*outTemp = (double)((Temperature - T2)/100.0);
//	*outPress = (double)((((D1_Pres * SENS) / 2097152) - OFF_) / 16384) / 100.0;	
	D1_Pres = MS583702BA_getConversion(0x48);
    D2_Temp = MS583702BA_getConversion(0x58);   
	printf ("%d  %d  ", D1_Pres, D2_Temp);
	int64_t dT;  
    int64_t Temperature;  
// 计算温度偏差
    dT = D2_Temp - ((uint32_t)Cal_C[5] << 8);
    
    // 计算实际温度
    Temperature = 2000 + (dT * (uint64_t)Cal_C[6] / 8388608);
    
    // 计算偏移量和灵敏度
    OFF_ = (uint64_t)Cal_C[2] * 131072 + (dT * (uint64_t)Cal_C[4] / 64);
    SENS = (uint64_t)Cal_C[1] * 65536 + (dT * (uint64_t)Cal_C[3] / 128);
    
    // 温度补偿
    if (Temperature < 2000) {
        T2 = (dT * dT) / 2147483648UL;
        OFF2 = 61 * (Temperature - 2000) * (Temperature - 2000) / 16;
        SENS2 = 2 * OFF2;
        
        if (Temperature < -1500) {
            OFF2 += 15 * (Temperature + 1500) * (Temperature + 1500);
            SENS2 += 8 * (Temperature + 1500) * (Temperature + 1500);
        }
        
        Temperature -= T2;
        OFF_ -= OFF2;
        SENS -= SENS2;
    }
    
    // 计算压力
    *outPress = (float)((float)(SENS * D1_Pres / 2097152) - OFF_) / 32768 / 10.0f;
    *outTemp = Temperature / 100.0f;
    
    // 调试输出
//    printf("MS5837: Temp=%.2fC, Press=%.2fhPa\n", *outTemp, *outPress);
}  

void sensorReadMS5837(double *TEMPERATURE,double *PRESSURE, double *DEPTH)
{
	static double  air_pressure = 985.0f;		// 默认大气压（正常是990-1010之间），保证算出来的是个正值，初始时刻是不是零深度并不重要
    MS5837_Getdata(TEMPERATURE,PRESSURE); 		//获取当前气压
	*DEPTH =  (*PRESSURE - air_pressure) / 0.983615f;
}
