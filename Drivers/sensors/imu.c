#include "usart.h"
#include "stm32f4xx_hal.h"  
#include "string.h"
#include "tim.h"
#include "math.h"
#include "gpio.h"
#include "imu.h"
float yaw=0.0, yaw1=0.0, //偏航角
	  pitch=0.0,pitch1=0.0,//俯仰
	  roll=0.0,roll1=0.0; //滚转
float phi=0.0,theta=0.0,psi=0.0;//欧拉角，弧度
float w1=0.0,w2=0.0,w3=0.0;//角速度，弧度
float ax=0.0,ay=0.0,az=0.0,ax1=0.0,ay1=0.0,az1=0.0;//初始化补偿系数
float wx=0.0,wy=0.0,wz=0.0,wx1=0.0,wy1=0.0,wz1=0.0;

uint8_t TxBuffer[256];
uint8_t TxCounter=0;
uint8_t count=0; 
uint8_t rx_buffer[50];
int cu2=0;//串口4中断子程序 计数
float ax0=0,ay0=0,az0=0,wx0=0,wy0=0,wz0=0,roll0=0,pitch0=0,yaw0=0;//陀螺仪参数偏移量
SAcc		stcAcc;
SGyro 	stcGyro;
SAngle 	stcAngle;
void CopeData(unsigned char ucData)//判断一帧完整数据
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//数据不满11个，则返回
	else
	{
		switch(ucRxBuffer[1])
		{
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);
//			printf("Raw Gyro: X=%d, Y=%d, Z=%d\r\n", 
//                           stcGyro.w[0], stcGyro.w[1], stcGyro.w[2]);
			break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;
	}
}

void USART6_Get_IMU(void)	//根据读到的陀螺仪数据计算当前的 加速度 角速度 角度
{
//	ax1=(float)-stcAcc.a[0]/32768*16;	
//	ay1=(float)-stcAcc.a[1]/32768*16;	
//	az1=(float)stcAcc.a[2]/32768*16;	
	wx1=(float)stcGyro.w[0]/32768*2000;	wx=wx1-wx0;
	wy1=(float)stcGyro.w[1]/32768*2000;	wy=wy1-wy0;
	wz1=(float)stcGyro.w[2]/32768*2000;	wz=wz1-wz0;
	roll1=(float)stcAngle.Angle[0]/32768*180;		roll=roll1-roll0;
	pitch1=(float)stcAngle.Angle[1]/32768*180;	pitch=pitch1-pitch0;
	yaw1=(float)stcAngle.Angle[2]/32768*180;		yaw=yaw1-yaw0;
	phi=roll*3.14f/180.f;theta=pitch*3.14f/180.f;psi=yaw*3.14f/180.f;
	w1=wx*3.14f/180.f;w2=wy*3.14f/180.f;w3=wz*3.14f/180.f;
}
void GetIMUData(float* out_roll, float* out_pitch, float* out_yaw, 
                float* out_wx, float* out_wy, float* out_wz)
{
    *out_roll = roll1;  
    *out_pitch = pitch1;
    *out_yaw = yaw1;
    *out_wx = wx;
    *out_wy = wy;
    *out_wz = wz;
}

void IMUinit(void)
{
//	printf("开始IMU校准...\r\n");
    
    // 读取10次原始数据求平均值
    int sum_wx = 0, sum_wy = 0, sum_wz = 0;
    for(int i=0; i<10; i++) {
        USART6_Get_IMU();  // 先读取一次确保数据更新
        sum_wx += stcGyro.w[0];
        sum_wy += stcGyro.w[1];
        sum_wz += stcGyro.w[2];
        HAL_Delay(50);
    }
    
    // 设置零偏（偏移量）
    wx0 = (float)sum_wx/10 /32768 * 2000;
    wy0 = (float)sum_wy/10 /32768 * 2000;
    wz0 = (float)sum_wz/10 /32768 * 2000;
    
//    printf("校准完成: wx0=%.2f, wy0=%.2f, wz0=%.2f\r\n", wx0, wy0, wz0);
}
