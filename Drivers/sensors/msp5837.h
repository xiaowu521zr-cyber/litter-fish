#ifndef __MSP5837_H
#define __MSP5837_H
#include "software_iic2.h"
#include <stdint.h> 
#define MS58xx_ADC_RD            0x00
#define MS58xx_PROM_RD 	         0xA0
#define MS58xx_PROM_CRC          0xAE
        
#define MS58xx_SlaveAddress      0xEE  
#define MS58xx_RST               0x1E  
        
#define MS58xx_D2_OSR_4096      	0x5A	// 9.04 mSec conversion time ( 110.62 Hz)
#define MS58xx_D1_OSR_4096      	0x4A
        
#define MS58xx_OSR256				0x40
#define MS58xx_OSR512				0x42
#define MS58xx_OSR1024				0x44
#define MS58xx_OSR2048				0x46
#define MS58xx_OSR4096				0x48
void MS5837_Init(void);
uint32_t MS583702BA_getConversion(uint8_t command);
void MS5837_Getdata(double *outTemp1, double *outPress1);
void sensorReadMS5837(double *TEMPERATURE,double *PRESSURE, double *DEPTH);

#endif
