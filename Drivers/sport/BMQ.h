#ifndef BMQ_H
#define BMQ_H

#include "stdio.h"	
#include <stdint.h>
extern uint32_t Cnt[4]; 
extern uint32_t _Angle[4];

void ProcessEncoderData(void);
void set_zero (void);
void re_angle (void);
void re_cnt (void);
void BMQ_Init(void);
#endif
