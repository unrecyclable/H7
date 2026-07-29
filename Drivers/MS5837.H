#ifndef __MS5837_H_
#define __MS5837_H_
#include "sys.h"

void MS5837_init(void);
void MS5837_Getdata(void);

extern uint64_t TEMP;
extern uint32_t Pressure;
extern uint32_t Depth;

#endif

