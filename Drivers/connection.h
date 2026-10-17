#ifndef CONNECTION
#define CONNECTION
#include "main.h"
void can_init();
void can_send(int id,uint8_t*data,int len);
void can_send_long(uint16_t id,uint8_t*data,int len);
typedef struct{
int head;
int bits;
float stick_lx;
float stick_ly;
float stick_rx;
float stick_ry;
float cam;
float light;
float stick_r;
float stick_p;
}controler;
extern int can_flag;
#endif