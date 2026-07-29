#ifndef CONNECTION
#define CONNECTION
#include "main.h"
void can_init();
void can_send(int id,uint8_t*data,int len);
typedef struct{
int head;
int bits;
float stick_lx;
float stick_ly;
float stick_rx;
float stick_ry;
float cam;
float light;

}controler;
extern int can_flag;
#endif