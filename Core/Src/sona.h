#include "main.h"
#include "FreeRTOS.h"
#include "event_groups.h"
uint16_t crc16(uint8_t*data,int len);
extern float sona2len;
extern float sona4len;
extern uint8_t sona_rx_buffer[64];
extern float pos_x,pos_y,pos_z;
void sona_send(UART_HandleTypeDef*huart);
void sona_task();
extern EventGroupHandle_t xEventGroup1;
extern int control_bits;
extern int sona_cableless_flag;