#ifndef SERIAL
#define SERIAL
#include "usart.h"
#define serial_rx_buffer_size  128
typedef struct {
    uint8_t  data[serial_rx_buffer_size];
    uint16_t len;
} serial_Msg_t;
void UART3_IT(void);
void serial_init();
void serial_parse_data(uint8_t*data,int len);
extern int move_flag;
extern int light_flag;
extern int y_flag;
extern int move_gain;
extern int y_gain;
#endif