#ifndef MS5837_UART_H
#define MS5837_UART_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

/**
 * @brief  启动 UART4 DMA + 空闲中断接收，并解析温度/深度
 * @note   在 HAL 库初始化并 MX_USART4_UART_Init() 后调用
 */
void ms5837_Init(void);
void UART3_IT_TASK(void);
void ms5837_test();
void ms5837_task();
void ms5837_parse_frame(const uint8_t *buf, int len);

/* 解析得到的物理量，单位：℃、m */
extern float ms5837_temperature;
extern float ms5837_depth;
extern float ms5837_pressure;


/* DMA 接收缓冲区大小（要大于一帧最大长度） */
#define UART3_DMA_BUF_SIZE  64
extern uint8_t uart3_rx_buffer[UART3_DMA_BUF_SIZE];
typedef struct {
    uint8_t  data[UART3_DMA_BUF_SIZE];
    uint16_t len;
} UART3_Msg_t;
extern UART3_Msg_t uart3_msg;
extern uint8_t uart3_it_flag;

#endif /* MS5837_UART_H */
