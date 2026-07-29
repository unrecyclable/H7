#include "serial.h"
#include "string.h"
serial_Msg_t uart3_msg;
uint8_t serial_rx_buffer[serial_rx_buffer_size];
void serial_parse_data(uint8_t*data,int len);
int move_flag=0,light_flag=0,y_flag=0;
int move_gain=0,y_gain=0;
void serial_init()
{
	MX_USART3_UART_Init();
	// 启动DMA接收
  //HAL_UART_Receive_DMA(&huart3, serial_rx_buffer, serial_rx_buffer_size);
	HAL_UART_Receive_IT(&huart3,serial_rx_buffer,4);
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		
		serial_parse_data(serial_rx_buffer,4);
		
		HAL_UART_Receive_IT(&huart3,serial_rx_buffer,4);
	}
}

/*void UART3_IT(void)
{
    // 空闲检测
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        // 停 DMA，算长度
        HAL_UART_DMAStop(&huart3);
        uint16_t len = serial_rx_buffer_size - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

            uart3_msg.len = len;
            memcpy(uart3_msg.data, serial_rx_buffer, len);
			//serial_parse_data(uart3_msg.data,uart3_msg.len);
        // 重启 DMA
       HAL_UART_Receive_DMA(&huart3, serial_rx_buffer, serial_rx_buffer_size);
    }
}*/
void UART3_IT(void)
{
    // 空闲检测
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
       
       HAL_UART_Receive_IT(&huart3, serial_rx_buffer, 4);
    }
}

int max_delta=200;
void serial_parse_data(uint8_t*data,int len)  //0xfe 0x10 xx(0x00~0x08)//运动
																							//0xfe 0x20 xx(0x00,0x01)//灯
{
	int bufferp=0;
	int flag=0;
	uint8_t ch=0;
	while(bufferp<len)
	{
		ch=data[bufferp];
		switch(flag)
		{
			case 0:
			if(ch==0xfe)
				flag=1;
			break;
			case 1:
				if(ch==0x10)
					flag=2;
				else if(ch==0x20)
					flag=3;
				else if(ch==0x30)
					flag=4;
				break;
			case 2:
					move_flag=ch;
					if(data[bufferp+1]!=0)
							move_gain=data[bufferp+1];
					flag=5;
					break;
			case 3:
					light_flag=ch;
					flag=5;
					break;
			case 4:
					y_flag=ch;
					if(data[bufferp+1]!=0)
							y_gain=data[bufferp+1];
					flag=5;
					break;
			default:
				
				break;
				
		}
		if(flag==5)
			break;
		bufferp++;
		}
	
	
}


