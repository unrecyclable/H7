#include "jy901.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ms5837_uart.h"
float accx, accy, accz, angx, angy, angz, magx, magy, magz;
float roll, pitch, yaw, temperature_jy901p;
float roll_last, pitch_last, yaw_last;
float roll_total, pitch_total, yaw_total, yaw_total_start;
float roll_start = 0;// 用于记录初始角度
float pitch_start = 0;// 用于记录初始角度
float yaw_start = 0; // 用于记录初始角度

int turns_of_pitch, turns_of_roll, turns_of_yaw;
uint8_t yaw_flag = 0; // 用于判断是否第一次接收 yaw 数据
uint8_t start_set = 0;

volatile uint8_t uart6_it_flag = 0; // 0表示没有新数据，1表示有新数据的到来
JY901_Msg_t uart6_msg;     // 作为全局变量，在中断与处理函数中传递接收到的数据以及长度

float rotation_matrix[9]={
0.9894,0.0076,-0.1450,
0.0076,0.9946,0.1036,
0.1450,-0.1036,0.9840
};
void rotation(float*acc,float*r)
{
	float temp[3];
	temp[0]=acc[0];
	temp[1]=acc[1];
	temp[2]=acc[2];
	float result[3]={};
	for(int i=0;i<3;i++)
	{
	for(int j=0;j<3;j++)
		{
				result[i]+=temp[j]*r[i*3+j];
		}
	
	}
	acc[0]=result[0];
	acc[1]=result[1];
	acc[2]=result[2];
	
}



/*—————— 内部类型 ——————*/
typedef struct
{
    int16_t a[3], T;
} SAcc;
typedef struct
{
    int16_t w[3], T;
} SGyro;
typedef struct
{
    int16_t Angle[3], T;
} SAngle;
typedef struct
{
    int16_t h[3], T;
} SMag;

/*—————— 模块静态数据 ——————*/
uint8_t jy901_rx_buffer[JY901_RX_BUFFER_SIZE];


/*——— 函数原型 ———*/
void JY901_ProcessTask(void *pvParameters);
void JY901_ParseData(const uint8_t *buf, uint16_t len);

/*—————— 公有接口 ——————*/
void JY901_UART_Init(void)
{
    
    // 启动DMA接收
    HAL_UART_Receive_DMA(&huart6, jy901_rx_buffer, JY901_RX_BUFFER_SIZE);
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
}

void uart6_receive_callback()
{
  HAL_UART_Transmit(&huart6,jy901_rx_buffer,JY901_RX_BUFFER_SIZE,100);
	HAL_UART_Receive_IT(&huart6,jy901_rx_buffer,JY901_RX_BUFFER_SIZE);
}
uint8_t jy901_read_buffer[8];

void JY901_ReadData(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;
    while (i + 10 < len)
    {
        // 找到包头 0x55，并且包类型在 0x51~0x54
        if (buf[i] == 0x55 && (buf[i + 1]<=0x5f)&&(buf[i + 1]>=0x55))
        {
            // 计算校验和：sum(buf[i] 到 buf[i+9])
            uint8_t sum = 0;
            for (uint8_t k = 0; k < 10; ++k)
            {
                sum += buf[i + k];
            }
            // 校验和通过
            if (sum == buf[i + 10])
            {
                // 指针 p 指向本包首地址
                const uint8_t *p = &buf[i];
                int16_t rawL, rawH;

                for(int i=0;i<8;i++)
							{
								jy901_read_buffer[i]=p[2+i];
							}
                                       
            }
            i += 11;
            continue;
        }
        // 非包头或校验失败，继续下一字节搜索
        ++i;
    }
}
uint16_t version = 0; // JY901 版本号
/**
 * @brief  按协议解析一段原始字节流
 * @param  buf 原始接收缓冲区
 * @param  len 缓冲区长度
 */
void JY901_ParseData(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;
    while (i + 10 < len)
    {
        // 找到包头 0x55，并且包类型在 0x51~0x54
        if (buf[i] == 0x55 && (buf[i + 1] >= 0x51 && buf[i + 1] <= 0x54))
        {
            // 计算校验和：sum(buf[i] 到 buf[i+9])
            uint8_t sum = 0;
            for (uint8_t k = 0; k < 10; ++k)
            {
                sum += buf[i + k];
            }
            // 校验和通过
            if (sum == buf[i + 10])
            {
                // 指针 p 指向本包首地址
                const uint8_t *p = &buf[i];
                int16_t rawL, rawH;

                switch (p[1])
                {
                case 0x51:
                    // 加速度包
                    // Ax = ((AxH<<8)|AxL)/32768*16*g
                    rawL = (int16_t)((p[3] << 8) | p[2]);
                    accx = (float)rawL / 32768.0f * 16.0f * 9.8f;
                    rawL = (int16_t)((p[5] << 8) | p[4]);
                    accy = (float)rawL / 32768.0f * 16.0f * 9.8f;
                    rawL = (int16_t)((p[7] << 8) | p[6]);
                    accz = (float)rawL / 32768.0f * 16.0f * 9.8f;
                    // 温度 = ((TH<<8)|TL)/100 ℃
										//accz=-accz;
                    rawL = (int16_t)((p[9] << 8) | p[8]);
                    temperature_jy901p = (float)rawL / 100.0f;
                    break;

                case 0x52:
                    // 角速度包
                    rawL = (int16_t)((p[3] << 8) | p[2]);
                    angx = (float)rawL / 32768.0f * 2000.0f;
                    rawL = (int16_t)((p[5] << 8) | p[4]);
                    angy = (float)rawL / 32768.0f * 2000.0f;
                    rawL = (int16_t)((p[7] << 8) | p[6]);
                    angz = (float)rawL / 32768.0f * 2000.0f;
                    // （如果需要读电压，可按协议解析 p[8]/p[9]）
                    break;

                case 0x53:
                    // 角度包
                    rawL = (int16_t)((p[3] << 8) | p[2]);
                    roll = (float)rawL / 32768.0f * 180.0f;
                    rawL = (int16_t)((p[5] << 8) | p[4]);
                    pitch = (float)rawL / 32768.0f * 180.0f;
                    rawL = (int16_t)((p[7] << 8) | p[6]);
                    yaw = (float)rawL / 32768.0f * 180.0f;
                    // 版本号=(VH<<8)|VL
                    rawH = p[9];
                    rawL = p[8];
                    version = (rawH << 8) | rawL;

//                    pitch+=pitch_start;
//                    roll-=roll_start;
//                    yaw-=yaw_start;

//                    roll = 180 + roll; // 将 roll 转化为 0-360 度
                    //pitch =  - pitch;//! 注意不要翘头过90度
//                    yaw = 180 + yaw; // 将 yaw 转化为 0-360 度


                    if (roll - roll_last > 270)
                    {
                        turns_of_roll -= 1;
                    }
                    else if (roll - roll_last < -270)
                    {
                        turns_of_roll += 1;
                    }
                    roll_total = roll + turns_of_roll * 360;

                    pitch_total = pitch;//! -90度到90度

                    if (yaw - yaw_last > 270)
                    {
                        turns_of_yaw -= 1;
                    }
                    else if (yaw - yaw_last < -270)
                    {
                        turns_of_yaw += 1;
                    }

                    yaw_total = yaw + turns_of_yaw * 360;
                    
                    // 更新上次值
                    roll_last = roll;
                    pitch_last = pitch;
                    yaw_last = yaw;

                    if (start_set == 0)
                    {
                        // 第一次接收数据，记录初始角度
                        roll_start = roll;
                        pitch_start = pitch;
                        yaw_start = yaw;
                        start_set = 1;
                    }
     
                    break;

                case 0x54:
                    // 磁场包
                    rawL = (int16_t)((p[3] << 8) | p[2]);
                    magx = (float)rawL;
                    rawL = (int16_t)((p[5] << 8) | p[4]);
                    magy = (float)rawL;
                    rawL = (int16_t)((p[7] << 8) | p[6]);
                    magz = (float)rawL;
                    // 温度同 0x51 包
                    rawL = (int16_t)((p[9] << 8) | p[8]);
                    temperature_jy901p = (float)rawL / 100.0f;
                    break;
									
                default:
                    break;
                }
            }
            // 跳过这个子包
            i += 11;
            continue;
        }
        // 非包头或校验失败，继续下一字节搜索
        ++i;
    }
}


void UART6_IT_HANDLE(void)
{
    // 空闲检测
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        // 停 DMA，算长度
			
        HAL_UART_DMAStop(&huart6);
        uint16_t len = JY901_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);

        if(uart6_it_flag==0)
				{ uart6_msg.len = len;
          memcpy(uart6_msg.data, jy901_rx_buffer, len);
					//JY901_ParseData(uart6_msg.data, uart6_msg.len);
					if(len!=0)
					{	//printf("ok\n");
						uart6_it_flag=1;
					}
				}
			
        // 重启 DMA
        HAL_UART_Receive_DMA(&huart6, jy901_rx_buffer, JY901_RX_BUFFER_SIZE);
    }
}
void jy901_test()
{
	
	
	while(1)
	{
		if(uart6_it_flag==1)
		{
		JY901_ParseData(uart6_msg.data, uart6_msg.len);

		//JY901_ReadData(uart6_msg.data,uart6_msg.len);
		//showarray(jy901_read_buffer,8);
		//printf("%f,%f,%f,%f,%f,%f\n",accx,accy,accz,magx,magy,magz);
		uart6_it_flag=0;
		HAL_Delay(100);
		}
	}
}
float intergrate=0;
float oaccx=0,oaccy=0,oaccz=0;
int count=0;
typedef struct{
	int head;
float roll;
	float pitch;
	float yaw;
}jy901_att;
jy901_att att={.head=0xfd};

void jy901_task()
{
	//JY901_UART_Init();
	float ax=0,ay=0,az=0;
	//int num=100;
	//int count=0;
	while(1)
	{
		if(uart6_it_flag==1)
		{
			count++;
			//printf("%d\n",count);
		JY901_ParseData(uart6_msg.data, uart6_msg.len);
		
			//安装矫正
					float acc[3];
			acc[0]=accx;
			acc[1]=accy;
			acc[2]=accz;
			rotation(acc,rotation_matrix);

			//取一位小数
			acc[0]=((float)((int)(acc[0]*10)))/10;
			acc[1]=((float)((int)(acc[1]*10)))/10;
			acc[2]=((float)((int)(acc[2]*10)))/10;
			acc[2]=-acc[2];
			if(fabs(acc[0]-oaccx)<=0.1)
			{acc[0]=oaccx;
			}
			if(fabs(acc[1]-oaccy)<=0.1)
			{acc[1]=oaccy;
			}
			if(fabs(acc[2]-oaccz)<=0.1)
			{acc[2]=oaccz;
			}
			if(fabs(acc[0])<=0.11)
			{acc[0]=0;
			}
			if(fabs(acc[1])<=0.11)
			{acc[1]=0;
			}
			if(fabs(acc[2])<=0.11)
			{acc[2]=0;
			}
			accx=acc[0];
			accy=acc[1];
			accz=acc[2];
			oaccx=accx;
			oaccy=accy;
			oaccz=accz;
			roll+=6.24;
			pitch+=8.17;
			if(fabs(roll)<1)
			{roll=0;}
			if(fabs(pitch)<1)
			{pitch=0;}
			if(fabs(yaw)<1)
			{yaw=0;}
			float data[3];
			//平均
//			if(count<num)
//			{
//			ax+=accx;
//			ay+=accy;
//			az+=accz;	
//			count++;				
//			}
//			else
//			{
//			ax/=num;ay/=num;az/=num;
//			printf("%f,%f,%f\n",ax,ay,az);
//				ax=0;ay=0;az=0;
//				count=0;
//			}
				uart6_it_flag=0;
if(count>=100)
{
	att.roll=roll;
	att.pitch=pitch;
	att.yaw=yaw;
	HAL_UART_Transmit(&huart8,(uint8_t*)&att,sizeof(jy901_att),0xff);
	count=0;
}
		}
		//intergrate+=accx;
		//printf("int=%f\n",intergrate);
		//printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n",accx,accy,accz,angx,angy,angz,roll,pitch,yaw);
//		char str[100];
//			sprintf(str,"roll=%f\n",roll);
//			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),100);
		//printf("%f,%f,%f,%f,%f,%f\n",accx,accy,accy,magx,magy,magz);
	//printf("%f,%f,%f\n",roll,pitch,yaw);
		//printf("r=%f,p=%f,y=%f\n",angx,angy,angz);
		

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
void jy901_read(uint8_t*buffer,int len)
{
HAL_UART_Receive(&huart6,buffer,len,0xff);
}
	
void jy901_send(uint8_t*cmd,int len)
{
HAL_UART_Transmit(&huart6,cmd,len,0xff);	
}
void jy901_write(uint8_t*cmd,int len)
{
uint8_t unlock[5]={0XFF,0XAA,0X69,0X88,0XB5};
jy901_send(unlock,5);
HAL_Delay(200);
jy901_send(cmd,len);
uint8_t save[5]={0xff,0xaa,0x00,0x00,0x00};
jy901_send(save,5);
}
void showarray(uint8_t*array,int len)
{
	for(int i=0;i<len;i++)
		printf("%02X",array[i]);
	printf("\n");
}
void jy901_version()
{
uint8_t cmd[5]={0xff,0xaa,0x27,0x30,0x00};
uint8_t set[5]={0xff,0xaa,0x30,0x16,0x03};
jy901_write(set,5);
HAL_Delay(10);
jy901_write(cmd,5);
HAL_Delay(10);

}
