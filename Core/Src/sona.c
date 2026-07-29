#include "main.h"
#include "tim.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "ms5837_uart.h"
#include "fdcan.h"
#include "stdio.h"
#include "connection.h"
#include "string.h"
#include "event_groups.h"
#include "connection.h"
#define M_PI 3.1415926
uint16_t crc16(uint8_t*data,int len)
{
uint16_t reg=0xffff;
	for(int i=0;i<len;i++)
	{
	reg=reg^data[i];
		for(int j=0;j<8;j++)
		{
		uint8_t bit=reg&0x01;
			reg>>=1;
			if(bit)
			{
				reg^=0xa001;
			}
		}
		
	}
	uint16_t low=reg&0xff;
	reg>>=8;
	reg|=(low<<8);
	return reg;
}
void sona_read(uint8_t devaddr,uint16_t reg,uint16_t len)
{
uint8_t data[8];
data[0]=devaddr;
	data[1]=0x03;
	data[2]=reg>>8;
	data[3]=reg&0xff;
	data[4]=len>>8;
	data[5]=len&0xff;
	uint16_t crc=crc16(data,6);
	data[6]=crc>>8;
	data[7]=crc&0xff;
	
}

void sona_write(uint8_t devaddr,uint16_t reg,uint16_t len)
{
uint8_t data[8];
data[0]=devaddr;
	data[1]=0x03;
	data[2]=reg>>8;
	data[3]=reg&0xff;
	data[4]=len>>8;
	data[5]=len&0xff;
	uint16_t crc=crc16(data,6);
	data[6]=crc>>8;
	data[7]=crc&0xff;
	
}

void timer_start()
{
	HAL_TIM_Base_Start_IT(&htim5);
	timer=0;
}

float timer_getperiod()
{
	//HAL_TIM_Base_Stop_IT(&htim5);
return (float)timer*10;//us
}
uint8_t sona_rx_buffer[64];

void sona_send(UART_HandleTypeDef*huart)
{
	uint8_t data[7]={0x01,0x6a,0x07,0x01,0xff,0x3f,0xb1};
	uint8_t data1[7]={0x02,0x6a,0x07,0x01,0xff,0x3f,0xb2};
	uint8_t*p;
	if(huart==&huart2)
		p=data;
	else if(huart==&huart4)
		p=data1;
HAL_UART_Transmit(huart,p,7,0xff);
	timer_start();
	HAL_UART_Receive_IT(huart,sona_rx_buffer,6);

}

float sona2len=0;
float sona4len=0;
//int sona_parse(uint8_t*data,int len)
//{
//	

//int p=0;
//int id;
//	int time;
//	int data_len=0;
//	int sum=0;
//	int sona_data;
//		for(;p<len;p++)
//	{
//	if(data[p]==0x8b)
//	{
//		data_len=data[p+1];
//		p=p-1;

//	break;
//	
//	}
//}
//for(int i=0;i<data_len-1;i++)
//	{
//	sum+=data[p+i];
//	
//	}
//	if((sum&0xff)==data[data_len-1])
//	{
//		if(data[p+3]==0)	
//	{	time=timer_getperiod();
//	id=data[p];
//	sona_data=data[p+4];
//	}
//	}
//	else{
//	return 1;
//	}
//	if(id==1)
//	{
//		sona2len=1340*time/2;
//	}

//else if(id==2)
//{
//sona4len=1340*time/2;

//}
//return 0;
//			
//		
//	}
float sona_parse(uint8_t*data,int len)
{
	

int p=0;
int id;
	int time;
	int data_len=0;
	int sum=0;
	int sona_data;
		for(;p<len;p++)
	{
	if(data[p]==0x8b)
	{
		data_len=data[p+1];
		p=p-1;

	break;
	
	}
}
for(int i=0;i<data_len-1;i++)
	{
	sum+=data[p+i];
	
	}
	if((sum&0xff)==data[data_len-1])
	{
		if(data[p+3]==0)	
	{	time=timer_getperiod();
	id=data[p];
	sona_data=data[p+4];
	}
	}
	else{
	return 1;
	}

return time;
			
		
	}
int sona_state=0;
int clac_flag=0;
float pos_x,pos_y,pos_z;
float lx,ly,lz;
int ch_flag=0;
float ldvalue=0;
	//latest
//void calc_pos(float d1,float d2,float theta)
//{
//float d=24;
//	float d_theta=45/180*M_PI;
//float x,y,z;	
//	float ax,ay,az;
//	float h=ms5837_depth;
//	x=(d1*d1-d2*d2)/(2*d);
//	y=(2*d*d*(d1*d1+d2*d2)-d*d*d*d-4*d*d*h*h-((d1*d1)-(d2*d2))*((d1*d1)-(d2*d2)))/(4*d*d);
//	y=sqrt(y);
//	z=h;
//	float x0,y0,z0;
//	x0=x;
//	y0=y;
//	z0=z;
//		theta+=d_theta;
//	ax=x0-(2*tan(theta)*(tan(theta)*x0-y0)/(pow(tan(theta),2)+1));
//	ay=y+2*(tan(theta)*x0-y0)/(pow(tan(theta),2)+1);
//	az=h;
//	
//	if(clac_flag==1)
//	{float verx,very,verz;
//	verx=x-lx;
//	very=y-ly;
//	verz=z-lz;
//	float dot=verx*lx+verx*ly+verz*lz;
//	float dvalue=d1*d1+d2*d2;
//	float ddvalue=dvalue-ldvalue;
//	if(ddvalue*dot>0)
//		ch_flag=0;
//	else if(ddvalue*dot<0)
//		ch_flag=1;
//	
//	ldvalue=dvalue;
//}
//	else{
//clac_flag=1;
//	ldvalue=d1*d1+d2*d2;
//	}
//	if(ch_flag==0)
//	{pos_x=x;
//		pos_y=y;
//	pos_z=z;}
//	else
//	{
//		pos_x=ax;
//		pos_y=ay;
//		pos_z=az;
//	}
//	lx=pos_x;
//	ly=pos_y;
//	lz=pos_z;


//	

//}
void calc_pos(float d1,float d2,float theta)
{
float d=0.263;
	float d_theta=45/180*M_PI;
float x,y,z;	
	float ax,ay,az;
	float h=ms5837_depth-0.03;
		//printf("d1=%f,d2=%f,h=%f\n",d1,d2,h);
	if(d1>=h&&d2>=h)
	{x=(d1*d1-d2*d2)/(2*d);

	y=(2*d*d*(d1*d1+d2*d2)-d*d*d*d-4*d*d*h*h-((d1*d1)-(d2*d2))*((d1*d1)-(d2*d2)))/(4*d*d);
	y=sqrt(y);
	z=h;
	float x0,y0,z0;
	x0=x;
	y0=y;
	z0=z;
	pos_x=x;
	pos_y=y;
	pos_z=z;
	}
	//printf("%f,%f,%f\n",pos_x,pos_y,pos_z);


	

}
//void calc_pos(float d1,float d2,float theta)
//{
//float d=24;
//	float d_theta=45/180*M_PI;
//float x,y,z;	
//	float ax,ay,az;
//	float h=ms5837_depth;
//	x=(d1*d1-d2*d2)/(2*d);
//	y=(2*d*d*(d1*d1+d2*d2)-d*d*d*d-4*d*d*h*h-((d1*d1)-(d2*d2))*((d1*d1)-(d2*d2)))/(4*d*d);
//	y=sqrt(y);
//	z=h;
//	float x0,y0,z0;
//	x0=x;
//	y0=y;
//	z0=z;
//		theta+=d_theta;
//	ax=x0-(2*tan(theta)*(tan(theta)*x0-y0)/(pow(tan(theta),2)+1));
//	ay=y+2*(tan(theta)*x0-y0)/(pow(tan(theta),2)+1);
//	az=h;
//	

//	if(ch_flag==0)
//	{pos_x=x;
//		pos_y=y;
//	pos_z=z;}
//	else
//	{
//		pos_x=ax;
//		pos_y=ay;
//		pos_z=az;
//	}
//	lx=pos_x;
//	ly=pos_y;
//	lz=pos_z;
//		float verx,very;
//		verx=-pos_x;
//		very=-pos_y;
//	
//	float k=theta-90/180*M_PI;
//	float averx=1;
//	float avery=tan(k);
//	
//	float dot=verx*averx+avery*very;
//	float dist=fabs(tan(theta)*pos_x-pos_y)/sqrt(tan(theta)*tan(theta)+1);
//	if(dist<=0.01&&dot>0)
//	{if(ch_flag==1)
//			ch_flag=0;
//		else
//			ch_flag=1;
//	}
//	
//		
//	

//	

//}
//uint8_t sona_can_rx_buffer[64];
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//	if(hfdcan->Instance==FDCAN1)
//	{
//		FDCAN_RxHeaderTypeDef rx;
//		HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&rx,sona_can_rx_buffer);
//if(rx.Identifier==0x01)
//{
//		timer_start();
//}
//		
//}
//	}



EventGroupHandle_t xEventGroup; 
EventGroupHandle_t xEventGroup1;
#define BIT_UART2_RX_DONE   (EventBits_t) 0x00000001  // ???0: ??1????
#define BIT_UART4_RX_DONE   (EventBits_t) 0x00000002  // ???1: ??2????
#define ALL_BITS            (BIT_UART2_RX_DONE | BIT_UART4_RX_DONE)
#define BIT_UART8_RX_DONE    0x04
int uart2itflag=0;
int uart4itflag=0;
	uint8_t sona2buffer[64];
	uint8_t sona4buffer[64];
uint8_t sona2datalen;
	uint8_t sona4datalen;
#define uart2_buffer_size 64
uint8_t uart2_rx_buffer[uart2_buffer_size]={0};
uint8_t u2=0;
float time2=0;
float time4=0;
void UART2_IT(void)
{
    // ????
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        // ? DMA,???
        HAL_UART_DMAStop(&huart2);
        volatile uint16_t len;
					len	= uart2_buffer_size- __HAL_DMA_GET_COUNTER(huart2.hdmarx);
			//if(uart2itflag==0)
				{
			//memcpy(sona2buffer, uart2_rx_buffer, len);
			//sona2datalen=len;
			time2=timer_getperiod();//sona_parse(sona2buffer,sona2datalen);
					//time2-=266800;
					time2-=185270;
					//if(time2>1000000)
						//time2=0;
					//if(time2<0)
						//time2=0;
					//printf("t2=%f\n",time2);
      sona2len=time2*1340*0.000001;
					u2=uart2_rx_buffer[4];
				//uart2itflag=1;
					
				}
				//if(len==7||len==8)
//				char str[20];
//				sprintf(str,"2\n");
//		HAL_UART_Transmit(&huart8,(uint8_t*)str,strlen(str),0xff);
//			for(int i=0;i<8;i++)
//			{
//			sprintf(str,"%x,",uart2_rx_buffer[i]);
//		HAL_UART_Transmit(&huart8,(uint8_t*)str,strlen(str),0xff);
//			}
//			sprintf(str,"\n");
//		HAL_UART_Transmit(&huart8,(uint8_t*)str,strlen(str),0xff);
			
			//float period=timer_getperiod();
//			printf("2\n");
//			for(int i=0;i<8;i++)
//			{
//			printf("%x,",uart2_rx_buffer[i]);
//	
//			}
//      printf("\n");
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
      HAL_UART_Receive_DMA(&huart2, uart2_rx_buffer, uart2_buffer_size);
		//xEventGroupSetBitsFromISR(xEventGroup, BIT_UART2_RX_DONE, NULL);
		if(uart2itflag==0&&time2!=0)
			uart2itflag=1;
    }
}
#define uart4_buffer_size 64
uint8_t uart4_rx_buffer[uart4_buffer_size]={0};
uint8_t u4=0;
void UART4_IT(void)
{
    // ????
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        // ? DMA,???
        HAL_UART_DMAStop(&huart4);
        //uint16_t len = uart4_buffer_size- __HAL_DMA_GET_COUNTER(huart4.hdmarx);
			
			//if(uart4itflag==0)
			{
			//memcpy(sona4buffer, uart4_rx_buffer, len);
				//sona4datalen=len;
			time4=timer_getperiod();//sona_parse(sona4buffer,sona4datalen);
				//time4-=439450;
				time4-=330650;
				//if(time4>1000000)
				//time4=0;
				//if(time4<0)
					//time4=0;
				//printf("t4=%f\n",time4);
				u4=uart4_rx_buffer[4];
      sona4len=time4*1340*0.000001;//-0.02+0.03+0.03;
			//uart4itflag=1;
				
			}
//				char str[20];
//				sprintf(str,"4\n");
//		HAL_UART_Transmit(&huart8,(uint8_t*)str,strlen(str),0xff);
//			for(int i=0;i<8;i++)
//			{
//			sprintf(str,"%x,",uart4_rx_buffer[i]);
//		HAL_UART_Transmit(&huart8,(uint8_t*)str,strlen(str),0xff);
//			}
//			sprintf(str,"\n");
//		HAL_UART_Transmit(&huart8,(uint8_t*)str,strlen(str),0xff);
			
//			printf("4\n");
//			for(int i=0;i<8;i++)
//			{
//			printf("%x,",uart4_rx_buffer[i]);
//	
//			}
//      printf("\n");
			//float time=0;
				//time=sona_parse(uart4_rx_buffer,len);
			    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
			HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer, uart4_buffer_size);
			//xEventGroupSetBitsFromISR(xEventGroup, BIT_UART4_RX_DONE, NULL);
			if(uart4itflag==0&&time4!=0)
				uart4itflag=1;
    }
		
}
//calibation
//void sona_task()	
//{

//	
//		HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer,uart4_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//		HAL_UART_Receive_DMA(&huart2, uart2_rx_buffer,uart2_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//	timer_start();
//	
//	while(1)
//	{
//uint8_t pack[8]={0x02,0x6a,0x07,0x01,0xff,0x3f,0xb2};
//if(time4!=0||time2!=0)
//	//printf("time:2=%f,4=%f\n",time2,time4);
//	time2=0;time4=0;
//	HAL_UART_Transmit(&huart2,pack,7,0xff);
//	timer=0;
//vTaskDelay(pdMS_TO_TICKS(1000));
//HAL_UART_Transmit(&huart4,pack,7,0xff);
//timer=0;
//vTaskDelay(pdMS_TO_TICKS(1000));	

//	}
//}

//void sona_task()	
//{
//	//can_init();
//	xEventGroup = xEventGroupCreate();
//	
//	
//		HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer,uart4_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//		HAL_UART_Receive_DMA(&huart2, uart2_rx_buffer,uart2_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//	while(1)
//	{

//		if(uart2itflag==1&&uart4itflag==1)
//		{
//			float time2=0;
//			time2=sona_parse(sona2buffer,sona2datalen);
//      sona2len=time2*1340/2*0.00001;
//			float time4=0;
//			time4=sona_parse(sona4buffer,sona4datalen);
//      sona4len=time4*1340/2*0.00001;
//			
//			uart2itflag=0;
//			uart4itflag=0;
//		}
//		
//uint8_t data[2]={0xac,0xbc};
//		HAL_UART_Transmit(&huart8,data,2,0xff);
//timer_start();
//EventBits_t uxReturnedBits;

//	uxReturnedBits = xEventGroupWaitBits(
//            xEventGroup,            // ???????
//            ALL_BITS,               // ???????
//            pdTRUE,                 // ????????????
//            pdTRUE,                 // ????????? (???)
//            pdMS_TO_TICKS(200)           // ?????,??????
//        );
//		if(uart2itflag==1&&uart4itflag==1)
//		{
//			calc_pos(sona2len,sona4len,0/180*M_PI);	
//			uart2itflag=0;
//			uart4itflag=0;
//			printf("%f,%f,%f\n",pos_x,pos_y,pos_z);
//		}


//	
//vTaskDelay(pdMS_TO_TICKS(100));
//	}
//}
//void sona_task()	
//{
//	//can_init();
//	xEventGroup = xEventGroupCreate();
//	xEventGroup1 = xEventGroupCreate();
//	    HAL_UART_Receive_DMA(&huart8, uart8_rx_buffer,64);

//    // ??????
//    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
//		HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer,uart4_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//		HAL_UART_Receive_DMA(&huart2, uart2_rx_buffer,uart2_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//	while(1)
//	{
//EventBits_t uxReturnedBits1;
//	uxReturnedBits1 = xEventGroupWaitBits(
//            xEventGroup1,            // ???????
//            BIT_UART8_RX_DONE,               // ???????
//            pdTRUE,                 // ????????????
//            pdFALSE,                 // ????????? (???)
//						portMAX_DELAY           // ?????,??????
//        );
//		

//timer_start();
//		printf("working\n");
//EventBits_t uxReturnedBits;
//	uxReturnedBits = xEventGroupWaitBits(
//            xEventGroup,            // ???????
//            ALL_BITS,               // ???????
//            pdTRUE,                 // ????????????
//            pdFALSE,                 // ????????? (???)
//            pdMS_TO_TICKS(500)           // ?????,??????
//        );
//		if(uart2itflag==1||uart4itflag==1)
//		if((uxReturnedBits&BIT_UART2_RX_DONE)||(uxReturnedBits&BIT_UART4_RX_DONE))
//		{
//			//calc_pos(sona2len,sona4len,0/180*M_PI);	
//			printf("get\n");
//			uart2itflag=0;
//			uart4itflag=0;
//			//printf("%f,%f,%f\n",pos_x,pos_y,pos_z);
//		}


//	
//vTaskDelay(pdMS_TO_TICKS(100));
//	}
//}


//void sona_task()	
//{
//	can_init();
//	xEventGroup = xEventGroupCreate();
//		HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer,uart4_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//		HAL_UART_Receive_DMA(&huart2, uart2_rx_buffer,uart2_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//	timer_start();
//	while(1)
//	{
//uint8_t data[8]={0xfa};

//if(time4!=0||time2!=0)
//	printf("time:2=%f,4=%f\n",time2,time4);
//	time2=0;time4=0;
//	can_send(0x01,data,8);
//	timer=0;
//		//printf("working\n");
////vTaskDelay(pdMS_TO_TICKS(100));
//EventBits_t uxReturnedBits;
//	uxReturnedBits = xEventGroupWaitBits(
//            xEventGroup,            // ???????
//            ALL_BITS,               // ???????
//            pdTRUE,                 // ????????????
//            pdFALSE,                 // ????????? (???)
//            pdMS_TO_TICKS(300)           // ?????,??????
//        );
////		if(uart2itflag==1||uart4itflag==1)
//////		//if((uxReturnedBits&BIT_UART2_RX_DONE)||(uxReturnedBits&BIT_UART4_RX_DONE))
////		{
////			//calc_pos(sona2len,sona4len,0/180*M_PI);	
////			//printf("get\n");
////			printf("2=%f,4=%f\n",time2,time4);
////			//printf("2=%f\n",sona2len);
////			//printf("4=%f\n",sona4len);
////			uart2itflag=0;
////			uart4itflag=0;
////			//printf("%f,%f,%f\n",pos_x,pos_y,pos_z);
////		}


//	
////vTaskDelay(pdMS_TO_TICKS(100));
//	}
//}

//使用剩下的那个声呐进行的测试
//void sona_task()	
//{
//	can_init();
//	xEventGroup = xEventGroupCreate();
//		HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer,uart4_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//		HAL_UART_Receive_DMA(&huart2, uart2_rx_buffer,uart2_buffer_size);
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//	timer_start();
//	while(1)
//	{

//if(time4!=0||time2!=0)
//	printf("time:2=%f,4=%f\n",time2,time4);
//	time2=0;time4=0;
//uint8_t pack[2]={0xac,0xbc};
//HAL_UART_Transmit(&huart8,pack,2,0xff);
//	timer=0;
//		//printf("working\n");
////vTaskDelay(pdMS_TO_TICKS(100));
//EventBits_t uxReturnedBits;
//	uxReturnedBits = xEventGroupWaitBits(
//            xEventGroup,            // ???????
//            ALL_BITS,               // ???????
//            pdTRUE,                 // ????????????
//            pdFALSE,                 // ????????? (???)
//            pdMS_TO_TICKS(100)           // ?????,??????
//        );
////		if(uart2itflag==1||uart4itflag==1)
//////		//if((uxReturnedBits&BIT_UART2_RX_DONE)||(uxReturnedBits&BIT_UART4_RX_DONE))
////		{
////			//calc_pos(sona2len,sona4len,0/180*M_PI);	
////			//printf("get\n");
////			printf("2=%f,4=%f\n",time2,time4);
////			//printf("2=%f\n",sona2len);
////			//printf("4=%f\n",sona4len);
////			uart2itflag=0;
////			uart4itflag=0;
////			//printf("%f,%f,%f\n",pos_x,pos_y,pos_z);
////		}


//	
////vTaskDelay(pdMS_TO_TICKS(100));
//	}
//}


//latest
void sona_task()	
{
	can_init();
	xEventGroup = xEventGroupCreate();
		HAL_UART_Receive_DMA(&huart4, uart4_rx_buffer,uart4_buffer_size);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart2, uart2_rx_buffer,uart2_buffer_size);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	uint8_t set_pack[8]={0xff,0x06,0x02,0x3c,0x00,0x04,0x5c,0x63};
	HAL_UART_Transmit(&huart2,set_pack,8,0xff);
	HAL_UART_Transmit(&huart4,set_pack,8,0xff);
	timer_start();
	uint8_t data[8]={0xfa};
	while(1)
	{

//if(time4!=0||time2!=0)
	//printf("time:2=%f,4=%f\n",time2,time4);
	time2=0;time4=0;
can_send(0x01,data,8);//new
	timer=0;
		//uint8_t pack[7]={0x02,0x6a,0x07,0x01,0xff,0x3f,0xb2};
		//HAL_UART_Transmit(&huart2,pack,7,0xff);
		//printf("working\n");
vTaskDelay(pdMS_TO_TICKS(1000));
//EventBits_t uxReturnedBits;
//	uxReturnedBits = xEventGroupWaitBits(
//            xEventGroup,            // ???????
//            BIT_UART2_RX_DONE,               // ???????
//            pdTRUE,                 // ????????????
//            pdFALSE,                 // ????????? (???)
//            pdMS_TO_TICKS(1500)           // ?????,??????
//        );
//EventBits_t uxReturnedBits1;
can_send(0x02,data,8);
timer=0;
		//HAL_UART_Transmit(&huart4,pack,7,0xff);
vTaskDelay(pdMS_TO_TICKS(1000));
//	uxReturnedBits1 = xEventGroupWaitBits(
//            xEventGroup,            // ???????
//            BIT_UART4_RX_DONE,               // ???????
//            pdTRUE,                 // ????????????
//            pdFALSE,                 // ????????? (???)
//            pdMS_TO_TICKS(3000)           // ?????,??????
//        );
		
		//if((uxReturnedBits&BIT_UART2_RX_DONE)&&(uxReturnedBits1&BIT_UART4_RX_DONE))
//		{
//			//calc_pos(sona2len,sona4len,0/180*M_PI);	
//			printf("get\n");q	

if(uart2itflag==1&&uart4itflag==1)
{	
//printf("2=%f,4=%f\n",time2,time4);
//printf("2=%f\n",sona2len);
//printf("4=%f\n",sona4len);
	calc_pos(sona2len,sona4len,0/180*M_PI);
	printf("pos=%f,%f,%f\n",pos_x,pos_y,pos_z);
}

//			uart2itflag=0;
//			uart4itflag=0;

			//printf("%f,%f,%f\n",pos_x,pos_y,pos_z);
//		}

if(uart2itflag==1)
	//printf("2:%f\n",time2);
	printf("2=%f\n",sona2len);
if(uart4itflag==1)
	//printf("4:%f\n",time4);
	printf("4=%f\n",sona4len);
uart2itflag=0;
uart4itflag=0;
//printf("2=%x,4=%x\n",u2,u4);
	}
}


//extern TaskHandle_t xTaskToNotify;
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(sona_parse(sona_rx_buffer,6)==0)
//	{if(sona_state==0)   
//		sona_state=1;
//else if(sona_state==1)
//sona_state=0;

//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//vTaskNotifyGiveFromISR(xTaskToNotify,&xHigherPriorityTaskWoken);
//}
//	__HAL_UART_CLEAR_IT(huart,UART_IT_RXNE);
//}

