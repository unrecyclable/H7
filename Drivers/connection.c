#include "main.h"
#include "fdcan.h"
#include <string.h>
#include "stdio.h"
#include "connection.h"
uint8_t can_rx_buffer[64];
int can_rx_flag=0;
uint8_t can_data[64];
void can_init()
{
		FDCAN_FilterTypeDef filter;
	filter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1=0;
	filter.FilterID2=0;
	filter.FilterIndex=0;
	filter.FilterType=FDCAN_FILTER_MASK;
	filter.IdType=FDCAN_STANDARD_ID;
	HAL_FDCAN_ConfigFilter(&hfdcan1,&filter);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	HAL_FDCAN_Start(&hfdcan1);
	
}
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//	if(hfdcan->Instance==FDCAN1)
//	{
//		FDCAN_RxHeaderTypeDef rx;
//		HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&rx,can_rx_buffer);
//		memcpy(can_data,can_rx_buffer,8);
//		for(int i=0;i<8;i++)
//		{
//			printf("%x",can_data[i]);
//		}
//		printf("\n");
//	}
//	
//}


//int can_flag=0;
//uint8_t data[40];
//int p=0;
//void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
//{
//	if(hfdcan->Instance==FDCAN1)
//	{
//		FDCAN_RxHeaderTypeDef rx;
//		HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&rx,can_rx_buffer);
//		if(can_flag==0&&can_rx_buffer[0]==0xfc)
//		{
//		can_flag=1;
//			p=0;
//		}
//		if(can_flag==1)
//		{
//		if(!(can_rx_buffer[0]==0xfd&&can_rx_buffer[1]==0))
//		{
//		memcpy(data+p,can_rx_buffer,8);
//			p+=8;
//		}
//		else
//		{
//			can_flag=0;
//			p=0;
//			for(int i=0;i<32;i++)
//		{
//			printf("%x",data[i]);
//		}
//		printf("\n");
//		}
//		
//		}
//		
//	}
//	
//	
//}
controler con;
int can_flag=0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if(hfdcan->Instance==FDCAN1)
	{
		FDCAN_RxHeaderTypeDef rx;
		HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&rx,can_rx_buffer);
		uint8_t* p=(uint8_t*)&con;
		if(rx.Identifier==0x400)
			memcpy(p,can_rx_buffer,8);
		else if(rx.Identifier==0x401)
			memcpy(p+8,can_rx_buffer,8);
		else if(rx.Identifier==0x402)
			memcpy(p+8*2,can_rx_buffer,8);		
		else if(rx.Identifier==0x403)
			memcpy(p+8*3,can_rx_buffer,8);
		can_flag=1;
	}
	//printf("%d,%f,%f,%f,%f\n",con.arm,con.stick_lx,con.stick_ly,con.stick_rx,con.stick_ry);
	//printf("%d,%f,%f\n",con.arm,con.stick_rx,con.stick_ry);
	
	
}
void can_test()
{
	
	
	
	
}
void can_send(int id,uint8_t*data,int len)
{
	FDCAN_TxHeaderTypeDef tx;
	tx.DataLength=len;
	tx.Identifier=id;
	tx.IdType=FDCAN_STANDARD_ID;
	tx.MessageMarker=0;
	tx.BitRateSwitch=FDCAN_BRS_OFF;
	tx.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
	tx.FDFormat=FDCAN_CLASSIC_CAN;
	tx.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
	tx.TxFrameType=FDCAN_DATA_FRAME;
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&tx,data)!=HAL_OK)
			{
				//Error_Handler();
			}
}
