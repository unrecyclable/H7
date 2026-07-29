#include "main.h"
#ifndef __MYIIC_H
#define __MYIIC_H
  	   		   
//IO��������
#define SDA_IN()  {GPIOH->MODER&=~(3<<(5*2));GPIOH->MODER|=0<<5*2;}	//PH5����ģʽ
#define SDA_OUT() {GPIOH->MODER&=~(3<<(5*2));GPIOH->MODER|=1<<5*2;} //PH5���ģʽ
//IO����
#define IIC_SCL   PHout(4) //SCL
#define IIC_SDA   PHout(5) //SDA
#define READ_SDA  PHin(5)  //����SDA

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	 
float ina226_busvoltage_v();
float ina226_shutvoltage_mv();
void ina226_init();
void ina226_reg_write(uint8_t reg,uint16_t data);
void ina226_reg_read(uint8_t reg,uint16_t*data);
#endif

