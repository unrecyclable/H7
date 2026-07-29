#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"
void SCL(int set)
{
	if(set==1)
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);
	}
		
}
void SDA(int set)
{
	if(set==1)
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0,GPIO_PIN_RESET);
	}
		
}


// ========== GPIOF0 (SDA) 切换宏 ==========
// 切换为浮空输入模式
#define PF0_MODE_INPUT_FLOATING()   do { \
    GPIOF->MODER &= ~GPIO_MODER_MODE0;          /* MODER0清0: 输入模式 */ \
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD0;          /* PUPDR0清0: 无上下拉（浮空）*/ \
} while(0)

// 切换为开漏输出模式（默认输出高电平，避免总线冲突）
#define PF0_MODE_OPEN_DRAIN()       do { \
    GPIOF->MODER &= ~GPIO_MODER_MODE0;          /* 先清空MODE0 */ \
    GPIOF->MODER |= GPIO_MODER_MODE0_0;         /* MODER0=01: 输出模式 */ \
    GPIOF->OTYPER |= GPIO_OTYPER_OT0;           /* OT0=1: 开漏输出 */ \
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD0;          /* 清除上下拉设置 */ \
    GPIOF->BSRR = GPIO_BSRR_BS0;                /* 设置输出高电平 */ \
} while(0)

// 读取PF0输入电平（仅在输入模式下有效）
#define PF0_READ()                  ((GPIOF->IDR & GPIO_IDR_ID0) ? 1 : 0)

// 控制PF0输出电平（仅在输出模式下有效）
#define PF0_WRITE(high)             do { \
    if(high) GPIOF->BSRR = GPIO_BSRR_BS0; \
    else GPIOF->BSRR = GPIO_BSRR_BR0; \
} while(0)


// ========== GPIOF1 (SCL) 切换宏 ==========
// 切换为开漏输出模式（SCL通常只需要输出模式）
#define PF1_MODE_OPEN_DRAIN()       do { \
    GPIOF->MODER &= ~GPIO_MODER_MODE1;          /* 先清空MODE1 */ \
    GPIOF->MODER |= GPIO_MODER_MODE1_0;         /* MODER1=01: 输出模式 */ \
    GPIOF->OTYPER |= GPIO_OTYPER_OT1;           /* OT1=1: 开漏输出 */ \
    GPIOF->PUPDR &= ~GPIO_PUPDR_PUPD1;          /* 清除上下拉设置 */ \
    GPIOF->BSRR = GPIO_BSRR_BS1;                /* 设置输出高电平 */ \
} while(0)

// 控制PF1输出电平
#define PF1_WRITE(high)             do { \
    if(high) GPIOF->BSRR = GPIO_BSRR_BS1; \
    else GPIOF->BSRR = GPIO_BSRR_BR1; \
} while(0)


// ========== I2C 操作函数 ==========
// SDA 引脚控制
void SDA_OUT()
{
    PF0_MODE_OPEN_DRAIN();
}

void SDA_IN()
{
    PF0_MODE_INPUT_FLOATING();
}

int READ_SDA()
{
    return HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0);
}


/*****************************************************************************
 * @name       :void IIC_Init(void)
 * @date       :2018-08-09 
 * @function   :Initialize IIC
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void IIC_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim5);
    SDA(1);
    SCL(1);  
}

/*****************************************************************************
 * @name       :void IIC_Start(void)
 * @date       :2018-08-09 
 * @function   :Generating IIC starting signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	SDA(1);	  	  
	SCL(1);
	delay_us(4);
 	SDA(0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
}	

/*****************************************************************************
 * @name       :void IIC_Stop(void)
 * @date       :2018-08-09 
 * @function   :Generating IIC stop signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/  
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	SCL(0);
	SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay_us(8);
	SCL(1); 
	delay_us(8);			
	SDA(1);//����I2C���߽����ź�				   	
}

/*****************************************************************************
 * @name       :uint8_t IIC_Wait_Ack(void)
 * @date       :2018-08-09 
 * @function   :Wait for the response signal
 * @parameters :None
 * @retvalue   :0-receive response signal successfully
								1-receive response signal unsuccessfully
******************************************************************************/ 
uint8_t IIC_Wait_Ack(void)
{
	volatile uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	SDA(1);delay_us(1);	   
	SCL(1);delay_us(1);	 
	while(READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL(0);//ʱ�����0 	   
	return 0;  
} 

/*****************************************************************************
 * @name       :void IIC_Ack(void)
 * @date       :2018-08-09 
 * @function   :Generate ACK response signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/ 
void IIC_Ack(void)
{
	SCL(0);
	SDA_OUT();
	SDA(0);
	delay_us(2);
	SCL(1);
	delay_us(2);
	SCL(0);
}

/*****************************************************************************
 * @name       :void IIC_NAck(void)
 * @date       :2018-08-09 
 * @function   :Don't generate ACK response signal
 * @parameters :None
 * @retvalue   :None
******************************************************************************/ 	    
void IIC_NAck(void)
{
	SCL(0);
	SDA_OUT();
	SDA(1);
	delay_us(2);
	SCL(1);
	delay_us(2);
	SCL(0);
}	

/*****************************************************************************
 * @name       :void IIC_Send_Byte(uint8_t txd)
 * @date       :2018-08-09 
 * @function   :send a byte data by IIC bus
 * @parameters :txd:Data to be sent
 * @retvalue   :None
******************************************************************************/				 				     	  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    SCL(0);
    for(t=0;t<8;t++)
    {              
        SDA((txd&0x80)>>7);
        txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		SCL(1);
		delay_us(2); 
		SCL(0);	
		delay_us(2);
    }	 
} 

/*****************************************************************************
 * @name       :uint8_t IIC_Read_Byte(unsigned char ack)
 * @date       :2018-08-09 
 * @function   :read a byte data by IIC bus
 * @parameters :ack:0-send nACK
									  1-send ACK
 * @retvalue   :Data to be read
******************************************************************************/		    
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        SCL(0); 
        delay_us(2);
		SCL(1);
        receive<<=1;
        if(READ_SDA())receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}
//void ina226_reg_read(uint8_t reg,uint16_t*data)
//{
//	
//		IIC_Start();
//		IIC_Send_Byte(0x40<<1|0);
//		IIC_Wait_Ack();
//		IIC_Send_Byte(reg);
//		IIC_Wait_Ack();
//		IIC_Start();
//		IIC_Send_Byte(0x40<<1|1);
//		IIC_Wait_Ack();
//	uint8_t temp[2];
//		temp[0]=IIC_Read_Byte(1);
//		temp[1]=IIC_Read_Byte(0);
//		*data=(uint16_t)temp[0]<<8|temp[1];
//	IIC_Stop();
//}

//void ina226_reg_write(uint8_t reg,uint16_t data)
//{
//	IIC_Start();
//		IIC_Send_Byte(0x40<<1|0);
//		IIC_Wait_Ack();
//		IIC_Send_Byte(0xfe);
//		IIC_Wait_Ack();
//	IIC_Send_Byte(data>>8);
//	IIC_Wait_Ack();
//	IIC_Send_Byte(data&0xff);
//		IIC_Wait_Ack();
//	IIC_Stop();
//}

int ina226_reg_write(uint8_t reg,uint8_t*data)
{
	int res=0;
res=HAL_I2C_Mem_Write(&hi2c2,0x40<<1,reg,I2C_MEMADD_SIZE_8BIT,data,2,0xff);
	return res;
}
int ina226_reg_read(uint8_t reg,uint8_t*data)
{
	int res=0;
	
	if(HAL_I2C_Master_Transmit(&hi2c2,0x40<<1,&reg,1,0xff)!=HAL_OK)
		res=HAL_ERROR;
	if(HAL_I2C_Master_Receive(&hi2c2,0x40<<1,data,2,0xff)!=HAL_OK)
		res=HAL_ERROR;
	return res;
}

#define Rshut 0.1
float currentlsb=0;
void ina226_set_calibration(float maxcurrent)
{
	currentlsb=maxcurrent/(1<<15);
	float temp=0.00512/(currentlsb*Rshut);
	ina226_reg_write(05,(uint16_t)temp);
	
}


void ina226_init()
{
	//IIC_Init();
	ina226_reg_write(0x00,0x4527);
	ina226_set_calibration(20);
	
}
float ina226_current_a()
{
	uint16_t data;
	ina226_reg_read(0x04,&data);
	float re;
	re=(float)data*currentlsb;
	return re;
}
float ina266_power()
{
	uint16_t data;
	ina226_reg_read(0x03,&data);
	float re;
	re=(float)data*currentlsb*25;
	return re;
}
float ina226_shutvoltage_mv()
{
	uint16_t data;
	ina226_reg_read(0x01,&data);
	float re;
	re=(float)data*0.0025;
	return re;
	
}
float ina226_busvoltage_v()
{
	uint16_t data;
	ina226_reg_read(0x02,&data);
	float re;
	re=(float)data*0.00125;
	return re;
	
}
//电阻100mΩ

void ina226_task()
{
	ina226_init();
	while(1)
	{
	
	float v=ina226_busvoltage_v();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}



















