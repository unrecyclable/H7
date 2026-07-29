#include "stm32h7xx.h"
#include "main.h"
#include "icm.h"
#include "FreeRTOS.h"
#include "task.h" 
#include "stdio.h"
uint16_t accx,accy,accz;
uint16_t gyrox,gyroy,gyroz;
float Gyro_Sensitivity,Acc_Sensitivity;

ICM42688_RAW_DATA ICM42688_RAW;
ICM42688_real_data ICM42688;
uint8_t ICM42688_Get_Data_Flag = 0;
int32_t bias_gyro_x,bias_gyro_y,bias_gyro_z;
int icm_reg_write(uint8_t	reg,uint8_t data)
{
	uint8_t temp;
	temp=((~(1<<7))&reg);
	return spi_reg_write(reg,&data,1);
	
}

int icm_reg_read(uint8_t reg,int len,uint8_t*data)
{
	uint8_t seg;
	seg=1<<7|reg;
	return spi_reg_receive(seg,data,len);
	
}

void icm_hard_init()
{
}
void icm_init()
{
	icm_hard_init();
	accx=0;accy=0;accz=0;
	gyrox=0;gyroy=0;gyroz=0;
	icm_reg_write(0x4e,0x0f);// reset
	HAL_Delay(10);
}

void icm_read_raw_acc_gyro()
{
	
	uint8_t data[12];
	icm_reg_read(0x1f,12,data);
	accx=data[0]<<8|data[1];
	accy=data[2]<<8|data[3];
	accz=data[4]<<8|data[5];
	gyrox=data[6]<<8|data[7];
	gyroy=data[8]<<8|data[9];
	gyroz=data[10]<<8|data[11];
	
	
}
static int Find_ICM42688(void)
{
		uint8_t ID=0;
		icm_reg_read(WHO_AM_I>>8,1,&ID);
		icm_reg_read(WHO_AM_I>>8,1,&ID);
	//!!
    if(ID!=ICM42688_ID)
    {
        
        return 1;
    }
		else
			return 0;
}
//???????????
static int ICM42688_SET_GYRO(GYRO_FSR gyro_fsr,GYRO_ODR gyro_odr)
{
    uint8_t WRITE_GYRO_CONFIG0 = 0x4F;
    uint8_t GYRO_CONFIG0 = 0x00;
    uint8_t GYRO_CONFIG_Verify = 0x00;
    uint16_t READ_GYRO_CONFIG0 = 0XCF00;
    switch(gyro_fsr)
        {
            case GYRO_15_625DPS:
                GYRO_CONFIG0|=0xE0;
                Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_15_625dps;
                break;
            case GYRO_31_25DPS:
                GYRO_CONFIG0|=0xC0;
                Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_31_25dps;
                break;
            case GYRO_62_5DPS:
                GYRO_CONFIG0|=0xA0;
                Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_62_5dps;
                break;
            case GYRO_125DPS:
                GYRO_CONFIG0|=0x80;
                Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_125dps;
                break;
            case GYRO_250DPS:
                GYRO_CONFIG0|=0x60;
                Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_250dps;
                break;
            case GYRO_500DPS:
                GYRO_CONFIG0|=0x40;
                Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_500dps;
                break;
            case GYRO_1000DPS:
                GYRO_CONFIG0|=0x20;
                Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_1000dps;
                break;
            case GYRO_2000DPS:
                GYRO_CONFIG0|=0x00;
                Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_2000dps;
                break;
        }
    switch(gyro_odr)
        {
            case GYRO_ODR_12_5HZ:
                GYRO_CONFIG0|=0x0B;
                break;
            case GYRO_ODR_25HZ:
                GYRO_CONFIG0|=0x0A;
                break;
            case GYRO_ODR_50HZ:
                GYRO_CONFIG0|=0x09;
                break;
            case GYRO_ODR_100HZ:
                GYRO_CONFIG0|=0x08;
                break;
            case GYRO_ODR_200HZ:
                GYRO_CONFIG0|=0x07;
                break;
            case GYRO_ODR_500HZ:
                GYRO_CONFIG0|=0x0F;
                break;
            case GYRO_ODR_1000HZ:
                GYRO_CONFIG0|=0x06;
                break;
            case GYRO_ODR_2000HZ:
                GYRO_CONFIG0|=0x05;
                break;
            case GYRO_ODR_4000HZ:
                GYRO_CONFIG0|=0x04;
                break;
            case GYRO_ODR_8000HZ:
                GYRO_CONFIG0|=0x03;
                break;
            case GYRO_ODR_16000HZ:
                GYRO_CONFIG0|=0x02;
                break;
            case GYRO_ODR_32000HZ:
                GYRO_CONFIG0|=0x01;
                break;
            }
   
		icm_reg_write(WRITE_GYRO_CONFIG0,GYRO_CONFIG0);
		icm_reg_read(READ_GYRO_CONFIG0,1,&GYRO_CONFIG_Verify);
    if(GYRO_CONFIG_Verify!=GYRO_CONFIG0)
    {
 
        return 1;
    }
		else
			return 0;
}
 //????????????
static int ICM42688_SET_ACC(ACC_FSR acc_fsr,ACC_ODR acc_odr)
{
    uint8_t WRITE_ACCEL_CONFIG0 = 0x50;
    uint8_t ACCEL_CONFIG0 = 0x00;
    uint8_t ACCEL_CONFIG_Verify = 0x00;
    uint16_t READ_ACCEL_CONFIG0 = 0XD000;
    switch(acc_fsr)
        {
            case ACC_2G:
                ACCEL_CONFIG0|=0x60;
                Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_2G;
                break;
            case ACC_4G:
                ACCEL_CONFIG0|=0x40;
                Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_4G;
                break;
            case ACC_8G:
                ACCEL_CONFIG0|=0x20;
                Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_8G;
                break;
            case ACC_16G:
                ACCEL_CONFIG0|=0x00;
                Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_16G;
                break;
        }
    switch(acc_odr)
        {
            case ACC_ODR_12_5HZ:
                ACCEL_CONFIG0|=0x0B;
                break;
            case ACC_ODR_25HZ:
                ACCEL_CONFIG0|=0x0A;
                break;
            case ACC_ODR_50HZ:
                ACCEL_CONFIG0|=0x09;
                break;
            case ACC_ODR_100HZ:
                ACCEL_CONFIG0|=0x08;
                break;
            case ACC_ODR_200HZ:
                ACCEL_CONFIG0|=0x07;
                break;
            case ACC_ODR_500HZ:
                ACCEL_CONFIG0|=0x0F;
                break;
            case ACC_ODR_1000HZ:
                ACCEL_CONFIG0|=0x06;
                break;
            case ACC_ODR_2000HZ:
                ACCEL_CONFIG0|=0x05;
                break;
            case ACC_ODR_4000HZ:
                ACCEL_CONFIG0|=0x04;
                break;
            case ACC_ODR_8000HZ:
                ACCEL_CONFIG0|=0x03;
                break;
            case ACC_ODR_16000HZ:
                ACCEL_CONFIG0|=0x02;
                break;
            case ACC_ODR_32000HZ:
                ACCEL_CONFIG0|=0x01;
                break;
            }
   
						icm_reg_write(WRITE_ACCEL_CONFIG0,ACCEL_CONFIG0);
						icm_reg_read(READ_ACCEL_CONFIG0,1,&ACCEL_CONFIG_Verify);
    
    if(ACCEL_CONFIG_Verify!=ACCEL_CONFIG0)
    {
        
        return 1;
    }
		return 0;
}

//???????
static int ICM42688_SET_FILTER(Bandwidth_Factor Gyro_Bandwidth_Factor,Filter_Order Gyro_Filter_Order,Bandwidth_Factor Acc_Bandwidth_Factor,Filter_Order Acc_Filter_Order)
{
    uint8_t WRITE_GYRO_ACCEL_CONFIG0=0X52;
    uint8_t WRITE_GYRO_CONFIG1=0X51;
    uint8_t WRITE_ACCEL_CONFIG1=0X53;
    uint8_t Gyro_Filter_Ord_Config = 0x00;
    uint8_t Acc_Filter_Ord_Config = 0x00;
    uint8_t Filter_Config=0x00;
    switch(Gyro_Bandwidth_Factor)
    {
        case Bandwidth_Factor_2:
           Filter_Config|=0x00;
            break;
        case Bandwidth_Factor_4:
           Filter_Config|=0x01;
            break;
        case Bandwidth_Factor_5:
           Filter_Config|=0x02;
            break;
        case Bandwidth_Factor_8:
           Filter_Config|=0x03;
            break;
        case Bandwidth_Factor_10:
           Filter_Config|=0x04;
            break;
        case Bandwidth_Factor_16:
           Filter_Config|=0x05;
            break;
        case Bandwidth_Factor_20:
           Filter_Config|=0x06;
            break;
        case Bandwidth_Factor_40:
           Filter_Config|=0x07;
            break;
        case Low_latency_1:
           Filter_Config|=0x0E;
            break;
        case Low_Latency_2:
           Filter_Config|=0x0F;
            break;
    }
    switch(Acc_Bandwidth_Factor)
    {
        case Bandwidth_Factor_2:
           Filter_Config|=0x00;
            break;
        case Bandwidth_Factor_4:
           Filter_Config|=0x10;
            break;
        case Bandwidth_Factor_5:
           Filter_Config|=0x20;
            break;
        case Bandwidth_Factor_8:
           Filter_Config|=0x30;
            break;
        case Bandwidth_Factor_10:
           Filter_Config|=0x40;
            break;
        case Bandwidth_Factor_16:
           Filter_Config|=0x50;
            break;
        case Bandwidth_Factor_20:
           Filter_Config|=0x60;
            break;
        case Bandwidth_Factor_40:
           Filter_Config|=0x70;
            break;
        case Low_latency_1:
           Filter_Config|=0xE0;
            break;
        case Low_Latency_2:
           Filter_Config|=0xF0;
            break;
    }
		icm_reg_write(WRITE_GYRO_ACCEL_CONFIG0,Filter_Config);
    switch(Gyro_Filter_Order)
    {
        case _1st:
            Gyro_Filter_Ord_Config|=0x02;
            break;
        case _2st:
            Gyro_Filter_Ord_Config|=0x06;
            break;
        case _3st:
            Gyro_Filter_Ord_Config|=0xA0;
            break;
    }
		icm_reg_write(WRITE_GYRO_CONFIG1,Gyro_Filter_Ord_Config);
  
    switch(Acc_Filter_Order)
    {
        case _1st:
            Acc_Filter_Ord_Config|=0x02;
            break;
        case _2st:
            Acc_Filter_Ord_Config|=0x06;
            break;
        case _3st:
            Acc_Filter_Ord_Config|=0xA0;
            break;
    }
		icm_reg_write(WRITE_ACCEL_CONFIG1,Acc_Filter_Ord_Config);
}

//?????
static void Rest_ICM42688(void)
{
    uint8_t WRITE_PWR_MGMT0 = 0X4E;
	icm_reg_write(WRITE_PWR_MGMT0,0x00);
    
}

//??????????????????
static void Set_ICM42688_LN_Mode(void)
{
    uint8_t WRITE_PWR_MGMT0 = 0X4E;
	icm_reg_write(WRITE_PWR_MGMT0,0x0f);
 
}

//ICM42688???? ??8???????? ICM42688???????
void ICM42688_Get_Data(void)
{
    uint8_t LSB_Data[12];
	icm_reg_read(READ_ACC_X_HIGH,12,LSB_Data);
    ICM42688_RAW.acc_x_lsb = ((uint16_t)LSB_Data[0]<<8 |LSB_Data[1]);
    ICM42688_RAW.acc_y_lsb = ((uint16_t)LSB_Data[2]<<8 |LSB_Data[2]);
    ICM42688_RAW.acc_z_lsb = ((uint16_t)LSB_Data[4]<<8 |LSB_Data[5]);
    ICM42688_RAW.gyro_x_lsb = ((uint16_t)LSB_Data[6]<<8 |LSB_Data[7]);
    ICM42688_RAW.gyro_y_lsb = ((uint16_t)LSB_Data[8]<<8 |LSB_Data[9]);
    ICM42688_RAW.gyro_z_lsb = ((uint16_t)LSB_Data[10]<<8 |LSB_Data[11]);
    ICM42688.gyro_x = (int16_t)ICM42688_RAW.gyro_x_lsb/Gyro_Sensitivity;
    ICM42688.gyro_y = (int16_t)ICM42688_RAW.gyro_y_lsb/Gyro_Sensitivity;
    ICM42688.gyro_z = (int16_t)ICM42688_RAW.gyro_z_lsb/Gyro_Sensitivity;
    ICM42688.acc_x = (int16_t)ICM42688_RAW.acc_x_lsb/Acc_Sensitivity;
    ICM42688.acc_y = (int16_t)ICM42688_RAW.acc_y_lsb/Acc_Sensitivity;
    ICM42688.acc_z = (int16_t)ICM42688_RAW.acc_z_lsb/Acc_Sensitivity;
}



void icm42688_init(ICM42688_CONFIG_STRUCT*ICM42688_CONFIG)
{

    Rest_ICM42688();             //??ICM42688
    HAL_Delay(10);
    Find_ICM42688();             //????ID
    ICM42688_SET_GYRO((*ICM42688_CONFIG).GYRO_FSR,(*ICM42688_CONFIG).GYRO_ODR);          //???????????
    ICM42688_SET_ACC((*ICM42688_CONFIG).ACC_FSR,(*ICM42688_CONFIG).ACC_ODR);             //????????????
    ICM42688_SET_FILTER((*ICM42688_CONFIG).Gyro_Bandwidth_Factor,(*ICM42688_CONFIG).Gyro_Filter_Order,(*ICM42688_CONFIG).Acc_Bandwidth_Factor,(*ICM42688_CONFIG).Acc_Filter_Order);      //????????????
    HAL_Delay(10);
    Set_ICM42688_LN_Mode();     //??????????,????????
    HAL_Delay(10);
}




	void icm_task()
	{
		ICM42688_CONFIG_STRUCT ICM42688_CONFIG={GYRO_250DPS,GYRO_ODR_100HZ,ACC_8G,ACC_ODR_500HZ,_2st,Bandwidth_Factor_4,_2st,Bandwidth_Factor_4,Bias_On_Chip_Off};
icm42688_init(&ICM42688_CONFIG);
		
		while(1)
		{
			
		ICM42688_Get_Data();
					//printf("%f,%f,%f,",ICM42688.acc_x,ICM42688.acc_y,ICM42688.acc_z);
		
		//printf("%f,%f,%f,",ICM42688.gyro_x-0.1,ICM42688.gyro_y-0.1,ICM42688.gyro_z-0.1);
		vTaskDelay(pdMS_TO_TICKS(100));
		
		}
	}