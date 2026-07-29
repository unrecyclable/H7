#include "stm32h7xx.h"
void icm_init();
extern uint16_t accx,accy,accz;
void icm_read_raw_acc_gyro();
extern uint16_t gyrox,gyroy,gyroz;
int icm_reg_read(uint8_t reg,int len,uint8_t*data);
#define ICM42688_ID           0X47                //ICM42688 ??ID
#define READ_ACC_X_HIGH       0X9F
#define WHO_AM_I              0xF500                          //ICM42688 ????ID??

#define SENSITIVITY_ICM42688_GYRO_15_625dps  2097.2           
#define SENSITIVITY_ICM42688_GYRO_31_25dps   1048.6
#define SENSITIVITY_ICM42688_GYRO_62_5dps    524.3
#define SENSITIVITY_ICM42688_GYRO_125dps     262.0
#define SENSITIVITY_ICM42688_GYRO_250dps     131.0
#define SENSITIVITY_ICM42688_GYRO_500dps     65.5
#define SENSITIVITY_ICM42688_GYRO_1000dps    32.8
#define SENSITIVITY_ICM42688_GYRO_2000dps    16.4

#define SENSITIVITY_ICM42688_ACC_16G  2048
#define SENSITIVITY_ICM42688_ACC_8G   4096
#define SENSITIVITY_ICM42688_ACC_4G   8192
#define SENSITIVITY_ICM42688_ACC_2G   16384
typedef struct ICM42688_RAW_DATA{

    uint16_t acc_x_lsb;
    uint16_t acc_y_lsb;
    uint16_t acc_z_lsb;
    uint16_t gyro_x_lsb;
    uint16_t gyro_y_lsb;
    uint16_t gyro_z_lsb;
    uint16_t temp_lsb; //??????
} ICM42688_RAW_DATA;//?????????LSB???? ??????????????

typedef struct ICM42688_real_data {

    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;//??????

} ICM42688_real_data;//??????????

typedef enum        //????????
{
    GYRO_ODR_12_5HZ,
    GYRO_ODR_25HZ,
    GYRO_ODR_50HZ,
    GYRO_ODR_100HZ,
    GYRO_ODR_200HZ,
    GYRO_ODR_500HZ,
    GYRO_ODR_1000HZ,
    GYRO_ODR_2000HZ,
    GYRO_ODR_4000HZ,
    GYRO_ODR_8000HZ,
    GYRO_ODR_16000HZ,
    GYRO_ODR_32000HZ,
}GYRO_ODR;          //ODR -->Output Data Rate

typedef enum        //?????????
{
    ACC_ODR_12_5HZ,
    ACC_ODR_25HZ,
    ACC_ODR_50HZ,
    ACC_ODR_100HZ,
    ACC_ODR_200HZ,
    ACC_ODR_500HZ,
    ACC_ODR_1000HZ,
    ACC_ODR_2000HZ,
    ACC_ODR_4000HZ,
    ACC_ODR_8000HZ,
    ACC_ODR_16000HZ,
    ACC_ODR_32000HZ,
}ACC_ODR;

typedef enum   //???????  DPS --> Degree Per Second
{
    GYRO_2000DPS,
    GYRO_1000DPS,
    GYRO_500DPS,
    GYRO_250DPS,
    GYRO_125DPS,
    GYRO_62_5DPS,
    GYRO_31_25DPS,
    GYRO_15_625DPS,
}GYRO_FSR;      //FSR --> Full Scan Range

typedef enum   //????????
{
    ACC_16G,
    ACC_8G,
    ACC_4G,
    ACC_2G,
}ACC_FSR;

typedef enum   //????????? ???? ??????
{
    _1st,      //??
    _2st,      //??
    _3st,      //??
}Filter_Order;

typedef enum   //??????  ?????????31 81? ???3dB??? ODR(???) ??????? ???? ???? ?????31??????
{
    Bandwidth_Factor_2,
    Bandwidth_Factor_4,
    Bandwidth_Factor_5,
    Bandwidth_Factor_8,
    Bandwidth_Factor_10,
    Bandwidth_Factor_16,
    Bandwidth_Factor_20,
    Bandwidth_Factor_40,
    Low_latency_1,
    Low_Latency_2,
}Bandwidth_Factor;

typedef enum   //?????????????? ??? ?????????? ????????????? ICM42688????? ??????
{
    Bias_On_Chip_On,
    Bias_On_Chip_Off,
}Bias_On_Chip;

typedef struct ICM42688_CONFIG_STRUCT
{
    GYRO_FSR GYRO_FSR;
    GYRO_ODR GYRO_ODR;
    ACC_FSR ACC_FSR;
    ACC_ODR ACC_ODR;
    Filter_Order Gyro_Filter_Order;
    Bandwidth_Factor Gyro_Bandwidth_Factor;
    Filter_Order Acc_Filter_Order;
    Bandwidth_Factor Acc_Bandwidth_Factor;
    Bias_On_Chip Bias_On_Chip;
}ICM42688_CONFIG_STRUCT;//ICM42688??????
//uint8 ICM42688_Get_Data_Flag = 0;
extern ICM42688_CONFIG_STRUCT ICM42688_CONFIG;
void icm42688_init(ICM42688_CONFIG_STRUCT*ICM42688_CONFIG);
void ICM42688_Get_Data(void);
extern ICM42688_real_data ICM42688;
extern uint8_t ICM42688_Get_Data_Flag;

void icm_task();