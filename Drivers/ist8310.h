#ifndef _IST_8310_H
#define _IST_8310_H

#include "main.h"
#include <math.h>
#define M_PI 3.1415926

#define IST8310_DEVICE_ID 0x10
#define IST8310_I2C_ADDRESS 0x0E

#define IST8310_RESET_RETRIES 10
#define IST8310_CALIBRATION_READINGS 10

#define IST8310_REGISTER_WHO_AM_I 0x00
#define IST8310_REGISTER_CNTL1 0x0A
#define IST8310_REGISTER_CNTL2 0x0B
#define IST8310_REGISTER_STR 0x0C
#define IST8310_REGISTER_AVGCNTL 0x41

typedef struct 
{
    float x;
    float y;
    float z;
}Vec3f;
extern Vec3f _raw;
extern Vec3f _magnetometer;

typedef enum  {
    IST8310_NO_AVERAGE_Y = 0x000,
    IST8310_2_AVERAGE_Y = 0x001,
    IST8310_4_AVERAGE_Y = 0x010,
    IST8310_8_AVERAGE_Y = 0x011,
    IST8310_16_AVERAGE_Y = 0x100
}IST8310AverageY;

typedef enum  {
    IST8310_NO_AVERAGE_X_Z = 0x000,
    IST8310_2_AVERAGE_X_Z = 0x001,
    IST8310_4_AVERAGE_X_Z = 0x010,
    IST8310_8_AVERAGE_X_Z = 0x011,
    IST8310_16_AVERAGE_X_Z = 0x100
}IST8310AverageXZ;

    // int setup(TwoWire *wire, Print *print, Leds *leds);
int ist8310_init();
    int ist8310_soft_reset();
    int ist8310_update();
    int ist8310_calibration();
    int ist8310_set_average(IST8310AverageY y, IST8310AverageXZ xz);

    void ist8310_set_flip_x_y(int flip);
    void ist8310_set_declination_offset_radians(float declination_radians);

   

    float ist8310_get_heading_degrees();

    int ist8310_read();
    int ist8310_loop_read(Vec3f*sum, size_t loops, int timeout);

    int ist8310_read_device_id();
    int ist8310_set_selftest(int enabled);


    extern int _flip_x_y;

    extern float _declination_offset_radians;
		
		int write_register(uint8_t reg, uint8_t value);
		int read_register(uint8_t reg, uint8_t *value);
		int read_register_multi(uint8_t reg, uint8_t *value,int len);


 void ist8310_task();


#endif // _IST_8310_H