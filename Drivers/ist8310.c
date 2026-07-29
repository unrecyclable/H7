#include "ist8310.h"
#include "main.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h" 
int read_register(uint8_t reg, uint8_t *value) {
    uint8_t status = i2c4_reg_receive(IST8310_I2C_ADDRESS,reg,value,1);
    if (status != 0) 
        return 1;
    
		else
				return 0;
}
int read_register_multi(uint8_t reg, uint8_t *value,int len) {
    uint8_t status = i2c4_reg_receive(IST8310_I2C_ADDRESS,reg,value,len);
    if (status != 0) 
        return 1;
    
		else
				return 0;
}
int write_register(uint8_t reg, uint8_t value) {
		uint8_t data=value; 
    uint8_t status = i2c4_reg_write(IST8310_I2C_ADDRESS,reg,&data,1);
    return status;
}

int _flip_x_y;
float _declination_offset_radians = 0.0;
   Vec3f _raw;
   Vec3f _magnetometer;
int ist8310_init() {

    _raw.x = 0.0;
    _raw.y = 0.0;
    _raw.z = 0.0;
    _flip_x_y = 0;

    int success = ist8310_soft_reset();
    if (success) {
        
        return 1;
    }
    return 0;
}

int ist8310_read_device_id() {
	uint8_t id;
    int success = read_register(IST8310_REGISTER_WHO_AM_I, &id);
    if (success) {
        return 1;
    }

    if (id != IST8310_DEVICE_ID) {
        return 1;
    }

    return 0;
}

int ist8310_soft_reset() {
    uint8_t reset_bit = 0xd;
    int success = write_register(IST8310_REGISTER_CNTL2, reset_bit);
    if (success) {
        
        return 1;
    }

    for (size_t i = 0; i < IST8310_RESET_RETRIES; i++) {
        success = ist8310_read_device_id();
        if (success) {
            continue;
        }

        uint8_t value;
        success = read_register(IST8310_REGISTER_CNTL2, &value);
        if (success) {
            continue;
        }

        if ((value & reset_bit) == 0) {
            return 0;
        }
    }

    return 1;
}



int ist8310_read() {
    int success = write_register(IST8310_REGISTER_CNTL1, 0x1);
    if (success) {
        return 1;
    }
		uint8_t data_ready=0;
		HAL_Delay(10);
		while(!(data_ready&0x01))
		{
			read_register(0x02,&data_ready);
			HAL_Delay(1);
		}
		uint8_t data[6];
		read_register_multi(0x03,data,6);


    // read low / high x
    int16_t x_low = data[0];
    int16_t x_high = data[1];

    int16_t x = (x_high << 8) | x_low;

    // read low / high y
    int16_t y_low = data[2];
    int16_t y_high = data[3];

    int16_t y = (y_high << 8) | y_low;

    // read low / high z
    int16_t z_low = data[4];
    int16_t z_high = data[5];

    int16_t z = (z_high << 8) | z_low;
		
    _raw.x = (float)(x)*0.048;
    _raw.y = (float)(y)*0.048;
    _raw.z = (float)(z)*0.076;

    return 0;
}

int ist8310_update() {
    int success = ist8310_read();
    if (success) {
        return 1;
    }

    float x = _raw.x;
    float y = _raw.y;

    // check if x and y needs to be fliped
    if (_flip_x_y) {
        x = _raw.y;
        y = _raw.x;
    }

    _magnetometer.x = x;
    _magnetometer.y = y;
    _magnetometer.z = _raw.z;

    return 0;
}

int ist8310_set_average(IST8310AverageY y, IST8310AverageXZ xz) {
    uint8_t value = ((uint8_t)(y) << 2) & (uint8_t)(xz);
    int success = write_register(IST8310_REGISTER_AVGCNTL, value);
    if (success) {
        return 1;
    }
    return 0;
}

Vec3f *get_raw() {
    return &_raw;
}

Vec3f *get_magnetometer() {
    return &_magnetometer;
}

int ist8310_set_selftest(int enabled) {
    uint8_t value = 0x0;
    if (enabled) {
        value = 1 << 6;
    }
    int success = write_register(IST8310_REGISTER_STR, value);
    if (success) {
        return 1;
    }
    return 0;
}

void ist8310_set_flip_x_y(int flip) {
    _flip_x_y = flip;
}

void ist8310_set_declination_offset_radians(float declination_radians) {
    _declination_offset_radians = declination_radians;
}

int ist8310_loop_read(Vec3f*sum, size_t loops, int timeout) {
    for (size_t i = 0; i < loops; i++) {
        int success = ist8310_read();
        if (success) {
            return 1;
        }

        sum->x += _raw.x;
        sum->y += _raw.y;
        sum->z += _raw.z;

        // _leds->sleep(timeout);
    }

    return 0;
}

int ist8310_calibration() {

    Vec3f test={.x=0.0, .y=0.0, .z=0.0};
	
    Vec3f normal={.x=0.0, .y=0.0, .z=0.0};

    Vec3f *tmp = &normal;

    for (size_t i = 0; i < 2; i++) {
        if (i == 1) {
            int success = ist8310_set_selftest(0);
            if (success) {
                return 1;
            }

            tmp = &test;
        }

        for (size_t j = 0; j < 30; j++) {
            int success = ist8310_read();
            if (success) {
                return 1;
            }

            if (j > 10) {
                tmp->x += _raw.x;
                tmp->y += _raw.y;
                tmp->z += _raw.z;
            }
        }
    }

    int success = ist8310_set_selftest(1);
    if (success) {
        return 1;
    }

    float x = fabsf(test.x - normal.x);
    float y = fabsf(test.y - normal.y);
    float z = fabsf(test.z - normal.z);
    float length = x + y + z;

    return 0;
}


float get_heading_degrees() {

    Vec3f *mag_latest_value = get_magnetometer();

    // I followed these calculations from https://github.com/sleemanj/HMC5883L_Simple
    float mag_west = 0 - mag_latest_value->x;
    float mag_north = mag_latest_value->y;

    float heading = atan2(mag_west, mag_north);

    // Adjust the heading by the declination
    heading += _declination_offset_radians;

    // Correct for when signs are reversed.
    if(heading < 0)
        heading += 2*M_PI;
        
    // Check for wrap due to addition of declination.
    if(heading > 2*M_PI)
        heading -= 2*M_PI;
    
    // Convert radians to degrees
    return heading * 180/M_PI; 
}

void ist8310_task()
{
	
ist8310_init();
	ist8310_set_average(IST8310_4_AVERAGE_Y,IST8310_4_AVERAGE_X_Z);
	while(1)
	{
	ist8310_update();
		//printf("%f,%f,%f\n",_magnetometer.x,_magnetometer.y,_magnetometer.z);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}