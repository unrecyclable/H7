#include "stm32h7xx.h"
// ???????
typedef struct {
    float q0, q1, q2, q3;       // ???
    float integralFBx, integralFBy, integralFBz; // ????
    float roll, pitch, yaw;     // ???(??)
} MahonyFilter;

// ????????
typedef struct {
    float ax, ay, az;   // ???? (m/s^2)
    float gx, gy, gz;   // ??? (rad/s)
    float mx, my, mz;   // ??? (uT)
} SensorData;

extern MahonyFilter filter;
extern SensorData sensor;
void Mahony_Init(MahonyFilter *filter);
void Mahony_Update(MahonyFilter *filter, SensorData *data, float dt);
void QuaternionToEuler(MahonyFilter *filter);