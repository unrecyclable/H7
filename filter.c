#include <stdio.h>
#include "filter.h"
#include <math.h>

// 数学常量
#define PI 3.141592653589793f
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)

// 滤波器参数
#define SAMPLE_FREQ 100.0f      // 采样频率(Hz)
#define TWO_KP 2.0f * 1.0f      // 比例增益(Kp)
#define TWO_KI 2.0f * 0.05f     // 积分增益(Ki)
MahonyFilter filter;
SensorData sensor;


// 初始化滤波器
void Mahony_Init(MahonyFilter *filter) {
    filter->q0 = 1.0f;
    filter->q1 = 0.0f;
    filter->q2 = 0.0f;
    filter->q3 = 0.0f;
    filter->integralFBx = 0.0f;
    filter->integralFBy = 0.0f;
    filter->integralFBz = 0.0f;
    filter->roll = 0.0f;
    filter->pitch = 0.0f;
    filter->yaw = 0.0f;
}

// 快速平方根倒数 (Quake III算法)
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// Mahony互补滤波更新 (加速度计+地磁计+陀螺仪)
void Mahony_Update(MahonyFilter *filter, SensorData *data, float dt) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float halfT = 0.5f * dt;

    // 加速度计归一化
    recipNorm = invSqrt(data->ax * data->ax + data->ay * data->ay + data->az * data->az);
    data->ax *= recipNorm;
    data->ay *= recipNorm;
    data->az *= recipNorm;

    // 地磁计归一化
    recipNorm = invSqrt(data->mx * data->mx + data->my * data->my + data->mz * data->mz);
    data->mx *= recipNorm;
    data->my *= recipNorm;
    data->mz *= recipNorm;

    // 四元数乘积预计算
    q0q0 = filter->q0 * filter->q0;
    q0q1 = filter->q0 * filter->q1;
    q0q2 = filter->q0 * filter->q2;
    q0q3 = filter->q0 * filter->q3;
    q1q1 = filter->q1 * filter->q1;
    q1q2 = filter->q1 * filter->q2;
    q1q3 = filter->q1 * filter->q3;
    q2q2 = filter->q2 * filter->q2;
    q2q3 = filter->q2 * filter->q3;
    q3q3 = filter->q3 * filter->q3;

    // 将地磁矢量从载体坐标系转换到导航坐标系 (h = R * m)
    hx = 2.0f * (data->mx * (0.5f - q2q2 - q3q3) +
                 data->my * (q1q2 - q0q3) +
                 data->mz * (q1q3 + q0q2));
    hy = 2.0f * (data->mx * (q1q2 + q0q3) +
                 data->my * (0.5f - q1q1 - q3q3) +
                 data->mz * (q2q3 - q0q1));
    hz = 2.0f * (data->mx * (q1q3 - q0q2) +
                 data->my * (q2q3 + q0q1) +
                 data->mz * (0.5f - q1q1 - q2q2));

    // 计算参考地磁方向 (只使用水平分量)
    bx = sqrtf(hx * hx + hy * hy);
    bz = hz;

    // 将参考地磁矢量旋转回载体坐标系 (w = R^T * b)
    wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) +
                 bz * (q1q3 - q0q2));
    wy = 2.0f * (bx * (q1q2 - q0q3) +
                 bz * (q2q3 + q0q1));
    wz = 2.0f * (bx * (q1q3 + q0q2) +
                 bz * (0.5f - q1q1 - q2q2));

    // 估计重力矢量 (v = R^T * g, g=[0,0,1])
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q2q3 + q0q1);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // 误差 = 测量矢量 × 估计矢量 (叉积)
    ex = (data->ay * vz - data->az * vy) + (data->my * wz - data->mz * wy);
    ey = (data->az * vx - data->ax * vz) + (data->mz * wx - data->mx * wz);
    ez = (data->ax * vy - data->ay * vx) + (data->mx * wy - data->my * wx);

    // 积分误差
    filter->integralFBx += TWO_KI * ex * dt;
    filter->integralFBy += TWO_KI * ey * dt;
    filter->integralFBz += TWO_KI * ez * dt;

    // 反馈修正陀螺仪误差
    float gx = data->gx + TWO_KP * ex + filter->integralFBx;
    float gy = data->gy + TWO_KP * ey + filter->integralFBy;
    float gz = data->gz + TWO_KP * ez + filter->integralFBz;

    // 四元数微分方程更新
    float qDot0 = -halfT * (filter->q1 * gx + filter->q2 * gy + filter->q3 * gz);
    float qDot1 =  halfT * (filter->q0 * gx + filter->q2 * gz - filter->q3 * gy);
    float qDot2 =  halfT * (filter->q0 * gy - filter->q1 * gz + filter->q3 * gx);
    float qDot3 =  halfT * (filter->q0 * gz + filter->q1 * gy - filter->q2 * gx);

    filter->q0 += qDot0;
    filter->q1 += qDot1;
    filter->q2 += qDot2;
    filter->q3 += qDot3;

    // 归一化四元数
    recipNorm = invSqrt(filter->q0 * filter->q0 +
                        filter->q1 * filter->q1 +
                        filter->q2 * filter->q2 +
                        filter->q3 * filter->q3);
    filter->q0 *= recipNorm;
    filter->q1 *= recipNorm;
    filter->q2 *= recipNorm;
    filter->q3 *= recipNorm;
}

// 四元数转欧拉角 (ZYX顺序: Yaw, Pitch, Roll)
void QuaternionToEuler(MahonyFilter *filter) {
    float q0 = filter->q0, q1 = filter->q1, q2 = filter->q2, q3 = filter->q3;
    
    // Roll (X轴)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    filter->roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch (Y轴)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        filter->pitch = copysignf(PI / 2.0f, sinp);
    else
        filter->pitch = asinf(sinp);
    
    // Yaw (Z轴)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    filter->yaw = atan2f(siny_cosp, cosy_cosp);
}
/*
// 使用示例
int main() {
    MahonyFilter filter;
    SensorData sensor;
    float dt = 1.0f / SAMPLE_FREQ;
    
    Mahony_Init(&filter);
    
    // 模拟传感器数据 (示例: 静止水平朝北)
    sensor.ax = 0.0f;    sensor.ay = 0.0f;    sensor.az = 9.81f;
    sensor.gx = 0.0f;    sensor.gy = 0.0f;    sensor.gz = 0.0f;
    sensor.mx = 30.0f;   sensor.my = 0.0f;    sensor.mz = -50.0f;
    
    for (int i = 0; i < 1000; i++) {
        Mahony_Update(&filter, &sensor, dt);
        QuaternionToEuler(&filter);
        
        printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n",
               filter.roll * RAD_TO_DEG,
               filter.pitch * RAD_TO_DEG,
               filter.yaw * RAD_TO_DEG);
    }
    
    return 0;
}*/