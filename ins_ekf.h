#ifndef INS_EKF_H
#define INS_EKF_H

#include <stdint.h>
#include <math.h>

// ======================== 配置宏 ========================
#define IMU_DT         0.05f   // 20Hz采样周期 (50ms)
#define GRAVITY        9.80665f
#define PI             3.141592653589793f

// 传感器噪声参数（根据JY901S和深度计调整）
#define ACCEL_NOISE_STD     0.01f   // m/s^2
#define GYRO_NOISE_STD      0.01f   // rad/s
#define ACCEL_BIAS_WALK     0.001f  // m/s^2/sqrt(s)
#define GYRO_BIAS_WALK      0.001f  // rad/s/sqrt(s)
#define ATTITUDE_NOISE_STD  0.02f   // rad (约1.15°)
#define DEPTH_NOISE_STD     0.005f  // m (0.2mbar≈2mm，此处稍保守)
#define NHC_VEL_NOISE_STD   0.1f    // m/s (运动约束速度噪声)

// 低通滤波系数（截止频率约5Hz，dt=0.05s）
#define LPF_ALPHA           0.3f    // 可调，0.2~0.5

// ======================== 数据结构 ========================
typedef struct {
    float p[3];      // 位置 (m)
    float v[3];      // 速度 (m/s)
    float q[4];      // 姿态四元数 [q0,q1,q2,q3]
    float bg[3];     // 陀螺零偏 (rad/s)
    float ba[3];     // 加速度计零偏 (m/s^2)
} INS_State_t;

typedef struct {
    float P[225];    // 协方差矩阵 15x15 (行主序)
    float Q[225];    // 过程噪声
    float R_att[9];  // 姿态量测噪声 3x3
    float R_depth;   // 深度量测噪声
    float R_nhc[4];  // 运动约束噪声 2x2
} EKF_Matrix_t;

typedef struct {
    float accel[3];    // 体坐标系加速度 (m/s^2)
    float gyro[3];     // 体坐标系角速度 (rad/s)
    float euler[3];    // JY901S输出的欧拉角 [roll, pitch, yaw] (rad)
    float depth;       // 深度 (m)
    uint8_t depth_new; // 1表示有新深度数据
} SensorData_t;

// ======================== 全局变量声明 ========================
extern INS_State_t ins;
extern EKF_Matrix_t ekf;
extern SensorData_t sensor;
extern uint8_t enable_nhc;   // 运动约束使能 (1启用, 0禁用)

// ======================== 公共接口 ========================
void INS_EKF_Init(void);                                 // 初始化
void INS_EKF_Step(float dt);                             // 主步进 (dt=0.05f)
void INS_EKF_SetSensorData(float acc[3], float gyro[3], float euler[3]); // 设置IMU数据（内部含低通滤波）
void INS_EKF_SetDepth(float depth);                      // 设置深度数据
void INS_EKF_GetResult(float p[3], float v[3], float euler[3]); // 获取导航结果
void ins_task();
#endif