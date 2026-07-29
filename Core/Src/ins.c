#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "jy901.h"
//==============================================================================
// 常量定义
//==============================================================================
#define PI                    3.141592653589793f
#define DEG2RAD               (PI / 180.0f)
#define RAD2DEG               (180.0f / PI)
//#define GRAVITY               9.80665f      // 重力加速度 m/s^2
#define GRAVITY               9.7f      // 重力加速度 m/s^2
// 互补滤波系数
#define FILTER_ALPHA          1.0f         // 陀螺仪权重
#define FILTER_BETA           (1.0f - FILTER_ALPHA)

// 加速度计修正阈值 (判断是否静止)
#define ACCEL_STATIC_THRESH   0.5f          // m/s^2

//==============================================================================
// 数据结构定义
//==============================================================================

// 三轴向量
typedef struct {
    float x;
    float y;
    float z;
} Vec3f;

// 欧拉角 (东北天坐标系)
typedef struct {
    float roll;      // 横滚角 (弧度)
    float pitch;     // 俯仰角 (弧度)
    float yaw;       // 偏航角 (弧度)
} EulerAngles;

// 导航状态
typedef struct {
    Vec3f position;      // 位置 (东北天, 单位:米)
    Vec3f velocity;      // 速度 (东北天, 单位:米/秒)
    EulerAngles attitude; // 姿态角 (弧度)
} NavState;

//==============================================================================
// 惯性导航系统结构体
//==============================================================================
typedef struct {
    NavState state;          // 当前导航状态
    NavState state_init;     // 初始状态
    float dt;                // 采样间隔
    uint8_t is_initialized;  // 初始化标志
    
    // 中间变量
    Vec3f accel_nav;         // 导航系加速度
    Vec3f accel_linear;      // 线性加速度 (扣除重力)
} INS_Handle;

//==============================================================================
// 辅助函数
//==============================================================================

static Vec3f vec3_add(Vec3f a, Vec3f b) {
    Vec3f result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

static Vec3f vec3_sub(Vec3f a, Vec3f b) {
    Vec3f result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

static Vec3f vec3_scale(Vec3f v, float scale) {
    Vec3f result = {v.x * scale, v.y * scale, v.z * scale};
    return result;
}

static float vec3_norm(Vec3f v) {
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

static Vec3f vec3_normalize(Vec3f v) {
    float norm = vec3_norm(v);
    Vec3f result = {0, 0, 0};
    if (norm > 1e-6f) {
        result.x = v.x / norm;
        result.y = v.y / norm;
        result.z = v.z / norm;
    }
    return result;
}

// 角度归一化到 [-PI, PI]
static float normalize_angle(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

//==============================================================================
// 核心算法函数
//==============================================================================

/**
 * @brief 由加速度计计算姿态角 (仅用于静止/低速状态)
 */
static void accel_to_attitude(Vec3f accel, EulerAngles *attitude) {
    Vec3f acc_norm = vec3_normalize(accel);
    attitude->pitch = asinf(-acc_norm.x);
    attitude->roll = atan2f(acc_norm.y, acc_norm.z);
    // 航向角保持原值 (加速度计无法计算航向)
}

/**
 * @brief 构建从机体到导航系的旋转矩阵 (Z-Y-X内旋)
 */
static void build_rotation_matrix(EulerAngles attitude, float R[3][3]) {
    float cr = cosf(attitude.roll);
    float sr = sinf(attitude.roll);
    float cp = cosf(attitude.pitch);
    float sp = sinf(attitude.pitch);
    float cy = cosf(attitude.yaw);
    float sy = sinf(attitude.yaw);
    
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;
    
    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;
    
    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

/**
 * @brief 将机体加速度转换到导航系
 */
static Vec3f transform_to_navigation(Vec3f accel_body, EulerAngles attitude) {
    float R[3][3];
    build_rotation_matrix(attitude, R);
    
    Vec3f result;
    result.x = R[0][0] * accel_body.x + R[0][1] * accel_body.y + R[0][2] * accel_body.z;
    result.y = R[1][0] * accel_body.x + R[1][1] * accel_body.y + R[1][2] * accel_body.z;
    result.z = R[2][0] * accel_body.x + R[2][1] * accel_body.y + R[2][2] * accel_body.z;
    return result;
}

/**
 * @brief 姿态更新 (陀螺仪积分 + 加速度计修正)
 */
static void attitude_update(INS_Handle *ins, Vec3f gyro, Vec3f accel, float dt) {
    EulerAngles *att = &ins->state.attitude;
    
//    // ===== 1. 陀螺仪预测 =====
//    float sin_roll = sinf(att->roll);
//    float cos_roll = cosf(att->roll);
//    float tan_pitch = tanf(att->pitch);
//    float cos_pitch = cosf(att->pitch);
//    
//    // 避免奇异性 (当 pitch 接近 ±90°)
//    if (fabsf(cos_pitch) > 1e-6f) {
//        float roll_dot = gyro.x + gyro.y * sin_roll * tan_pitch + gyro.z * cos_roll * tan_pitch;
//        float pitch_dot = gyro.y * cos_roll - gyro.z * sin_roll;
//        float yaw_dot = (gyro.y * sin_roll + gyro.z * cos_roll) / cos_pitch;
//        
//        att->roll += roll_dot * dt;
//        att->pitch += pitch_dot * dt;
//        att->yaw += yaw_dot * dt;
//    } else {
//        // 奇异性保护: 只更新俯仰和偏航
//        att->pitch += (gyro.y * cos_roll - gyro.z * sin_roll) * dt;
//        att->yaw += (gyro.y * sin_roll + gyro.z * cos_roll) / cos_pitch * dt;
//    }
//    
//    // ===== 2. 加速度计修正 (仅修正横滚和俯仰) =====
//    float accel_mag = vec3_norm(accel);
//    if (fabsf(accel_mag - GRAVITY) < ACCEL_STATIC_THRESH) {
//        EulerAngles att_accel;
//        accel_to_attitude(accel, &att_accel);
//        
//        // 互补滤波
//        att->roll = FILTER_ALPHA * att->roll + FILTER_BETA * att_accel.roll;
//        att->pitch = FILTER_ALPHA * att->pitch + FILTER_BETA * att_accel.pitch;
//    }
    
    // 角度归一化
    att->roll = normalize_angle(att->roll);
    att->pitch = normalize_angle(att->pitch);
    att->yaw = normalize_angle(att->yaw);
}

/**
 * @brief 速度和位置更新 (梯形积分)
 */
static void velocity_position_update(INS_Handle *ins, Vec3f accel_body, float dt) {
    // 1. 将加速度转换到导航系 (使用当前欧拉角)
    Vec3f accel_nav = transform_to_navigation(accel_body, ins->state.attitude);
    
    // 2. 扣除重力 (东北天: 天向为 +g)
    Vec3f accel_linear;
    accel_linear.x = accel_nav.x;          // 东向
    accel_linear.y = accel_nav.y;          // 北向
    accel_linear.z = accel_nav.z + GRAVITY; // 天向 (减去 -g)
    
    ins->accel_nav = accel_nav;
    ins->accel_linear = accel_linear;
	
    accel_linear.x=((float)((int)(accel_linear.x*10)))/10;
		accel_linear.y=((float)((int)(accel_linear.y*10)))/10;
		accel_linear.z=((float)((int)(accel_linear.z*10)))/10;
			if(fabs(accel_linear.x)<0.11)
				{accel_linear.x=0;}
			if(fabs(accel_linear.y)<0.11)
				{accel_linear.y=0;}
			if(fabs(accel_linear.z)<0.11)
				{accel_linear.z=0;}
    // 3. 速度更新 (梯形积分)
    Vec3f vel_old = ins->state.velocity;
    Vec3f vel_new;
    vel_new.x = vel_old.x + accel_linear.x * dt;
    vel_new.y = vel_old.y + accel_linear.y * dt;
    vel_new.z = vel_old.z + accel_linear.z * dt;
//		printf("%f,%f,%f\n",vel_new.x,vel_new.y,vel_new.z);
//				if(fabs(vel_new.x)<0.11)
//				{vel_new.x=0;}
//			if(fabs(vel_new.y)<0.11)
//				{vel_new.y=0;}
//			if(fabs(vel_new.z)<0.11)
//				{vel_new.z=0;}
    // 4. 位置更新 (梯形积分)
    Vec3f pos_old = ins->state.position;
    Vec3f pos_new;
    pos_new.x = pos_old.x + (vel_old.x + vel_new.x) * 0.5f * dt;
    pos_new.y = pos_old.y + (vel_old.y + vel_new.y) * 0.5f * dt;
    pos_new.z = pos_old.z + (vel_old.z + vel_new.z) * 0.5f * dt;


		//printf("%f,%f,%f,%f,%f,%f\n",pos_new.x,pos_new.y,pos_new.z,roll,pitch,yaw);
    ins->state.velocity = vel_new;
    ins->state.position = pos_new;
}

//==============================================================================
// ★★★ 对外接口函数 (清晰明确的输入接口) ★★★
//==============================================================================

/**
 * @brief 初始化惯性导航系统
 * @param ins 系统句柄
 * @param init_attitude 初始姿态角 (弧度) [roll, pitch, yaw]
 * @param init_position 初始位置 (米) [东, 北, 天]
 * @param init_velocity 初始速度 (米/秒) [东, 北, 天]
 * @param dt 采样间隔 (秒)
 */
void ins_init(INS_Handle *ins, EulerAngles init_attitude, 
              Vec3f init_position, Vec3f init_velocity, float dt) {
    // 保存初始状态
    ins->state_init.attitude = init_attitude;
    ins->state_init.position = init_position;
    ins->state_init.velocity = init_velocity;
    
    // 复制到当前状态
    ins->state.attitude = init_attitude;
    ins->state.position = init_position;
    ins->state.velocity = init_velocity;
    
    ins->dt = dt;
    ins->is_initialized = 1;
    
    // 清空中间变量
    ins->accel_nav = (Vec3f){0, 0, 0};
    ins->accel_linear = (Vec3f){0, 0, 0};
}

/**
 * @brief 【核心接口】输入姿态角 + IMU数据，更新导航状态
 * @param ins 系统句柄
 * @param attitude 当前时刻的外部姿态角 (弧度) [roll, pitch, yaw]
 * @param gyro 三轴角速度 (弧度/秒) [x, y, z]
 * @param accel 三轴加速度 (米/秒^2) [x, y, z]
 * @param dt 采样间隔 (秒)
 * @return 当前导航状态指针
 * 
 * @note 使用说明:
 *       1. attitude 是外部传感器(如磁力计/视觉)测量的姿态角
 *       2. 如果外部姿态不可靠，可以传入 {0,0,0}，系统会使用IMU积分
 *       3. 外部姿态会与IMU积分结果进行互补滤波融合
 */
NavState* ins_update_with_attitude(INS_Handle *ins, EulerAngles attitude,
                                   Vec3f gyro, Vec3f accel, float dt) {
    if (!ins->is_initialized) {
        return &ins->state;
    }
    
    // ===== 1. 外部姿态修正 =====
    // 使用外部姿态与当前姿态进行互补滤波融合
    // 这样可以平滑地引入外部姿态，避免突变
    EulerAngles *att = &ins->state.attitude;
    
    // 计算姿态差 (考虑角度环绕)
    float roll_diff = attitude.roll - att->roll;
    float pitch_diff = attitude.pitch - att->pitch;
    float yaw_diff = attitude.yaw - att->yaw;
    
    roll_diff = normalize_angle(roll_diff);
    pitch_diff = normalize_angle(pitch_diff);
    yaw_diff = normalize_angle(yaw_diff);
    
    // 外部姿态权重 (可根据外部传感器质量调整)
    // 这里使用 0.1，表示10%信任外部，90%信任IMU积分
    float ext_weight = 0;
    att->roll += roll_diff * ext_weight;
    att->pitch += pitch_diff * ext_weight;
    att->yaw += yaw_diff * ext_weight;
    
    att->roll = normalize_angle(att->roll);
    att->pitch = normalize_angle(att->pitch);
    att->yaw = normalize_angle(att->yaw);
    
    // ===== 2. IMU姿态更新 (陀螺仪积分 + 加速度计修正) =====
    attitude_update(ins, gyro, accel, dt);
    
    // ===== 3. 速度和位置更新 =====
    velocity_position_update(ins, accel, dt);
    
    return &ins->state;
}

/**
 * @brief 【简化接口】仅输入IMU数据 (不使用外部姿态)
 * @param ins 系统句柄
 * @param gyro 三轴角速度 (弧度/秒)
 * @param accel 三轴加速度 (米/秒^2)
 * @param dt 采样间隔 (秒)
 * @return 当前导航状态指针
 * 
 * @note 此接口不使用外部姿态，完全依赖IMU积分
 *       适用于短时间高精度导航
 */
NavState* ins_update_imu_only(INS_Handle *ins, Vec3f gyro, Vec3f accel, float dt) {
    if (!ins->is_initialized) {
        return &ins->state;
    }
    
    // 姿态更新 (陀螺仪积分 + 加速度计修正)
    attitude_update(ins, gyro, accel, dt);
    
    // 速度和位置更新
    velocity_position_update(ins, accel, dt);
    
    return &ins->state;
}

/**
 * @brief 【最简化接口】直接传入数值 (便于快速调用)
 * @param ins 系统句柄
 * @param roll, pitch, yaw 外部姿态角 (弧度)
 * @param gx, gy, gz 角速度 (弧度/秒)
 * @param ax, ay, az 加速度 (米/秒^2)
 * @param dt 采样间隔 (秒)
 * @return 当前导航状态指针
 */
NavState* ins_update_values(INS_Handle *ins, 
                            float roll, float pitch, float yaw,
                            float gx, float gy, float gz,
                            float ax, float ay, float az, 
                            float dt) {
    EulerAngles attitude = {roll, pitch, yaw};
    Vec3f gyro = {gx, gy, gz};
    Vec3f accel = {ax, ay, az};
    
    return ins_update_with_attitude(ins, attitude, gyro, accel, dt);
}

/**
 * @brief 获取当前导航状态
 */
NavState ins_get_state(INS_Handle *ins) {
    return ins->state;
}

/**
 * @brief 获取当前姿态角指针
 */
EulerAngles* ins_get_attitude_ptr(INS_Handle *ins) {
    return &ins->state.attitude;
}

/**
 * @brief 重置导航状态到初始值
 */
void ins_reset(INS_Handle *ins) {
    if (ins->is_initialized) {
        ins->state = ins->state_init;
    }
}

/**
 * @brief 设置外部姿态融合权重
 * @param weight 0~1之间，0表示完全信任IMU，1表示完全信任外部
 */
void ins_set_external_attitude_weight(INS_Handle *ins, float weight) {
    // 这个权重在 ins_update_with_attitude 中使用
    // 为简单起见，直接修改函数内的宏定义，或使用全局变量
    // 这里提供接口占位，实际使用可以扩展
    (void)ins;
    (void)weight;
}

//==============================================================================
// 使用示例
//==============================================================================

int ins_task() {
    // 1. 定义系统句柄
    INS_Handle ins;
    
    // 2. 设置初始状态
    EulerAngles init_att = {0.0f, 0.0f, 0.0f};  // 水平朝北
    Vec3f init_pos = {0.0f, 0.0f, 0.0f};        // 原点
    Vec3f init_vel = {0.0f, 0.0f, 0.0f};        // 静止
    float dt = 0.02f;                            // 
    
    // 3. 初始化
    ins_init(&ins, init_att, init_pos, init_vel, dt);
    
 
    while (1) {
        // ============================================================
        // 【数据输入】从你的传感器读取数据
        // ============================================================
        
        // 方式A: 如果有外部姿态 (例如来自磁力计或视觉)
//        float ext_roll = 0.0f;    // 外部测量的横滚角 (弧度)
//        float ext_pitch = 0.0f;   // 外部测量的俯仰角 (弧度)
//        float ext_yaw = 0.0f;     // 外部测量的偏航角 (弧度)
//        
//        // IMU原始数据 (已校准)
//        float gyro_x = 0.001f;    // 角速度 (弧度/秒)
//        float gyro_y = 0.002f;
//        float gyro_z = 0.003f;
//        float acc_x = 0.01f;      // 加速度 (米/秒^2)
//        float acc_y = 0.02f;
//        float acc_z = -GRAVITY + 0.01f;  // 水平静止时 Z轴为 -g
        
        // ============================================================
        // 【接口调用】三种方式任选其一
        // ============================================================
        
        // 方式1: 完整接口 (姿态角 + IMU数据)
        // EulerAngles ext_att = {ext_roll, ext_pitch, ext_yaw};
        // Vec3f gyro = {gyro_x, gyro_y, gyro_z};
        // Vec3f accel = {acc_x, acc_y, acc_z};
        // NavState* state = ins_update_with_attitude(&ins, ext_att, gyro, accel, dt);
        
        // 方式2: 仅IMU (不使用外部姿态)
        // Vec3f gyro = {gyro_x, gyro_y, gyro_z};
        // Vec3f accel = {acc_x, acc_y, acc_z};
        // NavState* state = ins_update_imu_only(&ins, gyro, accel, dt);
        
        // 方式3: 最简化接口 (直接传数值)
//        NavState* state = ins_update_values(&ins, 
//                                            roll/180*3.1415, pitch/180*3.1415, yaw/180*3.1415,
//                                            angx, angy, angz,
//                                            accx, accy, accz, 
//                                            dt);
NavState* state = ins_update_values(&ins, 
                                            roll/180*3.1415, pitch/180*3.1415, yaw/180*3.1415,
                                            0, 0, 0,
                                            accx, accy, accz, 
                                            dt);
   
       
//        printf("  位置: [%.3f, %.3f, %.3f] m\n", 
//               state->position.x, state->position.y, state->position.z);
//        printf("  速度: [%.3f, %.3f, %.3f] m/s\n", 
//               state->velocity.x, state->velocity.y, state->velocity.z);
//        printf("  姿态: [%.2f, %.2f, %.2f] deg\n\n",
//               state->attitude.roll * RAD2DEG,
//               state->attitude.pitch * RAD2DEG,
//               state->attitude.yaw * RAD2DEG);
//          printf("%f,%f\n", state->position.x, state->position.y);

//printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",accx,accy,accz,angx,angy,angz,roll/180*3.1415,pitch/180*3.1415,yaw/180*3.1415,state->position.x, state->position.y);
				vTaskDelay(20);
        
    }
    
    return 0;
}