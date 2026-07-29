#include "ins_ekf.h"
#include <string.h>
#include <stdlib.h>
#include "jy901.h"
#include "ms5837_uart.h"
#include "FreeRTOS.h"
#include "task.h"
// ======================== 全局变量 ========================
INS_State_t ins;
EKF_Matrix_t ekf;
SensorData_t sensor;
uint8_t enable_nhc = 1;

// 低通滤波状态（静态）
static float lpf_accel[3] = {0};
static float lpf_gyro[3]  = {0};

// ======================== 辅助函数声明 ========================
static void quat_multiply(const float *q1, const float *q2, float *q_out);
static void quat_normalize(float *q);
static void euler_to_quat(const float *euler, float *q);
static void quat_to_euler(const float *q, float *euler);
static void quat_to_dcm(const float *q, float *Cbn);
static void skew_matrix(const float *v, float *Sk);
static void matrix_multiply(const float *A, const float *B, float *C, int rowsA, int colsA, int colsB);
static void matrix_transpose(const float *A, float *At, int rows, int cols);
static void matrix_identity(float *A, int n);

// ======================== 初始化 ========================
void INS_EKF_Init(void) {
    // 初始状态（水平静止，航向0）
    float init_euler[3] = {0.0f, 0.0f, 0.0f};
    euler_to_quat(init_euler, ins.q);
    memset(ins.p, 0, sizeof(ins.p));
    memset(ins.v, 0, sizeof(ins.v));
    memset(ins.bg, 0, sizeof(ins.bg));
    memset(ins.ba, 0, sizeof(ins.ba));

    // 初始协方差 P
    matrix_identity(ekf.P, 15);
    for (int i=0; i<3; i++) {
        ekf.P[i*15+i] = 1.0f;
        ekf.P[(3+i)*15 + (3+i)] = 0.5f;
        ekf.P[(6+i)*15 + (6+i)] = 0.1f;
        ekf.P[(9+i)*15 + (9+i)] = 0.01f;
        ekf.P[(12+i)*15 + (12+i)] = 0.01f;
    }

    // 过程噪声 Q（针对20Hz适当增大）
    float accel_var = ACCEL_NOISE_STD * ACCEL_NOISE_STD;
    float gyro_var  = GYRO_NOISE_STD * GYRO_NOISE_STD;
    float bg_var    = GYRO_BIAS_WALK * GYRO_BIAS_WALK;
    float ba_var    = ACCEL_BIAS_WALK * ACCEL_BIAS_WALK;
    matrix_identity(ekf.Q, 15);
    for (int i=0; i<3; i++) {
        ekf.Q[i*15+i] = 1e-6f;
        ekf.Q[(3+i)*15 + (3+i)] = 1e-2f;   // 速度项放大（适应20Hz）
        ekf.Q[(6+i)*15 + (6+i)] = 1e-4f;   // 姿态项放大
        ekf.Q[(9+i)*15 + (9+i)] = bg_var;
        ekf.Q[(12+i)*15 + (12+i)] = ba_var;
    }

    // 量测噪声
    float att_var = ATTITUDE_NOISE_STD * ATTITUDE_NOISE_STD;
    matrix_identity(ekf.R_att, 3);
    for (int i=0; i<3; i++) ekf.R_att[i*3+i] = att_var;
    ekf.R_depth = DEPTH_NOISE_STD * DEPTH_NOISE_STD;

    float nhc_var = NHC_VEL_NOISE_STD * NHC_VEL_NOISE_STD;
    ekf.R_nhc[0] = nhc_var; ekf.R_nhc[1] = 0.0f;
    ekf.R_nhc[2] = 0.0f; ekf.R_nhc[3] = nhc_var;

    sensor.depth_new = 0;
    memset(lpf_accel, 0, sizeof(lpf_accel));
    memset(lpf_gyro, 0, sizeof(lpf_gyro));
}

// ======================== 机械编排 ========================
static void INS_Mechanization(float dt) {
    float gyro[3] = {sensor.gyro[0] - ins.bg[0],
                     sensor.gyro[1] - ins.bg[1],
                     sensor.gyro[2] - ins.bg[2]};
    float acc[3] = {sensor.accel[0] - ins.ba[0],
                    sensor.accel[1] - ins.ba[1],
                    sensor.accel[2] - ins.ba[2]};

    // 姿态更新
    float ang_inc[3] = {gyro[0]*dt, gyro[1]*dt, gyro[2]*dt};
    float norm = sqrtf(ang_inc[0]*ang_inc[0] + ang_inc[1]*ang_inc[1] + ang_inc[2]*ang_inc[2]);
    float q_omega[4];
    if (norm < 1e-12f) {
        q_omega[0] = 1.0f; q_omega[1] = 0.0f; q_omega[2] = 0.0f; q_omega[3] = 0.0f;
    } else {
        float half = 0.5f * norm;
        q_omega[0] = cosf(half);
        float s = sinf(half) / norm;
        q_omega[1] = ang_inc[0] * s;
        q_omega[2] = ang_inc[1] * s;
        q_omega[3] = ang_inc[2] * s;
    }
    float q_new[4];
    quat_multiply(ins.q, q_omega, q_new);
    quat_normalize(q_new);
    memcpy(ins.q, q_new, 4*sizeof(float));

    // 转换到导航系
    float Cbn[9];
    quat_to_dcm(ins.q, Cbn);
    float acc_nav[3];
    for (int i=0; i<3; i++) {
        acc_nav[i] = Cbn[i*3+0]*acc[0] + Cbn[i*3+1]*acc[1] + Cbn[i*3+2]*acc[2];
    }
    acc_nav[2] += GRAVITY;

    // 速度、位置更新
    float v_new[3];
    for (int i=0; i<3; i++) v_new[i] = ins.v[i] + acc_nav[i] * dt;
    for (int i=0; i<3; i++) ins.p[i] += (ins.v[i] + v_new[i]) * 0.5f * dt;
    memcpy(ins.v, v_new, 3*sizeof(float));
}

// ======================== EKF 预测 ========================
static void EKF_Predict(float dt) {
    float F[225] = {0};
    float Cbn[9];
    quat_to_dcm(ins.q, Cbn);
    float f_body[3] = {sensor.accel[0] - ins.ba[0],
                       sensor.accel[1] - ins.ba[1],
                       sensor.accel[2] - ins.ba[2]};
    float f_skew[9];
    skew_matrix(f_body, f_skew);

    for (int i=0; i<3; i++) F[i*15 + 3 + i] = 1.0f;
    float tmp[9];
    matrix_multiply(Cbn, f_skew, tmp, 3, 3, 3);
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            F[(3+i)*15 + 6+j] = -tmp[i*3+j];
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            F[(3+i)*15 + 12+j] = Cbn[i*3+j];
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            F[(6+i)*15 + 9+j] = -Cbn[i*3+j];

    float Phi[225];
    matrix_identity(Phi, 15);
    for (int i=0; i<15; i++)
        for (int j=0; j<15; j++)
            Phi[i*15+j] += F[i*15+j] * dt;

    // G (15x12)
    float G[180] = {0};
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            G[(3+i)*12 + j] = Cbn[i*3+j];
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            G[(6+i)*12 + 3+j] = -Cbn[i*3+j];
    for (int i=0; i<3; i++) G[(9+i)*12 + 6+i] = 1.0f;
    for (int i=0; i<3; i++) G[(12+i)*12 + 9+i] = 1.0f;

    float Qc[144] = {0};
    float accel_var = ACCEL_NOISE_STD*ACCEL_NOISE_STD;
    float gyro_var  = GYRO_NOISE_STD*GYRO_NOISE_STD;
    float bg_var    = GYRO_BIAS_WALK*GYRO_BIAS_WALK;
    float ba_var    = ACCEL_BIAS_WALK*ACCEL_BIAS_WALK;
    for (int i=0; i<3; i++) { Qc[i*12+i] = accel_var; }
    for (int i=0; i<3; i++) { Qc[(3+i)*12 + 3+i] = gyro_var; }
    for (int i=0; i<3; i++) { Qc[(6+i)*12 + 6+i] = bg_var; }
    for (int i=0; i<3; i++) { Qc[(9+i)*12 + 9+i] = ba_var; }

    float temp[225];
    matrix_multiply(G, Qc, temp, 15, 12, 12);
    float Gt[180];
    matrix_transpose(G, Gt, 15, 12);
    float Qd[225];
    matrix_multiply(temp, Gt, Qd, 15, 12, 15);
    for (int i=0; i<225; i++) Qd[i] *= dt;

    float temp2[225];
    matrix_multiply(Phi, ekf.P, temp2, 15, 15, 15);
    float Phit[225];
    matrix_transpose(Phi, Phit, 15, 15);
    float temp3[225];
    matrix_multiply(temp2, Phit, temp3, 15, 15, 15);
    for (int i=0; i<225; i++) ekf.P[i] = temp3[i] + Qd[i];
}

// ======================== EKF 姿态更新 ========================
static void EKF_Update_Attitude(void) {
    float euler_pred[3];
    quat_to_euler(ins.q, euler_pred);
    float y[3] = {sensor.euler[0] - euler_pred[0],
                  sensor.euler[1] - euler_pred[1],
                  sensor.euler[2] - euler_pred[2]};
    for (int i=0; i<3; i++) {
        while (y[i] > PI) y[i] -= 2*PI;
        while (y[i] < -PI) y[i] += 2*PI;
    }

    float H[45] = {0};
    for (int i=0; i<3; i++) H[i*15 + 6 + i] = 1.0f;

    float Ht[45];
    matrix_transpose(H, Ht, 3, 15);
    float temp[45];
    matrix_multiply(H, ekf.P, temp, 3, 15, 15);
    float S[9];
    matrix_multiply(temp, Ht, S, 3, 15, 3);
    for (int i=0; i<9; i++) S[i] += ekf.R_att[i];

    // 3x3逆
    float det = S[0]*(S[4]*S[8]-S[5]*S[7]) - S[1]*(S[3]*S[8]-S[5]*S[6]) + S[2]*(S[3]*S[7]-S[4]*S[6]);
    if (fabs(det) < 1e-12f) return;
    float invDet = 1.0f/det;
    float S_inv[9] = {
        (S[4]*S[8]-S[5]*S[7])*invDet, (S[2]*S[7]-S[1]*S[8])*invDet, (S[1]*S[5]-S[2]*S[4])*invDet,
        (S[5]*S[6]-S[3]*S[8])*invDet, (S[0]*S[8]-S[2]*S[6])*invDet, (S[2]*S[3]-S[0]*S[5])*invDet,
        (S[3]*S[7]-S[4]*S[6])*invDet, (S[1]*S[6]-S[0]*S[7])*invDet, (S[0]*S[4]-S[1]*S[3])*invDet
    };

    float K[45];
    float temp2[45];
    matrix_multiply(ekf.P, Ht, temp2, 15, 15, 3);
    matrix_multiply(temp2, S_inv, K, 15, 3, 3);

    float delta_x[15] = {0};
    for (int i=0; i<15; i++)
        for (int j=0; j<3; j++)
            delta_x[i] += K[i*3 + j] * y[j];

    // 修正
    ins.p[0] += delta_x[0]; ins.p[1] += delta_x[1]; ins.p[2] += delta_x[2];
    ins.v[0] += delta_x[3]; ins.v[1] += delta_x[4]; ins.v[2] += delta_x[5];
    float dphi[3] = {delta_x[6], delta_x[7], delta_x[8]};
    float dq[4] = {1.0f, 0.5f*dphi[0], 0.5f*dphi[1], 0.5f*dphi[2]};
    float q_new[4];
    quat_multiply(ins.q, dq, q_new);
    quat_normalize(q_new);
    memcpy(ins.q, q_new, 4*sizeof(float));
    ins.bg[0] += delta_x[9];  ins.bg[1] += delta_x[10]; ins.bg[2] += delta_x[11];
    ins.ba[0] += delta_x[12]; ins.ba[1] += delta_x[13]; ins.ba[2] += delta_x[14];

    float KH[225];
    matrix_multiply(K, H, KH, 15, 3, 15);
    float I[225];
    matrix_identity(I, 15);
    for (int i=0; i<225; i++) I[i] -= KH[i];
    float P_new[225];
    matrix_multiply(I, ekf.P, P_new, 15, 15, 15);
    memcpy(ekf.P, P_new, 225*sizeof(float));
}

// ======================== EKF 深度更新 ========================
static void EKF_Update_Depth(void) {
    if (!sensor.depth_new) return;
    float z = sensor.depth;
    float h = ins.p[2];
    float y = z - h;

    float H[15] = {0};
    H[2] = 1.0f;

    float S = ekf.R_depth;
    for (int i=0; i<15; i++)
        for (int j=0; j<15; j++)
            S += H[i] * ekf.P[i*15+j] * H[j];

    float K[15];
    for (int i=0; i<15; i++) {
        K[i] = 0.0f;
        for (int j=0; j<15; j++) K[i] += ekf.P[i*15+j] * H[j];
        K[i] /= S;
    }

    float delta_x[15];
    for (int i=0; i<15; i++) delta_x[i] = K[i] * y;

    ins.p[0] += delta_x[0]; ins.p[1] += delta_x[1]; ins.p[2] += delta_x[2];
    ins.v[0] += delta_x[3]; ins.v[1] += delta_x[4]; ins.v[2] += delta_x[5];
    float dphi[3] = {delta_x[6], delta_x[7], delta_x[8]};
    float dq[4] = {1.0f, 0.5f*dphi[0], 0.5f*dphi[1], 0.5f*dphi[2]};
    float q_new[4];
    quat_multiply(ins.q, dq, q_new);
    quat_normalize(q_new);
    memcpy(ins.q, q_new, 4*sizeof(float));
    ins.bg[0] += delta_x[9];  ins.bg[1] += delta_x[10]; ins.bg[2] += delta_x[11];
    ins.ba[0] += delta_x[12]; ins.ba[1] += delta_x[13]; ins.ba[2] += delta_x[14];

    float KH[225];
    for (int i=0; i<15; i++)
        for (int j=0; j<15; j++)
            KH[i*15+j] = K[i] * H[j];
    float I[225];
    matrix_identity(I, 15);
    for (int i=0; i<225; i++) I[i] -= KH[i];
    float P_new[225];
    matrix_multiply(I, ekf.P, P_new, 15, 15, 15);
    memcpy(ekf.P, P_new, 225*sizeof(float));

    sensor.depth_new = 0;
}

// ======================== 运动约束更新 ========================
static void EKF_Update_NHC(void) {
    if (!enable_nhc) return;

    float Cbn[9];
    quat_to_dcm(ins.q, Cbn);
    float Cnb[9];
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            Cnb[i*3+j] = Cbn[j*3+i];

    float v_body[3];
    for (int i=0; i<3; i++) {
        v_body[i] = Cnb[i*3+0]*ins.v[0] + Cnb[i*3+1]*ins.v[1] + Cnb[i*3+2]*ins.v[2];
    }
    float y[2] = {-v_body[1], -v_body[2]};

    float H[30] = {0};
    for (int i=0; i<2; i++) {
        for (int j=0; j<3; j++) {
            H[i*15 + 3 + j] = Cnb[(i+1)*3 + j];
        }
    }
    float v_skew[9];
    skew_matrix(ins.v, v_skew);
    float temp[9];
    matrix_multiply(Cnb, v_skew, temp, 3, 3, 3);
    for (int i=0; i<2; i++) {
        for (int j=0; j<3; j++) {
            H[i*15 + 6 + j] = -temp[(i+1)*3 + j];
        }
    }

    float Ht[30];
    matrix_transpose(H, Ht, 2, 15);
    float HP[30];
    matrix_multiply(H, ekf.P, HP, 2, 15, 15);
    float S[4];
    matrix_multiply(HP, Ht, S, 2, 15, 2);
    for (int i=0; i<4; i++) S[i] += ekf.R_nhc[i];

    float det = S[0]*S[3] - S[1]*S[2];
    if (fabs(det) < 1e-12f) return;
    float S_inv[4] = {S[3]/det, -S[1]/det, -S[2]/det, S[0]/det};

    float temp2[30];
    matrix_multiply(ekf.P, Ht, temp2, 15, 15, 2);
    float K[30];
    matrix_multiply(temp2, S_inv, K, 15, 2, 2);

    float delta_x[15] = {0};
    for (int i=0; i<15; i++) {
        delta_x[i] = K[i*2 + 0]*y[0] + K[i*2 + 1]*y[1];
    }

    ins.p[0] += delta_x[0]; ins.p[1] += delta_x[1]; ins.p[2] += delta_x[2];
    ins.v[0] += delta_x[3]; ins.v[1] += delta_x[4]; ins.v[2] += delta_x[5];
    float dphi[3] = {delta_x[6], delta_x[7], delta_x[8]};
    float dq[4] = {1.0f, 0.5f*dphi[0], 0.5f*dphi[1], 0.5f*dphi[2]};
    float q_new[4];
    quat_multiply(ins.q, dq, q_new);
    quat_normalize(q_new);
    memcpy(ins.q, q_new, 4*sizeof(float));
    ins.bg[0] += delta_x[9];  ins.bg[1] += delta_x[10]; ins.bg[2] += delta_x[11];
    ins.ba[0] += delta_x[12]; ins.ba[1] += delta_x[13]; ins.ba[2] += delta_x[14];

    float KH[225];
    matrix_multiply(K, H, KH, 15, 2, 15);
    float I[225];
    matrix_identity(I, 15);
    for (int i=0; i<225; i++) I[i] -= KH[i];
    float P_new[225];
    matrix_multiply(I, ekf.P, P_new, 15, 15, 15);
    memcpy(ekf.P, P_new, 225*sizeof(float));
}

// ======================== 主步进 ========================
void INS_EKF_Step(float dt) {
    INS_Mechanization(dt);
    EKF_Predict(dt);
    EKF_Update_Attitude();
    EKF_Update_Depth();
    EKF_Update_NHC();
}

// ======================== 传感器接口 ========================
void INS_EKF_SetSensorData(float acc[3], float gyro[3], float euler[3]) {
    // 一阶低通滤波，滤除高频振动（截止频率约5Hz）
    float alpha = LPF_ALPHA;
    for (int i = 0; i < 3; i++) {
        lpf_accel[i] = alpha * acc[i] + (1 - alpha) * lpf_accel[i];
        lpf_gyro[i]  = alpha * gyro[i]  + (1 - alpha) * lpf_gyro[i];
    }
    memcpy(sensor.accel, lpf_accel, 3*sizeof(float));
    memcpy(sensor.gyro,  lpf_gyro,  3*sizeof(float));
    memcpy(sensor.euler, euler, 3*sizeof(float));
}

void INS_EKF_SetDepth(float depth) {
    sensor.depth = depth;
    sensor.depth_new = 1;
}

void INS_EKF_GetResult(float p[3], float v[3], float euler[3]) {
    memcpy(p, ins.p, 3*sizeof(float));
    memcpy(v, ins.v, 3*sizeof(float));
    quat_to_euler(ins.q, euler);
}

// ======================== 辅助函数实现 ========================
static void quat_multiply(const float *q1, const float *q2, float *q_out) {
    q_out[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q_out[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    q_out[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    q_out[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void quat_normalize(float *q) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 1e-12f) {
        float inv = 1.0f/norm;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    }
}

static void euler_to_quat(const float *euler, float *q) {
    float phi = euler[0], theta = euler[1], psi = euler[2];
    float cy = cosf(psi*0.5f), sy = sinf(psi*0.5f);
    float cp = cosf(theta*0.5f), sp = sinf(theta*0.5f);
    float cr = cosf(phi*0.5f), sr = sinf(phi*0.5f);
    q[0] = cy*cp*cr + sy*sp*sr;
    q[1] = cy*cp*sr - sy*sp*cr;
    q[2] = sy*cp*sr + cy*sp*cr;
    q[3] = sy*cp*cr - cy*sp*sr;
    quat_normalize(q);
}

static void quat_to_euler(const float *q, float *euler) {
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    euler[0] = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
    euler[1] = asinf(2.0f*(q0*q2 - q3*q1));
    euler[2] = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
}

static void quat_to_dcm(const float *q, float *Cbn) {
    float q0=q[0], q1=q[1], q2=q[2], q3=q[3];
    Cbn[0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    Cbn[1] = 2.0f*(q1*q2 - q0*q3);
    Cbn[2] = 2.0f*(q1*q3 + q0*q2);
    Cbn[3] = 2.0f*(q1*q2 + q0*q3);
    Cbn[4] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    Cbn[5] = 2.0f*(q2*q3 - q0*q1);
    Cbn[6] = 2.0f*(q1*q3 - q0*q2);
    Cbn[7] = 2.0f*(q2*q3 + q0*q1);
    Cbn[8] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

static void skew_matrix(const float *v, float *Sk) {
    Sk[0] = 0.0f;    Sk[1] = -v[2]; Sk[2] = v[1];
    Sk[3] = v[2];    Sk[4] = 0.0f;  Sk[5] = -v[0];
    Sk[6] = -v[1];   Sk[7] = v[0];  Sk[8] = 0.0f;
}

static void matrix_multiply(const float *A, const float *B, float *C, int rowsA, int colsA, int colsB) {
    for (int i=0; i<rowsA; i++) {
        for (int j=0; j<colsB; j++) {
            float sum = 0.0f;
            for (int k=0; k<colsA; k++) {
                sum += A[i*colsA + k] * B[k*colsB + j];
            }
            C[i*colsB + j] = sum;
        }
    }
}

static void matrix_transpose(const float *A, float *At, int rows, int cols) {
    for (int i=0; i<rows; i++)
        for (int j=0; j<cols; j++)
            At[j*rows + i] = A[i*cols + j];
}

static void matrix_identity(float *A, int n) {
    memset(A, 0, n*n*sizeof(float));
    for (int i=0; i<n; i++) A[i*n + i] = 1.0f;
}
float acc[3],gyro[3],euler[3];
void ins_task()
{
INS_EKF_Init();
	while(1)
	{	
		acc[0]=accx;
		acc[1]=accy;
		acc[2]=accz;
		gyro[0]=angx;
		gyro[1]=angy;
		gyro[2]=angz;
		euler[0]=roll;
		euler[1]=pitch;
		euler[2]=yaw;
		
		INS_EKF_SetSensorData(acc, gyro, euler);
        INS_EKF_SetDepth(ms5837_depth);

		  INS_EKF_Step(0.05);   // 0.05f 

        // 获取结果并输出（示例）
        float p[3], v[3], euler[3];
        INS_EKF_GetResult(p, v, euler);
		printf("p=%f,%f,%f\n",p[0],p[1],p[2]);
		vTaskDelay(pdMS_TO_TICKS(20));
		
	}
	
}