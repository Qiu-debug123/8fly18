#include <Arduino.h>
#include "uwb.h"

// 构造函数：初始化与你的原代码完全一致
EKFFilter::EKFFilter(float dt, float q_pos, float q_vel, float q_bias, float r_uwb, float jump_threshold)
  : uwb_dt(dt), Q_pos(q_pos), Q_vel(q_vel), Q_bias(q_bias), R_uwb(r_uwb), uwb_jump_threshold(jump_threshold) {
  fusion_pos = 0.0f;
  fusion_vel = 0.0f;
  fusion_accel_bias = 0.0f; // 初始化加速度零偏为0
  predicted_pos = 0.0f;
  predicted_vel = 0.0f;
  predicted_accel_bias = 0.0f;
  Is_UWB_first_update = 1; // 标记第一次更新，用于初始化old_uwb_pos

  // 初始化P矩阵 (3x3)
  // 初始协方差矩阵对角元素初始化
  P[0][0] = 1.0f; P[0][1] = 0.0f; P[0][2] = 0.0f;
  P[1][0] = 0.0f; P[1][1] = 1.0f; P[1][2] = 0.0f;
  P[2][0] = 0.0f; P[2][1] = 0.0f; P[2][2] = 0.001f; // 零偏的初始不确定性
}

// 核心update函数：完全复现你的原EKF逻辑，逐行对应，现已加入加速度零偏估计
void EKFFilter::update(float imu_accel, float uwb_pos, int Is_UWB_work) {
  if(Is_UWB_first_update) {
    old_uwb_pos = uwb_pos; // 初始化old_uwb_pos为第一次接收到的UWB位置
    Is_UWB_first_update = 0; // 取消第一次更新标志
  }
  // --------- 第一阶段：预测（加速度更新） ---------
  // 每当加速度计产生数据时执行

  // 1. 状态预测:
  // p_pre = p + v * dt + 0.5 * (a_raw - b) * dt^2
  // v_pre = v + (a_raw - b) * dt
  // b_pre = b (假设零偏短期内是不变的)
  
  // 有效加速度 = 原始加速度 - 融合后的加速度零偏
  float effective_accel = imu_accel - fusion_accel_bias; 

  predicted_pos = fusion_pos + fusion_vel * uwb_dt + 0.5f * effective_accel * uwb_dt * uwb_dt;
  predicted_vel = fusion_vel + effective_accel * uwb_dt;
  predicted_accel_bias = fusion_accel_bias;

  // 2. 协方差预测 (P = A*P*A^T + Q):
  // 按照你提供的展开公式进行计算 (注意数组下标从0开始，对应公式中的1,2,3)
  // P[0][0] -> P11, P[0][1] -> P12, ..., P[2][2] -> P33
  
  // 暂存旧的P矩阵值，因为计算过程中P回被更新覆盖
  float old_P[3][3];
  for(int i=0; i<3; i++) for(int j=0; j<3; j++) old_P[i][j] = P[i][j];

  float dt = uwb_dt;
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;

  // P11 = P11 + 2*dt*P12 + dt^2*P22 - dt^2*P13 - dt^3*P23 + 0.25*dt^4*P33 + Q_p
  P[0][0] = old_P[0][0] + 2.0f*dt*old_P[0][1] + dt2*old_P[1][1] - dt2*old_P[0][2] - dt3*old_P[1][2] + 0.25f*dt4*old_P[2][2] + Q_pos;
  
  // P12 = P12 + dt*P22 - 0.5*dt^2*P13 - dt^2*P23 + 0.5*dt^3*P33
  P[0][1] = old_P[0][1] + dt*old_P[1][1] - 0.5f*dt2*old_P[0][2] - dt2*old_P[1][2] + 0.5f*dt3*old_P[2][2];
  P[1][0] = P[0][1]; // 对称
  
  // P13 = P13 + dt*P23 - 0.5*dt^2*P33
  P[0][2] = old_P[0][2] + dt*old_P[1][2] - 0.5f*dt2*old_P[2][2];
  P[2][0] = P[0][2]; // 对称
  
  // P22 = P22 - 2*dt*P23 + dt^2*P33 + Q_v
  P[1][1] = old_P[1][1] - 2.0f*dt*old_P[1][2] + dt2*old_P[2][2] + Q_vel;

  // P23 = P23 - dt*P33
  P[1][2] = old_P[1][2] - dt*old_P[2][2];
  P[2][1] = P[1][2]; // 对称
  
  // P33 = P33 + Q_b
  P[2][2] = old_P[2][2] + Q_bias;


  // --------- 第二阶段：更新（UWB 修正） ---------
  // 每当 UWB 有新数据时执行
  // 新增逻辑：即使UWB工作正常，如果测量值与上一次UWB位置偏差超过阈值（uwb_jump_threshold），
  // 则判定为数据跳变，将其视为无效数据处理，直接使用预测值作为融合结果。
  if (Is_UWB_work == 1 && abs(uwb_pos - old_uwb_pos) < uwb_jump_threshold) {
    // 1. 计算卡尔曼增益 K (3x1):
    // S = P11 + R
    float S = P[0][0] + R_uwb;
    K[0] = P[0][0] / S; // K1
    K[1] = P[1][0] / S; // K2 (注意P是对称阵，P21 = P12，即 P[1][0] = P[0][1])
    K[2] = P[2][0] / S; // K3 (P31 = P13，即 P[2][0] = P[0][2])

    // 2. 修正状态:
    float residual = uwb_pos - predicted_pos;
    
    fusion_pos = predicted_pos + K[0] * residual;
    fusion_vel = predicted_vel + K[1] * residual;
    // 这里 K3 实现了：根据位置误差自动倒推加速度零偏
    fusion_accel_bias = predicted_accel_bias + K[2] * residual; 

    // 3. 修正协方差 (P = (I - KH)P):
    // 暂存未更新的 P 矩阵值，用于计算
    for(int i=0; i<3; i++) for(int j=0; j<3; j++) old_P[i][j] = P[i][j];

    // P11 = (1 - K1) * P11
    P[0][0] = (1.0f - K[0]) * old_P[0][0];
    // P12 = (1 - K1) * P12
    P[0][1] = (1.0f - K[0]) * old_P[0][1];
    P[1][0] = P[0][1]; // 对称
    // P13 = (1 - K1) * P13
    P[0][2] = (1.0f - K[0]) * old_P[0][2];
    P[2][0] = P[0][2]; // 对称
    
    // P22 = P22 - K2 * P12
    P[1][1] = old_P[1][1] - K[1] * old_P[0][1];
    
    // P23 = P23 - K2 * P13
    P[1][2] = old_P[1][2] - K[1] * old_P[0][2];
    P[2][1] = P[1][2]; // 对称
    
    // P33 = P33 - K3 * P13
    P[2][2] = old_P[2][2] - K[2] * old_P[0][2];

  } else {
    // 你的逻辑：UWB无效或检测到跳变时更新预测值为融合值
    fusion_pos = predicted_pos;
    fusion_vel = predicted_vel;
    // 加速度零偏也直接使用预测值（假设无变化）
    fusion_accel_bias = predicted_accel_bias;
  }
  // 更新old_uwb_pos为当前UWB位置，为下一次跳变检测做准备
  old_uwb_pos = uwb_pos;
}

// 基础接口：仅获取位置/速度，匹配你的结构体
float EKFFilter::getPos() const { return fusion_pos; }
float EKFFilter::getSpeed() const { return fusion_vel; }
// 获取加速度零偏接口
float EKFFilter::getAccelBias() const { return fusion_accel_bias; }