#ifndef UWB_H
#define UWB_H
#include <Arduino.h>

// 你的原始滤波结果结构体，无修改
typedef struct {
  float speed;
  float pos;
} EKF_UWB_IMU_DATA;

// 二维卡尔曼滤波类，仅声明与你的逻辑匹配的成员
class EKFFilter {
  private:
    // 与你原静态变量完全一致的私有状态/参数
    float fusion_pos;
    float fusion_vel;
    float fusion_accel_bias; // 加速度零偏
    float predicted_pos;
    float predicted_vel;
    float predicted_accel_bias; // 预测的加速度零偏 (b_pre)
    float P[3][3];
    float K[3];
    const float uwb_dt;
    const float Q_pos;
    const float Q_vel;
    const float Q_bias; // 加速度零偏的过程噪声 (Q_b)
    const float R_uwb;
    const float uwb_jump_threshold; // UWB跳变阈值
    
    float old_uwb_pos; // 用于检测UWB数据跳变的上一个位置值
    int Is_UWB_first_update; // 标志是否第一次更新，用于初始化old_uwb_pos

  public:
    // 构造函数：默认参数与你的原代码数值一致，新增jump_threshold参数用于设定跳变阈值（默认1.0）
    // 新增 q_bias 参数用于设置加速度零偏的过程噪声，默认 1e-6 (0.000001f)
    EKFFilter(float dt = 0.01f, float q_pos = 0.000001f, float q_vel = 0.0005f, float q_bias = 0.000001f, float r_uwb = 0.04f, float jump_threshold = 0.1f);
    // 核心更新函数：参数与你的原函数完全一致
    void update(float imu_accel, float uwb_pos, int Is_UWB_work);
    // 仅保留获取结果的基础接口，匹配你的结构体
    float getPos() const;
    float getSpeed() const;
    float getAccelBias() const; // 获取加速度零偏
};

#endif