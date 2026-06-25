#ifndef T265_RECEIVER_H
#define T265_RECEIVER_H

#include <Arduino.h>

class T265Receiver {
public:
  struct PoseData {
    float pos_x, pos_y, pos_z;
    float quat_x, quat_y, quat_z, quat_w;
    float roll, pitch, yaw;  // 欧拉角（度）
  };

  void processByte(uint8_t b);
  void printPose();
  void printPose2();
  void printStats();
  void checkTimeout();

  PoseData pose;
  bool valid = false;
  uint32_t packets = 0, errors = 0;
  unsigned long lastTime = 0;

private:
  void quatToEuler(float qx, float qy, float qz, float qw, float& roll, float& pitch, float& yaw);
  float clip(float value, float min, float max);
};

#endif