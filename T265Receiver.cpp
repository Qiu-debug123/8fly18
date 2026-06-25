#include "T265Receiver.h"

void T265Receiver::processByte(uint8_t b) {
  static uint8_t state = 0;
  static uint8_t buf[28];
  static uint8_t idx = 0;
  
  switch (state) {
    case 0:
      if (b == 0xAA) state = 1;
      break;
    case 1:
      if (b == 0x55) { 
        state = 2; 
        idx = 0; 
      }
      else if (b != 0xAA) state = 0;
      break;
    case 2:
      buf[idx++] = b;
      if (idx >= 28) state = 3;
      break;
    case 3:
      {
        uint8_t chk = 0;
        for (int i = 0; i < 28; i++) chk ^= buf[i];
        if (chk == b) {
          memcpy(&pose, buf, 28);
          // 计算欧拉角
          quatToEuler(pose.quat_x, pose.quat_y, pose.quat_z, pose.quat_w, 
                      pose.roll, pose.pitch, pose.yaw);
          valid = true;
          packets++;
          lastTime = millis();
          printPose2();
        } else {
          errors++;
        }
        state = 0;
      }
      break;
  }
}

void T265Receiver::printPose() {
  Serial.print("Pos: ");
  Serial.print(pose.pos_x, 3); Serial.print(", ");
  Serial.print(pose.pos_y, 3); Serial.print(", ");
  Serial.print(pose.pos_z, 3);
  Serial.print("  Quat: ");
  Serial.print(pose.quat_x, 3); Serial.print(", ");
  Serial.print(pose.quat_y, 3); Serial.print(", ");
  Serial.print(pose.quat_z, 3); Serial.print(", ");
  Serial.print(pose.quat_w, 3);
  Serial.print("  Euler: ");
  Serial.print(pose.roll, 1); Serial.print(", ");
  Serial.print(pose.pitch, 1); Serial.print(", ");
  Serial.println(pose.yaw, 1);
}

void T265Receiver::printPose2() {
  // 格式: pos_x, pos_y, pos_z, roll, pitch, yaw (逗号分隔，适合串口绘图器)
  Serial.print(pose.pos_x, 3); Serial.print(",");
  Serial.print(pose.pos_y, 3); Serial.print(",");
  Serial.print(pose.pos_z, 3); Serial.print(",");
  Serial.print(pose.roll, 1); Serial.print(",");
  Serial.print(pose.pitch, 1); Serial.print(",");
  Serial.println(pose.yaw, 1);
}

void T265Receiver::printStats() {
  Serial.print("Packets: ");
  Serial.print(packets);
  Serial.print(", Errors: ");
  Serial.println(errors);
}

void T265Receiver::checkTimeout() {
  if (valid && millis() - lastTime > 200) {
    valid = false;
    Serial.println("Timeout!");
  }
}

float T265Receiver::clip(float value, float min, float max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

void T265Receiver::quatToEuler(float qx, float qy, float qz, float qw, 
                                float& roll, float& pitch, float& yaw) {
  // pitch = -asin(clip(2.0*(qx*qz - qw*qy), -1, 1))
  float val = 2.0f * (qx * qz - qw * qy);
  val = clip(val, -1.0f, 1.0f);
  pitch = -asin(val);
  
  // roll = atan2(2.0*(qw*qx + qy*qz), qw*qw - qx*qx - qy*qy + qz*qz)
  roll = atan2(2.0f * (qw * qx + qy * qz), 
               qw * qw - qx * qx - qy * qy + qz * qz);
  
  // yaw = atan2(2.0*(qw*qz + qx*qy), qw*qw + qx*qx - qy*qy - qz*qz)
  yaw = atan2(2.0f * (qw * qz + qx * qy), 
              qw * qw + qx * qx - qy * qy - qz * qz);
  
  // 转换为度
  roll = roll * 180.0f / PI;
  pitch = pitch * 180.0f / PI;
  yaw = yaw * 180.0f / PI;
}