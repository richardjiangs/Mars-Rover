// ============================================================
// MPU6050：直接读寄存器（不依赖第三方库）
// 上电静置标定陀螺零偏 -> 积分 gz 得偏航角 -> PID 直线保持
// ============================================================
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "state.h"

#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_GYRO_CONFIG 0x1B
#define MPU_GYRO_ZOUT_H 0x47
#define GYRO_LSB_PER_DPS 131.0f   // ±250dps 量程

struct ImuCtl {
  bool ok = false;
  float gzBias = 0;
  uint32_t lastUs = 0;
  // PID 内部量
  float integ = 0, lastErr = 0;

  bool begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    if (!writeReg(MPU_PWR_MGMT_1, 0x00)) return false;   // 退出睡眠
    writeReg(MPU_GYRO_CONFIG, 0x00);                     // ±250dps
    delay(50);
    calibrate();
    lastUs = micros();
    ok = true;
    return true;
  }

  // 上电静置标定（期间不要动车）
  void calibrate() {
    const int n = IMU_CALIB_MS / 5;
    float sum = 0;
    for (int i = 0; i < n; i++) { sum += readGzRaw(); delay(5); }
    gzBias = sum / n;
  }

  // 每个 loop 调用：积分航向
  void tick() {
    if (!ok) return;
    uint32_t now = micros();
    float dt = (now - lastUs) / 1e6f;
    lastUs = now;
    if (dt <= 0 || dt > 0.5f) return;               // 防止暂停后大跳变
    float gzDps = (readGzRaw() - gzBias) / GYRO_LSB_PER_DPS;
    if (fabsf(gzDps) < 0.05f) gzDps = 0;            // 死区抑制漂移
    rover.yawDeg += gzDps * dt;
    // 归一到 [-180,180)
    while (rover.yawDeg >= 180) rover.yawDeg -= 360;
    while (rover.yawDeg < -180) rover.yawDeg += 360;
  }

  void lockHeading() {                 // 直行段开始时锁定目标航向
    rover.targetYaw = rover.yawDeg;
    rover.yawHold = true;
    integ = 0; lastErr = 0;
  }
  void releaseHeading() { rover.yawHold = false; }

  // 返回差速修正量（加到 turn 上），直行保持用
  float headingCorrection() {
    if (!ok || !rover.yawHold) return 0;
    float err = angleDiff(rover.targetYaw, rover.yawDeg);   // 偏左为正 -> 需右修
    integ = constrain(integ + err * YAW_KI, -YAW_CORR_MAX, YAW_CORR_MAX);
    float d = (err - lastErr) * YAW_KD;
    lastErr = err;
    return constrain(err * YAW_KP + integ + d, -YAW_CORR_MAX, YAW_CORR_MAX);
  }

private:
  static float angleDiff(float a, float b) {   // a-b，折算到 [-180,180)
    float d = a - b;
    while (d >= 180) d -= 360;
    while (d < -180) d += 360;
    return d;
  }

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  float readGzRaw() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_GYRO_ZOUT_H);
    if (Wire.endTransmission(false) != 0) return gzBias;   // 读失败按零偏处理
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return gzBias;
    int16_t raw = (Wire.read() << 8) | Wire.read();
    return (float)raw;
  }
};

extern ImuCtl imu;
