// ============================================================
// 电机驱动：单 L298N，IN1-IN4 直接输出 LEDC PWM 实现调速
// 差速/坦克转向 + 软启动斜坡 + 占空比限幅（保护驱动板）
// 兼容 arduino-esp32 核心 2.x 与 3.x 的 LEDC API
// ============================================================
#pragma once
#include <Arduino.h>
#include "config.h"
#include "state.h"

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

static const int MOTOR_DUTY_MAX = (1 << MOTOR_PWM_BITS) - 1;

// core 2.x 需要手动分配 LEDC 通道；core 3.x 按引脚自动管理
#if ESP_ARDUINO_VERSION_MAJOR >= 3
inline void pwmSetup(int pin, int /*ch*/) { ledcAttach(pin, MOTOR_PWM_FREQ, MOTOR_PWM_BITS); }
inline void pwmWrite(int pin, int /*ch*/, uint32_t duty) { ledcWrite(pin, duty); }
#else
inline void pwmSetup(int pin, int ch) { ledcSetup(ch, MOTOR_PWM_FREQ, MOTOR_PWM_BITS); ledcAttachPin(pin, ch); }
inline void pwmWrite(int /*pin*/, int ch, uint32_t duty) { ledcWrite(ch, duty); }
#endif

struct MotorsCtl {
  // 目标与当前占空比，单位：百分比 -100..100（正=前进）
  float targetL = 0, targetR = 0;
  float curL = 0, curR = 0;
  int   dutyCapPct = MOTOR_MAX_PCT;   // 热控可以进一步压低
  uint32_t lastTickMs = 0;

  void begin() {
    pwmSetup(PIN_ML_FWD, 0); pwmSetup(PIN_ML_REV, 1);
    pwmSetup(PIN_MR_FWD, 2); pwmSetup(PIN_MR_REV, 3);
    applyRaw(0, 0);
  }

  // throttle/turn ∈ [-1,1]；turn>0 右转（左轮加速、右轮减速）
  void setDrive(float throttle, float turn) {
    throttle = constrain(throttle, -1.0f, 1.0f);
    turn     = constrain(turn,     -1.0f, 1.0f);
    float l = throttle + turn, r = throttle - turn;
    float m = max(fabsf(l), fabsf(r));
    if (m > 1.0f) { l /= m; r /= m; }          // 归一化防溢出
    targetL = l * 100.0f; targetR = r * 100.0f;
  }

  void stop() { targetL = targetR = 0; }

  // E-STOP：越过斜坡立即切断
  void hardStop() {
    targetL = targetR = curL = curR = 0;
    applyRaw(0, 0);
  }

  void tick() {
    uint32_t now = millis();
    if (now - lastTickMs < MOTOR_TICK_MS) return;
    lastTickMs = now;
    curL = ramp(curL, targetL);
    curR = ramp(curR, targetR);
    applyRaw(curL, curR);
  }

private:
  float ramp(float cur, float tgt) {
    float step = MOTOR_RAMP_PCT;
    if (tgt > cur) cur = min(cur + step, tgt);
    else if (tgt < cur) cur = max(cur - step, tgt);
    return cur;
  }

  void applyRaw(float lPct, float rPct) {
    float cap = (float)dutyCapPct;
    lPct = constrain(lPct, -cap, cap);
    rPct = constrain(rPct, -cap, cap);
    writeSide(lPct, PIN_ML_FWD, 0, PIN_ML_REV, 1);
    writeSide(rPct, PIN_MR_FWD, 2, PIN_MR_REV, 3);
  }

  void writeSide(float pct, int pinF, int chF, int pinR, int chR) {
    uint32_t duty = (uint32_t)(fabsf(pct) / 100.0f * MOTOR_DUTY_MAX);
    if (pct >= 0) { pwmWrite(pinF, chF, duty); pwmWrite(pinR, chR, 0); }
    else          { pwmWrite(pinF, chF, 0);    pwmWrite(pinR, chR, duty); }
  }
};

extern MotorsCtl motors;
