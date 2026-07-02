// ============================================================
// 3 路独立 HC-SR04（AUTO-L2 三向感知），轮询防串扰 + 3 点中值滤波
// ⚠ 所有 Echo 必须经 1k/2k 分压再进 ESP32（见 GPIO Map §1.2）
// ============================================================
#pragma once
#include <Arduino.h>
#include "config.h"
#include "state.h"

struct SonarCtl {
  struct One {
    int trig, echo;
    int hist[3] = { -1, -1, -1 };
    int idx = 0;
  } s[3];
  int cur = 0;
  uint32_t lastPingMs = 0;

  void begin() {
    s[0] = { PIN_SONAR_F_TRIG, PIN_SONAR_F_ECHO };
    s[1] = { PIN_SONAR_L_TRIG, PIN_SONAR_L_ECHO };
    s[2] = { PIN_SONAR_R_TRIG, PIN_SONAR_R_ECHO };
    for (auto& x : s) {
      pinMode(x.trig, OUTPUT); digitalWrite(x.trig, LOW);
      pinMode(x.echo, INPUT);
    }
  }

  // 每 SONAR_INTERVAL_MS 测一路，轮流来（防互相串扰）
  void tick() {
    uint32_t now = millis();
    if (now - lastPingMs < SONAR_INTERVAL_MS) return;
    lastPingMs = now;
    measure(s[cur]);
    cur = (cur + 1) % 3;
    rover.frontCm = median(s[0]);
    rover.leftCm  = median(s[1]);
    rover.rightCm = median(s[2]);
  }

private:
  void measure(One& x) {
    digitalWrite(x.trig, LOW);  delayMicroseconds(3);
    digitalWrite(x.trig, HIGH); delayMicroseconds(10);
    digitalWrite(x.trig, LOW);
    // 300cm 往返约 17.5ms，超时按 20ms 截断
    uint32_t us = pulseIn(x.echo, HIGH, 20000UL);
    int cm = (us == 0) ? -1 : (int)(us / 58UL);
    if (cm > SONAR_MAX_CM) cm = -1;
    x.hist[x.idx] = cm;
    x.idx = (x.idx + 1) % 3;
  }

  // 3 点中值；不足 3 个有效值时取最近有效值
  int median(One& x) {
    int v[3]; int n = 0;
    for (int i = 0; i < 3; i++) if (x.hist[i] >= 0) v[n++] = x.hist[i];
    if (n == 0) return -1;
    if (n == 1) return v[0];
    if (n == 2) return (v[0] + v[1]) / 2;
    // n==3 手写中值
    int a = v[0], b = v[1], c = v[2];
    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) { int t = b; b = c; c = t; }
    if (a > b) { int t = a; a = b; b = t; }
    return b;
  }
};

extern SonarCtl sonar;
