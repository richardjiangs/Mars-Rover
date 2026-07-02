// ============================================================
// 热控五态机（NORMAL/COLD_START/COLD_SLEEP/HOT_LIMIT/THERMAL_PAUSE）
// 阈值依据 docs/Ares_机械减震与保温系统补充说明.md §3.5.4
// ENABLE_DS18B20=0 时整个模块静默（thermal 遥测报 N/A）
// ============================================================
#pragma once
#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "motors.h"

#if ENABLE_DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

struct ThermalCtl {
#if ENABLE_DS18B20
  OneWire oneWire{ PIN_DS18B20 };
  DallasTemperature dallas{ &oneWire };
#endif
  uint32_t lastReadMs = 0;
  uint32_t coldStartSinceMs = 0;

  void begin() {
#if ENABLE_DS18B20
    dallas.begin();
    dallas.setWaitForConversion(false);   // 异步转换，不阻塞 loop
    dallas.requestTemperatures();
#endif
  }

  void tick() {
#if ENABLE_DS18B20
    uint32_t now = millis();
    if (now - lastReadMs < 2000) return;
    lastReadMs = now;
    float t = dallas.getTempCByIndex(0);
    dallas.requestTemperatures();
    if (t <= -100) return;                // 探头未就绪/未接
    rover.insideTempC = t;
    apply(t);
#endif
  }

  // 电机是否被热控禁止
  bool motionBlocked() {
    return rover.thermal == TH_COLD_SLEEP || rover.thermal == TH_THERMAL_PAUSE;
  }
  // 拍摄是否被热控禁止
  bool captureBlocked() { return rover.thermal == TH_THERMAL_PAUSE; }

private:
  void apply(float t) {
    ThermalState ns;
    if      (t < T_COLD_SLEEP)    ns = TH_COLD_SLEEP;
    else if (t < T_COLD_START)    ns = TH_COLD_START;
    else if (t < T_HOT_LIMIT)     ns = TH_NORMAL;
    else if (t < T_THERMAL_PAUSE) ns = TH_HOT_LIMIT;
    else                          ns = TH_THERMAL_PAUSE;

    if (ns != rover.thermal) {
      rover.thermal = ns;
      commsAlert(ns == TH_NORMAL ? "INFO" : "WARN", thermalName(ns));
    }
    switch (ns) {
      case TH_NORMAL:       motors.dutyCapPct = MOTOR_MAX_PCT; break;
      case TH_COLD_START:   motors.dutyCapPct = COLD_START_PCT; break;
      case TH_HOT_LIMIT:    motors.dutyCapPct = HOT_LIMIT_PCT; break;
      case TH_COLD_SLEEP:
      case TH_THERMAL_PAUSE: motors.stop(); break;
      default: break;
    }
  }
};

extern ThermalCtl thermal;
