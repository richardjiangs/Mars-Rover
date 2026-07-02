// ============================================================
// 能源与"100 天火星生存"模拟：
// - LDR 光照采样（可选）：持续黑暗 -> 深度休眠；光照/定时器唤醒
// - RTC 内存记录唤醒次数；达到 WAKE_LIMIT 触发 Final_Mission_Mode
// - 电池电压分压采样（可选）
// 依据 docs/Ares_火星车原型制作完整指南.md 步骤 5
// ============================================================
#pragma once
#include <Arduino.h>
#include <esp_sleep.h>
#include "config.h"
#include "state.h"
#include "motors.h"

// RTC 慢速内存：深度休眠不掉、断电才清零
RTC_DATA_ATTR uint32_t rtcWakeCount = 0;
RTC_DATA_ATTR uint32_t rtcPhotoSeq  = 0;   // 照片序号跨休眠连续

struct PowerCtl {
  uint32_t darkSinceMs = 0;

  void begin() {
    rtcWakeCount++;
    rover.wakeCount = rtcWakeCount;
    rover.finalMission = (rtcWakeCount >= WAKE_LIMIT);
#if ENABLE_LDR
    pinMode(PIN_LDR_DO, INPUT);
    analogSetPinAttenuation(PIN_LDR_AO, ADC_11db);
#endif
#if ENABLE_VBAT
    analogSetPinAttenuation(PIN_VBAT, ADC_11db);
#endif
  }

  void tick() {
#if ENABLE_VBAT
    // 3.3V 满量程 4095；分压系数在 config.h 按实测校准
    rover.batteryV = analogRead(PIN_VBAT) / 4095.0f * 3.3f * VBAT_DIVIDER;
#endif
#if ENABLE_LDR
    rover.lightRaw = analogRead(PIN_LDR_AO);
    // 最终任务模式无视黑夜；TELEOP/CAPTURE 期间也不打断
    bool sleepAllowed = !rover.finalMission &&
                        (rover.mode == MODE_IDLE || rover.mode == MODE_AUTO);
    if (sleepAllowed && rover.lightRaw >= 0 && rover.lightRaw < LDR_DARK_THRESHOLD) {
      if (darkSinceMs == 0) darkSinceMs = millis();
      else if (millis() - darkSinceMs > LDR_DARK_HOLD_MS) powerEnterSleepImpl("dark");
    } else {
      darkSinceMs = 0;
    }
#endif
  }

  void powerEnterSleepImpl(const char* why) {
    motors.hardStop();
    commsAlert("INFO", "ENTER_SLEEP");
    (void)why;
    delay(200);                       // 给告警一点发送时间
#if ENABLE_LDR
    // 天亮（DO 变高）或定时器，先到先醒
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_LDR_DO, 1);
#endif
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_TIMER_WAKE_S * 1000000ULL);
    esp_deep_sleep_start();           // 不返回；醒来即重启，wakeCount+1
  }
};

extern PowerCtl power;

inline void powerEnterSleep(const char* why) { power.powerEnterSleepImpl(why); }
