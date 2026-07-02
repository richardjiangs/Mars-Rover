// ============================================================
// 全车共享状态、模式定义与跨模块前向声明
// 模式与优先级依据 docs/Ares_Control_Architecture.md 第 8 节
// ============================================================
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// ---------- 运行模式 ----------
enum Mode : uint8_t { MODE_IDLE, MODE_AUTO, MODE_TELEOP, MODE_CAPTURE, MODE_ESTOP, MODE_SLEEP };
inline const char* modeName(Mode m) {
  switch (m) {
    case MODE_IDLE:    return "IDLE";
    case MODE_AUTO:    return "AUTO";
    case MODE_TELEOP:  return "TELEOP-PC";
    case MODE_CAPTURE: return "CAPTURE";
    case MODE_ESTOP:   return "E-STOP";
    case MODE_SLEEP:   return "SLEEP";
  }
  return "?";
}

// ---------- 热控状态（五态机） ----------
enum ThermalState : uint8_t { TH_NORMAL, TH_COLD_START, TH_COLD_SLEEP, TH_HOT_LIMIT, TH_THERMAL_PAUSE, TH_UNKNOWN };
inline const char* thermalName(ThermalState t) {
  switch (t) {
    case TH_NORMAL:        return "NORMAL";
    case TH_COLD_START:    return "COLD_START";
    case TH_COLD_SLEEP:    return "COLD_SLEEP";
    case TH_HOT_LIMIT:     return "HOT_LIMIT";
    case TH_THERMAL_PAUSE: return "THERMAL_PAUSE";
    default:               return "N/A";
  }
}

// ---------- 拍摄任务 ----------
enum TaskKind : uint8_t { TASK_NONE, TASK_PANORAMA, TASK_STEREO };

struct RoverState {
  Mode mode = MODE_IDLE;
  bool estopLatched = false;      // E-STOP 需显式清除
  // 遥控输入（TELEOP）
  float teleThrottle = 0, teleTurn = 0;
  uint32_t lastHeartbeatMs = 0;
  uint32_t lastDriveCmdMs = 0;
  // 姿态
  float yawDeg = 0;               // 陀螺积分航向
  float targetYaw = 0;            // 直行段锁定的目标航向
  bool  yawHold = false;
  // 超声波（cm，-1 表示无效）
  int frontCm = -1, leftCm = -1, rightCm = -1;
  // 热控
  ThermalState thermal = TH_UNKNOWN;
  float insideTempC = NAN;
  // 能源
  float batteryV = 0;
  int   lightRaw = -1;
  // 任务
  TaskKind task = TASK_NONE;
  int  taskIndex = 0;             // 当前步序号
  int  taskTotal = 0;
  // 相机
  char camStatus[8] = "OFF";      // OFF / READY / BUSY / ERR
  // 生存模拟
  uint32_t wakeCount = 0;
  bool finalMission = false;
};

extern RoverState rover;

// ---------- 跨模块接口（定义在各自模块里） ----------
void commsBroadcast(JsonDocument& doc);              // comms.h
void commsAlert(const char* level, const char* code); // comms.h
void missionHandleDrive(float throttle, float turn);  // mission.h
bool missionSetMode(Mode m, const char* reason);      // mission.h
void missionStartTask(TaskKind k, int stepDeg);       // mission.h
void missionStopTask();                               // mission.h
void missionEstop(bool active);                       // mission.h
void panoServoWrite(int deg);                         // mission.h（舵机）
void powerEnterSleep(const char* why);                // powersleep.h
