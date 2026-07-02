// ============================================================
// 任务状态机：模式仲裁 + AUTO-L2 避障 + 全景/伪3D 拍摄任务 + 云台舵机
// 模式与优先级依据 docs/Ares_Control_Architecture.md §8/§9/§10
// ============================================================
#pragma once
#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "motors.h"
#include "imu.h"
#include "thermal.h"
#include "camlink.h"
#include "powersleep.h"

// ---- 云台舵机：50Hz LEDC，500-2500us 对应 0-180°（省掉 ESP32Servo 依赖） ----
#define SERVO_PWM_BITS 16
#define SERVO_PWM_FREQ 50
#if ESP_ARDUINO_VERSION_MAJOR >= 3
inline void servoSetup() { ledcAttach(PIN_SERVO_PAN, SERVO_PWM_FREQ, SERVO_PWM_BITS); }
inline void servoDuty(uint32_t d) { ledcWrite(PIN_SERVO_PAN, d); }
#else
inline void servoSetup() { ledcSetup(8, SERVO_PWM_FREQ, SERVO_PWM_BITS); ledcAttachPin(PIN_SERVO_PAN, 8); }
inline void servoDuty(uint32_t d) { ledcWrite(8, d); }
#endif

inline void panoServoWriteImpl(int deg) {
  deg = constrain(deg, 0, 180);
  uint32_t us = 500 + (uint32_t)deg * 2000 / 180;
  servoDuty((uint32_t)((uint64_t)us * ((1UL << SERVO_PWM_BITS) - 1) / 20000ULL));
}

// ---- AUTO 模式子状态 ----
enum AutoPhase : uint8_t { AP_CRUISE, AP_BRAKE, AP_BACK, AP_TURN };
// ---- 拍摄任务子状态 ----
enum CapPhase : uint8_t { CP_MOVE_SERVO, CP_SHOOT, CP_WAIT_SHOT, CP_ADVANCE, CP_DONE };

struct MissionCtl {
  Mode prevMode = MODE_IDLE;       // CAPTURE 结束后回到哪
  AutoPhase autoPhase = AP_CRUISE;
  uint32_t phaseUntilMs = 0;
  int turnDir = 1;                 // 1=右转，-1=左转
  // 拍摄任务
  CapPhase capPhase = CP_DONE;
  int panDeg = 0;
  int stepDeg = PAN_STEP_DEG;
  uint32_t capWaitMs = 0;
  uint32_t advanceUntilMs = 0;
  char shotName[32];

  void begin() {
    servoSetup();
    panoServoWriteImpl(90);        // 云台回中
  }

  // ---------------- 模式仲裁 ----------------
  bool setMode(Mode m, const char* reason) {
    if (rover.estopLatched && m != MODE_IDLE) return false;   // E-STOP 未清除
    if (m == rover.mode) return true;
    // 收尾
    if (rover.mode == MODE_CAPTURE) stopTaskInternal();
    motors.stop();
    imu.releaseHeading();
    rover.mode = m;
    if (m == MODE_AUTO) { autoPhase = AP_CRUISE; imu.lockHeading(); }
    if (m == MODE_SLEEP) powerEnterSleep(reason);
    JsonDocument doc;
    doc["type"] = "mode"; doc["mode"] = modeName(m); doc["reason"] = reason;
    commsBroadcast(doc);
    return true;
  }

  void estop(bool active) {
    if (active) {
      rover.estopLatched = true;
      motors.hardStop();
      stopTaskInternal();
      rover.mode = MODE_ESTOP;
      commsAlert("ERROR", "ESTOP_ACTIVE");
    } else {
      // 显式清除：回 IDLE，不自动恢复原模式（§10.3）
      rover.estopLatched = false;
      rover.mode = MODE_IDLE;
      commsAlert("INFO", "ESTOP_CLEARED");
    }
  }

  // 电脑端驾驶输入：AUTO 下收到非零指令 -> 自动切 TELEOP（§8.2）
  void handleDrive(float throttle, float turn) {
    rover.lastDriveCmdMs = millis();
    if (rover.mode == MODE_AUTO && (fabsf(throttle) > 0.05f || fabsf(turn) > 0.05f))
      setMode(MODE_TELEOP, "operator_override");
    if (rover.mode != MODE_TELEOP) return;
    rover.teleThrottle = throttle;
    rover.teleTurn = turn;
  }

  void startTask(TaskKind k, int stepDegArg) {
    if (rover.mode == MODE_ESTOP || rover.mode == MODE_SLEEP) return;
    if (thermal.captureBlocked()) { commsAlert("WARN", "CAPTURE_BLOCKED_THERMAL"); return; }
    prevMode = (rover.mode == MODE_CAPTURE) ? MODE_IDLE : rover.mode;
    rover.task = k;
    rover.taskIndex = 0;
    stepDeg = (stepDegArg > 0) ? stepDegArg : PAN_STEP_DEG;
    if (k == TASK_PANORAMA) {
      rover.taskTotal = 180 / stepDeg + 1;   // 0..180 共 N 张
      panDeg = 0;
      capPhase = CP_MOVE_SERVO;
    } else if (k == TASK_STEREO) {
      rover.taskTotal = 2;
      panoServoWriteImpl(90);
      capPhase = CP_SHOOT;
    }
    setMode(MODE_CAPTURE, "task_start");
    rover.mode = MODE_CAPTURE;               // setMode 收尾时已 stopTask，这里重申
    rover.task = k;
    broadcastTask("RUNNING");
  }

  void stopTask() {
    stopTaskInternal();
    if (rover.mode == MODE_CAPTURE) setMode(prevMode, "task_stopped");
  }

  // ---------------- 主循环 ----------------
  void tick() {
    // TELEOP 心跳丢失 -> 停车转 IDLE（§8.2）
    if (rover.mode == MODE_TELEOP &&
        millis() - rover.lastHeartbeatMs > HEARTBEAT_TIMEOUT_MS) {
      motors.stop();
      commsAlert("WARN", "HEARTBEAT_LOST");
      setMode(MODE_IDLE, "heartbeat_timeout");
    }
    if (thermal.motionBlocked() && rover.mode != MODE_ESTOP) motors.stop();

    switch (rover.mode) {
      case MODE_TELEOP:  tickTeleop();  break;
      case MODE_AUTO:    tickAuto();    break;
      case MODE_CAPTURE: tickCapture(); break;
      default: break;   // IDLE/ESTOP/SLEEP 不驱动
    }
  }

private:
  void tickTeleop() {
    if (thermal.motionBlocked()) return;
    motors.setDrive(rover.teleThrottle, rover.teleTurn);
  }

  // AUTO-L2：直线巡航 + 前向 20cm 停车 -> 后退 -> 向宽侧转 -> 继续
  void tickAuto() {
    if (thermal.motionBlocked()) return;
    uint32_t now = millis();
    switch (autoPhase) {
      case AP_CRUISE: {
        if (rover.frontCm > 0 && rover.frontCm < OBSTACLE_STOP_CM) {
          motors.stop();
          autoPhase = AP_BRAKE;
          phaseUntilMs = now + 300;
          break;
        }
        motors.setDrive(AUTO_CRUISE, imu.headingCorrection());
        break;
      }
      case AP_BRAKE: {
        if (now < phaseUntilMs) break;
        // 读左右，向更宽的一侧转（无效读数按"更宽"处理）
        int l = rover.leftCm  < 0 ? SONAR_MAX_CM : rover.leftCm;
        int r = rover.rightCm < 0 ? SONAR_MAX_CM : rover.rightCm;
        turnDir = (r >= l) ? 1 : -1;
        motors.setDrive(-0.45f, 0);
        autoPhase = AP_BACK;
        phaseUntilMs = now + AVOID_BACK_MS;
        break;
      }
      case AP_BACK: {
        if (now < phaseUntilMs) break;
        motors.setDrive(0, 0.6f * turnDir);   // 原地转向
        autoPhase = AP_TURN;
        phaseUntilMs = now + AVOID_TURN_MS;
        break;
      }
      case AP_TURN: {
        if (now < phaseUntilMs) break;
        motors.stop();
        imu.lockHeading();                    // 新方向重新锁航向
        autoPhase = AP_CRUISE;
        break;
      }
    }
  }

  // 拍摄任务：全景 = 舵机步进 x 拍照；伪3D = 拍 -> 直行 6cm -> 拍
  void tickCapture() {
    uint32_t now = millis();
    switch (capPhase) {
      case CP_MOVE_SERVO:
        panoServoWriteImpl(panDeg);
        capWaitMs = now + PAN_SETTLE_MS;
        capPhase = CP_SHOOT;
        break;

      case CP_SHOOT: {
        if (now < capWaitMs) break;
        if (camlink.busy()) break;
        rtcPhotoSeq++;
        if (rover.task == TASK_PANORAMA)
          snprintf(shotName, sizeof(shotName), "pano_%03lu_deg%03d",
                   (unsigned long)rtcPhotoSeq, panDeg);
        else
          snprintf(shotName, sizeof(shotName), "stereo_%03lu_%s",
                   (unsigned long)rtcPhotoSeq,
                   rover.taskIndex == 0 ? "left" : "right");
        if (!camlink.capture(shotName)) { commsAlert("WARN", "CAM_BUSY"); break; }
        capPhase = CP_WAIT_SHOT;
        break;
      }

      case CP_WAIT_SHOT: {
        bool ok;
        if (!camlink.takeResult(ok)) break;
        if (!ok) commsAlert("WARN", "SHOT_FAILED");
        rover.taskIndex++;
        broadcastTask("RUNNING");
        if (rover.task == TASK_PANORAMA) {
          panDeg += stepDeg;
          if (panDeg > 180) { finishTask(); break; }
          capPhase = CP_MOVE_SERVO;
        } else {   // TASK_STEREO
          if (rover.taskIndex >= 2) { finishTask(); break; }
          // 左眼拍完 -> 锁航向直行 6cm（按标定速度换算时长）
          imu.lockHeading();
          advanceUntilMs = now + (uint32_t)(STEREO_ADVANCE_CM / CM_PER_SEC_CRUISE * 1000.0f);
          capPhase = CP_ADVANCE;
        }
        break;
      }

      case CP_ADVANCE:
        if (thermal.motionBlocked()) { motors.stop(); break; }
        if (now < advanceUntilMs) {
          motors.setDrive(AUTO_CRUISE, imu.headingCorrection());
        } else {
          motors.stop();
          imu.releaseHeading();
          capWaitMs = now + 500;    // 停稳再拍右眼
          capPhase = CP_SHOOT;
        }
        break;

      case CP_DONE:
        break;
    }
  }

  void finishTask() {
    broadcastTask("DONE");
    stopTaskInternal();
    setMode(prevMode, "capture_done");
  }

  void stopTaskInternal() {
    rover.task = TASK_NONE;
    capPhase = CP_DONE;
    motors.stop();
    panoServoWriteImpl(90);
  }

  void broadcastTask(const char* st) {
    JsonDocument doc;
    doc["type"] = "task_state";
    doc["name"] = (rover.task == TASK_PANORAMA) ? "capture_panorama"
                : (rover.task == TASK_STEREO)   ? "capture_stereo" : "none";
    doc["state"] = st;
    doc["index"] = rover.taskIndex;
    doc["total"] = rover.taskTotal;
    commsBroadcast(doc);
  }
};

extern MissionCtl mission;

// ---- state.h 前向声明的落地 ----
inline void missionHandleDrive(float t, float u) { mission.handleDrive(t, u); }
inline bool missionSetMode(Mode m, const char* r) { return mission.setMode(m, r); }
inline void missionStartTask(TaskKind k, int s)   { mission.startTask(k, s); }
inline void missionStopTask()                      { mission.stopTask(); }
inline void missionEstop(bool a)                   { mission.estop(a); }
inline void panoServoWrite(int deg)                { panoServoWriteImpl(deg); }
