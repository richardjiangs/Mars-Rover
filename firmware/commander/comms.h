// ============================================================
// 电脑端链路：Wi-Fi + WebSocket 服务器（端口 WS_PORT）
// 消息格式依据 docs/Ares_Control_Architecture.md §11.1（JSON）
//   下行: heartbeat / set_mode / drive / task / estop / pan / sleep_now
//   上行: telemetry(5Hz) / alert / task_state / mode
// 依赖库: WebSockets (links2004/arduinoWebSockets), ArduinoJson v7
// ============================================================
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "config.h"
#include "state.h"
#include "motors.h"

WebSocketsServer wsServer(WS_PORT);

struct CommsCtl {
  uint32_t lastTelemetryMs = 0;
  bool wifiOk = false;

  void begin() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[comms] WiFi connecting");
    // 最多等 15s；连不上继续跑（可先用串口调试底盘）
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
      delay(500); Serial.print(".");
    }
    wifiOk = (WiFi.status() == WL_CONNECTED);
    if (wifiOk) {
      Serial.printf("\n[comms] IP: %s  (dashboard 里填这个地址)\n",
                    WiFi.localIP().toString().c_str());
    } else {
      Serial.println("\n[comms] WiFi FAILED - 继续离线运行");
    }
    wsServer.begin();
    wsServer.onEvent([](uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
      if (type == WStype_TEXT) commsCtlHandle(num, payload, len);
      else if (type == WStype_CONNECTED) Serial.printf("[comms] client %u connected\n", num);
    });
  }

  void tick() {
    wsServer.loop();
    uint32_t now = millis();
    if (now - lastTelemetryMs >= TELEMETRY_MS) {
      lastTelemetryMs = now;
      sendTelemetry();
    }
  }

  void broadcast(JsonDocument& doc) {
    char buf[384];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    wsServer.broadcastTXT((uint8_t*)buf, n);
  }

  void alert(const char* level, const char* code) {
    JsonDocument doc;
    doc["type"] = "alert"; doc["level"] = level; doc["code"] = code;
    broadcast(doc);
    Serial.printf("[alert] %s %s\n", level, code);
  }

  void sendTelemetry() {
    JsonDocument doc;
    doc["type"] = "telemetry";
    doc["mode"] = modeName(rover.mode);
    doc["battery_v"] = ((int)(rover.batteryV * 100)) / 100.0f;
    doc["yaw"] = ((int)(rover.yawDeg * 10)) / 10.0f;
    // 归一化油门估计（-1..1，地图航迹推算用：速度 ≈ thr * CM_PER_SEC_CRUISE）
    doc["thr"] = ((int)((motors.curL + motors.curR) / 2.0f)) / 100.0f;
    doc["front_cm"] = rover.frontCm;
    doc["left_cm"] = rover.leftCm;
    doc["right_cm"] = rover.rightCm;
    doc["thermal"] = thermalName(rover.thermal);
    if (!isnan(rover.insideTempC))
      doc["inside_temp_c"] = ((int)(rover.insideTempC * 10)) / 10.0f;
    doc["cam"] = rover.camStatus;
    doc["wake_count"] = rover.wakeCount;
    doc["final_mission"] = rover.finalMission;
    if (rover.lightRaw >= 0) doc["light"] = rover.lightRaw;
    if (rover.task != TASK_NONE) {
      doc["task_index"] = rover.taskIndex;
      doc["task_total"] = rover.taskTotal;
    }
    broadcast(doc);
  }

  // 静态转发（onEvent 的 lambda 不能捕获 this）
  static void commsCtlHandle(uint8_t num, uint8_t* payload, size_t len);
};

extern CommsCtl comms;

inline void commsBroadcast(JsonDocument& doc) { comms.broadcast(doc); }
inline void commsAlert(const char* level, const char* code) { comms.alert(level, code); }

// ---------------- 指令分发 ----------------
inline void CommsCtl::commsCtlHandle(uint8_t num, uint8_t* payload, size_t len) {
  (void)num;
  JsonDocument doc;
  if (deserializeJson(doc, payload, len) != DeserializationError::Ok) return;
  const char* type = doc["type"] | "";

  if (strcmp(type, "heartbeat") == 0) {
    rover.lastHeartbeatMs = millis();

  } else if (strcmp(type, "set_mode") == 0) {
    const char* m = doc["mode"] | "";
    rover.lastHeartbeatMs = millis();
    if      (strcmp(m, "AUTO") == 0)      missionSetMode(MODE_AUTO, "pc_request");
    else if (strcmp(m, "TELEOP-PC") == 0) missionSetMode(MODE_TELEOP, "pc_request");
    else if (strcmp(m, "IDLE") == 0)      missionSetMode(MODE_IDLE, "pc_request");

  } else if (strcmp(type, "drive") == 0) {
    rover.lastHeartbeatMs = millis();
    missionHandleDrive(doc["throttle"] | 0.0f, doc["turn"] | 0.0f);

  } else if (strcmp(type, "task") == 0) {
    const char* name = doc["name"] | "";
    if (strcmp(name, "capture_panorama") == 0)
      missionStartTask(TASK_PANORAMA, doc["step_deg"] | PAN_STEP_DEG);
    else if (strcmp(name, "capture_stereo") == 0)
      missionStartTask(TASK_STEREO, 0);
    else if (strcmp(name, "stop") == 0)
      missionStopTask();

  } else if (strcmp(type, "estop") == 0) {
    missionEstop(doc["active"] | true);

  } else if (strcmp(type, "pan") == 0) {
    panoServoWrite(doc["deg"] | 90);

  } else if (strcmp(type, "sleep_now") == 0) {   // 休眠演示
    missionSetMode(MODE_SLEEP, "pc_request");
  }
}
