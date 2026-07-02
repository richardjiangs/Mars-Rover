// ============================================================
// 相机链路（UART2 主端，ASCII 行协议）
// 协议依据 docs/Ares_Control_Architecture.md §11.2：
//   下发: PING | CAPTURE STILL <name> | STREAM START/STOP | STATUS?
//   回传: ACK ... | DONE ... | ERR ... | BUSY | STATUS READY
// 异步实现：发出命令 -> loop 里轮询回行，不阻塞控制回路
// ============================================================
#pragma once
#include <Arduino.h>
#include "config.h"
#include "state.h"

enum CamOp : uint8_t { CAMOP_IDLE, CAMOP_WAIT_ACK, CAMOP_WAIT_DONE };

struct CamLink {
  CamOp op = CAMOP_IDLE;
  char pendingName[32] = "";
  uint32_t deadlineMs = 0;
  uint32_t lastPingMs = 0;
  bool lastResultOk = false;
  bool resultFresh = false;      // mission 取走后清零
  char rx[96];
  size_t rxLen = 0;

  void begin() {
#if ENABLE_CAMLINK
    Serial2.begin(CAM_BAUD, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
    strcpy(rover.camStatus, "OFF");
#endif
  }

  bool busy() { return op != CAMOP_IDLE; }

  // 请求拍一张名为 name 的照片；返回 false = 链路忙
  bool capture(const char* name) {
#if !ENABLE_CAMLINK
    (void)name; return false;
#else
    if (busy()) return false;
    snprintf(pendingName, sizeof(pendingName), "%s", name);
    Serial2.printf("CAPTURE STILL %s\n", pendingName);
    op = CAMOP_WAIT_ACK;
    deadlineMs = millis() + CAM_ACK_TIMEOUT_MS;
    strcpy(rover.camStatus, "BUSY");
    return true;
#endif
  }

  // mission 轮询：拍照流程结束时返回 true，并把成败放到 ok
  bool takeResult(bool& ok) {
    if (!resultFresh) return false;
    resultFresh = false;
    ok = lastResultOk;
    return true;
  }

  void tick() {
#if ENABLE_CAMLINK
    readLines();
    uint32_t now = millis();
    // 命令超时
    if (op != CAMOP_IDLE && now > deadlineMs) {
      op = CAMOP_IDLE;
      lastResultOk = false; resultFresh = true;
      strcpy(rover.camStatus, "ERR");
      commsAlert("WARN", "CAM_TIMEOUT");
    }
    // 空闲时周期 PING 保活
    if (op == CAMOP_IDLE && now - lastPingMs > CAM_PING_INTERVAL_MS) {
      lastPingMs = now;
      Serial2.print("PING\n");
    }
#endif
  }

private:
#if ENABLE_CAMLINK
  void readLines() {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      if (c == '\r') continue;
      if (c == '\n') { rx[rxLen] = 0; handleLine(rx); rxLen = 0; }
      else if (rxLen < sizeof(rx) - 1) rx[rxLen++] = c;
    }
  }

  void handleLine(const char* line) {
    if (strncmp(line, "ACK PING", 8) == 0) {
      if (op == CAMOP_IDLE && strcmp(rover.camStatus, "BUSY") != 0)
        strcpy(rover.camStatus, "READY");
    } else if (strncmp(line, "ACK CAPTURE", 11) == 0) {
      if (op == CAMOP_WAIT_ACK) {
        op = CAMOP_WAIT_DONE;
        deadlineMs = millis() + CAM_DONE_TIMEOUT_MS;
      }
    } else if (strncmp(line, "DONE CAPTURE", 12) == 0) {
      if (op == CAMOP_WAIT_DONE || op == CAMOP_WAIT_ACK) {
        op = CAMOP_IDLE;
        lastResultOk = true; resultFresh = true;
        strcpy(rover.camStatus, "READY");
      }
    } else if (strncmp(line, "ERR", 3) == 0) {
      if (op != CAMOP_IDLE) {
        op = CAMOP_IDLE;
        lastResultOk = false; resultFresh = true;
      }
      strcpy(rover.camStatus, "ERR");
      commsAlert("WARN", "CAM_ERR");
    } else if (strncmp(line, "BUSY", 4) == 0) {
      strcpy(rover.camStatus, "BUSY");
    } else if (strncmp(line, "STATUS", 6) == 0) {
      strcpy(rover.camStatus, strstr(line, "READY") ? "READY" : "BUSY");
    }
  }
#endif
};

extern CamLink camlink;
