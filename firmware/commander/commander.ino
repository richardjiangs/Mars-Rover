// ============================================================
// PROJECT ARES - Commander ESP32 主控固件
//
// 职责（docs/Ares_Control_Architecture.md §7.1）：
//   电机差速 + 软启动 | MPU6050 直线保持 | 三向超声波避障(AUTO-L2)
//   模式状态机 IDLE/AUTO/TELEOP-PC/CAPTURE/E-STOP/SLEEP
//   热控五态机 | 休眠唤醒 + Final_Mission_Mode | 云台舵机
//   UART2 指挥 ESP32-CAM | WebSocket 遥测/遥控（dashboard/index.html）
//
// 烧录前：改 config.h 里的 WIFI_SSID / WIFI_PASSWORD
// 需要的库：ArduinoJson(>=7)、WebSockets(links2004)，见本目录 README.md
// 安全：首次调试必须架空车轮！
// ============================================================
#include "config.h"
#include "state.h"
#include "motors.h"
#include "imu.h"
#include "sonar.h"
#include "thermal.h"
#include "powersleep.h"
#include "camlink.h"
#include "mission.h"
#include "comms.h"

// ---- 全局单例（各模块 extern 引用） ----
RoverState rover;
MotorsCtl  motors;
ImuCtl     imu;
SonarCtl   sonar;
ThermalCtl thermal;
PowerCtl   power;
CamLink    camlink;
MissionCtl mission;
CommsCtl   comms;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n==== PROJECT ARES Commander ====");

  motors.begin();          // 先确保电机输出为 0
  power.begin();           // 唤醒计数（RTC 内存）
  Serial.printf("[boot] wake_count=%lu final_mission=%d\n",
                (unsigned long)rover.wakeCount, rover.finalMission);

  Serial.println("[boot] IMU 标定中，请保持整车静止 2 秒...");
  if (!imu.begin()) {
    Serial.println("[boot] MPU6050 初始化失败！检查 I2C 接线（或在 config.h 互换 SDA/SCL）");
    commsAlert("ERROR", "IMU_FAIL");   // WiFi 未起时只打印串口
  }

  sonar.begin();
  thermal.begin();
  camlink.begin();
  mission.begin();
  comms.begin();           // 最后连 WiFi，串口会打印 IP

  // 最终任务模式：唤醒次数到 80 次后自动全速执行拍摄任务（步骤 5"冲刺模式"）
  if (rover.finalMission) {
    commsAlert("WARN", "FINAL_MISSION_MODE");
    missionSetMode(MODE_AUTO, "final_mission");
  }
  Serial.println("[boot] ready.");
}

void loop() {
  imu.tick();        // 姿态积分（尽量高频）
  sonar.tick();      // 三向测距轮询
  thermal.tick();    // 热控状态机
  power.tick();      // 电压/光照采样与入睡判断
  camlink.tick();    // 相机链路收发
  mission.tick();    // 模式与任务状态机（产生驱动指令）
  motors.tick();     // 软启动斜坡输出
  comms.tick();      // WebSocket 收发 + 5Hz 遥测
}
