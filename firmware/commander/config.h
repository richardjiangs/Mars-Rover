// ============================================================
// PROJECT ARES - Commander ESP32 全车配置
// 引脚与 hardware/Ares_V3_GPIO_Map.md 保持一致；改引脚先改那份文档
// ============================================================
#pragma once

// ---------- Wi-Fi（烧录前必填） ----------
#define WIFI_SSID       "YOUR_WIFI_SSID"      // 改成你的热点/路由器名
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"  // 改成密码
#define WS_PORT         81                    // WebSocket 服务端口（dashboard 连这里）
#define HOSTNAME        "ares-commander"

// ---------- 功能开关（未接的可选件保持 0，不影响编译运行） ----------
#define ENABLE_CAMLINK  1   // UART2 连接 ESP32-CAM（断开联调时可置 0）
#define ENABLE_DS18B20  0   // 舱内温度传感器（需安装 OneWire + DallasTemperature 库）
#define ENABLE_LDR      0   // 光敏模块（休眠唤醒演示）
#define ENABLE_VBAT     0   // 电池电压分压检测（GPIO36）

// ---------- 电机（单 L298N：IN1-IN4，直接 PWM 调速） ----------
#define PIN_ML_FWD      16  // 左侧 3 电机 正转（L298N IN1）
#define PIN_ML_REV      17  // 左侧 3 电机 反转（L298N IN2）
#define PIN_MR_FWD      18  // 右侧 3 电机 正转（L298N IN3）
#define PIN_MR_REV      19  // 右侧 3 电机 反转（L298N IN4）
#define MOTOR_PWM_FREQ  1000   // Hz，L298N(BJT) 适用
#define MOTOR_PWM_BITS  10     // 分辨率 0-1023
#define MOTOR_MAX_PCT   90     // 最高占空比限幅（保护单 L298N）
#define MOTOR_RAMP_PCT  5      // 软启动：每个控制周期最多变化 5%
#define MOTOR_TICK_MS   20     // 控制周期

// ---------- 超声波（3 路独立，F/L/R 若与实车不符只改这里） ----------
#define PIN_SONAR_F_TRIG 13
#define PIN_SONAR_F_ECHO 14   // 所有 Echo 必须经 1k/2k 分压！
#define PIN_SONAR_L_TRIG 27
#define PIN_SONAR_L_ECHO 26
#define PIN_SONAR_R_TRIG 33
#define PIN_SONAR_R_ECHO 32
#define SONAR_INTERVAL_MS 40   // 轮询间隔（三路防串扰）
#define SONAR_MAX_CM      300

// ---------- IMU（MPU6050，I2C） ----------
// 注意：手绘线路图标注 SCL-D21/SDA-D22 与此相反；若按手绘接线，把下面两行互换
#define PIN_I2C_SDA     21
#define PIN_I2C_SCL     22
#define IMU_CALIB_MS    2000   // 上电静置标定时长（期间别动车）

// ---------- 直线保持 PID（先只调 P，再加 D） ----------
#define YAW_KP          0.030f  // 每偏 1° 差速修正量（0-1 油门单位）
#define YAW_KI          0.000f
#define YAW_KD          0.008f
#define YAW_CORR_MAX    0.35f   // 差速修正限幅

// ---------- 自动驾驶（AUTO-L2 三向感知） ----------
#define OBSTACLE_STOP_CM   20   // 前方低于此距离停车选路
#define AUTO_CRUISE        0.55f // 巡航油门 0-1
#define AVOID_BACK_MS      400  // 遇障后退时长
#define AVOID_TURN_MS      600  // 原地转向时长
#define CM_PER_SEC_CRUISE  15.0f // 巡航速度标定值（T2 测试后按实测改！）
#define STEREO_ADVANCE_CM  6.0f  // 伪 3D 两拍之间直线前进距离（完整指南步骤 4）

// ---------- 云台舵机（SG90，0-180°） ----------
#define PIN_SERVO_PAN   4
#define PAN_STEP_DEG    30     // 全景每步角度
#define PAN_SETTLE_MS   600    // 舵机到位等待

// ---------- 相机链路（UART2，115200 8N1） ----------
#define PIN_CAM_TX      25   // -> ESP32-CAM GPIO3 (U0RXD)
#define PIN_CAM_RX      23   // <- ESP32-CAM GPIO1 (U0TXD)
#define CAM_BAUD        115200
#define CAM_ACK_TIMEOUT_MS   1500
#define CAM_DONE_TIMEOUT_MS  8000  // 拍照+上传较慢
#define CAM_PING_INTERVAL_MS 3000

// ---------- 遥测 / 心跳 ----------
#define TELEMETRY_MS         200   // 5Hz
#define HEARTBEAT_TIMEOUT_MS 1500  // TELEOP 心跳超时 -> 停车

// ---------- 休眠与"100 天生存"模拟 ----------
#define WAKE_LIMIT           80    // 唤醒次数达到后触发 Final_Mission_Mode
#define SLEEP_TIMER_WAKE_S   30    // 无 LDR 时的定时唤醒（演示用）
#define LDR_DARK_THRESHOLD   500   // analogRead(0-4095) 低于视为黑夜
#define LDR_DARK_HOLD_MS     10000 // 持续黑暗超过此时长才入睡
#define PIN_LDR_AO           34
#define PIN_LDR_DO           35
#define PIN_DS18B20          15
#define PIN_VBAT             36
#define VBAT_DIVIDER         4.03f // (100k+33k)/33k，按实测微调

// ---------- 热控阈值（docs/Ares_机械减震与保温系统补充说明.md 3.5.4） ----------
#define T_COLD_SLEEP    -10.0f
#define T_COLD_START      0.0f
#define T_HOT_LIMIT      45.0f
#define T_THERMAL_PAUSE  50.0f
#define HOT_LIMIT_PCT    60    // HOT_LIMIT 状态下的占空比上限
#define COLD_START_PCT   40    // COLD_START 状态下的占空比上限
