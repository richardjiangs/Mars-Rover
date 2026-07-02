# Commander 主控固件（ESP32 NodeMCU）

主控板固件：电机差速、直线保持、三向避障、模式状态机、休眠模拟、指挥相机板、WebSocket 遥测遥控。引脚与 `hardware/Ares_V3_GPIO_Map.md` 一致。

## 一、准备 Arduino IDE（一次性）

1. 安装 [Arduino IDE 2.x](https://www.arduino.cc/en/software)。
2. `文件 → 首选项 → 其他开发板管理器地址` 填入：
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. `工具 → 开发板 → 开发板管理器` 搜索 `esp32`，安装 **esp32 by Espressif Systems**（2.x 或 3.x 均可，本固件已做兼容）。
4. `工具 → 管理库` 安装两个库：
   - **ArduinoJson**（Benoit Blanchon，版本 ≥ 7.0）
   - **WebSockets**（Markus Sattler / links2004，版本 ≥ 2.4）
5. 可选件启用时才需要的库：
   - `ENABLE_DS18B20 1` → 再装 **OneWire** 和 **DallasTemperature**

## 二、烧录

1. 用 Arduino IDE 打开本目录（`commander.ino`）。
2. 修改 `config.h` 顶部的 `WIFI_SSID` / `WIFI_PASSWORD`（建议用电脑开热点，见 `docs/Ares_Control_Architecture.md` §6.1）。
3. 开发板选 **ESP32 Dev Module**，默认参数即可；插 USB 选对串口。
4. 点上传。若一直 `Connecting...`，按住板上 `BOOT` 键再点上传。
5. 打开串口监视器（115200），会看到 IMU 标定提示和 **WiFi IP 地址**——dashboard 里要填这个 IP。

## 三、首次上电自检顺序（务必架空车轮！）

1. 上电后 2 秒内不要动车（IMU 零偏标定）。
2. 串口出现 `ready.` 后，浏览器打开 `dashboard/index.html`，填 IP → 连接。
3. 依次验证：遥测数字在跳 → E-STOP 按下电机锁死 → TELEOP 下 WASD 车轮转向正确 → AUTO 下手挡超声波会触发停车转向。
4. 车轮转向不对：对调该侧电机接线，或在 `config.h` 里交换该侧 FWD/REV 引脚号。

## 四、config.h 里最常改的参数

| 参数 | 何时改 |
| --- | --- |
| `WIFI_SSID/PASSWORD` | 必改 |
| `PIN_SONAR_*` | 装车后发现 F/L/R 对不上时对调 |
| `PIN_I2C_SDA/SCL` | MPU6050 初始化失败时互换（手绘图与标准相反） |
| `YAW_KP/KD` | T2 直线测试跑偏时调（先 P 后 D，见测试指南） |
| `CM_PER_SEC_CRUISE` | T2 实测速度后回填（影响伪 3D 的 6cm 前进精度） |
| `ENABLE_LDR/DS18B20/VBAT` | 对应可选件装上后置 1 |
| `MOTOR_MAX_PCT` | 单 L298N 过热时调低 |

## 五、与其他部分的接口

- **对 dashboard**：WebSocket `ws://<板子IP>:81`，JSON 协议见 `docs/Ares_Control_Architecture.md` §11.1。
- **对 ESP32-CAM**：UART2（TX=GPIO25→CAM GPIO3，RX=GPIO23←CAM GPIO1），ASCII 行协议见 §11.2。烧录 CAM 时先拔掉这两根线。
- **单独调试底盘**：把 `config.h` 的 `ENABLE_CAMLINK` 置 0，可在不接相机板时消除 CAM_TIMEOUT 告警。
