# ESP32-CAM 视觉从板固件（AI-Thinker）

拍照 → HTTP 上传到电脑 `vision/receive_server.py`；提供 `/stream` MJPEG 实时流给 dashboard；通过串口应答 Commander 的拍照命令。协议见 `docs/Ares_Control_Architecture.md` §11.2。

**无需安装第三方库**（相机驱动、WebServer、HTTPClient 都在 ESP32 核心包里）。

## 烧录（用你买的下载器底板最省事）

1. 把 ESP32-CAM 插到 **下载器底板** 上，USB 连电脑。
   （没有底板的话：用 USB-TTL，5V/GND/U0R↔TX/U0T↔RX，且把 `GPIO0 接 GND` 进入烧录模式。）
2. ⚠ **如果板子已装上车：先断开与 Commander GPIO25/23 相连的两根串口线**，否则烧录会失败。
3. Arduino IDE 打开本目录（`cam.ino`），改 `config.h`：
   - `WIFI_SSID / WIFI_PASSWORD`：与 Commander 同一网络；
   - `PC_HOST`：电脑在该网络中的 IP（Windows 移动热点通常是 `192.168.137.1`）。
4. 开发板选 **AI Thinker ESP32-CAM**，点上传。
   （若列表里没有，选 ESP32 Dev Module 并把 PSRAM 设为 Enabled 也可。）
5. 上传完 **把 GPIO0 从 GND 断开**（用底板则拨开关/直接拔插），按 RST 重启。

## 自检

1. 串口监视器（115200）看 `# ip 192.168.x.x` —— 这是相机 IP。
2. 浏览器开 `http://<相机IP>/stream` 应看到实时画面。
3. 电脑先跑 `python vision/receive_server.py`，再访问 `http://<相机IP>/capture?name=test`，
   `vision/incoming/test.jpg` 出现即链路全通。
4. 装上车接回串口线后，Commander 遥测里 `cam` 字段应变为 `READY`。

## 板载指示灯

- 红色小灯（GPIO33）：常亮=启动中，闪烁=连 Wi-Fi，灭=就绪。
- 白色大灯（GPIO4）：默认不用；`config.h` 里 `USE_FLASH_LED 1` 可作拍照补光（耗电）。

## 已知限制

- 与 Commander 的链路复用烧录串口 U0：调试输出以 `# ` 开头，Commander 会忽略；但**烧录/看串口时必须断开车内串口线**。
- MJPEG 直播和拍照上传共用同一颗摄像头，直播中拍照会让画面停顿约 1 秒，正常现象。
