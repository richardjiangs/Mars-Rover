// ============================================================
// PROJECT ARES - ESP32-CAM 配置
// ============================================================
#pragma once

// ---------- Wi-Fi（烧录前必填，和 Commander 用同一个网络） ----------
#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"
#define HOSTNAME        "ares-cam"

// ---------- 电脑端收图服务（vision/receive_server.py） ----------
// 电脑开热点时，Windows 移动热点网关通常是 192.168.137.1
#define PC_HOST         "192.168.137.1"
#define PC_PORT         8000

// ---------- 图像参数 ----------
// 有 PSRAM 时用 FRAMESIZE_UXGA(1600x1200)，无 PSRAM 自动降到 SVGA(800x600)
#define JPEG_QUALITY    12      // 10-63，越小越清晰、文件越大
#define USE_FLASH_LED   0       // 拍照时闪光（GPIO4 大灯，耗电且刺眼，默认关）

// ---------- 串口协议（与 Commander UART2 相连，共用烧录口 U0） ----------
#define LINK_BAUD       115200

// ---------- HTTP 服务 ----------
#define HTTP_PORT       80      // /        信息页
                                // /stream  MJPEG 实时流（dashboard 视频窗口用）
                                // /capture?name=xxx  手动触发拍照上传（调试用）
