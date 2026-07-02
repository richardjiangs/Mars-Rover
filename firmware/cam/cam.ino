// ============================================================
// PROJECT ARES - ESP32-CAM 视觉从板固件（AI-Thinker）
//
// 职责（docs/Ares_Control_Architecture.md §7.2）：
//   1. UART 从协议：应答 Commander 的 PING / CAPTURE STILL / STATUS? / STREAM
//   2. 拍照后 HTTP POST 上传到电脑端 vision/receive_server.py
//   3. /stream 提供 MJPEG 实时流（dashboard 视频窗口直接连）
//
// ⚠ 本板与 Commander 的链路复用烧录串口 U0（GPIO1/3）：
//   烧录或接串口监视器前，必须先断开与 Commander GPIO25/23 的两根线！
// ⚠ 调试输出以 "# " 开头，Commander 会忽略这类行。
//
// 烧录前：改 config.h 的 WIFI_SSID / WIFI_PASSWORD / PC_HOST
// 无需第三方库（esp_camera / WebServer / HTTPClient 都在 ESP32 核心里）
// ============================================================
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "config.h"
#include "camera_pins.h"

WebServer server(HTTP_PORT);
bool streamEnabled = true;   // STREAM START/STOP 可开关
bool camReady = false;
bool busy = false;

char rxBuf[96];
size_t rxLen = 0;

// ---------------- 相机初始化 ----------------
bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  if (psramFound()) {
    c.frame_size = FRAMESIZE_UXGA;      // 1600x1200
    c.jpeg_quality = JPEG_QUALITY;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.frame_size = FRAMESIZE_SVGA;      // 800x600
    c.jpeg_quality = JPEG_QUALITY + 4;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
  }
  return esp_camera_init(&c) == ESP_OK;
}

// ---------------- 拍照并上传到电脑 ----------------
// 成功返回 true。照片以 name 命名存到电脑 vision/incoming/<name>.jpg
bool captureAndUpload(const char* name) {
  if (!camReady) return false;
  busy = true;
#if USE_FLASH_LED
  digitalWrite(FLASH_LED_GPIO, HIGH);
  delay(60);
#endif
  camera_fb_t* fb = esp_camera_fb_get();
#if USE_FLASH_LED
  digitalWrite(FLASH_LED_GPIO, LOW);
#endif
  if (!fb) { busy = false; return false; }

  bool ok = false;
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    char url[160];
    snprintf(url, sizeof(url), "http://%s:%d/upload?name=%s", PC_HOST, PC_PORT, name);
    if (http.begin(url)) {
      http.addHeader("Content-Type", "image/jpeg");
      http.setTimeout(6000);
      int code = http.POST(fb->buf, fb->len);
      ok = (code >= 200 && code < 300);
      http.end();
    }
  }
  esp_camera_fb_return(fb);
  busy = false;
  return ok;
}

// ---------------- UART 从协议 ----------------
void handleLine(char* line) {
  // 去掉行尾空白
  size_t n = strlen(line);
  while (n && (line[n-1] == ' ' || line[n-1] == '\t')) line[--n] = 0;
  if (n == 0) return;

  if (strcmp(line, "PING") == 0) {
    Serial.print("ACK PING\n");

  } else if (strncmp(line, "CAPTURE STILL ", 14) == 0) {
    const char* name = line + 14;
    if (busy) { Serial.print("BUSY\n"); return; }
    Serial.printf("ACK CAPTURE %s\n", name);
    if (captureAndUpload(name)) Serial.printf("DONE CAPTURE %s\n", name);
    else                        Serial.print("ERR UPLOAD_FAIL\n");

  } else if (strcmp(line, "STREAM START") == 0) {
    streamEnabled = true;  Serial.print("ACK STREAM START\n");
  } else if (strcmp(line, "STREAM STOP") == 0) {
    streamEnabled = false; Serial.print("ACK STREAM STOP\n");

  } else if (strcmp(line, "STATUS?") == 0) {
    Serial.printf("STATUS %s\n", busy ? "BUSY" : (camReady ? "READY" : "ERR"));
  }
  // 其他行（包括 Commander 误收的调试行）一律忽略
}

void uartPoll() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { rxBuf[rxLen] = 0; handleLine(rxBuf); rxLen = 0; }
    else if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = ch;
  }
}

// ---------------- HTTP：信息页 / 手动拍照 / MJPEG 流 ----------------
void handleRoot() {
  char msg[240];
  snprintf(msg, sizeof(msg),
           "PROJECT ARES CAM\ncam:%s stream:%s\nPOST target: http://%s:%d/upload\n"
           "endpoints: /stream  /capture?name=test\n",
           camReady ? "READY" : "ERR", streamEnabled ? "on" : "off", PC_HOST, PC_PORT);
  server.send(200, "text/plain", msg);
}

void handleCapture() {
  String name = server.hasArg("name") ? server.arg("name") : "manual";
  bool ok = captureAndUpload(name.c_str());
  server.send(ok ? 200 : 500, "text/plain", ok ? "DONE" : "ERR");
}

// MJPEG：占住当前连接持续推帧；期间仍轮询 UART，拍照命令可插队
void handleStream() {
  WiFiClient client = server.client();
  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
               "Access-Control-Allow-Origin: *\r\n\r\n");
  while (client.connected() && streamEnabled) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) break;
    client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                  (unsigned)fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);
    uartPoll();               // 直播时也响应 Commander
    delay(50);                // ~15fps 上限，给 Wi-Fi 留带宽
  }
  client.stop();
}

// ---------------- 主流程 ----------------
void setup() {
  pinMode(STATUS_LED_GPIO, OUTPUT);
  digitalWrite(STATUS_LED_GPIO, LOW);      // 亮 = 启动中
#if USE_FLASH_LED
  pinMode(FLASH_LED_GPIO, OUTPUT);
  digitalWrite(FLASH_LED_GPIO, LOW);
#endif

  Serial.begin(LINK_BAUD);
  Serial.print("# ARES CAM boot\n");

  camReady = cameraInit();
  Serial.printf("# camera %s, psram %s\n", camReady ? "ok" : "FAIL",
                psramFound() ? "yes" : "no");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    digitalWrite(STATUS_LED_GPIO, i % 2);  // 闪烁 = 连网中
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("# ip %s (dashboard 视频地址填这个)\n",
                  WiFi.localIP().toString().c_str());
    digitalWrite(STATUS_LED_GPIO, HIGH);   // 灭 = 就绪（低电平灯）
  } else {
    Serial.print("# wifi FAIL\n");
  }

  server.on("/", handleRoot);
  server.on("/capture", handleCapture);
  server.on("/stream", handleStream);
  server.begin();
}

void loop() {
  server.handleClient();
  uartPoll();
  delay(2);
}
