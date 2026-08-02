// ESP32 camera + wireless bridge for the self-balancing robot.
//
// Architecture: ESP32 streams MJPEG over WiFi; vision/SLAM runs on the
// computer; commands flow computer -> WebSocket -> UART -> Uno. The ESP32
// is a dumb, reliable pipe — no control logic lives here.
//
//   HTTP :80   /        status JSON (chip, camera, RSSI, uptime, counters)
//              /stream  MJPEG multipart stream
//   WS   :81   command channel: text in -> UART to Uno verbatim (+\n);
//              every UART line from the Uno is broadcast to all clients.
//              On last-client disconnect an e-stop frame is sent to the
//              Uno ($x*78) — never leave the robot driven with no console.
//   UART       Serial0 (UART0, the kit cable header) at 250000 baud,
//              matching the Uno console. Commands should be pre-framed by
//              the client as $payload*XOR; the Uno validates.
//
// POWER (CLAUDE.md protocol item 5): never connect USB-C and the robot
// UART cable at the same time. USB-C = flashing/bench only.

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>   // arduino-cli lib install "WebSockets"
#include "esp_camera.h"
#include "esp_http_server.h"
#include "camera_pins.h"
#include "secrets.h"            // WIFI_SSID / WIFI_PASS (git-ignored)

#define MDNS_NAME "robot-cam"
#define UNO_BAUD 38400
// Kit UART header wiring per Elegoo's stock S3 firmware
// (ESP32_CameraServer_AP_2023_V1.3: Serial2.begin(9600, SERIAL_8N1, 3, 40)):
// RX = GPIO3, TX = GPIO40. Stock runs 9600; 250000 was silent through the
// module level shifter, so the link runs 38400 (Uno console matches).
#define UNO_SERIAL Serial2
#define UNO_RX_PIN 3
#define UNO_TX_PIN 40

WebSocketsServer ws(81);
// Remote Uno reflash: avrdude -c arduino -P net:robot-cam.local:2323
// Flow: send 'R' over WS (Uno jumps to optiboot), avrdude retries sync
// through this raw TCP<->UART bridge at 115200; console returns to
// 38400 when the client disconnects.
WiFiServer flashServer(2323);
httpd_handle_t httpServer = NULL;
bool cameraOk = false;
uint32_t framesServed = 0;
uint32_t cmdsForwarded = 0;
uint32_t linesUp = 0;
char uartLine[220];
size_t uartLen = 0;

// ------------------------------------------------------------------ camera --
bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
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
  c.frame_size = FRAMESIZE_VGA;      // 640x480; drop to QVGA if fps poor
  c.jpeg_quality = 12;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c) == ESP_OK;
}

// ------------------------------------------------------------------- http --
static esp_err_t streamHandler(httpd_req_t* req) {
  if (!cameraOk) {
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no camera");
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  char hdr[64];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    size_t n = snprintf(hdr, sizeof(hdr),
        "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
        (unsigned)fb->len);
    if (httpd_resp_send_chunk(req, hdr, n) != ESP_OK ||
        httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len) != ESP_OK ||
        httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) {
      esp_camera_fb_return(fb);
      return ESP_OK;             // client left
    }
    esp_camera_fb_return(fb);
    framesServed++;
  }
}

static esp_err_t statusHandler(httpd_req_t* req) {
  char buf[320];
  snprintf(buf, sizeof(buf),
    "{\"chip\":\"%s\",\"camera\":%s,\"psram\":%s,\"rssi\":%d,"
    "\"uptime_s\":%lu,\"frames\":%lu,\"cmds\":%lu,\"uart_lines\":%lu,"
    "\"heap\":%lu}\n",
    ESP.getChipModel(), cameraOk ? "true" : "false",
    psramFound() ? "true" : "false", WiFi.RSSI(),
    (unsigned long)(millis() / 1000), (unsigned long)framesServed,
    (unsigned long)cmdsForwarded, (unsigned long)linesUp,
    (unsigned long)ESP.getFreeHeap());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

void startHttp() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  if (httpd_start(&httpServer, &cfg) == ESP_OK) {
    httpd_uri_t s = { .uri = "/stream", .method = HTTP_GET,
                      .handler = streamHandler, .user_ctx = NULL };
    httpd_uri_t i = { .uri = "/", .method = HTTP_GET,
                      .handler = statusHandler, .user_ctx = NULL };
    httpd_register_uri_handler(httpServer, &s);
    httpd_register_uri_handler(httpServer, &i);
  }
}

// -------------------------------------------------------------- websocket --
void wsEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t len) {
  switch (type) {
    case WStype_TEXT:
      UNO_SERIAL.write(payload, len);
      UNO_SERIAL.write('\n');
      cmdsForwarded++;
      break;
    case WStype_DISCONNECTED:
      if (ws.connectedClients() == 0) {
        // Last console just vanished: e-stop, same policy as log_trial.py
        UNO_SERIAL.print("$x*78\n");
      }
      break;
    default:
      break;
  }
}

// ------------------------------------------------------------------ setup --
void setup() {
  Serial.begin(115200);          // USB-CDC debug (bench only)
  UNO_SERIAL.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // low latency beats power savings here
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("wifi ok ip=%s rssi=%d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("wifi FAILED (will keep retrying)");
  }

  MDNS.begin(MDNS_NAME);
  MDNS.addService("http", "tcp", 80);

  cameraOk = initCamera();
  Serial.printf("camera %s, psram %s\n", cameraOk ? "OK" : "FAILED",
                psramFound() ? "yes" : "no");

  startHttp();
  ws.begin();
  ws.onEvent(wsEvent);
  flashServer.begin();
  Serial.printf("ready: http://%s.local/  ws://%s.local:81/\n",
                MDNS_NAME, MDNS_NAME);
}

// ------------------------------------------------------------------- loop --
void loop() {
  // Raw flash bridge takes over the UART while a client is connected
  WiFiClient fc = flashServer.accept();
  if (fc) {
    Serial.println("flash bridge: client connected, 115200");
    UNO_SERIAL.updateBaudRate(115200);
    uint8_t buf[256];
    uint32_t lastActivity = millis();
    while (fc.connected() && millis() - lastActivity < 30000) {
      int n = fc.available();
      if (n > 0) {
        n = fc.read(buf, min(n, (int)sizeof(buf)));
        UNO_SERIAL.write(buf, n);
        lastActivity = millis();
      }
      n = UNO_SERIAL.available();
      if (n > 0) {
        n = UNO_SERIAL.read(buf, min(n, (int)sizeof(buf)));
        fc.write(buf, n);
        lastActivity = millis();
      }
      ws.loop();          // keep WS alive-ish during long flashes
      delay(1);
    }
    fc.stop();
    UNO_SERIAL.updateBaudRate(UNO_BAUD);
    Serial.println("flash bridge: closed, back to console baud");
  }

  ws.loop();

  // UART -> WebSocket, line at a time
  while (UNO_SERIAL.available() > 0) {
    char ch = (char)UNO_SERIAL.read();
    if (ch == '\r') continue;
    if (ch == '\n' || uartLen >= sizeof(uartLine) - 1) {
      if (uartLen > 0) {
        uartLine[uartLen] = '\0';
        ws.broadcastTXT(uartLine, uartLen);
        linesUp++;
      }
      uartLen = 0;
    } else {
      uartLine[uartLen++] = ch;
    }
  }

  // WiFi self-heal
  static uint32_t lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
}
