#include <Arduino.h>
#include <WiFi.h>
#if __has_include("config.local.h")
#include "config.local.h"
#endif
#include "config.example.h"
#include <WebServer.h>
#include "web_page.h"

/* ================= WIFI STA ================= */
const char* ssid     = YEAR2_WIFI_SSID;        // tên WiFi
const char* password = YEAR2_WIFI_PASSWORD;   // mật khẩu WiFi

WebServer server(80);

/* ================= UART ESP32-C3 -> NANO ================= */
// GPIO4 ESP32-C3 nối vào D0/RX của Nano
#define ESP_TX_TO_NANO 4

// Không dùng RX, nhưng Serial1 cần khai báo
#define ESP_RX_UNUSED 5

HardwareSerial NanoSerial(1);

/* ================= SEND TO NANO ================= */
void sendToNano(char cmd) {
  NanoSerial.print(cmd);

  Serial.print("DA GUI SANG NANO: ");
  Serial.println(cmd);
}

/* ================= HTML PAGE ================= */
String htmlPage() {
  String page = YEAR2_WEB_PAGE;

  return page;
}

/* ================= WEB HANDLERS ================= */
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleCmd() {
  if (!YEAR2_ENABLE_REMOTE_CONTROL) {
    server.send(503, "text/plain", "Remote control disabled in source archive");
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    server.send(503, "text/plain", "WIFI NOT CONNECTED");
    return;
  }

  if (server.hasArg("key")) {
    String key = server.arg("key");

    if (key.length() > 0) {
      char cmd = key.charAt(0);

      if (cmd == '1') {
        sendToNano(cmd);
        server.send(200, "text/plain", "OK - SENT 1 TO NANO");
        return;
      }
    }
  }

  server.send(400, "text/plain", "BAD CMD");
}

/* ================= WIFI CONNECT ================= */
bool connectWiFi() {
  Serial.println();
  Serial.print("Dang ket noi WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  if (!YEAR2_ENABLE_REMOTE_CONTROL) {
    Serial.println("Wi-Fi/HTTP disabled in source archive");
    return false;
  }
  WiFi.begin(ssid, password);

  int retry = 0;

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retry++;

    if (retry > 30) {
      Serial.println();
      Serial.println("Ket noi WiFi that bai!");
      return false;
    }
  }

  Serial.println();
  Serial.println("Da ket noi WiFi thanh cong!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  return true;
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  // UART gửi sang Nano
  NanoSerial.begin(9600, SERIAL_8N1, ESP_RX_UNUSED, ESP_TX_TO_NANO);

  bool wifiOK = connectWiFi();

  if (wifiOK) {
    server.on("/", handleRoot);
    server.on("/cmd", HTTP_POST, handleCmd);

    server.begin();

    Serial.println("Web server started.");
    Serial.print("Mo trinh duyet vao: http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Khong tao web server vi WiFi chua ket noi.");
  }
}

/* ================= LOOP ================= */
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (YEAR2_ENABLE_REMOTE_CONTROL) server.handleClient();
  }
}