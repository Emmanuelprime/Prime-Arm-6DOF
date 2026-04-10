#include "esp_camera.h"
#include <WiFi.h>

// Camera pins for AI Thinker ESP32-CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_GPIO_NUM       4   // Onboard white flash LED (active HIGH)

// WiFi credentials - YOUR NETWORK
const char* ssid = "PrimeRobotics";
const char* password = "primerobotics123";

// Static IP configuration
IPAddress staticIP(192, 168, 18, 110);   // desired static IP
IPAddress gateway(192, 168, 18,   1);    // your router IP
IPAddress subnet(255, 255, 255,   0);
IPAddress dns(8, 8, 8, 8);

WiFiServer server(80);
WiFiServer ledServer(81);  // Dedicated LED control port

void setup() {
  Serial.begin(115200);
  
  // Initialize camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // config.frame_size = FRAMESIZE_QVGA;  // Lower resolution for better performance
  config.frame_size = FRAMESIZE_VGA;  // Lower resolution for better performance
  config.jpeg_quality = 12;           // Quality (0-63, lower is better)
  config.fb_count = 1;

  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x", err);
    return;
  }

  // Connect to WiFi
  WiFi.config(staticIP, gateway, subnet, dns);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi!");
    Serial.println("Please check your credentials");
    Serial.println("Creating access point instead...");
    
    // Fallback to AP mode
    WiFi.softAP("ESP32-CAM", "12345678");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("\nWiFi connected!");
    Serial.print("Static IP: http://");
    Serial.println(WiFi.localIP());
  }

  // LED off by default
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

  server.begin();
  ledServer.begin();
  Serial.println("LED control: http://<IP>:81/led/on  or  /led/off");
}

// ── Handles a single request on the LED server (port 81) ──────────────
void handleLedServer() {
  WiFiClient lc = ledServer.available();
  if (!lc) return;
  String req = "";
  unsigned long t = millis();
  while (lc.connected() && (millis() - t) < 200) {
    if (lc.available()) {
      char c = lc.read();
      if (c == '\n') break;
      if (c != '\r') req += c;
    }
  }
  if (req.indexOf("/led/on") >= 0) {
    digitalWrite(LED_GPIO_NUM, HIGH);
    Serial.println("LED ON");
  } else if (req.indexOf("/led/off") >= 0) {
    digitalWrite(LED_GPIO_NUM, LOW);
    Serial.println("LED OFF");
  }
  lc.println("HTTP/1.1 200 OK");
  lc.println("Content-Type: text/plain");
  lc.println("Access-Control-Allow-Origin: *");
  lc.println();
  lc.println("OK");
  lc.stop();
}

void loop() {
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("New Client");
    String requestLine = "";
    String currentLine = "";
    bool firstLine = true;

    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        if (c == '\n') {
          if (firstLine) {
            requestLine = currentLine;
            firstLine = false;
          }

          if (currentLine.length() == 0) {
            // ── LED control endpoints ──────────────────────────────
            if (requestLine.indexOf("GET /led/on") >= 0) {
              digitalWrite(LED_GPIO_NUM, HIGH);
              Serial.println("LED ON");
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/plain");
              client.println("Access-Control-Allow-Origin: *");
              client.println();
              client.println("LED ON");
              break;
            } else if (requestLine.indexOf("GET /led/off") >= 0) {
              digitalWrite(LED_GPIO_NUM, LOW);
              Serial.println("LED OFF");
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/plain");
              client.println("Access-Control-Allow-Origin: *");
              client.println();
              client.println("LED OFF");
              break;
            }

            // ── MJPEG stream (default) ─────────────────────────────
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type: multipart/x-mixed-replace; boundary=frame");
            client.println();

            while (client.connected()) {
              handleLedServer();  // service LED requests while streaming

              camera_fb_t *fb = esp_camera_fb_get();
              if (!fb) {
                Serial.println("Camera capture failed");
                break;
              }

              client.println("--frame");
              client.println("Content-Type: image/jpeg");
              client.println("Content-Length: " + String(fb->len));
              client.println();
              client.write(fb->buf, fb->len);
              client.println();

              esp_camera_fb_return(fb);
              delay(30);  // Adjust frame rate
            }
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
    Serial.println("Client disconnected");
  }
}