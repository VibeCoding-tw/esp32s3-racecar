#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_heap_caps.h>
#include <esp_camera.h>

// ==========================================
// 1. SAFE PIN DEFINITIONS (UPDATED)
// ==========================================

// --- Camera Pins (Freenove S3 WROOM) ---
#define PWDN_GPIO_NUM  48
#define RESET_GPIO_NUM -1 
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  3  // Pin 3 for SCL

#define Y9_GPIO_NUM    46
#define Y8_GPIO_NUM    45
#define Y7_GPIO_NUM    42
#define Y6_GPIO_NUM    41
#define Y5_GPIO_NUM    40
#define Y4_GPIO_NUM    39
#define Y3_GPIO_NUM    38 
#define Y2_GPIO_NUM    37
#define VSYNC_GPIO_NUM 5
#define HREF_GPIO_NUM  6
#define PCLK_GPIO_NUM  7

// --- Motor Pins (NEW SAFE MAPPING) ---
// We avoid GPIO 1, 2, 43, 44 (Serial Logs)
// We avoid GPIO 33-37 (PSRAM)
// We avoid GPIO 3-10, 38-48 (Camera)

#define MOTOR_A_FWD   11  // New Pin
#define MOTOR_A_REV   12  // New Pin
#define MOTOR_B_LEFT  13  // New Pin
#define MOTOR_B_RIGHT 14  // New Pin
#define MOTOR_STBY    21  // Keep on 21

// ==========================================
// 2. Globals
// ==========================================
WebSocketsServer webSocket(81);
AsyncWebServer server(80);

const int CH_A_FWD = 0;
const int CH_A_REV = 1;
const int CH_B_LEFT = 2;
const int CH_B_RIGHT = 3;
const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int MAX_DUTY = 200;

volatile int targetA = 0;
volatile int targetB = 0;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 300;

enum DriveMode { AUTO, MANUAL };
DriveMode currentMode = MANUAL;

// ==========================================
// 3. Helpers
// ==========================================

void sendLogMessage(const String& message) {
  Serial.println(message); // Now this should work always!
  webSocket.broadcastTXT(message.c_str(), message.length());
}

void emergencyStopNow() {
  targetA = targetB = 0;
  digitalWrite(MOTOR_STBY, LOW);
  ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, 0);
  ledcWrite(CH_B_LEFT, 0); ledcWrite(CH_B_RIGHT, 0);
  sendLogMessage("!!! EMERGENCY STOP !!!");
}

void jumpToFactory() {
  sendLogMessage("WiFi Failed. Rebooting...");
  const esp_partition_t* factory = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
  if (factory) esp_ota_set_boot_partition(factory);
  delay(500);
  ESP.restart();
}

void connectToWiFi() {
  unsigned long start = millis();
  WiFi.mode(WIFI_STA);
  WiFi.begin(); 
  Serial.println("Connecting to WiFi...");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    if (millis() - start > 15000) {
      jumpToFactory();
      return;
    }
  }
  sendLogMessage("WiFi Connected: " + WiFi.localIP().toString());
}

// ==========================================
// 4. Camera Setup
// ==========================================
bool initCamera() {
  Serial.println("--- Starting Camera Init ---");
  
  // 1. Check PSRAM
  if(psramFound()){
    Serial.printf("PSRAM Found: %u bytes free\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  } else {
    Serial.println("⚠️ WARNING: No PSRAM detected! Video will be low quality.");
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 37;
  config.pin_d1 = 38;
  config.pin_d2 = 39;
  config.pin_d3 = 40;
  config.pin_d4 = 41;
  config.pin_d5 = 42;
  config.pin_d6 = 45;
  config.pin_d7 = 46;
  config.pin_xclk = 10;
  config.pin_pclk = 7;
  config.pin_vsync = 5;
  config.pin_href = 6;
  config.pin_sscb_sda = 4;
  config.pin_sscb_scl = 3; // Must be 3
  config.pin_pwdn = 48;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // 2. Attempt Init
  Serial.println("Calling esp_camera_init...");
  esp_err_t err = esp_camera_init(&config);
  
  if (err != ESP_OK) {
    Serial.printf("❌ Camera init FAILED! Error: 0x%x\n", err);
    if (err == 0x106) Serial.println("  -> Error 0x106: Camera Not Supported (Check SCL/SDA pins or Ribbon Cable)");
    if (err == 0x101) Serial.println("  -> Error 0x101: Out of Memory (Check PSRAM settings)");
    return false;
  }
  
  Serial.println("✅ Camera init SUCCEEDED!");
  return true;
}
/*
bool initCamera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  
  sensor_t * s = esp_camera_sensor_get();
  if (s) { s->set_vflip(s, 1); s->set_hmirror(s, 1); }
  
  Serial.println("Camera init succeeded!");
  return true;
}
*/
// ==========================================
// 5. Handlers
// ==========================================

void httpStreamHandler(AsyncWebServerRequest *request) {
    AsyncWebServerResponse *response = request->beginResponseStream("multipart/x-mixed-replace;boundary=123456789000000000000987654321");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
    
    while (request->client()->connected()) {
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) { delay(50); continue; }
        char header[128];
        snprintf(header, 128, "--123456789000000000000987654321\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", fb->len);
        request->client()->write(header, strlen(header));
        request->client()->write((const char*)fb->buf, fb->len);
        request->client()->write("\r\n", 2);
        esp_camera_fb_return(fb);
        delay(1); 
    }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED: emergencyStopNow(); break;
    case WStype_TEXT: {
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, (const char*)payload); 
        if (!err) {
          int steer = doc["steer"] | 0;
          int throttle = doc["throttle"] | 0;
          if (currentMode == MANUAL) {
              targetA = throttle * MAX_DUTY / 100;
              targetB = steer * MAX_DUTY / 100;
              int speedA = constrain(targetA, -MAX_DUTY, MAX_DUTY);
              int speedB = constrain(targetB, -MAX_DUTY, MAX_DUTY);

              if (speedA != 0 || speedB != 0) digitalWrite(MOTOR_STBY, HIGH);
              else digitalWrite(MOTOR_STBY, LOW);

              if (speedA > 0) { ledcWrite(CH_A_FWD, speedA); ledcWrite(CH_A_REV, 0); } 
              else { ledcWrite(CH_A_FWD, 0); ledcWrite(CH_A_REV, abs(speedA)); }

              if (speedB > 0) { ledcWrite(CH_B_RIGHT, speedB); ledcWrite(CH_B_LEFT, 0); } 
              else { ledcWrite(CH_B_RIGHT, 0); ledcWrite(CH_B_LEFT, abs(speedB)); }
              lastCommandTime = millis(); 
          }
        }
    } break;
  }
}

const char index_html[] = R"rawliteral(
<!doctype html>
<html>
<head><meta name="viewport" content="width=device-width,initial-scale=1"><title>Robot</title>
<style>body{margin:0;background:#111;overflow:hidden;touch-action:none}
#video{width:100%;height:100vh;object-fit:contain}
.stick{width:100px;height:100px;background:#ffffff22;border-radius:50%;position:absolute;bottom:20px}
#stickL{left:20px} #stickR{right:20px}
.knob{width:40px;height:40px;background:#3b82f6;border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%)}
</style></head>
<body><img id="video" src=""><div id="stickL" class="stick"><div class="knob"></div></div><div id="stickR" class="stick"><div class="knob"></div></div>
<script>
const ws=new WebSocket(`ws://${location.hostname}:81`);
document.getElementById('video').src=`http://${location.hostname}/stream`;
function bind(id,cb){
  let act=false,el=document.getElementById(id),knob=el.children[0];
  const mv=e=>{if(!act)return;e.preventDefault();let t=e.touches?e.touches[0]:e,r=el.getBoundingClientRect(),m=r.width/2,x=t.clientX-r.left-m,y=t.clientY-r.top-m,d=Math.hypot(x,y);if(d>m){x*=m/d;y*=m/d}knob.style.transform=`translate(calc(-50% + ${x}px),calc(-50% + ${y}px))`;cb(x/m,y/m)};
  const end=()=>{act=false;knob.style.transform=`translate(-50%,-50%)`;cb(0,0)};
  el.onmousedown=el.ontouchstart=()=>{act=true};window.ontouchmove=window.onmousemove=mv;window.ontouchend=window.onmouseup=end;
}
bind('stickL',(x,y)=>ws.send(JSON.stringify({steer:Math.round(x*100)})));
bind('stickR',(x,y)=>ws.send(JSON.stringify({throttle:Math.round(-y*100)})));
</script></body></html>
)rawliteral";

// ==========================================
// 6. SETUP & LOOP
// ==========================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n--- SYSTEM STARTING ---");

  // 1. Config Motor Standby
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, LOW);
  
  // 2. Connect WiFi 
  connectToWiFi();

  // 3. Init Camera 
  if (initCamera()) {
      sendLogMessage("Camera ON. Stream at /stream");
  } else {
      sendLogMessage("Camera FAILED.");
  }

  // 4. Start Web Server
  setupOTA();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send(200, "text/html", index_html); });
  server.on("/stream", HTTP_GET, httpStreamHandler);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  // 5. Init Motor PWM
  // Using pins 11, 12, 13, 14 should NOT kill serial!
  Serial.println("Initializing Motors...");
  
  ledcSetup(CH_A_FWD, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_A_FWD, CH_A_FWD);
  ledcSetup(CH_A_REV, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_A_REV, CH_A_REV);
  ledcSetup(CH_B_LEFT, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_B_LEFT, CH_B_LEFT);
  ledcSetup(CH_B_RIGHT, PWM_FREQ, PWM_RES); ledcAttachPin(MOTOR_B_RIGHT, CH_B_RIGHT);
  
  Serial.println("System Ready!");
}

void loop() {
  ArduinoOTA.handle();
  webSocket.loop();
  
  if (millis() - lastCommandTime > COMMAND_TIMEOUT && (targetA!=0 || targetB!=0)) {
    targetA = targetB = 0;
    digitalWrite(MOTOR_STBY, LOW);
  }
}