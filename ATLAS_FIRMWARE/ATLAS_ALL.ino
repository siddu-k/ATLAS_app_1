// =============================================================
//  ATLAS_ALL.ino  -  SINGLE ESP32, TWO L298N drivers
//
//  L298N #1 -> Drive motors (left + right wheels)
//  L298N #2 -> Head (Pan motor + Gun motor)
//
//  Auto-stop is toggled at runtime via POST /autostop {"enabled":true|false}
//  Sensors always report to dashboard regardless of auto-stop state.
//
//  PIN MAP:
//  -- Drive L298N #1 --
//    MOTOR_EN=12 (bridge ENA+ENB), IN1=13, IN2=15, IN3=14, IN4=27
//  -- Head L298N #2 --
//    PAN_ENA=25, PAN_IN1=16, PAN_IN2=17
//    GUN_ENB=4,  GUN_IN4=2   (single direction pin)
//  -- Ultrasonics --
//    Front:       TRIG=5,  ECHO=18
//    Front-Left:  TRIG=19, ECHO=21
//    Front-Right: TRIG=22, ECHO=23
//    Back:        TRIG=33, ECHO=32
//    Down:        TRIG=26, ECHO=34
// =============================================================

#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

// ==================== WIFI ====================
const char *ssid = "vivoX200";
const char *password = "1234567890@@";

// ========= DRIVE L298N #1 PINS =========
#define MOTOR_EN 12
#define MOTOR_IN1 13
#define MOTOR_IN2 15
#define MOTOR_IN3 14
#define MOTOR_IN4 27

// ========= HEAD L298N #2 PINS =========
#define PAN_ENA 25
#define PAN_IN1 16
#define PAN_IN2 17
#define GUN_ENB 4
#define GUN_IN4 2

// ========= ULTRASONIC PINS =========
#define US_FRONT_TRIG 5
#define US_FRONT_ECHO 18
#define US_FRONT_LEFT_TRIG 19
#define US_FRONT_LEFT_ECHO 21
#define US_FRONT_RIGHT_TRIG 22
#define US_FRONT_RIGHT_ECHO 23
#define US_BACK_TRIG 33
#define US_BACK_ECHO 32
#define US_DOWN_TRIG 26
#define US_DOWN_ECHO 34

// ==================== CONSTANTS ====================
const int OBSTACLE_THRESHOLD_CM = 20;
const int POTHOLE_THRESHOLD_CM = 10;
const int PWMFreq = 1000;
const int PWMResolution = 8;
const unsigned long SENSOR_READ_INTERVAL = 100;

AsyncWebServer server(80);

// ==================== SENSOR STATE ====================
struct UltrasonicSensor {
  int trigPin, echoPin;
  float distance;
  String name;
};

UltrasonicSensor sensors[5] = {
    {US_FRONT_TRIG, US_FRONT_ECHO, 999.9, "front"},
    {US_FRONT_LEFT_TRIG, US_FRONT_LEFT_ECHO, 999.9, "front_left"},
    {US_FRONT_RIGHT_TRIG, US_FRONT_RIGHT_ECHO, 999.9, "front_right"},
    {US_BACK_TRIG, US_BACK_ECHO, 999.9, "back"},
    {US_DOWN_TRIG, US_DOWN_ECHO, 999.9, "down"}};

// ==================== CONTROL STATE ====================
int currentSpeed = 150;
String currentDirection = "stop";
int panSpeed = 200;
int gunSpeed = 255;
bool autoStopEnabled = true; // Runtime toggle from dashboard
unsigned long lastSensorRead = 0;

// =======================================================
//  DRIVE MOTOR FUNCTIONS
// =======================================================
void stopMotors() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  ledcWrite(MOTOR_EN, 0);
}

void setupMotors() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(MOTOR_EN, OUTPUT);
  ledcAttach(MOTOR_EN, PWMFreq, PWMResolution);
  stopMotors();
}

void moveForward(int speed) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  ledcWrite(MOTOR_EN, speed);
}

void moveBackward(int speed) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  ledcWrite(MOTOR_EN, speed);
}

void turnLeft(int speed) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  ledcWrite(MOTOR_EN, speed);
}

void turnRight(int speed) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  ledcWrite(MOTOR_EN, speed);
}

// =======================================================
//  PAN MOTOR FUNCTIONS
// =======================================================
void panLeft() {
  digitalWrite(PAN_IN1, HIGH);
  digitalWrite(PAN_IN2, LOW);
  ledcWrite(PAN_ENA, panSpeed);
}

void panRight() {
  digitalWrite(PAN_IN1, LOW);
  digitalWrite(PAN_IN2, HIGH);
  ledcWrite(PAN_ENA, panSpeed);
}

void panStop() {
  digitalWrite(PAN_IN1, LOW);
  digitalWrite(PAN_IN2, LOW);
  ledcWrite(PAN_ENA, 0);
}

// =======================================================
//  GUN MOTOR FUNCTIONS
// =======================================================
void gunOn() {
  digitalWrite(GUN_IN4, HIGH);
  ledcWrite(GUN_ENB, gunSpeed);
}

void gunOff() {
  digitalWrite(GUN_IN4, LOW);
  ledcWrite(GUN_ENB, 0);
}

void setUpHeadPins() {
  pinMode(PAN_IN1, OUTPUT);
  pinMode(PAN_IN2, OUTPUT);
  pinMode(GUN_IN4, OUTPUT);
  ledcAttach(PAN_ENA, PWMFreq, PWMResolution);
  ledcAttach(GUN_ENB, PWMFreq, PWMResolution);
  panStop();
  gunOff();
}

// =======================================================
//  ULTRASONIC
// =======================================================
void setupUltrasonic() {
  for (int i = 0; i < 5; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }
}

float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0)
    return 999.9;
  return duration * 0.034 / 2.0;
}

void updateSensors() {
  for (int i = 0; i < 5; i++)
    sensors[i].distance = readDistance(sensors[i].trigPin, sensors[i].echoPin);
}

bool isPathClear(const String &direction) {
  if (sensors[4].distance > POTHOLE_THRESHOLD_CM)
    return false;
  if (direction == "forward")
    return sensors[0].distance > OBSTACLE_THRESHOLD_CM &&
           sensors[1].distance > OBSTACLE_THRESHOLD_CM &&
           sensors[2].distance > OBSTACLE_THRESHOLD_CM;
  if (direction == "backward")
    return sensors[3].distance > OBSTACLE_THRESHOLD_CM;
  if (direction == "left")
    return sensors[1].distance > OBSTACLE_THRESHOLD_CM &&
           sensors[3].distance > OBSTACLE_THRESHOLD_CM;
  if (direction == "right")
    return sensors[2].distance > OBSTACLE_THRESHOLD_CM &&
           sensors[3].distance > OBSTACLE_THRESHOLD_CM;
  return true;
}

// =======================================================
//  REST API HANDLERS
// =======================================================
void handleGetStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<1024> doc;
  JsonArray sensorArray = doc.createNestedArray("ultrasonic");
  for (int i = 0; i < 5; i++) {
    JsonObject s = sensorArray.createNestedObject();
    s["name"] = sensors[i].name;
    s["distance"] = sensors[i].distance;
    s["obstacle"] = sensors[i].distance < OBSTACLE_THRESHOLD_CM;
  }
  doc["motor"]["direction"] = currentDirection;
  doc["motor"]["speed"] = currentSpeed;
  doc["auto_stop"] = autoStopEnabled;
  doc["system"]["wifi_rssi"] = WiFi.RSSI();
  doc["system"]["heap_free"] = ESP.getFreeHeap();
  doc["system"]["uptime"] = millis() / 1000;
  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

void handleMove(AsyncWebServerRequest *request, uint8_t *data, size_t len,
                size_t index, size_t total) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, data, len);
  String direction = doc["direction"] | "stop";
  int speed = doc["speed"] | 150;
  currentSpeed = constrain(speed, 0, 255);
  currentDirection = direction;

  if (autoStopEnabled && !isPathClear(direction)) {
    stopMotors();
    currentDirection = "blocked";
    request->send(200, "application/json",
                  "{\"status\":\"blocked\",\"message\":\"Obstacle detected\"}");
    return;
  }

  if (direction == "forward")
    moveForward(currentSpeed);
  else if (direction == "backward")
    moveBackward(currentSpeed);
  else if (direction == "left")
    turnLeft(currentSpeed);
  else if (direction == "right")
    turnRight(currentSpeed);
  else
    stopMotors();

  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleCamControl(AsyncWebServerRequest *request, uint8_t *data,
                      size_t len, size_t index, size_t total) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, data, len);
  String action = doc["action"] | "stop";
  if (action == "pan_left")
    panLeft();
  else if (action == "pan_right")
    panRight();
  else
    panStop();
  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleShootControl(AsyncWebServerRequest *request, uint8_t *data,
                        size_t len, size_t index, size_t total) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, data, len);
  String state = doc["state"] | "";
  if (state == "on")
    gunOn();
  else
    gunOff();
  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

void handlePanSpeedControl(AsyncWebServerRequest *request, uint8_t *data,
                           size_t len, size_t index, size_t total) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, data, len);
  panSpeed = constrain(doc["value"] | 200, 50, 255);
  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleGunSpeedControl(AsyncWebServerRequest *request, uint8_t *data,
                           size_t len, size_t index, size_t total) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, data, len);
  gunSpeed = constrain(doc["value"] | 255, 0, 255);
  ledcWrite(GUN_ENB, gunSpeed);
  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleAutoStop(AsyncWebServerRequest *request, uint8_t *data, size_t len,
                    size_t index, size_t total) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, data, len);
  autoStopEnabled = doc["enabled"] | autoStopEnabled;
  StaticJsonDocument<100> resp;
  resp["status"] = "ok";
  resp["auto_stop"] = autoStopEnabled;
  String json;
  serializeJson(resp, json);
  request->send(200, "application/json", json);
}

// =======================================================
//  SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ATLAS ALL-IN-ONE (Single ESP32, Two L298N) ===");

  setupMotors();
  setUpHeadPins();
  setupUltrasonic();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.println("================================");
    Serial.print("IP ADDRESS: ");
    Serial.println(WiFi.localIP());
    Serial.println("================================");
  } else {
    Serial.println("\nWiFi connection failed!");
  }

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods",
                                       "GET, POST, OPTIONS");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers",
                                       "Content-Type");

  // GET
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "text/plain", "ATLAS ALL OK");
  });
  server.on("/status", HTTP_GET, handleGetStatus);

  // OPTIONS (CORS preflight)
  const char *postRoutes[] = {"/move",    "/speed",   "/panspeed",
                              "/cam",     "/shoot",   "/gunspeed",
                              "/autostop", "/aux"};
  for (auto route : postRoutes)
    server.on(route, HTTP_OPTIONS,
              [](AsyncWebServerRequest *r) { r->send(200); });

  // POST
  server.on(
      "/move", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL, handleMove);
  server.on(
      "/cam", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL,
      handleCamControl);
  server.on(
      "/shoot", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL,
      handleShootControl);
  server.on(
      "/gunspeed", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL,
      handleGunSpeedControl);
  server.on(
      "/panspeed", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL,
      handlePanSpeedControl);
  server.on(
      "/autostop", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL,
      handleAutoStop);
  server.on(
      "/speed", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
         size_t index, size_t total) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, data, len);
        currentSpeed = constrain(doc["value"] | 150, 0, 255);
        request->send(200, "application/json", "{\"status\":\"ok\"}");
      });

  // AUX power (placeholder - returns not_implemented)
  server.on(
      "/aux", HTTP_POST,
      [](AsyncWebServerRequest *r) {},
      NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
         size_t index, size_t total) {
        request->send(200, "application/json",
                      "{\"status\":\"ok\",\"aux_state\":\"off\"}");
      });

  server.begin();
  Serial.println("HTTP server started");
}

// =======================================================
//  MAIN LOOP
// =======================================================
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    delay(5000);
    return;
  }
  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = now;
    updateSensors();
    // Auto-stop only if enabled from dashboard
    if (autoStopEnabled && currentDirection != "stop" &&
        currentDirection != "blocked" && !isPathClear(currentDirection)) {
      stopMotors();
      currentDirection = "blocked";
      Serial.println("Auto-stopped: Obstacle detected");
    }
  }
}
