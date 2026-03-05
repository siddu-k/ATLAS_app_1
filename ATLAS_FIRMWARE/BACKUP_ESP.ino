// =============================================================
//  atlas_all_nouv.ino  -  SINGLE ESP32, TWO L298N drivers
//
//  ** NO UV OBSTACLE STOP VERSION **
//  Ultrasonic sensor readings are sent to the dashboard normally,
//  but motors are NEVER stopped or blocked based on sensor data.
//
//  L298N #1 -> Drive motors (left + right wheels)
//  L298N #2 -> Head (Pan motor + Gun motor)
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
#define MOTOR_EN 12 // Bridge ENA & ENB on L298N #1 to this pin
#define MOTOR_IN1 13
#define MOTOR_IN2 15
#define MOTOR_IN3 14
#define MOTOR_IN4 27

// ========= HEAD L298N #2 PINS =========
// -- Pan Motor (Channel A) --
#define PAN_ENA 25
#define PAN_IN1 16
#define PAN_IN2 17

// -- Gun Motor (Channel B) - single direction pin --
#define GUN_ENB 4 // Gun motor speed (PWM)
#define GUN_IN4 2 // Gun direction pin (HIGH=ON)

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
// These thresholds are kept for reference / dashboard display only.
// They do NOT trigger any motor stop.
const int OBSTACLE_THRESHOLD_CM = 20;
const int POTHOLE_THRESHOLD_CM = 10;
const int PWMFreq = 1000;
const int PWMResolution = 8;
const unsigned long SENSOR_READ_INTERVAL = 100;

AsyncWebServer server(80);

// ==================== SENSOR STATE ====================
struct UltrasonicSensor {
  int trigPin;
  int echoPin;
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
//  GUN MOTOR FUNCTIONS  (single direction)
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

// =======================================================
//  REST API HANDLERS
// =======================================================

// GET /status  - returns sensor readings + motor state to dashboard
// NOTE: "obstacle" flag is still reported so the dashboard can display it,
//       but the firmware itself takes NO action based on it.
void handleGetStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<1024> doc;
  JsonArray sensorArray = doc.createNestedArray("ultrasonic");
  for (int i = 0; i < 5; i++) {
    JsonObject sensor = sensorArray.createNestedObject();
    sensor["name"] = sensors[i].name;
    sensor["distance"] = sensors[i].distance;
    // still flag so dashboard can show warning colour, but no stop logic
    sensor["obstacle"] = sensors[i].distance < OBSTACLE_THRESHOLD_CM;
  }
  doc["motor"]["direction"] = currentDirection;
  doc["motor"]["speed"] = currentSpeed;
  doc["system"]["wifi_rssi"] = WiFi.RSSI();
  doc["system"]["heap_free"] = ESP.getFreeHeap();
  doc["system"]["uptime"] = millis() / 1000;
  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

// POST /move  - ALWAYS executes the movement, never blocks on sensor data
void handleMove(AsyncWebServerRequest *request, uint8_t *data, size_t len,
                size_t index, size_t total) {
  StaticJsonDocument<200> doc;
  deserializeJson(doc, data);
  String direction = doc["direction"] | "stop";
  int speed = doc["speed"] | 150;
  currentSpeed = constrain(speed, 0, 255);
  currentDirection = direction;

  // --- No obstacle check here. Just drive. ---
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

  StaticJsonDocument<100> resp;
  resp["status"] = "ok";
  String json;
  serializeJson(resp, json);
  request->send(200, "application/json", json);
}

void handleCamControl(AsyncWebServerRequest *request, uint8_t *data, size_t len,
                      size_t index, size_t total) {
  String body = "";
  for (size_t i = 0; i < len; i++)
    body += (char)data[i];
  StaticJsonDocument<200> doc;
  deserializeJson(doc, body);
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
  String body = "";
  for (size_t i = 0; i < len; i++)
    body += (char)data[i];
  StaticJsonDocument<200> doc;
  deserializeJson(doc, body);
  String state = doc["state"] | "";
  if (state == "on")
    gunOn();
  else
    gunOff();
  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

// POST /panspeed  { "value": 200 }
void handlePanSpeedControl(AsyncWebServerRequest *request, uint8_t *data,
                           size_t len, size_t index, size_t total) {
  String body = "";
  for (size_t i = 0; i < len; i++)
    body += (char)data[i];
  StaticJsonDocument<200> doc;
  deserializeJson(doc, body);
  panSpeed = doc["value"] | 200;
  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleGunSpeedControl(AsyncWebServerRequest *request, uint8_t *data,
                           size_t len, size_t index, size_t total) {
  String body = "";
  for (size_t i = 0; i < len; i++)
    body += (char)data[i];
  StaticJsonDocument<200> doc;
  deserializeJson(doc, body);
  gunSpeed = doc["value"] | 255;
  ledcWrite(GUN_ENB, gunSpeed); // Apply immediately if gun is running
  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

// =======================================================
//  SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== ATLAS ALL-IN-ONE (No UV Stop) ===");

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
    Serial.println(
        "Endpoints: /move /speed /panspeed /status /cam /shoot /gunspeed");
  } else {
    Serial.println("\nWiFi connection failed!");
  }

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods",
                                       "GET, POST, OPTIONS");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers",
                                       "Content-Type");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "text/plain", "ATLAS ALL (No UV Stop) OK");
  });
  server.on("/status", HTTP_GET, handleGetStatus);

  server.on("/move", HTTP_OPTIONS,
            [](AsyncWebServerRequest *r) { r->send(200); });
  server.on("/speed", HTTP_OPTIONS,
            [](AsyncWebServerRequest *r) { r->send(200); });
  server.on("/panspeed", HTTP_OPTIONS,
            [](AsyncWebServerRequest *r) { r->send(200); });
  server.on("/cam", HTTP_OPTIONS,
            [](AsyncWebServerRequest *r) { r->send(200); });
  server.on("/shoot", HTTP_OPTIONS,
            [](AsyncWebServerRequest *r) { r->send(200); });
  server.on("/gunspeed", HTTP_OPTIONS,
            [](AsyncWebServerRequest *r) { r->send(200); });

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
      "/speed", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL,
      [](AsyncWebServerRequest *request, uint8_t *data, size_t len,
         size_t index, size_t total) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, data);
        currentSpeed = constrain(doc["value"] | 150, 0, 255);
        request->send(200, "application/json", "{\"status\":\"ok\"}");
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
    // Read all sensors every 100 ms so /status always has fresh data.
    // No obstacle check — motors are never auto-stopped here.
    updateSensors();
  }
}
