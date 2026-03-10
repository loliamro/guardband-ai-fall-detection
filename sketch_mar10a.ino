#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// WiFi
#define WIFI_SSID "KamalJallad"
#define WIFI_PASS "67676767"
#define PC_PORT   9000

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C
#define OLED_SDA 4
#define OLED_SCL 5

// MPU6050
#define MPU_SDA 21
#define MPU_SCL 22
#define MPU_ADDR 0x68

// LEDs
#define RED_LED 13
#define BLUE_LED 12

// Fall Thresholds
#define FALL_LOWER_G   0.6
#define FALL_UPPER_G   1.8
#define STILLNESS_G    0.50
#define FREEFALL_MS    50
#define CANCEL_MS      30000
#define SAMPLE_MS      10
#define BUF_SIZE       50
#define EMERGENCY_HOLD 5000

// AI Anomaly Thresholds
#define CALIB_SECONDS    60
#define ZSCORE_THRESH    3.0
#define TREMOR_THRESH    0.8
#define STILL_TIMEOUT_MS 120000
#define SLOW_COLLAPSE_G  0.3

// I2C buses
TwoWire I2C_MPU  = TwoWire(0);
TwoWire I2C_OLED = TwoWire(1);

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);

// WiFi server
WiFiServer server(PC_PORT);
WiFiClient client;

// FSM States
enum State { CALIBRATING, IDLE, FREEFALL, IMPACT, FALL_CONFIRMED, EMERGENCY };
State state = CALIBRATING;

// Fall detection vars
unsigned long freefallStart = 0;
unsigned long fallConfirmedAt = 0;
unsigned long emergencyStart = 0;
float magBuf[BUF_SIZE];
int bufIdx = 0;
bool bufFull = false;
float peakG = 0;

// AI Calibration vars
float calibSum = 0;
float calibSumSq = 0;
int calibCount = 0;
float calibMean = 0;
float calibStd = 0;
unsigned long calibStart = 0;

// AI Anomaly vars
unsigned long lastMovementTime = 0;
float recentMags[10];
int recentIdx = 0;
String anomalyType = "";
bool anomalyActive = false;
unsigned long anomalyStart = 0;

void connectWiFi() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(18, 0);
  display.println("== GUARDBAND AI ==");
  display.setCursor(20, 20);
  display.println("Connecting WiFi...");
  display.setCursor(10, 35);
  display.println(WIFI_SSID);
  display.display();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
    digitalWrite(BLUE_LED, !digitalRead(BLUE_LED));
  }

  if (WiFi.status() == WL_CONNECTED) {
    server.begin();
    display.clearDisplay();
    display.setCursor(18, 0);
    display.println("== GUARDBAND AI ==");
    display.setCursor(0, 15);
    display.println("WiFi Connected!");
    display.setCursor(0, 30);
    display.println("IP Address:");
    display.setCursor(0, 42);
    display.println(WiFi.localIP().toString());
    display.setCursor(0, 54);
    display.println("Open dashboard now!");
    display.display();
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    delay(3000);
  } else {
    display.clearDisplay();
    display.setCursor(18, 0);
    display.println("== GUARDBAND AI ==");
    display.setCursor(10, 25);
    display.println("WiFi FAILED!");
    display.setCursor(10, 40);
    display.println("Running offline");
    display.display();
    delay(2000);
  }
}

void setup() {
  Serial.begin(115200);

  I2C_MPU.begin(MPU_SDA, MPU_SCL);
  I2C_OLED.begin(OLED_SDA, OLED_SCL);
  delay(300);

  // Wake up MPU6050
  I2C_MPU.beginTransmission(MPU_ADDR);
  I2C_MPU.write(0x6B);
  I2C_MPU.write(0x00);
  I2C_MPU.endTransmission(true);
  delay(200);

  // Set accel range to +-4g
  I2C_MPU.beginTransmission(MPU_ADDR);
  I2C_MPU.write(0x1C);
  I2C_MPU.write(0x08);
  I2C_MPU.endTransmission(true);
  delay(100);

  // LEDs
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(RED_LED, LOW);

  // Start OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED NOT FOUND");
    while (true);
  }

  // Boot screen
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(15, 5);
  display.println("GUARD");
  display.setCursor(15, 30);
  display.println("BAND AI");
  display.setTextSize(1);
  display.setCursor(20, 55);
  display.println("Starting up...");
  display.display();
  delay(2000);

  // Connect WiFi
  connectWiFi();

  // Start calibration
  calibStart = millis();
  lastMovementTime = millis();
}

void readFull(float &Ax, float &Ay, float &Az,
              float &Gx, float &Gy, float &Gz) {
  I2C_MPU.beginTransmission(MPU_ADDR);
  I2C_MPU.write(0x3B);
  I2C_MPU.endTransmission(false);
  I2C_MPU.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = (I2C_MPU.read() << 8) | I2C_MPU.read();
  int16_t ay = (I2C_MPU.read() << 8) | I2C_MPU.read();
  int16_t az = (I2C_MPU.read() << 8) | I2C_MPU.read();
  I2C_MPU.read(); I2C_MPU.read();
  int16_t gx = (I2C_MPU.read() << 8) | I2C_MPU.read();
  int16_t gy = (I2C_MPU.read() << 8) | I2C_MPU.read();
  int16_t gz = (I2C_MPU.read() << 8) | I2C_MPU.read();

  Ax = ax / 8192.0; Ay = ay / 8192.0; Az = az / 8192.0;
  Gx = gx / 131.0;  Gy = gy / 131.0;  Gz = gz / 131.0;
}

float getStdDev() {
  int n = bufFull ? BUF_SIZE : bufIdx;
  if (n < 2) return 999.0;
  float sum = 0;
  for (int i = 0; i < n; i++) sum += magBuf[i];
  float mean = sum / n;
  float sq = 0;
  for (int i = 0; i < n; i++) sq += pow(magBuf[i] - mean, 2);
  return sqrt(sq / (n - 1));
}

float getRecentStdDev() {
  float sum = 0;
  for (int i = 0; i < 10; i++) sum += recentMags[i];
  float mean = sum / 10;
  float sq = 0;
  for (int i = 0; i < 10; i++) sq += pow(recentMags[i] - mean, 2);
  return sqrt(sq / 9);
}

void sendToPC(String msg) {
  if (client && client.connected()) {
    client.println(msg);
  }
}

void showCalibrating() {
  int elapsed = (millis() - calibStart) / 1000;
  int remaining = CALIB_SECONDS - elapsed;
  int progress = (elapsed * 100) / CALIB_SECONDS;
  int barWidth = (progress * 80) / 100;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(18, 0);
  display.println("== GUARDBAND AI ==");
  display.drawLine(0, 9, 127, 9, WHITE);
  display.setCursor(25, 14);
  display.println("AI Calibrating...");
  display.setCursor(10, 26);
  display.println("Move around normally!");
  display.drawRect(4, 38, 84, 10, WHITE);
  display.fillRect(4, 38, barWidth, 10, WHITE);
  display.setCursor(92, 38);
  display.print(progress);
  display.println("%");
  display.setCursor(35, 52);
  display.print("Time left: ");
  display.print(remaining);
  display.println("s");
  display.display();

  digitalWrite(BLUE_LED, (millis() / 500) % 2);
}

void showAnomaly(float Ax, float Ay, float Az, float mag) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(18, 0);
  display.println("== GUARDBAND AI ==");
  display.drawLine(0, 9, 127, 9, WHITE);
  display.setCursor(0, 12);
  display.println("!! WARNING !!");
  display.drawLine(0, 22, 127, 22, WHITE);
  display.setCursor(0, 26);
  display.print("Type: ");
  display.println(anomalyType);
  display.setCursor(0, 38);
  display.print("Ax:"); display.print(Ax, 2);
  display.setCursor(65, 38);
  display.print("|a|:"); display.println(mag, 2);
  display.drawLine(0, 48, 127, 48, WHITE);
  display.setCursor(0, 52);
  display.println("Check patient now!");
  display.display();
}

void updateOLED(float Ax, float Ay, float Az,
                float mag, String status, int countdown) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(18, 0);
  display.println("== GUARDBAND AI ==");
  display.drawLine(0, 9, 127, 9, WHITE);
  display.setCursor(0, 12);
  display.print("Status: ");
  display.println(status);
  display.drawLine(0, 22, 127, 22, WHITE);
  display.setCursor(0, 26);
  display.print("Ax:"); display.print(Ax, 2);
  display.setCursor(65, 26);
  display.print("Ay:"); display.println(Ay, 2);
  display.setCursor(0, 36);
  display.print("Az:"); display.print(Az, 2);
  display.setCursor(65, 36);
  display.print("|a|:"); display.println(mag, 2);
  display.drawLine(0, 46, 127, 46, WHITE);
  display.setCursor(0, 50);
  if (state == FALL_CONFIRMED) {
    display.print("Cancel in: ");
    display.print(countdown);
    display.println("s");
  } else if (state == EMERGENCY) {
    display.println("!! EMERGENCY !!");
  } else if (state == FREEFALL) {
    display.println("!! FREEFALL !!");
  } else {
    display.print("Peak: ");
    display.print(peakG, 2);
    display.println("g");
  }
  display.display();
}

void resetSystem() {
  state = IDLE;
  freefallStart = 0;
  bufIdx = 0;
  bufFull = false;
  peakG = 0;
  emergencyStart = 0;
  anomalyActive = false;
  anomalyType = "";
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
}

void checkAnomalies(float mag) {
  recentMags[recentIdx] = mag;
  recentIdx = (recentIdx + 1) % 10;

  float zScore = 0;
  if (calibStd > 0) {
    zScore = abs(mag - calibMean) / calibStd;
  }

  if (zScore > 0.5) lastMovementTime = millis();
  if (state != IDLE) return;

  if (anomalyActive) {
    if (millis() - anomalyStart > 10000) {
      anomalyActive = false;
      anomalyType = "";
      digitalWrite(RED_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
    }
    return;
  }

  // 1. Z-Score anomaly
  if (zScore > ZSCORE_THRESH) {
    anomalyActive = true;
    anomalyType = "Unusual Move";
    anomalyStart = millis();
    digitalWrite(RED_LED, HIGH);
    String alert = "{\"type\":\"anomaly\",\"anomaly\":\"unusual_movement\",\"zscore\":" + String(zScore) + "}";
    sendToPC(alert);
    return;
  }

  // 2. Tremor detection
  if (recentIdx == 0) {
    float recentStd = getRecentStdDev();
    if (recentStd > TREMOR_THRESH && recentStd < 1.5) {
      anomalyActive = true;
      anomalyType = "Tremor";
      anomalyStart = millis();
      digitalWrite(RED_LED, HIGH);
      String alert = "{\"type\":\"anomaly\",\"anomaly\":\"tremor\",\"std\":" + String(recentStd) + "}";
      sendToPC(alert);
      return;
    }
  }

  // 3. No movement
  if (millis() - lastMovementTime > STILL_TIMEOUT_MS) {
    anomalyActive = true;
    anomalyType = "No Movement";
    anomalyStart = millis();
    lastMovementTime = millis();
    digitalWrite(RED_LED, HIGH);
    sendToPC("{\"type\":\"anomaly\",\"anomaly\":\"no_movement\"}");
    return;
  }

  // 4. Slow collapse
  static unsigned long driftStart = 0;
  if (abs(mag - calibMean) > SLOW_COLLAPSE_G && mag < calibMean) {
    if (driftStart == 0) driftStart = millis();
    if (millis() - driftStart > 5000) {
      anomalyActive = true;
      anomalyType = "Slow Collapse";
      anomalyStart = millis();
      driftStart = 0;
      digitalWrite(RED_LED, HIGH);
      sendToPC("{\"type\":\"anomaly\",\"anomaly\":\"slow_collapse\"}");
      return;
    }
  } else {
    driftStart = 0;
  }
}

void loop() {
  // Handle new client connections
  if (!client || !client.connected()) {
    client = server.available();
  }

  static unsigned long lastSample = 0;
  if (millis() - lastSample < SAMPLE_MS) return;
  lastSample = millis();

  float Ax, Ay, Az, Gx, Gy, Gz;
  readFull(Ax, Ay, Az, Gx, Gy, Gz);
  float mag = sqrt(Ax*Ax + Ay*Ay + Az*Az);

  // CALIBRATION
  if (state == CALIBRATING) {
    calibSum += mag;
    calibSumSq += mag * mag;
    calibCount++;

    static unsigned long lastCalibDisplay = 0;
    if (millis() - lastCalibDisplay > 200) {
      lastCalibDisplay = millis();
      showCalibrating();
    }

    if (millis() - calibStart >= (CALIB_SECONDS * 1000)) {
      calibMean = calibSum / calibCount;
      float variance = (calibSumSq / calibCount) - (calibMean * calibMean);
      calibStd = sqrt(variance);
      state = IDLE;
      lastMovementTime = millis();
      digitalWrite(BLUE_LED, HIGH);
      digitalWrite(RED_LED, LOW);

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.setCursor(18, 0);
      display.println("== GUARDBAND AI ==");
      display.setCursor(20, 20);
      display.setTextSize(2);
      display.println("AI Ready!");
      display.setTextSize(1);
      display.setCursor(10, 50);
      display.print("Mean:");
      display.print(calibMean, 2);
      display.print(" Std:");
      display.println(calibStd, 2);
      display.display();

      sendToPC("{\"type\":\"calibration\",\"mean\":" + String(calibMean) + ",\"std\":" + String(calibStd) + "}");
      delay(2000);
    }
    return;
  }

  if (mag > peakG) peakG = mag;

  magBuf[bufIdx] = mag;
  bufIdx = (bufIdx + 1) % BUF_SIZE;
  if (bufIdx == 0) bufFull = true;

  checkAnomalies(mag);

  int countdown = 30 - (int)((millis() - fallConfirmedAt) / 1000);

  String status = "NORMAL";
  if (state == FREEFALL)            status = "FREEFALL";
  else if (state == IMPACT)         status = "IMPACT";
  else if (state == FALL_CONFIRMED) status = "FALL!";
  else if (state == EMERGENCY)      status = "EMERGENCY";

  // FSM
  switch (state) {
    case IDLE:
      if (!anomalyActive) {
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(RED_LED, LOW);
      }
      if (mag < FALL_LOWER_G) {
        if (freefallStart == 0) freefallStart = millis();
        if (millis() - freefallStart >= FREEFALL_MS) {
          state = FREEFALL;
          anomalyActive = false;
        }
      } else {
        freefallStart = 0;
      }
      break;

    case FREEFALL:
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(RED_LED, HIGH);
      if (mag > FALL_UPPER_G) {
        state = IMPACT;
      } else if (mag >= FALL_LOWER_G) {
        freefallStart = 0;
        state = IDLE;
      }
      break;

    case IMPACT:
      if (getStdDev() < STILLNESS_G) {
        state = FALL_CONFIRMED;
        fallConfirmedAt = millis();
        digitalWrite(RED_LED, HIGH);
        digitalWrite(BLUE_LED, LOW);
        sendToPC("{\"type\":\"fall\",\"event\":\"FALL_CONFIRMED\",\"peakG\":" + String(peakG) + "}");
      }
      break;

    case FALL_CONFIRMED:
      digitalWrite(RED_LED, (millis() / 300) % 2);
      if (millis() - fallConfirmedAt >= CANCEL_MS) {
        state = EMERGENCY;
        emergencyStart = millis();
      }
      break;

    case EMERGENCY:
      digitalWrite(RED_LED, HIGH);
      digitalWrite(BLUE_LED, LOW);
      sendToPC("{\"type\":\"emergency\",\"event\":\"EMERGENCY\",\"peakG\":" + String(peakG) + "}");
      if (millis() - emergencyStart >= EMERGENCY_HOLD) {
        resetSystem();
      }
      return;

    default:
      break;
  }

  // Update display every 100ms
  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay >= 100) {
    lastDisplay = millis();
    if (anomalyActive && state == IDLE) {
      showAnomaly(Ax, Ay, Az, mag);
    } else {
      updateOLED(Ax, Ay, Az, mag, status, countdown);
    }
  }

  // Send sensor data to PC every 100ms
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 100) {
    lastSend = millis();
    String json = "{\"type\":\"sensor\",\"ax\":" + String(Ax, 2) +
                  ",\"ay\":" + String(Ay, 2) +
                  ",\"az\":" + String(Az, 2) +
                  ",\"mag\":" + String(mag, 2) +
                  ",\"state\":" + String((int)state) + "}";
    sendToPC(json);
  }
}