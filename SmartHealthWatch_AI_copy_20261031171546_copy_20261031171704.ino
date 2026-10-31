
#define BLYNK_TEMPLATE_ID "TMPL2H7Lg3tqM"
#define BLYNK_TEMPLATE_NAME "Fall detection"
#define BLYNK_AUTH_TOKEN "CCAnUgW1aeYGniYouHSJFIUekuNgc_PF"

#define BLYNK_PRINT Serial
#include "gesture_model_optimized_c.h"

// ============================================================================
// LIBRARIES
// ============================================================================
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <math.h>

// ============================================================================
// WIFI + BLYNK NETWORK CREDENTIALS
// ============================================================================
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Ninou";
char pass[] = "12202005nmmm";

BlynkTimer timer;

// ============================================================================
// OLED DISPLAY CONFIGURATION
// ============================================================================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0, U8X8_PIN_NONE
);

// ============================================================================
// SENSOR INSTANCES
// ============================================================================
Adafruit_MPU6050 mpu;
MAX30105 particleSensor;

// ============================================================================
// RUNNER ANIMATION GRAPHICS
// ============================================================================
const unsigned char runner_A[] U8X8_PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x00,
  0x0C,0x00,0xFC,0x01,0xFC,0x00,0x0C,0x00,
  0x4C,0x06,0xCC,0x19,0x80,0x30,0x00,0x30,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

const unsigned char runner_B[] U8X8_PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,
  0x18,0x00,0x3E,0x00,0x18,0x00,0x18,0x00,
  0x18,0x0C,0x2C,0x0C,0x06,0x0C,0x06,0x06,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

// ============================================================================
// REAL-TIME CLOCK VARIABLES
// ============================================================================
int hours = 10;
int minutes = 30;
int seconds = 0;
unsigned long lastClockTick = 0;

// ============================================================================
// STEP COUNTER VARIABLES
// ============================================================================
int stepCount = 0;
float threshold = 14.0;
bool stepDetected = false;
unsigned long lastStepTime = 0;
unsigned long stepDelay = 300;
bool animState = false;

// ============================================================================
// HEART RATE VARIABLES
// ============================================================================
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE] = {0};
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg = 0;

// ============================================================================
// ECG SCREEN WAVE BUFFER
// ============================================================================
int waveBuffer[128];
int wavePhase = 0;

// ============================================================================
// FALL DETECTION CONSTANTS
// ============================================================================
const float FREE_FALL_THRESHOLD = 4.5;
const float IMPACT_THRESHOLD = 30.0;
const float STILLNESS_THRESHOLD = 0.5;

const unsigned long IMPACT_WINDOW = 1500;
const unsigned long STILLNESS_TIME = 5000;
const unsigned long ALERT_COOLDOWN = 15000;

// ============================================================================
// FALL DETECTION TRACKING STATES
// ============================================================================
bool freeFallDetected = false;
bool impactDetected = false;
bool alertSent = false;

unsigned long freeFallTime = 0;
unsigned long impactTime = 0;
unsigned long lastAlertTime = 0;

// ============================================================================
// RUNTIME TIMING TRACKERS
// ============================================================================
unsigned long lastDisplayTime = 0;

// Forward declarations
void drawClockScreen();
void drawHeartScreen();
void sendDataToBlynk();

// ============================================================================
// HARDWARE INITIALIZATION
// ============================================================================
void setup() {
  Serial.begin(115200);

  // ESP32-C3 Super Mini I2C pins
  Wire.begin(8, 9);

  // OLED initialization
  u8g2.begin();

  for (int i = 0; i < 128; i++) {
    waveBuffer[i] = 45;
  }

  // MPU6050 initialization
  Serial.println("Initializing MPU6050...");

  if (!mpu.begin()) {
    Serial.println("CRITICAL ERROR: MPU6050 not found");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 connected successfully");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // MAX30102 / MAX30105 initialization
  Serial.println("Initializing MAX30105...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("CRITICAL ERROR: MAX30105 not found");
    while (1) {
      delay(10);
    }
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.setSampleRate(100);
  particleSensor.setPulseWidth(411);
  particleSensor.setADCRange(4096);

  Serial.println("MAX30105 connected successfully");

  // Blynk connection
  Blynk.begin(auth, ssid, pass);

  // Upload data every second
  timer.setInterval(1000L, sendDataToBlynk);
}

// ============================================================================
// SEND DATA TO BLYNK
// ============================================================================
void sendDataToBlynk() {
  Blynk.virtualWrite(V2, stepCount);
  Blynk.virtualWrite(V3, beatAvg);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  Blynk.run();
  timer.run();

  unsigned long currentTime = millis();

  // 1. DATA ACQUISITION
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  long irValue = particleSensor.getIR();

  // Calculate acceleration magnitude
  float totalAcc = sqrt(
    (a.acceleration.x * a.acceleration.x) +
    (a.acceleration.y * a.acceleration.y) +
    (a.acceleration.z * a.acceleration.z)
  );

  // Calculate gyroscope magnitude
  float totalGyro = sqrt(
    (g.gyro.x * g.gyro.x) +
    (g.gyro.y * g.gyro.y) +
    (g.gyro.z * g.gyro.z)
  );

  // 2. PEDOMETER MONITORING
  if (totalAcc > threshold &&
      !stepDetected &&
      (currentTime - lastStepTime > stepDelay)) {

    stepCount++;
    stepDetected = true;
    lastStepTime = currentTime;
    animState = !animState;

    Serial.print("Step Registered -> Total: ");
    Serial.println(stepCount);
  }

  if (totalAcc < 10.5) {
    stepDetected = false;
  }

  // 3. HEART RATE MONITORING
  if (irValue > 20000) {
    if (checkForBeat(irValue)) {
      long delta = currentTime - lastBeat;
      lastBeat = currentTime;

      if (delta > 0) {
        beatsPerMinute = 60.0 / (delta / 1000.0);

        if (beatsPerMinute > 40 && beatsPerMinute < 220) {
          rates[rateSpot++] = (byte)beatsPerMinute;
          rateSpot %= RATE_SIZE;

          beatAvg = 0;

          for (byte i = 0; i < RATE_SIZE; i++) {
            beatAvg += rates[i];
          }

          beatAvg /= RATE_SIZE;

          Serial.print("Heartbeat Detected -> Running Avg BPM: ");
          Serial.println(beatAvg);
        }
      }

      wavePhase = 1;
    }
  } else {
    beatAvg = 0;
  }

  // 4. FALL DETECTION

  // Phase 1: Detect free fall
  if (!freeFallDetected && !impactDetected) {
    if (totalAcc < FREE_FALL_THRESHOLD) {
      freeFallDetected = true;
      freeFallTime = currentTime;

      Serial.println(
        "[FALL STATUS]: Phase 1 - Free fall detected!"
      );
    }
  }

  // Phase 2: Detect impact
  if (freeFallDetected && !impactDetected) {
    if ((currentTime - freeFallTime <= IMPACT_WINDOW) &&
        (totalAcc > IMPACT_THRESHOLD)) {

      impactDetected = true;
      impactTime = currentTime;

      Serial.println(
        "[FALL STATUS]: Phase 2 - High impact detected!"
      );

    } else if (currentTime - freeFallTime > IMPACT_WINDOW) {
      freeFallDetected = false;

      Serial.println(
        "[FALL STATUS]: Reset - no impact detected."
      );
    }
  }

  // Phase 3: Check for movement after impact
  if (impactDetected) {
    if (totalGyro > STILLNESS_THRESHOLD) {
      freeFallDetected = false;
      impactDetected = false;

      Serial.println(
        "[FALL STATUS]: Canceled - movement detected."
      );

    } else if (currentTime - impactTime >= STILLNESS_TIME) {

      if (!alertSent ||
          (currentTime - lastAlertTime > ALERT_COOLDOWN)) {

        Serial.println(
          "!!! EMERGENCY: SEVERE FALL DETECTED !!!"
        );

        Blynk.logEvent(
          "fall_detected",
          "A severe fall was detected. Please check the patient."
        );

        Blynk.virtualWrite(V4, "FALL DETECTED");

        alertSent = true;
        lastAlertTime = currentTime;
      }

      freeFallDetected = false;
      impactDetected = false;
    }
  }

  // 5. OLED DISPLAY UPDATE
  if (currentTime - lastDisplayTime > 50) {
    if (irValue < 20000) {
      drawClockScreen();
    } else {
      drawHeartScreen();
    }

    lastDisplayTime = currentTime;
  }

  // 6. CLOCK UPDATE
  if (currentTime - lastClockTick >= 1000) {
    lastClockTick = currentTime;

    seconds++;

    if (seconds > 59) {
      seconds = 0;
      minutes++;
    }

    if (minutes > 59) {
      minutes = 0;
      hours++;
    }

    if (hours > 23) {
      hours = 0;
    }

    // Send sensor data to Blynk
    Blynk.virtualWrite(V0, totalAcc);
    Blynk.virtualWrite(V1, totalGyro);
  }
}

// ============================================================================
// DISPLAY SCREEN A: WATCH FACE
// ============================================================================
void drawClockScreen() {
  u8g2.clearBuffer();

  if (animState) {
    u8g2.drawXBMP(5, 0, 16, 16, runner_A);
  } else {
    u8g2.drawXBMP(5, 0, 16, 16, runner_B);
  }

  u8g2.setFont(u8g2_font_ncenB08_tr);

  char stepStr[20];
  sprintf(stepStr, "%d Steps", stepCount);
  u8g2.drawStr(70, 12, stepStr);

  u8g2.drawLine(0, 18, 128, 18);

  int displayHour = hours;
  const char* ampm = "AM";

  if (hours >= 12) {
    ampm = "PM";

    if (hours > 12) {
      displayHour -= 12;
    }
  }

  if (displayHour == 0) {
    displayHour = 12;
  }

  char timeStr[10];
  sprintf(timeStr, "%02d:%02d", displayHour, minutes);

  u8g2.setFont(u8g2_font_helvB18_tf);
  u8g2.drawStr(25, 45, timeStr);

  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(105, 62, ampm);

  char bpmStr[15];
  sprintf(bpmStr, "%d BPM", beatAvg);
  u8g2.drawStr(0, 62, bpmStr);

  u8g2.sendBuffer();
}

// ============================================================================
// DISPLAY SCREEN B: HEART RATE WAVEFORM
// ============================================================================
void drawHeartScreen() {
  u8g2.clearBuffer();

  for (int i = 0; i < 127; i++) {
    waveBuffer[i] = waveBuffer[i + 1];
  }

  int base = 45;
  int newY = base;

  if (wavePhase > 0) {
    switch (wavePhase) {
      case 1: newY = base - 5; break;
      case 2: newY = base + 5; break;
      case 3: newY = base - 20; break;
      case 4: newY = base + 15; break;
      case 5: newY = base - 5; break;
      case 6: newY = base; break;
    }

    wavePhase++;

    if (wavePhase > 6) {
      wavePhase = 0;
    }

  } else {
    newY = base + random(-2, 2);
  }

  waveBuffer[127] = newY;

  for (int i = 0; i < 127; i++) {
    u8g2.drawLine(
      i, waveBuffer[i],
      i + 1, waveBuffer[i + 1]
    );
  }

  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 12, "Heart Rate");

  char bpmStr[10];

  if (beatAvg > 0) {
    sprintf(bpmStr, "%d BPM", beatAvg);
  } else {
    sprintf(bpmStr, "---");
  }

  int w = u8g2.getStrWidth(bpmStr);
  u8g2.drawStr(128 - w, 12, bpmStr);

  u8g2.sendBuffer();
}





