/*
   Original work: A BLE turbo trainer controller for Zwift

   Copyright (c) 2020 Peter Everett
   v1.0 Aug 2020 - Initial version

   Substantial modifications and new code:
   Copyright (c) 2024–2026 Adam Watson

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 3.
*/

enum ControlMode : uint8_t {
  MODE_IDLE = 0,
  MODE_SIM  = 1,   // grade -> steps
  MODE_ERG  = 2    // (speed, targetW) -> steps
};

enum LedPattern : uint8_t;  // forward declare the enum type

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>

static const char* FW_VERSION = "2026-01-14_02";  // change each build

volatile ControlMode gMode = MODE_IDLE;

// -------------------- SoftAP config --------------------
static const char* AP_SSID = "InsideRideCal";
static const char* AP_PASS = "insideride"; // >= 8 chars

// -------------------- HTTP server --------------------
static WebServer server(80);

// -------------------- OTA lock window --------------------
static constexpr uint32_t OTA_UNLOCK_WINDOW_MS = 60 * 1000; // 60 seconds
static uint32_t otaUnlockedUntilMs = 0;

// Optional Basic Auth for OTA
static const char* OTA_USER = "admin";
static const char* OTA_PASS = "change-me";

// Manual override latch
static volatile bool   gManualHoldActive = false;
static volatile int32_t gManualHoldTarget = 0;   // logical units 0..1000

static void handleRoot();
static void handleDiagJson();
static void handleGoto();
static void handleEnable();
static void handleGotoHold();
static void handleResumeZwift();
static void handleUpdateForm();
static void handleUpdateUpload();

static volatile bool gOtaInProgress = false;
static uint32_t gOtaLastProgressMs = 0;
static size_t gOtaBytes = 0;
static volatile bool gOtaDone = false;
static volatile bool gOtaOk = false;
static String gOtaErr;

// OTA runtime state
static uint32_t gOtaExpectedSize = 0;
static uint32_t gOtaWritten = 0;
static uint8_t  gOtaLastPct = 255;

// Policy knobs
static constexpr bool OTA_DENY_WHEN_BLE_CONNECTED = true;  // your request

static constexpr bool  OTA_REQUIRE_LOW_SPEED = true;
static constexpr float OTA_MAX_SPEED_MPH     = 1.0f;   // must be <= this to OTA


// ===================== Grid size =====================
constexpr int mph_N = 6;
constexpr int pos_N = 5;


// ===================== Pins =====================
static const int STEP_PIN    = D3;
static const int DIR_PIN     = D4;
static const int ENABLE_PIN  = D5;


static const int LIMIT_PIN   = D6; // INPUT_PULLUP, switch -> GND
static const int HALL_PIN    = D7; // INPUT_PULLUP, interrupt
static const int CAL_BTN_PIN = D8; // INPUT_PULLUP, momentary -> GND

static const int LED_PIN = D9;


// ===================== Millis =====================
unsigned long whileMillis;
unsigned long millisValue = 0;


// ===================== LED =====================
static inline bool otaIsUnlocked();  // defined later (keep static inline)

// These are defined later in the same .ino, so forward-declare them with matching linkage:
extern volatile bool gIsHoming;         // <-- only keep extern if the definitions are NOT static
extern volatile bool gRehomeRequested;  // <-- same
extern bool deviceConnected;            // <-- same

static const bool LED_ACTIVE_HIGH = true;  // set false if LED is wired to 3.3V and sinks to GND

static inline void ledWrite(bool on) {
  digitalWrite(LED_PIN, (on ^ !LED_ACTIVE_HIGH) ? HIGH : LOW);
}

enum LedPattern : uint8_t {
  LED_OFF = 0,
  LED_SOLID,
  LED_BLINK_SLOW,
  LED_BLINK_MED,
  LED_BLINK_FAST,
  LED_HEARTBEAT,
  LED_DOUBLE_BLIP,
  LED_TRIPLE_BLIP
};

static LedPattern gLedPattern = LED_OFF;

// internal timing state
static bool     gLedOn = false;
static uint32_t gLedNextMs = 0;
static uint8_t  gLedPhase = 0;

static void ledSetPattern(LedPattern p) {
  if (p == gLedPattern) return;
  gLedPattern = p;
  gLedPhase = 0;
  gLedOn = false;
  gLedNextMs = 0;
  ledWrite(false);
}

static void ledUpdate() {
  const uint32_t now = millis();
  if (gLedNextMs != 0 && (int32_t)(now - gLedNextMs) < 0) return;

  switch (gLedPattern) {
    case LED_OFF:
      ledWrite(false);
      gLedNextMs = now + 500;
      break;

    case LED_SOLID:
      ledWrite(true);
      gLedNextMs = now + 500;
      break;

    case LED_BLINK_SLOW: { // 1 Hz (500/500)
      gLedOn = !gLedOn;
      ledWrite(gLedOn);
      gLedNextMs = now + 500;
    } break;

    case LED_BLINK_MED: { // 2 Hz (250/250)
      gLedOn = !gLedOn;
      ledWrite(gLedOn);
      gLedNextMs = now + 250;
    } break;

    case LED_BLINK_FAST: { // 10 Hz (50/50) - still visible
      gLedOn = !gLedOn;
      ledWrite(gLedOn);
      gLedNextMs = now + 50;
    } break;

    case LED_HEARTBEAT: {
      // short ON (80ms) then long OFF (920ms)
      if (gLedPhase == 0) { ledWrite(true);  gLedNextMs = now + 80;  gLedPhase = 1; }
      else                { ledWrite(false); gLedNextMs = now + 920; gLedPhase = 0; }
    } break;

    case LED_DOUBLE_BLIP: {
      // ON 80, OFF 120, ON 80, OFF 1700
      switch (gLedPhase) {
        case 0: ledWrite(true);  gLedNextMs = now + 80;   gLedPhase = 1; break;
        case 1: ledWrite(false); gLedNextMs = now + 120;  gLedPhase = 2; break;
        case 2: ledWrite(true);  gLedNextMs = now + 80;   gLedPhase = 3; break;
        default:
          ledWrite(false); gLedNextMs = now + 1700; gLedPhase = 0; break;
      }
    } break;

    case LED_TRIPLE_BLIP: {
      // ON 80, OFF 120, ON 80, OFF 120, ON 80, OFF 1500
      switch (gLedPhase) {
        case 0: ledWrite(true);  gLedNextMs = now + 80;   gLedPhase = 1; break;
        case 1: ledWrite(false); gLedNextMs = now + 120;  gLedPhase = 2; break;
        case 2: ledWrite(true);  gLedNextMs = now + 80;   gLedPhase = 3; break;
        case 3: ledWrite(false); gLedNextMs = now + 120;  gLedPhase = 4; break;
        case 4: ledWrite(true);  gLedNextMs = now + 80;   gLedPhase = 5; break;
        default:
          ledWrite(false); gLedNextMs = now + 1500; gLedPhase = 0; break;
      }
    } break;
  }
}

static void ledSelectPattern() {
  // Highest priority first
  if (gOtaInProgress) {
    ledSetPattern(LED_BLINK_FAST);
    return;
  }

  if (otaIsUnlocked()) {
    ledSetPattern(LED_DOUBLE_BLIP);
    return;
  }

  if (gRehomeRequested) {             // or any other "fault" latch you want
    ledSetPattern(LED_TRIPLE_BLIP);
    return;
  }

  if (gIsHoming) {
    ledSetPattern(LED_BLINK_MED);
    return;
  }

  if (deviceConnected) {
    switch (gMode) {
      case MODE_ERG:  ledSetPattern(LED_SOLID);      return;
      case MODE_SIM:  ledSetPattern(LED_BLINK_SLOW); return;
      default:        ledSetPattern(LED_HEARTBEAT);  return;
    }
  }

  // Not connected: show something simpler
  switch (gMode) {
    case MODE_ERG:  ledSetPattern(LED_SOLID);      break;   // optional
    case MODE_SIM:  ledSetPattern(LED_BLINK_SLOW); break;   // optional
    default:        ledSetPattern(LED_OFF);        break;
  }
}

// ===================== Simulation bounds and variables =====================
const int upperInclineClamp = 10;
const int lowerInclineClamp = -5;  //changed from -10


float gradeFloat = 0.0;
int_least16_t grade = 0;


int roundedGrade = 0;
int currentBikeGrade = 0;


// ===================== Steps =====================
// Logical travel (engineering units)
static const int32_t LOGICAL_MIN = 0;
static const int32_t LOGICAL_MAX = 1000;
static volatile int32_t logStepPos = 0;
static volatile int32_t logStepTarget = 0;


// Physical travel (microsteps)
static const int32_t PHYS_MIN_STEPS = 0;
static const int32_t PHYS_MAX_STEPS = 6960;   // 870 * 8
static volatile int32_t physStepPos = 0;
static volatile int32_t physStepTarget = 0;


//Map logical->physical
static inline int32_t logicalToSteps(int32_t logical) {
  logical = constrain(logical, LOGICAL_MIN, LOGICAL_MAX);
  return (int32_t)((int64_t)logical * PHYS_MAX_STEPS / LOGICAL_MAX);
}


static inline int32_t stepsToLogical(int32_t steps) {
  steps = constrain(steps, PHYS_MIN_STEPS, PHYS_MAX_STEPS);
  return (int32_t)((int64_t)steps * LOGICAL_MAX / PHYS_MAX_STEPS);
}


// ===================== Stepper motion =====================
static float    stepSpeedSps   = 2500.0f;
static uint32_t stepIntervalUs = 5000;
static uint32_t lastStepUs     = 0;

// -------- Stepper disable below speed --------
static constexpr float SPEED_DISABLE_MPH = 2.0f;   // disable if below this
static constexpr float SPEED_ENABLE_MPH  = 2.3f;   // re-enable above this (hysteresis)
static constexpr uint32_t SPEED_HOLDOFF_MS = 800;  // must stay below threshold this long

// ===================== Soft ramp tuning =====================
static float    runSpeedSps = 2500.0f;   // you already have this
static float    jogSpeedSps = 800.0f;    // you already have this

// Soft-start after direction change
static float    rampStartSps    = 900.0f;   // first speed immediately after dir flip
static float    rampAccelSps2   = 6000.0f;  // acceleration in steps/s^2 (microsteps)
static uint32_t rampLastUs      = 0;
static float    rampCurSps      = 900.0f;

// Optional: slow down when close to target to reduce overshoot/missed steps
static int32_t  slowZoneSteps   = 200;      // microsteps from target where we slow down
static float    slowZoneSps     = 1000.0f;

// Track direction changes
static int8_t   gLastDir = 0;              // -1, 0, +1
static bool     gDirJustChanged = false;

// Compute interval from sps
static inline uint32_t spsToIntervalUs(float sps) {
  if (sps < 50.0f) sps = 50.0f;
  if (sps > 5000.0f) sps = 5000.0f;
  return (uint32_t)(1000000.0f / sps + 0.5f);
}

static uint32_t gBelowSpeedSinceMs = 0;
static bool gStepperAutoDisabled = false;

static void stepperSetJogSpeed() {
  stepperSetSpeed(jogSpeedSps);
}


static void stepperSetRunSpeed() {
  stepperSetSpeed(runSpeedSps);
}


static inline void stepperPulseOnce() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(stepIntervalUs);   // set by stepperSetSpeed()
}


static const bool STEPPER_DIR_INVERT = false;   // set true to reverse motor direction
static volatile bool    gStepEn     = false;


static inline void stepperSetDir(bool forward) {
  bool pinLevel = forward;
  if (STEPPER_DIR_INVERT) pinLevel = !pinLevel;
  digitalWrite(DIR_PIN, pinLevel ? HIGH : LOW);


  static uint32_t lastPrintMs = 0;
  uint32_t now = millis();
  if (now - lastPrintMs > 200) {
    lastPrintMs = now;
    //Serial.printf("[DIR] forward=%d invert=%d -> DIR=%d (pin %d)\n",
    //              (int)forward, (int)STEPPER_DIR_INVERT, (int)pinLevel, DIR_PIN);
  }
}

static void updateStepperEnableFromSpeed(float mph) {
  const uint32_t now = millis();

  // If currently enabled, start a timer when we go below threshold
  if (!gStepperAutoDisabled) {
    if (mph < SPEED_DISABLE_MPH) {
      if (gBelowSpeedSinceMs == 0) gBelowSpeedSinceMs = now;
      if (now - gBelowSpeedSinceMs >= SPEED_HOLDOFF_MS) {
        stepperEnable(false);
        gStepperAutoDisabled = true;
      }
    } else {
      gBelowSpeedSinceMs = 0; // reset timer if speed comes back up
    }
  }
  // If currently disabled, re-enable only when above a higher threshold
  else {
    if (mph > SPEED_ENABLE_MPH) {
      stepperEnable(true);
      gStepperAutoDisabled = false;
      gBelowSpeedSinceMs = 0;
    }
  }
}

//Guarded SpeedFromPos stuff
static constexpr float MAX_POWER_W = 2000.0f;


// “Online monotonic” deltas (tune)
static constexpr float MONO_DMPH = 0.5f;   // mph
static constexpr float MONO_DPOS = 5.0f;   // logical units (0–1000)


// Optional: cap power rise rate in extrapolated zones (tune)
static constexpr float MAX_DPDMPH = 120.0f; // W per 1 mph


static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}


// ===================== Limit debounce =====================
static const uint32_t LIMIT_DEBOUNCE_MS = 8;
static uint8_t  gLimitStable = 1; // 1=not pressed (pullup), 0=pressed
static uint8_t  gLimitRawLast = 1;
static uint32_t gLimitLastChangeMs = 0;


static inline void updateLimitDebounce() {
  uint8_t raw = (uint8_t)digitalRead(LIMIT_PIN);
  uint32_t now = millis();
  if (raw != gLimitRawLast) {
    gLimitRawLast = raw;
    gLimitLastChangeMs = now;
  } else {
    if ((now - gLimitLastChangeMs) >= LIMIT_DEBOUNCE_MS) {
      gLimitStable = raw;
    }
  }
}
static inline bool limitPressed() { return gLimitStable == 0; }

static bool calPressedEdge() {
  static uint8_t lastStable = HIGH;
  static uint8_t lastRaw = HIGH;
  static uint32_t lastChange = 0;
  static bool latched = false;

  uint8_t raw = digitalRead(CAL_BTN_PIN);
  uint32_t now = millis();

  if (raw != lastRaw) { lastRaw = raw; lastChange = now; }
  if ((now - lastChange) > 30) lastStable = lastRaw; // 30ms debounce

  if (lastStable == LOW && !latched) { latched = true; return true; }
  if (lastStable == HIGH) latched = false;
  return false;
}

// ===================== WebServer Go-To =====================
static inline int32_t clampLogical(int32_t v) {
  if (v < LOGICAL_MIN) return LOGICAL_MIN;
  if (v > LOGICAL_MAX) return LOGICAL_MAX;
  return v;
}

static void startApAndWeb() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[WiFi] SoftAP start %s\n", ok ? "OK" : "FAILED");
  Serial.print("[WiFi] AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/diag.json", HTTP_GET, handleDiagJson);
  server.on("/goto", HTTP_GET, handleGoto);
  server.on("/enable", HTTP_GET, handleEnable);

  server.on("/goto_hold", HTTP_GET, handleGotoHold);
  server.on("/resume_zwift", HTTP_POST, handleResumeZwift);

  // OTA endpoints
    // OTA endpoints (CORRECT PATTERN)
  server.on("/update", HTTP_GET, handleUpdateForm);
  server.on("/update", HTTP_POST, handleUpdatePostFinalizer, handleUpdateUpload);

  server.begin();
  Serial.println("[HTTP] Server started");
}

// ===================== Hall / RPM =====================
// Assumes 1 pulse per revolution
static volatile uint32_t gLastHallUs     = 0;
static volatile uint32_t gHallIntervalUs = 0;
static volatile uint32_t gHallEdges      = 0;


static const uint8_t HALL_PULSES_PER_REV = 6;     // you said 6 magnets per revolution
static const uint32_t HALL_MIN_DT_US     = 1500;  // reject pulses faster than this (tune as needed)


static volatile uint32_t gHallIsrCount = 0;


// --- RPM smoothing / deglitch ---
static float gRpmFiltered = 0.0f;


// Ignore additional edges for a short time after accepting a valid edge.
// This helps when you "wiggle" near the magnet and it chatters.
static const uint32_t HALL_HOLDOFF_US = 3000;  // 3ms; tune 2000-6000


static volatile uint32_t gLastAcceptedHallUs = 0;


// For smoothing time constant (bigger = smoother, slower response)
static const float RPM_FILTER_TAU_S = 0.60f;   // ~0.6s time constant
static uint32_t gLastRpmFilterMs = 0;


static uint32_t ftmsState = 0;


void IRAM_ATTR hallISR() {
  uint32_t now = micros();


  // Holdoff to suppress chatter when near a magnet
  uint32_t sinceAccepted = now - gLastAcceptedHallUs;
  if (gLastAcceptedHallUs != 0 && sinceAccepted < HALL_HOLDOFF_US) {
    return;
  }


  uint32_t dt = now - gLastHallUs;


  // Ignore very short intervals (bounce/noise)
  if (gLastHallUs != 0 && dt < HALL_MIN_DT_US) {
    return;
  }


  gLastHallUs = now;
  gHallIntervalUs = dt;
  gHallEdges++;


  gLastAcceptedHallUs = now;
}




static float readRPM() {
  noInterrupts();
  uint32_t dt   = gHallIntervalUs;
  uint32_t last = gLastHallUs;
  interrupts();


  if (last == 0) return 0.0f;
  if ((micros() - last) > 1000000UL) return 0.0f; // stopped >1s
  if (dt == 0) return 0.0f;


  // pulses per second
  float pps = 1e6f / (float)dt;


  // revolutions per second
  float rps = pps / (float)HALL_PULSES_PER_REV;


  return rps * 60.0f;
}


// ===================== Speed conversion (RPM -> MPH) =====================
static float currentRPM = 0.0f;


static constexpr float ROLLER_DIAMETER_IN = 3.25f;
static constexpr float INCHES_PER_MILE    = 63360.0f;
static constexpr float MINUTES_PER_HOUR   = 60.0f;


static constexpr float RPM_TO_MPH =
  (ROLLER_DIAMETER_IN * PI * MINUTES_PER_HOUR) / INCHES_PER_MILE;


static inline float rpmToMph(float rpm) {
  if (rpm <= 0.0f) return 0.0f;
  return rpm * RPM_TO_MPH;
}


static inline float mphToRpm(float mph) {
  if (mph <= 0.0f) return 0.0f;
  return mph / RPM_TO_MPH;
}


// Application-level speed signal
static float currentSpeedMph = 0.0f;


// ===================== Power from Speed/Pos Calibration data (5x5) =====================
double X [] = { 0, 5, 10, 15, 20, 25, 50 };
double Y [] = { 0, 250, 500, 750, 1000 };


const int Xcount = sizeof(X) / sizeof(X[0]); //X=mph
const int Ycount = sizeof(Y) / sizeof(Y[0]); //Y=pos


double Z[Xcount][Ycount] = {
  {   0,   0,   0,   0,     0 },
  {  52,  68,  80, 102,   124 },
  { 117, 143, 217, 280,   343 },
  { 188, 246, 383, 490,   597 },
  { 265, 380, 580, 732,   884 },
  { 349, 544, 806, 1006, 1206 },
  { 861, 1806, 2388, 2856, 3324 }
};


// ==================== Power from Speed/Pos Bilinear Interpolation =========================
double powerFromSpeedPos(double x, double y) {
  int xIndex, yIndex;


  if ((x < X[0]) || (x > X[Xcount - 1])) {
    Serial.println(F("x not in range"));
    return 0;  // arbitrary...
  }


  if ((y < Y[0]) || (y > Y[Ycount - 1])) {
    Serial.println(F("y not in range"));
    return 0;  // arbitrary...
  }


  for (int i = Xcount - 2; i >= 0; --i)
    if (x >= X[i]) {
      xIndex = i;
      break;
    }


  for (int i = Ycount - 2; i >= 0; --i)
    if (y >= Y[i]) {
      yIndex = i;
      break;
    }
/*
  Serial.print(F("X:"));
  Serial.print(x);
  Serial.print(F(" in ["));
  Serial.print(X[xIndex]);
  Serial.print(F(","));
  Serial.print(X[xIndex + 1]);
  Serial.print(F("] and Y:"));
  Serial.print(y);
  Serial.print(F(" in ["));
  Serial.print(Y[yIndex]);
  Serial.print(F(","));
  Serial.print(Y[yIndex + 1]);
  Serial.println(F("]"));
*/
  // https://en.wikipedia.org/wiki/Bilinear_interpolation
  // Q11 = (x1, y1), Q12 = (x1, y2), Q21 = (x2, y1), and Q22 = (x2, y2)
  double x1, y1, x2, y2;
  double fQ11, fQ12, fQ21, fQ22;
  double fxy1, fxy2, fxy;


  x1 = X[xIndex];
  x2 = X[xIndex + 1];
  y1 = Y[yIndex];
  y2 = Y[yIndex + 1];


  fQ11 = Z[xIndex][yIndex];
  fQ12 = Z[xIndex][yIndex + 1];
  fQ21 = Z[xIndex + 1][yIndex];
  fQ22 = Z[xIndex + 1][yIndex + 1];


  fxy1 = ((x2 - x) / (x2 - x1)) * fQ11 + ((x - x1) / (x2 - x1)) * fQ21;
  fxy2 = ((x2 - x) / (x2 - x1)) * fQ12 + ((x - x1) / (x2 - x1)) * fQ22;


  fxy = ((y2 - y) / (y2 - y1)) * fxy1 + ((y - y1) / (y2 - y1)) * fxy2;


  return fxy;
}

//Random variables
int currentEstPower = 0;

// ===================== Steps from Speed/Power Calibration data (5x5) =====================

double Xw [] = { 0, 5, 10, 15, 20, 25, 50 };
double Yw [] = { 0, 100, 150, 200, 250, 300, 400, 600, 1000 };


const int Xcountw = sizeof(Xw) / sizeof(Xw[0]); //X=mph
const int Ycountw = sizeof(Yw) / sizeof(Yw[0]); //Y=pos


double Zw[Xcountw][Ycountw] = {
  {   0,    0,    0,    0,    0,    0,    0,    0,    0 },
  {   0,  739, 1000, 1000, 1000, 1000, 1000, 1000, 1000 },
  {   0,    0,  212,  442,  651,  841, 1000, 1000, 1000 },
  {   0,    0,    0,   70,  198,  322,  560,  996, 1000 },
  {   0,    0,    0,    0,    0,   79,  238,  552, 1000 },
  {   0,    0,    0,    0,    0,    0,   67,  285,  745 },
  {   0,    0,    0,    0,    0,    0,    0,    0,   26 }
};


// ==================== Steps from Speed/Power Bilinear Interpolation =========================
double stepFromPowerSpeed(double xw, double yw) {
  int xIndexw, yIndexw;


  if ((xw < Xw[0]) || (xw > Xw[Xcountw - 1])) {
    Serial.println(F("xw not in range"));
    return 0;  // arbitrary...
  }


  if ((yw < Yw[0]) || (yw > Yw[Ycountw - 1])) {
    Serial.println(F("yw not in range"));
    return 0;  // arbitrary...
  }


  for (int iw = Xcountw - 2; iw >= 0; --iw)
    if (xw >= Xw[iw]) {
      xIndexw = iw;
      break;
    }


  for (int iw = Ycountw - 2; iw >= 0; --iw)
    if (yw >= Yw[iw]) {
      yIndexw = iw;
      break;
    }
/*
  Serial.print(F("X:"));
  Serial.print(x);
  Serial.print(F(" in ["));
  Serial.print(X[xIndex]);
  Serial.print(F(","));
  Serial.print(X[xIndex + 1]);
  Serial.print(F("] and Y:"));
  Serial.print(y);
  Serial.print(F(" in ["));
  Serial.print(Y[yIndex]);
  Serial.print(F(","));
  Serial.print(Y[yIndex + 1]);
  Serial.println(F("]"));
*/
  // https://en.wikipedia.org/wiki/Bilinear_interpolation
  // Q11 = (x1, y1), Q12 = (x1, y2), Q21 = (x2, y1), and Q22 = (x2, y2)
  double x1w, y1w, x2w, y2w;
  double fQ11w, fQ12w, fQ21w, fQ22w;
  double fxy1w, fxy2w, fxyw;


  x1w = Xw[xIndexw];
  x2w = Xw[xIndexw + 1];
  y1w = Yw[yIndexw];
  y2w = Yw[yIndexw + 1];


  fQ11w = Zw[xIndexw][yIndexw];
  fQ12w = Zw[xIndexw][yIndexw + 1];
  fQ21w = Zw[xIndexw + 1][yIndexw];
  fQ22w = Zw[xIndexw + 1][yIndexw + 1];


  fxy1w = ((x2w - xw) / (x2w - x1w)) * fQ11w + ((xw - x1w) / (x2w - x1w)) * fQ21w;
  fxy2w = ((x2w - xw) / (x2w - x1w)) * fQ12w + ((xw - x1w) / (x2w - x1w)) * fQ22w;


  fxyw = ((y2w - yw) / (y2w - y1w)) * fxy1w + ((yw - y1w) / (y2w - y1w)) * fxy2w;


  return fxyw;
}

// ===================== Stepper =====================
static void stepperEnable(bool en) {
  gStepEn = en;
  digitalWrite(ENABLE_PIN, en ? LOW : HIGH);

  if (en) {
    // reset ramp timing so the first steps after enable are gentle
    gLastDir   = 0;
    rampCurSps = min(rampStartSps, runSpeedSps);
    rampLastUs = micros();
    lastStepUs = 0;
  }

  Serial.printf("[STEP] enable=%s\n", en ? "ON" : "OFF");
}



static void stepperSetSpeed(float sps) {
  if (sps < 50) sps = 50;
  if (sps > 4000) sps = 4000;

  runSpeedSps = sps;      // <-- this is the important line for soft ramp
  stepSpeedSps = sps;     // for logging/visibility only
  stepIntervalUs = spsToIntervalUs(sps);

  Serial.printf("[STEP] target=%.1f sps interval=%lu us\n", runSpeedSps, (unsigned long)stepIntervalUs);
}



// ===================== Homing =====================
volatile bool gIsHoming = false;
volatile bool gRehomeRequested = false;

static uint32_t gLastLimitTripMs = 0;
static const uint32_t REHOME_COOLDOWN_MS = 2000;   // don't rehome repeatedly if switch chatters

static bool homeStepper() {
  Serial.println("[HOME] Starting homing...");
  stepperEnable(true);
  stepperSetJogSpeed();   // use your slower manual speed


  // settle debounce
  for (int i = 0; i < 30; i++) { updateLimitDebounce(); delay(2); }


  // If switch already pressed, back off (positive direction) until released
  if (limitPressed()) {
    Serial.println("[HOME] Switch active at start; backing off (+)...");
    stepperSetDir(true); // forward=true => positive (away from switch per your convention)
    uint32_t t0 = millis();
    while (limitPressed() && (millis() - t0) < 2000) {
      stepperPulseOnce();
      updateLimitDebounce();
      delay(1);
    }
  }


  // Seek switch by stepping negative (toward switch) until pressed
  Serial.println("[HOME] Seeking switch (-)...");
  stepperSetDir(false); // forward=false => negative (toward switch)
  uint32_t t0 = millis();
  const uint32_t timeoutMs = 10000;


  while (!limitPressed() && (millis() - t0) < timeoutMs) {
    stepperPulseOnce();
    updateLimitDebounce();
    delay(1);
  }


  if (!limitPressed()) {
    Serial.println("[HOME] WARNING: did not reach switch (timeout).");
    return false;
  }


  // Back off a little so you're not resting on the switch
  Serial.println("[HOME] Switch reached; backing off (+)...");
  stepperSetDir(true);
  for (int i = 0; i < 300; i++) stepperPulseOnce();


  // Now define your coordinate system
  logStepPos = 0;
  logStepTarget = 0;
  physStepPos = 0;
  physStepTarget = 0;


  Serial.println("[HOME] Done. pos=0");

  // after homing done:
  stepperSetRunSpeed();   // restores runSpeedSps to your normal target

  return true;
}

static void requestRehome(const char* why) {
  uint32_t now = millis();
  if (gIsHoming) return;
  if (now - gLastLimitTripMs < REHOME_COOLDOWN_MS) return;

  gLastLimitTripMs = now;
  gRehomeRequested = true;

  // Freeze motion immediately (prevents driving into the switch)
  logStepTarget = logStepPos;

  Serial.printf("[SAFETY] Limit hit -> rehome requested (%s)\n", why);
}

/*
// ===================== Grade -> target steps (simple linear mapping) =====================
static int32_t gradeToSteps(float gradePercent) {
  const float MIN_GRADE = -2.0f;   // lowest resistance
  const float MAX_GRADE = 20.0f;   // highest resistance


  // Clamp grade first
  if (gradePercent < MIN_GRADE) gradePercent = MIN_GRADE;
  if (gradePercent > MAX_GRADE) gradePercent = MAX_GRADE;


  // Normalize to 0..1
  float t = (gradePercent - MIN_GRADE) / (MAX_GRADE - MIN_GRADE);


  // Map to step range
  return (int32_t)lroundf(
    LOGICAL_MIN + t * (LOGICAL_MAX - LOGICAL_MIN)
  );
}
*/

// ===================== Stepper from Speed/Grade Calibration data (5x5) =====================
double Xstep [] = { 0, 5, 10, 15, 20, 25, 30, 50 };
double Ystep [] = { -4, 0, 2, 4, 6, 8, 10 };

const int Xcountstep = sizeof(Xstep) / sizeof(Xstep[0]); //X=mph
const int Ycountstep = sizeof(Ystep) / sizeof(Ystep[0]); //Y=grade



double Zstep[Xcountstep][Ycountstep] = {
  {   0,  167,  333,  500,  667,  833, 1000 },
  {   0,  167,  333,  500,  667,  833, 1000 },
  {   0,  167,  333,  500,  667,  833, 1000 },
  {   0,  167,  333,  500,  667,  833, 1000 },
  {   0,  167,  333,  500,  667,  833, 1000 },
  { 167,  333,  500,  677,  834, 1000, 1000 },
  { 333,  500,  677,  834, 1000, 1000, 1000 },
  { 500,  500,  677,  834, 1000, 1000, 1000 }
};


// ==================== Stepper from Speed/Grade Bilinear Interpolation =========================
double gradeToSteps(double xstep, double ystep) {
  int xIndexstep, yIndexstep;


  if ((xstep < Xstep[0]) || (xstep > Xstep[Xcountstep - 1])) {
    Serial.println(F("xstep not in range"));
    return 0;  // arbitrary...
  }


  if ((ystep < Ystep[0]) || (ystep > Ystep[Ycountstep - 1])) {
    Serial.println(F("ystep not in range"));
    return 0;  // arbitrary...
  }


  for (int istep = Xcountstep - 2; istep >= 0; --istep)
    if (xstep >= Xstep[istep]) {
      xIndexstep = istep;
      break;
    }


  for (int istep = Ycountstep - 2; istep >= 0; --istep)
    if (ystep >= Ystep[istep]) {
      yIndexstep = istep;
      break;
    }
/*
  Serial.print(F("X:"));
  Serial.print(x);
  Serial.print(F(" in ["));
  Serial.print(X[xIndex]);
  Serial.print(F(","));
  Serial.print(X[xIndex + 1]);
  Serial.print(F("] and Y:"));
  Serial.print(y);
  Serial.print(F(" in ["));
  Serial.print(Y[yIndex]);
  Serial.print(F(","));
  Serial.print(Y[yIndex + 1]);
  Serial.println(F("]"));
*/
  // https://en.wikipedia.org/wiki/Bilinear_interpolation
  // Q11 = (x1, y1), Q12 = (x1, y2), Q21 = (x2, y1), and Q22 = (x2, y2)
  double x1step, y1step, x2step, y2step;
  double fQ11step, fQ12step, fQ21step, fQ22step;
  double fxy1step, fxy2step, fxystep;


  x1step = Xstep[xIndexstep];
  x2step = Xstep[xIndexstep + 1];
  y1step = Ystep[yIndexstep];
  y2step = Ystep[yIndexstep + 1];


  fQ11step = Zstep[xIndexstep][yIndexstep];
  fQ12step = Zstep[xIndexstep][yIndexstep + 1];
  fQ21step = Zstep[xIndexstep + 1][yIndexstep];
  fQ22step = Zstep[xIndexstep + 1][yIndexstep + 1];


  fxy1step = ((x2step - xstep) / (x2step - x1step)) * fQ11step + ((xstep - x1step) / (x2step - x1step)) * fQ21step;
  fxy2step = ((x2step - xstep) / (x2step - x1step)) * fQ12step + ((xstep - x1step) / (x2step - x1step)) * fQ22step;


  fxystep = ((y2step - ystep) / (y2step - y1step)) * fxy1step + ((ystep - y1step) / (y2step - y1step)) * fxy2step;


  return fxystep;
}

// ===================== Execute stepper =====================
static void stepperGoto(int32_t pos) {
  if (pos < LOGICAL_MIN) pos = LOGICAL_MIN;
  if (pos > LOGICAL_MAX) pos = LOGICAL_MAX;
  logStepTarget = pos;
  //Serial.printf("[STEP] goto=%ld\n", (long)logStepTarget);
}


static void stepperUpdate() {
  if (!gStepEn) return;

  physStepTarget = logicalToSteps(logStepTarget);

  if (physStepPos == physStepTarget) return;

  // Safety clamp
  if (physStepPos < PHYS_MIN_STEPS) physStepPos = PHYS_MIN_STEPS;
  if (physStepPos > PHYS_MAX_STEPS) physStepPos = PHYS_MAX_STEPS;

  // Determine direction to move
  int32_t delta = physStepTarget - physStepPos;
  int8_t dir = (delta > 0) ? +1 : -1;

  // Detect direction change
  if (dir != gLastDir && gLastDir != 0) {
    gDirJustChanged = true;
    rampCurSps = min(rampStartSps, runSpeedSps);
    rampLastUs = micros();
  }
  gLastDir = dir;

  // --- Choose current speed based on ramp + proximity slow zone ---
  float targetSps = runSpeedSps;

  // Slow zone near target (optional but recommended)
  int32_t absDelta = (delta >= 0) ? delta : -delta;
  if (absDelta <= slowZoneSteps) {
    if (targetSps > slowZoneSps) targetSps = slowZoneSps;
  }

  // Apply soft ramp (accelerate from rampCurSps -> targetSps)
  uint32_t nowUs = micros();
  if (rampLastUs == 0) rampLastUs = nowUs;
  float dt = (nowUs - rampLastUs) * 1e-6f;
  rampLastUs = nowUs;

  if (rampCurSps < targetSps) {
    rampCurSps += rampAccelSps2 * dt;
    if (rampCurSps > targetSps) rampCurSps = targetSps;
  } else {
    // If target dropped (e.g., slow zone), clamp down immediately
    rampCurSps = targetSps;
  }

  stepIntervalUs = spsToIntervalUs(rampCurSps);

  // Time gate
  if ((nowUs - lastStepUs) < stepIntervalUs) return;
  lastStepUs = nowUs;

  // Set DIR pin
  stepperSetDir(dir > 0);

  // Step pulse
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, LOW);

  // Update positions
  physStepPos += (dir > 0) ? 1 : -1;
  logStepPos = stepsToLogical(physStepPos);
}

// ===================== BLE stuff =====================
BLEServer* pServer = NULL;
BLECharacteristic* pIndoorBike = NULL;
BLECharacteristic* pFeature = NULL;
BLECharacteristic* pControlPoint = NULL;
BLECharacteristic* pStatus = NULL;


BLEAdvertisementData advert;
BLEAdvertisementData scan_response;
BLEAdvertising* pAdvertising;


bool deviceConnected = false;
bool oldDeviceConnected = false;
int value = 0;  //This is the value sent as "nothing".  We need to send something for some of the charactistics or it won't work.

#define FTMSDEVICE_FTMS_UUID "00001826-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_INDOOR_BIKE_CHAR_UUID "00002AD2-0000-1000-8000-00805F9B34FB"
//#define FTMSDEVICE_RESISTANCE_RANGE_CHAR_UUID "00002AD6-0000-1000-8000-00805F9B34FB"
//#define FTMSDEVICE_POWER_RANGE_CHAR_UUID "00002AD8-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_FTMS_FEATURE_CHAR_UUID "00002ACC-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_FTMS_CONTROL_POINT_CHAR_UUID "00002AD9-0000-1000-8000-00805F9B34FB"
#define FTMSDEVICE_FTMS_STATUS_CHAR_UUID "00002ADA-0000-1000-8000-00805F9B34FB"


static uint32_t nextControlAllowedMs = 0;   // replaces whileMillis/while spin
static uint32_t lastFtmsNotifyMs     = 0;
static const uint32_t FTMS_NOTIFY_PERIOD_MS = 100; // 10 Hz



// already in your code (keep these, but ensure they are globals, not shadowed)
static volatile int16_t ergTargetW  = 0;
static volatile int32_t tgtLogical  = 0;  // what loop uses for stepperGoto()
static volatile uint32_t gLastSimRxMs = 0;
static volatile uint32_t gLastErgRxMs = 0;



static constexpr uint16_t IB_FLAG_INSTANT_CADENCE = (1 << 2);  // Instantaneous Cadence present
static constexpr uint16_t IB_FLAG_INSTANT_POWER   = (1 << 6);  // Instantaneous Power present

static constexpr uint16_t FTMSDEVICE_INDOOR_BIKE_CHARDef =
  (IB_FLAG_INSTANT_CADENCE | IB_FLAG_INSTANT_POWER);

uint16_t speedOut = 100;
int16_t powerOut = 100;

uint8_t FTMSDEVICE_INDOOR_BIKE_CHARData[10] = {
  (uint8_t)(FTMSDEVICE_INDOOR_BIKE_CHARDef & 0xff),
  (uint8_t)(FTMSDEVICE_INDOOR_BIKE_CHARDef >> 8),

  // Instantaneous Speed (uint16, 0.01 m/s) - optional; if you aren't using it, leave 0
  0x00, 0x00,

  // Instantaneous Cadence (uint16, 0.5 rpm)
  0x00, 0x00,

  // Instantaneous Power (int16, watts)
  0x00, 0x00,

  // Padding / unused (not required; leave out if you prefer exact sizing)
  0x00, 0x00
};


//response/acknowledgement to send to the client after writing to control point
uint8_t replyDs[3] = { 0x80, 0x00, 0x01 };  // set up replyDS array with 3 byte values for control point resp to zwift


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device Connected");
  };


  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

static const char* modeName(ControlMode m) {
  switch (m) {
    case MODE_SIM:  return "SIM";
    case MODE_ERG:  return "ERG";
    default:        return "IDLE";
  }
}

static void printDiag() {
Serial.printf(
  "Grade=%.2f%% | Step Position=%ld Steps | Speed=%.2f MPH | Est Power=%d W | ERG Target =%d W | FTMS State=%s\n" ,
  gradeFloat,
  (long)logStepPos,
  currentSpeedMph,
  currentEstPower,
  ergTargetW,
  modeName(gMode)
);


Serial.printf("grade=%.2f tgtLogical=%ld physTgt=%ld physPos=%ld logPos=%ld\n",
              gradeFloat,
              (long)logStepTarget,
              (long)logicalToSteps(logStepTarget),
              (long)physStepPos,
              (long)logStepPos);


}


static void noData() {
  Serial.println("No data received..");
}


static void ftmsAck(uint8_t reqOpcode, uint8_t resultCode = 0x01) {
  // Response Code (0x80), Request Opcode, Result Code (0x01 = success)
  uint8_t rsp[3] = { 0x80, reqOpcode, resultCode };
  pControlPoint->setValue(rsp, sizeof(rsp));
  pControlPoint->indicate();
}


class ControlPointCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* chr) override {
    String v = chr->getValue();
    if (v.length() == 0) return;


    const uint8_t* rx = (const uint8_t*)v.c_str();
    const size_t   n  = v.length();


    const uint8_t opcode = rx[0];


    switch (opcode) {
      case 0x00: // Request control
        ftmsState = 0;
        ftmsAck(0x00, 0x01);
        Serial.println("0x00");
        break;


      case 0x01: // Reset
        gMode = MODE_IDLE;
        ftmsState = 1;
        ftmsAck(0x01, 0x01);
        Serial.println("Reset");
        break;


      case 0x05: // ERG mode
        if (n < 3) { ftmsAck(0x05, 0x04); break; } // invalid parameter

        {
          int16_t w = (int16_t)((uint16_t)rx[1] | ((uint16_t)rx[2] << 8));
          w = constrain(w, 0, 4000);
          ergTargetW = w;         // IMPORTANT: update global (no shadowing)
          gLastErgRxMs = millis();
          gMode = MODE_ERG;       // <<< mode set here
        }

        ftmsState = 5;
        ftmsAck(0x05, 0x01);
        break;



      case 0x07: // Start/Resume
        ftmsState = 7;
        ftmsAck(0x07, 0x01);
        Serial.println("Resume");
        break;


      case 0x11: { // Simulation params (Zwift grade)
        if (n < 7) { ftmsAck(0x11, 0x04); break; } // invalid parameter
      
        int16_t gradeRaw = (int16_t)((rx[4] << 8) | rx[3]);

        gradeFloat = gradeRaw / 100.0f;
        /*if (gradeFloat < 0) gradeFloat *= 2.0f;*/

        gMode = MODE_SIM;                      // <<< mode set here

        ftmsState = 11;
        ftmsAck(0x11, 0x01);
      } break;



      default:
        // leave ftmsState unchanged
        ftmsAck(opcode, 0x02); // 0x02 = Op Code Not Supported
        break;
    }
  }
};

static void otaMarkAppValidAfterBoot() {
  // If rollback is enabled in the bootloader, new OTA images boot as "pending verify".
  // Once we decide things look good, we mark valid to cancel rollback.
  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err == ESP_OK) {
    Serial.println("[OTA] App marked VALID (rollback canceled if pending).");
  } else if (err == ESP_ERR_OTA_ROLLBACK_INVALID_STATE) {
    // Normal if rollback isn’t enabled or app isn't in pending state.
    Serial.println("[OTA] Rollback not pending (or not enabled).");
  } else {
    Serial.printf("[OTA] Mark valid failed: %d\n", (int)err);
  }
}

static const esp_partition_t* getNextOtaPartition() {
  return esp_ota_get_next_update_partition(nullptr);
}

static void otaPrintProgress(uint32_t written, uint32_t total) {
  if (total == 0) return;
  uint8_t pct = (uint8_t)((written * 100UL) / total);
  if (pct != gOtaLastPct && (pct % 5 == 0 || pct == 100)) { // every 5%
    gOtaLastPct = pct;
    Serial.printf("[OTA] Progress: %u%% (%lu/%lu)\n",
                  (unsigned)pct, (unsigned long)written, (unsigned long)total);
  }
}

static inline bool otaIsUnlocked() {
  return otaUnlockedUntilMs != 0 && (int32_t)(otaUnlockedUntilMs - millis()) > 0;
}

static void otaUnlockNow() {
  otaUnlockedUntilMs = millis() + OTA_UNLOCK_WINDOW_MS;
  Serial.printf("[OTA] Unlocked for %lu ms\n", (unsigned long)OTA_UNLOCK_WINDOW_MS);
}

static String htmlEscape(const String& in) {
  String s = in;
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  s.replace("'", "&#39;");
  return s;
}

// ---- JSON diagnostics ----
static void handleDiagJson() {
  String json = "{";

  json += "\"mode\":\"" + String(modeName(gMode)) + "\",";
  json += "\"grade\":" + String(gradeFloat, 2) + ",";
  json += "\"speed_mph\":" + String(currentSpeedMph, 2) + ",";
  json += "\"est_power_w\":" + String(currentEstPower) + ",";
  json += "\"erg_target_w\":" + String((int)ergTargetW) + ",";

  json += "\"log_pos\":" + String((long)logStepPos) + ",";
  json += "\"log_tgt\":" + String((long)logStepTarget) + ",";
  json += "\"phys_pos\":" + String((long)physStepPos) + ",";
  json += "\"phys_tgt\":" + String((long)logicalToSteps(logStepTarget)) + ",";

  json += "\"run_sps\":" + String(runSpeedSps, 1) + ",";
  json += "\"step_enabled\":" + String(gStepEn ? "true" : "false") + ",";
  json += "\"limit_pressed\":" + String(limitPressed() ? "true" : "false") + ",";
  json += "\"ota_unlocked\":" + String(otaIsUnlocked() ? "true" : "false") + ",";

  json += "\"ap_ip\":\"" + WiFi.softAPIP().toString() + "\"";

  json += "}";
  server.send(200, "application/json", json);
}

// ---- HTML main page with diagnostics and goto form ----
static void handleRoot() {
  String html;
  html.reserve(2200);

  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>InsideRide Controller</title></head><body>";
  html += "<h2>InsideRide Controller</h2>";

  html += "<p><b>AP IP:</b> " + WiFi.softAPIP().toString() + "</p>";
  html += "<p><b>Mode:</b> " + String(modeName(gMode)) + "</p>";

  html += "<h3>Diagnostics</h3><pre>";
  html += "Grade: " + String(gradeFloat, 2) + " %\n";
  html += "Speed: " + String(currentSpeedMph, 2) + " MPH\n";
  html += "Est Power: " + String(currentEstPower) + " W\n";
  html += "ERG Target: " + String((int)ergTargetW) + " W\n";
  html += "\n";
  html += "Log Pos: " + String((long)logStepPos) + " / Tgt: " + String((long)logStepTarget) + "\n";
  html += "Phys Pos: " + String((long)physStepPos) + " / Tgt: " + String((long)logicalToSteps(logStepTarget)) + "\n";
  html += "Run Speed: " + String(runSpeedSps, 1) + " steps/s\n";
  html += "Stepper Enabled: " + String(gStepEn ? "YES" : "NO") + "\n";
  html += "Limit Pressed: " + String(limitPressed() ? "YES" : "NO") + "\n";
  html += "OTA: " + String(otaIsUnlocked() ? "UNLOCKED" : "LOCKED (press CAL)") + "\n";
  html += "OTA Speed Gate: <= " + String(OTA_MAX_SPEED_MPH, 2) + " MPH\n";
  html += "</pre>";

  html += "<p><a href='/diag.json'>/diag.json</a></p>";

  html += "<h3>Actions</h3>";
  html += "<ul>";
  html += "<li><a href='/enable?on=1'>Enable stepper</a></li>";
  html += "<li><a href='/enable?on=0'>Disable stepper</a></li>";
  html += "<li><a href='/update'>Firmware Update (locked unless CAL pressed)</a></li>";
  html += "</ul>";

  html += "<p><b>Mode:</b> ";
  html += modeName(gMode);
  html += "</p>";

  html += "<p><b>Manual Hold:</b> ";
  html += (gManualHoldActive ? "ACTIVE" : "OFF");
  html += " | <b>Hold Target:</b> ";
  html += String((long)gManualHoldTarget);
  html += "</p>";

  html += "<p><b>Speed:</b> ";
  html += String(currentSpeedMph, 2);
  html += " MPH | <b>Grade:</b> ";
  html += String(gradeFloat, 2);
  html += "% | <b>ERG Target:</b> ";
  html += String((int)ergTargetW);
  html += " W</p>";

  html += "<p><b>Pos:</b> ";
  html += String((long)logStepPos);
  html += " | <b>Tgt:</b> ";
  html += String((long)logStepTarget);
  html += "</p>";

  html += "<hr/>";

  // Go-To Hold form
  html += "<form action='/goto_hold' method='GET'>"
          "<label>Go-To (logical 0-1000):</label><br/>"
          "<input name='pos' type='number' min='0' max='1000' step='1' value='500'/>"
          "<button type='submit'>Go-To (Hold)</button>"
          "</form>";

  // Resume button
  html += "<form action='/resume_zwift' method='POST' style='margin-top:10px;'>"
          "<button type='submit'>Resume Software Control</button>"
          "</form>";

  html += "<p><b>FW:</b> " + String(FW_VERSION) + "</p>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ---- Go-to handler ----
static void handleGoto() {
  if (!server.hasArg("pos")) {
    server.send(400, "text/plain", "Missing pos. Use /goto?pos=500");
    return;
  }
  long pos = server.arg("pos").toInt();
  if (pos < LOGICAL_MIN) pos = LOGICAL_MIN;
  if (pos > LOGICAL_MAX) pos = LOGICAL_MAX;

  stepperEnable(true);     // optional: auto-enable on goto
  stepperGoto((int32_t)pos);

  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "OK");
}

static void handleGotoHold() {
  if (!server.hasArg("pos")) {
    server.send(400, "text/plain", "Missing pos");
    return;
  }

  int32_t pos = (int32_t)server.arg("pos").toInt();
  pos = clampLogical(pos);

  gManualHoldTarget = pos;
  gManualHoldActive = true;

  // Optional: ensure stepper enabled when user commands a hold
  stepperEnable(true);

  Serial.printf("[WEB] Manual HOLD active: target=%ld\n", (long)pos);

  // Redirect back to root
  server.sendHeader("Location", "/", true);
  server.send(303);
}

static void handleResumeZwift() {
  gManualHoldActive = false;

  Serial.println("[WEB] Manual HOLD cleared; returning to Zwift control.");

  server.sendHeader("Location", "/", true);
  server.send(303);
}

// ---- Enable/disable handler ----
static void handleEnable() {
  if (!server.hasArg("on")) {
    server.send(400, "text/plain", "Missing on. Use /enable?on=1 or /enable?on=0");
    return;
  }
  int on = server.arg("on").toInt();
  stepperEnable(on != 0);

  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "OK");
}

// ---- OTA form ----
static void handleUpdateForm() {
  if (!server.authenticate(OTA_USER, OTA_PASS)) {
    return server.requestAuthentication();
  }

  if (!otaIsUnlocked()) {
    server.send(403, "text/plain",
                "OTA is LOCKED. Press CAL to unlock for 60 seconds, then refresh /update.");
    return;
  }

  if (OTA_DENY_WHEN_BLE_CONNECTED && deviceConnected) {
    server.send(423, "text/plain",
                "OTA is disabled while BLE is connected. Disconnect Zwift, then retry.");
    return;
  }
  
  if (OTA_REQUIRE_LOW_SPEED && (currentSpeedMph > OTA_MAX_SPEED_MPH)) {
    String msg = "OTA is disabled while moving. Current speed is " +
                 String(currentSpeedMph, 2) + " MPH. Stop the wheel (<= " +
                 String(OTA_MAX_SPEED_MPH, 2) + " MPH) and retry.";
    server.send(423, "text/plain", msg);
    return;
  }
  const esp_partition_t* p = getNextOtaPartition();
  String partInfo = p ? String((unsigned long)p->size) : String("unknown");

  String html =
    "<!doctype html><html><body>"
    "<h3>Firmware Update</h3>"
    "<p>OTA is UNLOCKED.</p>"
    "<p><b>Next OTA partition size:</b> " + partInfo + " bytes</p>"
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='firmware'>"
    "<input type='submit' value='Update'>"
    "</form>"
    "<p>Note: BLE must be disconnected during OTA.</p>"
    "</body></html>";

  server.send(200, "text/html", html);
}


// ---- OTA upload ----
static void handleUpdateUpload() {
  if (!server.authenticate(OTA_USER, OTA_PASS)) {
    return server.requestAuthentication();
  }

  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {

    // Gate checks ONLY here
    if (!otaIsUnlocked()) {
      gOtaInProgress = false;
      gOtaDone = true; gOtaOk = false; gOtaErr = "OTA locked (press CAL)";
      return;
    }
    if (OTA_DENY_WHEN_BLE_CONNECTED && deviceConnected) {
      gOtaInProgress = false;
      gOtaDone = true; gOtaOk = false; gOtaErr = "BLE connected; OTA denied";
      return;
    }
    if (OTA_REQUIRE_LOW_SPEED && (currentSpeedMph > OTA_MAX_SPEED_MPH)) {
      gOtaInProgress = false;
      gOtaDone = true; gOtaOk = false; gOtaErr = "Moving too fast for OTA";
      return;
    }

    // Optional: reject a second start while one is running
    if (gOtaInProgress) {
      Update.abort();
      gOtaInProgress = false;
    }

    Serial.printf("[OTA] Upload start: %s\n", upload.filename.c_str());

    // Consume unlock once upload is accepted
    otaUnlockedUntilMs = 0;

    gOtaDone = false; gOtaOk = false; gOtaErr = "";
    gOtaInProgress = true;

    stepperEnable(false);
    gOtaWritten = 0;
    gOtaLastPct = 255;

    const esp_partition_t* part = getNextOtaPartition();
    if (!part) {
      Update.abort();
      gOtaDone = true; gOtaOk = false; gOtaErr = "No OTA partition found";
      gOtaInProgress = false;
      return;
    }

    gOtaExpectedSize = (uint32_t)part->size;

    if (!Update.begin(gOtaExpectedSize)) {
      Serial.print("[OTA] Update.begin FAILED: ");
      Update.printError(Serial);
      gOtaDone = true; gOtaOk = false; gOtaErr = "Update.begin failed";
      gOtaInProgress = false;
      return;
    }

    Serial.printf("[OTA] Update.begin OK. slot_size=%lu\n", (unsigned long)gOtaExpectedSize);
    return;
  }

  // For WRITE/END/ABORTED: do NOT check otaIsUnlocked() again
  if (!gOtaInProgress) {
    // Stray callbacks; ignore
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE) {
    size_t w = Update.write(upload.buf, upload.currentSize);
    gOtaWritten += (uint32_t)w;
    otaPrintProgress(gOtaWritten, gOtaExpectedSize);
    return;
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      Serial.print("[OTA] Update.end FAILED: ");
      Update.printError(Serial);
      gOtaDone = true; gOtaOk = false; gOtaErr = "Update.end failed";
      gOtaInProgress = false;
      return;
    }

    gOtaDone = true; gOtaOk = true;
    gOtaInProgress = false;
    Serial.println("[OTA] Success, awaiting finalizer response...");
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED) {
    Serial.println("[OTA] Upload aborted.");
    Update.abort();
    gOtaDone = true; gOtaOk = false; gOtaErr = "Upload aborted";
    gOtaInProgress = false;
    return;
  }
}


static void handleUpdatePostFinalizer() {
  if (!server.authenticate(OTA_USER, OTA_PASS)) {
    return server.requestAuthentication();
  }

  if (!gOtaDone) {
    // Should not happen often, but keeps behavior deterministic
    server.send(500, "text/plain", "OTA not completed.");
    return;
  }

  if (gOtaOk) {
    server.send(200, "text/plain", "Update OK. Rebooting...");
    delay(250);
    ESP.restart();
  } else {
    String msg = "Update failed: " + gOtaErr;
    server.send(500, "text/plain", msg);
  }
}


void setup() {
  Serial.begin(115200);
  delay(100);

  otaMarkAppValidAfterBoot();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);


  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(CAL_BTN_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  ledWrite(false);
  ledSetPattern(LED_BLINK_MED); // optional “boot” indication

  Serial.println("Starting initialisation routine");


  runSpeedSps = 2500.0f;
  jogSpeedSps = 1600.0f;
  stepperSetRunSpeed();   // safer at boot for homing


  stepperEnable(true);
  homeStepper();


  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  Serial.printf("[FW] %s\n", FW_VERSION);

  //Setup BLE
  Serial.println("Creating BLE server...");
  BLEDevice::init("InsideRideFTMS");


  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());


  // Create the BLE Service
  Serial.println("Define service...");
  BLEService* pService = pServer->createService(FTMSDEVICE_FTMS_UUID);


  // Create BLE Characteristics
  Serial.println("Define characteristics");
  pIndoorBike = pService->createCharacteristic(FTMSDEVICE_INDOOR_BIKE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pIndoorBike->addDescriptor(new BLE2902());
  pControlPoint = pService->createCharacteristic(FTMSDEVICE_FTMS_CONTROL_POINT_CHAR_UUID, BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_WRITE);
  pControlPoint->addDescriptor(new BLE2902());
  static ControlPointCallbacks sCpCb;
  pFeature = pService->createCharacteristic(FTMSDEVICE_FTMS_FEATURE_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
  pFeature->addDescriptor(new BLE2902());
  pStatus = pService->createCharacteristic(FTMSDEVICE_FTMS_STATUS_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pStatus->addDescriptor(new BLE2902());
  pControlPoint->setCallbacks(&sCpCb);


  // Start the service
  Serial.println("Staring BLE service...");
  pService->start();

  // FTMS Feature (0x2ACC) - enable Indoor Bike Simulation support.
  // This is the same payload you had in the working version.
  uint8_t feature[8] = { 0x00, 0x00, 0x00, 0x00, 0x0C, 0x20, 0x00, 0x00 };
  pFeature->setValue(feature, 8);


  // Start advertising
  Serial.println("Define the advertiser...");
  pAdvertising = BLEDevice::getAdvertising();


  pAdvertising->setScanResponse(true);
  pAdvertising->addServiceUUID(FTMSDEVICE_FTMS_UUID);
  pAdvertising->setMinPreferred(0x06);  // set value to 0x00 to not advertise this parameter
  Serial.println("Starting advertiser...");
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");


  Serial.println("Waiting for");


  Serial.println("connection...");

  startApAndWeb();

}


/*
  The main loop which runs continuously.
  Checks whether something is connected
  - Receive bike simulation data, adjust the trainer and send acknowledgement back
*/




void loop() {
  server.handleClient();
  // If OTA is running, do not do BLE notify / stepper / heavy Serial prints.
  // Keep loop tight so WiFi can receive upload data reliably.
  if (gOtaInProgress) {
    // Only service web + button (optional) while uploading.
    ledSelectPattern();
    ledUpdate();
    delay(1);
    return;
  }

  const uint32_t nowMs = millis();
  static uint32_t delayMs = 0;
  const int printTime = 1000;
  static uint32_t bleDelayMs = 0;
  const int bleDelayTime = 100;
  static uint32_t lastPowerNotifyMs = 0;
  const uint32_t POWER_NOTIFY_PERIOD_MS = 100; // 10 Hz

  if (millis() - delayMs >= printTime){
    printDiag();
    /*Serial.printf("simAgeMs=%lu\n", (unsigned long)(millis() - gLastSimRxMs));*/
    delayMs = millis();
  }

  float rpmRaw = readRPM();
  static uint32_t lastIbDataMs = 0;


  // Exponential smoothing with time-based alpha
  if (gLastRpmFilterMs == 0) gLastRpmFilterMs = nowMs;
  float dtS = (nowMs - gLastRpmFilterMs) * 0.001f;
  gLastRpmFilterMs = nowMs;

  

  // alpha = dt / (tau + dt)
  float alpha = (dtS <= 0) ? 1.0f : (dtS / (RPM_FILTER_TAU_S + dtS));


  // If you stop, let it decay quickly to 0 instead of lingering forever
  if (rpmRaw < 0.5f) {
    alpha = 0.35f; // quicker decay when stopping
  }


  gRpmFiltered = gRpmFiltered + alpha * (rpmRaw - gRpmFiltered);
  currentRPM = gRpmFiltered;
  currentSpeedMph = rpmToMph(currentRPM);

  updateStepperEnableFromSpeed(currentSpeedMph);

  updateLimitDebounce();

  // If the limit is hit during normal operation, schedule a rehome.
  // (This covers cases where you drift and physically touch the switch.)
  if (!gIsHoming && limitPressed()) {
    requestRehome("limitPressed in loop");
  }

  // Perform rehome if requested
  if (gRehomeRequested && !gIsHoming) {
    gRehomeRequested = false;

    // Optional: pause control updates while homing
    ControlMode prevMode = gMode;
    gMode = MODE_IDLE;

    gIsHoming = true;

    // Ensure motor is enabled and use jog speed for homing
    stepperEnable(true);
    bool ok = homeStepper();

    gIsHoming = false;

    if (!ok) {
      Serial.println("[SAFETY] Rehome failed (timeout). Holding position.");
      // You might choose to disable motor here:
      // stepperEnable(false);
      gMode = MODE_IDLE;
      tgtLogical = 0;
    } else {
      Serial.println("[SAFETY] Rehome complete.");
      // After homing, return to previous mode
      gMode = prevMode;

      // Force target to current position initially to prevent sudden jump
      tgtLogical = logStepPos;
    }
  }


  static uint32_t lastTargetUpdateMs = 0;
  if (millis() - lastTargetUpdateMs >= 50) { // 20 Hz target update
    lastTargetUpdateMs = millis();

    ControlMode mode = gMode; // read volatile once

    if (mode == MODE_ERG) {
      // ERG: compute target from speed + target watts continuously
      int16_t w = ergTargetW; // read volatile once
      int32_t t = (int32_t)lround(stepFromPowerSpeed((double)currentSpeedMph,
                                                    (double)w));
      t = constrain(t, LOGICAL_MIN, LOGICAL_MAX);
      tgtLogical = t;
    }
    else if (mode == MODE_SIM) {
    double gradeClamp = gradeFloat;

    // clamp grade to table range
    if (gradeClamp < Ystep[0]) gradeClamp = Ystep[0];
    if (gradeClamp > Ystep[Ycountstep - 1]) gradeClamp = Ystep[Ycountstep - 1];

    // clamp speed to table range (or allow your function to handle it)
    double speedClamp = currentSpeedMph;
    if (speedClamp < Xstep[0]) speedClamp = Xstep[0];
    if (speedClamp > Xstep[Xcountstep - 1]) speedClamp = Xstep[Xcountstep - 1];

    int32_t t = (int32_t)lround(gradeToSteps(speedClamp, gradeClamp));
    tgtLogical = constrain(t, LOGICAL_MIN, LOGICAL_MAX);

    }

  }
  
  int32_t target;

  if (gManualHoldActive) {
    target = gManualHoldTarget;     // manual override wins
  } else {
    target = tgtLogical;            // Zwift control (ERG/SIM)
  }

  stepperGoto(target);
  stepperUpdate();



  if (deviceConnected && millis() - lastPowerNotifyMs >= POWER_NOTIFY_PERIOD_MS) {
  lastPowerNotifyMs = millis();

  // --- Compute estimated power ---
  currentEstPower = (int)lround(
    powerFromSpeedPos(
      (double)currentSpeedMph,
      (double)logStepPos
      )
    );
    
    
    // cadence: 0 below 2 mph, 90 rpm above 2 mph
    uint16_t cadence_rpm = (currentSpeedMph < 2.0f) ? 0 : 90;
    uint16_t cadence_ftms = cadence_rpm * 2; // 0.5 rpm units

    // write cadence at bytes [4..5] (per layout above)
    FTMSDEVICE_INDOOR_BIKE_CHARData[4] = (uint8_t)(cadence_ftms & 0xFF);
    FTMSDEVICE_INDOOR_BIKE_CHARData[5] = (uint8_t)(cadence_ftms >> 8);

    // write power at bytes [6..7]
    int16_t p = (int16_t)constrain(currentEstPower, 0, 2000);
    FTMSDEVICE_INDOOR_BIKE_CHARData[6] = (uint8_t)(p & 0xFF);
    FTMSDEVICE_INDOOR_BIKE_CHARData[7] = (uint8_t)((uint16_t)p >> 8);

    // notify with correct length (10)
    pIndoorBike->setValue(FTMSDEVICE_INDOOR_BIKE_CHARData, sizeof(FTMSDEVICE_INDOOR_BIKE_CHARData));
    pIndoorBike->notify();

  }

  // Press CAL to unlock OTA window
  if (calPressedEdge()) {
    otaUnlockNow();
  }
  ledSelectPattern();
  ledUpdate();
}

