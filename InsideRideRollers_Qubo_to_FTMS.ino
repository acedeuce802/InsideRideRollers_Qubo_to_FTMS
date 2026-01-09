/*
   A BLE turbo trainer controller for Zwift


   Program receives environment date from the Zwift game (via BLE) and adjusts the resistance of a turbo trainer using a stepper motor
   Uses an ESP32 and includes an LCD display for feedback.


   Copyright 2020 Peter Everett
   v1.0 Aug 2020 - Initial version


   This work is licensed the GNU General Public License v3 (GPL-3)
   Modified for Proform TDF bike generation 2 by Greg Masik April 1, 2021. Removed stepper motor and LCD display
   April 28, modified to allow for zwift half grade for negative grades. May 10, 2021 added I2C LCD display for grade.
   January 5, 2022- Added case 0x01 to allow for reset request from Zwift after Dec 2021 update.


   Modified by Adam Watson January 3rd, 2024:
     Only kept BLE interface for FTMS from source code.  
     Added case 0x05 for ERG mode control for ESP32 to receive target power
     Added hall sensor input to calculate RPM of cycling rollers and smooth with an averaged array
     Added servo control, to control magnetic resistance to cycling rollers
     Added bilinear interpolation for inputs speed and target power, and output servo position


*/


//Tested with Xiao ESP32-C3

enum ControlMode : uint8_t {
  MODE_IDLE = 0,
  MODE_SIM  = 1,   // grade -> steps
  MODE_ERG  = 2    // (speed, targetW) -> steps
};

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>


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


// ===================== Millis =====================
unsigned long whileMillis;
unsigned long millisValue = 0;


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
static float    stepSpeedSps   = 1600.0f;
static uint32_t stepIntervalUs = 5000;
static uint32_t lastStepUs     = 0;


static float jogSpeedSps = 800.0f;   // manual moves (home/jog/goto)
static float runSpeedSps = 1600.0f;  // normal tracking moves (if you use it later)




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

// ===================== Power from Speed/Pos Calibration data (5x5) =====================

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


// ==================== Power from Speed/Pos Bilinear Interpolation =========================
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
  Serial.printf("[STEP] enable=%s\n", en ? "ON" : "OFF");
}


static void stepperSetSpeed(float sps) {
  if (sps < 50) sps = 50;
  if (sps > 4000) sps = 4000;
  stepSpeedSps = sps;
  stepIntervalUs = (uint32_t)(1000000.0f / sps + 0.5f);
  Serial.printf("[STEP] speed=%.1f sps interval=%lu us\n", stepSpeedSps, (unsigned long)stepIntervalUs);
}


// ===================== Homing =====================
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
  return true;
}


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


  // Safety clamp (should never trigger in normal operation)
  if (physStepPos < PHYS_MIN_STEPS) physStepPos = PHYS_MIN_STEPS;
  if (physStepPos > PHYS_MAX_STEPS) physStepPos = PHYS_MAX_STEPS;


  uint32_t now = micros();
  if ((now - lastStepUs) < stepIntervalUs) return;
  lastStepUs = now;


  bool forward = (physStepTarget > physStepPos);
  stepperSetDir(forward);


  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, LOW);


  physStepPos += forward ? 1 : -1;
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

static volatile ControlMode gMode = MODE_IDLE;

// already in your code (keep these, but ensure they are globals, not shadowed)
static volatile int16_t ergTargetW  = 0;
static volatile int32_t tgtLogical  = 0;  // what loop uses for stepperGoto()
static volatile uint32_t gLastSimRxMs = 0;
static volatile uint32_t gLastErgRxMs = 0;



const uint16_t FTMSDEVICE_INDOOR_BIKE_CHARDef = 0b0000000001000100;  // flags for indoor bike data characteristics - power and cadence
uint16_t speedOut = 100;
int16_t powerOut = 100;
uint8_t FTMSDEVICE_INDOOR_BIKE_CHARData[8] = {  // values for setup - little endian order
  (uint8_t)(FTMSDEVICE_INDOOR_BIKE_CHARDef & 0xff),
  (uint8_t)(FTMSDEVICE_INDOOR_BIKE_CHARDef >> 8),
  (uint8_t)(speedOut & 0xff),
  (uint8_t)(speedOut >> 8),
  (uint8_t)(speedOut & 0xff),
  (uint8_t)(speedOut >> 8),
  0x64,
  0
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
        if (gradeFloat < 0) gradeFloat *= 2.0f;

        tgtLogical = gradeToSteps(gradeFloat); // update immediately on SIM packet
        gLastSimRxMs = millis();
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


void setup() {
  Serial.begin(115200);
  delay(100);


  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);


  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(CAL_BTN_PIN, INPUT_PULLUP);


  Serial.println("Starting initialisation routine");


  runSpeedSps = 1600.0f;
  jogSpeedSps = 1600.0f;
  stepperSetRunSpeed();   // safer at boot for homing


  stepperEnable(true);
  homeStepper();


  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);


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


}


/*
  The main loop which runs continuously.
  Checks whether something is connected
  - Receive bike simulation data, adjust the trainer and send acknowledgement back
*/




void loop() {
  const uint32_t nowMs = millis();
  static uint32_t delayMs = 0;
  const int printTime = 1000;
  static uint32_t bleDelayMs = 0;
  const int bleDelayTime = 100;
  static uint32_t lastPowerNotifyMs = 0;
  const uint32_t POWER_NOTIFY_PERIOD_MS = 100; // 10 Hz


  if (millis() - delayMs >= printTime){
    printDiag();
    Serial.printf("simAgeMs=%lu\n", (unsigned long)(millis() - gLastSimRxMs));
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


  updateLimitDebounce();

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
  

    }
  // else SIM: gTgtLogical is updated in case 0x11 (or you can compute here from gradeFloat)
  }
  
  stepperGoto(tgtLogical);
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
    
    
    FTMSDEVICE_INDOOR_BIKE_CHARData[6] = (uint8_t)(currentEstPower & 0xFF);
    FTMSDEVICE_INDOOR_BIKE_CHARData[7] = (uint8_t)(currentEstPower >> 8);

    pIndoorBike->setValue(FTMSDEVICE_INDOOR_BIKE_CHARData, 8);
    pIndoorBike->notify();
  }


}
