/*
 * sensors.cpp - Sensor Reading and Power Calculation Implementation
 */

#include "sensors.h"
#include "stepper_control.h"
#include "ble_trainer.h"  // For deviceConnected status

// ==================== GLOBAL SENSOR DATA ====================
float currentRPM = 0.0f;
float currentSpeedMph = 0.0f;
float currentPowerWatts = 0.0f;

// ==================== HALL SENSOR STATE ====================
static volatile uint32_t gLastHallUs = 0;
static volatile uint32_t gHallIntervalUs = 0;
static volatile uint32_t gHallEdges = 0;
static volatile uint32_t gLastAcceptedHallUs = 0;

// Hall sensor configuration (using values from config.h)
static const uint32_t HALL_MIN_DT_US = 1500;       // Reject pulses faster than this
static const uint32_t HALL_HOLDOFF_US = 3000;      // Anti-chatter holdoff (3ms)

// RPM filtering
static float gRpmFiltered = 0.0f;
static const float RPM_FILTER_TAU_S = 0.60f;       // Time constant for smoothing
static uint32_t gLastRpmFilterMs = 0;

// Speed conversion constants (using ROLLER_DIAMETER_IN from config.h)
static constexpr float INCHES_PER_MILE = 63360.0f;
static constexpr float MINUTES_PER_HOUR = 60.0f;
static constexpr float RPM_TO_MPH = (ROLLER_DIAMETER_IN * PI * MINUTES_PER_HOUR) / INCHES_PER_MILE;

// ==================== POWER CALIBRATION DATA ====================
// Speed grid (mph)
static double X[] = { 0, 5, 10, 15, 20, 25, 50 };
// Position grid (logical units)
static double Y[] = { 0, 250, 500, 750, 1000 };

static const int Xcount = sizeof(X) / sizeof(X[0]);
static const int Ycount = sizeof(Y) / sizeof(Y[0]);

// Power calibration table [speed][position] = watts
static double Z[7][5] = {
  {   0,   0,   0,   0,     0 },  // 0 mph
  {  52,  68,  80, 102,   124 },  // 5 mph
  { 117, 143, 217, 280,   343 },  // 10 mph
  { 188, 246, 383, 490,   597 },  // 15 mph
  { 265, 380, 580, 732,   884 },  // 20 mph
  { 349, 544, 806, 1006, 1206 },  // 25 mph
  { 861, 1806, 2388, 2856, 3324 } // 50 mph
};

// ==================== HALL SENSOR ISR ====================

void IRAM_ATTR hallISR() {
  uint32_t now = micros();
  
  // Holdoff to suppress chatter
  uint32_t sinceAccepted = now - gLastAcceptedHallUs;
  if (gLastAcceptedHallUs != 0 && sinceAccepted < HALL_HOLDOFF_US) {
    return;
  }
  
  uint32_t dt = now - gLastHallUs;
  
  // Ignore very short intervals (bounce/noise)
  if (gLastHallUs != 0 && dt < HALL_MIN_DT_US) {
    return;
  }
  
  // Accept this edge
  gLastHallUs = now;
  gHallIntervalUs = dt;
  gHallEdges++;
  gLastAcceptedHallUs = now;
}

// ==================== HELPER FUNCTIONS ====================

static float readRPM() {
  noInterrupts();
  uint32_t dt = gHallIntervalUs;
  uint32_t last = gLastHallUs;
  interrupts();
  
  // Check for valid data
  if (last == 0) return 0.0f;
  if ((micros() - last) > 1000000UL) return 0.0f;  // Stopped >1s
  if (dt == 0) return 0.0f;
  
  // Calculate RPM
  float pps = 1e6f / (float)dt;                    // Pulses per second
  float rps = pps / (float)HALL_PULSES_PER_REV;    // Revolutions per second
  return rps * 60.0f;                              // RPM
}

static float filterRPM(float rawRpm) {
  uint32_t now = millis();
  
  if (gLastRpmFilterMs == 0) {
    gLastRpmFilterMs = now;
    gRpmFiltered = rawRpm;
    return rawRpm;
  }
  
  float dt = (now - gLastRpmFilterMs) / 1000.0f;  // seconds
  gLastRpmFilterMs = now;
  
  // Exponential moving average
  float alpha = dt / (RPM_FILTER_TAU_S + dt);
  gRpmFiltered = alpha * rawRpm + (1.0f - alpha) * gRpmFiltered;
  
  return gRpmFiltered;
}

// ==================== CONVERSION FUNCTIONS ====================

float rpmToMph(float rpm) {
  if (rpm <= 0.0f) return 0.0f;
  return rpm * RPM_TO_MPH;
}

float mphToRpm(float mph) {
  if (mph <= 0.0f) return 0.0f;
  return mph / RPM_TO_MPH;
}

// ==================== POWER CALCULATION ====================

double powerFromSpeedPos(double speedMph, double posLogical) {
  // Bounds check
  if ((speedMph < X[0]) || (speedMph > X[Xcount - 1])) {
    Serial.println("[POWER] Speed out of range");
    return 0;
  }
  
  if ((posLogical < Y[0]) || (posLogical > Y[Ycount - 1])) {
    Serial.println("[POWER] Position out of range");
    return 0;
  }
  
  // Find grid indices
  int xIndex = 0, yIndex = 0;
  
  for (int i = Xcount - 2; i >= 0; --i) {
    if (speedMph >= X[i]) {
      xIndex = i;
      break;
    }
  }
  
  for (int i = Ycount - 2; i >= 0; --i) {
    if (posLogical >= Y[i]) {
      yIndex = i;
      break;
    }
  }
  
  // Bilinear interpolation
  double x1 = X[xIndex];
  double x2 = X[xIndex + 1];
  double y1 = Y[yIndex];
  double y2 = Y[yIndex + 1];
  
  double fQ11 = Z[xIndex][yIndex];
  double fQ12 = Z[xIndex][yIndex + 1];
  double fQ21 = Z[xIndex + 1][yIndex];
  double fQ22 = Z[xIndex + 1][yIndex + 1];
  
  // Interpolate in x direction
  double fxy1 = ((x2 - speedMph) / (x2 - x1)) * fQ11 + ((speedMph - x1) / (x2 - x1)) * fQ21;
  double fxy2 = ((x2 - speedMph) / (x2 - x1)) * fQ12 + ((speedMph - x1) / (x2 - x1)) * fQ22;
  
  // Interpolate in y direction
  double power = ((y2 - posLogical) / (y2 - y1)) * fxy1 + ((posLogical - y1) / (y2 - y1)) * fxy2;
  
  return power;
}

// ==================== ERG MODE: STEP FROM POWER/SPEED ====================

// ERG mode calibration: Speed grid (mph)
static double Xw[] = { 0, 5, 10, 15, 20, 25, 50 };
// ERG mode calibration: Power grid (watts)
static double Yw[] = { 0, 100, 150, 200, 250, 300, 400, 600, 1000 };

static const int Xcountw = sizeof(Xw) / sizeof(Xw[0]);
static const int Ycountw = sizeof(Yw) / sizeof(Yw[0]);

// ERG mode table: [speed][power] = position
static double Zw[7][9] = {
  {   0,    0,    0,    0,    0,    0,    0,    0,    0 },  // 0 mph
  {   0,  739, 1000, 1000, 1000, 1000, 1000, 1000, 1000 },  // 5 mph
  {   0,    0,  212,  442,  651,  841, 1000, 1000, 1000 },  // 10 mph
  {   0,    0,    0,   70,  198,  322,  560,  996, 1000 },  // 15 mph
  {   0,    0,    0,    0,    0,   79,  238,  552, 1000 },  // 20 mph
  {   0,    0,    0,    0,    0,    0,   67,  285,  745 },  // 25 mph
  {   0,    0,    0,    0,    0,    0,    0,    0,   26 }   // 50 mph
};

double stepFromPowerSpeed(double speedMph, double targetWatts) {
  // Bounds check
  if ((speedMph < Xw[0]) || (speedMph > Xw[Xcountw - 1])) {
    Serial.println("[ERG] Speed out of range");
    return 0;
  }
  
  if ((targetWatts < Yw[0]) || (targetWatts > Yw[Ycountw - 1])) {
    Serial.println("[ERG] Power out of range");
    return 0;
  }
  
  // Find grid indices
  int xIndex = 0, yIndex = 0;
  
  for (int i = Xcountw - 2; i >= 0; --i) {
    if (speedMph >= Xw[i]) {
      xIndex = i;
      break;
    }
  }
  
  for (int i = Ycountw - 2; i >= 0; --i) {
    if (targetWatts >= Yw[i]) {
      yIndex = i;
      break;
    }
  }
  
  // Bilinear interpolation
  double x1 = Xw[xIndex];
  double x2 = Xw[xIndex + 1];
  double y1 = Yw[yIndex];
  double y2 = Yw[yIndex + 1];
  
  double fQ11 = Zw[xIndex][yIndex];
  double fQ12 = Zw[xIndex][yIndex + 1];
  double fQ21 = Zw[xIndex + 1][yIndex];
  double fQ22 = Zw[xIndex + 1][yIndex + 1];
  
  double fxy1 = ((x2 - speedMph) / (x2 - x1)) * fQ11 + ((speedMph - x1) / (x2 - x1)) * fQ21;
  double fxy2 = ((x2 - speedMph) / (x2 - x1)) * fQ12 + ((speedMph - x1) / (x2 - x1)) * fQ22;
  
  double position = ((y2 - targetWatts) / (y2 - y1)) * fxy1 + ((targetWatts - y1) / (y2 - y1)) * fxy2;
  
  return position;
}

// ==================== SIM MODE: STEP FROM GRADE/SPEED ====================

// SIM mode calibration: Speed grid (mph)
static double Xstep[] = { 0, 5, 10, 15, 20, 25, 30, 50 };
// SIM mode calibration: Grade grid (percent)
static double Ystep[] = { -4, 0, 2, 4, 6, 8, 10 };

static const int Xcountstep = sizeof(Xstep) / sizeof(Xstep[0]);
static const int Ycountstep = sizeof(Ystep) / sizeof(Ystep[0]);

// SIM mode table: [speed][grade] = position
static double Zstep[8][7] = {
  {   0,  167,  333,  500,  667,  833, 1000 },  // 0 mph
  {   0,  167,  333,  500,  667,  833, 1000 },  // 5 mph
  {   0,  167,  333,  500,  667,  833, 1000 },  // 10 mph
  {   0,  167,  333,  500,  667,  833, 1000 },  // 15 mph
  {   0,  167,  333,  500,  667,  833, 1000 },  // 20 mph
  { 167,  333,  500,  677,  834, 1000, 1000 },  // 25 mph
  { 333,  500,  677,  834, 1000, 1000, 1000 },  // 30 mph
  { 500,  500,  677,  834, 1000, 1000, 1000 }   // 50 mph
};

double gradeToSteps(double speedMph, double gradePercent) {
  // Bounds check
  if ((speedMph < Xstep[0]) || (speedMph > Xstep[Xcountstep - 1])) {
    Serial.println("[SIM] Speed out of range");
    return 500;  // Default to mid position
  }
  
  if ((gradePercent < Ystep[0]) || (gradePercent > Ystep[Ycountstep - 1])) {
    Serial.println("[SIM] Grade out of range");
    // Clamp instead of returning 0
    gradePercent = constrain(gradePercent, Ystep[0], Ystep[Ycountstep - 1]);
  }
  
  // Find grid indices
  int xIndex = 0, yIndex = 0;
  
  for (int i = Xcountstep - 2; i >= 0; --i) {
    if (speedMph >= Xstep[i]) {
      xIndex = i;
      break;
    }
  }
  
  for (int i = Ycountstep - 2; i >= 0; --i) {
    if (gradePercent >= Ystep[i]) {
      yIndex = i;
      break;
    }
  }
  
  // Bilinear interpolation
  double x1 = Xstep[xIndex];
  double x2 = Xstep[xIndex + 1];
  double y1 = Ystep[yIndex];
  double y2 = Ystep[yIndex + 1];
  
  double fQ11 = Zstep[xIndex][yIndex];
  double fQ12 = Zstep[xIndex][yIndex + 1];
  double fQ21 = Zstep[xIndex + 1][yIndex];
  double fQ22 = Zstep[xIndex + 1][yIndex + 1];
  
  double fxy1 = ((x2 - speedMph) / (x2 - x1)) * fQ11 + ((speedMph - x1) / (x2 - x1)) * fQ21;
  double fxy2 = ((x2 - speedMph) / (x2 - x1)) * fQ12 + ((speedMph - x1) / (x2 - x1)) * fQ22;
  
  double position = ((y2 - gradePercent) / (y2 - y1)) * fxy1 + ((gradePercent - y1) / (y2 - y1)) * fxy2;
  
  return position;
}

// ==================== PUBLIC FUNCTIONS ====================

void sensorsInit() {
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
  
  Serial.println("✓ Hall sensor initialized");
  Serial.printf("  Pulses per rev: %d\n", HALL_PULSES_PER_REV);
  Serial.printf("  Roller diameter: %.2f inches\n", ROLLER_DIAMETER_IN);
}

void sensorsUpdate() {
  // Read and filter RPM
  float rawRpm = readRPM();
  currentRPM = filterRPM(rawRpm);
  
  // Convert to speed
  currentSpeedMph = rpmToMph(currentRPM);
  
  // Calculate power from speed and current stepper position
  currentPowerWatts = powerFromSpeedPos(currentSpeedMph, (double)logStepPos);
  
  // Debug output (less frequent to avoid blocking web server)
  static uint32_t lastDebugMs = 0;
  if (millis() - lastDebugMs > 10000) {  // Changed from 5000 to 10000 (10 seconds)
    lastDebugMs = millis();
    Serial.printf("[SENSORS] RPM: %.1f  Speed: %.1f mph  Power: %.0f W  Pos: %ld\n",
                  currentRPM, currentSpeedMph, currentPowerWatts, logStepPos);
    Serial.printf("[STATUS] BLE Connected: %s\n", deviceConnected ? "YES" : "NO");
  }
}
