/*
 * stepper_control.cpp - Stepper Motor Control Implementation
 */

#include "stepper_control.h"

// ==================== GLOBAL STATE ====================
volatile ControlMode gMode = MODE_IDLE;
volatile int32_t logStepPos = 0;
volatile int32_t logStepTarget = 0;
volatile int32_t physStepPos = 0;
volatile int32_t physStepTarget = 0;
volatile bool gIsHoming = false;
volatile bool gRehomeRequested = false;
volatile bool gManualHoldActive = false;
volatile int32_t gManualHoldTarget = 0;

// ==================== STEPPER MOTION STATE ====================
static float stepSpeedSps = DEFAULT_STEP_SPEED_SPS;
static uint32_t stepIntervalUs = 400;  // will be updated
static uint32_t lastStepUs = 0;

// Direction tracking
static int8_t gLastDir = 0;  // -1, 0, +1
static bool gDirJustChanged = false;

// Soft ramp parameters
static float runSpeedSps = DEFAULT_STEP_SPEED_SPS;
static float jogSpeedSps = HOMING_SPEED_SPS;
static float rampStartSps = 900.0f;
static float rampAccelSps2 = 6000.0f;
static uint32_t rampLastUs = 0;
static float rampCurSps = 900.0f;
static int32_t slowZoneSteps = 200;
static float slowZoneSps = 1000.0f;

// Enable/disable state
static const bool STEPPER_DIR_INVERT = false;
volatile bool gStepEn = false;  // Non-static so web_server can access it
static bool gStepperAutoDisabled = false;

// Speed-based disable
static const float SPEED_DISABLE_MPH = 2.0f;
static const float SPEED_ENABLE_MPH = 2.3f;
static const uint32_t SPEED_HOLDOFF_MS = 800;
static uint32_t gBelowSpeedSinceMs = 0;

// Thermal idle disable
static const int32_t STEP_ON_DEADBAND_LOG = 12;
static const int32_t STEP_OFF_DEADBAND_LOG = 6;
static const uint32_t STEP_IDLE_OFF_MS = 1500;
static uint32_t gSettledSinceMs = 0;

// Limit switch debounce
static const uint32_t LIMIT_DEBOUNCE_MS = 8;
static uint8_t gLimitStable = 1;
static uint8_t gLimitRawLast = 1;
static uint32_t gLimitLastChangeMs = 0;

// Homing state
static uint32_t gLastLimitTripMs = 0;
static const uint32_t REHOME_COOLDOWN_MS = 2000;

// ==================== HELPER FUNCTIONS ====================

static inline int32_t iabs32(int32_t v) {
  return (v < 0) ? -v : v;
}

static inline uint32_t spsToIntervalUs(float sps) {
  if (sps < 50.0f) sps = 50.0f;
  if (sps > 5000.0f) sps = 5000.0f;
  return (uint32_t)(1000000.0f / sps + 0.5f);
}

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

static inline bool limitPressed() {
  return gLimitStable == 0;
}

static inline void stepperPulseOnce() {
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(stepIntervalUs);
}

static inline void stepperSetDir(bool forward) {
  bool pinLevel = forward;
  if (STEPPER_DIR_INVERT) pinLevel = !pinLevel;
  digitalWrite(DIR_PIN, pinLevel ? HIGH : LOW);
}

static void stepperSetSpeed(float sps) {
  if (sps < 50) sps = 50;
  if (sps > 4000) sps = 4000;
  
  runSpeedSps = sps;
  stepSpeedSps = sps;
  stepIntervalUs = spsToIntervalUs(sps);
  
  Serial.printf("[STEP] speed=%.1f sps interval=%lu us\n", runSpeedSps, (unsigned long)stepIntervalUs);
}

static void stepperSetJogSpeed() {
  stepperSetSpeed(jogSpeedSps);
}

static void stepperSetRunSpeed() {
  stepperSetSpeed(runSpeedSps);
}

static void updateStepperEnableFromError() {
  // Safety: always energized while homing or when rehome is pending
  if (gIsHoming || gRehomeRequested) {
    if (!gStepEn) stepperEnable(true);
    gSettledSinceMs = 0;
    return;
  }
  
  const uint32_t now = millis();
  const int32_t err = iabs32(logStepTarget - logStepPos);
  
  if (!gStepEn) {
    // Currently disabled: enable when error is large
    if (err >= STEP_ON_DEADBAND_LOG) {
      stepperEnable(true);
      gSettledSinceMs = 0;
    }
    return;
  }
  
  // Currently enabled: start settle timer when within OFF band
  if (err <= STEP_OFF_DEADBAND_LOG) {
    if (gSettledSinceMs == 0) gSettledSinceMs = now;
    
    if ((now - gSettledSinceMs) >= STEP_IDLE_OFF_MS) {
      stepperEnable(false);
      gSettledSinceMs = 0;
    }
  } else {
    gSettledSinceMs = 0;
  }
}

// ==================== PUBLIC FUNCTIONS ====================

void stepperInit() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ENABLE_PIN, HIGH);  // Disabled
  
  stepperSetRunSpeed();
  
  Serial.println("✓ Stepper initialized");
}

void stepperEnable(bool en) {
  gStepEn = en;
  digitalWrite(ENABLE_PIN, en ? LOW : HIGH);
  
  if (en) {
    // Reset ramp timing for gentle start
    gLastDir = 0;
    rampCurSps = min(rampStartSps, runSpeedSps);
    rampLastUs = micros();
    lastStepUs = 0;
  }
  
  Serial.printf("[STEP] enable=%s\n", en ? "ON" : "OFF");
}

void stepperUpdate() {
  updateLimitDebounce();
  updateStepperEnableFromError();
  
  if (!gStepEn) return;
  
  // Calculate error
  int32_t err = logStepTarget - logStepPos;
  if (err == 0) return;
  
  // Determine direction
  int8_t curDir = (err > 0) ? 1 : -1;
  
  // Check for direction change
  if (curDir != gLastDir && gLastDir != 0) {
    gDirJustChanged = true;
    rampCurSps = rampStartSps;
    rampLastUs = micros();
  }
  gLastDir = curDir;
  
  // Set direction
  stepperSetDir(curDir > 0);
  
  // Determine speed (with ramp and slow zone)
  float targetSps = runSpeedSps;
  
  // Slow down near target
  int32_t absErr = iabs32(err);
  if (absErr <= slowZoneSteps) {
    targetSps = slowZoneSps;
  }
  
  // Soft ramp after direction change
  if (gDirJustChanged || rampCurSps < targetSps) {
    uint32_t now = micros();
    float dt = (now - rampLastUs) / 1000000.0f;
    rampLastUs = now;
    
    rampCurSps += rampAccelSps2 * dt;
    if (rampCurSps >= targetSps) {
      rampCurSps = targetSps;
      gDirJustChanged = false;
    }
    
    stepIntervalUs = spsToIntervalUs(rampCurSps);
  }
  
  // Step if interval elapsed
  uint32_t nowUs = micros();
  if (lastStepUs == 0 || (nowUs - lastStepUs) >= stepIntervalUs) {
    lastStepUs = nowUs;
    
    // Take one step (non-blocking - just pulse)
    digitalWrite(STEP_PIN, HIGH);
    // Don't use delayMicroseconds here - let it pulse naturally
    digitalWrite(STEP_PIN, LOW);
    
    // Update position
    if (curDir > 0) {
      physStepPos++;
    } else {
      physStepPos--;
    }
    logStepPos = stepsToLogical(physStepPos);
    
    // Constrain to limits
    if (physStepPos < PHYS_MIN_STEPS) physStepPos = PHYS_MIN_STEPS;
    if (physStepPos > PHYS_MAX_STEPS) physStepPos = PHYS_MAX_STEPS;
  }
}

void stepperSetTarget(int32_t logicalTarget) {
  // Clamp to logical limits
  if (logicalTarget < LOGICAL_MIN) logicalTarget = LOGICAL_MIN;
  if (logicalTarget > LOGICAL_MAX) logicalTarget = LOGICAL_MAX;
  
  logStepTarget = logicalTarget;
  physStepTarget = logicalToSteps(logicalTarget);
}

void stepperHome() {
  Serial.println("[HOME] Starting homing...");
  gIsHoming = true;
  stepperEnable(true);
  stepperSetJogSpeed();
  
  // Settle debounce
  for (int i = 0; i < 30; i++) {
    updateLimitDebounce();
    delay(2);
  }
  
  // If switch already pressed, back off
  if (limitPressed()) {
    Serial.println("[HOME] Switch active; backing off...");
    stepperSetDir(true);  // Move away from switch
    uint32_t t0 = millis();
    while (limitPressed() && (millis() - t0) < 2000) {
      stepperPulseOnce();
      updateLimitDebounce();
      delay(1);
    }
  }
  
  // Seek switch
  Serial.println("[HOME] Seeking switch...");
  stepperSetDir(false);  // Move toward switch
  uint32_t t0 = millis();
  while (!limitPressed() && (millis() - t0) < 10000) {
    stepperPulseOnce();
    updateLimitDebounce();
    delay(1);
  }
  
  if (!limitPressed()) {
    Serial.println("[HOME] FAILED - timeout");
    gIsHoming = false;
    return;
  }
  
  // Back off slightly
  Serial.println("[HOME] Backing off from switch...");
  stepperSetDir(true);
  for (int i = 0; i < 100; i++) {
    stepperPulseOnce();
  }
  
  // Set zero position
  physStepPos = PHYS_MIN_STEPS;
  logStepPos = stepsToLogical(physStepPos);
  logStepTarget = logStepPos;
  physStepTarget = physStepPos;
  
  stepperSetRunSpeed();
  gIsHoming = false;
  gRehomeRequested = false;
  
  Serial.println("[HOME] Complete");
  Serial.printf("  Position: phys=%ld log=%ld\n", physStepPos, logStepPos);
}

// ==================== CONVERSION FUNCTIONS ====================

int32_t logicalToSteps(int32_t logical) {
  logical = constrain(logical, LOGICAL_MIN, LOGICAL_MAX);
  return (int32_t)((int64_t)logical * PHYS_MAX_STEPS / LOGICAL_MAX);
}

int32_t stepsToLogical(int32_t steps) {
  steps = constrain(steps, PHYS_MIN_STEPS, PHYS_MAX_STEPS);
  return (int32_t)((int64_t)steps * LOGICAL_MAX / PHYS_MAX_STEPS);
}
