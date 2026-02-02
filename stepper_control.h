/*
 * stepper_control.h - Stepper Motor Control Interface
 */

#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <Arduino.h>
#include "config.h"

// ==================== CONTROL MODES ====================
enum ControlMode : uint8_t {
  MODE_IDLE = 0,
  MODE_SIM  = 1,   // grade -> steps
  MODE_ERG  = 2    // (speed, targetW) -> steps
};

extern volatile ControlMode gMode;

// ==================== STEPPER STATE ====================
extern volatile int32_t logStepPos;
extern volatile int32_t logStepTarget;
extern volatile int32_t physStepPos;
extern volatile int32_t physStepTarget;
extern volatile bool gStepEn;  // Motor enable state

extern volatile bool gIsHoming;
extern volatile bool gRehomeRequested;
extern volatile bool gManualHoldActive;
extern volatile int32_t gManualHoldTarget;

// ==================== FUNCTIONS ====================
void stepperInit();
void stepperUpdate();
void stepperSetTarget(int32_t logicalTarget);
void stepperHome();
void stepperEnable(bool enable);

// Safety functions
void stepperRequestRehome(const char* reason);
bool stepperLimitPressed();
void stepperUpdateLimitDebounce();
void stepperUpdateSpeedBasedEnable(float speedMph);

// Conversion functions
int32_t logicalToSteps(int32_t logical);
int32_t stepsToLogical(int32_t steps);

#endif // STEPPER_CONTROL_H
