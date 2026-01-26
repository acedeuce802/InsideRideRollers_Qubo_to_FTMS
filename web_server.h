/*
 * web_server.h - WiFi AP and Web Server Interface
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include "config.h"
#include "stepper_control.h"  // For ControlMode, logStepPos, logStepTarget, gStepEn

// ==================== OTA STATE ====================
extern volatile bool gOtaInProgress;
extern volatile bool gOtaDone;
extern volatile bool gOtaOk;
extern String gOtaErr;

// ==================== EXTERNAL VARIABLES FOR DIAGNOSTICS ====================
// Variables from main .ino file
extern volatile int16_t ergTargetWatts;
extern volatile float simGradePercent;
extern float currentSpeedMph;
extern float currentPowerWatts;
extern bool deviceConnected;

// Control state from main .ino
extern volatile ControlMode gMode;
extern volatile bool gManualHoldActive;
extern volatile int32_t gManualHoldTarget;

// ==================== FUNCTIONS ====================
void webServerInit();
void webServerUpdate();
bool otaIsUnlocked();
void otaUnlock();

#endif // WEB_SERVER_H
