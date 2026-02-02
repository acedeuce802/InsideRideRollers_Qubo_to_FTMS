/*
 * calibration.h - Calibration and Settings Storage
 *
 * Stores user-adjustable calibration values and WiFi settings in non-volatile storage
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include "config.h"

// ==================== IDLE CURVE COEFFICIENTS ====================
// Cubic polynomial: pos = a + b*speed + c*speed^2 + d*speed^3
extern float gIdleCurveA;
extern float gIdleCurveB;
extern float gIdleCurveC;
extern float gIdleCurveD;

// ==================== WIFI SETTINGS ====================
extern char gWifiSsid[64];
extern char gWifiPass[64];
extern bool gWifiConfigured;  // true if user has saved WiFi settings

// ==================== FUNCTIONS ====================
void calibrationInit();         // Load from NVS (call in setup)
void calibrationSave();         // Save calibration to NVS
void calibrationReset();        // Reset calibration to defaults

// WiFi settings
void wifiSettingsSave(const char* ssid, const char* pass);
void wifiSettingsClear();       // Clear saved WiFi (revert to AP-only mode)

// Calculate IDLE position from speed using current coefficients
int32_t idlePositionFromSpeed(float speedMph);

#endif // CALIBRATION_H
