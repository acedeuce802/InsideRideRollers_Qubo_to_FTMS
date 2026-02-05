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

// ==================== DEVICE IDENTITY ====================
extern char gDeviceId[8];       // Last 4 hex digits of MAC (e.g., "A1B2")
extern char gDeviceName[32];    // User-friendly name (e.g., "Trainer1")
extern bool gDeviceNameSet;     // true if user has set a custom name

// ==================== CALIBRATION TABLE DIMENSIONS ====================
// Power table: speed (7) x position (5) -> watts
static const int POWER_TABLE_ROWS = 7;   // Speed breakpoints
static const int POWER_TABLE_COLS = 5;   // Position breakpoints

// ERG table: speed (7) x power (9) -> position
static const int ERG_TABLE_ROWS = 7;     // Speed breakpoints
static const int ERG_TABLE_COLS = 9;     // Power breakpoints

// SIM table: speed (8) x grade (7) -> position
static const int SIM_TABLE_ROWS = 8;     // Speed breakpoints
static const int SIM_TABLE_COLS = 7;     // Grade breakpoints

// ==================== FUNCTIONS ====================
void calibrationInit();         // Load from NVS (call in setup)
void calibrationSave();         // Save IDLE curve calibration to NVS
void calibrationReset();        // Reset IDLE curve to defaults
void calibrationTablesLoad();   // Load calibration tables from NVS

// WiFi settings
void wifiSettingsSave(const char* ssid, const char* pass);
void wifiSettingsClear();       // Clear saved WiFi (revert to AP-only mode)

// Device identity
void deviceNameSave(const char* name);  // Save custom device name
void deviceNameClear();                 // Clear custom name (use default based on MAC)
const char* getEffectiveHostname();     // Get hostname for mDNS (custom name or "insideride-XXXX")
const char* getEffectiveApSsid();       // Get AP SSID (custom name or "InsideRide-XXXX")

// Calculate IDLE position from speed using current coefficients
int32_t idlePositionFromSpeed(float speedMph);

// ==================== CALIBRATION TABLE FUNCTIONS ====================
// Power table (speed x position -> watts)
void powerTableSave();
void powerTableReset();
void powerTableSet(int row, int col, double value);
double powerTableGet(int row, int col);

// ERG table (speed x power -> position)
void ergTableSave();
void ergTableReset();
void ergTableSet(int row, int col, double value);
double ergTableGet(int row, int col);

// SIM table (speed x grade -> position)
void simTableSave();
void simTableReset();
void simTableSet(int row, int col, double value);
double simTableGet(int row, int col);

// Get axis values (read-only, fixed)
double powerSpeedAxis(int idx);    // X axis for power table
double powerPosAxis(int idx);      // Y axis for power table
double ergSpeedAxis(int idx);      // X axis for ERG table
double ergPowerAxis(int idx);      // Y axis for ERG table
double simSpeedAxis(int idx);      // X axis for SIM table
double simGradeAxis(int idx);      // Y axis for SIM table

#endif // CALIBRATION_H
