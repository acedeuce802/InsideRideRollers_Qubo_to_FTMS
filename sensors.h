/*
 * sensors.h - Sensor Reading and Power Calculation
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "config.h"

// ==================== SENSOR DATA ====================
extern float currentRPM;
extern float currentSpeedMph;
extern float currentPowerWatts;

// ==================== FUNCTIONS ====================
void sensorsInit();
void sensorsUpdate();

// Hall sensor ISR (must be public for attachInterrupt)
void IRAM_ATTR hallISR();

// Conversion functions
float rpmToMph(float rpm);
float mphToRpm(float mph);

// Power calculation from speed and position
double powerFromSpeedPos(double speedMph, double posLogical);

// ERG mode: Calculate position from target power and current speed
double stepFromPowerSpeed(double speedMph, double targetWatts);

// SIM mode: Calculate position from grade and current speed
double gradeToSteps(double speedMph, double gradePercent);

// ==================== CALIBRATION TABLE ACCESS ====================
// These provide direct access to the calibration tables for the web UI
// Tables are stored in sensors.cpp but accessed via calibration.h functions

// Power table [speed_idx][pos_idx] = watts
extern double gPowerTable[7][5];
extern double gPowerSpeedAxis[7];
extern double gPowerPosAxis[5];

// ERG table [speed_idx][power_idx] = position
extern double gErgTable[7][9];
extern double gErgSpeedAxis[7];
extern double gErgPowerAxis[9];

// SIM table [speed_idx][grade_idx] = position
extern double gSimTable[8][7];
extern double gSimSpeedAxis[8];
extern double gSimGradeAxis[7];

#endif // SENSORS_H
