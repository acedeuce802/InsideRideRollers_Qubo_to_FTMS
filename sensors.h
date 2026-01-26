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

#endif // SENSORS_H
