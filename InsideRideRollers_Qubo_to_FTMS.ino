/*
 * InsideRide FTMS Trainer - Main File
 * 
 * Original work: A BLE turbo trainer controller for Zwift
 * Copyright (c) 2020 Peter Everett
 * 
 * Substantial modifications and new code:
 * Copyright (c) 2024–2026 Adam Watson
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 */

#include "config.h"
#include "ble_trainer.h"
#include "stepper_control.h"
#include "led_control.h"
#include "web_server.h"
#include "sensors.h"

// ==================== CONTROL STATE ====================
volatile int16_t ergTargetWatts = 0;        // ERG mode target power
volatile float simGradePercent = 0.0f;      // SIM mode grade
static int32_t targetLogicalPosition = 0;   // Computed target position

// ==================== TIMING ====================
uint32_t lastPowerNotifyMs = 0;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("=====================================");
  Serial.println("   InsideRide FTMS Smart Trainer    ");
  Serial.println("=====================================");
  Serial.printf("Firmware Version: %s\n", FW_VERSION);
  Serial.println();

  // Initialize subsystems
  ledInit();
  stepperInit();
  stepperHome();       // Home stepper on power-up
  sensorsInit();       // Initialize Hall sensor
  bleInit();
  webServerInit();

  Serial.println("✓ System Ready");
  Serial.println("=====================================");
}

// ==================== MAIN LOOP ====================
void loop() {
  // PRIORITY 1: Handle web server requests first (critical for responsiveness)
  webServerUpdate();
  
  // PRIORITY 2: Update LED (fast, non-blocking)
  ledUpdate();
  
  // PRIORITY 3: Update stepper (can be CPU intensive)
  static uint32_t lastStepperUpdate = 0;
  if (millis() - lastStepperUpdate >= 5) {  // Only update every 5ms
    lastStepperUpdate = millis();
    stepperUpdate();
  }
  
  // PRIORITY 4: Read sensors (less frequently)
  static uint32_t lastSensorUpdate = 0;
  if (millis() - lastSensorUpdate >= 100) {  // Only every 100ms
    lastSensorUpdate = millis();
    sensorsUpdate();
  }

  // ==================== ERG/SIM MODE TARGET CALCULATION ====================
  // Update target position based on current mode (20 Hz)
  static uint32_t lastTargetUpdateMs = 0;
  if (millis() - lastTargetUpdateMs >= 50) {
    lastTargetUpdateMs = millis();
    
    switch (gMode) {
      case MODE_ERG: {
        // ERG mode: Calculate position from target watts and current speed
        int32_t pos = (int32_t)lround(stepFromPowerSpeed(currentSpeedMph, ergTargetWatts));
        targetLogicalPosition = constrain(pos, LOGICAL_MIN, LOGICAL_MAX);
        break;
      }
      
      case MODE_SIM: {
        // SIM mode: Calculate position from grade and current speed
        int32_t pos = (int32_t)lround(gradeToSteps(currentSpeedMph, simGradePercent));
        targetLogicalPosition = constrain(pos, LOGICAL_MIN, LOGICAL_MAX);
        break;
      }
      
      case MODE_IDLE:
      default:
        // IDLE mode: Use speed-based resistance curve
        // y = -8.95 + 20.3x - 0.597x^2 + 0.0101x^3
        double speed = constrain(currentSpeedMph, 0.0, 50.0);
        double pos = -8.95 + 20.3 * speed - 0.597 * speed * speed + 0.0101 * speed * speed * speed;
        targetLogicalPosition = constrain((int32_t)lround(pos), LOGICAL_MIN, LOGICAL_MAX);
        break;
    }
    
    // Apply manual override if active
    if (gManualHoldActive) {
      stepperSetTarget(gManualHoldTarget);
    } else {
      stepperSetTarget(targetLogicalPosition);
    }
  }

  // Notify BLE at regular intervals
  if (deviceConnected && millis() - lastPowerNotifyMs >= POWER_NOTIFY_PERIOD_MS) {
    lastPowerNotifyMs = millis();
    bleNotifyPower(currentPowerWatts, currentSpeedMph, currentRPM);

    // Debug - show we're trying to notify
    static uint32_t notifyAttempts = 0;
    notifyAttempts++;
    if (notifyAttempts % 50 == 0) {  // Every 50 notifications (5 seconds at 10Hz)
      Serial.printf("[MAIN] BLE notify attempts: %lu (Power=%.0fW)\n", notifyAttempts, currentPowerWatts);
    }
  }

  // BLE keep-alive: periodically restart advertising to stay discoverable
  bleKeepAlive();

  // NO delay() - let loop run as fast as possible
  yield();  // Just yield to WiFi stack
}

// ==================== BLE CONTROL POINT HANDLERS ====================
// These are called from ble_trainer.cpp when Zwift sends commands

void handleRequestControl() {
  Serial.println("BLE: Request Control");
  // Send success response
  uint8_t response[3] = {0x80, 0x00, 0x01};  // Response, Request Control, Success
  if (pControlPoint) {
    pControlPoint->setValue(response, 3);
    pControlPoint->indicate();
  }
}

void handleResetControl() {
  Serial.println("BLE: Reset");
  gMode = MODE_IDLE;
  uint8_t response[3] = {0x80, 0x01, 0x01};  // Response, Reset, Success
  if (pControlPoint) {
    pControlPoint->setValue(response, 3);
    pControlPoint->indicate();
  }
}

void handleSetTargetPower(uint16_t watts) {
  Serial.printf("BLE: Set Target Power = %d W\n", watts);
  gMode = MODE_ERG;
  ergTargetWatts = watts;
  
  uint8_t response[3] = {0x80, 0x05, 0x01};  // Response, Set Target Power, Success
  if (pControlPoint) {
    pControlPoint->setValue(response, 3);
    pControlPoint->indicate();
  }
}

void handleSetTargetResistance(uint8_t level) {
  Serial.printf("BLE: Set Target Resistance = %d\n", level);
  gMode = MODE_ERG;
  
  // Map resistance level to stepper position
  int32_t target = map(level, 0, 100, LOGICAL_MIN, LOGICAL_MAX);
  stepperSetTarget(target);
  
  uint8_t response[3] = {0x80, 0x04, 0x01};  // Response, Set Target Resistance, Success
  if (pControlPoint) {
    pControlPoint->setValue(response, 3);
    pControlPoint->indicate();
  }
}

void handleSetIndoorBikeSimulation(int16_t windSpeed, int16_t grade, uint8_t crr, uint8_t cw) {
  float gradePercent = grade / 100.0f;
  Serial.printf("BLE: Simulation Mode - Grade: %.1f%%\n", gradePercent);
  
  gMode = MODE_SIM;
  simGradePercent = gradePercent;
  
  uint8_t response[3] = {0x80, 0x11, 0x01};  // Response, Set Indoor Bike Sim, Success
  if (pControlPoint) {
    pControlPoint->setValue(response, 3);
    pControlPoint->indicate();
  }
}

void handleStartResume() {
  Serial.println("BLE: Start/Resume");
  // Could be used to enable stepper or other actions
  
  uint8_t response[3] = {0x80, 0x07, 0x01};  // Response, Start/Resume, Success
  if (pControlPoint) {
    pControlPoint->setValue(response, 3);
    pControlPoint->indicate();
  }
}

void handleStopPause(uint8_t stopType) {
  Serial.printf("BLE: Stop/Pause (type: %d)\n", stopType);
  gMode = MODE_IDLE;
  
  uint8_t response[3] = {0x80, 0x08, 0x01};  // Response, Stop/Pause, Success
  if (pControlPoint) {
    pControlPoint->setValue(response, 3);
    pControlPoint->indicate();
  }
}
