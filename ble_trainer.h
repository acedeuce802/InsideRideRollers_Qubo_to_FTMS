/*
 * ble_trainer.h - BLE FTMS Trainer Interface
 * 
 * Handles all Bluetooth Low Energy communication for FTMS trainer
 * Uses ESP32 BLE library (NOT NimBLE)
 */

#ifndef BLE_TRAINER_H
#define BLE_TRAINER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "config.h"

// ==================== BLE STATE ====================
extern bool deviceConnected;
extern BLECharacteristic* pIndoorBike;
extern BLECharacteristic* pControlPoint;
extern BLECharacteristic* pFeature;
extern BLECharacteristic* pStatus;

// ==================== BLE FUNCTIONS ====================
void bleInit();
void bleNotifyPower(float watts, float speedMph, float cadenceRpm);
void bleNotifyStatus(uint8_t status);
void bleKeepAlive();  // Call periodically to prevent BLE from going dormant

// ==================== CONTROL POINT HANDLERS ====================
// These are called by BLE callbacks when Zwift sends commands
void handleRequestControl();
void handleResetControl();
void handleSetTargetPower(uint16_t watts);
void handleSetTargetResistance(uint8_t level);
void handleSetIndoorBikeSimulation(int16_t windSpeed, int16_t grade, uint8_t crr, uint8_t cw);
void handleStartResume();
void handleStopPause(uint8_t stopType);

#endif // BLE_TRAINER_H
