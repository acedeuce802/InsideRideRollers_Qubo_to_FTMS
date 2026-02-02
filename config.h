/*
 * config.h - Configuration and Constants
 * 
 * All user-configurable settings and hardware definitions
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==================== VERSION ====================
static const char* FW_VERSION = "2026-02_01_02";

// ==================== ZWIFT SPOOFING ====================
// CRITICAL: Zwift checks BOTH advertising name AND manufacturer
// If you change these, Zwift may not accept the trainer when power meter is connected

// Advertising name (what users see in BLE scans)
static const char* BLE_DEVICE_NAME = "KICKR CORE A1B2";  // Matches working code

// Device Information Service (what Zwift checks for whitelist)
static const char* BLE_MANUFACTURER = "Wahoo Fitness";
static const char* BLE_MODEL = "KICKR";
static const char* BLE_FIRMWARE_VERSION = "4.10.0";

// LEGAL NOTE: Spoofing Wahoo identity may have legal implications.
// For commercial use, contact Zwift to whitelist your own manufacturer.
// For personal use, understand the risks involved.

// To use your own branding (may not work with power meter connected):
// static const char* BLE_DEVICE_NAME = "InsideRideFTMS";
// static const char* BLE_MANUFACTURER = "InsideRide";
// static const char* BLE_MODEL = "Smart Trainer";

// ==================== WIFI & WEB SERVER ====================
static const char* AP_SSID = "InsideRideCal";
static const char* AP_PASS = "insideride"; // >= 8 chars

// OTA Authentication
static const char* OTA_USER = "admin";
static const char* OTA_PASS = "change-me";

// OTA Settings
static constexpr uint32_t OTA_UNLOCK_WINDOW_MS = 60 * 1000; // 60 seconds
static constexpr bool OTA_DENY_WHEN_BLE_CONNECTED = true;
static constexpr bool OTA_REQUIRE_LOW_SPEED = true;
static constexpr float OTA_MAX_SPEED_MPH = 1.0f;

// ==================== HARDWARE PINS ====================
static const int STEP_PIN = D3;
static const int DIR_PIN = D4;
static const int ENABLE_PIN = D5;
static const int LIMIT_PIN = D6;    // INPUT_PULLUP, switch -> GND
static const int HALL_PIN = D7;     // INPUT_PULLUP, interrupt
static const int CAL_BTN_PIN = D8;  // INPUT_PULLUP, momentary -> GND
static const int LED_PIN = D9;

// LED Configuration
static const bool LED_ACTIVE_HIGH = true;  // false if LED sinks to GND

// ==================== STEPPER MOTOR ====================
// Logical travel (engineering units)
static const int32_t LOGICAL_MIN = 0;
static const int32_t LOGICAL_MAX = 1000;

// Physical travel (microsteps)
static const int32_t PHYS_MIN_STEPS = 0;
static const int32_t PHYS_MAX_STEPS = 6960;   // 870 * 8

// Stepper speeds
static const float DEFAULT_STEP_SPEED_SPS = 2500.0f;
static const float HOMING_SPEED_SPS = 800.0f;

// ==================== SIMULATION ====================
static const int UPPER_INCLINE_CLAMP = 10;
static const int LOWER_INCLINE_CLAMP = -5;

// Grid size for power/speed lookup
static constexpr int MPH_GRID_SIZE = 6;
static constexpr int POS_GRID_SIZE = 5;

// ==================== BLE TIMING ====================
static constexpr uint32_t POWER_NOTIFY_PERIOD_MS = 100;  // 10 Hz (matches original)

// ==================== HALL SENSOR / RPM ====================
static const uint8_t HALL_PULSES_PER_REV = 6;      // Number of magnets
static constexpr float ROLLER_DIAMETER_IN = 3.25f;  // Roller diameter in inches

#endif // CONFIG_H
