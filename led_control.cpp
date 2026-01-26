/*
 * led_control.cpp - LED Status Indicator Implementation
 */

#include "led_control.h"
#include "stepper_control.h"
#include "ble_trainer.h"
#include "web_server.h"

// ==================== LED STATE ====================
static LedPattern gLedPattern = LED_OFF;
static bool gLedOn = false;
static uint32_t gLedNextMs = 0;
static uint8_t gLedPhase = 0;

// ==================== HELPER FUNCTIONS ====================

static inline void ledWrite(bool on) {
  digitalWrite(LED_PIN, (on ^ !LED_ACTIVE_HIGH) ? HIGH : LOW);
}

// ==================== PUBLIC FUNCTIONS ====================

void ledInit() {
  pinMode(LED_PIN, OUTPUT);
  ledWrite(false);
  Serial.println("✓ LED initialized");
}

void ledSetPattern(LedPattern p) {
  if (p == gLedPattern) return;
  gLedPattern = p;
  gLedPhase = 0;
  gLedOn = false;
  gLedNextMs = 0;
  ledWrite(false);
}

// Forward declaration
static void ledSelectPattern();

void ledUpdate() {
  // Update pattern based on system state
  ledSelectPattern();
  
  // Execute current pattern
  const uint32_t now = millis();
  if (gLedNextMs != 0 && (int32_t)(now - gLedNextMs) < 0) return;
  
  switch (gLedPattern) {
    case LED_OFF:
      ledWrite(false);
      gLedNextMs = now + 500;
      break;
      
    case LED_SOLID:
      ledWrite(true);
      gLedNextMs = now + 500;
      break;
      
    case LED_BLINK_SLOW:  // 1 Hz (500/500)
      gLedOn = !gLedOn;
      ledWrite(gLedOn);
      gLedNextMs = now + 500;
      break;
      
    case LED_BLINK_MED:  // 2 Hz (250/250)
      gLedOn = !gLedOn;
      ledWrite(gLedOn);
      gLedNextMs = now + 250;
      break;
      
    case LED_BLINK_FAST:  // 10 Hz (50/50)
      gLedOn = !gLedOn;
      ledWrite(gLedOn);
      gLedNextMs = now + 50;
      break;
      
    case LED_HEARTBEAT:
      // Short ON (80ms) then long OFF (920ms)
      if (gLedPhase == 0) {
        ledWrite(true);
        gLedNextMs = now + 80;
        gLedPhase = 1;
      } else {
        ledWrite(false);
        gLedNextMs = now + 920;
        gLedPhase = 0;
      }
      break;
      
    case LED_DOUBLE_BLIP:
      // ON 80, OFF 120, ON 80, OFF 1700
      switch (gLedPhase) {
        case 0:
          ledWrite(true);
          gLedNextMs = now + 80;
          gLedPhase = 1;
          break;
        case 1:
          ledWrite(false);
          gLedNextMs = now + 120;
          gLedPhase = 2;
          break;
        case 2:
          ledWrite(true);
          gLedNextMs = now + 80;
          gLedPhase = 3;
          break;
        default:
          ledWrite(false);
          gLedNextMs = now + 1700;
          gLedPhase = 0;
          break;
      }
      break;
      
    case LED_TRIPLE_BLIP:
      // ON 80, OFF 120, ON 80, OFF 120, ON 80, OFF 1500
      switch (gLedPhase) {
        case 0:
          ledWrite(true);
          gLedNextMs = now + 80;
          gLedPhase = 1;
          break;
        case 1:
          ledWrite(false);
          gLedNextMs = now + 120;
          gLedPhase = 2;
          break;
        case 2:
          ledWrite(true);
          gLedNextMs = now + 80;
          gLedPhase = 3;
          break;
        case 3:
          ledWrite(false);
          gLedNextMs = now + 120;
          gLedPhase = 4;
          break;
        case 4:
          ledWrite(true);
          gLedNextMs = now + 80;
          gLedPhase = 5;
          break;
        default:
          ledWrite(false);
          gLedNextMs = now + 1500;
          gLedPhase = 0;
          break;
      }
      break;
  }
}

static void ledSelectPattern() {
  // Highest priority first
  if (gOtaInProgress) {
    ledSetPattern(LED_BLINK_FAST);
    return;
  }
  
  if (otaIsUnlocked()) {
    ledSetPattern(LED_DOUBLE_BLIP);
    return;
  }
  
  if (gRehomeRequested) {
    ledSetPattern(LED_TRIPLE_BLIP);
    return;
  }
  
  if (gIsHoming) {
    ledSetPattern(LED_BLINK_MED);
    return;
  }
  
  if (deviceConnected) {
    switch (gMode) {
      case MODE_ERG:
        ledSetPattern(LED_SOLID);
        return;
      case MODE_SIM:
        ledSetPattern(LED_BLINK_SLOW);
        return;
      default:
        ledSetPattern(LED_HEARTBEAT);
        return;
    }
  }
  
  // Not connected
  switch (gMode) {
    case MODE_ERG:
      ledSetPattern(LED_SOLID);
      break;
    case MODE_SIM:
      ledSetPattern(LED_BLINK_SLOW);
      break;
    default:
      ledSetPattern(LED_OFF);
      break;
  }
}
