/*
 * led_control.h - LED Status Indicator Control
 */

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>
#include "config.h"

// ==================== LED PATTERNS ====================
enum LedPattern : uint8_t {
  LED_OFF = 0,
  LED_SOLID,
  LED_BLINK_SLOW,
  LED_BLINK_MED,
  LED_BLINK_FAST,
  LED_HEARTBEAT,
  LED_DOUBLE_BLIP,
  LED_TRIPLE_BLIP
};

// ==================== FUNCTIONS ====================
void ledInit();
void ledUpdate();
void ledSetPattern(LedPattern pattern);

#endif // LED_CONTROL_H
