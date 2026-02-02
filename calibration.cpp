/*
 * calibration.cpp - Calibration and Settings Storage Implementation
 */

#include "calibration.h"
#include <Preferences.h>
#include <nvs_flash.h>

// ==================== GLOBAL CALIBRATION DATA ====================
float gIdleCurveA = IDLE_CURVE_DEFAULT_A;
float gIdleCurveB = IDLE_CURVE_DEFAULT_B;
float gIdleCurveC = IDLE_CURVE_DEFAULT_C;
float gIdleCurveD = IDLE_CURVE_DEFAULT_D;

// ==================== WIFI SETTINGS ====================
char gWifiSsid[64] = "";
char gWifiPass[64] = "";
bool gWifiConfigured = false;

// ==================== NVS STORAGE ====================
static Preferences prefs;
static const char* NVS_NAMESPACE = "calibration";

// ==================== PUBLIC FUNCTIONS ====================

void calibrationInit() {
  Serial.println("[CAL] calibrationInit starting...");

  // Initialize NVS flash if needed
  esp_err_t err = nvs_flash_init();
  Serial.printf("[CAL] nvs_flash_init() returned: %d (ESP_OK=%d)\n", err, ESP_OK);

  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Serial.println("[CAL] NVS partition needs erase, erasing...");
    esp_err_t eraseErr = nvs_flash_erase();
    Serial.printf("[CAL] nvs_flash_erase() returned: %d\n", eraseErr);
    err = nvs_flash_init();
    Serial.printf("[CAL] nvs_flash_init() after erase returned: %d\n", err);
  }

  if (err != ESP_OK) {
    Serial.printf("[CAL] ERROR: NVS flash init failed: %d\n", err);
    // Try one more time with erase
    Serial.println("[CAL] Attempting forced erase and reinit...");
    nvs_flash_erase();
    delay(100);
    err = nvs_flash_init();
    if (err != ESP_OK) {
      Serial.printf("[CAL] ERROR: NVS still failing after forced erase: %d\n", err);
      return;
    }
  }
  Serial.println("[CAL] NVS flash initialized OK");

  // Small delay to ensure NVS is ready
  delay(50);

  // Try to open in read-write mode (creates namespace if it doesn't exist)
  Serial.printf("[CAL] Attempting prefs.begin('%s', false)...\n", NVS_NAMESPACE);
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    Serial.println("[CAL] ERROR: Failed to open NVS namespace!");
    Serial.println("[CAL] Trying to erase and reinitialize NVS completely...");

    // Force complete NVS reset
    nvs_flash_erase();
    delay(100);
    nvs_flash_init();
    delay(50);

    // Try again
    if (!prefs.begin(NVS_NAMESPACE, false)) {
      Serial.println("[CAL] ERROR: Still cannot open NVS namespace after erase!");
      Serial.println("[CAL] Will use default values (not persisted)");
      return;
    }
  }
  Serial.println("[CAL] NVS namespace opened successfully");

  // Load IDLE curve coefficients (use defaults if not found)
  gIdleCurveA = prefs.getFloat("idleA", IDLE_CURVE_DEFAULT_A);
  gIdleCurveB = prefs.getFloat("idleB", IDLE_CURVE_DEFAULT_B);
  gIdleCurveC = prefs.getFloat("idleC", IDLE_CURVE_DEFAULT_C);
  gIdleCurveD = prefs.getFloat("idleD", IDLE_CURVE_DEFAULT_D);

  // Load WiFi settings
  String ssid = prefs.getString("wifiSsid", "");
  String pass = prefs.getString("wifiPass", "");

  Serial.printf("[CAL] Raw NVS read: ssid='%s' (len=%d)\n", ssid.c_str(), ssid.length());

  if (ssid.length() > 0) {
    strncpy(gWifiSsid, ssid.c_str(), sizeof(gWifiSsid) - 1);
    gWifiSsid[sizeof(gWifiSsid) - 1] = '\0';
    strncpy(gWifiPass, pass.c_str(), sizeof(gWifiPass) - 1);
    gWifiPass[sizeof(gWifiPass) - 1] = '\0';
    gWifiConfigured = true;
  } else {
    gWifiSsid[0] = '\0';
    gWifiPass[0] = '\0';
    gWifiConfigured = false;
  }

  prefs.end();

  Serial.println("[CAL] Settings loaded from NVS:");
  Serial.printf("  IDLE curve: %.3f + %.3f*v + %.4f*v^2 + %.5f*v^3\n",
                gIdleCurveA, gIdleCurveB, gIdleCurveC, gIdleCurveD);
  if (gWifiConfigured) {
    Serial.printf("  WiFi SSID: '%s' (configured)\n", gWifiSsid);
  } else {
    Serial.println("  WiFi: Not configured (AP mode only)");
  }
}

void calibrationSave() {
  prefs.begin(NVS_NAMESPACE, false);  // Read-write mode

  prefs.putFloat("idleA", gIdleCurveA);
  prefs.putFloat("idleB", gIdleCurveB);
  prefs.putFloat("idleC", gIdleCurveC);
  prefs.putFloat("idleD", gIdleCurveD);

  prefs.end();

  Serial.println("[CAL] Calibration saved to NVS");
}

void calibrationReset() {
  gIdleCurveA = IDLE_CURVE_DEFAULT_A;
  gIdleCurveB = IDLE_CURVE_DEFAULT_B;
  gIdleCurveC = IDLE_CURVE_DEFAULT_C;
  gIdleCurveD = IDLE_CURVE_DEFAULT_D;

  calibrationSave();

  Serial.println("[CAL] Calibration reset to defaults");
}

void wifiSettingsSave(const char* ssid, const char* pass) {
  Serial.printf("[CAL] wifiSettingsSave called with SSID='%s'\n", ssid);

  if (!prefs.begin(NVS_NAMESPACE, false)) {
    Serial.println("[CAL] ERROR: Failed to open NVS for writing!");
    return;
  }

  size_t written1 = prefs.putString("wifiSsid", ssid);
  size_t written2 = prefs.putString("wifiPass", pass);

  Serial.printf("[CAL] NVS write results: ssid=%d bytes, pass=%d bytes\n", written1, written2);

  prefs.end();

  // Update runtime variables
  strncpy(gWifiSsid, ssid, sizeof(gWifiSsid) - 1);
  gWifiSsid[sizeof(gWifiSsid) - 1] = '\0';  // Ensure null termination
  strncpy(gWifiPass, pass, sizeof(gWifiPass) - 1);
  gWifiPass[sizeof(gWifiPass) - 1] = '\0';  // Ensure null termination
  gWifiConfigured = (strlen(ssid) > 0);

  Serial.printf("[CAL] WiFi settings saved: SSID='%s', configured=%s\n",
                gWifiSsid, gWifiConfigured ? "true" : "false");
}

void wifiSettingsClear() {
  prefs.begin(NVS_NAMESPACE, false);

  prefs.remove("wifiSsid");
  prefs.remove("wifiPass");

  prefs.end();

  gWifiSsid[0] = '\0';
  gWifiPass[0] = '\0';
  gWifiConfigured = false;

  Serial.println("[CAL] WiFi settings cleared");
}

int32_t idlePositionFromSpeed(float speedMph) {
  float speed = constrain(speedMph, 0.0f, 50.0f);
  float pos = gIdleCurveA
            + gIdleCurveB * speed
            + gIdleCurveC * speed * speed
            + gIdleCurveD * speed * speed * speed;
  return constrain((int32_t)lroundf(pos), LOGICAL_MIN, LOGICAL_MAX);
}
