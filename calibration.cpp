/*
 * calibration.cpp - Calibration and Settings Storage Implementation
 */

#include "calibration.h"
#include "sensors.h"
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

  // Load calibration tables (Power, ERG, SIM)
  calibrationTablesLoad();
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

// ==================== CALIBRATION TABLE FUNCTIONS ====================

// Default tables (stored in sensors.cpp as DEFAULT_*_TABLE)
// We need local copies for reset functionality

static const double DEFAULT_POWER_TABLE[7][5] = {
  {   0,   0,   0,   0,     0 },
  {  52,  68,  80, 102,   124 },
  { 117, 143, 217, 280,   343 },
  { 188, 246, 383, 490,   597 },
  { 265, 380, 580, 732,   884 },
  { 349, 544, 806, 1006, 1206 },
  { 861, 1806, 2388, 2856, 3324 }
};

static const double DEFAULT_ERG_TABLE[7][9] = {
  {   0,    0,    0,    0,    0,    0,    0,    0,    0 },
  {   0,  739, 1000, 1000, 1000, 1000, 1000, 1000, 1000 },
  {   0,    0,  212,  442,  651,  841, 1000, 1000, 1000 },
  {   0,    0,    0,   70,  198,  322,  560,  996, 1000 },
  {   0,    0,    0,    0,    0,   79,  238,  552, 1000 },
  {   0,    0,    0,    0,    0,    0,   67,  285,  745 },
  {   0,    0,    0,    0,    0,    0,    0,    0,   26 }
};

static const double DEFAULT_SIM_TABLE[8][7] = {
  {   0,  167,  333,  500,  667,  833, 1000 },
  {   0,  167,  333,  500,  667,  833, 1000 },
  {   0,  167,  333,  500,  667,  833, 1000 },
  {   0,  167,  333,  500,  667,  833, 1000 },
  {   0,  167,  333,  500,  667,  833, 1000 },
  { 167,  333,  500,  677,  834, 1000, 1000 },
  { 333,  500,  677,  834, 1000, 1000, 1000 },
  { 500,  500,  677,  834, 1000, 1000, 1000 }
};

// ==================== POWER TABLE ====================

void powerTableSave() {
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    Serial.println("[CAL] ERROR: Failed to open NVS for power table save");
    return;
  }

  // Store as blob
  prefs.putBytes("powerTbl", gPowerTable, sizeof(gPowerTable));
  prefs.end();

  Serial.println("[CAL] Power table saved to NVS");
}

void powerTableReset() {
  for (int i = 0; i < POWER_TABLE_ROWS; i++) {
    for (int j = 0; j < POWER_TABLE_COLS; j++) {
      gPowerTable[i][j] = DEFAULT_POWER_TABLE[i][j];
    }
  }
  powerTableSave();
  Serial.println("[CAL] Power table reset to defaults");
}

void powerTableSet(int row, int col, double value) {
  if (row >= 0 && row < POWER_TABLE_ROWS && col >= 0 && col < POWER_TABLE_COLS) {
    gPowerTable[row][col] = value;
  }
}

double powerTableGet(int row, int col) {
  if (row >= 0 && row < POWER_TABLE_ROWS && col >= 0 && col < POWER_TABLE_COLS) {
    return gPowerTable[row][col];
  }
  return 0;
}

// ==================== ERG TABLE ====================

void ergTableSave() {
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    Serial.println("[CAL] ERROR: Failed to open NVS for ERG table save");
    return;
  }

  prefs.putBytes("ergTbl", gErgTable, sizeof(gErgTable));
  prefs.end();

  Serial.println("[CAL] ERG table saved to NVS");
}

void ergTableReset() {
  for (int i = 0; i < ERG_TABLE_ROWS; i++) {
    for (int j = 0; j < ERG_TABLE_COLS; j++) {
      gErgTable[i][j] = DEFAULT_ERG_TABLE[i][j];
    }
  }
  ergTableSave();
  Serial.println("[CAL] ERG table reset to defaults");
}

void ergTableSet(int row, int col, double value) {
  if (row >= 0 && row < ERG_TABLE_ROWS && col >= 0 && col < ERG_TABLE_COLS) {
    gErgTable[row][col] = value;
  }
}

double ergTableGet(int row, int col) {
  if (row >= 0 && row < ERG_TABLE_ROWS && col >= 0 && col < ERG_TABLE_COLS) {
    return gErgTable[row][col];
  }
  return 0;
}

// ==================== SIM TABLE ====================

void simTableSave() {
  if (!prefs.begin(NVS_NAMESPACE, false)) {
    Serial.println("[CAL] ERROR: Failed to open NVS for SIM table save");
    return;
  }

  prefs.putBytes("simTbl", gSimTable, sizeof(gSimTable));
  prefs.end();

  Serial.println("[CAL] SIM table saved to NVS");
}

void simTableReset() {
  for (int i = 0; i < SIM_TABLE_ROWS; i++) {
    for (int j = 0; j < SIM_TABLE_COLS; j++) {
      gSimTable[i][j] = DEFAULT_SIM_TABLE[i][j];
    }
  }
  simTableSave();
  Serial.println("[CAL] SIM table reset to defaults");
}

void simTableSet(int row, int col, double value) {
  if (row >= 0 && row < SIM_TABLE_ROWS && col >= 0 && col < SIM_TABLE_COLS) {
    gSimTable[row][col] = value;
  }
}

double simTableGet(int row, int col) {
  if (row >= 0 && row < SIM_TABLE_ROWS && col >= 0 && col < SIM_TABLE_COLS) {
    return gSimTable[row][col];
  }
  return 0;
}

// ==================== AXIS ACCESSORS ====================

double powerSpeedAxis(int idx) {
  if (idx >= 0 && idx < POWER_TABLE_ROWS) return gPowerSpeedAxis[idx];
  return 0;
}

double powerPosAxis(int idx) {
  if (idx >= 0 && idx < POWER_TABLE_COLS) return gPowerPosAxis[idx];
  return 0;
}

double ergSpeedAxis(int idx) {
  if (idx >= 0 && idx < ERG_TABLE_ROWS) return gErgSpeedAxis[idx];
  return 0;
}

double ergPowerAxis(int idx) {
  if (idx >= 0 && idx < ERG_TABLE_COLS) return gErgPowerAxis[idx];
  return 0;
}

double simSpeedAxis(int idx) {
  if (idx >= 0 && idx < SIM_TABLE_ROWS) return gSimSpeedAxis[idx];
  return 0;
}

double simGradeAxis(int idx) {
  if (idx >= 0 && idx < SIM_TABLE_COLS) return gSimGradeAxis[idx];
  return 0;
}

// ==================== LOAD TABLES FROM NVS ====================

void calibrationTablesLoad() {
  if (!prefs.begin(NVS_NAMESPACE, true)) {  // Read-only
    Serial.println("[CAL] No saved calibration tables found, using defaults");
    return;
  }

  // Load power table if it exists
  size_t len = prefs.getBytesLength("powerTbl");
  if (len == sizeof(gPowerTable)) {
    prefs.getBytes("powerTbl", gPowerTable, sizeof(gPowerTable));
    Serial.println("[CAL] Power table loaded from NVS");
  }

  // Load ERG table if it exists
  len = prefs.getBytesLength("ergTbl");
  if (len == sizeof(gErgTable)) {
    prefs.getBytes("ergTbl", gErgTable, sizeof(gErgTable));
    Serial.println("[CAL] ERG table loaded from NVS");
  }

  // Load SIM table if it exists
  len = prefs.getBytesLength("simTbl");
  if (len == sizeof(gSimTable)) {
    prefs.getBytes("simTbl", gSimTable, sizeof(gSimTable));
    Serial.println("[CAL] SIM table loaded from NVS");
  }

  prefs.end();
}
