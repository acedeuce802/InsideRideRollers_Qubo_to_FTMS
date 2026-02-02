/*
 * ble_trainer.cpp - BLE FTMS Trainer Implementation (ESP32 BLE - NOT NimBLE)
 */

#include "ble_trainer.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ==================== BLE GLOBAL STATE ====================
bool deviceConnected = false;
BLEServer* pServer = NULL;
BLECharacteristic* pIndoorBike = NULL;
BLECharacteristic* pControlPoint = NULL;
BLECharacteristic* pFeature = NULL;
BLECharacteristic* pStatus = NULL;

// Indoor Bike Data flags
static constexpr uint16_t IB_FLAG_INSTANT_CADENCE = (1 << 2);
static constexpr uint16_t IB_FLAG_INSTANT_POWER = (1 << 6);
static constexpr uint16_t FTMS_INDOOR_BIKE_FLAGS = (IB_FLAG_INSTANT_CADENCE | IB_FLAG_INSTANT_POWER);

static uint8_t FTMS_INDOOR_BIKE_DATA[10] = {
  (uint8_t)(FTMS_INDOOR_BIKE_FLAGS & 0xFF),
  (uint8_t)(FTMS_INDOOR_BIKE_FLAGS >> 8),
  0x00, 0x00,  // Speed (not used)
  0x00, 0x00,  // Cadence
  0x00, 0x00,  // Power
  0x00, 0x00   // Padding
};

// ==================== BLE CALLBACKS ====================

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE Client connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Client disconnected");
    // Restart advertising
    BLEDevice::getAdvertising()->start();
  }
};

class ControlPointCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    Serial.println("[BLE CP] *** onWrite() CALLED ***");
    
    String value = pCharacteristic->getValue();  // NOT .c_str()!
    
    if (value.length() < 1) return;
    
    uint8_t opcode = value[0];
    
    Serial.printf("[BLE CP] Opcode 0x%02X received, length=%d, bytes: ", opcode, value.length());
    for (int i = 0; i < value.length() && i < 12; i++) {
      Serial.printf("%02X ", (uint8_t)value[i]);
    }
    Serial.println();
    
    switch (opcode) {
      case 0x00:  // Request Control
        Serial.println("[BLE CP] -> Request Control");
        handleRequestControl();
        break;
        
      case 0x01:  // Reset
        Serial.println("[BLE CP] -> Reset");
        handleResetControl();
        break;
        
      case 0x05:  // Set Target Power
        if (value.length() >= 3) {
          uint16_t watts = (uint8_t)value[1] | ((uint8_t)value[2] << 8);
          Serial.printf("[BLE CP] -> Set Target Power: %d W\n", watts);
          handleSetTargetPower(watts);
        }
        break;
        
      case 0x04:  // Set Target Resistance Level
        if (value.length() >= 2) {
          uint8_t level = value[1];
          Serial.printf("[BLE CP] -> Set Target Resistance: %d\n", level);
          handleSetTargetResistance(level);
        }
        break;
        
      case 0x11:  // Set Indoor Bike Simulation Parameters
        if (value.length() >= 7) {
          int16_t windSpeed = (uint8_t)value[1] | ((uint8_t)value[2] << 8);
          int16_t grade = (uint8_t)value[3] | ((uint8_t)value[4] << 8);
          uint8_t crr = value[5];
          uint8_t cw = value[6];
          float gradePercent = grade / 100.0f;
          Serial.printf("[BLE CP] -> Simulation: Grade=%.2f%%, Wind=%d\n", gradePercent, windSpeed);
          handleSetIndoorBikeSimulation(windSpeed, grade, crr, cw);
        } else {
          Serial.printf("[BLE CP] -> Simulation command too short! len=%d\n", value.length());
        }
        break;
        
      case 0x07:  // Start or Resume
        Serial.println("[BLE CP] -> Start/Resume");
        handleStartResume();
        break;
        
      case 0x08:  // Stop or Pause
        if (value.length() >= 2) {
          uint8_t stopType = value[1];
          Serial.printf("[BLE CP] -> Stop/Pause: type=%d\n", stopType);
          handleStopPause(stopType);
        }
        break;
        
      default:
        Serial.printf("[BLE CP] -> UNHANDLED opcode: 0x%02X\n", opcode);
        break;
    }
  }
};

// ==================== BLE INITIALIZATION ====================

void bleInit() {
  Serial.println("===========================================");
  Serial.println("Initializing BLE (ESP32 BLE)...");

  // Initialize BLE with Wahoo name
  BLEDevice::init(BLE_DEVICE_NAME);
  
  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  Serial.println("  BLE Server created");

  // ===== Create Services First (before adding characteristics) =====
  // FTMS service needs more handles: 4 characteristics × 3 handles each + 1 service = 13
  // Using 15 to be safe (default is often too small and causes crashes)
  BLEService* pService = pServer->createService(BLEUUID((uint16_t)0x1826), 15);  // FTMS
  BLEService* pDis = pServer->createService(BLEUUID((uint16_t)0x180A), 10);      // Device Info

  // ===== Device Information Service Characteristics =====
  BLECharacteristic* chMan = pDis->createCharacteristic(
    BLEUUID((uint16_t)0x2A29), 
    BLECharacteristic::PROPERTY_READ
  );
  chMan->setValue(BLE_MANUFACTURER);  // "Wahoo Fitness"
  
  BLECharacteristic* chModel = pDis->createCharacteristic(
    BLEUUID((uint16_t)0x2A24), 
    BLECharacteristic::PROPERTY_READ
  );
  chModel->setValue(BLE_MODEL);  // "KICKR"
  
  BLECharacteristic* chFw = pDis->createCharacteristic(
    BLEUUID((uint16_t)0x2A26), 
    BLECharacteristic::PROPERTY_READ
  );
  chFw->setValue(BLE_FIRMWARE_VERSION);  // "4.10.0"
  
  Serial.println("  Device Info Service configured (Wahoo spoofing)");

  // ===== FTMS Service Characteristics =====
  Serial.println("  Creating FTMS characteristics...");
  
  // Indoor Bike Data (0x2AD2)
  pIndoorBike = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2AD2),
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pIndoorBike->addDescriptor(new BLE2902());

  // Fitness Machine Control Point (0x2AD9)
  pControlPoint = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2AD9),
    BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_WRITE
  );
  pControlPoint->addDescriptor(new BLE2902());

  // Fitness Machine Feature (0x2ACC)
  pFeature = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2ACC),
    BLECharacteristic::PROPERTY_READ
  );
  // NOTE: Working code does NOT set value - leave default/empty
  pFeature->addDescriptor(new BLE2902());

  // Training Status (0x2ADA)
  pStatus = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2ADA),
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pStatus->addDescriptor(new BLE2902());
  
  // Set callbacks AFTER all characteristics created (matches working code)
  static ControlPointCallbacks cpCallbacks;
  pControlPoint->setCallbacks(&cpCallbacks);

  Serial.println("  FTMS Service configured");

  // Start FTMS service first
  Serial.println("  Starting FTMS service...");
  pService->start();
  delay(100);  // Allow BLE stack to stabilize
  Serial.println("  FTMS service started");

  // ===== Start Advertising =====
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->stop();
  
  // Set advertising data
  BLEAdvertisementData advData;
  advData.setFlags(0x06);
  advData.addData(String("\x03\x03\x26\x18", 4));  // FTMS service UUID
  
  BLEAdvertisementData scanData;
  scanData.setName(BLE_DEVICE_NAME);
  
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->setScanResponseData(scanData);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  
  pAdvertising->start();
  delay(50);  // Allow advertising to stabilize

  // Start Device Info Service AFTER advertising (matches working code)
  Serial.println("  Starting Device Info service...");
  pDis->start();
  delay(50);  // Allow service to stabilize

  Serial.println("✓ BLE Started (ESP32 BLE)");
  Serial.printf("  Device Name: %s\n", BLE_DEVICE_NAME);
  Serial.printf("  Manufacturer: %s (for Zwift whitelist)\n", BLE_MANUFACTURER);
  Serial.printf("  Model: %s\n", BLE_MODEL);
  Serial.println("===========================================");
}

// ==================== BLE NOTIFICATION FUNCTIONS ====================

void bleNotifyPower(float watts, float speedMph, float cadenceRpm) {
  if (!deviceConnected || pIndoorBike == NULL) {
    return;
  }

  // Cadence: 0 below 2 mph, 90 rpm above 2 mph (like original)
  uint16_t cadence_rpm = (speedMph < 2.0f) ? 0 : 90;
  uint16_t cadence_ftms = cadence_rpm * 2;  // 0.5 rpm units

  // Power
  int16_t powerValue = (int16_t)constrain(watts, 0, 2000);

  // Update the data array
  FTMS_INDOOR_BIKE_DATA[4] = (uint8_t)(cadence_ftms & 0xFF);
  FTMS_INDOOR_BIKE_DATA[5] = (uint8_t)(cadence_ftms >> 8);
  FTMS_INDOOR_BIKE_DATA[6] = (uint8_t)(powerValue & 0xFF);
  FTMS_INDOOR_BIKE_DATA[7] = (uint8_t)((uint16_t)powerValue >> 8);

  // Send notification
  pIndoorBike->setValue(FTMS_INDOOR_BIKE_DATA, sizeof(FTMS_INDOOR_BIKE_DATA));
  pIndoorBike->notify();
  
  // Debug output
  static uint32_t notifyCount = 0;
  static uint32_t lastDebug = 0;
  notifyCount++;
  
  if (millis() - lastDebug > 2000) {
    lastDebug = millis();
    Serial.printf("[BLE] Notifications: %lu, Power=%dW, Cadence=%d RPM, Speed=%.1f mph\n", 
                  notifyCount, powerValue, cadence_rpm, speedMph);
  }
}

void bleNotifyStatus(uint8_t status) {
  if (!deviceConnected || pStatus == NULL) return;

  uint8_t data[2] = {status, 0x00};
  pStatus->setValue(data, 2);
  pStatus->notify();
}

// ==================== BLE KEEP-ALIVE ====================
// Periodically restart advertising to prevent BLE from becoming dormant
// Also detects RPM activity to wake from idle state

static uint32_t lastAdvertisingRestartMs = 0;
static constexpr uint32_t ADVERTISING_RESTART_INTERVAL_MS = 30000;  // Every 30 seconds
static bool wasIdle = true;  // Track idle state for RPM wake detection

void bleKeepAlive() {
  // Only restart advertising when NOT connected
  if (deviceConnected) {
    lastAdvertisingRestartMs = millis();  // Reset timer while connected
    wasIdle = false;
    return;
  }

  // Get current RPM from sensors (declared in sensors.h)
  extern float currentRPM;
  bool hasActivity = (currentRPM > 5.0f);  // Consider >5 RPM as activity

  // Wake on RPM: If we were idle and now detect pedaling, restart advertising immediately
  if (wasIdle && hasActivity) {
    wasIdle = false;
    lastAdvertisingRestartMs = millis();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->stop();
    delay(10);
    pAdvertising->start();

    Serial.println("[BLE] Advertising restarted (RPM wake)");
    return;
  }

  // Track idle state
  if (!hasActivity) {
    wasIdle = true;
  }

  // Periodically restart advertising to stay discoverable
  if (millis() - lastAdvertisingRestartMs >= ADVERTISING_RESTART_INTERVAL_MS) {
    lastAdvertisingRestartMs = millis();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->stop();
    delay(10);  // Brief pause
    pAdvertising->start();

    Serial.println("[BLE] Advertising restarted (keep-alive)");
  }
}
