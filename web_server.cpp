/*
 * web_server.cpp - WiFi AP and Web Server Implementation
 */

#include "web_server.h"
#include "stepper_control.h"
#include "ble_trainer.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

// ==================== OTA STATE ====================
volatile bool gOtaInProgress = false;
volatile bool gOtaDone = false;
volatile bool gOtaOk = false;
String gOtaErr = "";

static uint32_t otaUnlockedUntilMs = 0;
static uint32_t gOtaLastProgressMs = 0;
static size_t gOtaBytes = 0;
static uint32_t gOtaExpectedSize = 0;
static uint32_t gOtaWritten = 0;
static uint8_t gOtaLastPct = 255;

// ==================== WEB SERVER ====================
static WebServer server(80);

// ==================== HELPER FUNCTIONS ====================

static inline int32_t clampLogical(int32_t v) {
  if (v < LOGICAL_MIN) return LOGICAL_MIN;
  if (v > LOGICAL_MAX) return LOGICAL_MAX;
  return v;
}

// ==================== WEB HANDLERS ====================

static void handleRoot() {
  Serial.println("[HTTP] Root handler called - serving HTML");
  
  String html = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>InsideRide Trainer Control</title>
  <style>
    body {
      font-family: Arial, sans-serif;
      max-width: 800px;
      margin: 20px auto;
      padding: 20px;
      background: #f0f0f0;
    }
    .container {
      background: white;
      padding: 20px;
      border-radius: 8px;
      box-shadow: 0 2px 4px rgba(0,0,0,0.1);
      margin-bottom: 20px;
    }
    h1 {
      color: #333;
      margin-top: 0;
    }
    h2 {
      color: #666;
      border-bottom: 2px solid #007bff;
      padding-bottom: 5px;
    }
    .status-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin: 15px 0;
    }
    .status-item {
      padding: 10px;
      background: #f8f9fa;
      border-radius: 4px;
      border-left: 3px solid #007bff;
    }
    .status-label {
      font-size: 12px;
      color: #666;
      text-transform: uppercase;
    }
    .status-value {
      font-size: 24px;
      font-weight: bold;
      color: #333;
      margin-top: 5px;
    }
    .control-group {
      margin: 15px 0;
      padding: 15px;
      background: #f8f9fa;
      border-radius: 4px;
    }
    .input-group {
      display: flex;
      gap: 10px;
      align-items: center;
      margin: 10px 0;
    }
    input[type="number"] {
      flex: 1;
      padding: 10px;
      font-size: 16px;
      border: 2px solid #ddd;
      border-radius: 4px;
    }
    button {
      padding: 10px 20px;
      font-size: 16px;
      border: none;
      border-radius: 4px;
      cursor: pointer;
      transition: background 0.3s;
    }
    .btn-primary {
      background: #007bff;
      color: white;
    }
    .btn-primary:hover {
      background: #0056b3;
    }
    .btn-success {
      background: #28a745;
      color: white;
    }
    .btn-success:hover {
      background: #218838;
    }
    .btn-warning {
      background: #ffc107;
      color: black;
    }
    .btn-warning:hover {
      background: #e0a800;
    }
    .btn-danger {
      background: #dc3545;
      color: white;
    }
    .btn-danger:hover {
      background: #c82333;
    }
    .btn-block {
      width: 100%;
      margin: 5px 0;
    }
    .mode-indicator {
      display: inline-block;
      padding: 5px 15px;
      border-radius: 20px;
      font-weight: bold;
      margin-left: 10px;
    }
    .mode-idle { background: #6c757d; color: white; }
    .mode-erg { background: #28a745; color: white; }
    .mode-sim { background: #007bff; color: white; }
    .mode-manual { background: #ffc107; color: black; }
    .warning {
      background: #fff3cd;
      border: 1px solid #ffc107;
      padding: 10px;
      border-radius: 4px;
      margin: 10px 0;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>🚴 InsideRide Trainer</h1>
    <div>
      <strong>Mode:</strong> 
      <span id="mode" class="mode-indicator mode-idle">IDLE</span>
    </div>
  </div>

  <div class="container">
    <h2>📊 Live Diagnostics</h2>
    <div class="status-grid">
      <div class="status-item">
        <div class="status-label">Roller Speed</div>
        <div class="status-value" id="speed">0.0 mph</div>
      </div>
      <div class="status-item">
        <div class="status-label">Estimated Power</div>
        <div class="status-value" id="power">0 W</div>
      </div>
      <div class="status-item">
        <div class="status-label">Current Position</div>
        <div class="status-value" id="position">0</div>
      </div>
      <div class="status-item">
        <div class="status-label">Target Position</div>
        <div class="status-value" id="target">0</div>
      </div>
      <div class="status-item">
        <div class="status-label">ERG Target</div>
        <div class="status-value" id="erg_target">-- W</div>
      </div>
      <div class="status-item">
        <div class="status-label">SIM Grade</div>
        <div class="status-value" id="sim_grade">-- %</div>
      </div>
      <div class="status-item">
        <div class="status-label">Motor</div>
        <div class="status-value" id="motor">OFF</div>
      </div>
      <div class="status-item">
        <div class="status-label">BLE</div>
        <div class="status-value" id="ble">Disconnected</div>
      </div>
    </div>
    <div style="margin-top: 10px; padding: 10px; background: #e7f3ff; border-left: 3px solid #007bff; border-radius: 4px;">
      <small style="color: #666;">
        ℹ️ <strong>Motor Auto-Enable:</strong> Motor enables at 2.3 mph, disables at 2.0 mph
      </small>
    </div>
  </div>

  <div class="container">
    <h2>🎮 Manual Control</h2>
    
    <div class="control-group">
      <label for="goto_input"><strong>Go To Position (0-1000):</strong></label>
      <div class="input-group">
        <input type="number" id="goto_input" min="0" max="1000" value="500" />
        <button class="btn-primary" onclick="gotoPosition()">Go To</button>
      </div>
      <small style="color: #666;">Enter any value from 0 (min resistance) to 1000 (max resistance)</small>
    </div>

    <div id="manual_warning" class="warning" style="display:none;">
      ⚠️ Manual override active - Zwift control disabled
    </div>

    <button class="btn-success btn-block" onclick="resumeZwift()">
      ▶️ Resume Zwift Control
    </button>

  </div>

  <div class="container">
    <h2>🔄 OTA Firmware Update</h2>
    <p style="color: #888; font-size: 14px; margin: 5px 0;">Current version: )HTML";
  html += FW_VERSION;
  html += R"HTML(</p>
    <form method="POST" action="/update" enctype="multipart/form-data">
      <input type="file" name="update" accept=".bin" style="margin: 10px 0;">
      <button type="submit" class="btn-primary btn-block">📤 Upload Firmware</button>
    </form>
  </div>

  <script>
    function updateDiag() {
      fetch('/diag.json')
        .then(r => r.json())
        .then(d => {
          // Update all diagnostics
          document.getElementById('speed').textContent = d.speed.toFixed(1) + ' mph';
          document.getElementById('power').textContent = Math.round(d.power) + ' W';
          document.getElementById('position').textContent = d.pos;
          document.getElementById('target').textContent = d.target;
          // Update motor status with color
          let motorEl = document.getElementById('motor');
          if (d.enabled) {
            motorEl.textContent = '✓ ENABLED';
            motorEl.style.color = '#28a745';
          } else {
            motorEl.textContent = 'DISABLED';
            motorEl.style.color = '#6c757d';
          }
          document.getElementById('ble').textContent = d.ble ? 'Connected' : 'Disconnected';
          
          // Update mode
          let modeText = d.mode;
          let modeClass = 'mode-idle';
          if (d.manual_hold) {
            modeText = 'MANUAL';
            modeClass = 'mode-manual';
          } else if (d.mode === 'ERG') {
            modeClass = 'mode-erg';
          } else if (d.mode === 'SIM') {
            modeClass = 'mode-sim';
          }
          
          let modeEl = document.getElementById('mode');
          modeEl.textContent = modeText;
          modeEl.className = 'mode-indicator ' + modeClass;
          
          // Show/hide manual warning
          document.getElementById('manual_warning').style.display = 
            d.manual_hold ? 'block' : 'none';
          
          // Update ERG target
          if (d.mode === 'ERG') {
            document.getElementById('erg_target').textContent = d.erg_watts + ' W';
          } else {
            document.getElementById('erg_target').textContent = '-- W';
          }
          
          // Update SIM grade
          if (d.mode === 'SIM') {
            document.getElementById('sim_grade').textContent = d.sim_grade.toFixed(1) + ' %';
          } else {
            document.getElementById('sim_grade').textContent = '-- %';
          }
        })
        .catch(e => console.error('Update failed:', e));
    }

    function gotoPosition() {
      let pos = document.getElementById('goto_input').value;
      fetch('/goto_hold?pos=' + pos)
        .then(r => r.text())
        .then(msg => {
          console.log(msg);
          updateDiag();
        });
    }

    function resumeZwift() {
      fetch('/resume_zwift', {method: 'POST'})
        .then(r => r.text())
        .then(msg => {
          console.log(msg);
          updateDiag();
        });
    }

    // Update every 1 second
    setInterval(updateDiag, 1000);
    updateDiag(); // Initial update
  </script>
</body>
</html>
  )HTML";
  
  Serial.println("[HTTP] Sending HTML response");
  server.send(200, "text/html", html);
  Serial.println("[HTTP] Response sent");
}

static void handleDiagJson() {
  // Build JSON with all diagnostic data
  String modeStr;
  switch (gMode) {
    case MODE_ERG: modeStr = "ERG"; break;
    case MODE_SIM: modeStr = "SIM"; break;
    default: modeStr = "IDLE"; break;
  }
  
  String json = "{";
  json += "\"ble\":" + String(deviceConnected ? "true" : "false") + ",";
  json += "\"pos\":" + String(logStepPos) + ",";
  json += "\"target\":" + String(logStepTarget) + ",";
  json += "\"mode\":\"" + modeStr + "\",";
  json += "\"manual_hold\":" + String(gManualHoldActive ? "true" : "false") + ",";
  json += "\"enabled\":" + String(gStepEn ? "true" : "false") + ",";
  json += "\"speed\":" + String(currentSpeedMph, 2) + ",";
  json += "\"power\":" + String(currentPowerWatts, 1) + ",";
  json += "\"erg_watts\":" + String(ergTargetWatts) + ",";
  json += "\"sim_grade\":" + String(simGradePercent, 2);
  json += "}";
  
  server.send(200, "application/json", json);
}

static void handleGoto() {
  if (!server.hasArg("pos")) {
    server.send(400, "text/plain", "Missing pos parameter");
    return;
  }
  
  int32_t pos = server.arg("pos").toInt();
  pos = clampLogical(pos);
  
  stepperSetTarget(pos);
  
  server.send(200, "text/plain", "OK - Moving to " + String(pos));
}

static void handleEnable() {
  if (!server.hasArg("on")) {
    server.send(400, "text/plain", "Missing on parameter");
    return;
  }
  
  bool enable = server.arg("on") == "1";
  stepperEnable(enable);
  
  server.send(200, "text/plain", enable ? "Motor enabled" : "Motor disabled");
}

static void handleGotoHold() {
  if (!server.hasArg("pos")) {
    server.send(400, "text/plain", "Missing pos");
    return;
  }
  
  gManualHoldTarget = clampLogical(server.arg("pos").toInt());
  gManualHoldActive = true;
  
  server.send(200, "text/plain", "Manual hold active at " + String(gManualHoldTarget));
}

static void handleResumeZwift() {
  gManualHoldActive = false;
  server.send(200, "text/plain", "Manual hold released - Zwift control resumed");
}

static void handleUpdateForm() {
  server.send(200, "text/html", R"HTML(
<!DOCTYPE html>
<html>
<head><title>OTA Update</title></head>
<body>
  <h2>Firmware Update</h2>
  <form method="POST" action="/update" enctype="multipart/form-data">
    <input type="file" name="update">
    <button type="submit">Upload</button>
  </form>
  <p><a href="/">Back to main</a></p>
</body>
</html>
  )HTML");
}

static void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("[OTA] Starting: %s\n", upload.filename.c_str());
    gOtaBytes = 0;
    gOtaLastPct = 255;
    gOtaDone = false;
    gOtaOk = false;
    gOtaErr = "";

    // Check if BLE connected - deny OTA to prevent disruption
    if (OTA_DENY_WHEN_BLE_CONNECTED && deviceConnected) {
      Serial.println("[OTA] DENIED - BLE connected (disconnect Zwift first)");
      gOtaDone = true;
      gOtaOk = false;
      gOtaErr = "BLE connected - disconnect Zwift before updating firmware";
      gOtaInProgress = false;
      return;
    }

    gOtaInProgress = true;

    if (!Update.begin()) {
      Update.printError(Serial);
      gOtaInProgress = false;
      gOtaDone = true;
      gOtaOk = false;
      gOtaErr = "Update.begin() failed";
    }
  }
  else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    } else {
      gOtaBytes += upload.currentSize;
      gOtaLastProgressMs = millis();
      
      // Print progress
      uint8_t pct = (gOtaBytes * 100) / upload.totalSize;
      if (pct != gOtaLastPct && (pct % 10) == 0) {
        gOtaLastPct = pct;
        Serial.printf("[OTA] Progress: %d%%\n", pct);
      }
    }
  }
  else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("[OTA] Success: %u bytes\n", upload.totalSize);
      gOtaOk = true;
    } else {
      Update.printError(Serial);
      gOtaOk = false;
    }
    gOtaDone = true;
    gOtaInProgress = false;
  }
}

static void handleUpdatePostFinalizer() {
  if (gOtaOk) {
    server.send(200, "text/html", R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta http-equiv="refresh" content="10;url=/">
  <title>Update Success</title>
</head>
<body>
  <h2>Update successful!</h2>
  <p>Rebooting in 10 seconds...</p>
</body>
</html>
    )HTML");

    delay(1000);
    Serial.println("[OTA] Rebooting...");
    ESP.restart();
  } else {
    // Build error page with specific error message
    String html = R"HTML(
<!DOCTYPE html>
<html>
<head><title>Update Failed</title></head>
<body>
  <h2>Update FAILED</h2>
  <p style="color: red; font-weight: bold;">)HTML";
    html += gOtaErr.length() > 0 ? gOtaErr : "Unknown error";
    html += R"HTML(</p>
  <p><a href="/">Return to main page</a></p>
  <p><a href="/update">Try again</a></p>
</body>
</html>
    )HTML";
    server.send(500, "text/html", html);
  }
}

// ==================== PUBLIC FUNCTIONS ====================

void webServerInit() {
  Serial.println("===========================================");
  Serial.println("Starting WiFi AP...");
  
  // Disconnect from any previous WiFi
  WiFi.disconnect();
  delay(100);
  
  WiFi.mode(WIFI_AP);
  delay(100);
  
  // Configure IP settings explicitly
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  
  Serial.println("Configuring AP IP settings...");
  if (WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("  AP Config: OK");
  } else {
    Serial.println("  AP Config: FAILED");
  }
  
  // Set up WiFi event handlers BEFORE starting AP
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("======================================");
    Serial.println("[WiFi EVENT] Station CONNECTED!");
    Serial.printf("  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
      info.wifi_ap_staconnected.mac[0],
      info.wifi_ap_staconnected.mac[1],
      info.wifi_ap_staconnected.mac[2],
      info.wifi_ap_staconnected.mac[3],
      info.wifi_ap_staconnected.mac[4],
      info.wifi_ap_staconnected.mac[5]);
    Serial.printf("  Number of stations: %d\n", WiFi.softAPgetStationNum());
    Serial.println("======================================");
  }, ARDUINO_EVENT_WIFI_AP_STACONNECTED);
  
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    Serial.println("======================================");
    Serial.println("[WiFi EVENT] Station DISCONNECTED!");
    Serial.printf("  MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
      info.wifi_ap_stadisconnected.mac[0],
      info.wifi_ap_stadisconnected.mac[1],
      info.wifi_ap_stadisconnected.mac[2],
      info.wifi_ap_stadisconnected.mac[3],
      info.wifi_ap_stadisconnected.mac[4],
      info.wifi_ap_stadisconnected.mac[5]);
    Serial.printf("  Number of stations: %d\n", WiFi.softAPgetStationNum());
    Serial.println("======================================");
  }, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  
  // Start the AP
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  
  Serial.printf("WiFi AP Status: %s\n", ok ? "SUCCESS" : "FAILED");
  
  if (ok) {
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP Password: ");
    Serial.println(AP_PASS);
    Serial.print("AP IP Address: ");
    Serial.println(IP);
    Serial.printf("AP MAC Address: %s\n", WiFi.softAPmacAddress().c_str());
    Serial.printf("Number of connected stations: %d\n", WiFi.softAPgetStationNum());
    Serial.println("Connect to this network and browse to:");
    Serial.print("  http://");
    Serial.println(IP);
  } else {
    Serial.println("ERROR: Failed to start WiFi AP!");
    Serial.println("Check if WiFi is already in use or disabled");
  }
  
  // Register handlers
  server.on("/", HTTP_GET, handleRoot);
  server.on("/test", HTTP_GET, []() {
    Serial.println("[HTTP] Test endpoint hit!");
    server.send(200, "text/plain", "Web server is working!");
  });
  server.on("/diag.json", HTTP_GET, handleDiagJson);
  server.on("/goto", HTTP_GET, handleGoto);
  server.on("/enable", HTTP_GET, handleEnable);
  server.on("/goto_hold", HTTP_GET, handleGotoHold);
  server.on("/resume_zwift", HTTP_POST, handleResumeZwift);
  server.on("/update", HTTP_GET, handleUpdateForm);
  server.on("/update", HTTP_POST, handleUpdatePostFinalizer, handleUpdateUpload);
  
  // Add a catch-all handler for debugging
  server.onNotFound([]() {
    Serial.printf("[HTTP] 404: %s\n", server.uri().c_str());
    server.send(404, "text/plain", "Not found");
  });
  
  server.begin();
  Serial.println("✓ Web server started on port 80");
  Serial.println("===========================================");
}

void webServerUpdate() {
  static bool firstCall = true;
  if (firstCall) {
    Serial.println("[HTTP] webServerUpdate() is being called in loop");
    firstCall = false;
  }
  
  server.handleClient();
  
  // Debug: Show client requests periodically
  static uint32_t lastDebug = 0;
  static uint32_t requestCount = 0;
  
  if (server.client()) {
    requestCount++;
  }
  
  if (millis() - lastDebug > 5000 && requestCount > 0) {
    Serial.printf("[HTTP] Served %lu requests in last 5s\n", requestCount);
    requestCount = 0;
    lastDebug = millis();
  }
}

bool otaIsUnlocked() {
  return (millis() < otaUnlockedUntilMs);
}

void otaUnlock() {
  otaUnlockedUntilMs = millis() + OTA_UNLOCK_WINDOW_MS;
  Serial.println("[OTA] Unlocked for 60 seconds");
}
