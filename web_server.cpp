/*
 * web_server.cpp - WiFi AP and Web Server Implementation
 */

#include "web_server.h"
#include "stepper_control.h"
#include "ble_trainer.h"
#include "sensors.h"
#include "calibration.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Update.h>
#include <ESPmDNS.h>
#include <esp_ota_ops.h>

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

// ==================== WEBSOCKET SERVER ====================
static WebSocketsServer webSocket(81);
static uint32_t lastWsBroadcastMs = 0;
static const uint32_t WS_BROADCAST_INTERVAL_MS = 200;  // 5 Hz updates

// ==================== WIFI MODE TRACKING ====================
static bool gWifiClientMode = false;  // true = connected to home WiFi, false = AP mode

// ==================== HELPER FUNCTIONS ====================

static inline int32_t clampLogical(int32_t v) {
  if (v < LOGICAL_MIN) return LOGICAL_MIN;
  if (v > LOGICAL_MAX) return LOGICAL_MAX;
  return v;
}

// ==================== WEBSOCKET FUNCTIONS ====================

static String buildDiagJson() {
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
  json += "\"sim_grade\":" + String(simGradePercent, 2) + ",";
  json += "\"wifi_client\":" + String(gWifiClientMode ? "true" : "false");
  json += "}";
  return json;
}

static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected\n", num);
      break;
    case WStype_CONNECTED: {
      Serial.printf("[WS] Client #%u connected\n", num);
      // Send initial state immediately
      String diagJson = buildDiagJson();
      webSocket.sendTXT(num, diagJson);
      break;
    }
    case WStype_TEXT:
      // Could handle commands here if needed
      break;
    default:
      break;
  }
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

    <div class="control-group">
      <label for="grade_input"><strong>Go To Grade (-4% to 10%):</strong></label>
      <div class="input-group">
        <input type="number" id="grade_input" min="-4" max="10" step="0.5" value="0" />
        <button class="btn-primary" onclick="gotoGrade()">Set Grade</button>
      </div>
      <small style="color: #666;">Simulates hill grade using the SIM mode resistance curve</small>
    </div>

    <div id="manual_warning" class="warning" style="display:none;">
      ⚠️ Manual override active - App control disabled
    </div>

    <button class="btn-success btn-block" onclick="resumeApp()">
      ▶️ Resume App Control
    </button>

  </div>

  <div class="container">
    <h2>⚙️ IDLE Curve Calibration</h2>
    <p style="color: #888; font-size: 14px; margin: 5px 0;">
      Adjust the speed-to-position curve: pos = a + b×speed + c×speed² + d×speed³
    </p>
    <div class="control-group">
      <div class="status-grid" style="grid-template-columns: 1fr 1fr;">
        <div class="input-group" style="flex-direction: column; align-items: stretch;">
          <label for="cal_a"><strong>a (constant):</strong></label>
          <input type="number" id="cal_a" step="0.01" style="width: 100%;" />
        </div>
        <div class="input-group" style="flex-direction: column; align-items: stretch;">
          <label for="cal_b"><strong>b (linear):</strong></label>
          <input type="number" id="cal_b" step="0.01" style="width: 100%;" />
        </div>
        <div class="input-group" style="flex-direction: column; align-items: stretch;">
          <label for="cal_c"><strong>c (quadratic):</strong></label>
          <input type="number" id="cal_c" step="0.001" style="width: 100%;" />
        </div>
        <div class="input-group" style="flex-direction: column; align-items: stretch;">
          <label for="cal_d"><strong>d (cubic):</strong></label>
          <input type="number" id="cal_d" step="0.0001" style="width: 100%;" />
        </div>
      </div>
      <div style="display: flex; gap: 10px; margin-top: 10px;">
        <button class="btn-primary" onclick="saveCalibration()">💾 Save</button>
        <button class="btn-warning" onclick="resetCalibration()">↩️ Reset to Defaults</button>
      </div>
      <div id="cal_status" style="margin-top: 10px; display: none; padding: 8px; border-radius: 4px;"></div>
    </div>
  </div>

  <div class="container">
    <h2>📶 WiFi Settings</h2>
    <div class="control-group">
      <p style="font-size: 13px; color: #666; margin: 5px 0 10px 0;">
        <strong>Status:</strong> <span id="wifi_status">--</span> |
        <strong>IP:</strong> <span id="wifi_ip">--</span> |
        <strong>Signal:</strong> <span id="wifi_rssi">--</span>
      </p>
      <div style="margin-bottom: 10px;">
        <label for="wifi_ssid"><strong>Network (SSID):</strong></label>
        <input type="text" id="wifi_ssid" placeholder="Type your network name" style="width: 100%; padding: 10px; font-size: 16px; border: 2px solid #ddd; border-radius: 4px; box-sizing: border-box;">
      </div>
      <div style="margin-bottom: 10px;">
        <label for="wifi_pass"><strong>Password:</strong></label>
        <input type="password" id="wifi_pass" placeholder="Enter WiFi password" style="width: 100%; padding: 10px; font-size: 16px; border: 2px solid #ddd; border-radius: 4px; box-sizing: border-box;">
      </div>
      <div style="display: flex; gap: 10px; flex-wrap: wrap;">
        <button class="btn-success" onclick="saveWifi()">💾 Save</button>
        <button class="btn-warning" onclick="restartDevice()">🔄 Restart</button>
        <button class="btn-danger" onclick="clearWifi()">🗑️ Clear</button>
      </div>
      <div id="wifi_msg" style="margin-top: 10px; display: none; padding: 8px; border-radius: 4px;"></div>
      <div style="margin-top: 15px; padding: 12px; background: #fff3cd; border: 1px solid #ffc107; border-radius: 4px;">
        <strong>⚠️ Important:</strong> After clicking Save, please wait up to 60 seconds.
        The page may become unresponsive - this is normal. Do NOT power cycle the device.
        After saving, click Restart, then reconnect to your home WiFi and browse to
        <strong>http://insideride.local</strong> or the IP address shown in the device logs.
      </div>
    </div>
  </div>

  <div class="container">
    <h2>🔄 OTA Firmware Update</h2>
    <p style="color: #888; font-size: 14px; margin: 5px 0;">Current version: )HTML";
  html += FW_VERSION;
  html += R"HTML(</p>
    <div id="ota_blocked" class="warning" style="display:none;">
      ⚠️ <strong>OTA Blocked:</strong> Disconnect App/BLE before updating firmware
    </div>
    <form method="POST" action="/update" enctype="multipart/form-data" id="ota_form">
      <input type="file" name="update" accept=".bin" style="margin: 10px 0;" id="ota_file">
      <button type="submit" class="btn-primary btn-block" id="ota_btn">📤 Upload Firmware</button>
    </form>
    <div style="margin-top: 15px; padding-top: 15px; border-top: 1px solid #ddd;">
      <p style="font-size: 13px; color: #666; margin: 5px 0;">
        <strong>Partition:</strong> <span id="ota_partition">--</span> |
        <strong>State:</strong> <span id="ota_state">--</span>
      </p>
      <button class="btn-warning btn-block" id="rollback_btn" onclick="rollbackFirmware()" style="display:none;">
        ↩️ Rollback to Previous Firmware
      </button>
    </div>
  </div>

  <script>
    function updateDiag() {
      fetch('/diag.json')
        .then(r => r.json())
        .then(d => applyDiagData(d))
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

    function gotoGrade() {
      let grade = document.getElementById('grade_input').value;
      fetch('/grade_hold?grade=' + grade)
        .then(r => r.text())
        .then(msg => {
          console.log(msg);
          updateDiag();
        });
    }

    function resumeApp() {
      fetch('/resume_app', {method: 'POST'})
        .then(r => r.text())
        .then(msg => {
          console.log(msg);
          updateDiag();
        });
    }

    function loadCalibration() {
      fetch('/calibration.json')
        .then(r => r.json())
        .then(d => {
          document.getElementById('cal_a').value = d.a;
          document.getElementById('cal_b').value = d.b;
          document.getElementById('cal_c').value = d.c;
          document.getElementById('cal_d').value = d.d;
        });
    }

    function saveCalibration() {
      let a = document.getElementById('cal_a').value;
      let b = document.getElementById('cal_b').value;
      let c = document.getElementById('cal_c').value;
      let d = document.getElementById('cal_d').value;
      fetch('/calibration?a=' + a + '&b=' + b + '&c=' + c + '&d=' + d, {method: 'POST'})
        .then(r => r.text())
        .then(msg => {
          showCalStatus(msg, true);
        })
        .catch(e => showCalStatus('Error: ' + e, false));
    }

    function resetCalibration() {
      fetch('/calibration/reset', {method: 'POST'})
        .then(r => r.text())
        .then(msg => {
          showCalStatus(msg, true);
          loadCalibration();
        });
    }

    function showCalStatus(msg, success) {
      let el = document.getElementById('cal_status');
      el.textContent = msg;
      el.style.display = 'block';
      el.style.background = success ? '#d4edda' : '#f8d7da';
      el.style.color = success ? '#155724' : '#721c24';
      setTimeout(() => { el.style.display = 'none'; }, 3000);
    }

    // ==================== WEBSOCKET WITH FALLBACK ====================
    let ws = null;
    let wsConnected = false;
    let pollInterval = null;

    function applyDiagData(d) {
      document.getElementById('speed').textContent = d.speed.toFixed(1) + ' mph';
      document.getElementById('power').textContent = Math.round(d.power) + ' W';
      document.getElementById('position').textContent = d.pos;
      document.getElementById('target').textContent = d.target;

      let motorEl = document.getElementById('motor');
      if (d.enabled) {
        motorEl.textContent = '✓ ENABLED';
        motorEl.style.color = '#28a745';
      } else {
        motorEl.textContent = 'DISABLED';
        motorEl.style.color = '#6c757d';
      }

      let bleEl = document.getElementById('ble');
      bleEl.textContent = d.ble ? 'Connected' : 'Disconnected';
      bleEl.style.color = d.ble ? '#28a745' : '#6c757d';

      let otaBtn = document.getElementById('ota_btn');
      let otaFile = document.getElementById('ota_file');
      let otaBlocked = document.getElementById('ota_blocked');
      if (d.ble) {
        otaBtn.disabled = true;
        otaBtn.style.opacity = '0.5';
        otaBtn.style.cursor = 'not-allowed';
        otaFile.disabled = true;
        otaBlocked.style.display = 'block';
      } else {
        otaBtn.disabled = false;
        otaBtn.style.opacity = '1';
        otaBtn.style.cursor = 'pointer';
        otaFile.disabled = false;
        otaBlocked.style.display = 'none';
      }

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

      document.getElementById('manual_warning').style.display = d.manual_hold ? 'block' : 'none';
      document.getElementById('erg_target').textContent = d.mode === 'ERG' ? d.erg_watts + ' W' : '-- W';
      document.getElementById('sim_grade').textContent = d.mode === 'SIM' ? d.sim_grade.toFixed(1) + ' %' : '-- %';
    }

    function connectWebSocket() {
      let wsUrl = 'ws://' + window.location.hostname + ':81/';
      ws = new WebSocket(wsUrl);

      ws.onopen = function() {
        console.log('[WS] Connected');
        wsConnected = true;
        if (pollInterval) {
          clearInterval(pollInterval);
          pollInterval = null;
        }
      };

      ws.onmessage = function(evt) {
        try {
          let d = JSON.parse(evt.data);
          applyDiagData(d);
        } catch (e) {
          console.error('[WS] Parse error:', e);
        }
      };

      ws.onclose = function() {
        console.log('[WS] Disconnected, falling back to polling');
        wsConnected = false;
        ws = null;
        if (!pollInterval) {
          pollInterval = setInterval(updateDiag, 1000);
        }
        setTimeout(connectWebSocket, 3000);
      };

      ws.onerror = function(err) {
        console.error('[WS] Error:', err);
        ws.close();
      };
    }

    function loadOtaInfo() {
      fetch('/ota_info.json')
        .then(r => r.json())
        .then(d => {
          document.getElementById('ota_partition').textContent = d.running_partition;
          document.getElementById('ota_state').textContent = d.ota_state;
          let rollbackBtn = document.getElementById('rollback_btn');
          if (d.can_rollback) {
            rollbackBtn.style.display = 'block';
          } else {
            rollbackBtn.style.display = 'none';
          }
        })
        .catch(e => console.error('OTA info failed:', e));
    }

    function rollbackFirmware() {
      if (!confirm('Roll back to the previous firmware version?\\n\\nThe device will restart.')) return;
      fetch('/ota_rollback', {method: 'POST'})
        .then(r => r.text())
        .then(msg => {
          alert(msg);
        })
        .catch(e => alert('Rollback failed: ' + e));
    }

    // ==================== WIFI FUNCTIONS ====================
    function loadWifiStatus() {
      fetch('/wifi_status.json')
        .then(r => r.json())
        .then(d => {
          document.getElementById('wifi_status').textContent = d.client_mode ? 'Connected' : 'AP Mode';
          document.getElementById('wifi_ip').textContent = d.ip;
          document.getElementById('wifi_rssi').textContent = d.client_mode ? d.rssi + ' dBm' : 'N/A';
          if (d.configured && d.ssid) {
            document.getElementById('wifi_ssid').value = d.ssid;
          }
        })
        .catch(e => console.error('WiFi status failed:', e));
    }

    function saveWifi() {
      let ssid = document.getElementById('wifi_ssid').value.trim();
      let pass = document.getElementById('wifi_pass').value;
      if (!ssid) {
        showWifiMsg('Please enter a network name', false);
        return;
      }
      if (!confirm('Save WiFi settings?\\n\\nSSID: ' + ssid + '\\n\\nAfter saving, click Restart to connect.')) return;
      showWifiMsg('Saving...', true);
      fetch('/wifi_save?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass), {method: 'POST', signal: AbortSignal.timeout(8000)})
        .then(r => r.text())
        .then(msg => {
          showWifiMsg('✓ ' + msg, true);
        })
        .catch(e => {
          // Save likely succeeded even if response was lost - tell user to restart
          showWifiMsg('Settings likely saved (connection interrupted). Click Restart to apply.', true);
        });
    }

    function restartDevice() {
      if (!confirm('Restart the device now?')) return;
      fetch('/wifi_restart', {method: 'POST'})
        .then(r => r.text())
        .then(msg => {
          showWifiMsg('Restarting... Reconnect to the new network.', true);
        })
        .catch(e => showWifiMsg('Restart failed: ' + e, false));
    }

    function clearWifi() {
      if (!confirm('Clear WiFi settings?\\n\\nThe device will use AP mode on next restart.')) return;
      fetch('/wifi_clear', {method: 'POST'})
        .then(r => r.text())
        .then(msg => {
          showWifiMsg(msg, true);
          document.getElementById('wifi_ssid').value = '';
          document.getElementById('wifi_pass').value = '';
        })
        .catch(e => showWifiMsg('Clear failed: ' + e, false));
    }

    function showWifiMsg(msg, success) {
      let el = document.getElementById('wifi_msg');
      el.textContent = msg;
      el.style.display = 'block';
      el.style.background = success ? '#d4edda' : '#f8d7da';
      el.style.color = success ? '#155724' : '#721c24';
      setTimeout(() => { el.style.display = 'none'; }, 5000);
    }

    // Start with WebSocket, fallback to polling
    connectWebSocket();
    updateDiag(); // Initial fetch
    loadCalibration();
    loadOtaInfo();
    loadWifiStatus();
  </script>
</body>
</html>
  )HTML";
  
  Serial.println("[HTTP] Sending HTML response");
  server.send(200, "text/html", html);
  Serial.println("[HTTP] Response sent");
}

static void handleDiagJson() {
  // Use shared function for consistency
  server.send(200, "application/json", buildDiagJson());
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

static void handleGradeHold() {
  if (!server.hasArg("grade")) {
    server.send(400, "text/plain", "Missing grade");
    return;
  }

  float grade = server.arg("grade").toFloat();
  grade = constrain(grade, -4.0f, 10.0f);

  // Use gradeToSteps to convert grade to position (uses current speed)
  int32_t pos = (int32_t)lround(gradeToSteps(currentSpeedMph, grade));
  pos = clampLogical(pos);

  gManualHoldTarget = pos;
  gManualHoldActive = true;

  server.send(200, "text/plain", "Grade " + String(grade, 1) + "% -> position " + String(pos));
}

static void handleResumeApp() {
  gManualHoldActive = false;
  server.send(200, "text/plain", "Manual hold released - App control resumed");
}

static void handleCalibrationJson() {
  String json = "{";
  json += "\"a\":" + String(gIdleCurveA, 4) + ",";
  json += "\"b\":" + String(gIdleCurveB, 4) + ",";
  json += "\"c\":" + String(gIdleCurveC, 5) + ",";
  json += "\"d\":" + String(gIdleCurveD, 6);
  json += "}";
  server.send(200, "application/json", json);
}

static void handleCalibrationSet() {
  if (!server.hasArg("a") || !server.hasArg("b") || !server.hasArg("c") || !server.hasArg("d")) {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  gIdleCurveA = server.arg("a").toFloat();
  gIdleCurveB = server.arg("b").toFloat();
  gIdleCurveC = server.arg("c").toFloat();
  gIdleCurveD = server.arg("d").toFloat();

  calibrationSave();

  server.send(200, "text/plain", "Calibration saved");
}

static void handleCalibrationReset() {
  calibrationReset();
  server.send(200, "text/plain", "Calibration reset to defaults");
}

// ==================== WIFI SETTINGS HANDLERS ====================

static void handleWifiStatus() {
  String json = "{";
  json += "\"client_mode\":" + String(gWifiClientMode ? "true" : "false") + ",";
  json += "\"configured\":" + String(gWifiConfigured ? "true" : "false") + ",";
  json += "\"ssid\":\"" + String(gWifiConfigured ? gWifiSsid : "") + "\",";
  if (gWifiClientMode) {
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI());
  } else {
    json += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
    json += "\"rssi\":0";
  }
  json += "}";
  server.send(200, "application/json", json);
}

static void handleWifiSave() {
  Serial.println("==========================================");
  Serial.println("[WiFi] handleWifiSave called");
  Serial.printf("[WiFi] HTTP Method: %s\n", server.method() == HTTP_POST ? "POST" : "GET");
  Serial.printf("[WiFi] Number of args: %d\n", server.args());

  for (int i = 0; i < server.args(); i++) {
    Serial.printf("[WiFi] Arg %d: '%s' = '%s'\n", i, server.argName(i).c_str(), server.arg(i).c_str());
  }

  if (!server.hasArg("ssid")) {
    Serial.println("[WiFi] ERROR: Missing SSID argument!");
    server.send(400, "text/plain", "Missing SSID");
    return;
  }

  String ssid = server.arg("ssid");
  String pass = server.hasArg("pass") ? server.arg("pass") : "";

  // Validate SSID
  if (ssid.length() == 0) {
    Serial.println("[WiFi] ERROR: Empty SSID!");
    server.send(400, "text/plain", "SSID cannot be empty");
    return;
  }

  Serial.printf("[WiFi] Saving credentials - SSID: '%s', Pass length: %d\n", ssid.c_str(), pass.length());

  // Save to NVS
  wifiSettingsSave(ssid.c_str(), pass.c_str());

  Serial.printf("[WiFi] After save - gWifiConfigured: %s, gWifiSsid: '%s'\n",
                gWifiConfigured ? "true" : "false", gWifiSsid);
  Serial.println("==========================================");

  server.send(200, "text/plain", "WiFi settings saved! Click Restart to connect to your network.");
}

static void handleWifiClear() {
  wifiSettingsClear();
  server.send(200, "text/plain", "WiFi settings cleared. Device will use AP mode on next restart.");
}

static void handleWifiRestart() {
  server.send(200, "text/plain", "Restarting WiFi...");
  delay(500);
  ESP.restart();
}

static void handleOtaInfo() {
  const esp_partition_t* running = esp_ota_get_running_partition();
  const esp_partition_t* nextUpdate = esp_ota_get_next_update_partition(NULL);
  esp_ota_img_states_t otaState;
  esp_ota_get_state_partition(running, &otaState);

  String stateStr = "UNKNOWN";
  switch (otaState) {
    case ESP_OTA_IMG_NEW: stateStr = "NEW"; break;
    case ESP_OTA_IMG_PENDING_VERIFY: stateStr = "PENDING_VERIFY"; break;
    case ESP_OTA_IMG_VALID: stateStr = "VALID"; break;
    case ESP_OTA_IMG_INVALID: stateStr = "INVALID"; break;
    case ESP_OTA_IMG_ABORTED: stateStr = "ABORTED"; break;
    default: break;
  }

  String json = "{";
  json += "\"version\":\"" + String(FW_VERSION) + "\",";
  json += "\"running_partition\":\"" + String(running->label) + "\",";
  json += "\"running_address\":\"0x" + String(running->address, HEX) + "\",";
  json += "\"running_size\":" + String(running->size) + ",";
  json += "\"next_update_partition\":\"" + String(nextUpdate->label) + "\",";
  json += "\"ota_state\":\"" + stateStr + "\",";
  json += "\"can_rollback\":" + String(esp_ota_check_rollback_is_possible() ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

static void handleOtaRollback() {
  if (!esp_ota_check_rollback_is_possible()) {
    server.send(400, "text/plain", "Rollback not possible - no previous valid firmware");
    return;
  }

  server.send(200, "text/plain", "Rolling back to previous firmware... Device will restart.");
  delay(500);

  esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot();
  if (err != ESP_OK) {
    Serial.printf("[OTA] Rollback failed: %d\n", err);
  }
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
      Serial.println("[OTA] DENIED - BLE connected (disconnect App first)");
      gOtaDone = true;
      gOtaOk = false;
      gOtaErr = "BLE connected - disconnect App before updating firmware";
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

// ==================== HELPER: START AP MODE ====================
static bool startAPMode() {
  Serial.println("[WiFi] Starting AP mode...");

  WiFi.mode(WIFI_AP);
  delay(100);

  // Configure IP settings explicitly
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("[WiFi] AP Config FAILED");
    return false;
  }

  // Set up WiFi event handlers for AP mode
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
  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("[WiFi] AP start FAILED");
    return false;
  }

  gWifiClientMode = false;

  IPAddress IP = WiFi.softAPIP();
  Serial.println("[WiFi] AP Mode Active");
  Serial.printf("  SSID: %s\n", AP_SSID);
  Serial.printf("  Password: %s\n", AP_PASS);
  Serial.printf("  IP: %s\n", IP.toString().c_str());

  return true;
}

// ==================== HELPER: TRY CLIENT MODE ====================
static bool tryClientModeWith(const char* ssid, const char* pass) {
  if (strlen(ssid) == 0) {
    return false;
  }

  Serial.printf("[WiFi] Attempting to connect to '%s'...\n", ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - startMs > WIFI_STA_TIMEOUT_MS) {
      Serial.println("[WiFi] Client connection timeout");
      WiFi.disconnect();
      return false;
    }
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  gWifiClientMode = true;

  Serial.println("[WiFi] Client Mode Connected!");
  Serial.printf("  SSID: %s\n", ssid);
  Serial.printf("  IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());

  return true;
}

static bool tryClientMode() {
  // First try NVS-stored credentials
  if (gWifiConfigured && strlen(gWifiSsid) > 0) {
    Serial.println("[WiFi] Trying saved WiFi credentials...");
    if (tryClientModeWith(gWifiSsid, gWifiPass)) {
      return true;
    }
    Serial.println("[WiFi] Saved credentials failed");
  }

  // Then try config.h credentials (for developer use)
  if (strlen(WIFI_STA_SSID) > 0) {
    Serial.println("[WiFi] Trying config.h credentials...");
    return tryClientModeWith(WIFI_STA_SSID, WIFI_STA_PASS);
  }

  Serial.println("[WiFi] No WiFi credentials configured");
  return false;
}

// ==================== PUBLIC FUNCTIONS ====================

void webServerInit() {
  Serial.println("===========================================");
  Serial.println("Starting WiFi...");

  // Disconnect from any previous WiFi
  WiFi.disconnect();
  delay(100);

  // Try client mode first, fallback to AP
  bool wifiOk = false;
  if (tryClientMode()) {
    wifiOk = true;
  } else {
    Serial.println("[WiFi] Falling back to AP mode...");
    wifiOk = startAPMode();
  }

  if (wifiOk) {
    // Start mDNS responder
    if (MDNS.begin("insideride")) {
      MDNS.addService("http", "tcp", 80);
      Serial.println("✓ mDNS started: http://insideride.local");
    } else {
      Serial.println("✗ mDNS failed to start");
    }

    if (gWifiClientMode) {
      Serial.println("Browse to:");
      Serial.printf("  http://%s  or  http://insideride.local\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("Connect to this network and browse to:");
      Serial.printf("  http://%s  or  http://insideride.local\n", WiFi.softAPIP().toString().c_str());
    }
  } else {
    Serial.println("ERROR: Failed to start WiFi!");
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
  server.on("/grade_hold", HTTP_GET, handleGradeHold);
  server.on("/resume_app", HTTP_POST, handleResumeApp);
  server.on("/calibration.json", HTTP_GET, handleCalibrationJson);
  server.on("/calibration", HTTP_POST, handleCalibrationSet);
  server.on("/calibration/reset", HTTP_POST, handleCalibrationReset);
  server.on("/wifi_status.json", HTTP_GET, handleWifiStatus);
  server.on("/wifi_save", HTTP_POST, handleWifiSave);
  server.on("/wifi_clear", HTTP_POST, handleWifiClear);
  server.on("/wifi_restart", HTTP_POST, handleWifiRestart);
  server.on("/ota_info.json", HTTP_GET, handleOtaInfo);
  server.on("/ota_rollback", HTTP_POST, handleOtaRollback);
  server.on("/update", HTTP_GET, handleUpdateForm);
  server.on("/update", HTTP_POST, handleUpdatePostFinalizer, handleUpdateUpload);
  
  // Add a catch-all handler for debugging
  server.onNotFound([]() {
    Serial.printf("[HTTP] 404: %s\n", server.uri().c_str());
    server.send(404, "text/plain", "Not found");
  });
  
  server.begin();
  Serial.println("✓ Web server started on port 80");

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("✓ WebSocket server started on port 81");

  Serial.println("===========================================");
}

void webServerUpdate() {
  static bool firstCall = true;
  if (firstCall) {
    Serial.println("[HTTP] webServerUpdate() is being called in loop");
    firstCall = false;
  }

  server.handleClient();
  webSocket.loop();

  // Broadcast diagnostics over WebSocket at 5 Hz
  if (millis() - lastWsBroadcastMs >= WS_BROADCAST_INTERVAL_MS) {
    lastWsBroadcastMs = millis();
    if (webSocket.connectedClients() > 0) {
      String diagJson = buildDiagJson();
      webSocket.broadcastTXT(diagJson);
    }
  }

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
