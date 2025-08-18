/*
 * SDM120CT Modbus Emulator
 * 
 * Description:
 * This program emulates the SDM120CT Modbus meter and integrates with Home Assistant via MQTT.
 * It supports telemetry publishing, setup mode toggle, restart functionality, and inverter connection status.
 * 
 * Example JSON to send meter data to mqtt_topic_subscribe
 * {
 *   "voltage": 230.0,
 *   "current": 1.5,
 *   "activePower": 345.0,
 *   "apparentPower": 360.0,
 *   "reactivePower": 25.0,
 *   "powerFactor": 0.96,
 *   "frequency": 50.0,
 *   "importEnergy": 20.0,
 *   "exportEnergy": 0.0
 * }
 * 
 * Note:
 * For Meter 1, the inverter expects to see a negative value when the house is drawing power from the grid
 * and a positive value only if the house is exporting energy.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <EspMQTTClient.h>
#include <ModbusServerRTU.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "Private.h"
#include <time.h>

// ========================= Debug Config =========================
// Enable or disable debug logs if not defined in Private.h
#ifndef DEBUG_HEX_FRAMES
#define DEBUG_HEX_FRAMES false  // Set true to enable Modbus HEX frame logs
#endif

// Enable or disable Wi-Fi debug logs if not defined in Private.h
#ifndef DEBUG_WIFI
#define DEBUG_MQTT true
#endif

// ========================= RS-485 Config =========================
#define RS485_TX 17
#define RS485_RX 16
#define RS485_RTS 4 // DE/RE control

// ========================= Wi-Fi & MQTT Config =========================
// Secrets pulled from Private.h file
const char* ssid = secret_wifi_ssid;
const char* password = secret_wifi_password;
const char* mqtt_server = secret_mqtt_server;
const char* mqtt_username = secret_mqtt_username;
const char* mqtt_password = secret_mqtt_password;
const int mqtt_port = 1883;


// Define the Wi-Fi PHY mode if not defined in Private.h
#ifndef ENABLE_WIFI_PHY_MODE_11G
#define ENABLE_WIFI_PHY_MODE_11G 0  // Set to 1 to enable 11G PHY mode
#endif

// Correct MQTT Topics for Home Assistant auto-discovery
const char* mqtt_topic_subscribe = "homeassistant/sensor/sdm120ct/meter_data";
const char* mqtt_topic_setupmode = "homeassistant/switch/sdm120ct/setup_mode";
const char* mqtt_topic_status = "homeassistant/sensor/sdm120ct/status";


// Device info for Home Assistant
const char* device_name = "SDM120CT Modbus Emulator";
const char* device_model = "ESP32 SDM120CT Modbus Emulator";
const char* device_manufacturer = "Genestealer";
const char* device_id = "sdm120ct_emulator";

// ========================= Modbus Config =========================
// If not defined in Private.h, use this default ID
#ifndef DEFAULT_METER_ID
#define DEFAULT_METER_ID 1
#endif

// ========================= Modbus Server =========================
ModbusServerRTU MBserver(2000, RS485_RTS); // 2000ms timeout

// ========================= SDM120CT Registers =========================
float voltage = 240.0;        // V
float current = 1.25;         // A
float activePower = 300.0;    // W
float apparentPower = 310.0;  // VA
float reactivePower = 20.0;   // VAr
float powerFactor = 0.97;
float phaseAngle = 0.5;       // degrees
float frequency = 50.0;       // Hz
float importEnergy = 15.0;    // kWh
float exportEnergy = 0.0;     // kWh
float importReactiveEnergy = 1.2; // kVArh
float exportReactiveEnergy = 0.0; // kVArh
float totalActiveEnergy = 15.0;   // kWh
float totalReactiveEnergy = 2.0;  // kVArh

// ========================= Configurable Registers =========================
uint16_t meterID = DEFAULT_METER_ID; // Default ID
uint16_t baudSetting = 2;            // 2 = 9600 baud
uint16_t paritySetting = 0;          // 0=None, 1=Even, 2=Odd

// Setup mode flag
bool setupMode = false; // Controlled via MQTT now

// ========================= MQTT Client =========================
EspMQTTClient mqtt(
  ssid,
  password,
  mqtt_server,
  mqtt_username,
  mqtt_password,
  "ESP32_Meter",
  mqtt_port
);

// ========================= NTP Configuration =========================

// Define the NTP server if missing from the private.h file
#ifndef secret_local_timeclock_server
#define secret_local_timeclock_server "pool.ntp.org"
#endif

const char* ntpServer = secret_local_timeclock_server;
const long gmtOffset_sec = 0; // Adjust for your timezone
const int daylightOffset_sec = 0; // Adjust for daylight savings

// ========================= Utility =========================
// Convert float to two Modbus registers (word-swapped)
void floatToRegisters(float value, uint16_t *regHi, uint16_t *regLo) {
  uint16_t regs[2];
  memcpy(regs, &value, sizeof(float));
  *regHi = regs[1];
  *regLo = regs[0];
}

// Function: calculateWiFiSignalStrengthPercentage
// Description: Converts RSSI to a percentage value (0-100%).
int calculateWiFiSignalStrengthPercentage(int rssi) {
  int strength = constrain(rssi, -100, -50); // Clamp RSSI to a reasonable range
  return map(strength, -100, -50, 0, 100);   // Map RSSI to percentage (0-100%)
}

// Debug: Print response frame
void printHexFrame(ModbusMessage &msg) {
#if DEBUG_HEX_FRAMES
  Serial.print("[RESP HEX] ");
  for (size_t i = 0; i < msg.size(); i++) {
    Serial.printf("%02X ", msg[i]);
  }
  Serial.println();
#endif
}

// ========================= MQTT Auto-Discovery Templates =========================
// Home Assistant MQTT Discovery configurations for all sensors and switches

// Discovery for Setup Mode Switch
String jsonDiscoverySetupMode = R"rawliteral(
{
  "name": "Setup Mode",
  "uniq_id": "sdm120ct_setup_mode",
  "obj_id": "sdm120ct_setup_mode",
  "ic": "mdi:wrench",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "stat_t": "homeassistant/meter/setupmode/state",
  "cmd_t": "homeassistant/meter/setupmode",
  "pl_on": "ON",
  "pl_off": "OFF",
  "stat_on": "ON",
  "stat_off": "OFF",
  "ent_cat": "config",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";


// Discovery for Device Uptime
String jsonDiscoveryUptime = R"rawliteral(
{
  "name": "Device Uptime",
  "uniq_id": "sdm120ct_uptime",
  "obj_id": "sdm120ct_uptime",
  "device_class": "timestamp",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "stat_t": "homeassistant/sensor/sdm120ct/uptime",
  "frc_upd": "true",
  "ent_cat": "diagnostic",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";

// Discovery for IP Address
String jsonDiscoveryWifiIP = R"rawliteral(
{
  "name": "IP Address",
  "uniq_id": "sdm120ct_wifi_ip",
  "obj_id": "sdm120ct_wifi_ip",
  "ic": "mdi:ip-network",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "stat_t": "homeassistant/sensor/sdm120ct/wifi_ip",
  "frc_upd": "true",
  "ent_cat": "diagnostic",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";

// Discovery for MAC Address
String jsonDiscoveryMacAddress = R"rawliteral(
{
  "name": "MAC Address",
  "uniq_id": "sdm120ct_mac_address",
  "obj_id": "sdm120ct_mac_address",
  "ic": "mdi:folder-key-network",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "stat_t": "homeassistant/sensor/sdm120ct/mac_address",
  "frc_upd": "true",
  "ent_cat": "diagnostic",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";

// Discovery for WiFi SSID
String jsonDiscoverySSID = R"rawliteral(
{
  "name": "WiFi SSID",
  "uniq_id": "sdm120ct_wifi_ssid",
  "obj_id": "sdm120ct_wifi_ssid",
  "ic": "mdi:wifi",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "stat_t": "homeassistant/sensor/sdm120ct/ssid",
  "frc_upd": "true",
  "ent_cat": "diagnostic",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";

// Discovery for WiFi BSSID
String jsonDiscoveryBSSID = R"rawliteral(
{
  "name": "WiFi BSSID",
  "uniq_id": "sdm120ct_wifi_bssid",
  "obj_id": "sdm120ct_wifi_bssid",
  "ic": "mdi:router-wireless",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "stat_t": "homeassistant/sensor/sdm120ct/bssid",
  "frc_upd": "true",
  "ent_cat": "diagnostic",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";

// Discovery for WiFi RSSI
String jsonDiscoveryWifiRSSI = R"rawliteral(
{
  "name": "WiFi RSSI",
  "uniq_id": "sdm120ct_wifi_rssi",
  "obj_id": "sdm120ct_wifi_rssi",
  "ic": "mdi:wifi-strength-2",
  "unit_of_meas": "dBm",
  "device_class": "signal_strength",
  "state_class": "measurement",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "stat_t": "homeassistant/sensor/sdm120ct/wifi_rssi",
  "frc_upd": "true",
  "ent_cat": "diagnostic",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";

// Discovery for WiFi Signal Percentage
String jsonDiscoveryWifiSignalPercentage = R"rawliteral(
{
  "name": "WiFi Signal",
  "uniq_id": "sdm120ct_wifi_signal_percentage",
  "obj_id": "sdm120ct_wifi_signal_percentage",
  "ic": "mdi:wifi-strength-4",
  "unit_of_meas": "%",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "stat_t": "homeassistant/sensor/sdm120ct/wifi_signal_percentage",
  "frc_upd": "true",
  "ent_cat": "diagnostic",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";

// Discovery for Restart Button
String jsonDiscoveryRestartButton = R"rawliteral(
{
  "name": "Restart Device",
  "uniq_id": "sdm120ct_restart",
  "obj_id": "sdm120ct_restart",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "cmd_t": "homeassistant/sensor/sdm120ct/restart",
  "pl_prs": "restart",
  "ent_cat": "config",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";

// Discovery for Inverter Connection Status
String jsonDiscoveryInverterConnected = R"rawliteral(
{
  "name": "Inverter Connected",
  "uniq_id": "sdm120ct_inverter_connected",
  "obj_id": "sdm120ct_inverter_connected",
  "device_class": "connectivity",
  "qos": 0,
  "avty_t": "homeassistant/sensor/sdm120ct/status",
  "stat_t": "homeassistant/binary_sensor/sdm120ct/inverter_connected",
  "pl_on": "ON",
  "pl_off": "OFF",
  "stat_on": "ON",
  "stat_off": "OFF",
  "ent_cat": "diagnostic",
  "dev": {
    "ids": ["sdm120ct_emulator"],
    "name": "SDM120CT Modbus Emulator",
    "mdl": "ESP32 SDM120CT Modbus Emulator",
    "mf": "Genestealer"
  }
}
)rawliteral";

// ========================= MQTT Functions =========================
// Function: publishWifiDetails
// Description: Publishes Wi-Fi diagnostics (IP, RSSI, signal strength, etc.) to MQTT.
void publishWifiDetails() {
  if (DEBUG_MQTT) Serial.println("[MQTT] Publishing Wi-Fi details");
  String wifiIP = WiFi.localIP().toString();
  int wifiRSSI = WiFi.RSSI();
  int wifiSignalPercentage = calculateWiFiSignalStrengthPercentage(wifiRSSI);
  String macAddress = WiFi.macAddress();
  String wifiSSID = WiFi.SSID();
  String wifiBSSID = WiFi.BSSIDstr();
  String status = (WiFi.status() == WL_CONNECTED) ? "online" : "offline";

  // Uptime calculation
  unsigned long uptimeMillis = millis();
  time_t uptimeSeconds = uptimeMillis / 1000;
  time_t now = time(nullptr);

  // Check if the current time is valid
  if (now < 100000) { // Arbitrary threshold to detect invalid time
    Serial.println("[WARN] Invalid system time detected. Using fallback uptime.");
    now = uptimeSeconds; // Fallback to uptime seconds
  }

  time_t uptimeTimestamp = now - uptimeSeconds;
  char uptimeISO[32];
  strftime(uptimeISO, sizeof(uptimeISO), "%FT%TZ", gmtime(&uptimeTimestamp));

  // Publish diagnostic sensors
  mqtt.publish("homeassistant/sensor/sdm120ct/wifi_ip", wifiIP, true);
  mqtt.publish("homeassistant/sensor/sdm120ct/wifi_rssi", String(wifiRSSI), true);
  mqtt.publish("homeassistant/sensor/sdm120ct/wifi_signal_percentage", String(wifiSignalPercentage), true);
  mqtt.publish("homeassistant/sensor/sdm120ct/mac_address", macAddress, true);
  mqtt.publish("homeassistant/sensor/sdm120ct/ssid", wifiSSID, true);
  mqtt.publish("homeassistant/sensor/sdm120ct/bssid", wifiBSSID, true);
  mqtt.publish("homeassistant/sensor/sdm120ct/status", status, true);
  mqtt.publish("homeassistant/sensor/sdm120ct/uptime", uptimeISO, true);

  if (DEBUG_MQTT) Serial.println("[MQTT] Wi-Fi details published");
}


// binary sensor for inverter connection status
bool inverterConnected = false;
unsigned long lastPollTime = 0;

void publishInverterConnectionStatus(bool forcePublish = false) {
  static bool lastPublishedState = false;

  if (forcePublish || inverterConnected != lastPublishedState) {
    Serial.println("[MQTT] Publishing inverter connection status");
    String status = inverterConnected ? "ON" : "OFF";
    mqtt.publish("homeassistant/binary_sensor/sdm120ct/inverter_connected", status, true);
    lastPublishedState = inverterConnected;
  }
}

// Update connection status in Modbus handlers
void updateInverterConnectionStatus(bool forcePublish = false) {
  // Check if the inverter is connected based on the last poll time
  inverterConnected = (millis() - lastPollTime) <= INVERTER_CONNECTION_TIMEOUT_MS;

  // Publish the connection status only if it has changed or if forced
  publishInverterConnectionStatus(forcePublish);
}

// ========================= MQTT Callback =========================
void onConnectionEstablished() {
  if (DEBUG_MQTT) Serial.println("[MQTT] Connected to MQTT Broker");



  // Configure Arduino OTA
  if (DEBUG_MQTT) Serial.println("[MQTT] Configure Arduino OTA...");
  ArduinoOTA.setHostname("SDM120CT_EMULATOR");
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd updating.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  // Configure NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("[INFO] NTP server configured.");
  delay(5000); // Give it a moment for the time to sync the print out the time
  time_t tnow = time(nullptr);
  struct tm *ptm = gmtime(&tnow);
  Serial.printf("Current date (UTC) : %04d/%02d/%02d %02d:%02d/%02d - %s\n", ptm->tm_year + 1900, ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec, String(tnow, DEC).c_str());
  

  // Subscribe for meter values (JSON)
  mqtt.subscribe(mqtt_topic_subscribe, [](const String &payload) {
    if (DEBUG_MQTT) Serial.println("[MQTT] Data received: " + payload);

    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println("[MQTT] JSON parse error");
      return;
    }

    // Update registers with received values or default to zero/blank
    voltage = doc.containsKey("voltage") ? doc["voltage"] : 0.0;
    current = doc.containsKey("current") ? doc["current"] : 0.0;
    activePower = doc.containsKey("activePower") ? doc["activePower"] : 0.0;
    apparentPower = doc.containsKey("apparentPower") ? doc["apparentPower"] : 0.0;
    reactivePower = doc.containsKey("reactivePower") ? doc["reactivePower"] : 0.0;
    powerFactor = doc.containsKey("powerFactor") ? doc["powerFactor"] : 0.0;
    frequency = doc.containsKey("frequency") ? doc["frequency"] : 0.0;
    importEnergy = doc.containsKey("importEnergy") ? doc["importEnergy"] : 0.0;
    exportEnergy = doc.containsKey("exportEnergy") ? doc["exportEnergy"] : 0.0;

    Serial.println("[MQTT] Meter data updated from MQTT.");
  });

  // Subscribe for setup mode toggle
  mqtt.subscribe(mqtt_topic_setupmode, [](const String &payload) {
    setupMode = (payload == "ON" || payload == "1" || payload == "true");
    Serial.printf("[MQTT] Setup Mode: %s\n", setupMode ? "ENABLED" : "DISABLED");
    
    // Publish setup mode state
    mqtt.publish("homeassistant/meter/setupmode/state", setupMode ? "ON" : "OFF", true);
  });

  // Subscribe for restart command
  mqtt.subscribe("homeassistant/sensor/sdm120ct/restart", [](const String &payload) {
    if (payload == "restart") {
      Serial.println("[MQTT] Restart command received. Restarting...");
      ESP.restart();
    }
  });

  // Send MQTT Auto-Discovery configurations for Home Assistant
  if (DEBUG_MQTT) Serial.println("[MQTT] Publishing Home Assistant Auto-Discovery configurations...");
  
  delay(100);
  mqtt.publish("homeassistant/switch/sdm120ct_setup_mode/config", jsonDiscoverySetupMode, true);
  delay(100);

  
  // Diagnostic sensors
  mqtt.publish("homeassistant/sensor/sdm120ct_uptime/config", jsonDiscoveryUptime, true);
  delay(100);
  mqtt.publish("homeassistant/sensor/sdm120ct_wifi_ip/config", jsonDiscoveryWifiIP, true);
  delay(100);
  mqtt.publish("homeassistant/sensor/sdm120ct_mac_address/config", jsonDiscoveryMacAddress, true);
  delay(100);
  mqtt.publish("homeassistant/sensor/sdm120ct_wifi_ssid/config", jsonDiscoverySSID, true);
  delay(100);
  mqtt.publish("homeassistant/sensor/sdm120ct_wifi_bssid/config", jsonDiscoveryBSSID, true);
  delay(100);
  mqtt.publish("homeassistant/sensor/sdm120ct_wifi_rssi/config", jsonDiscoveryWifiRSSI, true);
  delay(100);
  mqtt.publish("homeassistant/sensor/sdm120ct_wifi_signal_percentage/config", jsonDiscoveryWifiSignalPercentage, true);
  delay(100);

  // Publish Restart Button Auto-Discovery
  mqtt.publish("homeassistant/button/sdm120ct_restart/config", jsonDiscoveryRestartButton, true);
  delay(100);

  // Publish Inverter Connection Status Auto-Discovery
  mqtt.publish("homeassistant/binary_sensor/sdm120ct_inverter_connected/config", jsonDiscoveryInverterConnected, true);
  delay(100);

  if (DEBUG_MQTT) Serial.println("[MQTT] Auto-Discovery configurations sent");



  // Publish initial states
  publishWifiDetails();
  
  // Publish initial setup mode state
  mqtt.publish("homeassistant/meter/setupmode/state", setupMode ? "ON" : "OFF", true);


  if (DEBUG_MQTT) Serial.println("[MQTT] Setup complete - Ready to go!");
}

// ========================= Modbus Handlers =========================
// Handle Read Input Registers (FC=04)
ModbusMessage handleRead(ModbusMessage request) {
  lastPollTime = millis();
  updateInverterConnectionStatus();

  uint16_t startAddr, words;
  request.get(2, startAddr);
  request.get(4, words);
  Serial.printf("[REQ INPUT READ] Addr=%u Words=%u\n", startAddr, words);

  ModbusMessage response;
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  for (uint16_t i = 0; i < words; i += 2) {
    uint16_t regAddr = startAddr + i;
    uint16_t hi = 0, lo = 0;

    // Fill known SDM120CT registers
    if (regAddr == 0x0000) floatToRegisters(voltage, &hi, &lo);
    else if (regAddr == 0x0006) floatToRegisters(current, &hi, &lo);
    else if (regAddr == 0x000C) floatToRegisters(activePower, &hi, &lo);
    else if (regAddr == 0x0012) floatToRegisters(apparentPower, &hi, &lo);
    else if (regAddr == 0x0018) floatToRegisters(reactivePower, &hi, &lo);
    else if (regAddr == 0x001E) floatToRegisters(powerFactor, &hi, &lo);
    else if (regAddr == 0x0024) floatToRegisters(phaseAngle, &hi, &lo);
    else if (regAddr == 0x0046) floatToRegisters(frequency, &hi, &lo);
    else if (regAddr == 0x0048) floatToRegisters(importEnergy, &hi, &lo);
    else if (regAddr == 0x004A) floatToRegisters(exportEnergy, &hi, &lo);
    else if (regAddr == 0x0156) floatToRegisters(totalActiveEnergy, &hi, &lo);
    else if (regAddr == 0x0158) floatToRegisters(totalReactiveEnergy, &hi, &lo);

    response.add(hi);
    response.add(lo);
  }

  printHexFrame(response);
  delay(50);  // Modbus RTU turnaround delay
  return response;
}

// Handle Read Holding Registers (FC=03)
ModbusMessage handleReadHolding(ModbusMessage request) {
  lastPollTime = millis();
  updateInverterConnectionStatus();

  uint16_t startAddr, words;
  request.get(2, startAddr);
  request.get(4, words);
  Serial.printf("[REQ HOLD READ] Addr=%u Words=%u\n", startAddr, words);

  ModbusMessage response;
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  for (uint16_t i = 0; i < words; i++) {
    uint16_t value = 0;
    if (startAddr + i == 0x0014) value = meterID;
    else if (startAddr + i == 0x001C) value = baudSetting;
    else if (startAddr + i == 0x0012) value = paritySetting;
    response.add(value);
  }

  printHexFrame(response);
  return response;
}

// Handle Write Holding Registers (FC=16)
ModbusMessage handleWriteHolding(ModbusMessage request) {
  lastPollTime = millis();
  updateInverterConnectionStatus();

  uint16_t startAddr, words;
  request.get(2, startAddr);
  request.get(4, words);

  Serial.printf("[REQ HOLD WRITE] Addr=%u Words=%u\n", startAddr, words);

  if (!setupMode) {
    Serial.println("[WARN] Write ignored - setup mode disabled.");
    ModbusMessage exception;
    exception.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_FUNCTION);
    return exception;
  }

  // Data starts here
  size_t index = 7;
  for (uint16_t i = 0; i < words; i++) {
    uint16_t value;
    request.get(index, value);
    index += 2;

    if (startAddr + i == 0x0014) {
      meterID = value;
      Serial.printf("Meter ID updated: %u\n", meterID);
    } else if (startAddr + i == 0x001C) {
      baudSetting = value;
      Serial.printf("Baud setting updated: %u\n", baudSetting);
    } else if (startAddr + i == 0x0012) {
      paritySetting = value;
      Serial.printf("Parity setting updated: %u\n", paritySetting);
    }
  }

  ModbusMessage response;
  response.add(request.getServerID(), request.getFunctionCode());
  response.add(startAddr);
  response.add(words);

  printHexFrame(response);
  return response;
}

// ========================= Setup =========================
void setup() {
  Serial.begin(115200);
  Serial.println("Starting SDM120CT Emulator with MQTT");

  mqtt.enableDebuggingMessages();
  mqtt.enableHTTPWebUpdater();
  mqtt.enableOTA(); // optional OTA


  // Increase the max packet size to handle large MQTT payloads
  mqtt.setMaxPacketSize(2048); // Set to a size larger than your longest payload

    // Set the Last Will and Testament (LWT)
  mqtt.enableLastWillMessage("homeassistant/sensor/sdm120ct/status", "offline", true);  // You can activate the retain flag by setting the third parameter to true

  // Register Modbus handlers
  MBserver.registerWorker(meterID, READ_INPUT_REGISTER, &handleRead);
  MBserver.registerWorker(meterID, READ_HOLD_REGISTER, &handleReadHolding);
  MBserver.registerWorker(meterID, WRITE_MULT_REGISTERS, &handleWriteHolding);

  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  MBserver.begin(Serial2);

  Serial.println("Modbus RTU server ready. Waiting for inverter...");




}

void loop() {
  mqtt.loop(); // EspMQTTClient handles Wi-Fi & MQTT internally
  ArduinoOTA.handle(); // Handle OTA updates
  
  // Periodic telemetry updates every 60 seconds
  static unsigned long lastTelemetryUpdate = 0;
  if (millis() - lastTelemetryUpdate > 60000) {
    lastTelemetryUpdate = millis();
    if (mqtt.isConnected()) {
      publishWifiDetails();
      updateInverterConnectionStatus(true);
    }
  }
}
