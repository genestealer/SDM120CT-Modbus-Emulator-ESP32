// Private.h - Example configuration file for secrets and settings
// Replace the placeholder values with your actual credentials and settings.
// Do NOT commit your real secrets to version control.

#ifndef PRIVATE_H
#define PRIVATE_H

// WiFi credentials
const char* secret_wifi_ssid = "YOUR_WIFI_SSID";
const char* secret_wifi_password = "YOUR_WIFI_PASSWORD";

// MQTT broker credentials
const char* secret_mqtt_server = "YOUR_MQTT_SERVER";
const char* secret_mqtt_username = "YOUR_MQTT_USERNAME";
const char* secret_mqtt_password = "YOUR_MQTT_PASSWORD";

// Optional: Local NTP server
#define secret_local_timeclock_server "pool.ntp.org"

// Optional: Default Modbus meter ID
#define DEFAULT_METER_ID 1

#endif // PRIVATE_H
