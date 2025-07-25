#include <Arduino.h>
#include <ModbusServerRTU.h>

using namespace Modbus;

// RS-485 pin config
#define RS485_TX 17
#define RS485_RX 16
#define RS485_RTS 4 // DE/RE control

// Meter ID
#define DEFAULT_METER_ID 1

// Modbus RTU server
ModbusServerRTU MBserver(2000, RS485_RTS); // 2000ms timeout

// --- SDM120CT Values ---
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

// Configurable holding registers (setup)
uint16_t meterID = DEFAULT_METER_ID; // Default ID
uint16_t baudSetting = 2;            // 2 = 9600 baud
uint16_t paritySetting = 0;          // 0 = None, 1 = Even, 2 = Odd

// Setup mode flag
bool setupMode = true; // Later controlled by MQTT

// Utility: Convert float to two Modbus registers (word-swapped)
void floatToRegisters(float value, uint16_t *regHi, uint16_t *regLo) {
  uint16_t regs[2];
  memcpy(regs, &value, sizeof(float));
  *regHi = regs[1]; // SDM120 word order: high word first
  *regLo = regs[0];
}

// Debug: Print response frame
void printHexFrame(ModbusMessage &msg) {
  Serial.print("[RESP HEX] ");
  for (size_t i = 0; i < msg.size(); i++) {
    Serial.printf("%02X ", msg[i]);
  }
  Serial.println();
}

// ----------------- Handlers -----------------

// Handle Read Input Registers (FC=04)
ModbusMessage handleRead(ModbusMessage request) {
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
    else if (regAddr == 0x004C) floatToRegisters(importReactiveEnergy, &hi, &lo);
    else if (regAddr == 0x004E) floatToRegisters(exportReactiveEnergy, &hi, &lo);
    else if (regAddr == 0x0156) floatToRegisters(totalActiveEnergy, &hi, &lo);
    else if (regAddr == 0x0158) floatToRegisters(totalReactiveEnergy, &hi, &lo);
    // Unknown registers remain zero (hi=0, lo=0)

    response.add(hi);
    response.add(lo);
  }

  // Debug
  // printHexFrame(response);

  // Add Modbus RTU timing gap before sending
 // Add turnaround delay before responding
  delay(50);  // 30 ms for inverter stability

  return response; // The library sends it
}

// Handle Read Holding Registers (FC=03)
ModbusMessage handleReadHolding(ModbusMessage request) {
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

  return response;
}

// Handle Write Holding Registers (FC=16)
ModbusMessage handleWriteHolding(ModbusMessage request) {
  uint16_t startAddr, words;
  request.get(2, startAddr);
  request.get(4, words);
  uint8_t byteCount;
  request.get(6, byteCount);

  Serial.printf("[REQ HOLD WRITE] Addr=%u Words=%u\n", startAddr, words);

  if (!setupMode) {
    Serial.println("[WARN] Write ignored - setup mode disabled.");
    // Respond with exception if setup mode is off
    ModbusMessage exception;
    exception.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_FUNCTION);
    return exception;
  }

  size_t index = 7; // First data byte
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

  return response;
}

// ----------------- Setup & Loop -----------------

void setup() {
  Serial.begin(115200);
  Serial.println("Starting SDM120CT Emulator with Setup Mode...");

  // Register Modbus workers
  MBserver.registerWorker(meterID, READ_INPUT_REGISTER, &handleRead);
  MBserver.registerWorker(meterID, READ_HOLD_REGISTER, &handleReadHolding);
  MBserver.registerWorker(meterID, WRITE_MULT_REGISTERS, &handleWriteHolding);

  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);
  MBserver.begin(Serial2);

  Serial.println("Modbus RTU server ready. Waiting for inverter...");
}

void loop() {
  static uint32_t lastUpdate = 0;
  if (millis() - lastUpdate > 3000) {
    lastUpdate = millis();
    // Simulate realistic changes
    voltage += random(-1, 2) * 0.1;
    current += random(-1, 2) * 0.003;
    activePower = voltage * current * powerFactor;
    apparentPower = voltage * current;
    reactivePower = 5.0 + random(0, 3);
    importEnergy += 0.001;

    Serial.printf("[Update] V=%.1fV I=%.3fA P=%.1fW E=%.3fkWh\n",
                  voltage, current, activePower, importEnergy);
  }
}
