#include <Arduino.h>
#include <ModbusServerRTU.h>

using namespace Modbus;

// RS-485 pin config
#define RS485_TX 17
#define RS485_RX 16
#define RS485_RTS 4 // DE/RE control

#define METER_ID  1  // Modbus Slave ID (1-247)

// Modbus RTU server
ModbusServerRTU MBserver(2000, RS485_RTS); // timeout = 2000ms

// --- SDM120CT Values for testing ---
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

// Utility: Convert float to two Modbus registers (word-swapped)
void floatToRegisters(float value, uint16_t *regHi, uint16_t *regLo) {
  uint16_t regs[2];
  memcpy(regs, &value, sizeof(float));
  *regHi = regs[1]; // SDM120 word order: high word first
  *regLo = regs[0];
}

void printHexFrame(ModbusMessage &msg) {
  Serial.print("[RESP HEX] ");
  for (size_t i = 0; i < msg.size(); i++) {
    Serial.printf("%02X ", msg[i]);
  }
  Serial.println();
}


// Handle Read Input Registers (FC=04)
ModbusMessage handleRead(ModbusMessage request) {
  uint16_t startAddr, words;
  request.get(2, startAddr);
  request.get(4, words);

  Serial.printf("[REQ] FC=%02X Addr=%u Words=%u\n", request.getFunctionCode(), startAddr, words);

  ModbusMessage response;
  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  // Prepare 72 registers (144 bytes)
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
  delay(20);  // 20 ms for inverter stability

  return response; // The library sends it
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting SDM120CT Full Emulator (72 registers)");

  MBserver.registerWorker(METER_ID, READ_INPUT_REGISTER, &handleRead);

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
