# SDM120CT Modbus RTU Meter Emulator (ESP32 + MQTT + Home Assistant)

Eastron SDM120CT ESP32 Modbus RTU Emulator

> **Note:** This is a working proof of concept and may have bugs. Use with caution and report any issues you encounter.

## Project Motivation

This project exists to emulate the SDM120CT Modbus meter for use with my "Eaton xStorage Home" battery and inverter system. The Eaton xStorage Home expects to communicate with a compatible Modbus energy meter, but in my setup, I use this emulator to provide the required data and integration. For more details on the inverter and battery API, see my related project: [Eaton xStorage Home API Documentation](https://github.com/genestealer/eaton-xstorage-home-api-doc).

## Description

This project emulates the SDM120CT Modbus meter and integrates with Home Assistant via MQTT. It supports telemetry publishing, setup mode toggle, restart functionality, and inverter connection status.

## Note

For Meter 1, the inverter expects to see a negative value when the house is drawing power from the grid and a positive value only if the house is exporting energy.

## Features

- MQTT integration with Home Assistant
- Telemetry publishing
- Setup mode toggle
- Restart functionality
- Inverter connection status

## Example JSON

For Eaton xStorage: the meter values for current, activePower, apparentPower, and reactivePower must be positive when exporting (sending power out) and negative when importing (drawing power in).

```json
{
  "voltage": 230.0,
  "current": 1.5,
  "activePower": 345.0,
  "apparentPower": 360.0,
  "reactivePower": 25.0,
  "powerFactor": 0.96,
  "frequency": 50.0,
  "importEnergy": 20.0,
  "exportEnergy": 0.0
}
```

## Secrets and Configuration

This project uses a `Private.h` file for WiFi and MQTT credentials, as well as other configuration settings. An example file (`include/Private.example.h`) is provided. **Do not commit your real secrets to version control.** Copy `Private.example.h` to `Private.h` and fill in your own values.

## SDM120CT Registers Emulated

[SDM120 Modbus Protocol PDF](https://www.eastroneurope.com/images/uploads/products/protocol/SDM120-MODBUS_Protocol.pdf)

Register list cross-referenced using the "REGISTERS LIST FOR SDM DEVICES" table in the SDM Energy Meter library ([SDM.h](https://github.com/reaper7/SDM_Energy_Meter/blob/master/SDM.h#L103)), which covers SDM72, SDM120, SDM220, SDM230, SDM630, and DDM18SD Modbus Energy meters.

The following Modbus registers are emulated by this project for the SDM120CT:

**Input Registers (Function Code 04):**

- `0x0000` — Voltage
- `0x0006` — Current
- `0x000C` — Active Power
- `0x0012` — Apparent Power
- `0x0018` — Reactive Power
- `0x001E` — Power Factor
- `0x0024` — Phase Angle
- `0x0046` — Frequency
- `0x0048` — Import Energy
- `0x004A` — Export Energy
- `0x0156` — Total Active Energy
- `0x0158` — Total Reactive Energy

**Holding Registers (Function Code 03/16):**

- `0x0014` — Meter ID
- `0x001C` — Baud Setting
- `0x0012` — Parity Setting

Sources:
[eModbus library](https://github.com/eModbus/eModbus/tree/master)
[SDM Energy Meter project](https://github.com/reaper7/SDM_Energy_Meter/tree/master?tab=readme-ov-file#reading)
[SDM120 Modbus Protocol PDF](https://www.eastroneurope.com/images/uploads/products/protocol/SDM120-MODBUS_Protocol.pdf)

Register list cross-referenced using the "REGISTERS LIST FOR SDM DEVICES" table in the SDM Energy Meter library ([SDM.h](https://github.com/reaper7/SDM_Energy_Meter/blob/master/SDM.h#L103)), which covers SDM72, SDM120, SDM220, SDM230, SDM630, and DDM18SD Modbus Energy meters.


## Home Assistant Helpers for Inverter Demand Management

To dynamically control the `activePower` value sent to the inverter via MQTT, this setup uses three helpers created in Home Assistant. These helpers allow fine-tuning of the inverter demand logic to prevent grid export, limit discharge rates, and adapt to real-time battery output.

### Helper Overview

####  `input_number.maximum_inverter_power_limit`
- **Purpose:** Sets the **maximum allowed output** from the inverter to prevent over-discharge and protect battery cell balancing.
- **Parameters:**
  - Minimum: `0`
  - Maximum: `3700`
  - Step Size: `50`
  - Unit: `W`

####  `input_number.inverter_power_offset`
- **Purpose:** Adds a small positive or negative **bias** to influence inverter behavior.
  - When battery output is **below** the maximum limit, the offset is **added** to the demand to encourage discharge.
  - When battery output **exceeds** the limit, the offset is **negated** and sent as a negative value to reduce output.
- **Parameters:**
  - Minimum: `0`
  - Maximum: `200`
  - Step Size: `5`
  - Unit: `W`

####  `sensor.inverter_demand_power`
- **Purpose:** Template sensor that calculates the **final activePower value** to be published to the inverter via MQTT.
- **Logic:**
  - Uses real-time net power and battery output data.
  - Applies a configurable **offset** to bias the inverter into discharging just slightly more than actual need (to reduce grid export).
  - Caps the output using a configurable **maximum inverter power limit** to protect the battery from over-discharge.
  - If the battery is already over the power limit, sends a **negative value** to pull back inverter output.
  - Result is rounded to **1 decimal place** for stability and clarity.

- **Depends on:**
  - `sensor.solar_panels_current_net_power_consumption_watts`
  - `sensor.storage_battery_power_production`
  - `input_number.maximum_inverter_power_limit`
  - `input_number.inverter_power_offset`

- **Output:** Positive power demand in watts (or negative value to reduce inverter output).

#### 🧠 Template Logic (Jinja2):

```jinja2
{% set net_power = states('sensor.solar_panels_current_net_power_consumption_watts') | float %}
{% set battery_output = states('sensor.storage_battery_power_production') | float(0) %}
{% set limit = states('input_number.maximum_inverter_power_limit') | float(500) %}
{% set offset = states('input_number.inverter_power_offset') | float(100) %}

{% if battery_output > limit %}
  {{ (-1 * offset) | round(1) }}
{% else %}
  {% set adjusted_power = net_power - offset %}
  {% set max_allowable = limit - battery_output %}
  {% set demand = adjusted_power if adjusted_power > 0 else 0 %}
  {{ [demand, max_allowable] | min | round(1) }}
{% endif %}
```
### Example Use

The value from `sensor.inverter_demand_power` is used in the MQTT payload to populate the `activePower` field of the SDM120CT Modbus emulator. This ensures the inverter sees a load slightly greater than reality (preventing export) but never exceeds a safe discharge rate.
