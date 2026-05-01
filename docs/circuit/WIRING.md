# Circuit Wiring Notes

Reference the Fritzing diagram image in `/docs/circuit/wiring_diagram.jpg`
for the full visual layout. This file describes all connections in text form.

---

## ESP32 Pin Assignments

### Gas Sensors (Analog — ADC1 channel, 3.3V logic)

All MQ-series sensors use a breakout board with VCC, GND, AO (analog out),
and DO (digital out, unused here). Connect AO to the ESP32 ADC pin.
Use a 5V supply for VCC; the breakout board's comparator circuit
keeps AO within 0–3.3V for the ESP32 ADC.

| Sensor | ESP32 Pin | Notes |
|---|---|---|
| MQ-2  (LPG, smoke)   | GPIO 34 | ADC1_CH6, input only |
| MQ-4  (Methane)      | GPIO 35 | ADC1_CH7, input only |
| MQ-8  (Hydrogen)     | GPIO 32 | ADC1_CH4 |
| MQ-7  (CO)           | GPIO 33 | ADC1_CH5 |
| MQ-135 (NH3, NOx)    | GPIO 25 | ADC2_CH8 — do not use with WiFi |
| MQ-3  (Alcohol)      | GPIO 26 | ADC2_CH9 — do not use with WiFi |

> Note: ADC2 pins (GPIO 25, 26) cannot be used while WiFi is active on the
> ESP32. If WiFi is needed, move MQ-135 and MQ-3 to ADC1 pins (GPIO 32–39).

### PM2.5 Dust Sensor (GP2Y1010AU0F)

| Signal | ESP32 Pin | Notes |
|---|---|---|
| LED control (ILED) | GPIO 14 | Active LOW — digitalWrite LOW to pulse |
| Analog output (Vo) | GPIO 27 | ADC2_CH7 |
| VCC | 5V | Requires 5V supply |
| GND | GND | |

Add a 150Ω resistor and 220µF capacitor between VCC and GND on the sensor
as per datasheet recommendation to filter supply noise.

### DHT11 (Temperature & Humidity)

| Signal | ESP32 Pin |
|---|---|
| Data | GPIO 4 |
| VCC | 3.3V |
| GND | GND |

Add a 10kΩ pull-up resistor between Data and VCC.

### BMP280 (Pressure & Altitude) — I2C

| Signal | ESP32 Pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| VCC | 3.3V |
| GND | GND |
| SDO/ADDR | GND (sets I2C address to 0x76) |

### MPU6050 (IMU — Accelerometer + Gyroscope) — I2C

Shares the same I2C bus as BMP280.

| Signal | ESP32 Pin |
|---|---|
| SDA | GPIO 21 |
| SCL | GPIO 22 |
| VCC | 3.3V |
| GND | GND |
| AD0 | GND (sets I2C address to 0x68) |
| INT | GPIO 19 (optional interrupt pin) |

---

## ESP32 to Raspberry Pi UART Connection

The ESP32 UART0 (default Serial) transmits telemetry to the Raspberry Pi.

> IMPORTANT: The ESP32 is 3.3V logic. The Raspberry Pi GPIO is also 3.3V.
> Do NOT use a 5V-logic UART adapter between them.

| ESP32 | Raspberry Pi |
|---|---|
| TX0 (GPIO 1) | GPIO 15 / UART RX (Pin 10) |
| RX0 (GPIO 3) | GPIO 14 / UART TX (Pin 8) |
| GND | GND (Pin 6) |

On the Raspberry Pi, enable UART in `/boot/config.txt`:
```
enable_uart=1
dtoverlay=disable-bt
```
Then disable the serial console:
```
sudo systemctl disable serial-getty@ttyS0.service
```
The port will be available as `/dev/ttyS0` or `/dev/ttyAMA0`.

---

## Power Supply

| Component | Voltage | Current (approx) |
|---|---|---|
| ESP32 | 3.3V (onboard regulator from 5V USB) | ~240 mA peak |
| MQ sensors (x6) | 5V | ~150 mA each (900 mA total) |
| DHT11 | 3.3V | ~2.5 mA |
| BMP280 | 3.3V | ~1.1 mA |
| MPU6050 | 3.3V | ~3.9 mA |
| GP2Y1010 | 5V | ~20 mA |
| Raspberry Pi 3B | 5V | ~700 mA idle, ~1.2 A load |

Recommended: Use a dedicated 5V/3A BEC (battery eliminator circuit) from
the drone's LiPo battery for the sensor payload. Do not power the MQ sensors
directly from the ESP32 3.3V pin — the combined current draw will exceed
the onboard regulator limit.

---

## I2C Address Summary

| Device | I2C Address |
|---|---|
| BMP280 | 0x76 (SDO=GND) or 0x77 (SDO=VCC) |
| MPU6050 | 0x68 (AD0=GND) or 0x69 (AD0=VCC) |

Both devices can coexist on the same I2C bus (GPIO 21/22 on ESP32).
