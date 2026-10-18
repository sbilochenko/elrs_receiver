# ELRS Receiver

## Hardware

This project uses the following hardware:

- **Microcontroller:** ESP32 DevKit (ESP32-WROOM-32)
- **Radio receiver:** ELRS 915 MHz receiver (Bayck)
- **Communication protocol:** CRSF over UART
- **Receiver connection:** UART RX on ESP32 GPIO16
- **Power supply:** 5V

## Features

- Receives RC channel data from the ELRS receiver via CRSF.
- Provides a Wi-Fi access point for web-based joystick calibration.
- Supports calibration of minimum, center, and maximum channel values.
- Allows configuration of the joystick dead zone.
- Saves calibration settings in ESP32 non-volatile storage.
- Outputs normalized linear and angular control values over USB serial.

## Hardware Connections

### ELRS Receiver to ESP32

The ELRS 915 MHz receiver communicates with the ESP32 using the CRSF protocol over UART.

| ELRS Receiver Pin | ESP32 Pin | Description |
|---|---|---|
| `G` | `GND` | Ground |
| `V` | `5V` | Power supply |
| `T` | `GPIO16 (RX2)` | CRSF UART data to ESP32 |
| `R` | Not connected | UART transmit pin not used |

### ESP32 UART Configuration

- **UART interface:** `Serial2`
- **Baud rate:** `420000`
- **RX pin:** `GPIO16`
- **TX pin:** `GPIO17` (configured but not connected to the receiver)

### USB Serial Connection

The ESP32 connects to the Raspberry Pi 5 through USB.

- **Baud rate:** `115200`
- **Data format:** `RC,linear,angular,valid`
- **Purpose:** Sends normalized joystick commands to the ROS 2 node.

**Note:** The receiver's `T` pin is connected to the ESP32's UART RX pin because `T` transmits CRSF data from the receiver. The receiver's `R` pin is not connected in this configuration.