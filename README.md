# ESP32 ALDL Wireless Bridge

**ESP32 firmware for reading GM 160-baud PWM ALDL data from a 1986–1988 Pontiac Fiero 2.8L V6 (1227170 ECM) and streaming it wirelessly over Bluetooth SPP.**

This project turns an ESP32 into a reliable, low-latency wireless ALDL interface. It decodes the raw 160-baud PWM signal in real time and forwards clean 25-byte data frames over Bluetooth to the companion Android app ([esp32-aldl-android](https://git.i3omb.com/gronod/esp32-aldl-android)).

---

## Features

- **Real-time 160-baud PWM ALDL decoding** — Custom bit-banged decoder running in a high-priority FreeRTOS task
- **Robust pulse handling** — Handles glitches, merged pulses, and idle gaps gracefully
- **Hard frame synchronization** — Prepends `0xAA 0x55` header for reliable packet alignment on the receiving side
- **Bluetooth SPP (Serial Port Profile)** — Appears as a classic Bluetooth serial device named `ESP32-ALDL`
- **Decoupled architecture** — ISR → Ring buffer → Decoder task → Bluetooth transmit queue
- **Status reporting** — Periodic status output over USB serial
- **Low CPU usage** on the decoder core while maintaining timing accuracy

---

## Supported Hardware

- **ECM**: GM 1227170 (1986–1988 Pontiac Fiero 2.8L V6)
- **ALDL Mode**: `$24` / `$24A` mask (25-byte continuous broadcast frames)
- **Microcontroller**: ESP32 (any dev board with sufficient GPIO — tested on ESP32-WROOM-32 and ESP32-S3)
- **Bluetooth**: Classic Bluetooth (BR/EDR) — works with most Android devices

---

## Pin Connections

| Signal       | ESP32 Pin     | Notes                                      |
|--------------|---------------|--------------------------------------------|
| ALDL Data In | **GPIO 4**    | Connect to ALDL pin **M** (data line)     |
| GND          | GND           | Must share ground with the car/ECU        |
| 5V / 3.3V    | —             | **Do not** power the ESP32 from the ALDL line |

> **Important**: The ALDL line is a 5V PWM signal. The ESP32 GPIO is 3.3V tolerant for input in most cases, but a simple voltage divider or level shifter (e.g., 1kΩ + 2kΩ) is recommended for long-term reliability.

---

## How It Works

### Signal Decoding
The firmware uses an interrupt service routine (ISR) triggered on any edge of GPIO 4. Pulse widths are measured using the ESP32 high-resolution timer and pushed into a lock-free ring buffer.

A dedicated decoder task (`aldlDecodeTask`) processes the pulse stream:

- Classifies pulses as `0`, `1`, glitch, merged pulse, or idle gap
- Reconstructs bytes using the standard 160-baud ALDL PWM encoding
- Assembles complete 25-byte frames
- Validates and enqueues valid frames

### Bluetooth Transmission
A separate task (`btTransmitTask`) pulls decoded frames from a FreeRTOS queue and transmits them over Bluetooth SPP with a 2-byte hard sync header: