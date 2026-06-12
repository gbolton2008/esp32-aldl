# ESP32 ALDL Wireless Bridge

**ESP32 native C firmware using ESP-IDF for reading GM 160-baud PWM ALDL data from a 1986–1988 Pontiac Fiero 2.8L V6 (1227170 ECM) and streaming it wirelessly over Bluetooth SPP.**

This project turns an ESP32 into a reliable, low-latency wireless ALDL interface. It decodes the raw 160-baud PWM signal in real time and forwards clean 25-byte data frames over Bluetooth to the companion Android app ([esp32-aldl-android](https://git.i3omb.com/gronod/esp32-aldl-android)).

It is implemented as a native ESP-IDF application in C, removing the overhead of the Arduino framework.

---

## Features

- **Native C ESP-IDF Implementation** — Built directly on ESP-IDF v5.2+ for maximum performance and low memory footprint
- **Real-time 160-baud PWM ALDL decoding** — Custom bit-banged decoder running in a high-priority FreeRTOS task (`aldlDecodeTask`)
- **Robust pulse handling** — Handles glitches, merged pulses, and idle gaps gracefully
- **Hard frame synchronization** — Prepends `0xAA 0x55` header for reliable packet alignment on the receiving side
- **Bluedroid Bluetooth Classic SPP** — Connects directly via Serial Port Profile (SPP) using "Just Works" Secure Simple Pairing (SSP) configuration
- **Decoupled RTOS Architecture** — ISR -> Ring Buffer -> Decoder task -> Bluetooth transmit queue -> BT TX task
- **Periodic status reporting** — Diagnostics reported over ESP-IDF log outputs (`ESP_LOGI`)

---

## Project Structure

```
esp32-aldl/
├── CMakeLists.txt         # Top-level CMake build configuration
├── sdkconfig.defaults     # Kconfig options (enabling BT, SPP, HZ=1000)
├── README.md              # Project documentation
└── main/
    ├── CMakeLists.txt     # Main component build script
    └── main.c             # C application logic (decoder and Bluetooth SPP)
```

---

## Supported Hardware

- **ECM**: GM 1227170 (1986–1988 Pontiac Fiero 2.8L V6)
- **ALDL Mode**: `$24` / `$24A` mask (25-byte continuous broadcast frames)
- **Microcontroller**: ESP32 (Classic ESP32 with Classic Bluetooth capability)
- **Bluetooth**: Classic Bluetooth (BR/EDR)

---

## Pin Connections

| Signal       | ESP32 Pin     | Notes                                         |
|--------------|---------------|-----------------------------------------------|
| ALDL Data In | **GPIO 4**    | Connect to ALDL pin **M** (data line)         |
| GND          | GND           | Must share ground with the car/ECU            |
| 5V / 3.3V    | —             | **Do not** power the ESP32 from the ALDL line |

> [!WARNING]
> The ALDL line is a 5V PWM signal. Since ESP32 GPIO pins are not 5V-tolerant, a simple voltage divider (e.g., 1kΩ + 2kΩ) or a bidirectional level shifter is required to step down the signal to 3.3V for long-term hardware reliability.

---

## Prerequisites & Installation

To build and compile the firmware, you need the official **ESP-IDF v5.2.1** toolchain installed.

1. Clone ESP-IDF (v5.2.1 stable release):
   ```bash
   mkdir -p ~/esp
   git clone -b v5.2.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
   ```

2. Run the toolchain installation:
   ```bash
   cd ~/esp/esp-idf
   ./install.sh esp32
   ```

3. Initialize the environment:
   ```bash
   source ~/esp/esp-idf/export.sh
   ```

4. *(Optional)* If `cmake` or `ninja` are missing in your environment, install them inside the active Python virtual environment:
   ```bash
   pip install cmake ninja
   ```

---

## Building and Flashing

Once the toolchain environment is loaded, execute the following commands in the `esp32-aldl` root workspace folder:

### 1. Set Build Target
```bash
idf.py set-target esp32
```
This command generates the build directories and loads settings from `sdkconfig.defaults` (enabling Bluetooth SPP and setting the FreeRTOS clock frequency to 1000 Hz).

### 2. Compile Project
```bash
idf.py build
```
This builds the bootloader, partition table, ESP-IDF drivers, Bluetooth stack, and the main application code, producing `build/esp32-aldl.bin`.

### 3. Flash & Monitor
Flash the firmware to the ESP32 and open the terminal console to view serial output (replace `/dev/ttyUSB0` with your target serial port):
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

To exit the serial monitor, press `Ctrl + ]`.

---

## How It Works

### Signal Decoding
The firmware registers an Interrupt Service Routine (ISR) triggered on any edge (both rising and falling) of **GPIO 4**. Pulse widths are calculated in microseconds using `esp_timer_get_time()` and pushed to a volatile ring buffer.

The high-priority `aldlDecodeTask` pops pulses from the ring buffer:
- Glitches under `300us` are discarded.
- Idle gaps over `13500us` reset the decoder.
- Pulses are classified into Logical `0` (approx. `1.11ms`), Logical `1` (approx. `4.16ms`), or Merged pulses.
- Decoded bits are reconstructed into 25-byte frames.

### Bluetooth Transmission
When a 25-byte frame is fully received, it is pushed to `bt_queue`. 
The `btTransmitTask` waits on the queue:
- Prepend the 2-byte hard-sync header (`0xAA 0x55`).
- Sends the packet (`27 bytes` total) over the Bluetooth Classic Serial Port Profile connection via the `esp_spp_write()` API.
- The device advertises itself under the classic Bluetooth name `ESP32-ALDL`.