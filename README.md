# GamePad Emulator

A Classic Bluetooth gamepad emulator built on ESP32 using ESP-IDF framework, designed for cross-platform compatibility.

## Project Status

This project is currently under active development. Contributions are welcome and appreciated.

## Overview

This project enables ESP32 devices to emulate a DualShock 4 gamepad using Classic Bluetooth, ensuring compatibility with most gaming platforms including PlayStation, PC, and mobile devices.

## Getting Started

### Prerequisites

- ESP-IDF framework installed and configured
- ESP32 development board
- USB cable for programming and power

### Project Structure

After creating your ESP-IDF project, organize the directory structure as follows:

```
PS4_CONTROLLER/
├── build/
├── components/
│   └── ps4_hid/
│       ├── build/
│       ├── main/
│       │   ├── driver/
│       │   ├── CMakeLists.txt
│       │   ├── esp_hid_device_main.c
│       │   ├── esp_hid_gap.c
│       │   └── esp_hid_gap.h
│       ├── Kconfig.projbuild
│       ├── .gitignore
│       ├── CMakeLists.txt
│       ├── README.md
│       ├── sdkconfig
│       ├── sdkconfig.defaults
│       ├── sdkconfig.defaults.esp32c3
│       ├── sdkconfig.defaults.esp32c6
│       ├── sdkconfig.defaults.esp32s3
│       └── sdkconfig.old
├── main/
├── CMakeLists.txt
├── sdkconfig
└── sdkconfig.old
```

Note: You may customize the directory names and structure, but ensure that CMakeLists.txt reflects any changes you make.

## Installation and Configuration

### Step 1: Create ESP-IDF Project

```bash
idf.py create-project your_project_name
```

### Step 2: Configure Controller Inputs

No source-code lines need to be commented out. On the original ESP32 target,
the controller task samples the left stick from ADC channels 6 and 7 (GPIO 34
and GPIO 35) and the Cross button from active-low GPIO 32. The remaining
controls are exposed as `g_axis`, `g_btn`, `g_trigger`, and `g_hat` in
`esp_hid_device_main.h`; the GPIO/ADC inputs take precedence for LX, LY, and
Cross.

The emulator sends 79-byte Bluetooth input reports and updates their CRC
before transmission. Its gyro and accelerometer values represent an idle
controller.

### Step 4: Build and Flash

```bash
idf.py build
idf.py flash
idf.py monitor
```

## Hardware Connection (Optional)

When ready to connect physical controls, the default pin mapping is:

- GPIO 34: Left Stick X-axis (ADC Channel 6)
- GPIO 35: Left Stick Y-axis (ADC Channel 7)
- GPIO 32: Cross button (with pull-up resistor)

To enable hardware support, uncomment the previously disabled functions and rebuild the project.

## Features

- DualShock 4 HID report emulation
- Classic Bluetooth connectivity
- Cross-platform compatibility
- Configurable button and axis mappings
- Support for multiple ESP32 variants (ESP32, ESP32-C3, ESP32-C6, ESP32-S3)


## Support

For questions, issues, or feature requests, please open an issue in the project repository.

## Acknowledgments

Built using the ESP-IDF framework by Espressif Systems.