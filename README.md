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

### Step 2: Test Controller Inputs

The connected controller task runs `send_gamepad_report_test()`: it sweeps
all four axes of both sticks with offset two-second cycles, while analog R2
rises from 0% to 100% and analog L2 falls from 100% to 0%. At the same time,
it tests each of the 14 digital buttons in sequence, holding each for 500 ms,
then tests all eight D-pad directions and a neutral D-pad position. The
complete button and D-pad sequence repeats every 23 seconds. Both analog
triggers use the DS4's 0-255 range; the digital L2 and R2 buttons are tested
separately in their turns.

To use physical inputs instead, change the report task to call
`send_gamepad_report()`. On the original ESP32, that function reads the left
stick from ADC channels 6 and 7 (GPIO 34 and GPIO 35), and reads Cross from
active-low GPIO 32. The remaining controls are exposed as `g_axis`, `g_btn`,
`g_trigger`, and `g_hat` in `esp_hid_device_main.h`.

The emulator sends 79-byte Bluetooth input reports and updates their CRC
before transmission. Its gyro and accelerometer values represent an idle
controller. Bluedroid's HID L2CAP MTU is set to 128 bytes so the 78-byte
report payload is not rejected by the framework's default 64-byte limit.

ESP-IDF's HID device profile publishes the HID SDP record, including the
configured report descriptor. The application separately registers the Device
ID Profile record; it does not create a second raw HID SDP record.

The Bluetooth calibration Feature Report 0x05 uses a 40-byte payload, including
its CRC32, in addition to the report ID. Its calibration field order and CRC
are required by DS4 hosts. Gyroscope calibration values are selected to match
the DS4's 1024-count-per-degree-per-second scale.

The ESP32 RGB LED follows the DS4 Bluetooth output report's lightbar color when
the host marks that color valid. Connection-state colors do not override the
host-selected lightbar; the LED turns off when the HID connection ends.

After successful Bluetooth pairing, the firmware saves that host as the latest
paired device, removes older bonds, and attempts to reconnect to it when the HID
profile starts at boot. If no host was saved but exactly one Bluetooth bond
already exists, that bond is adopted as the reconnect target. With multiple
existing bonds and no saved target, pair the intended host once; after a
successful pairing, the other bonds are removed.

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