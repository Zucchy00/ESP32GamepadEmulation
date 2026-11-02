/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef ESP_HID_DEVICE_MAIN_H
#define ESP_HID_DEVICE_MAIN_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize and start the PS4 HID device
 * 
 * This function initializes all necessary components including:
 * - LED controller (LEDC)
 * - NVS flash
 * - Bluetooth HID GAP
 * - PS4 HID device profile
 * - SDP (Service Discovery Protocol)
 * 
 * This is the main entry point for the PS4 controller emulation.
 */
void start_ps4_hid(void);

/**
 * @brief Initialize LEDC for RGB LED control
 * 
 * Configures three LEDC channels for controlling an RGB LED
 * connected to the specified GPIO pins.
 */
void init_ledc(void);

/**
 * @brief Set RGB LED color
 * 
 * @param r Red component (0-255)
 * @param g Green component (0-255)
 * @param b Blue component (0-255)
 */
void set_rgb_color(uint8_t r, uint8_t g, uint8_t b);

typedef enum {
    DS4_HAT_N = 0,
    DS4_HAT_NE,
    DS4_HAT_E,
    DS4_HAT_SE,
    DS4_HAT_S,
    DS4_HAT_SW,
    DS4_HAT_W,
    DS4_HAT_NW,
    DS4_HAT_NONE
} ds4_hat_t;

/*
 * Controller state sampled by send_gamepad_report().
 * Axis order is LX, LY, RX, RY. Button indexes are Square, Cross, Circle,
 * Triangle, L1, R1, L2, R2, Options, L3, R3, Share, PS, Touchpad.
 * Trigger order is L2, R2. ADC/GPIO sampling overrides LX, LY, and Cross.
 */
extern uint8_t g_axis[4];
extern bool g_btn[14];
extern uint8_t g_trigger[2];
extern ds4_hat_t g_hat;

/**
 * @brief Send a PS4 gamepad HID report
 * 
 * Sends a complete PS4 controller report including:
 * - Analog stick positions
 * - Button states
 * - D-pad direction
 * - Trigger values
 * - Fixed gyro and accelerometer values for an idle controller
 */
void send_gamepad_report(void);

/**
 * @brief Send a repeating test cycle for all buttons and D-pad directions.
 *
 * Sweeps the left stick and ramps analog R2 over two seconds while pressing
 * each digital button and D-pad direction individually for 500 ms.
 */
void send_gamepad_report_test(void);

/**
 * @brief Calculate CRC32 checksum
 * 
 * Computes an IEEE CRC-32 checksum. Call crc32_init_table() first.
 * 
 * @param data Pointer to data buffer
 * @param len Length of data in bytes
 * @return uint32_t CRC32 checksum value
 */
uint32_t crc32(const uint8_t *data, size_t len);

/**
 * @brief Initialize CRC32 lookup table
 * 
 * Must be called once before using crc32() function.
 */
void crc32_init_table(void);

// LED GPIO pin definitions
#define RED_LED_PIN     25
#define GREEN_LED_PIN   26
#define BLUE_LED_PIN    27

#ifdef __cplusplus
}
#endif

#endif // ESP_HID_DEVICE_MAIN_H