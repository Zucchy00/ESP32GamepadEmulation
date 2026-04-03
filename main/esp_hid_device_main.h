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

/**
 * @brief Send a PS4 gamepad HID report
 * 
 * Sends a complete PS4 controller report including:
 * - Analog stick positions
 * - Button states
 * - D-pad direction
 * - Trigger values
 * - Gyroscope and accelerometer data (placeholder)
 */
void send_gamepad_report(void);

/**
 * @brief Send HID report in fragments
 * 
 * Splits large HID reports into smaller chunks to avoid
 * Bluetooth packet size limitations.
 * 
 * @param report Pointer to the report data
 * @param len Length of the report in bytes
 */
void send_hid_report_fragmented(uint8_t *report, size_t len);

/**
 * @brief Start the HID demo task
 * 
 * Creates a FreeRTOS task that periodically sends gamepad reports.
 * This function is called automatically when a device connects.
 */
void bt_hid_task_start_up(void);

/**
 * @brief Stop the HID demo task
 * 
 * Signals the demo task to exit and cleans up resources.
 * This function is called automatically when a device disconnects.
 */
void bt_hid_task_shut_down(void);

/**
 * @brief Calculate CRC32 checksum
 * 
 * Computes a CRC32 checksum for PS4 report integrity.
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

// Maximum Bluetooth HID packet size
#define MAX_BT_HID_SIZE 52

#ifdef __cplusplus
}
#endif

#endif // ESP_HID_DEVICE_MAIN_H