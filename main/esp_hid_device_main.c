/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include "driver/ledc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_sdp_api.h"

#include "esp_hidd.h"
#include "esp_hid_gap.h"

static const char *TAG = "HID_DEV_DEMO";

// Define LED pins and LEDC channels
#define RED_LED_PIN     25
#define GREEN_LED_PIN   26
#define BLUE_LED_PIN    27

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_R   LEDC_CHANNEL_0
#define LEDC_OUTPUT_G   LEDC_CHANNEL_1
#define LEDC_OUTPUT_B   LEDC_CHANNEL_2
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT // 8-bit resolution
#define LEDC_FREQUENCY  5000             // 5 kHz

void init_ledc()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[3] = {
        {
            .channel    = LEDC_OUTPUT_R,
            .duty       = 0,
            .gpio_num   = RED_LED_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = LEDC_OUTPUT_G,
            .duty       = 0,
            .gpio_num   = GREEN_LED_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        },
        {
            .channel    = LEDC_OUTPUT_B,
            .duty       = 0,
            .gpio_num   = BLUE_LED_PIN,
            .speed_mode = LEDC_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER
        }
    };

    for (int ch = 0; ch < 3; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }
}

void set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_R, 255 - r);
    ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_R);

    ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_G, 255 - g);
    ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_G);

    ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_B, 255 - b);
    ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_B);
}



typedef struct
{
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static local_param_t s_bt_hid_param = {0};
const unsigned char controllerReportMap[] = {
    0x05, 	0x01, 	 	 //Usage Page (Generic Desktop Controls) 	
    0x09, 	0x05, 	 	 //Usage (Game Pad) 	CA – A manual control or cursor device. A game pad minimally consists of a thumbactivated rocker switch that controls two axes (X and Y) and has four buttons. The rocker switch consists of four contact closures for up, down, right, and left.
    0xA1, 	0x01, 	 	 //Collection (Application) 	Start
    0x85, 	0x01, 	 	 //Report ID (1) 	Used by the FW to determine what data has been send or need to be transmitted
    0x09, 	0x30, 	 	 //Usage (X) 	DV – A linear translation in the X direction. Report values should increase as the control’s position is moved from left to right. (for sticks)
    0x09, 	0x31, 	 	 //Usage (Y) 	DV – (Y direction values should increase from far to near)
    0x09, 	0x32, 	 	 //Usage (Z) 	DV – (Z direction values should increase from high to low)
    0x09, 	0x35, 	 	 //Usage (Rz) 	DV – A rotation about the Z axis. Angular position report values follow the righthand rule.
    0x15, 	0x00, 	 	 //Logical Minimum (0) 	Value for sticks
    0x26, 	0xFF, 	0x00,// 	Logical Maximum (255) 	Value
    0x75, 	0x08, 	 	 //Report Size (8) 	8 bits per variable
    0x95, 	0x04, 	 	 //Report Count (4) 	4 variables
    0x81, 	0x02, 	 	 //Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position) 	send variables (bytes 01 to 04)
    0x09, 	0x39, 	 	 //Usage (Hat switch) 	DV – A specialized mechanical configuration of switches generating a variable value with a null state. The switches are arranged around a springloaded knob. When the knob is tilted in the direction of a switch, its contacts are closed. A typical example is four switches that are capable of generating information about four possible directions in which the knob can be tilted. Intermediate positions can also be decoded if the hardware allows two switches to be reported simultaneously. (for Dpad)
    0x15, 	0x00, 	 	 //Logical Minimum (0) 	Value (0=N)
    0x25, 	0x07, 	 	 //Logical Maximum (7) 	Value (7=NW)
    0x35, 	0x00, 	 	 //Physical Minimum (0) 	
    0x46, 	0x3B, 	0x01,// 	Physical Maximum (315) 	
    0x65, 	0x14, 	 	 //Unit (System: English Rotation, Length: Centimeter) 	
    0x75, 	0x04, 	 	 //Report Size (4) 	(4 bits)
    0x95, 	0x01, 	 	 //Report Count (1) 	1 variable (Dpad)
    0x81, 	0x42, 	 	 //Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State) 	send (byte 05 low nibble lsb)
    0x65, 	0x00, 	 	 //Unit (None) 	
    0x05, 	0x09, 	 	 //Usage Page (Button) 	face buttons
    0x19, 	0x01, 	 	 //Usage Minimum (0x01) 	first button
    0x29, 	0x0E, 	 	 //Usage Maximum (0x0E) 	last button (14)
    0x15, 	0x00, 	 	 //Logical Minimum (0) 	Value for each button (status: 0 or 1)
    0x25, 	0x01, 	 	 //Logical Maximum (1) 	
    0x75, 	0x01, 	 	 //Report Size (1) 	1 bit per button
    0x95, 	0x0E, 	 	 //Report Count (14) 	14 bits
    0x81, 	0x02, 	 	 //Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position) 	send (byte 05 high nibble msb, 8 bits of byte 06 and binary bit 0 & 1 of byte 07)
    0x06, 	0x00, 	0xFF,// 	Usage Page (Vendor Defined 0xFF00) 	
    0x09, 	0x20, 	 	 //Usage (0x20) 	Counter
    0x75, 	0x06, 	 	 //Report Size (6) 	6 bits
    0x95, 	0x01, 	 	 //Report Count (1) 	
    0x15, 	0x00, 	 	 //Logical Minimum (0) 	
    0x25, 	0x7F, 	 	 //Logical Maximum (127) 	Note: REPORT_SIZE (6) is too small for LOGICAL_MAXIMUM (127) which needs 7 bits
    0x81, 	0x02, 	 	 //Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position) 	send (byte 07: msb 6 bits)
    0x05, 	0x01, 	 	 //Usage Page (Generic Desktop Controls) 	Shoulderpads Trigger
    0x09, 	0x33, 	 	 //Usage (Rx) 	DV – A rotation about the X axis. Angular position report values follow the right hand rule (L2 trigger)
    0x09, 	0x34, 	 	 //Usage (Ry) 	DV – A rotation about the Z axis. Angular position report values follow the right hand rule (R2 trigger)
    0x15, 	0x00, 	 	 //Logical Minimum (0) 	
    0x26, 	0xFF, 	0x00,// 	Logical Maximum (255) 	
    0x75, 	0x08, 	 	 //Report Size (8) 	8 bits
    0x95, 	0x02, 	 	 //Report Count (2) 	2 bytes
    0x81, 	0x02, 	 	 //Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position) 	send bytes 08 & 09
    0x06, 	0x00, 	0xFF,// 	Usage Page (Vendor Defined 0xFF00) 	
    0x09, 	0x21, 	 	 //Usage (0x21) 	
    0x95, 	0x36, 	 	 //Report Count (54) 	
    0x81, 	0x02, 	 	 //Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position) 	
    0x85, 	0x05, 	 	 //Report ID (5) 	
    0x09, 	0x22, 	 	 //Usage (0x22) 	
    0x95, 	0x1F, 	 	 //Report Count (31) 	
    0x91, 	0x02, 	 	 //Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x04, 	 	 //Report ID (4) 	
    0x09, 	0x23, 	 	 //Usage (0x23) 	
    0x95, 	0x24, 	 	 //Report Count (36) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	DV – declared in an Input report and is used as a notification to the host that the contents of a specific Feature report has changed
    0x85, 	0x02, 	 	 //Report ID (2) 	
    0x09, 	0x24, 	 	 //Usage (0x24) 	
    0x95, 	0x24, 	 	 //Report Count (36) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x08, 	 	 //Report ID (8) 	
    0x09, 	0x25, 	 	 //Usage (0x25) 	
    0x95, 	0x03, 	 	 //Report Count (3) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x10, 	 	 //Report ID (16) 	
    0x09, 	0x26, 	 	 //Usage (0x26) 	
    0x95, 	0x04, 	 	 //Report Count (4) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x11, 	 	 //Report ID (17) 	
    0x09, 	0x27, 	 	 //Usage (0x27) 	
    0x95, 	0x02, 	 	 //Report Count (2) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x12, 	 	 //Report ID (18) 	
    0x06, 	0x02, 	0xFF,// 	Usage Page (Vendor Defined 0xFF02) 	
    0x09, 	0x21, 	 	 //Usage (0x21) 	
    0x95, 	0x0F, 	 	 //Report Count (15) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x13, 	 	 //Report ID (19) 	
    0x09, 	0x22, 	 	 //Usage (0x22) 	
    0x95, 	0x16, 	 	 //Report Count (22) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x14, 	 	 //Report ID (20) 	
    0x06, 	0x05, 	0xFF,// 	Usage Page (Vendor Defined 0xFF05) 	
    0x09, 	0x20, 	 	 //Usage (0x20) 	
    0x95, 	0x10, 	 	 //Report Count (16) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x15, 	 	 //Report ID (21) 	
    0x09, 	0x21, 	 	 //Usage (0x21) 	
    0x95, 	0x2C, 	 	 //Report Count (44) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x06, 	0x80, 	0xFF,// 	Usage Page (Vendor Defined 0xFF80) 	
    0x85, 	0x80, 	 	 //Report ID (128) 	
    0x09, 	0x20, 	 	 //Usage (0x20) 	
    0x95, 	0x06, 	 	 //Report Count (6) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x81, 	 	 //Report ID (129) 	
    0x09, 	0x21, 	 	 //Usage (0x21) 	
    0x95, 	0x06, 	 	 //Report Count (6) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x82, 	 	 //Report ID (130) 	
    0x09, 	0x22, 	 	 //Usage (0x22) 	
    0x95, 	0x05, 	 	 //Report Count (5) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x83, 	 	 //Report ID (131) 	
    0x09, 	0x23, 	 	 //Usage (0x23) 	
    0x95, 	0x01, 	 	 //Report Count (1) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x84, 	 	 //Report ID (132) 	
    0x09, 	0x24, 	 	 //Usage (0x24) 	
    0x95, 	0x04, 	 	 //Report Count (4) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x85, 	 	 //Report ID (133) 	
    0x09, 	0x25, 	 	 //Usage (0x25) 	
    0x95, 	0x06, 	 	 //Report Count (6) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x86, 	 	 //Report ID (134) 	
    0x09, 	0x26, 	 	 //Usage (0x26) 	
    0x95, 	0x06, 	 	 //Report Count (6) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x87, 	 	 //Report ID (135) 	
    0x09, 	0x27, 	 	 //Usage (0x27) 	
    0x95, 	0x23, 	 	 //Report Count (35) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x88, 	 	 //Report ID (136) 	
    0x09, 	0x28, 	 	 //Usage (0x28) 	
    0x95, 	0x22, 	 	 //Report Count (34) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x89, 	 	 //Report ID (137) 	
    0x09, 	0x29, 	 	 //Usage (0x29) 	
    0x95, 	0x02, 	 	 //Report Count (2) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x90, 	 	 //Report ID (144) 	
    0x09, 	0x30, 	 	 //Usage (X) 	DV – A linear translation in the X direction. Report values should increase as the control’s position is moved from left to right.
    0x95, 	0x05, 	 	 //Report Count (5) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x91, 	 	 //Report ID (145) 	
    0x09, 	0x31, 	 	 //Usage (Y) 	DV – A linear translation in the Y direction. Report values should increase as the control’s position is moved from far to near.
    0x95, 	0x03, 	 	 //Report Count (3) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x92, 	 	 //Report ID (146) 	
    0x09, 	0x32, 	 	 //Usage (Z) 	DV – A linear translation in the Z direction. Report values should increase as the control’s position is moved from high to low (Z).
    0x95, 	0x03, 	 	 //Report Count (3) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0x93, 	 	 //Report ID (147) 	
    0x09, 	0x33, 	 	 //Usage (Rx) 	DV – A rotation about the X axis. Angular position report values follow the righthand rule.
    0x95, 	0x0C, 	 	 //Report Count (12) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA0, 	 	 //Report ID (160) 	
    0x09, 	0x40, 	 	 //Usage (Vx) 	DV – A vector in the X direction. Report values should increase as the vector increases in the positive X direction (from left to right). Negative values represent vectors in the negative X direction.
    0x95, 	0x06, 	 	 //Report Count (6) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA1, 	 	 //Report ID (161) 	
    0x09, 	0x41, 	 	 //Usage (Vy) 	DV – A vector in the Y direction. Report values should increase as the vector increases in the positive Y direction (from far to near). Negative values represent vectors in the negative Y direction.
    0x95, 	0x01, 	 	 //Report Count (1) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA2, 	 	 //Report ID (162) 	
    0x09, 	0x42, 	 	 //Usage (Vz) 	DV – (Z direction from high to low)
    0x95, 	0x01, 	 	 //Report Count (1) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA3, 	 	 //Report ID (163) 	
    0x09, 	0x43, 	 	 //Usage (Vbrx) 	DV – A vector in the X direction relative to the body of an object. Report values should increase as the vector increases in the positive X direction (forward). Negative values represent vectors in the negative X direction. X is the “forward” axis for an object
    0x95, 	0x30, 	 	 //Report Count (48) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA4, 	 	 //Report ID (164) 	
    0x09, 	0x44, 	 	 //Usage (Vbry) 	DV – (Y direction to the right from an observer facing forward on the object)
    0x95, 	0x0D, 	 	 //Report Count (13) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA5, 	 	 //Report ID (165) 	
    0x09, 	0x45, 	 	 //Usage (Vbrz) 	DV – A vector in the Z direction relative to the body of an object. Report values should increase as the vector increases in the positive Z direction (down from an observer facing forward on the object). Negative values represent vectors in the negative Z direction.
    0x95, 	0x15, 	 	 //Report Count (21) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA6, 	 	 //Report ID (166) 	
    0x09, 	0x46, 	 	 //Usage (Vno) 	DV– A non oriented vector or value. The units define a physical measurement not related to a specific axis or orientation. An example would be pressure or temperature.
    0x95, 	0x15, 	 	 //Report Count (21) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xF0, 	 	 //Report ID (240) 	
    0x09, 	0x47, 	 	 //Usage (Feature Notification) 	
    0x95, 	0x3F, 	 	 //Report Count (63) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xF1, 	 	 //Report ID (241) 	
    0x09, 	0x48, 	 	 //Usage (Resolution Multiplier) 	DV– Defines a (if a device has the capability to vary the resolution of one or more of its controls) Resolution Multiplier for a (all) Control:
    0x95, 	0x3F, 	 	 //Report Count (63) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xF2, 	 	 //Report ID (242) 	
    0x09, 	0x49, 	 	 //Usage (0x49) 	
    0x95, 	0x0F, 	 	 //Report Count (15) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA7, 	 	 //Report ID (167) 	
    0x09, 	0x4A, 	 	 //Usage (0x4A) 	
    0x95, 	0x01, 	 	 //Report Count (1) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA8, 	 	 //Report ID (168) 	
    0x09, 	0x4B, 	 	 //Usage (0x4B) 	
    0x95, 	0x01, 	 	 //Report Count (1) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xA9, 	 	 //Report ID (169) 	
    0x09, 	0x4C, 	 	 //Usage (0x4C) 	
    0x95, 	0x08, 	 	 //Report Count (8) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xAA, 	 	 //Report ID (170) 	
    0x09, 	0x4E, 	 	 //Usage (0x4E) 	
    0x95, 	0x01, 	 	 //Report Count (1) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xAB, 	 	 //Report ID (171) 	
    0x09, 	0x4F, 	 	 //Usage (0x4F) 	
    0x95, 	0x39, 	 	 //Report Count (57) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xAC, 	 	 //Report ID (172) 	
    0x09, 	0x50, 	 	 //Usage (0x50) 	
    0x95, 	0x39, 	 	 //Report Count (57) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xAD, 	 	 //Report ID (173) 	
    0x09, 	0x51, 	 	 //Usage (0x51) 	
    0x95, 	0x0B, 	 	 //Report Count (11) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xAE, 	 	 //Report ID (174) 	
    0x09, 	0x52, 	 	 //Usage (0x52) 	
    0x95, 	0x01, 	 	 //Report Count (1) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xAF, 	 	 //Report ID (175) 	
    0x09, 	0x53, 	 	 //Usage (0x53) 	
    0x95, 	0x02, 	 	 //Report Count (2) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0x85, 	0xB0, 	 	 //Report ID (176) 	
    0x09, 	0x54, 	 	 //Usage (0x54) 	
    0x95, 	0x3F, 	 	 //Report Count (63) 	
    0xB1, 	0x02, 	 	 //Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Nonvolatile) 	
    0xC0                 //Collection 	End 
};




static esp_hid_raw_report_map_t bt_report_maps[] = {
    {
        .data = controllerReportMap,
        .len = sizeof(controllerReportMap)
    },
};

static esp_hid_device_config_t bt_hid_config = {
    .vendor_id          = 0x054C,
    .product_id         = 0x09cc,
    .version            = 0x0100,
    .device_name        = "Wireless Controller",
    .manufacturer_name  = "Sony",
    .serial_number      = "129110990184280",
    .report_maps        = bt_report_maps,
    .report_maps_len    = 1
};

// send the buttons, change in x, and change in y
void send_gamepad_report(
    uint16_t buttons,
    uint8_t hat,
    uint8_t lx, uint8_t ly,
    uint8_t rx, uint8_t ry,
    uint8_t l2, uint8_t r2
) {
    uint8_t report[10] = {0};

    report[0] = 0x01;                // Report ID
    report[1] = buttons & 0xFF;      // Buttons 0–7
    report[2] = (buttons >> 8) & 0x3F; // Buttons 8–13 (6 bits), 2 bits padding
    report[3] = hat & 0x0F;          // D-pad (4 bits), neutral = 0x08
    report[4] = lx;                  // Left Stick X
    report[5] = ly;                  // Left Stick Y
    report[6] = rx;                  // Right Stick X
    report[7] = ry;                  // Right Stick Y
    report[8] = l2;                  // Left Trigger (Rx)
    report[9] = r2;                  // Right Trigger (Ry)

    if (s_bt_hid_param.hid_dev && esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        esp_hidd_dev_input_set(s_bt_hid_param.hid_dev, 0, 0x01, report, sizeof(report));
    } else {
        ESP_LOGW(TAG, "HID device not connected. Skipping report.");
    }
}



void bt_hid_demo_task(void *pvParameters)
{
    static const char* help_string =
        "########################################################################\n"
        "BT HID PS4-style gamepad demo:\n"
        "This demo periodically sends gamepad reports using all controls.\n"
        "########################################################################\n";
    printf("%s\n", help_string);

    uint16_t buttons = 0;
    uint8_t hat = 0x08;  // Neutral (no D-pad direction)
    uint8_t lx = 128;    // Left stick center
    uint8_t ly = 128;
    uint8_t rx = 128;    // Right stick center
    uint8_t ry = 128;
    uint8_t l2 = 0;      // Trigger pressure
    uint8_t r2 = 0;

    while (1) {
        // Press X (b0)
        buttons = 0b0000000000000001;
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // Press Circle (b1)
        buttons = 0b0000000000000010;
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // Press Triangle (b3)
        buttons = 0b0000000000001000;
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // Press R1 (b5) + R2 full (analog)
        buttons = 0b0000000000100000;
        r2 = 255;
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // Simulate D-pad Up (hat = 0)
        buttons = 0;
        hat = 0x00;
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // Simulate D-pad Down (hat = 4)
        hat = 0x04;
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        // Move left stick left → right
        hat = 0x08; // Neutral
        lx = 0;     // Far left
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(150 / portTICK_PERIOD_MS);

        lx = 255;   // Far right
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(150 / portTICK_PERIOD_MS);

        // Move right stick up → down
        rx = 128;
        ry = 0;     // Up
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(150 / portTICK_PERIOD_MS);

        ry = 255;   // Down
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);
        vTaskDelay(150 / portTICK_PERIOD_MS);

        // Reset everything
        buttons = 0;
        hat = 0x08;
        lx = ly = rx = ry = 128;
        l2 = r2 = 0;
        send_gamepad_report(buttons, hat, lx, ly, rx, ry, l2, r2);

        vTaskDelay(400 / portTICK_PERIOD_MS);
    }
}

void bt_hid_task_start_up(void)
{
    xTaskCreate(bt_hid_demo_task, "bt_hid_demo_task", 2 * 1024, NULL, configMAX_PRIORITIES - 3, &s_bt_hid_param.task_hdl);
    return;
}

void bt_hid_task_shut_down(void)
{
    if (s_bt_hid_param.task_hdl) {
        vTaskDelete(s_bt_hid_param.task_hdl);
        s_bt_hid_param.task_hdl = NULL;
    }
}

static void bt_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BT";

    switch (event) {
    case ESP_HIDD_START_EVENT: {
        if (param->start.status == ESP_OK) {
            ESP_LOGI(TAG, "START OK");
            ESP_LOGI(TAG, "Setting to connectable, discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            set_rgb_color(0, 0, 255); // BLUE
        } else {
            ESP_LOGE(TAG, "START failed!");
            set_rgb_color(255, 0, 0); // RED
        }
        break;
    }
    case ESP_HIDD_CONNECT_EVENT: {
        if (param->connect.status == ESP_OK) {
            ESP_LOGI(TAG, "CONNECT OK");
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            bt_hid_task_start_up();
            set_rgb_color(0, 255, 0); // GREEN
        } else {
            ESP_LOGE(TAG, "CONNECT failed!");
            set_rgb_color(255, 0, 0); // RED
        }
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT: {
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT: {
        ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT: {
        ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT: {
        if (param->disconnect.status == ESP_OK) {
            ESP_LOGI(TAG, "DISCONNECT OK");
            bt_hid_task_shut_down();
            ESP_LOGI(TAG, "Setting to connectable, discoverable again");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            set_rgb_color(255, 255, 255); // WHITE
        } else {
            ESP_LOGE(TAG, "DISCONNECT failed!");
            set_rgb_color(255, 0, 0); // RED
        }
        break;
    }
    case ESP_HIDD_STOP_EVENT: {
        ESP_LOGI(TAG, "STOP");
        set_rgb_color(0, 255, 255); // RED
        break;
    }
    default:
        break;
    }
    return;
}

static void esp_sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    switch (event) {
    case ESP_SDP_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SDP_INIT_EVT: status:%d", param->init.status);
        if (param->init.status == ESP_SDP_SUCCESS) {
            esp_bluetooth_sdp_dip_record_t dip_record = {
                .hdr =
                    {
                        .type = ESP_SDP_TYPE_DIP_SERVER,
                    },
                .vendor           = bt_hid_config.vendor_id,
                .vendor_id_source = ESP_SDP_VENDOR_ID_SRC_BT,
                .product          = bt_hid_config.product_id,
                .version          = bt_hid_config.version,
                .primary_record   = true,
            };
            esp_sdp_create_record((esp_bluetooth_sdp_record_t *)&dip_record);
        }
        break;
    case ESP_SDP_DEINIT_EVT:
        ESP_LOGI(TAG, "ESP_SDP_DEINIT_EVT: status:%d", param->deinit.status);
        break;
    case ESP_SDP_SEARCH_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SDP_SEARCH_COMP_EVT: status:%d", param->search.status);
        break;
    case ESP_SDP_CREATE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SDP_CREATE_RECORD_COMP_EVT: status:%d, handle:0x%x", param->create_record.status,
                 param->create_record.record_handle);
        break;
    case ESP_SDP_REMOVE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SDP_REMOVE_RECORD_COMP_EVT: status:%d", param->remove_record.status);
        break;
    default:
        break;
    }
}

static const char *TAGSERIAL = "SerialNumber";

// Function to generate a casual serial number
static void generate_serial_number(char *serial_number, size_t len) {
    const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    srand(time(NULL));
    for (int i = 0; i < len - 1; i++) {
        serial_number[i] = charset[rand() % (sizeof(charset) - 1)];
    }
    serial_number[len - 1] = '\0';
    ESP_LOGI(TAGSERIAL, "Generated Serial Number: %s", serial_number);
}


void app_main(void)
{
    char serial_number[13];

    init_ledc();
    set_rgb_color(255, 255, 255);
    // Generate the serial number during runtime
    generate_serial_number(serial_number, sizeof(serial_number));

    // Set the serial number in the configuration
    bt_hid_config.serial_number = serial_number;
    esp_err_t ret;
#if HID_DEV_MODE == HIDD_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID device or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_DEV_MODE);
    ret = esp_hid_gap_init(HID_DEV_MODE);
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "setting device name");
    esp_bt_gap_set_device_name(bt_hid_config.device_name);
    ESP_LOGI(TAG, "setting cod major, peripheral");
    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_PERIPHERAL_JOYSTICK;
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "setting bt device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&bt_hid_config, ESP_HID_TRANSPORT_BT, bt_hidd_event_callback, &s_bt_hid_param.hid_dev));
    ESP_ERROR_CHECK(esp_sdp_register_callback(esp_sdp_cb));
    ESP_ERROR_CHECK(esp_sdp_init());
}