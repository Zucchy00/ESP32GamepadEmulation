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
#include "esp_timer.h"

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

#define BIT_WRITE(byte, bit, val) \
    ((val) ? ((byte) |=  (1 << (bit))) : ((byte) &= ~(1 << (bit))))

static const char *TAG = "HID_DEV_DEMO";

int _count = 0;
/* initialize all axes to center (127) */
uint8_t _axisPosition[4] = {127, 127, 127, 127};
/* buttons as bools (all false) */
bool _buttonState[14] = {false};
/* triggers */
uint8_t _triggerPosition[2] = {0, 0};
int _hatDirection = 8;

static uint32_t crc32_table[256];

/* connection flag — set in event callback */
static volatile bool g_hid_connected = false;

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
#define MAX_BT_HID_SIZE 672

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
    0x05,0x01,
    0x09,0x05,
    0xA1,0x01,
    0x85,0x01,
    0x09,0x30,
    0x09,0x31,
    0x09,0x32,
    0x09,0x35,
    0x15,0x00,
    0x26,0xFF,0x00,
    0x75,0x08,
    0x95,0x04,
    0x81,0x02,
    0x09,0x39,
    0x15,0x00,
    0x25,0x07,
    0x35,0x00,
    0x46,0x3B,0x01,
    0x65,0x14,
    0x75,0x04,
    0x95,0x01,
    0x81,0x42,
    0x65,0x00,
    0x05,0x09,
    0x19,0x01,
    0x29,0x0E,
    0x15,0x00,
    0x25,0x01,
    0x75,0x01,
    0x95,0x0E,
    0x81,0x02,
    0x06,0x00,0xFF,
    0x09,0x20,
    0x75,0x06,
    0x95,0x01,
    0x15,0x00,
    0x25,0x7F,
    0x81,0x02,
    0x05,0x01,
    0x09,0x33,
    0x09,0x34,
    0x15,0x00,
    0x26,0xFF,0x00,
    0x75,0x08,
    0x95,0x02,
    0x81,0x02,
    0x06,0x00,0xFF,
    0x09,0x21,
    0x95,0x36,
    0x81,0x02,
    0x85,0x05,
    0x09,0x22,
    0x95,0x1F,
    0x91,0x02,
    0x85,0x04,
    0x09,0x23,
    0x95,0x24,
    0xB1,0x02,
    0x85,0x02,
    0x09,0x24,
    0x95,0x24,
    0xB1,0x02,
    0x85,0x08,
    0x09,0x25,
    0x95,0x03,
    0xB1,0x02,
    0x85,0x10,
    0x09,0x26,
    0x95,0x04,
    0xB1,0x02,
    0x85,0x11,
    0x09,0x27,
    0x95,0x02,
    0xB1,0x02,
    0x85,0x12,
    0x06,0x02,0xFF,
    0x09,0x21,
    0x95,0x0F,
    0xB1,0x02,
    0x85,0x13,
    0x09,0x22,
    0x95,0x16,
    0xB1,0x02,
    0x85,0x14,
    0x06,0x05,0xFF,
    0x09,0x20,
    0x95,0x10,
    0xB1,0x02,
    0x85,0x15,
    0x09,0x21,
    0x95,0x2C,
    0xB1,0x02,
    0x06,0x80,0xFF,
    0x85,0x80,
    0x09,0x20,
    0x95,0x06,
    0xB1,0x02,
    0x85,0x81,
    0x09,0x21,
    0x95,0x06,
    0xB1,0x02,
    0x85,0x82,
    0x09,0x22,
    0x95,0x05,
    0xB1,0x02,
    0x85,0x83,
    0x09,0x23,
    0x95,0x01,
    0xB1,0x02,
    0x85,0x84,
    0x09,0x24,
    0x95,0x04,
    0xB1,0x02,
    0x85,0x85,
    0x09,0x25,
    0x95,0x06,
    0xB1,0x02,
    0x85,0x86,
    0x09,0x26,
    0x95,0x06,
    0xB1,0x02,
    0x85,0x87,
    0x09,0x27,
    0x95,0x23,
    0xB1,0x02,
    0x85,0x88,
    0x09,0x28,
    0x95,0x22,
    0xB1,0x02,
    0x85,0x89,
    0x09,0x29,
    0x95,0x02,
    0xB1,0x02,
    0x85,0x90,
    0x09,0x30,
    0x95,0x05,
    0xB1,0x02,
    0x85,0x91,
    0x09,0x31,
    0x95,0x03,
    0xB1,0x02,
    0x85,0x92,
    0x09,0x32,
    0x95,0x03,
    0xB1,0x02,
    0x85,0x93,
    0x09,0x33,
    0x95,0x0C,
    0xB1,0x02,
    0x85,0xA0,
    0x09,0x40,
    0x95,0x06,
    0xB1,0x02,
    0x85,0xA1,
    0x09,0x41,
    0x95,0x01,
    0xB1,0x02,
    0x85,0xA2,
    0x09,0x42,
    0x95,0x01,
    0xB1,0x02,
    0x85,0xA3,
    0x09,0x43,
    0x95,0x30,
    0xB1,0x02,
    0x85,0xA4,
    0x09,0x44,
    0x95,0x0D,
    0xB1,0x02,
    0x85,0xA5,
    0x09,0x45,
    0x95,0x15,
    0xB1,0x02,
    0x85,0xA6,
    0x09,0x46,
    0x95,0x15,
    0xB1,0x02,
    0x85,0xF0,
    0x09,0x47,
    0x95,0x3F,
    0xB1,0x02,
    0x85,0xF1,
    0x09,0x48,
    0x95,0x3F,
    0xB1,0x02,
    0x85,0xF2,
    0x09,0x49,
    0x95,0x0F,
    0xB1,0x02,
    0x85,0xA7,
    0x09,0x4A,
    0x95,0x01,
    0xB1,0x02,
    0x85,0xA8,
    0x09,0x4B,
    0x95,0x01,
    0xB1,0x02,
    0x85,0xA9,
    0x09,0x4C,
    0x95,0x08,
    0xB1,0x02,
    0x85,0xAA,
    0x09,0x4E,
    0x95,0x01,
    0xB1,0x02,
    0x85,0xAB,
    0x09,0x4F,
    0x95,0x39,
    0xB1,0x02,
    0x85,0xAC,
    0x09,0x50,
    0x95,0x39,
    0xB1,0x02,
    0x85,0xAD,
    0x09,0x51,
    0x95,0x0B,
    0xB1,0x02,
    0x85,0xAE,
    0x09,0x52,
    0x95,0x01,
    0xB1,0x02,
    0x85,0xAF,
    0x09,0x53,
    0x95,0x02,
    0xB1,0x02,
    0x85,0xB0,
    0x09,0x54,
    0x95,0x3F,
    0xB1,0x02,
    0xC0
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

static void crc32_init_table(void) {
    const uint32_t poly = 0x04C11DB7;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i << 24;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80000000)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
        crc32_table[i] = crc;
    }
}

uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t idx = (crc >> 24) ^ data[i];
        crc = (crc << 8) ^ crc32_table[idx];
    }
    return crc;
}

void send_hid_report_fragmented(uint8_t *report, size_t len) {
    size_t offset = 0;
    uint8_t report_id = report[0];  // Use the real report ID (0x11)

    while (offset < len - 1) {  // exclude report_id byte from len
        size_t chunk = ((len - 1 - offset) > MAX_BT_HID_SIZE) ? MAX_BT_HID_SIZE : (len - 1 - offset);
        esp_hidd_dev_input_set(s_bt_hid_param.hid_dev, 0, report_id, report + 1 + offset, chunk);
        offset += chunk;
        vTaskDelay(pdMS_TO_TICKS(5)); // small delay to avoid congestion
    }
}

void send_gamepad_report(void) {
    if (!esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) return;

    static uint8_t counter = 0; // incrementing report counter

    uint8_t report[79] = {0};

    // --- Header ---
    report[0] = 0x11;  // Report ID
    report[1] = 0xC0;  // Constant
    report[2] = 0x00;  // Protocol-specific

    // --- Sticks (centered at 0x80) ---
    report[3] = 0x80; // Left Stick X
    report[4] = 0x80; // Left Stick Y
    report[5] = 0x80; // Right Stick X
    report[6] = 0x80; // Right Stick Y

    // --- D-Pad + face buttons ---
    report[7] = 0x08; // Neutral D-Pad
    BIT_WRITE(report[7], 4, _buttonState[0]); // Square
    BIT_WRITE(report[7], 5, _buttonState[1]); // Cross
    BIT_WRITE(report[7], 6, _buttonState[2]); // Circle
    BIT_WRITE(report[7], 7, _buttonState[3]); // Triangle

    // --- Shoulder + misc buttons ---
    report[8] = 0;
    BIT_WRITE(report[8], 0, _buttonState[4]); // L1
    BIT_WRITE(report[8], 1, _buttonState[5]); // R1
    BIT_WRITE(report[8], 2, _buttonState[6]); // L2 digital
    BIT_WRITE(report[8], 3, _buttonState[7]); // R2 digital
    BIT_WRITE(report[8], 4, _buttonState[8]); // Share
    BIT_WRITE(report[8], 5, _buttonState[9]); // Options
    BIT_WRITE(report[8], 6, _buttonState[10]); // L3
    BIT_WRITE(report[8], 7, _buttonState[11]); // R3

    // --- Counter + special buttons ---
    report[9]  = counter++;       // simple increment counter
    report[10] = 0x00;            // T-PAD / PS button (0 = released)

    // --- Triggers ---
    report[11] = _triggerPosition[0]; // Left Trigger (0-255)
    report[12] = _triggerPosition[1]; // Right Trigger (0-255)

    // --- Timestamp (2 bytes) ---
    report[13] = 0x00;
    report[14] = 0x00;

    // --- Battery / status ---
    report[15] = 0xFF; // battery full

    // --- Gyro: Angular velocity X,Y,Z ---
    report[16] = 0x00; report[17] = 0x00; // X
    report[18] = 0x00; report[19] = 0x00; // Y
    report[20] = 0x00; report[21] = 0x00; // Z

    // --- Acceleration X,Y,Z ---
    report[22] = 0x00; report[23] = 0x00; // X
    report[24] = 0x00; report[25] = 0x00; // Y
    report[26] = 0x00; report[27] = 0x00; // Z

    // --- Padding unknown bytes ---
    for (int i = 28; i <= 32; i++) report[i] = 0x00;

    // --- USB/Phone/Battery flags ---
    report[33] = 0x00;

    // --- More padding ---
    report[34] = 0x00;
    report[35] = 0x00;

    // --- Trackpad packets ---
    report[36] = 0x00; // number of packets
    report[37] = 0x00; // packet counter

    // Trackpad fingers (finger1 & finger2) - inactive
    for (int i = 38; i <= 72; i++) report[i] = 0x00;

    // --- Unknown bytes before CRC ---
    report[73] = 0x00;
    report[74] = 0x00;

    // --- CRC-32 over first 75 bytes ---
    uint32_t crc = crc32(report, 75);
    report[75] = (crc >> 24) & 0xFF;
    report[76] = (crc >> 16) & 0xFF;
    report[77] = (crc >> 8) & 0xFF;
    report[78] = crc & 0xFF;

    // --- Send the report in chunks ---
    send_hid_report_fragmented(report, sizeof(report));
}

void bt_hid_demo_task(void *pvParameters)
{
    static const char* help_string =
        "########################################################################\n"
        "BT HID PS4-style gamepad demo:\n"
        "This demo periodically sends gamepad reports using all controls.\n"
        "########################################################################\n";
    printf("%s\n", help_string);

    for (;;) {
        if (!g_hid_connected) {
            // Bail out cleanly if disconnected
            break;
        }

        if(esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) send_gamepad_report();

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Always delete yourself to free the handle
    s_bt_hid_param.task_hdl = NULL;
    vTaskDelete(NULL);
}

void bt_hid_task_start_up(void)
{
    if (s_bt_hid_param.task_hdl) {
        // already running
        return;
    }

    g_hid_connected = true;
    vTaskDelay(pdMS_TO_TICKS(50)); // small guard delay
    xTaskCreate(bt_hid_demo_task, "bt_hid_demo_task", 4096, NULL, 5, &s_bt_hid_param.task_hdl);
}

void bt_hid_task_shut_down(void)
{
    g_hid_connected = false;  // tell task to exit

    if (s_bt_hid_param.task_hdl) {
        // The task will see g_hid_connected == false and call vTaskDelete(NULL)
        // Wait briefly if you want to be sure it's gone
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


static void bt_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "EVENT_CALLBACK";

    ESP_LOGI(TAG, "base:%s id:%" PRId32 " (event:%d)", base, id, event);

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
            g_hid_connected = true;
            ESP_LOGI(TAG, "CONNECT OK");
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            bt_hid_task_start_up();
            set_rgb_color(0, 255, 0); // GREEN
            esp_hid_transport_t transport = esp_hidd_dev_transport_get(s_bt_hid_param.hid_dev);

            switch (transport) {
                case ESP_HID_TRANSPORT_BT:
                    printf("HID transport = BT Classic\n");
                    break;
                case ESP_HID_TRANSPORT_BLE:
                    printf("HID transport = BLE\n");
                    break;
                case ESP_HID_TRANSPORT_USB:
                    printf("HID transport = USB\n");
                    break;
                default:
                    printf("HID transport = Unknown (%d)\n", transport);
                    break;
            }

        } else {
            g_hid_connected = false;
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
    ESP_LOGI(TAG, "OUTPUT[%u]: usage=%s ID: %2u, Len: %d",
        param->output.map_index,
        esp_hid_usage_str(param->output.usage),
        param->output.report_id,
        param->output.length);

        if (param->output.length >= 11 && param->output.report_id == 0x11) {
            uint8_t *out = param->output.data;

            // Rumble
            uint8_t rumble_right = out[6]; // small motor
            uint8_t rumble_left  = out[7]; // big motor
            ESP_LOGI(TAG, "Rumble: left=%d, right=%d", rumble_left, rumble_right);

            // Lightbar RGB
            uint8_t r = out[8];
            uint8_t g = out[9];
            uint8_t b = out[10];
            ESP_LOGI(TAG, "Lightbar RGB: R=%d G=%d B=%d", r, g, b);
            set_rgb_color(r, g, b);

            // (Optional) LED fade / flash values in out[11..14]
        }

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
            g_hid_connected = false;
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
        g_hid_connected = false;
        ESP_LOGI(TAG, "STOP");
        set_rgb_color(0, 255, 255); // RED
        break;
    }
    default:
        break;
    }
    return;
}

static const char *sdp_event_to_str(esp_sdp_cb_event_t event)
{
    switch (event) {
        case ESP_SDP_INIT_EVT:              return "SDP INIT";
        case ESP_SDP_DEINIT_EVT:            return "SDP DEINIT";
        case ESP_SDP_SEARCH_COMP_EVT:       return "SDP SEARCH COMPLETE";
        case ESP_SDP_CREATE_RECORD_COMP_EVT:return "SDP CREATE RECORD COMPLETE";
        case ESP_SDP_REMOVE_RECORD_COMP_EVT:return "SDP REMOVE RECORD COMPLETE";
        default:                            return "UNKNOWN SDP EVENT";
    }
}

static void esp_sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    ESP_LOGI(TAG, "SDP callback: %s (%d)", sdp_event_to_str(event), event);

    switch (event) {
    case ESP_SDP_INIT_EVT:
        ESP_LOGI(TAG, "INIT: status=%d", param->init.status);
        if (param->init.status == ESP_SDP_SUCCESS) {
            // --- DIP / PnP Record for PS4 ---
            esp_bluetooth_sdp_dip_record_t dip_record = {
                .hdr = {
                    .type = ESP_SDP_TYPE_DIP_SERVER,
                },
                .vendor           = bt_hid_config.vendor_id,
                .vendor_id_source = ESP_SDP_VENDOR_ID_SRC_BT,
                .product          = bt_hid_config.product_id,
                .version          = bt_hid_config.version,
                .primary_record   = true,
            };
            esp_err_t err = esp_sdp_create_record((esp_bluetooth_sdp_record_t *)&dip_record);
            ESP_LOGI(TAG, "Creating DIP record, esp_sdp_create_record() returned: %s", esp_err_to_name(err));

            // --- HID Service Record for PS4 Controller ---
            esp_bluetooth_sdp_raw_record_t hid_raw_record = {
                .hdr = {
                    .type = ESP_SDP_TYPE_RAW,
                    .service_name_length = strlen(bt_hid_config.device_name),
                    .service_name = bt_hid_config.device_name,
                    .user1_ptr = (uint8_t *)controllerReportMap,
                    .user1_ptr_len = sizeof(controllerReportMap),
                    .rfcomm_channel_number = -1,
                    .l2cap_psm = 0x11,
                }
            };
            err = esp_sdp_create_record((esp_bluetooth_sdp_record_t *)&hid_raw_record);
            ESP_LOGI(TAG, "Creating HID record, esp_sdp_create_record() returned: %s", esp_err_to_name(err));
        }
        break;

    case ESP_SDP_CREATE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "CREATE RECORD COMPLETE: status=%d, handle=0x%x",
                 param->create_record.status, param->create_record.record_handle);
        break;

    case ESP_SDP_REMOVE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "REMOVE RECORD COMPLETE: status=%d", param->remove_record.status);
        break;

    case ESP_SDP_DEINIT_EVT:
        ESP_LOGI(TAG, "DEINIT: status=%d", param->deinit.status);
        break;

    case ESP_SDP_SEARCH_COMP_EVT:
        ESP_LOGI(TAG, "SEARCH COMPLETE: status=%d", param->search.status);
        break;

    default:
        ESP_LOGW(TAG, "Unhandled SDP event %d", event);
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
    char serial_number[13]; // 12 chars + null

    // --- Initialize peripherals ---
    init_ledc();
    set_rgb_color(255, 255, 255);  // white LED as startup indicator

    // --- Generate dynamic serial number ---
    generate_serial_number(serial_number, sizeof(serial_number));
    ESP_LOGI(TAGSERIAL, "Using serial number: %s", serial_number);

    // --- Initialize CRC table for PS4 report integrity ---
    crc32_init_table();

    // --- Initialize NVS ---
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // --- Initialize HID GAP ---
    ESP_LOGI(TAG, "Initializing HID GAP");
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_DEV_MODE));

    // --- Set device name & COD ---
    ESP_LOGI(TAG, "Setting device name and class of device");
    esp_bt_gap_set_device_name(bt_hid_config.device_name);

    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_PERIPHERAL_JOYSTICK;
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);

    vTaskDelay(pdMS_TO_TICKS(1000)); // small guard delay

    // --- Update serial in HID config BEFORE initializing HID ---
    bt_hid_config.serial_number = serial_number;

    // --- Initialize HID device ---
    ESP_LOGI(TAG, "Initializing HID device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&bt_hid_config, ESP_HID_TRANSPORT_BT, bt_hidd_event_callback, &s_bt_hid_param.hid_dev)
    );

    // --- Register SDP callback and initialize SDP ---
    ESP_LOGI(TAG, "Registering SDP callback and initializing SDP");
    ESP_ERROR_CHECK(esp_sdp_register_callback(esp_sdp_cb));
    ESP_ERROR_CHECK(esp_sdp_init());
}
