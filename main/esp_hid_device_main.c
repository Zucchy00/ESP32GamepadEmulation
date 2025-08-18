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

static uint8_t current_report_id = 0x01;

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


static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    // ... fill in the rest of 256 values or generate dynamically
};

// Simple CRC32 function
uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t idx = (crc >> 24) ^ data[i];
        crc = (crc << 8) ^ crc32_table[idx];
    }
    return crc;
}



void send_gamepad_report(void) {
    uint8_t report[79] = {0}; // 64 max over BT HID
    size_t len = 0;

    // Report header
    report[0] = 0xA1;              // DATA | INPUT
    report[1] = 0x11;

    // === Extended DS4 report ===
    report[2] = 0xC0; // Constant
    report[3] = 0x00; // Report ID (USB style)

    report[4] = 0x80; // LX
    report[5] = 0x80; // LY
    report[6] = 0x80; // RX
    report[7] = 0x80; // RY

    /*
    0x08 (8)	Released (no D-Pad pressed)
    0x00	Up (N)
    0x01	Up-Right (NE)
    0x02	Right (E)
    0x03	Down-Right (SE)
    0x04	Down (S)
    0x05	Down-Left (SW)
    0x06	Left (W)
    0x07	Up-Left (NW)
    */

    report[8] = 0x08;
    BIT_WRITE(report[8], 7, 0); // Triangle
    BIT_WRITE(report[8], 6, 0); // Circle
    BIT_WRITE(report[8], 5, 1); // Cross
    BIT_WRITE(report[8], 4, 0); // Square

    report[9] = 0;
    BIT_WRITE(report[9], 0, 0);  // L1
    BIT_WRITE(report[9], 1, 0);  // R1
    BIT_WRITE(report[9], 2, 0);  // L2
    BIT_WRITE(report[9], 3, 0);  // R2
    BIT_WRITE(report[9], 4, 0);  // Share
    BIT_WRITE(report[9], 5, 0);  // Options
    BIT_WRITE(report[9], 6, 0); // L3
    BIT_WRITE(report[9], 7, 0); // R3

    report[10] = (_count & 0x3F) << 2;
    BIT_WRITE(report[10], 0, 0); // PS
    BIT_WRITE(report[10], 1, 0); // Touchpad click

    report[11] = 0x00;
    report[12] = 0x00;

    uint16_t t = (uint16_t)(esp_timer_get_time() / 1000ULL);
    report[13] = t & 0xFF;
    report[14] = t >> 8;

    report[15] = 0xFF; // Battery example

    for (int i = 16; i <= 27; i++) report[i] = 0x00;

    // Unknown/zero bytes
    for (int i = 28; i <= 32; i++) report[i] = 0x00;

    // Misc byte
    report[33] = 0x00;

    // Unknown bytes
    report[34] = 0x00;
    report[35] = 0x00;

    // Trackpad packets
    for (int i = 36; i <= 72; i++) report[i] = 0x00;

    // Unknown small bytes
    report[73] = 0x00;
    report[74] = 0x00;

     // Compute CRC-32 for first 75 bytes
    uint32_t crc = crc32(report, 75);
    report[75] = (crc >> 24) & 0xFF;
    report[76] = (crc >> 16) & 0xFF;
    report[77] = (crc >> 8) & 0xFF;
    report[78] = crc & 0xFF;

    esp_hidd_dev_input_set(s_bt_hid_param.hid_dev, 0, 0x01, report, sizeof(report));

    _count++;
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

        send_gamepad_report();

        vTaskDelay(pdMS_TO_TICKS(300));
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
    vTaskDelay(pdMS_TO_TICKS(150)); // small guard delay
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
        }
        break;

    case ESP_SDP_DEINIT_EVT:
        ESP_LOGI(TAG, "DEINIT: status=%d", param->deinit.status);
        break;

    case ESP_SDP_SEARCH_COMP_EVT:
        ESP_LOGI(TAG, "SEARCH COMPLETE: status=%d", param->search.status);
        break;

    case ESP_SDP_CREATE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "CREATE RECORD COMPLETE: status=%d, handle=0x%x",
                 param->create_record.status, param->create_record.record_handle);
        break;

    case ESP_SDP_REMOVE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "REMOVE RECORD COMPLETE: status=%d", param->remove_record.status);
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