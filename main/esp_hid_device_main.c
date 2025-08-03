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

typedef struct
{
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static local_param_t s_bt_hid_param = {0};
const unsigned char controllerReportMap[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x05,       // Usage (Gamepad)
    0xA1, 0x01,       // Collection (Application)

    0x85, 0x01,       //   Report ID (1)

    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (Button 1)
    0x29, 0x0E,       //   Usage Maximum (Button 14)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x95, 0x0E,       //   Report Count (14)
    0x75, 0x01,       //   Report Size (1)
    0x81, 0x02,       //   Input (Data,Var,Abs)

    0x95, 0x02,       //   Report Count (2) - padding
    0x75, 0x01,
    0x81, 0x03,       //   Input (Cnst,Var,Abs)

    0x05, 0x01,       //   Usage Page (Generic Desktop)
    0x09, 0x39,       //   Usage (Hat switch)
    0x15, 0x00,
    0x25, 0x07,
    0x35, 0x00,
    0x46, 0x3B, 0x01, //   Physical Max (315)
    0x65, 0x14,       //   Unit (Eng Rot:Angular Pos)
    0x75, 0x04,
    0x95, 0x01,
    0x81, 0x02,       //   Input (Data,Var,Abs)

    0x75, 0x04,
    0x95, 0x01,
    0x81, 0x03,       //   Padding

    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x09, 0x32,       //   Usage (Z)         -> rightX
    0x09, 0x35,       //   Usage (Rz)        -> rightY
    0x09, 0x33,       //   Usage (Rx)        -> left trigger
    0x09, 0x34,       //   Usage (Ry)        -> right trigger

    0x15, 0x00,
    0x26, 0xFF, 0x00, //   Logical Max (255)
    0x75, 0x08,
    0x95, 0x06,
    0x81, 0x02,       //   Input (Data,Var,Abs)

    0xC0              // End Collection
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
    uint8_t report[9] = {0};

    report[0] = 0x01;                // Report ID
    report[1] = buttons & 0xFF;      // Buttons 0-7
    report[2] = (buttons >> 8) & 0x3F; // Buttons 8-13 (6 bits), upper 2 bits = 0
    report[3] = hat & 0x0F;          // D-pad (0-7), 0x08 = neutral
    report[4] = lx;                  // Left Stick X
    report[5] = ly;                  // Left Stick Y
    report[6] = rx;                  // Right Stick X
    report[7] = ry;                  // Right Stick Y
    report[8] = l2;                  // Left Trigger
    report[9] = r2;                  // Right Trigger

    esp_hidd_dev_input_set(s_bt_hid_param.hid_dev, 0, 0x01, report, sizeof(report));
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
        } else {
            ESP_LOGE(TAG, "START failed!");
        }
        break;
    }
    case ESP_HIDD_CONNECT_EVENT: {
        if (param->connect.status == ESP_OK) {
            ESP_LOGI(TAG, "CONNECT OK");
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            bt_hid_task_start_up();
        } else {
            ESP_LOGE(TAG, "CONNECT failed!");
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
        } else {
            ESP_LOGE(TAG, "DISCONNECT failed!");
        }
        break;
    }
    case ESP_HIDD_STOP_EVENT: {
        ESP_LOGI(TAG, "STOP");
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