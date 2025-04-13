/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

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
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x05, // Usage (Gamepad)
    0xA1, 0x01, // Collection (Application)
    0x05, 0x09, // Usage Page (Button)
    0x19, 0x01, // Usage Minimum (Button 1)
    0x29, 0x08, // Usage Maximum (Button 8)
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x01, // Logical Maximum (1)
    0x95, 0x08, // Report Count (8)
    0x75, 0x01, // Report Size (1)
    0x81, 0x02, // Input (Data, Var, Abs)
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x39, // Usage (Hat switch)
    0x15, 0x00, // Logical Minimum (0)
    0x25, 0x07, // Logical Maximum (7)
    0x35, 0x00, // Physical Minimum (0)
    0x46, 0x3B, 0x01, // Physical Maximum (315)
    0x65, 0x14, // Unit (Eng Rot:Angular Pos)
    0x75, 0x04, // Report Size (4)
    0x95, 0x01, // Report Count (1)
    0x81, 0x02, // Input (Data, Var, Abs)
    0x75, 0x04, // Report Size (4)
    0x95, 0x01, // Report Count (1)
    0x81, 0x03, // Input (Cnst, Var, Abs)
    0x09, 0x30, // Usage (X)
    0x09, 0x31, // Usage (Y)
    0x15, 0x81, // Logical Minimum (-127)
    0x25, 0x7F, // Logical Maximum (127)
    0x75, 0x08, // Report Size (8)
    0x95, 0x02, // Report Count (2)
    0x81, 0x02, // Input (Data, Var, Abs)
    0xC0        // End Collection
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
void send_gamepad_report(uint8_t report_id, uint8_t buttons, uint8_t x_axis, uint8_t y_axis)
{
    // HID report buffer (size of 4 bytes)
    uint8_t buffer[4] = {0};  

    // Assign values to the report
    buffer[0] = report_id;       // Report ID (e.g., 1 for gamepad report)
    buffer[1] = buttons;         // Buttons pressed (bitmask)
    buffer[2] = x_axis;          // X-axis position (0-255)
    buffer[3] = y_axis;          // Y-axis position (0-255)

    // Send the report
    esp_hidd_dev_input_set(s_bt_hid_param.hid_dev, 0, 0, buffer, sizeof(buffer));
}

void bt_hid_demo_task(void *pvParameters)
{
    static const char* help_string = "########################################################################\n"\
    "BT HID gamepad demo usage:\n"\
    "This demo will periodically send gamepad reports without user input.\n"\
    "########################################################################\n";
    printf("%s\n", help_string);

    uint8_t buttons = 0; // Initial state of buttons
    uint8_t x_axis = 128; // Centered joystick X position
    uint8_t y_axis = 128; // Centered joystick Y position

    while (1) {
        // Simulate a button press and release
        buttons = 0b00000001; // Example button press (e.g., button 1)
        send_gamepad_report(0x01, buttons, x_axis, y_axis);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Simulate button press duration

        buttons = 0b00000000; // Release button
        send_gamepad_report(0x01, buttons, x_axis, y_axis);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Simulate joystick movements (left, right, up, down)
        x_axis = 64; // Joystick left
        send_gamepad_report(0x01, 0b00000000, x_axis, y_axis);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        x_axis = 192; // Joystick right
        send_gamepad_report(0x01, 0b00000000, x_axis, y_axis);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        y_axis = 64; // Joystick up
        send_gamepad_report(0x01, 0b00000000, x_axis, y_axis);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        y_axis = 192; // Joystick down
        send_gamepad_report(0x01, 0b00000000, x_axis, y_axis);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Delay for the next cycle
        vTaskDelay(200 / portTICK_PERIOD_MS);
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


void app_main(void)
{
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
