/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
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

typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t rx;
    uint16_t ry;
    uint16_t z : 10;
    uint8_t  : 6;
    uint16_t rz : 10;
    uint8_t  : 6;
    uint16_t buttons : 10;
    uint8_t  : 6;
    uint8_t  hat : 4;
    uint8_t  : 4;
    uint8_t  system_menu : 1;
    uint8_t  : 7;
    uint8_t  battery;
} __attribute__((packed)) inputReport_t;

typedef struct {
    uint8_t enable_actuators : 4;
    uint8_t : 4;
    uint8_t magnitude[4];
    uint8_t duration;
    uint8_t start_delay;
    uint8_t loop_count;
} __attribute__((packed)) outputReport_t;


#if CONFIG_BT_HID_DEVICE_ENABLED
static local_param_t s_bt_hid_param = {0};
// Example for 8 buttons, 1 hat switch, 2 axes (X and Y)
const uint8_t hid_descriptor[] = {
    0x05,0x01,0x09,0x05,0xA1,0x01,0xA1,0x00,0x09,0x30,0x09,0x31,0x15,0x00,0x27,0xFF,0xFF,
    0x00,0x00,0x95,0x02,0x75,0x10,0x81,0x02,0xC0,0xA1,0x00,0x09,0x33,0x09,0x34,0x15,0x00,
    0x27,0xFF,0xFF,0x00,0x00,0x95,0x02,0x75,0x10,0x81,0x02,0xC0,0x05,0x01,0x09,0x32,0x15,
    0x00,0x26,0xFF,0x03,0x95,0x01,0x75,0x0A,0x81,0x02,0x15,0x00,0x25,0x00,0x75,0x06,0x95,
    0x01,0x81,0x03,0x05,0x01,0x09,0x35,0x15,0x00,0x26,0xFF,0x03,0x95,0x01,0x75,0x0A,0x81,
    0x02,0x15,0x00,0x25,0x00,0x75,0x06,0x95,0x01,0x81,0x03,0x05,0x09,0x19,0x01,0x29,0x0A,
    0x95,0x0A,0x75,0x01,0x81,0x02,0x15,0x00,0x25,0x00,0x75,0x06,0x95,0x01,0x81,0x03,0x05,
    0x01,0x09,0x39,0x15,0x01,0x25,0x08,0x35,0x00,0x46,0x3B,0x01,0x66,0x14,0x00,0x75,0x04,
    0x95,0x01,0x81,0x42,0x75,0x04,0x95,0x01,0x15,0x00,0x25,0x00,0x35,0x00,0x45,0x00,0x65,
    0x00,0x81,0x03,0xA1,0x02,0x05,0x0F,0x09,0x97,0x15,0x00,0x25,0x01,0x75,0x04,0x95,0x01,
    0x91,0x02,0x15,0x00,0x25,0x00,0x91,0x03,0x09,0x70,0x15,0x00,0x25,0x64,0x75,0x08,0x95,
    0x04,0x91,0x02,0x09,0x50,0x66,0x01,0x10,0x55,0x0E,0x26,0xFF,0x00,0x95,0x01,0x91,0x02,
    0x09,0xA7,0x91,0x02,0x65,0x00,0x55,0x00,0x09,0x7C,0x91,0x02,0xC0,0x05,0x01,0x09,0x80,
    0xA1,0x00,0x09,0x85,0x15,0x00,0x25,0x01,0x95,0x01,0x75,0x01,0x81,0x02,0x15,0x00,0x25,
    0x00,0x75,0x07,0x95,0x01,0x81,0x03,0xC0,0x05,0x06,0x09,0x20,0x15,0x00,0x26,0xFF,0x00,
    0x75,0x08,0x95,0x01,0x81,0x02,0xC0
};



static esp_hid_raw_report_map_t bt_report_maps[] = {
    {
        .data = hid_descriptor,
        .len = sizeof(hid_descriptor)
    },
};

static esp_hid_device_config_t bt_hid_config = {
    .vendor_id          = 0x045E,                    // Microsoft Corp
    .product_id         = 0x028E,                    // Xbox 360 Wireless Controller (or 0x02FF for Xbox One S/Wireless)
    .version            = 0x0110,                    // Device version
    .device_name        = "Xbox Wireless Controller",
    .manufacturer_name  = "Microsoft",
    .serial_number      = "XBOX123456789",           // Example serial number
    .report_maps        = bt_report_maps,
    .report_maps_len    = 1
};

void send_gamepad_report(const inputReport_t *report) {
    esp_hidd_dev_input_set(
        s_bt_hid_param.hid_dev,
        0, // report map index
        0, // report ID (none defined in this descriptor)
        (uint8_t *)report,
        sizeof(inputReport_t)
    );
}

void bt_hid_demo_task(void *pvParameters)
{
    static const char* help_string =
        "########################################################################\n"
        "BT HID gamepad demo usage:\n"
        "This demo will periodically send gamepad reports without user input.\n"
        "########################################################################\n";
    printf("%s\n", help_string);

    inputReport_t report = {0};

    // Initialize axes to center
    report.x = 32768;  // 0-65535
    report.y = 32768;
    report.rx = 32768;
    report.ry = 32768;

    // Initialize hat switch (1-8, 0 = neutral)
    report.hat = 0;

    // Initialize battery
    report.battery = 100; // Full battery

    while (1) {
        // ----- Simulate a button press -----
        report.buttons = 0b0000000001; // Button 1 pressed
        send_gamepad_report(&report);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Release button
        report.buttons = 0;
        send_gamepad_report(&report);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // ----- Simulate joystick movements -----
        report.x = 16384; // Joystick left
        send_gamepad_report(&report);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        report.x = 49152; // Joystick right
        send_gamepad_report(&report);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        report.y = 16384; // Joystick up
        send_gamepad_report(&report);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        report.y = 49152; // Joystick down
        send_gamepad_report(&report);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // ----- Simulate hat switch -----
        report.hat = 1; // Up
        send_gamepad_report(&report);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        report.hat = 5; // Down
        send_gamepad_report(&report);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        report.hat = 0; // Neutral
        send_gamepad_report(&report);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Delay before next cycle
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}




void bt_hid_task_start_up(void)
{
    xTaskCreate(bt_hid_demo_task, "bt_hid_demo_task", 2 * 1024, NULL, configMAX_PRIORITIES - 3, &s_bt_hid_param.task_hdl);
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
#endif

void app_main(void)
{
    esp_err_t ret;
#if HID_DEV_MODE == HIDD_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID device");
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

#if CONFIG_BT_HID_DEVICE_ENABLED
    ESP_LOGI(TAG, "setting device name");
    esp_bt_dev_set_device_name(bt_hid_config.device_name);
    ESP_LOGI(TAG, "setting cod major, peripheral");
    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "setting bt device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&bt_hid_config, ESP_HID_TRANSPORT_BT, bt_hidd_event_callback, &s_bt_hid_param.hid_dev));
#endif
}
