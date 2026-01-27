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
// #include "esp_adc/adc_oneshot.h"
// adc_oneshot_unit_handle_t adc1_handle;


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
#define MAX_BT_HID_SIZE 52

// void init_hardware_pins(void) {
//     // 1. Initialize ADC Unit 1
//     adc_oneshot_unit_init_cfg_t init_config1 = {
//         .unit_id = ADC_UNIT_1,
//     };
//     adc_oneshot_new_unit(&init_config1, &adc1_handle);

//     // 2. Configure ADC Channels (Pin 34 = CH6, Pin 35 = CH7)
//     adc_oneshot_chan_cfg_t config = {
//         .bitwidth = ADC_BITWIDTH_DEFAULT,
//         .atten = ADC_ATTEN_DB_12,
//     };
//     adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config); // Pin 34
//     adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config); // Pin 35

//     // 3. Setup Button Pin (GPIO 32)
//     gpio_config_t io_conf = {
//         .mode = GPIO_MODE_INPUT,
//         .pin_bit_mask = (1ULL << GPIO_NUM_32),
//         .pull_up_en = 1
//     };
//     gpio_config(&io_conf);
// }


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
const uint8_t ds4v1_hid_descriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x09, 0x30,        //   Usage (X)
    0x09, 0x31,        //   Usage (Y)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x04,        //   Report Count (4)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x05, 0x01,
    0x09, 0x33,        //   Usage (Rx)
    0x09, 0x34,        //   Usage (Ry)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data,Var,Abs)

    // Buttons
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (Button 1)
    0x29, 0x0E,        //   Usage Maximum (Button 14)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x0E,        //   Report Count (14)
    0x81, 0x02,        //   Input (Data,Var,Abs)

    // Dpad
    0x05, 0x01,
    0x09, 0x39,        //   Usage (Hat switch)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x07,        //   Logical Maximum (7)
    0x35, 0x00,        //   Physical Minimum (0)
    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
    0x65, 0x14,        //   Unit (Degrees)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x42,        //   Input (Data,Var,Abs,Null)
    0x65, 0x00,        //   Unit (None)

    // Padding
    0x75, 0x04,
    0x95, 0x01,
    0x81, 0x03,

    // Triggers
    0x05, 0x02,
    0x09, 0xC4,        //   Usage (Vibrator) – actually left trigger
    0x09, 0xC5,        //   Usage (Vibrator) – right trigger
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x02,
    0x81, 0x02,

    // Touchpad, gyro, accel (full DS4v1 block)
    // --- REQUIRED FOR LINUX + PS4 ACCEPTANCE ---
    0x06, 0x00, 0xFF,  
    0x09, 0x20,
    0x75, 0x06,
    0x95, 0x01,
    0x15, 0x00,
    0x25, 0x7F,
    0x81, 0x02,

    // Touch + sensor packet (36 bytes)
    0x06, 0x00, 0xFF,
    0x09, 0x21,
    0x95, 0x36,
    0x81, 0x02,

    // Output Report ID 0x02: rumble/LED
    0x85, 0x02,
    0x09, 0x22,
    0x95, 0x1F,
    0x91, 0x02,

    // Feature Report ID 0x05 (mandatory)
    0x85, 0x05,
    0x09, 0x23,
    0x95, 0x2F,
    0xB1, 0x02,

    0xC0              // End Collection
};

static esp_hid_raw_report_map_t bt_report_maps[] = {
    {
        .data = ds4v1_hid_descriptor,
        .len = sizeof(ds4v1_hid_descriptor)
    },
};

static esp_hid_device_config_t bt_hid_config = {
    .vendor_id          = 0x054C,
    .product_id         = 0x05C4,
    .version            = 0x0100,
    .device_name        = "Wireless Controller",
    .manufacturer_name  = "Sony",
    .serial_number      = "00000001",
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

/**
 * @brief Reads physical sensors and transmits a 79-byte DualShock 4 HID report.
 * Moves from static/random data to real-time ADC (joysticks) and GPIO (buttons).
 */

// void send_gamepad_report(void) {
//     if (!esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) return;

//     static uint8_t counter = 0;
//     int raw_x = 2048; // Default center
//     int raw_y = 2048; // Default center

//     // 1. READ REAL-TIME ADC VALUES
//     // ADC_CHANNEL_6 = GPIO 34 (X Axis)
//     // ADC_CHANNEL_7 = GPIO 35 (Y Axis)
//     adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw_x);
//     adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &raw_y);

//     // 2. SCALE 12-BIT (0-4095) TO 8-BIT (0-255)
//     // We divide by 16 because 4096 / 16 = 256
//     _axisPosition[0] = (uint8_t)(raw_x / 16); // Left Stick X
//     _axisPosition[1] = (uint8_t)(raw_y / 16); // Left Stick Y
    
//     // Right stick remains centered (127) for now
//     _axisPosition[2] = 127; 
//     _axisPosition[3] = 127;

//     // 3. READ PHYSICAL BUTTON (GPIO 32)
//     // If using a pull-up, 0 means the button is physically pressed to GND
//     _buttonState[1] = (gpio_get_level(GPIO_NUM_32) == 0); // Mapping to 'Cross' button

//     // 4. CONSTRUCT THE 79-BYTE PS4 REPORT
//     uint8_t report[79] = {0};

//     // Header
//     report[0] = 0x01;  // Report ID
//     report[1] = 0xC0;
//     report[2] = 0x00;

//     // Analog sticks
//     report[3] = _axisPosition[0];
//     report[4] = _axisPosition[1];
//     report[5] = _axisPosition[2];
//     report[6] = _axisPosition[3];

//     // D-Pad (8 = neutral) + Face buttons
//     int hat = 8; 
//     report[7] = hat;  
//     BIT_WRITE(report[7], 4, _buttonState[0]); // Square
//     BIT_WRITE(report[7], 5, _buttonState[1]); // Cross (Pin 32)
//     BIT_WRITE(report[7], 6, _buttonState[2]); // Circle
//     BIT_WRITE(report[7], 7, _buttonState[3]); // Triangle

//     // Shoulder & misc buttons
//     report[8] = 0;
//     for (int i = 4; i <= 11; i++) {
//         BIT_WRITE(report[8], i - 4, _buttonState[i]);
//     }

//     report[9]  = counter++; // Sequence counter
//     report[10] = 0x00;      // PS Button / Touchpad click

//     // Triggers (0-255)
//     report[11] = 0; // L2
//     report[12] = 0; // R2

//     // Battery status (0xFF = Full/Plugged in)
//     report[15] = 0xFF;

//     // Zero out the remaining bytes (Gyro, Accel, Touchpad)
//     for (int i = 16; i < 79; i++) report[i] = 0x00;

//     // 5. SEND DATA
//     send_hid_report_fragmented(report, sizeof(report));
// }

void send_gamepad_report(void) {
    if (!esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) return;

    static uint8_t counter = 0;

    // --- Randomize analog sticks ---
    _axisPosition[0] = rand() % 256; // Left X
    _axisPosition[1] = rand() % 256; // Left Y
    _axisPosition[2] = rand() % 256; // Right X
    _axisPosition[3] = rand() % 256; // Right Y

    // --- Randomize triggers ---
    _triggerPosition[0] = rand() % 256; // L2
    _triggerPosition[1] = rand() % 256; // R2

    // --- Randomize buttons ---
    for (int i = 0; i < 14; i++) {
        _buttonState[i] = rand() & 1;
    }

    // --- Randomize D-Pad safely ---
    int hat = rand() % 9; // 0–7 valid, 8 = neutral

    uint8_t report[79] = {0};  // must be initialized BEFORE assignment

    // --- Header ---
    report[0] = 0x01;  // Report ID
    report[1] = 0xC0;
    report[2] = 0x00;

    // --- Analog sticks ---
    report[3] = _axisPosition[0];
    report[4] = _axisPosition[1];
    report[5] = _axisPosition[2];
    report[6] = _axisPosition[3];

    // --- D-Pad + face buttons ---
    report[7] = hat;  // D-Pad value
    BIT_WRITE(report[7], 4, _buttonState[0]); // Square
    BIT_WRITE(report[7], 5, _buttonState[1]); // Cross
    BIT_WRITE(report[7], 6, _buttonState[2]); // Circle
    BIT_WRITE(report[7], 7, _buttonState[3]); // Triangle

    // --- Shoulder & misc buttons ---
    report[8] = 0;
    for (int i = 4; i <= 11; i++) {
        BIT_WRITE(report[8], i - 4, _buttonState[i]);
    }

    report[9]  = counter++;
    report[10] = 0x00; // PS / touchpad

    // --- Triggers ---
    report[11] = _triggerPosition[0];
    report[12] = _triggerPosition[1];

    // --- Battery full ---
    report[15] = 0xFF;

    // --- Zero out gyro, accel, trackpad, padding ---
    for (int i = 16; i < 79; i++) report[i] = 0x00;

    // --- Send the report ---
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

        vTaskDelay(pdMS_TO_TICKS(30));
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

// static const char *sdp_event_to_str(esp_sdp_cb_event_t event)
// {
//     switch (event) {
//         case ESP_SDP_INIT_EVT:              return "SDP INIT";
//         case ESP_SDP_DEINIT_EVT:            return "SDP DEINIT";
//         case ESP_SDP_SEARCH_COMP_EVT:       return "SDP SEARCH COMPLETE";
//         case ESP_SDP_CREATE_RECORD_COMP_EVT:return "SDP CREATE RECORD COMPLETE";
//         case ESP_SDP_REMOVE_RECORD_COMP_EVT:return "SDP REMOVE RECORD COMPLETE";
//         default:                            return "UNKNOWN SDP EVENT";
//     }
// }

static const uint8_t ds4v1_sdp_record[] = {
    // Service Record Handle
    0x36, 0x00, 0x6D,

    // Attribute: ServiceClassIDList
    0x09, 0x00, 0x01,
    0x35, 0x03,
    0x19, 0x11, 0x24,

    // Attribute: ProtocolDescriptorList (HID Control, PSM 0x11)
    0x09, 0x00, 0x04,
    0x35, 0x0D,
    0x35, 0x06,
    0x19, 0x01, 0x00,
    0x09, 0x00, 0x11,
    0x35, 0x03,
    0x19, 0x00, 0x11,

    // Attribute: AdditionalProtocolDescriptorList (HID Interrupt, PSM 0x13)
    0x09, 0x00, 0x0D,
    0x35, 0x0F,
    0x35, 0x0D,
    0x35, 0x06,
    0x19, 0x01, 0x00,
    0x09, 0x00, 0x13,
    0x35, 0x03,
    0x19, 0x00, 0x11,

    // Attribute: ServiceName
    0x09, 0x01, 0x00,
    0x25, 0x13,
    'W','i','r','e','l','e','s','s',' ',
    'C','o','n','t','r','o','l','l','e','r',

    // Attribute: BluetoothProfileDescriptorList
    0x09, 0x02, 0x01,
    0x35, 0x08,
    0x35, 0x06,
    0x19, 0x11, 0x24,
    0x09, 0x01, 0x11,

    // HIDParserVersion
    0x09, 0x02, 0x02,
    0x09, 0x01, 0x11,

    // HIDDeviceSubclass
    0x09, 0x02, 0x03,
    0x08, 0x40,

    // HIDCountryCode
    0x09, 0x02, 0x04,
    0x08, 0x00,

    // HIDVirtualCable
    0x09, 0x02, 0x05,
    0x28, 0x01,

    // HIDReconnectInitiate
    0x09, 0x02, 0x06,
    0x28, 0x01,

    // Sony-required HID flags
    0x09, 0x02, 0x09,
    0x28, 0x01,

    0x09, 0x02, 0x0A,
    0x28, 0x01,

    // HIDDescriptorList (placeholder – real descriptor provided by HID stack)
    0x09, 0x02, 0x0D,
    0x35, 0x0C,
    0x35, 0x0A,
    0x08, 0x22,
    0x25, 0x07,
    0x00, 0x00, 0x00, 0x00
};


static void esp_sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    ESP_LOGI(TAG, "SDP callback: %d", event);

    switch (event) {

    case ESP_SDP_INIT_EVT: {
        ESP_LOGI(TAG, "SDP INIT status=%d", param->init.status);
        if (param->init.status != ESP_SDP_SUCCESS) return;

        //
        // 1. Create DS4 DIP Record
        //
        esp_bluetooth_sdp_dip_record_t dip_record = {
            .hdr = { .type = ESP_SDP_TYPE_DIP_SERVER },
            .vendor = 0x054C,
            .vendor_id_source = ESP_SDP_VENDOR_ID_SRC_BT,
            .product = 0x05C4,
            .version = 0x0100,
            .primary_record = true,
        };

        esp_err_t err = esp_sdp_create_record((esp_bluetooth_sdp_record_t *)&dip_record);
        ESP_LOGI(TAG, "Create DIP record: %s", esp_err_to_name(err));


        //
        // 2. Register Raw HID SDP Record
        //
        const char *service_name = "Wireless Controller";

        esp_bluetooth_sdp_raw_record_t raw = {
            .hdr = {
                .type = ESP_SDP_TYPE_RAW,
                .service_name = (char *)service_name,
                .service_name_length = strlen(service_name),

                .user1_ptr = (uint8_t *)ds4v1_sdp_record,  // pointer to your SDP record
                .user1_ptr_len = sizeof(ds4v1_sdp_record), // length of the SDP record

                .rfcomm_channel_number = -1,
                .l2cap_psm = 0x11
            }
        };




        err = esp_sdp_create_record((esp_bluetooth_sdp_record_t *)&raw);
        ESP_LOGI(TAG, "Create HID record: %s", esp_err_to_name(err));
        break;
    }

    case ESP_SDP_CREATE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "Record created: status=%d, handle=%x",
                 param->create_record.status,
                 param->create_record.record_handle);
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


void start_ps4_hid(void)
{
    char serial_number[13]; // 12 chars + null

    // --- Initialize peripherals ---
    init_ledc();
    // init_hardware_pins();  
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
