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
#include <stdbool.h>
#include "esp_random.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
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
#include "esp_adc/adc_oneshot.h"

adc_oneshot_unit_handle_t adc1_handle;

#define BIT_WRITE(byte, bit, val) \
    ((val) ? ((byte) |=  (1U << (bit))) : ((byte) &= ~(1U << (bit))))

static const char *TAG = "HID_DEV_DEMO";
static const char *TAGSERIAL = "SerialNumber";

int _count = 0;
uint8_t _axisPosition[4] = {127, 127, 127, 127};
bool _buttonState[14] = {false};
uint8_t _triggerPosition[2] = {0, 0};
int _hatDirection = 8;

static uint32_t crc32_table[256];
static volatile bool g_hid_connected = false;
static char g_serial_number[13] = "00000001";

// Define LED pins and LEDC channels
#define RED_LED_PIN     25
#define GREEN_LED_PIN   26
#define BLUE_LED_PIN    27

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_R   LEDC_CHANNEL_0
#define LEDC_OUTPUT_G   LEDC_CHANNEL_1
#define LEDC_OUTPUT_B   LEDC_CHANNEL_2
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY  5000

#define DS4_BT_INPUT_REPORT_ID   0x11
#define DS4_BT_OUTPUT_REPORT_ID  0x11
#define DS4_BT_FEATURE_CALIB_ID  0x05
#define DS4_BT_INPUT_TOTAL_LEN   79
#define DS4_BT_INPUT_PAYLOAD_LEN 78
#define DS4_BT_CRC_OFFSET        75

typedef struct
{
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static local_param_t s_bt_hid_param = {0};

void init_hardware_pins(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config)); // GPIO34
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config)); // GPIO35

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_32),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void init_ledc(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

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
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[ch]));
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

/*
 * Minimal DS4 BT-compatible descriptor for this implementation:
 * - Input Report 0x11
 * - Output Report 0x11
 * - Feature Report 0x05
 *
 * This is kept internally consistent with the packets we actually send.
 */
static const uint8_t ds4_bt_hid_descriptor[] = {
    0x05, 0x01,                    // Usage Page (Generic Desktop)
    0x09, 0x05,                    // Usage (Game Pad)
    0xA1, 0x01,                    // Collection (Application)

    // ----------------------------------------------------------
    // Input Report ID 0x11 (BT full-style report)
    // payload = 78 bytes after report ID
    // [0..1]  BT header
    // [2..5]  LX,LY,RX,RY
    // hat + face
    // shoulder/share/options/L3/R3
    // PS + touchpad click + counter
    // [9..10] L2,R2
    // [11..77] vendor/sensors/touch/CRC region
    // ----------------------------------------------------------
    0x85, 0x11,                    // Report ID (17)

    // 2 BT header bytes
    0x06, 0x00, 0xFF,              // Usage Page (Vendor 0xFF00)
    0x09, 0x20,                    // Usage 0x20
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x02,
    0x81, 0x02,                    // Input (Data,Var,Abs)

    // Sticks
    0x05, 0x01,                    // Usage Page (Generic Desktop)
    0x09, 0x30,                    // X
    0x09, 0x31,                    // Y
    0x09, 0x32,                    // Z
    0x09, 0x35,                    // Rz
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x04,
    0x81, 0x02,

    // Hat
    0x05, 0x01,
    0x09, 0x39,                    // Hat switch
    0x15, 0x00,
    0x25, 0x07,
    0x35, 0x00,
    0x46, 0x3B, 0x01,
    0x65, 0x14,
    0x75, 0x04,
    0x95, 0x01,
    0x81, 0x42,                    // Input (Data,Var,Abs,Null)
    0x65, 0x00,

    // Face buttons in upper nibble of same byte
    0x05, 0x09,                    // Usage Page (Button)
    0x19, 0x01,                    // Button 1
    0x29, 0x04,                    // Button 4
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x04,
    0x81, 0x02,

    // L1,R1,L2btn,R2btn,Share,Options,L3,R3
    0x19, 0x05,                    // Button 5
    0x29, 0x0C,                    // Button 12
    0x75, 0x01,
    0x95, 0x08,
    0x81, 0x02,

    // PS, touchpad click
    0x19, 0x0D,                    // Button 13
    0x29, 0x0E,                    // Button 14
    0x75, 0x01,
    0x95, 0x02,
    0x81, 0x02,

    // 6-bit counter
    0x06, 0x00, 0xFF,
    0x09, 0x21,
    0x15, 0x00,
    0x25, 0x3F,
    0x75, 0x06,
    0x95, 0x01,
    0x81, 0x02,

    // Analog triggers
    0x05, 0x02,                    // Simulation Controls
    0x09, 0xC4,                    // Accelerator
    0x09, 0xC5,                    // Brake
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x02,
    0x81, 0x02,

    // Remaining bytes: sensors, touch, padding, CRC
    0x06, 0x00, 0xFF,
    0x09, 0x22,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x43,                    // 67 bytes
    0x81, 0x02,

    // Output Report ID 0x11 (BT rumble/lightbar style)
    0x85, 0x11,
    0x06, 0x00, 0xFF,
    0x09, 0x23,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x4D,                    // 77 bytes after report ID
    0x91, 0x02,                    // Output (Data,Var,Abs)

    // Feature Report ID 0x05 (BT calibration block style)
    0x85, 0x05,
    0x06, 0x00, 0xFF,
    0x09, 0x24,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x28,                    // 40 bytes after report ID
    0xB1, 0x02,                    // Feature (Data,Var,Abs)

    0xC0                           // End Collection
};

static esp_hid_raw_report_map_t bt_report_maps[] = {
    {
        .data = ds4_bt_hid_descriptor,
        .len = sizeof(ds4_bt_hid_descriptor)
    },
};

static esp_hid_device_config_t bt_hid_config = {
    .vendor_id          = 0x054C,
    .product_id         = 0x05C4,
    .version            = 0x0100,
    .device_name        = "Wireless Controller",
    .manufacturer_name  = "Sony",
    .serial_number      = g_serial_number,
    .report_maps        = bt_report_maps,
    .report_maps_len    = 1
};

/* ---------- CRC32 LE (reflected) used by DS4 BT reports ---------- */
static void crc32_init_table(void)
{
    const uint32_t poly = 0xEDB88320U;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1U) ? ((crc >> 1) ^ poly) : (crc >> 1);
        }
        crc32_table[i] = crc;
    }
}

static uint32_t crc32_le_compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc32_table[(crc ^ data[i]) & 0xFFU];
    }
    return ~crc;
}

static void generate_serial_number(char *serial_number, size_t len)
{
    const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    srand((unsigned)time(NULL));
    for (size_t i = 0; i < len - 1; i++) {
        serial_number[i] = charset[rand() % (sizeof(charset) - 1)];
    }
    serial_number[len - 1] = '\0';
    ESP_LOGI(TAGSERIAL, "Generated Serial Number: %s", serial_number);
}

/* ---------- Raw SDP record builder ---------- */

static uint8_t ds4_sdp_record[1024];
static size_t ds4_sdp_record_len = 0;

static inline void sdp_put_u8(uint8_t **p, uint8_t v)
{
    *(*p)++ = v;
}

static inline void sdp_put_u16(uint8_t **p, uint16_t v)
{
    *(*p)++ = (uint8_t)((v >> 8) & 0xFF);
    *(*p)++ = (uint8_t)(v & 0xFF);
}

static inline void sdp_put_attr_id(uint8_t **p, uint16_t attr_id)
{
    sdp_put_u8(p, 0x09);
    sdp_put_u16(p, attr_id);
}

static inline void sdp_put_bool_attr(uint8_t **p, uint16_t attr_id, bool value)
{
    sdp_put_attr_id(p, attr_id);
    sdp_put_u8(p, 0x28);
    sdp_put_u8(p, value ? 0x01 : 0x00);
}

static inline void sdp_put_u8_attr(uint8_t **p, uint16_t attr_id, uint8_t value)
{
    sdp_put_attr_id(p, attr_id);
    sdp_put_u8(p, 0x08);
    sdp_put_u8(p, value);
}

static inline void sdp_put_u16_attr(uint8_t **p, uint16_t attr_id, uint16_t value)
{
    sdp_put_attr_id(p, attr_id);
    sdp_put_u8(p, 0x09);
    sdp_put_u16(p, value);
}

static inline void sdp_put_text_attr(uint8_t **p, uint16_t attr_id, const char *text)
{
    size_t len = strlen(text);
    sdp_put_attr_id(p, attr_id);

    if (len <= 255) {
        sdp_put_u8(p, 0x25);
        sdp_put_u8(p, (uint8_t)len);
    } else {
        sdp_put_u8(p, 0x26);
        sdp_put_u16(p, (uint16_t)len);
    }

    memcpy(*p, text, len);
    *p += len;
}

static size_t build_ds4_sdp_record(uint8_t *buf, size_t buf_size)
{
    if (buf_size < 512) {
        return 0;
    }

    uint8_t *p = buf;

    // Root sequence
    sdp_put_u8(&p, 0x36); // seq16
    uint8_t *root_len_ptr = p;
    p += 2;

    // 0x0001 ServiceClassIDList = HID
    sdp_put_attr_id(&p, 0x0001);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x03);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x1124);

    // 0x0004 ProtocolDescriptorList = L2CAP/HIDP control PSM 0x11
    sdp_put_attr_id(&p, 0x0004);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x0D);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x06);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x0100); // L2CAP
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0011); // HID control PSM
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x03);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x0011); // HIDP

    // 0x0005 BrowseGroupList
    sdp_put_attr_id(&p, 0x0005);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x03);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x1002);

    // 0x0006 LanguageBaseAttributeIDList
    sdp_put_attr_id(&p, 0x0006);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x09);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x656E); // "en"
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x006A); // UTF-8 MIBenum
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0100);

    // 0x0009 BluetoothProfileDescriptorList = HID 1.00
    sdp_put_attr_id(&p, 0x0009);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x08);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x06);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x1124);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0100);

    // 0x000D AdditionalProtocolDescriptorLists = interrupt PSM 0x13
    sdp_put_attr_id(&p, 0x000D);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x0F);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x0D);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x06);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x0100); // L2CAP
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0013); // HID interrupt PSM
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x03);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x0011); // HIDP

    // Strings
    sdp_put_text_attr(&p, 0x0100, "Wireless Controller");
    sdp_put_text_attr(&p, 0x0101, "DualShock 4 Compatible");
    sdp_put_text_attr(&p, 0x0102, "Sony");

    // HID attributes
    sdp_put_u16_attr(&p, 0x0200, 0x0100); // HIDDeviceReleaseNumber
    sdp_put_u16_attr(&p, 0x0201, 0x0111); // HIDParserVersion
    sdp_put_u8_attr (&p, 0x0202, 0x08);   // HIDDeviceSubclass
    sdp_put_u8_attr (&p, 0x0203, 0x00);   // HIDCountryCode
    sdp_put_bool_attr(&p, 0x0204, true);  // HIDVirtualCable
    sdp_put_bool_attr(&p, 0x0205, true);  // HIDReconnectInitiate

    // 0x0206 HIDDescriptorList
    sdp_put_attr_id(&p, 0x0206);
    sdp_put_u8(&p, 0x36); // seq16
    uint8_t *desc_list_len_ptr = p;
    p += 2;

    sdp_put_u8(&p, 0x36); // seq16
    uint8_t *desc_entry_len_ptr = p;
    p += 2;

    sdp_put_u8(&p, 0x08); // uint8
    sdp_put_u8(&p, 0x22); // Report descriptor

    if (sizeof(ds4_bt_hid_descriptor) <= 255) {
        sdp_put_u8(&p, 0x25);
        sdp_put_u8(&p, (uint8_t)sizeof(ds4_bt_hid_descriptor));
    } else {
        sdp_put_u8(&p, 0x26);
        sdp_put_u16(&p, (uint16_t)sizeof(ds4_bt_hid_descriptor));
    }

    memcpy(p, ds4_bt_hid_descriptor, sizeof(ds4_bt_hid_descriptor));
    p += sizeof(ds4_bt_hid_descriptor);

    size_t desc_entry_len = (size_t)(p - (desc_entry_len_ptr + 2));
    desc_entry_len_ptr[0] = (uint8_t)((desc_entry_len >> 8) & 0xFF);
    desc_entry_len_ptr[1]  = (uint8_t)(desc_entry_len & 0xFF);

    size_t desc_list_len = (size_t)(p - (desc_list_len_ptr + 2));
    desc_list_len_ptr[0] = (uint8_t)((desc_list_len >> 8) & 0xFF);
    desc_list_len_ptr[1]  = (uint8_t)(desc_list_len & 0xFF);

    // 0x0207 HIDLangIDBaseList
    sdp_put_attr_id(&p, 0x0207);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x08);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x06);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0409); // en-US
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0100);

    sdp_put_bool_attr(&p, 0x0208, false); // HIDSDPDisable
    sdp_put_bool_attr(&p, 0x0209, true);  // HIDBatteryPower
    sdp_put_bool_attr(&p, 0x020A, true);  // HIDRemoteWake
    sdp_put_u16_attr(&p, 0x020B, 0x0100); // HIDProfileVersion
    sdp_put_u16_attr(&p, 0x020C, 0x0C80); // HIDSupervisionTimeout
    sdp_put_bool_attr(&p, 0x020D, true);  // HIDNormallyConnectable
    sdp_put_bool_attr(&p, 0x020E, false); // HIDBootDevice
    sdp_put_u16_attr(&p, 0x020F, 0x0640); // HIDSSRHostMaxLatency
    sdp_put_u16_attr(&p, 0x0210, 0x0320); // HIDSSRHostMinTimeout

    size_t root_len = (size_t)(p - (root_len_ptr + 2));
    root_len_ptr[0] = (uint8_t)((root_len >> 8) & 0xFF);
    root_len_ptr[1]  = (uint8_t)(root_len & 0xFF);

    return (size_t)(p - buf);
}

/* ---------- HID send helpers ---------- */

static void send_hid_report(uint8_t *report, size_t len)
{
    if (!s_bt_hid_param.hid_dev || !esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        return;
    }

    if (len < 2) {
        return;
    }

    esp_err_t err = esp_hidd_dev_input_set(
        s_bt_hid_param.hid_dev,
        0,
        report[0],
        report + 1,
        len - 1
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_hidd_dev_input_set failed: %s", esp_err_to_name(err));
    }
}

static void ds4_fill_touch_notouch(uint8_t *base)
{
    memset(base, 0, 9);
    base[1] = 0x80;
    base[5] = 0x80;
}

void send_gamepad_report_random(void)
{
    if (!s_bt_hid_param.hid_dev || !esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        return;
    }

    static uint8_t counter = 0;
    static uint16_t timestamp = 0;

    _axisPosition[0] = (uint8_t)(esp_random() & 0xFF); // LX
    _axisPosition[1] = (uint8_t)(esp_random() & 0xFF); // LY
    _axisPosition[2] = (uint8_t)(esp_random() & 0xFF); // RX
    _axisPosition[3] = (uint8_t)(esp_random() & 0xFF); // RY

    for (int i = 0; i < 14; i++) {
        _buttonState[i] = ((esp_random() % 20) == 0);  // low probability press
    }

    _triggerPosition[0] = (uint8_t)(esp_random() & 0xFF); // L2
    _triggerPosition[1] = (uint8_t)(esp_random() & 0xFF); // R2

    uint8_t report[DS4_BT_INPUT_TOTAL_LEN] = {0};

    // Report ID
    report[0] = DS4_BT_INPUT_REPORT_ID;

    // BT header
    report[1] = 0xC0;
    report[2] = 0x00;

    // Sticks
    report[3] = _axisPosition[0];
    report[4] = _axisPosition[1];
    report[5] = _axisPosition[2];
    report[6] = _axisPosition[3];

    // Hat + face buttons
    report[7] = (uint8_t)(esp_random() % 9); // 0..7 directions, 8 neutral
    BIT_WRITE(report[7], 4, _buttonState[0]); // Square
    BIT_WRITE(report[7], 5, _buttonState[1]); // Cross
    BIT_WRITE(report[7], 6, _buttonState[2]); // Circle
    BIT_WRITE(report[7], 7, _buttonState[3]); // Triangle

    // L1,R1,L2btn,R2btn,Share,Options,L3,R3
    report[8] = 0x00;
    for (int i = 4; i <= 11; i++) {
        BIT_WRITE(report[8], i - 4, _buttonState[i]);
    }

    // PS + touchpad click + counter
    report[9] = 0x00;
    BIT_WRITE(report[9], 0, _buttonState[12]); // PS
    BIT_WRITE(report[9], 1, _buttonState[13]); // Touchpad click
    report[9] |= (uint8_t)((counter & 0x3F) << 2);
    counter++;

    // Analog triggers
    report[10] = _triggerPosition[0];
    report[11] = _triggerPosition[1];

    // Timestamp
    timestamp += 188;
    report[12] = (uint8_t)(timestamp & 0xFF);
    report[13] = (uint8_t)((timestamp >> 8) & 0xFF);

    // Temperature
    report[14] = 0x00;

    // Battery / cable flags
    report[32] = 0x1B;
    report[33] = 0x00;
    report[34] = 0x00;

    // Touch sample count
    report[35] = 0x01;

    for (int i = 0; i < 4; i++) {
        ds4_fill_touch_notouch(&report[36 + i * 9]);
    }

    // CRC over prefix 0xA1 + report[0..74]
    uint8_t crc_input[76];
    crc_input[0] = 0xA1;
    memcpy(&crc_input[1], report, DS4_BT_CRC_OFFSET);

    uint32_t crc = crc32_le_compute(crc_input, sizeof(crc_input));
    report[75] = (uint8_t)(crc & 0xFF);
    report[76] = (uint8_t)((crc >> 8) & 0xFF);
    report[77] = (uint8_t)((crc >> 16) & 0xFF);
    report[78] = (uint8_t)((crc >> 24) & 0xFF);

    send_hid_report(report, sizeof(report));
}


void send_gamepad_report(void)
{
    if (!s_bt_hid_param.hid_dev || !esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        return;
    }

    static uint8_t counter = 0;
    static uint16_t timestamp = 0;

    int raw_x = 2048;
    int raw_y = 2048;

    adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw_x);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &raw_y);

    _axisPosition[0] = (uint8_t)(raw_x >> 4); // LX
    _axisPosition[1] = (uint8_t)(raw_y >> 4); // LY
    _axisPosition[2] = 127;                   // RX
    _axisPosition[3] = 127;                   // RY

    _buttonState[1] = (gpio_get_level(GPIO_NUM_32) == 0); // Cross

    uint8_t report[DS4_BT_INPUT_TOTAL_LEN] = {0};

    // Report ID
    report[0] = DS4_BT_INPUT_REPORT_ID;

    // BT header
    report[1] = 0xC0;
    report[2] = 0x00;

    // Sticks
    report[3] = _axisPosition[0];
    report[4] = _axisPosition[1];
    report[5] = _axisPosition[2];
    report[6] = _axisPosition[3];

    // Hat + face buttons
    report[7] = 0x08;
    BIT_WRITE(report[7], 4, _buttonState[0]); // Square
    BIT_WRITE(report[7], 5, _buttonState[1]); // Cross
    BIT_WRITE(report[7], 6, _buttonState[2]); // Circle
    BIT_WRITE(report[7], 7, _buttonState[3]); // Triangle

    // L1,R1,L2btn,R2btn,Share,Options,L3,R3
    report[8] = 0x00;
    for (int i = 4; i <= 11; i++) {
        BIT_WRITE(report[8], i - 4, _buttonState[i]);
    }

    // PS + touchpad click + counter
    report[9] = 0x00;
    BIT_WRITE(report[9], 0, _buttonState[12]); // PS
    BIT_WRITE(report[9], 1, _buttonState[13]); // Touchpad click
    report[9] |= (uint8_t)((counter & 0x3F) << 2);
    counter++;

    // Analog triggers
    report[10] = _triggerPosition[0];
    report[11] = _triggerPosition[1];

    // Timestamp
    timestamp += 188;
    report[12] = (uint8_t)(timestamp & 0xFF);
    report[13] = (uint8_t)((timestamp >> 8) & 0xFF);

    // Temperature
    report[14] = 0x00;

    // Battery / cable flags
    report[32] = 0x1B;
    report[33] = 0x00;
    report[34] = 0x00;

    // Touch sample count
    report[35] = 0x01;

    for (int i = 0; i < 4; i++) {
        ds4_fill_touch_notouch(&report[36 + i * 9]);
    }

    // CRC over prefix 0xA1 + report[0..74]
    uint8_t crc_input[76];
    crc_input[0] = 0xA1;
    memcpy(&crc_input[1], report, DS4_BT_CRC_OFFSET);

    uint32_t crc = crc32_le_compute(crc_input, sizeof(crc_input));
    report[75] = (uint8_t)(crc & 0xFF);
    report[76] = (uint8_t)((crc >> 8) & 0xFF);
    report[77] = (uint8_t)((crc >> 16) & 0xFF);
    report[78] = (uint8_t)((crc >> 24) & 0xFF);

    send_hid_report(report, sizeof(report));
}


/* ---------- Demo task ---------- */

void bt_hid_demo_task(void *pvParameters)
{
    static const char* help_string =
        "########################################################################\n"
        "BT HID PS4-style gamepad demo:\n"
        "This demo periodically sends DS4-style BT reports.\n"
        "########################################################################\n";
    printf("%s\n", help_string);

    for (;;) {
        if (!g_hid_connected) {
            break;
        }

        if (esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
            send_gamepad_report_random();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_bt_hid_param.task_hdl = NULL;
    vTaskDelete(NULL);
}

void bt_hid_task_start_up(void)
{
    if (s_bt_hid_param.task_hdl) {
        return;
    }

    g_hid_connected = true;
    vTaskDelay(pdMS_TO_TICKS(50));
    xTaskCreate(bt_hid_demo_task, "bt_hid_demo_task", 4096, NULL, 5, &s_bt_hid_param.task_hdl);
}

void bt_hid_task_shut_down(void)
{
    g_hid_connected = false;
    if (s_bt_hid_param.task_hdl) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ---------- HIDD callback ---------- */

static void bt_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDD_START_EVENT:
        if (param->start.status == ESP_OK) {
            set_rgb_color(0, 0, 255);
        } else {
            set_rgb_color(255, 0, 0);
        }
        break;

    case ESP_HIDD_CONNECT_EVENT:
        if (param->connect.status == ESP_OK) {
            g_hid_connected = true;
            bt_hid_task_start_up();
            set_rgb_color(0, 255, 0);
        } else {
            g_hid_connected = false;
            set_rgb_color(255, 0, 0);
        }
        break;

    case ESP_HIDD_OUTPUT_EVENT:
        if (param->output.report_id >= 0x11 && param->output.report_id <= 0x19 && param->output.length >= 11) {
            uint8_t *out = param->output.data;
            set_rgb_color(out[8], out[9], out[10]);
        }
        break;

    case ESP_HIDD_FEATURE_EVENT:
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        g_hid_connected = false;
        bt_hid_task_shut_down();
        set_rgb_color(255, 255, 255);
        break;

    case ESP_HIDD_STOP_EVENT:
        g_hid_connected = false;
        set_rgb_color(0, 255, 255);
        break;

    default:
        break;
    }
}

/* ---------- SDP callback ---------- */

static void esp_sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    ESP_LOGI(TAG, "SDP callback: %d", event);

    switch (event) {
    case ESP_SDP_INIT_EVT: {
        ESP_LOGI(TAG, "SDP INIT status=%d", param->init.status);
        if (param->init.status != ESP_SDP_SUCCESS) {
            return;
        }

        // 1. PnP / DIP record
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

        // 2. Proper HID SDP record
        ds4_sdp_record_len = build_ds4_sdp_record(ds4_sdp_record, sizeof(ds4_sdp_record));
        if (!ds4_sdp_record_len) {
            ESP_LOGE(TAG, "Failed to build DS4 SDP record");
            return;
        }

        static char service_name[] = "Wireless Controller";

        esp_bluetooth_sdp_raw_record_t raw = {
            .hdr = {
                .type = ESP_SDP_TYPE_RAW,
                .service_name = service_name,
                .service_name_length = sizeof(service_name),
                .user1_ptr = ds4_sdp_record,
                .user1_ptr_len = ds4_sdp_record_len,
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

/* ---------- Entry ---------- */

void start_ps4_hid(void)
{
    // peripherals
    init_ledc();
    init_hardware_pins();
    set_rgb_color(255, 255, 255);

    // serial must persist after function returns
    generate_serial_number(g_serial_number, sizeof(g_serial_number));
    bt_hid_config.serial_number = g_serial_number;
    ESP_LOGI(TAGSERIAL, "Using serial number: %s", g_serial_number);

    // CRC table
    crc32_init_table();

    // NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // HID GAP
    ESP_LOGI(TAG, "Initializing HID GAP");
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_DEV_MODE));

    // Device name and class
    ESP_LOGI(TAG, "Setting device name and class of device");
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(bt_hid_config.device_name));

    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_PERIPHERAL_JOYSTICK;
    ESP_ERROR_CHECK(esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR));

    vTaskDelay(pdMS_TO_TICKS(500));

    // HID device init
    ESP_LOGI(TAG, "Initializing HID device");
    ESP_ERROR_CHECK(
        esp_hidd_dev_init(&bt_hid_config, ESP_HID_TRANSPORT_BT, bt_hidd_event_callback, &s_bt_hid_param.hid_dev)
    );

    // SDP init
    ESP_LOGI(TAG, "Registering SDP callback and initializing SDP");
    ESP_ERROR_CHECK(esp_sdp_register_callback(esp_sdp_cb));
    ESP_ERROR_CHECK(esp_sdp_init());
    vTaskDelay(pdMS_TO_TICKS(200));
}
