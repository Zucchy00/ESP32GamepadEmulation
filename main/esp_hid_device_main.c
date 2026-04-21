/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/*
 * ============================================================
 *  DS4-over-BT HID device — complete implementation
 * ============================================================
 *
 * Key design decisions
 * --------------------
 * 1. "Can not find report!" from BT_HIDD
 *    Bluedroid's hidd_conn.c emits this when it receives a GET_REPORT
 *    on the control channel for a report ID it cannot find in its
 *    internal table.  The internal table is built by parsing the HID
 *    descriptor; it SKIPS any report that starts under a vendor usage
 *    page (0xFF00).  Because our report 0x11 begins with vendor-page
 *    items the parser never registers it, so every GET_REPORT(0x11)
 *    logs the error.
 *
 *    Fix: open the descriptor with a standard Generic Desktop /
 *    Gamepad application collection BEFORE any vendor-page item is
 *    encountered.  With the Application collection tag in place the
 *    parser correctly registers report 0x11 and auto-responds.
 *    Vendor-page items still appear inside the collection for the
 *    bytes that need them.
 *
 * 2. Sending input reports
 *    esp_hidd_dev_input_set() calls the same broken report-lookup.
 *    We bypass it by calling esp_bt_hid_device_send_report() directly
 *    (interrupt channel, type INTRDATA).
 *
 * 3. Feature report 0x05 (calibration)
 *    The host sends GET_REPORT(FEATURE, 0x05) after connection.
 *    We respond in ESP_HIDD_FEATURE_EVENT (trans_type == GET_REPORT).
 *    Payload is 37 bytes matching 0x95,0x25 in the descriptor.
 *
 * 4. Output report 0x11 (lightbar + rumble)
 *    Received via ESP_HIDD_OUTPUT_EVENT.  Offsets inside the payload
 *    are per DS4-BT spec.
 *
 * 5. Rumble
 *    Two motors (right = high-freq, left = low-freq) driven with LEDC
 *    PWM on separate GPIO pins. Duty cycle = rumble value / 255.
 *
 * 6. Full button / axis / hat / trigger mapping
 *    All 14 buttons, 4 axes, 2 analog triggers, hat switch.
 *
 * 7. CRC32 on every BT input report
 *    Seed = 0xA1 || report[0..74], appended at report[75..78].
 *
 * DS4-BT input report layout (79 bytes, report ID = 0x11)
 * -------------------------------------------------------
 *  [0]      Report ID  = 0x11
 *  [1]      0xC0  (BT header flags)
 *  [2]      0x00
 *  [3]      LX
 *  [4]      LY
 *  [5]      RX
 *  [6]      RY
 *  [7]      hat(3:0) | square(4) | cross(5) | circle(6) | triangle(7)
 *  [8]      L1(0) R1(1) L2(2) R2(3) share(4) options(5) L3(6) R3(7)
 *  [9]      PS(0) tpad(1) counter(7:2)
 *  [10]     L2 analog
 *  [11]     R2 analog
 *  [12:13]  timestamp LE
 *  [14]     temperature
 *  [15:20]  gyro X/Y/Z  LE int16
 *  [21:26]  accel X/Y/Z LE int16
 *  [27:31]  reserved
 *  [32]     battery status
 *  [33:34]  reserved
 *  [35]     touch sample count
 *  [36:71]  4× touch sample (9 bytes each)
 *  [72:74]  reserved
 *  [75:78]  CRC32 LE
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
#include "esp_hidd_api.h"   /* esp_bt_hid_device_send_report() */

/* ------------------------------------------------------------------ */
/*  Compile-time configuration                                          */
/* ------------------------------------------------------------------ */

/* RGB LED (common-anode: 0 = full on, 255 = off) */
#define RED_LED_PIN     25
#define GREEN_LED_PIN   26
#define BLUE_LED_PIN    27

/* Rumble motors */
#define RUMBLE_R_PIN    18   /* right / high-freq */
#define RUMBLE_L_PIN    19   /* left  / low-freq  */

/* LEDC */
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_OUTPUT_R       LEDC_CHANNEL_0
#define LEDC_OUTPUT_G       LEDC_CHANNEL_1
#define LEDC_OUTPUT_B       LEDC_CHANNEL_2
#define LEDC_OUTPUT_RUM_R   LEDC_CHANNEL_3
#define LEDC_OUTPUT_RUM_L   LEDC_CHANNEL_4
#define LEDC_DUTY_RES       LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY      5000

/* DS4 protocol constants */
#define DS4_BT_INPUT_REPORT_ID   0x11
#define DS4_BT_INPUT_TOTAL_LEN   79     /* report ID + 78 payload bytes */
#define DS4_BT_CRC_OFFSET        75     /* first byte of CRC field       */
#define DS4_ACCEL_RES_PER_G      8192
#define DS4_ACCEL_Y_REST         (-DS4_ACCEL_RES_PER_G)
#define DS4_TEMPERATURE_DEFAULT  0x65

/* Output report 0x11 payload byte offsets (0-based within data[]) */
#define DS4_OUT_RUMBLE_R   5
#define DS4_OUT_RUMBLE_L   6
#define DS4_OUT_LED_R      7
#define DS4_OUT_LED_G      8
#define DS4_OUT_LED_B      9

/* Hat-switch values */
#define HAT_N   0
#define HAT_NE  1
#define HAT_E   2
#define HAT_SE  3
#define HAT_S   4
#define HAT_SW  5
#define HAT_W   6
#define HAT_NW  7
#define HAT_NONE 8

/* ------------------------------------------------------------------ */
/*  Macros                                                              */
/* ------------------------------------------------------------------ */

#define BIT_WRITE(byte, bit, val) \
    ((val) ? ((byte) |=  (1U << (bit))) : ((byte) &= ~(1U << (bit))))

/* ------------------------------------------------------------------ */
/*  Logging tags                                                        */
/* ------------------------------------------------------------------ */

static const char *TAG       = "HID_DEV_DEMO";
static const char *TAGSERIAL = "SerialNumber";

/* ------------------------------------------------------------------ */
/*  Global state                                                        */
/* ------------------------------------------------------------------ */

adc_oneshot_unit_handle_t adc1_handle;

/* Public controller state — write these from your application */
uint8_t  g_axis[4]    = {127, 127, 127, 127}; /* LX LY RX RY */
bool     g_btn[14]    = {false};               /* see button map below */
uint8_t  g_trigger[2] = {0, 0};               /* L2 R2 analog */
int      g_hat        = HAT_NONE;              /* HAT_N..HAT_NW or HAT_NONE */

/*
 * Button index map:
 *   0 = Square    4 = L1      8  = Options
 *   1 = Cross     5 = R1      9  = L3
 *   2 = Circle    6 = L2      10 = R3
 *   3 = Triangle  7 = R2      11 = Share
 *                             12 = PS
 *                             13 = Touchpad
 */

static uint32_t          crc32_table[256];
static volatile bool     g_hid_connected = false;
static char              g_serial_number[13] = "00000001";

typedef struct {
    TaskHandle_t    task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t         protocol_mode;
    uint8_t        *buffer;
} local_param_t;

static local_param_t s_bt_hid_param = {0};

/* ------------------------------------------------------------------ */
/*  HID descriptor                                                      */
/* ------------------------------------------------------------------ */
/*
 * The descriptor MUST open with a standard Generic Desktop / Gamepad
 * Application Collection before any vendor-page (0xFF00) items appear.
 * Bluedroid's internal HID descriptor parser only registers a report in
 * its lookup table when the current application collection usage is a
 * known standard usage.  If the very first usage-page tag is vendor
 * (0xFF00) the parser silently skips the entire report — causing
 * "Can not find report!" whenever the host sends GET_REPORT(0x11).
 *
 * Report byte accounting (payload after report ID):
 *   bytes  1- 2  vendor header (2 bytes)
 *   bytes  3- 6  axes LX/LY/RX/RY (4 bytes)
 *   byte   7     hat(4 bits) + buttons 0-3 (4 bits)
 *   byte   8     buttons 4-11 (8 bits)
 *   byte   9     buttons 12-13 (2 bits) + counter (6 bits)
 *   bytes 10-11  L2/R2 analog (2 bytes)
 *   bytes 12-78  IMU/battery/touch/CRC (67 bytes vendor)
 *   Total payload = 2+4+1+1+1+2+67 = 78 bytes  ✓
 */
static const uint8_t ds4_bt_hid_descriptor[] = {
    /* ---- Application collection: Generic Desktop, Gamepad ---- */
    0x05, 0x01,         /* Usage Page (Generic Desktop)           */
    0x09, 0x05,         /* Usage (Gamepad)                        */
    0xA1, 0x01,         /* Collection (Application)               */

    /* ---- Input Report 0x11 ---- */
    0x85, 0x11,         /* Report ID (0x11)                       */

    /* bytes 1-2: vendor BT header */
    0x06, 0x00, 0xFF,   /* Usage Page (Vendor 0xFF00)             */
    0x09, 0x20,         /* Usage (0x20)                           */
    0x15, 0x00,         /* Logical Minimum (0)                    */
    0x26, 0xFF, 0x00,   /* Logical Maximum (255)                  */
    0x75, 0x08,         /* Report Size (8)                        */
    0x95, 0x02,         /* Report Count (2)                       */
    0x81, 0x02,         /* Input (Data, Var, Abs)                 */

    /* bytes 3-6: LX LY RX RY */
    0x05, 0x01,         /* Usage Page (Generic Desktop)           */
    0x09, 0x30,         /* Usage (X)                              */
    0x09, 0x31,         /* Usage (Y)                              */
    0x09, 0x32,         /* Usage (Z)                              */
    0x09, 0x35,         /* Usage (Rz)                             */
    0x15, 0x00,         /* Logical Minimum (0)                    */
    0x26, 0xFF, 0x00,   /* Logical Maximum (255)                  */
    0x75, 0x08,         /* Report Size (8)                        */
    0x95, 0x04,         /* Report Count (4)                       */
    0x81, 0x02,         /* Input (Data, Var, Abs)                 */

    /* byte 7 bits [3:0]: hat switch */
    0x09, 0x39,         /* Usage (Hat Switch)                     */
    0x15, 0x00,         /* Logical Minimum (0)                    */
    0x25, 0x07,         /* Logical Maximum (7)                    */
    0x35, 0x00,         /* Physical Minimum (0)                   */
    0x46, 0x3B, 0x01,   /* Physical Maximum (315)                 */
    0x65, 0x14,         /* Unit (Degrees)                         */
    0x75, 0x04,         /* Report Size (4)                        */
    0x95, 0x01,         /* Report Count (1)                       */
    0x81, 0x42,         /* Input (Data, Var, Abs, Null)           */
    0x65, 0x00,         /* Unit (None)                            */

    /* byte 7 bits [7:4]: Square/Cross/Circle/Triangle */
    0x05, 0x09,         /* Usage Page (Button)                    */
    0x19, 0x01,         /* Usage Minimum (1)                      */
    0x29, 0x04,         /* Usage Maximum (4)                      */
    0x15, 0x00,         /* Logical Minimum (0)                    */
    0x25, 0x01,         /* Logical Maximum (1)                    */
    0x75, 0x01,         /* Report Size (1)                        */
    0x95, 0x04,         /* Report Count (4)                       */
    0x81, 0x02,         /* Input (Data, Var, Abs)                 */

    /* byte 8: L1/R1/L2/R2/Share/Options/L3/R3 */
    0x19, 0x05,         /* Usage Minimum (5)                      */
    0x29, 0x0C,         /* Usage Maximum (12)                     */
    0x75, 0x01,         /* Report Size (1)                        */
    0x95, 0x08,         /* Report Count (8)                       */
    0x81, 0x02,         /* Input (Data, Var, Abs)                 */

    /* byte 9 bits [1:0]: PS / Touchpad */
    0x19, 0x0D,         /* Usage Minimum (13)                     */
    0x29, 0x0E,         /* Usage Maximum (14)                     */
    0x75, 0x01,         /* Report Size (1)                        */
    0x95, 0x02,         /* Report Count (2)                       */
    0x81, 0x02,         /* Input (Data, Var, Abs)                 */

    /* byte 9 bits [7:2]: 6-bit counter */
    0x06, 0x00, 0xFF,   /* Usage Page (Vendor)                    */
    0x09, 0x21,         /* Usage (0x21)                           */
    0x15, 0x00,         /* Logical Minimum (0)                    */
    0x25, 0x3F,         /* Logical Maximum (63)                   */
    0x75, 0x06,         /* Report Size (6)                        */
    0x95, 0x01,         /* Report Count (1)                       */
    0x81, 0x02,         /* Input (Data, Var, Abs)                 */

    /* bytes 10-11: L2/R2 analog (simulation page) */
    0x05, 0x02,         /* Usage Page (Simulation Controls)       */
    0x09, 0xC4,         /* Usage (Accelerator)                    */
    0x09, 0xC5,         /* Usage (Brake)                          */
    0x15, 0x00,         /* Logical Minimum (0)                    */
    0x26, 0xFF, 0x00,   /* Logical Maximum (255)                  */
    0x75, 0x08,         /* Report Size (8)                        */
    0x95, 0x02,         /* Report Count (2)                       */
    0x81, 0x02,         /* Input (Data, Var, Abs)                 */

    /* bytes 12-78: IMU/battery/touch/CRC (67 bytes vendor) */
    0x06, 0x00, 0xFF,   /* Usage Page (Vendor)                    */
    0x09, 0x22,         /* Usage (0x22)                           */
    0x15, 0x00,         /* Logical Minimum (0)                    */
    0x26, 0xFF, 0x00,   /* Logical Maximum (255)                  */
    0x75, 0x08,         /* Report Size (8)                        */
    0x95, 0x43,         /* Report Count (67)                      */
    0x81, 0x02,         /* Input (Data, Var, Abs)                 */

    /* ---- Output Report 0x11 (77 bytes: rumble + lightbar + misc) ---- */
    0x85, 0x11,         /* Report ID (0x11)                       */
    0x06, 0x00, 0xFF,   /* Usage Page (Vendor)                    */
    0x09, 0x23,         /* Usage (0x23)                           */
    0x15, 0x00,         /* Logical Minimum (0)                    */
    0x26, 0xFF, 0x00,   /* Logical Maximum (255)                  */
    0x75, 0x08,         /* Report Size (8)                        */
    0x95, 0x4D,         /* Report Count (77)                      */
    0x91, 0x02,         /* Output (Data, Var, Abs)                */

    /* ---- Feature Report 0x05 (37 bytes: IMU calibration) ---- */
    0x85, 0x05,         /* Report ID (0x05)                       */
    0x06, 0x00, 0xFF,   /* Usage Page (Vendor)                    */
    0x09, 0x24,         /* Usage (0x24)                           */
    0x15, 0x00,         /* Logical Minimum (0)                    */
    0x26, 0xFF, 0x00,   /* Logical Maximum (255)                  */
    0x75, 0x08,         /* Report Size (8)                        */
    0x95, 0x25,         /* Report Count (37)  ← exact calib size  */
    0xB1, 0x02,         /* Feature (Data, Var, Abs)               */

    /* ---- Feature Report 0x02 (37 bytes: MAC address — required by hid-sony BT init) ---- */
    0x85, 0x02,
    0x06, 0x00, 0xFF,
    0x09, 0x25,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x25,         /* 37 bytes */
    0xB1, 0x02,

    /* ---- Feature Report 0xA3 (49 bytes: firmware info — required by hid-sony BT init) ---- */
    0x85, 0xA3,
    0x06, 0x00, 0xFF,
    0x09, 0x26,
    0x15, 0x00,
    0x26, 0xFF, 0x00,
    0x75, 0x08,
    0x95, 0x31,         /* 49 bytes */
    0xB1, 0x02,

    0xC0                /* End Collection                         */
};

static esp_hid_raw_report_map_t bt_report_maps[] = {
    {
        .data = ds4_bt_hid_descriptor,
        .len  = sizeof(ds4_bt_hid_descriptor)
    },
};

static esp_hid_device_config_t bt_hid_config = {
    .vendor_id         = 0x054C,
    .product_id        = 0x05C4,
    .version           = 0x0100,
    .device_name       = "Wireless Controller",
    .manufacturer_name = "Sony",
    .serial_number     = g_serial_number,
    .report_maps       = bt_report_maps,
    .report_maps_len   = 1
};

/* ------------------------------------------------------------------ */
/*  CRC32 (reflected / LE) — used by DS4 BT input reports             */
/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */
/*  Serial number                                                       */
/* ------------------------------------------------------------------ */

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

/* ------------------------------------------------------------------ */
/*  LE helper writers                                                   */
/* ------------------------------------------------------------------ */

static inline void put_le16(uint8_t *buf, int16_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

static inline void put_le16u(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

/* ------------------------------------------------------------------ */
/*  Hardware init                                                       */
/* ------------------------------------------------------------------ */

void init_hardware_pins(void)
{
    /* ADC — left stick */
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t adc_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &adc_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &adc_cfg));

    /* Button input (active-low with internal pull-up) */
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << GPIO_NUM_32),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void init_ledc(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    /* RGB + rumble channels */
    const ledc_channel_config_t channels[5] = {
        { .channel = LEDC_OUTPUT_R,     .duty = 0, .gpio_num = RED_LED_PIN,   .speed_mode = LEDC_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER },
        { .channel = LEDC_OUTPUT_G,     .duty = 0, .gpio_num = GREEN_LED_PIN, .speed_mode = LEDC_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER },
        { .channel = LEDC_OUTPUT_B,     .duty = 0, .gpio_num = BLUE_LED_PIN,  .speed_mode = LEDC_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER },
        { .channel = LEDC_OUTPUT_RUM_R, .duty = 0, .gpio_num = RUMBLE_R_PIN,  .speed_mode = LEDC_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER },
        { .channel = LEDC_OUTPUT_RUM_L, .duty = 0, .gpio_num = RUMBLE_L_PIN,  .speed_mode = LEDC_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER },
    };
    for (int ch = 0; ch < 5; ch++) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[ch]));
    }
}

/* ------------------------------------------------------------------ */
/*  RGB LED                                                             */
/* ------------------------------------------------------------------ */

void set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    /* Common-anode: 0 = full brightness, 255 = off */
    ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_R, 255 - r);
    ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_R);
    ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_G, 255 - g);
    ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_G);
    ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_B, 255 - b);
    ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_B);
}

/* ------------------------------------------------------------------ */
/*  Rumble motors                                                       */
/* ------------------------------------------------------------------ */

static void set_rumble(uint8_t right_intensity, uint8_t left_intensity)
{
    /* Direct PWM duty = motor intensity (0..255) */
    ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_RUM_R, right_intensity);
    ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_RUM_R);
    ledc_set_duty(LEDC_MODE, LEDC_OUTPUT_RUM_L, left_intensity);
    ledc_update_duty(LEDC_MODE, LEDC_OUTPUT_RUM_L);
}

/* ------------------------------------------------------------------ */
/*  Raw SDP record builder                                              */
/* ------------------------------------------------------------------ */

static uint8_t ds4_sdp_record[1024];
static size_t  ds4_sdp_record_len = 0;

static inline void sdp_put_u8(uint8_t **p, uint8_t v)          { *(*p)++ = v; }
static inline void sdp_put_u16(uint8_t **p, uint16_t v)
{
    *(*p)++ = (uint8_t)((v >> 8) & 0xFF);
    *(*p)++ = (uint8_t)(v & 0xFF);
}
static inline void sdp_put_attr_id(uint8_t **p, uint16_t id)
{
    sdp_put_u8(p, 0x09); sdp_put_u16(p, id);
}
static inline void sdp_put_bool_attr(uint8_t **p, uint16_t id, bool v)
{
    sdp_put_attr_id(p, id); sdp_put_u8(p, 0x28); sdp_put_u8(p, v ? 0x01 : 0x00);
}
static inline void sdp_put_u8_attr(uint8_t **p, uint16_t id, uint8_t v)
{
    sdp_put_attr_id(p, id); sdp_put_u8(p, 0x08); sdp_put_u8(p, v);
}
static inline void sdp_put_u16_attr(uint8_t **p, uint16_t id, uint16_t v)
{
    sdp_put_attr_id(p, id); sdp_put_u8(p, 0x09); sdp_put_u16(p, v);
}
static inline void sdp_put_text_attr(uint8_t **p, uint16_t id, const char *text)
{
    size_t len = strlen(text);
    sdp_put_attr_id(p, id);
    if (len <= 255) { sdp_put_u8(p, 0x25); sdp_put_u8(p, (uint8_t)len); }
    else            { sdp_put_u8(p, 0x26); sdp_put_u16(p, (uint16_t)len); }
    memcpy(*p, text, len); *p += len;
}

static size_t build_ds4_sdp_record(uint8_t *buf, size_t buf_size)
{
    if (buf_size < 512) return 0;
    uint8_t *p = buf;

    sdp_put_u8(&p, 0x36);
    uint8_t *root_len_ptr = p; p += 2;

    /* ServiceClassIDList: HID */
    sdp_put_attr_id(&p, 0x0001);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x03);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x1124);

    /* ProtocolDescriptorList */
    sdp_put_attr_id(&p, 0x0004);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x0D);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x06);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x0100);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0011);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x03);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x0011);

    /* BrowseGroupList */
    sdp_put_attr_id(&p, 0x0005);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x03);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x1002);

    /* LanguageBaseAttributeIDList */
    sdp_put_attr_id(&p, 0x0006);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x09);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x656E);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x006A);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0100);

    /* BluetoothProfileDescriptorList */
    sdp_put_attr_id(&p, 0x0009);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x08);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x06);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x1124);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0100);

    /* AdditionalProtocolDescriptorLists */
    sdp_put_attr_id(&p, 0x000D);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x0F);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x0D);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x06);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x0100);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0013);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x03);
    sdp_put_u8(&p, 0x19); sdp_put_u16(&p, 0x0011);

    /* Human-readable strings */
    sdp_put_text_attr(&p, 0x0100, "Wireless Controller");
    sdp_put_text_attr(&p, 0x0101, "DualShock 4 Compatible");
    sdp_put_text_attr(&p, 0x0102, "Sony");

    /* HID attributes */
    sdp_put_u16_attr(&p, 0x0200, 0x0100);
    sdp_put_u16_attr(&p, 0x0201, 0x0111);
    sdp_put_u8_attr (&p, 0x0202, 0x08);
    sdp_put_u8_attr (&p, 0x0203, 0x00);
    sdp_put_bool_attr(&p, 0x0204, true);
    sdp_put_bool_attr(&p, 0x0205, true);

    /* HIDDescriptorList */
    sdp_put_attr_id(&p, 0x0206);
    sdp_put_u8(&p, 0x36);
    uint8_t *desc_list_len_ptr = p; p += 2;
    sdp_put_u8(&p, 0x36);
    uint8_t *desc_entry_len_ptr = p; p += 2;
    sdp_put_u8(&p, 0x08); sdp_put_u8(&p, 0x22);
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
    desc_entry_len_ptr[1] = (uint8_t)(desc_entry_len       & 0xFF);
    size_t desc_list_len = (size_t)(p - (desc_list_len_ptr + 2));
    desc_list_len_ptr[0] = (uint8_t)((desc_list_len >> 8) & 0xFF);
    desc_list_len_ptr[1] = (uint8_t)(desc_list_len       & 0xFF);

    /* HIDLanguageIDBase */
    sdp_put_attr_id(&p, 0x0207);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x08);
    sdp_put_u8(&p, 0x35); sdp_put_u8(&p, 0x06);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0409);
    sdp_put_u8(&p, 0x09); sdp_put_u16(&p, 0x0100);

    sdp_put_bool_attr(&p, 0x0208, false);
    sdp_put_bool_attr(&p, 0x0209, true);
    sdp_put_bool_attr(&p, 0x020A, true);
    sdp_put_u16_attr(&p, 0x020B, 0x0100);
    sdp_put_u16_attr(&p, 0x020C, 0x0C80);
    sdp_put_bool_attr(&p, 0x020D, true);
    sdp_put_bool_attr(&p, 0x020E, false);
    sdp_put_u16_attr(&p, 0x020F, 0x0640);
    sdp_put_u16_attr(&p, 0x0210, 0x0320);

    size_t root_len = (size_t)(p - (root_len_ptr + 2));
    root_len_ptr[0] = (uint8_t)((root_len >> 8) & 0xFF);
    root_len_ptr[1] = (uint8_t)(root_len       & 0xFF);

    return (size_t)(p - buf);
}

/* ------------------------------------------------------------------ */
/*  Touch sample helper (not touching)                                  */
/* ------------------------------------------------------------------ */

static void ds4_fill_touch_notouch(uint8_t *base, uint8_t ts)
{
    memset(base, 0, 9);
    base[0] = ts;
    base[1] = 0x80;   /* Finger0: not-touching bit set */
    base[5] = 0x80;   /* Finger1: not-touching bit set */
}

/* ------------------------------------------------------------------ */
/*  Fill common DS4 BT report fields (IMU, battery, touch, CRC)        */
/*  Call after bytes [0..11] are already filled.                        */
/* ------------------------------------------------------------------ */

static void ds4_fill_common_fields(uint8_t *report,
                                   uint16_t *timestamp_ptr,
                                   uint8_t  *touch_seq_ptr)
{
    /* Timestamp: ~5.33 µs per unit; increment by 188 per 1 ms */
    *timestamp_ptr += 188;
    put_le16u(&report[12], *timestamp_ptr);

    /* Temperature */
    report[14] = DS4_TEMPERATURE_DEFAULT;

    /* Gyroscope at rest = 0 */
    put_le16(&report[15], 0);
    put_le16(&report[17], 0);
    put_le16(&report[19], 0);

    /* Accelerometer: gravity on Y axis only */
    put_le16(&report[21], 0);
    put_le16(&report[23], (int16_t)DS4_ACCEL_Y_REST);
    put_le16(&report[25], 0);

    /* Battery: BT, full, no cable */
    report[32] = 0x0A;
    report[33] = 0x00;
    report[34] = 0x00;

    /* Touch: 4 samples, all not-touching */
    report[35] = 0x00;
    uint8_t ts = *touch_seq_ptr;
    for (int i = 0; i < 4; i++) {
        ds4_fill_touch_notouch(&report[36 + i * 9], ts++);
    }
    *touch_seq_ptr = ts;

    /* CRC32: seed byte 0xA1, then report[0..74] */
    uint8_t crc_input[76];
    crc_input[0] = 0xA1;
    memcpy(&crc_input[1], report, DS4_BT_CRC_OFFSET);
    uint32_t crc = crc32_le_compute(crc_input, sizeof(crc_input));
    report[75] = (uint8_t)(crc        & 0xFF);
    report[76] = (uint8_t)((crc >> 8) & 0xFF);
    report[77] = (uint8_t)((crc >>16) & 0xFF);
    report[78] = (uint8_t)((crc >>24) & 0xFF);
}

/* ------------------------------------------------------------------ */
/*  Feature Report 0x05 — IMU calibration                              */
/*                                                                      */
/*  Per https://www.psdevwiki.com/ps4/DS4-BT#GET_FEATURE               */
/*  37-byte payload (matches descriptor 0x95,0x25):                    */
/*   [0..5]   Gyro bias X/Y/Z  (LE int16, 0 = no bias)                */
/*   [6..17]  Gyro ±sensitivity X/Y/Z (LE int16)                       */
/*   [18..21] Gyro speed ±range in deg/s (LE int16)                    */
/*   [22..33] Accel ±sensitivity X/Y/Z (LE int16)                      */
/*   [34..36] Reserved (zero)                                           */
/* ------------------------------------------------------------------ */

static void send_ds4_feature_report_05(void)
{
    if (!s_bt_hid_param.hid_dev ||
        !esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        return;
    }

    uint8_t feat[37] = {0};

    /* Gyro bias: zero (no offset) */

    /* Gyro full-scale ±8192 LSB = ±2000 dps */
    put_le16(&feat[6],   8192);
    put_le16(&feat[8],  -8192);
    put_le16(&feat[10],  8192);
    put_le16(&feat[12], -8192);
    put_le16(&feat[14],  8192);
    put_le16(&feat[16], -8192);

    /* Speed range ±2000 deg/s */
    put_le16(&feat[18],  2000);
    put_le16(&feat[20], -2000);

    /* Accel full-scale ±8192 LSB = ±4 g (matches DS4_ACCEL_RES_PER_G) */
    put_le16(&feat[22],  8192);
    put_le16(&feat[24], -8192);
    put_le16(&feat[26],  8192);
    put_le16(&feat[28], -8192);
    put_le16(&feat[30],  8192);
    put_le16(&feat[32], -8192);

    /* feat[34..36] reserved, already 0 */

    esp_err_t err = esp_bt_hid_device_send_report(
        ESP_HIDD_REPORT_TYPE_FEATURE,
        0x05,
        sizeof(feat),
        feat
    );
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "feature 0x05 send failed: %s", esp_err_to_name(err));
    }
}

static void send_ds4_feature_report_02(void)
{
    if (!s_bt_hid_param.hid_dev ||
        !esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        return;
    }

    uint8_t feat[37] = {0};
    feat[0] = 0x02;  /* report ID echo */

    /* Fill MAC from the ESP32's own Bluetooth address */
    const uint8_t *mac = esp_bt_dev_get_address();
    if (mac) {
        /* ESP32 address is little-endian; DS4 reports it big-endian */
        feat[1] = mac[5];
        feat[2] = mac[4];
        feat[3] = mac[3];
        feat[4] = mac[2];
        feat[5] = mac[1];
        feat[6] = mac[0];
    }
    /* [7..36] remain zero */

    esp_err_t err = esp_bt_hid_device_send_report(
        ESP_HIDD_REPORT_TYPE_FEATURE,
        0x02,
        sizeof(feat),
        feat
    );
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "feature 0x02 send failed: %s", esp_err_to_name(err));
    }
}

/*
 * Feature Report 0xA3 — Firmware / hardware version string
 *
 * Linux hid-sony calls GET_FEATURE(0xA3, 49) during ds4_get_calibration_data()
 * to read hardware version. If it gets an error it still continues, but
 * some kernels / SDL versions refuse to enumerate axes without it.
 *
 * Layout (49 bytes):
 *   [0]      0xA3 (report echo)
 *   [1..48]  ASCII hardware/firmware string, zero-padded
 *
 * We send a plausible version string matching a real DS4 CUH-ZCT1.
 */
static void send_ds4_feature_report_a3(void)
{
    if (!s_bt_hid_param.hid_dev ||
        !esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        return;
    }

    uint8_t feat[49] = {0};
    feat[0] = 0xA3;

    /* Firmware string as reported by a real DS4 (CUH-ZCT1E, hw 1.0) */
    const char *fw = "Jun  9 2016";
    const char *hw = "USB 5.0";
    memcpy(&feat[1],  fw, strlen(fw));
    memcpy(&feat[13], hw, strlen(hw));
    feat[24] = 0x00;
    feat[25] = 0x01;  /* hardware revision */
    feat[26] = 0x08;
    feat[27] = 0x00;

    esp_err_t err = esp_bt_hid_device_send_report(
        ESP_HIDD_REPORT_TYPE_FEATURE,
        0xA3,
        sizeof(feat),
        feat
    );
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "feature 0xA3 send failed: %s", esp_err_to_name(err));
    }
}

/* ------------------------------------------------------------------ */
/*  Build and send one DS4 BT input report from g_axis/g_btn/g_trigger */
/* ------------------------------------------------------------------ */

static void send_hid_report_raw(uint8_t *report, size_t len)
{
    if (!s_bt_hid_param.hid_dev ||
        !esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        return;
    }
    if (len != DS4_BT_INPUT_TOTAL_LEN) {
        ESP_LOGW(TAG, "Bad report length %u", (unsigned)len);
        return;
    }
    /*
     * Bypass esp_hidd_dev_input_set() which calls the broken report-map
     * lookup.  Send directly on the interrupt channel instead.
     */
    esp_err_t err = esp_bt_hid_device_send_report(
        ESP_HIDD_REPORT_TYPE_INTRDATA,
        report[0],   /* 0x11 */
        len - 1,
        report + 1
    );
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "send_report failed: %s", esp_err_to_name(err));
    }
}

void send_gamepad_report(void)
{
    if (!s_bt_hid_param.hid_dev ||
        !esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        return;
    }

    static uint8_t  counter   = 0;
    static uint16_t timestamp = 0;
    static uint8_t  touch_seq = 0;

    /* Read ADC → left stick axes */
    int raw_x = 2048, raw_y = 2048;
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw_x);
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &raw_y);
    g_axis[0] = (uint8_t)(raw_x >> 4);
    g_axis[1] = (uint8_t)(raw_y >> 4);

    /* Read button (Cross) from GPIO32, active-low */
    g_btn[1] = (gpio_get_level(GPIO_NUM_32) == 0);

    /* Build 79-byte report */
    uint8_t report[DS4_BT_INPUT_TOTAL_LEN] = {0};
    report[0] = DS4_BT_INPUT_REPORT_ID;
    report[1] = 0xC0;   /* BT header: transaction type + param */
    report[2] = 0x00;

    /* Axes */
    report[3] = g_axis[0];   /* LX */
    report[4] = g_axis[1];   /* LY */
    report[5] = g_axis[2];   /* RX */
    report[6] = g_axis[3];   /* RY */

    /*
     * Byte 7: hat switch (bits 3:0) + Square/Cross/Circle/Triangle (bits 7:4)
     * Hat: 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW, 8=none
     */
    report[7]  = (uint8_t)(g_hat & 0x0F);
    BIT_WRITE(report[7], 4, g_btn[0]);   /* Square    */
    BIT_WRITE(report[7], 5, g_btn[1]);   /* Cross     */
    BIT_WRITE(report[7], 6, g_btn[2]);   /* Circle    */
    BIT_WRITE(report[7], 7, g_btn[3]);   /* Triangle  */

    /* Byte 8: L1/R1/L2/R2/Share/Options/L3/R3 */
    report[8] = 0;
    BIT_WRITE(report[8], 0, g_btn[4]);   /* L1        */
    BIT_WRITE(report[8], 1, g_btn[5]);   /* R1        */
    BIT_WRITE(report[8], 2, g_btn[6]);   /* L2 digital */
    BIT_WRITE(report[8], 3, g_btn[7]);   /* R2 digital */
    BIT_WRITE(report[8], 4, g_btn[11]);  /* Share     */
    BIT_WRITE(report[8], 5, g_btn[8]);   /* Options   */
    BIT_WRITE(report[8], 6, g_btn[9]);   /* L3        */
    BIT_WRITE(report[8], 7, g_btn[10]);  /* R3        */

    /* Byte 9: PS(0) / Touchpad(1) / counter(7:2) */
    report[9] = 0;
    BIT_WRITE(report[9], 0, g_btn[12]);  /* PS button  */
    BIT_WRITE(report[9], 1, g_btn[13]);  /* Touchpad   */
    report[9] |= (uint8_t)((counter & 0x3F) << 2);
    counter++;

    /* Bytes 10-11: L2 / R2 analog */
    report[10] = g_trigger[0];
    report[11] = g_trigger[1];

    ds4_fill_common_fields(report, &timestamp, &touch_seq);
    send_hid_report_raw(report, sizeof(report));
}

/* ------------------------------------------------------------------ */
/*  Test / demo report (cycles axes and cross button automatically)     */
/* ------------------------------------------------------------------ */

void send_gamepad_report_test(void)
{
    if (!s_bt_hid_param.hid_dev ||
        !esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
        return;
    }

    static uint8_t  counter   = 0;
    static uint16_t timestamp = 0;
    static uint8_t  touch_seq = 0;
    static uint32_t step      = 0;

    uint8_t lx = 128, ly = 128, rx = 128, ry = 128;
    bool cross = false;

    step++;
    switch ((step / 100) % 4) {
        case 0: lx = 0;        break;
        case 1: lx = 255;      break;
        case 2: ly = 0;        break;
        case 3: cross = true;  break;
    }

    uint8_t report[DS4_BT_INPUT_TOTAL_LEN] = {0};
    report[0] = DS4_BT_INPUT_REPORT_ID;
    report[1] = 0xC0;
    report[2] = 0x00;
    report[3] = lx;
    report[4] = ly;
    report[5] = rx;
    report[6] = ry;
    report[7] = 0x08;   /* hat = none */
    BIT_WRITE(report[7], 5, cross);
    report[8] = 0x00;
    report[9] = (uint8_t)((counter & 0x3F) << 2);
    counter++;
    report[10] = 0x00;
    report[11] = 0x00;

    ds4_fill_common_fields(report, &timestamp, &touch_seq);
    send_hid_report_raw(report, sizeof(report));
}

/* ------------------------------------------------------------------ */
/*  Main polling task                                                   */
/* ------------------------------------------------------------------ */

static void bt_hid_demo_task(void *pvParameters)
{
    printf(
        "########################################################################\n"
        "BT HID DS4-compatible gamepad running.\n"
        "########################################################################\n\n"
    );

    for (;;) {
        if (!g_hid_connected) break;
        if (esp_hidd_dev_connected(s_bt_hid_param.hid_dev)) {
            send_gamepad_report_test();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_bt_hid_param.task_hdl = NULL;
    vTaskDelete(NULL);
}

static void bt_hid_task_start_up(void)
{
    if (s_bt_hid_param.task_hdl) return;
    g_hid_connected = true;
    vTaskDelay(pdMS_TO_TICKS(50));
    xTaskCreate(bt_hid_demo_task, "bt_hid_task", 4096, NULL, 5,
                &s_bt_hid_param.task_hdl);
}

static void bt_hid_task_shut_down(void)
{
    g_hid_connected = false;
    if (s_bt_hid_param.task_hdl) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ------------------------------------------------------------------ */
/*  HIDD event callback                                                 */
/* ------------------------------------------------------------------ */
/*
 * esp_hidd_event_data_t members (from esp_hidd.h):
 *   .connect     { dev, status }
 *   .disconnect  { dev, reason }
 *   .output      { dev, usage, report_id, length, data, map_index }
 *   .feature     { dev, usage, report_id, length, data, map_index }
 *                NOTE: feature fires for SET_REPORT only in the
 *                esp_hid wrapper; GET_REPORT is handled internally by
 *                bluedroid after our descriptor fix.
 *   .protocol_mode { dev, protocol_mode, map_index }
 *   .control     { dev, control, map_index }
 *
 * There is NO .start, .get_report, .trans_type, or .report_type in this
 * version.  ESP_HIDD_START_EVENT has a null param — guard accordingly.
 *
 * The FEATURE_EVENT is also used by some IDF versions for GET_REPORT
 * responses via trans_type; where that field exists it is accessed as
 * param->feature.trans_type.  Since we cannot guarantee its presence
 * we always call send_ds4_feature_report_05() when report_id == 0x05
 * regardless, which is safe because the PS4 only ever GETs it.
 */

static void bt_hidd_event_callback(void *handler_args,
                                   esp_event_base_t base,
                                   int32_t id,
                                   void *event_data)
{
    esp_hidd_event_t       event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {

    /* ---- Stack started ---- */
    case ESP_HIDD_START_EVENT:
        /*
         * param may be NULL or contain a status field depending on IDF
         * version; use it only if non-NULL and check the connect event
         * for confirmation.
         */
        ESP_LOGI(TAG, "HID stack started");
        set_rgb_color(0, 0, 255);   /* blue = waiting */
        break;

    /* ---- Host connected ---- */
    case ESP_HIDD_CONNECT_EVENT:
        if (param && param->connect.status == ESP_OK) {
            ESP_LOGI(TAG, "Host connected");
            g_hid_connected = true;
            bt_hid_task_start_up();
            set_rgb_color(0, 255, 0);   /* green = connected */
        } else {
            ESP_LOGW(TAG, "Connect failed");
            g_hid_connected = false;
            set_rgb_color(255, 0, 0);
        }
        break;

    /* ---- Output report from host (rumble + lightbar) ---- */
    case ESP_HIDD_OUTPUT_EVENT:
        /*
         * DS4 BT Output Report 0x11 payload layout (0-indexed within data[]):
         *   [0]  0xC0 (transaction header byte 1)
         *   [1]  0x20 (transaction header byte 2)
         *   [2]  enable flags (bit0=rumble, bit1=lightbar, bit2=flash)
         *   [3]  enable flags ext
         *   [4]  padding
         *   [5]  RumbleRight (high-freq, weak motor)
         *   [6]  RumbleLeft  (low-freq,  strong motor)
         *   [7]  LightbarRed
         *   [8]  LightbarGreen
         *   [9]  LightbarBlue
         *  [10]  Lightbar flash on duration  (×10 ms)
         *  [11]  Lightbar flash off duration (×10 ms)
         */
        if (param->output.report_id == 0x11 &&
            param->output.length    >= 10) {

            const uint8_t *out = param->output.data;
            uint8_t rumble_r = out[DS4_OUT_RUMBLE_R];
            uint8_t rumble_l = out[DS4_OUT_RUMBLE_L];
            uint8_t led_r    = out[DS4_OUT_LED_R];
            uint8_t led_g    = out[DS4_OUT_LED_G];
            uint8_t led_b    = out[DS4_OUT_LED_B];

            ESP_LOGI(TAG,
                     "Output → Lightbar R=%u G=%u B=%u  "
                     "Rumble right=%u left=%u",
                     led_r, led_g, led_b, rumble_r, rumble_l);

            set_rgb_color(led_r, led_g, led_b);
            set_rumble(rumble_r, rumble_l);
        } else {
            ESP_LOGW(TAG, "Unhandled OUTPUT id=%u len=%u",
                     (unsigned)param->output.report_id,
                     (unsigned)param->output.length);
        }
        break;

    /* ---- Feature report request (GET/SET from host) ---- */
    case ESP_HIDD_FEATURE_EVENT:
        ESP_LOGI(TAG, "FEATURE id=0x%02x len=%u",
                 (unsigned)param->feature.report_id,
                 (unsigned)param->feature.length);

        switch (param->feature.report_id) {
        case 0x02:
            send_ds4_feature_report_02();   /* MAC address  — hid-sony Linux init */
            break;
        case 0x05:
            send_ds4_feature_report_05();   /* IMU calibration — PS4 + SDL        */
            break;
        case 0xA3:
            send_ds4_feature_report_a3();   /* Firmware version — hid-sony Linux  */
            break;
        default:
            ESP_LOGW(TAG, "Unhandled FEATURE id=0x%02x",
                     (unsigned)param->feature.report_id);
            break;
        }
        break;

    /* ---- Protocol mode change (Report ↔ Boot) ---- */
    case ESP_HIDD_PROTOCOL_MODE_EVENT:
        s_bt_hid_param.protocol_mode = param->protocol_mode.protocol_mode;
        ESP_LOGI(TAG, "Protocol mode → %s",
                 s_bt_hid_param.protocol_mode ? "REPORT" : "BOOT");
        break;

    /* ---- Host disconnected ---- */
    case ESP_HIDD_DISCONNECT_EVENT:
        ESP_LOGI(TAG, "Host disconnected (reason=%d)",
                 param ? param->disconnect.reason : -1);
        g_hid_connected = false;
        set_rumble(0, 0);
        bt_hid_task_shut_down();
        set_rgb_color(255, 255, 255);   /* white = idle */
        break;

    /* ---- Stack stopped ---- */
    case ESP_HIDD_STOP_EVENT:
        ESP_LOGI(TAG, "HID stack stopped");
        g_hid_connected = false;
        set_rumble(0, 0);
        set_rgb_color(0, 255, 255);
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/*  SDP callback                                                        */
/* ------------------------------------------------------------------ */

static void esp_sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    ESP_LOGI(TAG, "SDP event %d", event);

    switch (event) {
    case ESP_SDP_INIT_EVT:
        ESP_LOGI(TAG, "SDP INIT status=%d", param->init.status);
        if (param->init.status != ESP_SDP_SUCCESS) return;

        /* DIP (Device ID Profile) record */
        {
            esp_bluetooth_sdp_dip_record_t dip = {
                .hdr            = { .type = ESP_SDP_TYPE_DIP_SERVER },
                .vendor         = 0x054C,
                .vendor_id_source = ESP_SDP_VENDOR_ID_SRC_BT,
                .product        = 0x05C4,
                .version        = 0x0100,
                .primary_record = true,
            };
            esp_err_t err = esp_sdp_create_record(
                (esp_bluetooth_sdp_record_t *)&dip);
            ESP_LOGI(TAG, "DIP record: %s", esp_err_to_name(err));
        }

        /* HID service record */
        {
            ds4_sdp_record_len = build_ds4_sdp_record(
                ds4_sdp_record, sizeof(ds4_sdp_record));
            if (!ds4_sdp_record_len) {
                ESP_LOGE(TAG, "Failed to build DS4 SDP record");
                return;
            }
            static char svc_name[] = "Wireless Controller";
            esp_bluetooth_sdp_raw_record_t raw = {
                .hdr = {
                    .type                = ESP_SDP_TYPE_RAW,
                    .service_name        = svc_name,
                    .service_name_length = sizeof(svc_name),
                    .user1_ptr           = ds4_sdp_record,
                    .user1_ptr_len       = ds4_sdp_record_len,
                    .rfcomm_channel_number = -1,
                    .l2cap_psm           = 0x11
                }
            };
            esp_err_t err = esp_sdp_create_record(
                (esp_bluetooth_sdp_record_t *)&raw);
            ESP_LOGI(TAG, "HID record: %s", esp_err_to_name(err));
        }
        break;

    case ESP_SDP_CREATE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "SDP record created: status=%d handle=0x%x",
                 param->create_record.status,
                 param->create_record.record_handle);
        break;

    default:
        ESP_LOGW(TAG, "Unhandled SDP event %d", event);
        break;
    }
}

/* ------------------------------------------------------------------ */
/*  Entry point                                                         */
/* ------------------------------------------------------------------ */

void start_ps4_hid(void)
{
    init_ledc();
    init_hardware_pins();
    set_rgb_color(255, 255, 255);   /* white = booting */

    generate_serial_number(g_serial_number, sizeof(g_serial_number));
    bt_hid_config.serial_number = g_serial_number;
    ESP_LOGI(TAGSERIAL, "Serial: %s", g_serial_number);

    crc32_init_table();

    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* BT GAP */
    ESP_LOGI(TAG, "Initializing HID GAP");
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_DEV_MODE));
    ESP_ERROR_CHECK(esp_bt_gap_set_device_name(bt_hid_config.device_name));

    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    cod.minor = ESP_BT_COD_MINOR_PERIPHERAL_JOYSTICK;
    ESP_ERROR_CHECK(esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR));
    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                             ESP_BT_GENERAL_DISCOVERABLE));

    vTaskDelay(pdMS_TO_TICKS(500));

    /* HID device */
    ESP_LOGI(TAG, "Initializing HID device");
    ESP_ERROR_CHECK(esp_hidd_dev_init(&bt_hid_config,
                                      ESP_HID_TRANSPORT_BT,
                                      bt_hidd_event_callback,
                                      &s_bt_hid_param.hid_dev));

    /* SDP */
    ESP_LOGI(TAG, "Initializing SDP");
    ESP_ERROR_CHECK(esp_sdp_register_callback(esp_sdp_cb));
    ESP_ERROR_CHECK(esp_sdp_init());
    vTaskDelay(pdMS_TO_TICKS(200));
}