 
/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "esp_log.h"
#include "esp_hidd_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_err.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_gap_bt_api.h"
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define PS4_REPORT_BUFFER_SIZE                  64

static const char local_device_name[] = "Wireless Controller";

typedef struct {
    esp_hidd_app_param_t app_param;
    esp_hidd_qos_param_t both_qos;
    uint8_t protocol_mode;
    SemaphoreHandle_t controller_mutex;
    TaskHandle_t controller_task_hdl;
    uint8_t buffer[PS4_REPORT_BUFFER_SIZE];
    int8_t x_dir;
} local_param_t;

static local_param_t s_local_param = {0};

// HID report descriptor for a generic mouse. The contents of the report are:
// 3 buttons, moving information for X and Y cursors, information for a wheel.
uint8_t hid_ps4_descriptor[] = {
    0x05, 0x01,                   // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                   // USAGE (Gamepad)
    0xA1, 0x01,                   // COLLECTION (Application)
    0x85, 0x01,                   // REPORT_ID (1)

    // Buttons (Cross, Circle, Square, Triangle, L1, R1, etc.)
    0x05, 0x09,                   // USAGE_PAGE (Button)
    0x19, 0x01,                   // USAGE_MINIMUM (Button 1)
    0x29, 0x10,                   // USAGE_MAXIMUM (Button 16)
    0x15, 0x00,                   // LOGICAL_MINIMUM (0)
    0x25, 0x01,                   // LOGICAL_MAXIMUM (1)
    0x75, 0x01,                   // REPORT_SIZE (1)
    0x95, 0x10,                   // REPORT_COUNT (16)
    0x81, 0x02,                   // INPUT (Data,Var,Abs)

    // Left stick X and Y
    0x05, 0x01,                   // USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                   // USAGE (X)
    0x09, 0x31,                   // USAGE (Y)
    0x15, 0x81,                   // LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                   // LOGICAL_MAXIMUM (127)
    0x75, 0x08,                   // REPORT_SIZE (8)
    0x95, 0x02,                   // REPORT_COUNT (2)
    0x81, 0x02,                   // INPUT (Data,Var,Abs)

    // Right stick X and Y
    0x09, 0x32,                   // USAGE (Z)
    0x09, 0x35,                   // USAGE (Rz)
    0x15, 0x81,                   // LOGICAL_MINIMUM (-127)
    0x25, 0x7F,                   // LOGICAL_MAXIMUM (127)
    0x75, 0x08,                   // REPORT_SIZE (8)
    0x95, 0x02,                   // REPORT_COUNT (2)
    0x81, 0x02,                   // INPUT (Data,Var,Abs)

    // Pressure-sensitive L2 and R2 triggers
    0x05, 0x01,                   // USAGE_PAGE (Generic Desktop)
    0x09, 0x33,                   // USAGE (Rx)
    0x09, 0x34,                   // USAGE (Ry)
    0x15, 0x00,                   // LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x00,             // LOGICAL_MAXIMUM (255)
    0x75, 0x08,                   // REPORT_SIZE (8)
    0x95, 0x02,                   // REPORT_COUNT (2)
    0x81, 0x02,                   // INPUT (Data,Var,Abs)

    0xC0                          // END_COLLECTION
};

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }
    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

const int hid_ps4_descriptor_len = sizeof(hid_ps4_descriptor);

/**
 * @brief Integrity check of the report ID and report type for GET_REPORT request from HID host.
 *        Boot Protocol Mode requires report ID. For Report Protocol Mode, when the report descriptor
 *        does not declare report ID Global ITEMS, the report ID does not exist in the GET_REPORT request,
 *        and a value of 0 for report_id will occur in ESP_HIDD_GET_REPORT_EVT callback parameter.
 */
bool check_report_id_type(uint8_t report_id, uint8_t report_type)
{
    bool ret = false;
    xSemaphoreTake(s_local_param.controller_mutex, portMAX_DELAY);
    do {
        if (report_type != ESP_HIDD_REPORT_TYPE_INPUT) {
            break;
        }
        if (s_local_param.protocol_mode == ESP_HIDD_BOOT_MODE) {
            if (report_id == ESP_HIDD_BOOT_REPORT_ID_MOUSE) {
                ret = true;
                break;
            }
        } else {
            if (report_id == 0) {
                ret = true;
                break;
            }
        }
    } while (0);

    if (!ret) {
        if (s_local_param.protocol_mode == ESP_HIDD_BOOT_MODE) {
            esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID);
        } else {
            esp_bt_hid_device_report_error(ESP_HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID);
        }
    }
    xSemaphoreGive(s_local_param.controller_mutex);
    return ret;
}

// send the buttons, change in x, and change in y

void send_ps4_report(uint16_t buttons, int8_t left_x, int8_t left_y, int8_t right_x, int8_t right_y, uint8_t l2, uint8_t r2)
{
    xSemaphoreTake(s_local_param.controller_mutex, portMAX_DELAY);

    s_local_param.buffer[0] = buttons & 0xFF;       // Lower 8 bits of buttons
    s_local_param.buffer[1] = (buttons >> 8) & 0xFF; // Upper 8 bits of buttons
    s_local_param.buffer[2] = left_x;               // Left stick X
    s_local_param.buffer[3] = left_y;               // Left stick Y
    s_local_param.buffer[4] = right_x;              // Right stick X
    s_local_param.buffer[5] = right_y;              // Right stick Y
    s_local_param.buffer[6] = l2;                   // L2 trigger
    s_local_param.buffer[7] = r2;                   // R2 trigger

    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x01, 8, s_local_param.buffer);
    xSemaphoreGive(s_local_param.controller_mutex);
}

// move the mouse left and right
void controller_task(void *pvParameters)
{
    const char *TAG = "controller_task";
    ESP_LOGI(TAG, "Starting PS4 controller emulation");

    uint16_t buttons = 0;
    bool toggle_button = false;
    uint8_t l2 = 0, r2 = 0;
    bool increasing_l2 = true, increasing_r2 = true;

    for (;;) {
        buttons = toggle_button ? 0x0001 : 0x0002; // Toggle buttons
        toggle_button = !toggle_button;

        if (increasing_l2) {
            l2 += 5;
            if (l2 >= 255) increasing_l2 = false;
        } else {
            l2 -= 5;
            if (l2 <= 0) increasing_l2 = true;
        }

        if (increasing_r2) {
            r2 += 5;
            if (r2 >= 255) increasing_r2 = false;
        } else {
            r2 -= 5;
            if (r2 <= 0) increasing_r2 = true;
        }

        ESP_LOGI(TAG, "Buttons: 0x%04X, L2: %d, R2: %d", buttons, l2, r2);
        send_ps4_report(buttons, 0, 0, 0, 0, l2, r2);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


 


void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "esp_bt_gap_cb";
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            ESP_LOG_BUFFER_HEX(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;
    default:
        ESP_LOGI(TAG, "event: %d", event);
        break;
    }
    return;
}

void bt_app_task_start_up(void)
{
    s_local_param.controller_mutex = xSemaphoreCreateMutex();
    memset(s_local_param.buffer, 0, PS4_REPORT_BUFFER_SIZE);
    xTaskCreate(controller_task, "controller_task", 2 * 1024, NULL, configMAX_PRIORITIES - 3, &s_local_param.controller_task_hdl);
    return;
}

void bt_app_task_shut_down(void)
{
    if (s_local_param.controller_task_hdl) {
        vTaskDelete(s_local_param.controller_task_hdl);
        s_local_param.controller_task_hdl = NULL;
    }

    if (s_local_param.controller_mutex) {
        vSemaphoreDelete(s_local_param.controller_mutex);
        s_local_param.controller_mutex = NULL;
    }
    return;
}

void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    static const char *TAG = "esp_bt_hidd_cb";
    switch (event) {
    case ESP_HIDD_INIT_EVT:
        if (param->init.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters");
            esp_bt_hid_device_register_app(&s_local_param.app_param, &s_local_param.both_qos, &s_local_param.both_qos);
        } else {
            ESP_LOGE(TAG, "init hidd failed!");
        }
        break;
    case ESP_HIDD_DEINIT_EVT:
        break;
    case ESP_HIDD_REGISTER_APP_EVT:
        if (param->register_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters success!");
            ESP_LOGI(TAG, "setting to connectable, discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            if (param->register_app.in_use) {
                ESP_LOGI(TAG, "start virtual cable plug!");
                esp_bt_hid_device_connect(param->register_app.bd_addr);
            }
        } else {
            ESP_LOGE(TAG, "setting hid parameters failed!");
        }
        break;
    case ESP_HIDD_UNREGISTER_APP_EVT:
        if (param->unregister_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "unregister app success!");
        } else {
            ESP_LOGE(TAG, "unregister app failed!");
        }
        break;
    case ESP_HIDD_OPEN_EVT:
        if (param->open.status == ESP_HIDD_SUCCESS) {
            if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
                ESP_LOGI(TAG, "connecting...");
            } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
                ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                         param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                         param->open.bd_addr[5]);
                bt_app_task_start_up();
                ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "open failed!");
        }
        break;
    case ESP_HIDD_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_CLOSE_EVT");
        if (param->close.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                ESP_LOGI(TAG, "disconnecting...");
            } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    case ESP_HIDD_SEND_REPORT_EVT:
        if (param->send_report.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d", param->send_report.report_id,
                     param->send_report.report_type);
        } else {
            ESP_LOGE(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d, status:%d, reason:%d",
                     param->send_report.report_id, param->send_report.report_type, param->send_report.status,
                     param->send_report.reason);
        }
        break;
    case ESP_HIDD_REPORT_ERR_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
        break;
    case ESP_HIDD_GET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d", param->get_report.report_id,
                 param->get_report.report_type, param->get_report.buffer_size);
        if (check_report_id_type(param->get_report.report_id, param->get_report.report_type)) {
            uint8_t report_id;
            uint16_t report_len;
            if (s_local_param.protocol_mode == ESP_HIDD_REPORT_MODE) {
                report_id = 0;
                report_len = PS4_REPORT_BUFFER_SIZE;
            } else {
                // Boot Mode
                report_id = ESP_HIDD_BOOT_REPORT_ID_MOUSE;
                report_len = ESP_HIDD_BOOT_REPORT_SIZE_MOUSE - 1;
            }
            xSemaphoreTake(s_local_param.controller_mutex, portMAX_DELAY);
            esp_bt_hid_device_send_report(param->get_report.report_type, report_id, report_len, s_local_param.buffer);
            xSemaphoreGive(s_local_param.controller_mutex);
        } else {
            ESP_LOGE(TAG, "check_report_id failed!");
        }
        break;
    case ESP_HIDD_SET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT");
        break;
    case ESP_HIDD_SET_PROTOCOL_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT");
        if (param->set_protocol.protocol_mode == ESP_HIDD_BOOT_MODE) {
            ESP_LOGI(TAG, "  - boot protocol");
            xSemaphoreTake(s_local_param.controller_mutex, portMAX_DELAY);
            s_local_param.x_dir = -1;
            xSemaphoreGive(s_local_param.controller_mutex);
        } else if (param->set_protocol.protocol_mode == ESP_HIDD_REPORT_MODE) {
            ESP_LOGI(TAG, "  - report protocol");
        }
        xSemaphoreTake(s_local_param.controller_mutex, portMAX_DELAY);
        s_local_param.protocol_mode = param->set_protocol.protocol_mode;
        xSemaphoreGive(s_local_param.controller_mutex);
        break;
    case ESP_HIDD_INTR_DATA_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
        break;
    case ESP_HIDD_VC_UNPLUG_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
        if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    default:
        break;
    }
}

void app_main(void)
{
    const char *TAG = "app_main";
    esp_err_t ret;
    char bda_str[18] = {0};

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "initialize controller failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "enable controller failed: %s", esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if (CONFIG_EXAMPLE_SSP_ENABLED == false)
    bluedroid_cfg.ssp_en = false;
#endif
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "enable bluedroid failed: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "setting device name");
    esp_bt_gap_set_device_name(local_device_name);

    ESP_LOGI(TAG, "setting cod major, peripheral");
    esp_bt_cod_t cod = {0};
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Initialize HID SDP information and L2CAP parameters.
    // to be used in the call of `esp_bt_hid_device_register_app` after profile initialization finishes
    do {
        s_local_param.app_param.name = "Wireless Controller";
        s_local_param.app_param.description = "Wireless Controller";
        s_local_param.app_param.provider = "Sony";
        s_local_param.app_param.subclass = ESP_HID_CLASS_GPD; // keep same with minor class of COD
        s_local_param.app_param.desc_list = hid_ps4_descriptor;
        s_local_param.app_param.desc_list_len = hid_ps4_descriptor_len;

        memset(&s_local_param.both_qos, 0, sizeof(esp_hidd_qos_param_t)); // don't set the qos parameters
    } while (0);

    // Report Protocol Mode is the default mode, according to Bluetooth HID specification
    s_local_param.protocol_mode = ESP_HIDD_REPORT_MODE;

    ESP_LOGI(TAG, "register hid device callback");
    esp_bt_hid_device_register_callback(esp_bt_hidd_cb);

    ESP_LOGI(TAG, "starting hid device");
    esp_bt_hid_device_init();

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
    ESP_LOGI(TAG, "exiting");
}
