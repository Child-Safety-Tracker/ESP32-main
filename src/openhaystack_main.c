#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "nvs_flash.h"
#include "esp_partition.h"

#include "esp_mac.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_random.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"

// #define GATTS_TAG "GATTS_DEMO"

// /// Declare the static function
// static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
// static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// #define GATTS_SERVICE_UUID_TEST_A 0x00FF
// #define GATTS_CHAR_UUID_TEST_A 0xFF01
// #define GATTS_DESCR_UUID_TEST_A 0x3333
// #define GATTS_NUM_HANDLE_TEST_A 4

// #define GATTS_SERVICE_UUID_TEST_B 0x00EE
// #define GATTS_CHAR_UUID_TEST_B 0xEE01
// #define GATTS_DESCR_UUID_TEST_B 0x2222
// #define GATTS_NUM_HANDLE_TEST_B 4

// static char test_device_name[ESP_BLE_ADV_DATA_LEN_MAX] = "ESP_GATTS_DEMO";

// #define TEST_MANUFACTURER_DATA_LEN 17

// #define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

// #define PREPARE_BUF_MAX_SIZE 1024

// static uint8_t char1_str[] = {0x11, 0x22, 0x33};
// static esp_gatt_char_prop_t a_property = 0;
// static esp_gatt_char_prop_t b_property = 0;

// static esp_attr_value_t gatts_demo_char1_val =
//     {
//         .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
//         .attr_len = sizeof(char1_str),
//         .attr_value = char1_str,
// };

// static uint8_t adv_config_done = 0;
// #define adv_config_flag (1 << 0)
// #define scan_rsp_config_flag (1 << 1)

// #ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
// static uint8_t raw_adv_data[] = {
//     /* Flags */
//     0x02, ESP_BLE_AD_TYPE_FLAG, 0x06, // Length 2, Data Type ESP_BLE_AD_TYPE_FLAG, Data 1 (LE General Discoverable Mode, BR/EDR Not Supported)
//     /* TX Power Level */
//     0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB, // Length 2, Data Type ESP_BLE_AD_TYPE_TX_PWR, Data 2 (-21)
//     /* Complete 16-bit Service UUIDs */
//     0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xAB, 0xCD // Length 3, Data Type ESP_BLE_AD_TYPE_16SRV_CMPL, Data 3 (UUID)
// };

// static uint8_t raw_scan_rsp_data[] = {
//     /* Complete Local Name */
//     0x0F, ESP_BLE_AD_TYPE_NAME_CMPL, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D', 'E', 'M', 'O' // Length 15, Data Type ESP_BLE_AD_TYPE_NAME_CMPL, Data (ESP_GATTS_DEMO)
// };
// #else

// static uint8_t adv_service_uuid128[32] = {
//     /* LSB <--------------------------------------------------------------------------------> MSB */
//     // first uuid, 16bit, [12],[13] is the value
//     0xfb,
//     0x34,
//     0x9b,
//     0x5f,
//     0x80,
//     0x00,
//     0x00,
//     0x80,
//     0x00,
//     0x10,
//     0x00,
//     0x00,
//     0xEE,
//     0x00,
//     0x00,
//     0x00,
//     // second uuid, 32bit, [12], [13], [14], [15] is the value
//     0xfb,
//     0x34,
//     0x9b,
//     0x5f,
//     0x80,
//     0x00,
//     0x00,
//     0x80,
//     0x00,
//     0x10,
//     0x00,
//     0x00,
//     0xFF,
//     0x00,
//     0x00,
//     0x00,
// };

// // The length of adv data must be less than 31 bytes
// // static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
// // adv data
// static esp_ble_adv_data_t adv_data = {
//     .set_scan_rsp = false,
//     .include_name = true,
//     .include_txpower = false,
//     .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
//     .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
//     .appearance = 0x00,
//     .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
//     .p_manufacturer_data = NULL, //&test_manufacturer[0],
//     .service_data_len = 0,
//     .p_service_data = NULL,
//     .service_uuid_len = sizeof(adv_service_uuid128),
//     .p_service_uuid = adv_service_uuid128,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
// };
// // scan response data
// static esp_ble_adv_data_t scan_rsp_data = {
//     .set_scan_rsp = true,
//     .include_name = true,
//     .include_txpower = true,
//     //.min_interval = 0x0006,
//     //.max_interval = 0x0010,
//     .appearance = 0x00,
//     .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
//     .p_manufacturer_data = NULL, //&test_manufacturer[0],
//     .service_data_len = 0,
//     .p_service_data = NULL,
//     .service_uuid_len = sizeof(adv_service_uuid128),
//     .p_service_uuid = adv_service_uuid128,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
// };

// #endif /* CONFIG_SET_RAW_ADV_DATA */

// static esp_ble_adv_params_t adv_params = {
//     .adv_int_min = 0x20,
//     .adv_int_max = 0x40,
//     .adv_type = ADV_TYPE_IND,
//     .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
//     //.peer_addr            =
//     //.peer_addr_type       =
//     .channel_map = ADV_CHNL_ALL,
//     .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
// };

// #define PROFILE_NUM 2
// #define PROFILE_A_APP_ID 0
// #define PROFILE_B_APP_ID 1

// struct gatts_profile_inst
// {
//     esp_gatts_cb_t gatts_cb;
//     uint16_t gatts_if;
//     uint16_t app_id;
//     uint16_t conn_id;
//     uint16_t service_handle;
//     esp_gatt_srvc_id_t service_id;
//     uint16_t char_handle;
//     esp_bt_uuid_t char_uuid;
//     esp_gatt_perm_t perm;
//     esp_gatt_char_prop_t property;
//     uint16_t descr_handle;
//     esp_bt_uuid_t descr_uuid;
// };

// /* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
// static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
//     [PROFILE_A_APP_ID] = {
//         .gatts_cb = gatts_profile_a_event_handler,
//         .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//     },
//     [PROFILE_B_APP_ID] = {
//         .gatts_cb = gatts_profile_b_event_handler, /* This demo does not implement, similar as profile A */
//         .gatts_if = ESP_GATT_IF_NONE,              /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//     },
// };

// typedef struct
// {
//     uint8_t *prepare_buf;
//     int prepare_len;
// } prepare_type_env_t;

// static prepare_type_env_t a_prepare_write_env;
// static prepare_type_env_t b_prepare_write_env;

// void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
// void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

// static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
// {
//     switch (event)
//     {
// #ifdef CONFIG_SET_RAW_ADV_DATA
//     case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
//         adv_config_done &= (~adv_config_flag);
//         if (adv_config_done == 0)
//         {
//             esp_ble_gap_start_advertising(&adv_params);
//         }
//         break;
//     case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
//         adv_config_done &= (~scan_rsp_config_flag);
//         if (adv_config_done == 0)
//         {
//             esp_ble_gap_start_advertising(&adv_params);
//         }
//         break;
// #else
//     case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
//         adv_config_done &= (~adv_config_flag);
//         if (adv_config_done == 0)
//         {
//             esp_ble_gap_start_advertising(&adv_params);
//         }
//         break;
//     case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
//         adv_config_done &= (~scan_rsp_config_flag);
//         if (adv_config_done == 0)
//         {
//             esp_ble_gap_start_advertising(&adv_params);
//         }
//         break;
// #endif
//     case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
//         // advertising start complete event to indicate advertising start successfully or failed
//         if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
//         {
//             ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
//             break;
//         }
//         ESP_LOGI(GATTS_TAG, "Advertising start successfully");
//         break;
//     case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
//         if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
//         {
//             ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
//             break;
//         }
//         ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
//         break;
//     case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
//         ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
//                  param->update_conn_params.status,
//                  param->update_conn_params.conn_int,
//                  param->update_conn_params.latency,
//                  param->update_conn_params.timeout);
//         break;
//     case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
//         ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
//                  param->pkt_data_length_cmpl.status,
//                  param->pkt_data_length_cmpl.params.rx_len,
//                  param->pkt_data_length_cmpl.params.tx_len);
//         break;
//     default:
//         break;
//     }
// }

// void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
// {
//     esp_gatt_status_t status = ESP_GATT_OK;
//     if (param->write.need_rsp)
//     {
//         if (param->write.is_prep)
//         {
//             if (param->write.offset > PREPARE_BUF_MAX_SIZE)
//             {
//                 status = ESP_GATT_INVALID_OFFSET;
//             }
//             else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
//             {
//                 status = ESP_GATT_INVALID_ATTR_LEN;
//             }
//             if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
//             {
//                 prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
//                 prepare_write_env->prepare_len = 0;
//                 if (prepare_write_env->prepare_buf == NULL)
//                 {
//                     ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
//                     status = ESP_GATT_NO_RESOURCES;
//                 }
//             }

//             esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
//             if (gatt_rsp)
//             {
//                 gatt_rsp->attr_value.len = param->write.len;
//                 gatt_rsp->attr_value.handle = param->write.handle;
//                 gatt_rsp->attr_value.offset = param->write.offset;
//                 gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
//                 memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
//                 esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
//                 if (response_err != ESP_OK)
//                 {
//                     ESP_LOGE(GATTS_TAG, "Send response error\n");
//                 }
//                 free(gatt_rsp);
//             }
//             else
//             {
//                 ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
//                 status = ESP_GATT_NO_RESOURCES;
//             }
//             if (status != ESP_GATT_OK)
//             {
//                 return;
//             }
//             memcpy(prepare_write_env->prepare_buf + param->write.offset,
//                    param->write.value,
//                    param->write.len);
//             prepare_write_env->prepare_len += param->write.len;
//         }
//         else
//         {
//             esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
//         }
//     }
// }

// void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
// {
//     if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
//     {
//         ESP_LOG_BUFFER_HEX(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
//     }
//     else
//     {
//         ESP_LOGI(GATTS_TAG, "Prepare write cancel");
//     }
//     if (prepare_write_env->prepare_buf)
//     {
//         free(prepare_write_env->prepare_buf);
//         prepare_write_env->prepare_buf = NULL;
//     }
//     prepare_write_env->prepare_len = 0;
// }

// static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
// {
//     switch (event)
//     {
//     case ESP_GATTS_REG_EVT:
//         ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
//         gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
//         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
//         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
//         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

//         esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(test_device_name);
//         if (set_dev_name_ret)
//         {
//             ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
//         }
// #ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
//         esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
//         if (raw_adv_ret)
//         {
//             ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
//         }
//         adv_config_done |= adv_config_flag;
//         esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
//         if (raw_scan_ret)
//         {
//             ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
//         }
//         adv_config_done |= scan_rsp_config_flag;
// #else
//         // config adv data
//         esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
//         if (ret)
//         {
//             ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
//         }
//         adv_config_done |= adv_config_flag;
//         // config scan response data
//         ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
//         if (ret)
//         {
//             ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
//         }
//         adv_config_done |= scan_rsp_config_flag;

// #endif
//         esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
//         break;
//     case ESP_GATTS_READ_EVT:
//     {
//         ESP_LOGI(GATTS_TAG, "Characteristic read, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
//         esp_gatt_rsp_t rsp;
//         memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
//         rsp.attr_value.handle = param->read.handle;
//         rsp.attr_value.len = 4;
//         rsp.attr_value.value[0] = 0xde;
//         rsp.attr_value.value[1] = 0xed;
//         rsp.attr_value.value[2] = 0xbe;
//         rsp.attr_value.value[3] = 0xef;
//         esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
//                                     ESP_GATT_OK, &rsp);
//         break;
//     }
//     case ESP_GATTS_WRITE_EVT:
//     {
//         ESP_LOGI(GATTS_TAG, "Characteristic write, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
//         if (!param->write.is_prep)
//         {
//             ESP_LOGI(GATTS_TAG, "value len %d, value ", param->write.len);
//             ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
//             if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
//             {
//                 uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
//                 if (descr_value == 0x0001)
//                 {
//                     if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
//                     {
//                         ESP_LOGI(GATTS_TAG, "Notification enable");
//                         uint8_t notify_data[15];
//                         for (int i = 0; i < sizeof(notify_data); ++i)
//                         {
//                             notify_data[i] = i % 0xff;
//                         }
//                         // the size of notify_data[] need less than MTU size
//                         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
//                                                     sizeof(notify_data), notify_data, false);
//                     }
//                 }
//                 else if (descr_value == 0x0002)
//                 {
//                     if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
//                     {
//                         ESP_LOGI(GATTS_TAG, "Indication enable");
//                         uint8_t indicate_data[15];
//                         for (int i = 0; i < sizeof(indicate_data); ++i)
//                         {
//                             indicate_data[i] = i % 0xff;
//                         }
//                         // the size of indicate_data[] need less than MTU size
//                         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
//                                                     sizeof(indicate_data), indicate_data, true);
//                     }
//                 }
//                 else if (descr_value == 0x0000)
//                 {
//                     ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
//                 }
//                 else
//                 {
//                     ESP_LOGE(GATTS_TAG, "Unknown descriptor value");
//                     ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
//                 }
//             }
//         }
//         example_write_event_env(gatts_if, &a_prepare_write_env, param);
//         break;
//     }
//     case ESP_GATTS_EXEC_WRITE_EVT:
//         ESP_LOGI(GATTS_TAG, "Execute write");
//         esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
//         example_exec_write_event_env(&a_prepare_write_env, param);
//         break;
//     case ESP_GATTS_MTU_EVT:
//         ESP_LOGI(GATTS_TAG, "MTU exchange, MTU %d", param->mtu.mtu);
//         break;
//     case ESP_GATTS_UNREG_EVT:
//         break;
//     case ESP_GATTS_CREATE_EVT:
//         ESP_LOGI(GATTS_TAG, "Service create, status %d, service_handle %d", param->create.status, param->create.service_handle);
//         gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
//         gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
//         gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

//         esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
//         a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//         esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
//                                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
//                                                         a_property,
//                                                         &gatts_demo_char1_val, NULL);
//         if (add_char_ret)
//         {
//             ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
//         }
//         break;
//     case ESP_GATTS_ADD_INCL_SRVC_EVT:
//         break;
//     case ESP_GATTS_ADD_CHAR_EVT:
//     {
//         uint16_t length = 0;
//         const uint8_t *prf_char;

//         ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, service_handle %d",
//                  param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
//         gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
//         gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
//         gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
//         esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
//         if (get_attr_ret == ESP_FAIL)
//         {
//             ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
//         }

//         ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
//         for (int i = 0; i < length; i++)
//         {
//             ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x", i, prf_char[i]);
//         }
//         esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
//                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
//         if (add_descr_ret)
//         {
//             ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
//         }
//         break;
//     }
//     case ESP_GATTS_ADD_CHAR_DESCR_EVT:
//         gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
//         ESP_LOGI(GATTS_TAG, "Descriptor add, status %d, attr_handle %d, service_handle %d",
//                  param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
//         break;
//     case ESP_GATTS_DELETE_EVT:
//         break;
//     case ESP_GATTS_START_EVT:
//         ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d",
//                  param->start.status, param->start.service_handle);
//         break;
//     case ESP_GATTS_STOP_EVT:
//         break;
//     case ESP_GATTS_CONNECT_EVT:
//     {
//         esp_ble_conn_update_params_t conn_params = {0};
//         memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
//         /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
//         conn_params.latency = 0;
//         conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
//         conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
//         conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
//         ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote " ESP_BD_ADDR_STR "",
//                  param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
//         gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
//         // start sent the update connection parameters to the peer device.
//         esp_ble_gap_update_conn_params(&conn_params);
//         break;
//     }
//     case ESP_GATTS_DISCONNECT_EVT:
//         ESP_LOGI(GATTS_TAG, "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%02x",
//                  ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
//         esp_ble_gap_start_advertising(&adv_params);
//         break;
//     case ESP_GATTS_CONF_EVT:
//         ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
//         if (param->conf.status != ESP_GATT_OK)
//         {
//             ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
//         }
//         break;
//     case ESP_GATTS_OPEN_EVT:
//     case ESP_GATTS_CANCEL_OPEN_EVT:
//     case ESP_GATTS_CLOSE_EVT:
//     case ESP_GATTS_LISTEN_EVT:
//     case ESP_GATTS_CONGEST_EVT:
//     default:
//         break;
//     }
// }

// static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
// {
//     switch (event)
//     {
//     case ESP_GATTS_REG_EVT:
//         ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
//         gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true;
//         gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00;
//         gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
//         gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_B;

//         esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_B_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_B);
//         break;
//     case ESP_GATTS_READ_EVT:
//     {
//         ESP_LOGI(GATTS_TAG, "Characteristic read, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
//         esp_gatt_rsp_t rsp;
//         memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
//         rsp.attr_value.handle = param->read.handle;
//         rsp.attr_value.len = 4;
//         rsp.attr_value.value[0] = 0xde;
//         rsp.attr_value.value[1] = 0xed;
//         rsp.attr_value.value[2] = 0xbe;
//         rsp.attr_value.value[3] = 0xef;
//         esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
//                                     ESP_GATT_OK, &rsp);
//         break;
//     }
//     case ESP_GATTS_WRITE_EVT:
//     {
//         ESP_LOGI(GATTS_TAG, "Characteristic write, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
//         if (!param->write.is_prep)
//         {
//             ESP_LOGI(GATTS_TAG, "value len %d, value ", param->write.len);
//             ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
//             if (gl_profile_tab[PROFILE_B_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
//             {
//                 uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
//                 if (descr_value == 0x0001)
//                 {
//                     if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
//                     {
//                         ESP_LOGI(GATTS_TAG, "Notification enable");
//                         uint8_t notify_data[15];
//                         for (int i = 0; i < sizeof(notify_data); ++i)
//                         {
//                             notify_data[i] = i % 0xff;
//                         }
//                         // the size of notify_data[] need less than MTU size
//                         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
//                                                     sizeof(notify_data), notify_data, false);
//                     }
//                 }
//                 else if (descr_value == 0x0002)
//                 {
//                     if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
//                     {
//                         ESP_LOGI(GATTS_TAG, "Indication enable");
//                         uint8_t indicate_data[15];
//                         for (int i = 0; i < sizeof(indicate_data); ++i)
//                         {
//                             indicate_data[i] = i % 0xff;
//                         }
//                         // the size of indicate_data[] need less than MTU size
//                         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_B_APP_ID].char_handle,
//                                                     sizeof(indicate_data), indicate_data, true);
//                     }
//                 }
//                 else if (descr_value == 0x0000)
//                 {
//                     ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
//                 }
//                 else
//                 {
//                     ESP_LOGE(GATTS_TAG, "Unknown value");
//                 }
//             }
//         }
//         example_write_event_env(gatts_if, &b_prepare_write_env, param);
//         break;
//     }
//     case ESP_GATTS_EXEC_WRITE_EVT:
//         ESP_LOGI(GATTS_TAG, "Execute write");
//         esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
//         example_exec_write_event_env(&b_prepare_write_env, param);
//         break;
//     case ESP_GATTS_MTU_EVT:
//         ESP_LOGI(GATTS_TAG, "MTU exchange, MTU %d", param->mtu.mtu);
//         break;
//     case ESP_GATTS_UNREG_EVT:
//         break;
//     case ESP_GATTS_CREATE_EVT:
//         ESP_LOGI(GATTS_TAG, "Service create, status %d,  service_handle %d", param->create.status, param->create.service_handle);
//         gl_profile_tab[PROFILE_B_APP_ID].service_handle = param->create.service_handle;
//         gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
//         gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_B;

//         esp_ble_gatts_start_service(gl_profile_tab[PROFILE_B_APP_ID].service_handle);
//         b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
//         esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
//                                                         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
//                                                         b_property,
//                                                         NULL, NULL);
//         if (add_char_ret)
//         {
//             ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
//         }
//         break;
//     case ESP_GATTS_ADD_INCL_SRVC_EVT:
//         break;
//     case ESP_GATTS_ADD_CHAR_EVT:
//         ESP_LOGI(GATTS_TAG, "Characteristic add, status %d, attr_handle %d, service_handle %d",
//                  param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

//         gl_profile_tab[PROFILE_B_APP_ID].char_handle = param->add_char.attr_handle;
//         gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
//         gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
//         esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_B_APP_ID].service_handle, &gl_profile_tab[PROFILE_B_APP_ID].descr_uuid,
//                                      ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
//                                      NULL, NULL);
//         break;
//     case ESP_GATTS_ADD_CHAR_DESCR_EVT:
//         gl_profile_tab[PROFILE_B_APP_ID].descr_handle = param->add_char_descr.attr_handle;
//         ESP_LOGI(GATTS_TAG, "Descriptor add, status %d, attr_handle %d, service_handle %d",
//                  param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
//         break;
//     case ESP_GATTS_DELETE_EVT:
//         break;
//     case ESP_GATTS_START_EVT:
//         ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d",
//                  param->start.status, param->start.service_handle);
//         break;
//     case ESP_GATTS_STOP_EVT:
//         break;
//     case ESP_GATTS_CONNECT_EVT:
//         ESP_LOGI(GATTS_TAG, "Connected, conn_id %d, remote " ESP_BD_ADDR_STR "",
//                  param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
//         gl_profile_tab[PROFILE_B_APP_ID].conn_id = param->connect.conn_id;
//         break;
//     case ESP_GATTS_CONF_EVT:
//         ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
//         if (param->conf.status != ESP_GATT_OK)
//         {
//             ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
//         }
//         break;
//     case ESP_GATTS_DISCONNECT_EVT:
//     case ESP_GATTS_OPEN_EVT:
//     case ESP_GATTS_CANCEL_OPEN_EVT:
//     case ESP_GATTS_CLOSE_EVT:
//     case ESP_GATTS_LISTEN_EVT:
//     case ESP_GATTS_CONGEST_EVT:
//     default:
//         break;
//     }
// }

// static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
// {
//     /* If event is register event, store the gatts_if for each profile */
//     if (event == ESP_GATTS_REG_EVT)
//     {
//         if (param->reg.status == ESP_GATT_OK)
//         {
//             gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
//         }
//         else
//         {
//             ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
//                      param->reg.app_id,
//                      param->reg.status);
//             return;
//         }
//     }

//     /* If the gatts_if equal to profile A, call profile A cb handler,
//      * so here call each profile's callback */
//     do
//     {
//         int idx;
//         for (idx = 0; idx < PROFILE_NUM; idx++)
//         {
//             if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
//                 gatts_if == gl_profile_tab[idx].gatts_if)
//             {
//                 if (gl_profile_tab[idx].gatts_cb)
//                 {
//                     gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
//                 }
//             }
//         }
//     } while (0);
// }

// void app_main(void)
// {
//     esp_err_t ret;

//     // Initialize NVS.
//     ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
//     {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

// #if CONFIG_EXAMPLE_CI_PIPELINE_ID
//     memcpy(test_device_name, esp_bluedroid_get_example_name(), ESP_BLE_ADV_NAME_LEN_MAX);
// #endif

//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ret = esp_bt_controller_init(&bt_cfg);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_bluedroid_init();
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }
//     ret = esp_bluedroid_enable();
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_ble_gatts_register_callback(gatts_event_handler);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
//         return;
//     }
//     ret = esp_ble_gap_register_callback(gap_event_handler);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
//         return;
//     }
//     ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
//         return;
//     }
//     ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
//         return;
//     }
//     esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
//     if (local_mtu_ret)
//     {
//         ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
//     }

//     return;
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /* Delay between advertisement. Advertisment will only be transmitted for a short period of time (20ms) and the device will go to sleep.
// Higher delay = less power consumption, but more inaccurate tracking
//  */
// #define DELAY_IN_S 60
// /* Define how often (long) a key will be reused after switching to the next one
// This is for using less keys after all. The interval for one key is (DELAY_IN_S * REUSE_CYCLES => 60s * 30 cycles = changes key every 30 min)
// Smaller number of cycles = key changes more often, but more keys needed.
//  */
// #define REUSE_CYCLES 30

// #define GATTS_TAG "GATTS_DEMO"

// /// Declare the static function
// static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
// // static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// #define GATTS_SERVICE_UUID_TEST_A 0x00FF
// #define GATTS_CHAR_UUID_TEST_A 0xFF01
// #define GATTS_DESCR_UUID_TEST_A 0x3333
// #define GATTS_NUM_HANDLE_TEST_A 4

// // #define GATTS_SERVICE_UUID_TEST_B 0x00EE
// // #define GATTS_CHAR_UUID_TEST_B 0xEE01
// // #define GATTS_DESCR_UUID_TEST_B 0x2222
// // #define GATTS_NUM_HANDLE_TEST_B 4

// static char test_device_name[ESP_BLE_ADV_DATA_LEN_MAX] = "ESP_GATTS_DEMO";
// uint8_t ch_map[5] = {0x00, 0x00, 0x00, 0x10, 0x00}; // Bit 36 được đặt là 1

// #define TEST_MANUFACTURER_DATA_LEN 17

// #define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

// #define PREPARE_BUF_MAX_SIZE 1024

// static const char *LOG_TAG = "macless_haystack";

// /** Callback function for BT events */
// static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// /** Random device address */
// static esp_bd_addr_t rnd_addr = {0xFF, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// /** Advertisement payload */
// static uint8_t adv_data[31] = {
//     0x1e,       /* Length (30) */
//     0xff,       /* Manufacturer Specific Data (type 0xff) */
//     0x4c, 0x00, /* Company ID (Apple) */
//     0x12, 0x19, /* Offline Finding type and length */
//     0x00,       /* State */
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, /* First two bits */
//     0x00, /* Hint (0x00) */
// };

// /* https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gap_ble.html#_CPPv420esp_ble_adv_params_t */
// static esp_ble_adv_params_t ble_adv_params = {
//     // Advertising min interval:
//     // Minimum advertising interval for undirected and low duty cycle
//     // directed advertising. Range: 0x0020 to 0x4000 Default: N = 0x0800
//     // (1.28 second) Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
//     .adv_int_min = 0x0020, // 20ms
//     // Advertising max interval:
//     // Maximum advertising interval for undirected and low duty cycle
//     // directed advertising. Range: 0x0020 to 0x4000 Default: N = 0x0800
//     // (1.28 second) Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
//     .adv_int_max = 0x0020, // 20ms
//     // Advertisement type
//     .adv_type = ADV_TYPE_NONCONN_IND,
//     // Use the random address
//     .own_addr_type = BLE_ADDR_TYPE_RANDOM,
//     // All channels
//     .channel_map =  ADV_CHNL_ALL,
//     // Allow both scan and connection requests from anyone.
//     .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
// };

// static uint8_t char1_str[] = {0x11, 0x22, 0x33};
// static esp_gatt_char_prop_t a_property = 0;
// // static esp_gatt_char_prop_t b_property = 0;

// static esp_attr_value_t gatts_demo_char1_val =
//     {
//         .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
//         .attr_len = sizeof(char1_str),
//         .attr_value = char1_str,
// };

// static uint8_t adv_config_done = 0;
// int load_bytes_from_partition(uint8_t *dst, size_t size, int offset)
// {
//     const esp_partition_t *keypart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS, "key");
//     if (keypart == NULL)
//     {
//         ESP_LOGE(LOG_TAG, "Could not find key partition");
//         return 1;
//     }
//     esp_err_t status;
//     status = esp_partition_read(keypart, offset, dst, size);
//     if (status != ESP_OK)
//     {
//         ESP_LOGE(LOG_TAG, "Could not read key from partition: %s", esp_err_to_name(status));
//     }
//     return status;
// }

// // static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
// // {
// //     esp_err_t err;

// //     switch (event)
// //     {
// //     case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
// //         esp_ble_gap_start_advertising(&ble_adv_params);
// //         break;

// //     case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
// //         // adv start complete event to indicate adv start successfully or failed
// //         if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
// //         {
// //             ESP_LOGE(LOG_TAG, "advertising start failed: %s", esp_err_to_name(err));
// //         }
// //         else
// //         {
// //             ESP_LOGI(LOG_TAG, "advertising has started.");
// //         }
// //         break;

// //     case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
// //         if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
// //         {
// //             ESP_LOGE(LOG_TAG, "adv stop failed: %s", esp_err_to_name(err));
// //         }
// //         else
// //         {
// //             ESP_LOGI(LOG_TAG, "stop adv successfully");
// //         }
// //         break;
// //     default:
// //         break;
// //     }
// // }

// void set_addr_from_key(esp_bd_addr_t addr, uint8_t *public_key)
// {
//     addr[0] = public_key[0] | 0b11000000;
//     addr[1] = public_key[1];
//     addr[2] = public_key[2];
//     addr[3] = public_key[3];
//     addr[4] = public_key[4];
//     addr[5] = public_key[5];
// }

// void set_payload_from_key(uint8_t *payload, uint8_t *public_key)
// {
//     /* copy last 22 bytes */
//     memcpy(&payload[7], &public_key[6], 22);
//     /* append two bits of public key */
//     payload[29] = public_key[0] >> 6;
// }

// uint8_t get_key_count()
// {
//     uint8_t keyCount[1];
//     if (load_bytes_from_partition(keyCount, sizeof(keyCount), 0) != ESP_OK)
//     {
//         ESP_LOGE(LOG_TAG, "Could not read the key count, stopping.");
//         return 0;
//     }
//     ESP_LOGE(LOG_TAG, "Found %i keys", keyCount[0]);
//     return keyCount[0];
// }
// static uint8_t public_key[28];
// #define adv_config_flag (1 << 0)
// #define scan_rsp_config_flag (1 << 1)

// #ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
// static uint8_t raw_adv_data[] = {
//     /* Flags */
//     0x02, ESP_BLE_AD_TYPE_FLAG, 0x06, // Length 2, Data Type ESP_BLE_AD_TYPE_FLAG, Data 1 (LE General Discoverable Mode, BR/EDR Not Supported)
//     /* TX Power Level */
//     0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB, // Length 2, Data Type ESP_BLE_AD_TYPE_TX_PWR, Data 2 (-21)
//     /* Complete 16-bit Service UUIDs */
//     0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xAB, 0xCD // Length 3, Data Type ESP_BLE_AD_TYPE_16SRV_CMPL, Data 3 (UUID)
// };

// static uint8_t raw_scan_rsp_data[] = {
//     /* Complete Local Name */
//     0x0F, ESP_BLE_AD_TYPE_NAME_CMPL, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D', 'E', 'M', 'O' // Length 15, Data Type ESP_BLE_AD_TYPE_NAME_CMPL, Data (ESP_GATTS_DEMO)
// };
// #else

// // The length of adv data must be less than 31 bytes
// // static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
// // adv data
// static esp_ble_adv_data_t adv_data1 = {
//     .set_scan_rsp = false,
//     .include_name = true,
//     .include_txpower = false,
//     .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
//     .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
//     .appearance = 0x00,
//     .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
//     .p_manufacturer_data = NULL, //&test_manufacturer[0],
//     .service_data_len = 0,
//     .p_service_data = NULL,
//     // .service_uuid_len = sizeof(adv_service_uuid128),
//     // .p_service_uuid = adv_service_uuid128,
//     .service_uuid_len = sizeof(adv_data),
//     .p_service_uuid = adv_data,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
// };
// // scan response data
// static esp_ble_adv_data_t scan_rsp_data = {
//     .set_scan_rsp = true,
//     .include_name = true,
//     .include_txpower = true,
//     //.min_interval = 0x0006,
//     //.max_interval = 0x0010,
//     .appearance = 0x00,
//     .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
//     .p_manufacturer_data = NULL, //&test_manufacturer[0],
//     .service_data_len = 0,
//     .p_service_data = NULL,
//     // .service_uuid_len = sizeof(adv_service_uuid128),
//     // .p_service_uuid = adv_service_uuid128,
//     .service_uuid_len = sizeof(adv_data),
//     .p_service_uuid = adv_data,
//     .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
// };

// #endif /* CONFIG_SET_RAW_ADV_DATA */

// // static esp_ble_adv_params_t adv_params = {
// //     .adv_int_min = 0x20,
// //     .adv_int_max = 0x40,
// //     .adv_type = ADV_TYPE_IND,
// //     .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
// //     //.peer_addr            =
// //     //.peer_addr_type       =
// //     .channel_map = 0x10,
// //     .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
// // };

// #define PROFILE_NUM 2
// #define PROFILE_A_APP_ID 0
// #define PROFILE_B_APP_ID 1

// struct gatts_profile_inst
// {
//     esp_gatts_cb_t gatts_cb;
//     uint16_t gatts_if;
//     uint16_t app_id;
//     uint16_t conn_id;
//     uint16_t service_handle;
//     esp_gatt_srvc_id_t service_id;
//     uint16_t char_handle;
//     esp_bt_uuid_t char_uuid;
//     esp_gatt_perm_t perm;
//     esp_gatt_char_prop_t property;
//     uint16_t descr_handle;
//     esp_bt_uuid_t descr_uuid;
// };

// /* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
// static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
//     [PROFILE_A_APP_ID] = {
//         .gatts_cb = gatts_profile_a_event_handler,
//         .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//     },
//     // [PROFILE_B_APP_ID] = {
//     //     .gatts_cb = gatts_profile_b_event_handler, /* This demo does not implement, similar as profile A */
//     //     .gatts_if = ESP_GATT_IF_NONE,              /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
//     // },
// };

// typedef struct
// {
//     uint8_t *prepare_buf;
//     int prepare_len;
// } prepare_type_env_t;

// static prepare_type_env_t a_prepare_write_env;
// static prepare_type_env_t b_prepare_write_env;

// void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
// void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

// static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
// {
//     switch (event)
//     {
// #ifdef CONFIG_SET_RAW_ADV_DATA
//     case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
//         adv_config_done &= (~adv_config_flag);
//         if (adv_config_done == 0)
//         {
//             esp_ble_gap_start_advertising(&adv_params);
//         }
//         break;
//     case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
//         adv_config_done &= (~scan_rsp_config_flag);
//         if (adv_config_done == 0)
//         {
//             esp_ble_gap_start_advertising(&adv_params);
//         }
//         break;
// #else
//     case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
//         adv_config_done &= (~adv_config_flag);
//         if (adv_config_done == 0)
//         {
//             // esp_ble_gap_start_advertising(&adv_params);
//             esp_ble_gap_start_advertising(&ble_adv_params);
//         }
//         break;
//     case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
//         adv_config_done &= (~scan_rsp_config_flag);
//         if (adv_config_done == 0)
//         {
//             // esp_ble_gap_start_advertising(&adv_params);
//              esp_ble_gap_start_advertising(&ble_adv_params);
//         }
//         break;
// #endif
//     case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
//         // advertising start complete event to indicate advertising start successfully or failed
//         if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
//         {
//             ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
//             break;
//         }
//         ESP_LOGI(GATTS_TAG, "Advertising start successfully");
//         break;
//     case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
//         if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
//         {
//             ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
//             break;
//         }
//         ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
//         break;
//     case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
//         ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
//                  param->update_conn_params.status,
//                  param->update_conn_params.conn_int,
//                  param->update_conn_params.latency,
//                  param->update_conn_params.timeout);
//         break;
//     case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
//         ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
//                  param->pkt_data_length_cmpl.status,
//                  param->pkt_data_length_cmpl.params.rx_len,
//                  param->pkt_data_length_cmpl.params.tx_len);
//         break;
//     default:
//         break;
//     }
// }

// void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
// {
//     esp_gatt_status_t status = ESP_GATT_OK;
//     if (param->write.need_rsp)
//     {
//         if (param->write.is_prep)
//         {
//             if (param->write.offset > PREPARE_BUF_MAX_SIZE)
//             {
//                 status = ESP_GATT_INVALID_OFFSET;
//             }
//             else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
//             {
//                 status = ESP_GATT_INVALID_ATTR_LEN;
//             }
//             if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
//             {
//                 prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
//                 prepare_write_env->prepare_len = 0;
//                 if (prepare_write_env->prepare_buf == NULL)
//                 {
//                     ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
//                     status = ESP_GATT_NO_RESOURCES;
//                 }
//             }

//             esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
//             if (gatt_rsp)
//             {
//                 gatt_rsp->attr_value.len = param->write.len;
//                 gatt_rsp->attr_value.handle = param->write.handle;
//                 gatt_rsp->attr_value.offset = param->write.offset;
//                 gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
//                 memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
//                 esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
//                 if (response_err != ESP_OK)
//                 {
//                     ESP_LOGE(GATTS_TAG, "Send response error\n");
//                 }
//                 free(gatt_rsp);
//             }
//             else
//             {
//                 ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
//                 status = ESP_GATT_NO_RESOURCES;
//             }
//             if (status != ESP_GATT_OK)
//             {
//                 return;
//             }
//             memcpy(prepare_write_env->prepare_buf + param->write.offset,
//                    param->write.value,
//                    param->write.len);
//             prepare_write_env->prepare_len += param->write.len;
//         }
//         else
//         {
//             esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
//         }
//     }
// }

// void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
// {
//     if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
//     {
//         ESP_LOG_BUFFER_HEX(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
//     }
//     else
//     {
//         ESP_LOGI(GATTS_TAG, "Prepare write cancel");
//     }
//     if (prepare_write_env->prepare_buf)
//     {
//         free(prepare_write_env->prepare_buf);
//         prepare_write_env->prepare_buf = NULL;
//     }
//     prepare_write_env->prepare_len = 0;
// }

// static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
// {
//     switch (event)
//     {
//      case ESP_GATTS_REG_EVT:
//         ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
//         gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
//         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
//         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
//         gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

//         esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(test_device_name);
//         if (set_dev_name_ret){
//             ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
//         }
// #ifdef CONFIG_EXAMPLE_SET_RAW_ADV_DATA
//         esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
//         if (raw_adv_ret){
//             ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
//         }
//         adv_config_done |= adv_config_flag;
//         esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
//         if (raw_scan_ret){
//             ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
//         }
//         adv_config_done |= scan_rsp_config_flag;
// #else
//         //config adv data
//         esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
//         if (ret){
//             ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
//         }
//         adv_config_done |= adv_config_flag;
//         //config scan response data
//         ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
//         if (ret){
//             ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
//         }
//         adv_config_done |= scan_rsp_config_flag;

// #endif
//         esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
//         break;
//     case ESP_GATTS_READ_EVT: {
//         ESP_LOGI(GATTS_TAG, "Characteristic read, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
//         esp_gatt_rsp_t rsp;
//         memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
//         rsp.attr_value.handle = param->read.handle;
//         rsp.attr_value.len = 4;
//         rsp.attr_value.value[0] = 0xde;
//         rsp.attr_value.value[1] = 0xed;
//         rsp.attr_value.value[2] = 0xbe;
//         rsp.attr_value.value[3] = 0xef;
//         esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
//                                     ESP_GATT_OK, &rsp);
//         break;
//     }
//     case ESP_GATTS_WRITE_EVT:
//     {
//         ESP_LOGI(GATTS_TAG, "Characteristic write, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
//         if (!param->write.is_prep)
//         {
//             ESP_LOGI(GATTS_TAG, "value len %d, value ", param->write.len);
//             ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
//             if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
//             {
//                 uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
//                 if (descr_value == 0x0001)
//                 {
//                     if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
//                     {
//                         ESP_LOGI(GATTS_TAG, "Notification enable");
//                         uint8_t notify_data[15];
//                         for (int i = 0; i < sizeof(notify_data); ++i)
//                         {
//                             notify_data[i] = i % 0xff;
//                         }
//                         // the size of notify_data[] need less than MTU size
//                         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
//                                                     sizeof(notify_data), notify_data, false);
//                     }
//                 }
//                 else if (descr_value == 0x0002)
//                 {
//                     if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
//                     {
//                         ESP_LOGI(GATTS_TAG, "Indication enable");
//                         uint8_t indicate_data[15];
//                         for (int i = 0; i < sizeof(indicate_data); ++i)
//                         {
//                             indicate_data[i] = i % 0xff;
//                         }
//                         // the size of indicate_data[] need less than MTU size
//                         esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
//                                                     sizeof(indicate_data), indicate_data, true);
//                     }
//                 }
//                 else if (descr_value == 0x0000)
//                 {
//                     ESP_LOGI(GATTS_TAG, "Notification/Indication disable");
//                 }
//                 else
//                 {
//                     ESP_LOGE(GATTS_TAG, "Unknown descriptor value");
//                     ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);
//                 }
//             }
//         }
//         example_write_event_env(gatts_if, &a_prepare_write_env, param);
//         break;
//     }

//     default:
//         break;
//     }
// }

// static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
// {
//     /* If event is register event, store the gatts_if for each profile */
//     if (event == ESP_GATTS_REG_EVT)
//     {
//         if (param->reg.status == ESP_GATT_OK)
//         {
//             gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
//         }
//         else
//         {
//             ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
//                      param->reg.app_id,
//                      param->reg.status);
//             return;
//         }
//     }

//     /* If the gatts_if equal to profile A, call profile A cb handler,
//      * so here call each profile's callback */
//     do
//     {
//         int idx;
//         for (idx = 0; idx < PROFILE_NUM; idx++)
//         {
//             if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
//                 gatts_if == gl_profile_tab[idx].gatts_if)
//             {
//                 if (gl_profile_tab[idx].gatts_cb)
//                 {
//                     gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
//                 }
//             }
//         }
//     } while (0);
// }

// void app_main(void)
// {
//     // ESP_ERROR_CHECK(nvs_flash_init());
//     // ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
//     // esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     // esp_bt_controller_init(&bt_cfg);
//     // esp_bt_controller_enable(ESP_BT_MODE_BLE);
//     esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
//     esp_bluedroid_init_with_cfg(&bluedroid_cfg);
//     esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
//     // esp_bluedroid_enable();

//     ESP_LOGI(LOG_TAG, "application initialized");

//     /* Start with a random index */
//     uint8_t key_count = get_key_count();
//     uint8_t key_index = (esp_random() % key_count);
//     uint8_t cycle = 0;
//     esp_err_t ret;

//     // Initialize NVS.
//     ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
//     {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

// #if CONFIG_EXAMPLE_CI_PIPELINE_ID
//     memcpy(test_device_name, esp_bluedroid_get_example_name(), ESP_BLE_ADV_NAME_LEN_MAX);
// #endif

//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     ret = esp_bt_controller_init(&bt_cfg);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     // ret = esp_bluedroid_init();
//     // if (ret)
//     // {
//     //     ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
//     //     return;
//     // }
//     ret = esp_bluedroid_enable();
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
//         return;
//     }

//     ret = esp_ble_gatts_register_callback(gatts_event_handler);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
//         return;
//     }
//     ret = esp_ble_gap_register_callback(gap_event_handler);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
//         return;
//     }
//     ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
//         return;
//     }
//     ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
//     if (ret)
//     {
//         ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
//         return;
//     }
//     esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
//     if (local_mtu_ret)
//     {
//         ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
//     }
//     while (true)
//     {
//         esp_err_t status;
//         // Shift for keycount size + keylength * index
//         int address = 1 + (key_index * sizeof(public_key));
//         ESP_LOGI(LOG_TAG, "Loading key with index %d at address %d", key_index, address);
//         if (load_bytes_from_partition(public_key, sizeof(public_key), address) != ESP_OK)
//         {
//             ESP_LOGE(LOG_TAG, "Could not read the key, stopping.");
//             return;
//         }
//         ESP_LOGI(LOG_TAG, "using key with start %02x %02x", public_key[0], public_key[1]);
//         set_addr_from_key(rnd_addr, public_key);
//         set_payload_from_key(adv_data, public_key);

//         ESP_LOGI(LOG_TAG, "using device address: %02x %02x %02x %02x %02x %02x", rnd_addr[0], rnd_addr[1], rnd_addr[2], rnd_addr[3], rnd_addr[4], rnd_addr[5]);
//         // register the scan callback function to the gap module
//         // if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK)
//         if ((status = esp_ble_gap_register_callback(gap_event_handler)) != ESP_OK)
//         {
//             ESP_LOGE(LOG_TAG, "gap register error: %s", esp_err_to_name(status));
//             return;
//         }

//         if ((status = esp_ble_gap_set_rand_addr(rnd_addr)) != ESP_OK)
//         {
//             ESP_LOGE(LOG_TAG, "couldn't set random address: %s", esp_err_to_name(status));
//             return;
//         }
//         if ((esp_ble_gap_config_adv_data_raw((uint8_t *)&adv_data, sizeof(adv_data))) != ESP_OK)
//         {
//             ESP_LOGE(LOG_TAG, "couldn't configure BLE adv: %s", esp_err_to_name(status));
//             return;
//         }
//         ESP_LOGI(LOG_TAG, "Sending beacon (with key index %d)", key_index);
//         vTaskDelay(10);
//         esp_ble_gap_stop_advertising(); // Stop immediately after first beacon

//         vTaskDelay(10);
//         ESP_LOGI(LOG_TAG, "Going to sleep");
//         vTaskDelay(10);
//         esp_sleep_enable_timer_wakeup(DELAY_IN_S * 1000000); // sleep
//         esp_light_sleep_start();

//         // Execution continues here after wakeup
//         ESP_LOGI(LOG_TAG, "Returned from light sleep");
//         if (cycle >= REUSE_CYCLES)
//         {
//             ESP_LOGI(LOG_TAG, "Max cycles %d are reached. Changing key ", cycle);
//             key_index = (key_index + 1) % key_count; // Back to zero if out of range
//             cycle = 0;
//         }
//         else
//         {
//             ESP_LOGI(LOG_TAG, "Current cycle is %d. Reusing key. ", cycle);
//             cycle++;
//         }
//     }

//     return;
// }

/////////////////////////////////////////////////////////////////////CONNECTION//////////////////////////////////////////////////////////////////////////
#define GATTS_TAG "GATTS_DEMO"
#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define SAMPLE_DEVICE_NAME "ESP32_BLE"
#define SVC_INST_ID 0

static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
} gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
         if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed");
            } else {
                ESP_LOGI(GATTS_TAG, "Advertising start successfully");
            }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT");
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT");
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep) {
                ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
                esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                // Xử lý dữ liệu nhận được ở đây
            }
            break;
    default:
        break;
    }
}

void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_profile_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_APP_ID);
}

/////////////////////////////////////////////////////////////////////////BEACON////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////BEACON////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////BEACON////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// /* Delay between advertisement. Advertisment will only be transmitted for a short period of time (20ms) and the device will go to sleep.
// Higher delay = less power consumption, but more inaccurate tracking
//  */
// #define DELAY_IN_S 60
// /* Define how often (long) a key will be reused after switching to the next one
// This is for using less keys after all. The interval for one key is (DELAY_IN_S * REUSE_CYCLES => 60s * 30 cycles = changes key every 30 min)
// Smaller number of cycles = key changes more often, but more keys needed.
//  */
// #define REUSE_CYCLES 30

// static const char *LOG_TAG = "macless_haystack";

// /** Callback function for BT events */
// static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// /** Random device address */
// static esp_bd_addr_t rnd_addr = {0xFF, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// /** Advertisement payload */
// static uint8_t adv_data[31] = {
//     0x1e,       /* Length (30) */
//     0xff,       /* Manufacturer Specific Data (type 0xff) */
//     0x4c, 0x00, /* Company ID (Apple) */
//     0x12, 0x19, /* Offline Finding type and length */
//     0x00,       /* State */
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//     0x00, /* First two bits */
//     0x00, /* Hint (0x00) */
// };

// /* https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gap_ble.html#_CPPv420esp_ble_adv_params_t */
// static esp_ble_adv_params_t ble_adv_params = {
//     // Advertising min interval:
//     // Minimum advertising interval for undirected and low duty cycle
//     // directed advertising. Range: 0x0020 to 0x4000 Default: N = 0x0800
//     // (1.28 second) Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
//     .adv_int_min = 0x0020, // 20ms
//     // Advertising max interval:
//     // Maximum advertising interval for undirected and low duty cycle
//     // directed advertising. Range: 0x0020 to 0x4000 Default: N = 0x0800
//     // (1.28 second) Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
//     .adv_int_max = 0x0020, // 20ms
//     // Advertisement type
//     .adv_type = ADV_TYPE_NONCONN_IND,
//     // Use the random address
//     .own_addr_type = BLE_ADDR_TYPE_RANDOM,
//     // All channels
//     .channel_map = ADV_CHNL_ALL,
//     // Allow both scan and connection requests from anyone.
//     .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
// };

// int load_bytes_from_partition(uint8_t *dst, size_t size, int offset)
// {
//     const esp_partition_t *keypart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS, "key");
//     if (keypart == NULL)
//     {
//         ESP_LOGE(LOG_TAG, "Could not find key partition");
//         return 1;
//     }
//     esp_err_t status;
//     status = esp_partition_read(keypart, offset, dst, size);
//     if (status != ESP_OK)
//     {
//         ESP_LOGE(LOG_TAG, "Could not read key from partition: %s", esp_err_to_name(status));
//     }
//     return status;
// }

// static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
// {
//     esp_err_t err;

//     switch (event)
//     {
//     case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
//         esp_ble_gap_start_advertising(&ble_adv_params);
//         break;

//     case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
//         // adv start complete event to indicate adv start successfully or failed
//         if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
//         {
//             ESP_LOGE(LOG_TAG, "advertising start failed: %s", esp_err_to_name(err));
//         }
//         else
//         {
//             ESP_LOGI(LOG_TAG, "advertising has started.");
//         }
//         break;

//     case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
//         if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
//         {
//             ESP_LOGE(LOG_TAG, "adv stop failed: %s", esp_err_to_name(err));
//         }
//         else
//         {
//             ESP_LOGI(LOG_TAG, "stop adv successfully");
//         }
//         break;
//     default:
//         break;
//     }
// }

// void set_addr_from_key(esp_bd_addr_t addr, uint8_t *public_key)
// {
//     addr[0] = public_key[0] | 0b11000000;
//     addr[1] = public_key[1];
//     addr[2] = public_key[2];
//     addr[3] = public_key[3];
//     addr[4] = public_key[4];
//     addr[5] = public_key[5];
// }

// void set_payload_from_key(uint8_t *payload, uint8_t *public_key)
// {
//     /* copy last 22 bytes */
//     memcpy(&payload[7], &public_key[6], 22);
//     /* append two bits of public key */
//     payload[29] = public_key[0] >> 6;
// }

// uint8_t get_key_count()
// {
//     uint8_t keyCount[1];
//     if (load_bytes_from_partition(keyCount, sizeof(keyCount), 0) != ESP_OK)
//     {
//         ESP_LOGE(LOG_TAG, "Could not read the key count, stopping.");
//         return 0;
//     }
//     ESP_LOGE(LOG_TAG, "Found %i keys", keyCount[0]);
//     return keyCount[0];
// }
// static uint8_t public_key[28];
// void app_main(void)
// {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     esp_bt_controller_init(&bt_cfg);
//     esp_bt_controller_enable(ESP_BT_MODE_BLE);
//     esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
//     esp_bluedroid_init_with_cfg(&bluedroid_cfg);
//     esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
//     esp_bluedroid_enable();

//     ESP_LOGI(LOG_TAG, "application initialized");

//     /* Start with a random index */
//     uint8_t key_count = get_key_count();
//     uint8_t key_index = (esp_random() % key_count);
//     uint8_t cycle = 0;
//     while (true)
//     {
//         esp_err_t status;
//         // Shift for keycount size + keylength * index
//         int address = 1 + (key_index * sizeof(public_key));
//         ESP_LOGI(LOG_TAG, "Loading key with index %d at address %d", key_index, address);
//         if (load_bytes_from_partition(public_key, sizeof(public_key), address) != ESP_OK)
//         {
//             ESP_LOGE(LOG_TAG, "Could not read the key, stopping.");
//             return;
//         }
//         ESP_LOGI(LOG_TAG, "using key with start %02x %02x", public_key[0], public_key[1]);
//         set_addr_from_key(rnd_addr, public_key);
//         set_payload_from_key(adv_data, public_key);

//         ESP_LOGI(LOG_TAG, "using device address: %02x %02x %02x %02x %02x %02x", rnd_addr[0], rnd_addr[1], rnd_addr[2], rnd_addr[3], rnd_addr[4], rnd_addr[5]);
//         // register the scan callback function to the gap module
//         if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK)
//         {
//             ESP_LOGE(LOG_TAG, "gap register error: %s", esp_err_to_name(status));
//             return;
//         }

//         if ((status = esp_ble_gap_set_rand_addr(rnd_addr)) != ESP_OK)
//         {
//             ESP_LOGE(LOG_TAG, "couldn't set random address: %s", esp_err_to_name(status));
//             return;
//         }
//         if ((esp_ble_gap_config_adv_data_raw((uint8_t *)&adv_data, sizeof(adv_data))) != ESP_OK)
//         {
//             ESP_LOGE(LOG_TAG, "couldn't configure BLE adv: %s", esp_err_to_name(status));
//             return;
//         }
//         ESP_LOGI(LOG_TAG, "Sending beacon (with key index %d)", key_index);
//         vTaskDelay(10);
//         esp_ble_gap_stop_advertising(); // Stop immediately after first beacon

//         vTaskDelay(10);
//         ESP_LOGI(LOG_TAG, "Going to sleep");
//         vTaskDelay(10);
//         esp_sleep_enable_timer_wakeup(DELAY_IN_S * 1000000); // sleep
//         esp_light_sleep_start();

//         // Execution continues here after wakeup
//         ESP_LOGI(LOG_TAG, "Returned from light sleep");
//         if (cycle >= REUSE_CYCLES)
//         {
//             ESP_LOGI(LOG_TAG, "Max cycles %d are reached. Changing key ", cycle);
//             key_index = (key_index + 1) % key_count; // Back to zero if out of range
//             cycle = 0;
//         }
//         else
//         {
//             ESP_LOGI(LOG_TAG, "Current cycle is %d. Reusing key. ", cycle);
//             cycle++;
//         }
//     }
// }
