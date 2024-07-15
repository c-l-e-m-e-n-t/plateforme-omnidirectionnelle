#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

// Définition des broches pour les moteurs
#define MOTOR_FL_PWM 25
#define MOTOR_FL_IN1 26
#define MOTOR_FL_IN2 27
#define MOTOR_BL_PWM 14
#define MOTOR_BL_IN1 12
#define MOTOR_BL_IN2 13
#define MOTOR_FR_PWM 15
#define MOTOR_FR_IN1 2
#define MOTOR_FR_IN2 4
#define MOTOR_BR_PWM 5
#define MOTOR_BR_IN1 18
#define MOTOR_BR_IN2 19

// Configuration PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Résolution du duty cycle sur 8 bits
#define LEDC_FREQUENCY          (5000) // Fréquence PWM en Hz

// Structure pour représenter un moteur
typedef struct {
    int pwm_channel;
    int pwm_pin;
    int in1_pin;
    int in2_pin;
} motor_t;

// Déclaration des moteurs
motor_t motor_fl = {0, MOTOR_FL_PWM, MOTOR_FL_IN1, MOTOR_FL_IN2};
motor_t motor_fr = {1, MOTOR_FR_PWM, MOTOR_FR_IN1, MOTOR_FR_IN2};
motor_t motor_bl = {2, MOTOR_BL_PWM, MOTOR_BL_IN1, MOTOR_BL_IN2};
motor_t motor_br = {3, MOTOR_BR_PWM, MOTOR_BR_IN1, MOTOR_BR_IN2};

// Fonction pour initialiser un moteur
void init_motor(motor_t motor) {
    // Configuration du canal PWM
    ledc_channel_config_t ledc_channel = {
        .channel    = motor.pwm_channel,
        .duty       = 0,
        .gpio_num   = motor.pwm_pin,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&ledc_channel);

    // Configuration des broches de direction
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << motor.in1_pin) | (1ULL << motor.in2_pin),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
}

// Structure pour les commandes de mouvement
typedef struct {
    int16_t forward;
    int16_t right;
    int16_t rotation;
} __attribute__((packed)) movement_command_t;

static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static esp_gatt_char_prop_t a_property = 0;
static esp_attr_value_t gatts_demo_char1_val = {
    .attr_max_len = sizeof(movement_command_t),
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
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
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst gl_profile_tab[1] = {
    [0] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

// Fonction pour contrôler un moteur
void set_motor_speed(motor_t motor, int speed) {
    // La vitesse va de -255 à 255
    if (speed > 0) {
        gpio_set_level(motor.in1_pin, 1);
        gpio_set_level(motor.in2_pin, 0);
    } else if (speed < 0) {
        gpio_set_level(motor.in1_pin, 0);
        gpio_set_level(motor.in2_pin, 1);
    } else {
        gpio_set_level(motor.in1_pin, 0);
        gpio_set_level(motor.in2_pin, 0);
    }
    ledc_set_duty(LEDC_MODE, motor.pwm_channel, abs(speed));
    ledc_update_duty(LEDC_MODE, motor.pwm_channel);
}

// Fonction pour mettre à jour les mouvements de la plateforme
void update_platform_movement(movement_command_t cmd) {
    int fl_speed = cmd.forward - cmd.right - cmd.rotation;
    int fr_speed = cmd.forward + cmd.right + cmd.rotation;
    int bl_speed = cmd.forward + cmd.right - cmd.rotation;
    int br_speed = cmd.forward - cmd.right + cmd.rotation;

    set_motor_speed(motor_fl, fl_speed);
    set_motor_speed(motor_fr, fr_speed);
    set_motor_speed(motor_bl, bl_speed);
    set_motor_speed(motor_br, br_speed);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~adv_config_flag);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~scan_rsp_config_flag);
            if (adv_config_done == 0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
            }
            break;
        default:
            break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(GATTS_DEMO_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[0].service_id, GATTS_NUM_HANDLE);
            break;
        case ESP_GATTS_WRITE_EVT:
            if (param->write.len == sizeof(movement_command_t)) {
                movement_command_t cmd;
                memcpy(&cmd, param->write.value, sizeof(movement_command_t));
                update_platform_movement(cmd);
            }
            break;
        // Ajoutez d'autres cas si nécessaire
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[0].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < 1; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


// Tâche pour gérer les mouvements de la plateforme
void platform_control_task(void *pvParameters) {
    while (1) {
        update_platform_movement(current_command);
        vTaskDelay(50 / portTICK_PERIOD_MS);  // Mise à jour toutes les 50ms
    }
}

void app_main(void) {

    // Initialisation de NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialisation du Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(0);
    // Configuration du timer PWM
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Initialisation des moteurs
    init_motor(motor_fl);
    init_motor(motor_fr);
    init_motor(motor_bl);
    init_motor(motor_br);

    // Initialisation du Bluetooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    // Initialisation du SPP
    esp_spp_register_callback(esp_spp_cb);
    esp_spp_init(ESP_SPP_MODE_CB);

    // Création de la tâche de contrôle de la plateforme
    xTaskCreate(platform_control_task, "platform_control", 2048, NULL, 5, NULL);
}