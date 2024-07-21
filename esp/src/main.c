#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"

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

#define DEVICE_NAME "ESP32_OMNI_PLATFORM"
#define SPP_SERVER_NAME "SPP_SERVER"

static const char *TAG = "SPP_SERVER";

// Callback pour les événements SPP
void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_MODE_CB, 0, SPP_SERVER_NAME);
            break;
        case ESP_SPP_DATA_IND_EVT:
            ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT");
            // Recevoir les données
            char data[1024];
            int length = param->data_ind.len;
            memcpy(data, param->data_ind.data, length);
            data[length] = '\0';
            ESP_LOGI(TAG, "Received data: %s", data);
            processCommand(data);
            break;
        case ESP_SPP_CONG_EVT:
            ESP_LOGI(TAG, "ESP_SPP_CONG_EVT");
            break;
        case ESP_SPP_WRITE_EVT:
            ESP_LOGI(TAG, "ESP_SPP_WRITE_EVT");
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
            break;
        default:
            break;
    }
}

// Fonction pour traiter les commandes reçues
void processCommand(const char *command) {
    // Exemple de commande: "50 -30 100 0"
    int x1, y1, x2, y2;
    sscanf(command, "%d %d %d %d", &x1, &y1, &x2, &y2);

    // Calcul des vitesses des moteurs en fonction des valeurs des joysticks
    int speedFL = x1 - y1 - x2;
    int speedFR = x1 + y1 + x2;
    int speedBL = x1 + y1 - x2;
    int speedBR = x1 - y1 + x2;

    // Appliquer les vitesses aux moteurs
    setMotorSpeed(MOTOR_FL_PWM, MOTOR_FL_IN1, MOTOR_FL_IN2, speedFL);
    setMotorSpeed(MOTOR_FR_PWM, MOTOR_FR_IN1, MOTOR_FR_IN2, speedFR);
    setMotorSpeed(MOTOR_BL_PWM, MOTOR_BL_IN1, MOTOR_BL_IN2, speedBL);
    setMotorSpeed(MOTOR_BR_PWM, MOTOR_BR_IN1, MOTOR_BR_IN2, speedBR);
}

// Fonction pour régler la vitesse et la direction des moteurs
void setMotorSpeed(int pwmPin, int in1Pin, int in2Pin, int speed) {
    bool dir = speed >= 0;
    int pwmValue = abs(speed);

    gpio_set_level(in1Pin, dir ? 1 : 0);
    gpio_set_level(in2Pin, dir ? 0 : 1);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, pwmPin, pwmValue);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, pwmPin);
}

void app_main() {
    // Initialisation des moteurs
    gpio_set_direction(MOTOR_FL_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_FL_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BL_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BL_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_FR_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_FR_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BR_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_BR_IN2, GPIO_MODE_OUTPUT);

    // Configuration du PWM pour les moteurs
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_FL_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ledc_channel);

    // Configuration pour les autres moteurs
    ledc_channel.gpio_num = MOTOR_BL_PWM;
    ledc_channel_config(&ledc_channel);

    ledc_channel.gpio_num = MOTOR_FR_PWM;
    ledc_channel_config(&ledc_channel);

    ledc_channel.gpio_num = MOTOR_BR_PWM;
    ledc_channel_config(&ledc_channel);

    // Initialisation du Bluetooth
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_spp_register_callback(spp_callback);
    esp_spp_init(ESP_SPP_MODE_CB);
}
