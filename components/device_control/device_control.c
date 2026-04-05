/**
 * @file device_control.c
 * @brief Реализация компонента управления оборудованием HydroNFT
 *
 * @author HydroNFT Team
 * @version 2.0
 * @date 2026
 */

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "device_control.h"

static const char *TAG = "device_control";

// ============================================================================
// СОСТОЯНИЕ КОМПОНЕНТА
// ============================================================================

static bool s_initialized = false;
static SemaphoreHandle_t s_dht_data_mutex = NULL;
static float s_dht_temperature = 0.0f;
static float s_dht_humidity = 0.0f;
static bool s_dht_data_valid = false;

/// Единый режим работы системы (по умолчанию MANUAL — безопасный режим после перезагрузки)
static _Atomic device_mode_t s_mode = DEVICE_MODE_MANUAL;

/// Флаг остановки ADS1115 для OTA
static _Atomic bool s_ads1115_stop_requested = false;

/// Callback для уведомления об изменении состояния GPIO
static device_state_change_cb_t s_state_cb = NULL;

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ
// ============================================================================

esp_err_t device_control_init(void)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Уже инициализирован");
        return ESP_OK;
    }

    // Создаём мьютекс для защиты данных DHT
    s_dht_data_mutex = xSemaphoreCreateMutex();
    if (s_dht_data_mutex == NULL) {
        ESP_LOGE(TAG, "Критическая ошибка: не удалось создать мьютекс DHT");
        return ESP_ERR_NO_MEM;
    }

    // ==========================================================================
    // КОНФИГУРАЦИЯ GPIO
    // ==========================================================================

    // Насос циркуляции воды (GPIO 19)
    gpio_config_t pump_io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << DEVICE_CONTROL_PUMP_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    esp_err_t err = gpio_config(&pump_io_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка конфигурации GPIO насоса: %s", esp_err_to_name(err));
        return err;
    }

    // Фитолампа (GPIO 18)
    gpio_config_t light_io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << DEVICE_CONTROL_LIGHT_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    err = gpio_config(&light_io_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка конфигурации GPIO света: %s", esp_err_to_name(err));
        return err;
    }

    // Электроклапан (GPIO 4)
    gpio_config_t valve_io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << DEVICE_CONTROL_VALVE_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    err = gpio_config(&valve_io_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка конфигурации GPIO клапана: %s", esp_err_to_name(err));
        return err;
    }

    // Устанавливаем GPIO в безопасное состояние (выключено)
    gpio_set_level(DEVICE_CONTROL_PUMP_PIN, 0);
    gpio_set_level(DEVICE_CONTROL_LIGHT_PIN, 0);
    gpio_set_level(DEVICE_CONTROL_VALVE_PIN, 0);

    s_initialized = true;
    ESP_LOGI(TAG, "GPIO инициализированы (насос=%d, свет=%d, клапан=%d)",
             DEVICE_CONTROL_PUMP_PIN, DEVICE_CONTROL_LIGHT_PIN, DEVICE_CONTROL_VALVE_PIN);
    ESP_LOGI(TAG, "Мьютекс DHT создан, все устройства в режиме MANUAL (безопасный режим)");

    return ESP_OK;
}

// ============================================================================
// УПРАВЛЕНИЕ GPIO
// ============================================================================

void device_control_set_pump_state(bool state)
{
    gpio_set_level(DEVICE_CONTROL_PUMP_PIN, state ? 1 : 0);
    ESP_LOGI(TAG, "Насос: %s", state ? "ВКЛ" : "ВЫКЛ");
    if (s_state_cb) s_state_cb("hydro/switch/pump/state", state);
}

void device_control_set_light_state(bool state)
{
    gpio_set_level(DEVICE_CONTROL_LIGHT_PIN, state ? 1 : 0);
    ESP_LOGI(TAG, "Свет: %s", state ? "ВКЛ" : "ВЫКЛ");
    if (s_state_cb) s_state_cb("hydro/switch/light/state", state);
}

void device_control_set_valve_state(bool state)
{
    gpio_set_level(DEVICE_CONTROL_VALVE_PIN, state ? 1 : 0);
    ESP_LOGI(TAG, "Клапан: %s", state ? "ОТКРЫТ" : "ЗАКРЫТ");
    if (s_state_cb) s_state_cb("hydro/switch/valve/state", state);
}

bool device_control_get_pump_state(void)
{
    return gpio_get_level(DEVICE_CONTROL_PUMP_PIN);
}

bool device_control_get_light_state(void)
{
    return gpio_get_level(DEVICE_CONTROL_LIGHT_PIN);
}

bool device_control_get_valve_state(void)
{
    return gpio_get_level(DEVICE_CONTROL_VALVE_PIN);
}

// ============================================================================
// РЕЖИМЫ РАБОТЫ (AUTO / MANUAL)
// ============================================================================

void device_control_set_mode(device_mode_t mode)
{
    atomic_store(&s_mode, mode);
    ESP_LOGI(TAG, "Режим системы: %s", device_control_mode_to_string(mode));
}

device_mode_t device_control_get_mode(void)
{
    return atomic_load(&s_mode);
}

const char *device_control_mode_to_string(device_mode_t mode)
{
    switch (mode) {
    case DEVICE_MODE_MANUAL: return "manual";
    case DEVICE_MODE_AUTO:
    default:                 return "auto";
    }
}

device_mode_t device_control_mode_from_string(const char *str)
{
    if (strcmp(str, "manual") == 0) {
        return DEVICE_MODE_MANUAL;
    }
    return DEVICE_MODE_AUTO;
}

// ============================================================================
// CALLBACK ИЗМЕНЕНИЯ СОСТОЯНИЯ GPIO
// ============================================================================

void device_control_register_state_cb(device_state_change_cb_t cb)
{
    s_state_cb = cb;
}

// ============================================================================
// ДАННЫЕ DHT
// ============================================================================

void device_control_set_dht_data(float temperature, float humidity, bool valid)
{
    if (s_dht_data_mutex != NULL) {
        xSemaphoreTake(s_dht_data_mutex, portMAX_DELAY);
        s_dht_temperature = temperature;
        s_dht_humidity = humidity;
        s_dht_data_valid = valid;
        xSemaphoreGive(s_dht_data_mutex);
    }
}

float device_control_get_dht_temperature(void)
{
    float temp = 0.0f;
    if (s_dht_data_mutex != NULL) {
        xSemaphoreTake(s_dht_data_mutex, portMAX_DELAY);
        temp = s_dht_temperature;
        xSemaphoreGive(s_dht_data_mutex);
    }
    return temp;
}

float device_control_get_dht_humidity(void)
{
    float hum = 0.0f;
    if (s_dht_data_mutex != NULL) {
        xSemaphoreTake(s_dht_data_mutex, portMAX_DELAY);
        hum = s_dht_humidity;
        xSemaphoreGive(s_dht_data_mutex);
    }
    return hum;
}

bool device_control_is_dht_data_valid(void)
{
    bool valid = false;
    if (s_dht_data_mutex != NULL) {
        xSemaphoreTake(s_dht_data_mutex, portMAX_DELAY);
        valid = s_dht_data_valid;
        xSemaphoreGive(s_dht_data_mutex);
    }
    return valid;
}

// ============================================================================
// ОСТАНОВКА ADS1115 ДЛЯ OTA
// ============================================================================

void device_control_ads1115_stop_for_ota(void)
{
    atomic_store(&s_ads1115_stop_requested, true);
    ESP_LOGI(TAG, "ADS1115: запрос на остановку для OTA отправлен");
}

bool device_control_ads1115_is_stop_requested(void)
{
    return atomic_load(&s_ads1115_stop_requested);
}

// ============================================================================
// СИГНАЛИЗАЦИЯ ЗАДАЧ РАСПИСАНИЯ (EventGroup)
// ============================================================================

static EventGroupHandle_t s_schedule_event = NULL;

#define SCHEDULE_MODE_CHANGED_BIT    BIT0
#define SCHEDULE_OTA_FLAG_BIT        BIT1

void device_control_set_schedule_event(void *event)
{
    s_schedule_event = (EventGroupHandle_t)event;
}

void device_control_notify_mode_changed(void)
{
    if (s_schedule_event != NULL) {
        xEventGroupSetBits(s_schedule_event, SCHEDULE_MODE_CHANGED_BIT);
    }
}

void device_control_notify_ota_flag(void)
{
    if (s_schedule_event != NULL) {
        xEventGroupSetBits(s_schedule_event, SCHEDULE_OTA_FLAG_BIT);
    }
}
