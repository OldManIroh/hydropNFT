#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_sens.h"
#include "esp_check.h"
#include "esp_log.h"
#include "touch_sens_example_config.h"
#include "water_level_capacitance.h"

// Тег для логирования
static const char *TAG = "WATER_LEVEL";

// Статические переменные для хранения состояния датчика
static touch_sensor_handle_t s_sens_handle = NULL;
static touch_channel_handle_t s_chan_handle = NULL;
static water_level_config_t s_config = {0};
static bool s_is_sensor_enabled = false;

// Внутренняя функция для начального сканирования (калибровка порогов)
static esp_err_t water_level_do_initial_scanning(void)
{
    if (!s_sens_handle || !s_chan_handle) {
        return ESP_ERR_INVALID_STATE;
    }

    // Включаем датчик для начального сканирования
    ESP_RETURN_ON_ERROR(touch_sensor_enable(s_sens_handle), TAG, "Failed to enable touch sensor");

    // Выполняем несколько сканирований для стабилизации
    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_ERROR(touch_sensor_trigger_oneshot_scanning(s_sens_handle, 2000), 
                           TAG, "Failed to trigger scanning");
    }

    // Отключаем датчик
    ESP_RETURN_ON_ERROR(touch_sensor_disable(s_sens_handle), TAG, "Failed to disable touch sensor");

    // Перенастраиваем пороги на основе эталона
    uint32_t benchmark[TOUCH_SAMPLE_CFG_NUM] = {};
#if SOC_TOUCH_SUPPORT_BENCHMARK
    ESP_RETURN_ON_ERROR(touch_channel_read_data(s_chan_handle, TOUCH_CHAN_DATA_TYPE_BENCHMARK, benchmark), 
                       TAG, "Failed to read benchmark");
#else
    ESP_RETURN_ON_ERROR(touch_channel_read_data(s_chan_handle, TOUCH_CHAN_DATA_TYPE_SMOOTH, benchmark), 
                       TAG, "Failed to read smooth data");
#endif

    touch_channel_config_t chan_cfg = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
    for (int j = 0; j < TOUCH_SAMPLE_CFG_NUM; j++) {
#if SOC_TOUCH_SENSOR_VERSION == 1
        chan_cfg.abs_active_thresh[j] = (uint32_t)(benchmark[j] * (1 - s_config.threshold_ratio));
        ESP_LOGI(TAG, "Выборка %d: эталон=%"PRIu32", порог=%"PRIu32, 
                j, benchmark[j], chan_cfg.abs_active_thresh[j]);
#else
        chan_cfg.active_thresh[j] = (uint32_t)(benchmark[j] * s_config.threshold_ratio);
        ESP_LOGI(TAG, "Выборка %d: эталон=%"PRIu32", порог=%"PRIu32, 
                j, benchmark[j], chan_cfg.active_thresh[j]);
#endif
    }

    return touch_sensor_reconfig_channel(s_chan_handle, &chan_cfg);
}

bool water_level_init(const water_level_config_t *config, water_level_state_t *state)
{
    if (!config || !state) {
        ESP_LOGE(TAG, "Неверные параметры");
        return false;
    }

    // Сохраняем конфигурацию
    memcpy(&s_config, config, sizeof(water_level_config_t));
    
    // Инициализируем состояние
    memset(state, 0, sizeof(water_level_state_t));
    state->is_initialized = false;

    // Создаем контроллер сенсора
    touch_sensor_sample_config_t sample_cfg[TOUCH_SAMPLE_CFG_NUM] = EXAMPLE_TOUCH_SAMPLE_CFG_DEFAULT();
    touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(TOUCH_SAMPLE_CFG_NUM, sample_cfg);
    
    esp_err_t err = touch_sensor_new_controller(&sens_cfg, &s_sens_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось создать контроллер датчика касания: %s", esp_err_to_name(err));
        return false;
    }

    // Создаем канал касания
    touch_channel_config_t chan_cfg = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
    err = touch_sensor_new_channel(s_sens_handle, s_config.touch_channel, &chan_cfg, &s_chan_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось создать канал касания: %s", esp_err_to_name(err));
        touch_sensor_del_controller(s_sens_handle);
        s_sens_handle = NULL;
        return false;
    }

    // Получаем информацию о канале
    touch_chan_info_t chan_info = {};
    err = touch_sensor_get_channel_info(s_chan_handle, &chan_info);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Канал касания %d включен на GPIO%d", 
                s_config.touch_channel, chan_info.chan_gpio);
    }

    // Настраиваем фильтр
    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    err = touch_sensor_config_filter(s_sens_handle, &filter_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось настроить фильтр: %s", esp_err_to_name(err));
        water_level_deinit(state);
        return false;
    }

    // Выполняем начальное сканирование
    err = water_level_do_initial_scanning();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось выполнить начальное сканирование: %s", esp_err_to_name(err));
        water_level_deinit(state);
        return false;
    }

    // Включаем датчик и запускаем непрерывное сканирование
    err = touch_sensor_enable(s_sens_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось включить датчик касания: %s", esp_err_to_name(err));
        water_level_deinit(state);
        return false;
    }

    err = touch_sensor_start_continuous_scanning(s_sens_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось запустить непрерывное сканирование: %s", esp_err_to_name(err));
        water_level_deinit(state);
        return false;
    }

    s_is_sensor_enabled = true;
    state->is_initialized = true;
    ESP_LOGI(TAG, "Датчик уровня воды успешно инициализирован");
    return true;
}

bool water_level_calibrate_baseline(water_level_state_t *state)
{
    if (!state || !state->is_initialized || !s_chan_handle) {
        ESP_LOGE(TAG, "Датчик не инициализирован");
        return false;
    }

    ESP_LOGI(TAG, "Калибровка базовой линии (без воды)...");

    uint32_t smooth_data[TOUCH_SAMPLE_CFG_NUM] = {};
    uint64_t baseline_sum = 0;

    // Собираем несколько измерений для усреднения
    for (uint16_t i = 0; i < s_config.baseline_samples; i++) {
        esp_err_t err = touch_channel_read_data(s_chan_handle, TOUCH_CHAN_DATA_TYPE_SMOOTH, smooth_data);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Не удалось прочитать данные во время калибровки: %s", esp_err_to_name(err));
            return false;
        }
        
        baseline_sum += smooth_data[0];
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    state->baseline = baseline_sum / s_config.baseline_samples;
    ESP_LOGI(TAG, "Baseline calibrated: %"PRIu32, state->baseline);
    return true;
}

bool water_level_read(water_level_state_t *state)
{
    if (!state || !state->is_initialized || !s_chan_handle) {
        ESP_LOGE(TAG, "Датчик не инициализирован");
        return false;
    }

    uint32_t smooth_data[TOUCH_SAMPLE_CFG_NUM] = {};
    esp_err_t err = touch_channel_read_data(s_chan_handle, TOUCH_CHAN_DATA_TYPE_SMOOTH, smooth_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось прочитать данные: %s", esp_err_to_name(err));
        return false;
    }

    state->current_value = smooth_data[0];
    state->delta = (int32_t)smooth_data[0] - (int32_t)state->baseline;
    
    return true;
}

int32_t water_level_get_delta(const water_level_state_t *state)
{
    if (!state) {
        return 0;
    }
    return state->delta;
}

uint32_t water_level_get_value(const water_level_state_t *state)
{
    if (!state) {
        return 0;
    }
    return state->current_value;
}

bool water_level_is_water_detected(const water_level_state_t *state, int32_t threshold)
{
    if (!state) {
        return false;
    }
    return (state->delta > threshold);
}

bool water_level_deinit(water_level_state_t *state)
{
    if (state) {
        state->is_initialized = false;
    }

    if (s_is_sensor_enabled) {
        touch_sensor_stop_continuous_scanning(s_sens_handle);
        touch_sensor_disable(s_sens_handle);
        s_is_sensor_enabled = false;
    }

    if (s_chan_handle) {
        touch_sensor_del_channel(s_chan_handle);
        s_chan_handle = NULL;
    }

    if (s_sens_handle) {
        touch_sensor_del_controller(s_sens_handle);
        s_sens_handle = NULL;
    }

    ESP_LOGI(TAG, "Датчик уровня воды деинициализирован");
    return true;
}
