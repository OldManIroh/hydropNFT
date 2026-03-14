#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_sens.h"
#include "esp_check.h"
#include "esp_log.h"                      // для ESP_LOGI
#include "touch_sens_example_config.h"

// ------------------------------------------------------------
// Пользовательские настройки
// ------------------------------------------------------------
#define EXAMPLE_TOUCH_CHANNEL_NUM           1          // используем один канал
#define EXAMPLE_TOUCH_BASELINE_SAMPLES      10         // сколько измерений усреднять для базовой линии
#define EXAMPLE_TOUCH_SAMPLE_CFG_NUM        TOUCH_SAMPLE_CFG_NUM

// Тег для логирования
static const char *TAG = "WATER_LEVEL";

// Коэффициент для автоматического расчёта порога
static float s_thresh2bm_ratio[EXAMPLE_TOUCH_CHANNEL_NUM] = { 0.015f };

// Идентификатор канала (TOUCH_MIN_CHAN_ID обычно 0 → GPIO4)
static int s_channel_id[EXAMPLE_TOUCH_CHANNEL_NUM] = {
    TOUCH_MIN_CHAN_ID,
};

// ------------------------------------------------------------
// Функция начального сканирования (калибровка порогов)
// ------------------------------------------------------------
static void example_touch_do_initial_scanning(touch_sensor_handle_t sens_handle,
                                              touch_channel_handle_t chan_handle[])
{
    ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));

    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(touch_sensor_trigger_oneshot_scanning(sens_handle, 2000));
    }

    ESP_ERROR_CHECK(touch_sensor_disable(sens_handle));

    ESP_LOGI(TAG, "Начальные эталоны и новые пороги:");
    for (int i = 0; i < EXAMPLE_TOUCH_CHANNEL_NUM; i++) {
        uint32_t benchmark[EXAMPLE_TOUCH_SAMPLE_CFG_NUM] = {};
#if SOC_TOUCH_SUPPORT_BENCHMARK
        ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_BENCHMARK, benchmark));
#else
        ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[i], TOUCH_CHAN_DATA_TYPE_SMOOTH, benchmark));
#endif
        touch_channel_config_t chan_cfg = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
        for (int j = 0; j < EXAMPLE_TOUCH_SAMPLE_CFG_NUM; j++) {
#if SOC_TOUCH_SENSOR_VERSION == 1
            chan_cfg.abs_active_thresh[j] = (uint32_t)(benchmark[j] * (1 - s_thresh2bm_ratio[i]));
            ESP_LOGI(TAG, "  выборка %d: эталон=%"PRIu32", порог=%"PRIu32, j, benchmark[j], chan_cfg.abs_active_thresh[j]);
#else
            chan_cfg.active_thresh[j] = (uint32_t)(benchmark[j] * s_thresh2bm_ratio[i]);
#endif
        }
        ESP_ERROR_CHECK(touch_sensor_reconfig_channel(chan_handle[i], &chan_cfg));
    }
}

// ------------------------------------------------------------
// Главная функция
// ------------------------------------------------------------
void app_main(void)
{
    touch_sensor_handle_t sens_handle = NULL;
    touch_channel_handle_t chan_handle[EXAMPLE_TOUCH_CHANNEL_NUM];

    // 1. Создаём контроллер сенсора
    touch_sensor_sample_config_t sample_cfg[EXAMPLE_TOUCH_SAMPLE_CFG_NUM] = EXAMPLE_TOUCH_SAMPLE_CFG_DEFAULT();
    touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(EXAMPLE_TOUCH_SAMPLE_CFG_NUM, sample_cfg);
    ESP_ERROR_CHECK(touch_sensor_new_controller(&sens_cfg, &sens_handle));

    // 2. Создаём канал касания
    touch_channel_config_t chan_cfg = EXAMPLE_TOUCH_CHAN_CFG_DEFAULT();
    for (int i = 0; i < EXAMPLE_TOUCH_CHANNEL_NUM; i++) {
        ESP_ERROR_CHECK(touch_sensor_new_channel(sens_handle, s_channel_id[i], &chan_cfg, &chan_handle[i]));

        touch_chan_info_t chan_info = {};
        ESP_ERROR_CHECK(touch_sensor_get_channel_info(chan_handle[i], &chan_info));
        ESP_LOGI(TAG, "Канал касания %d включён на GPIO%d", s_channel_id[i], chan_info.chan_gpio);
    }

    // 3. Настраиваем фильтр
    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    ESP_ERROR_CHECK(touch_sensor_config_filter(sens_handle, &filter_cfg));

    // 4. Выполняем начальное сканирование
    example_touch_do_initial_scanning(sens_handle, chan_handle);

    // 5. Включаем сенсор и запускаем непрерывное сканирование
    ESP_ERROR_CHECK(touch_sensor_enable(sens_handle));
    ESP_ERROR_CHECK(touch_sensor_start_continuous_scanning(sens_handle));

    // 6. Собираем базовую линию (значение без воды)
    uint32_t smooth_data[EXAMPLE_TOUCH_SAMPLE_CFG_NUM] = {};
    uint64_t baseline_sum = 0;

    ESP_LOGI(TAG, "Сбор базовой линии (без воды)...");
    for (int i = 0; i < EXAMPLE_TOUCH_BASELINE_SAMPLES; i++) {
        ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[0], TOUCH_CHAN_DATA_TYPE_SMOOTH, smooth_data));
        baseline_sum += smooth_data[0];
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    uint32_t baseline = baseline_sum / EXAMPLE_TOUCH_BASELINE_SAMPLES;
    ESP_LOGI(TAG, "Базовая линия = %"PRIu32, baseline);

    // 7. Основной цикл измерения
    while (1) {
        ESP_ERROR_CHECK(touch_channel_read_data(chan_handle[0], TOUCH_CHAN_DATA_TYPE_SMOOTH, smooth_data));
        int32_t delta = (int32_t)smooth_data[0] - (int32_t)baseline;
        ESP_LOGI(TAG, "сглаж = %"PRIu32", дельта = %ld", smooth_data[0], delta);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}