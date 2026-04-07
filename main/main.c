/**
 * @file main.c
 * @brief Главный файл приложения HydroNFT
 * 
 * Точка входа приложения. Инициализирует компоненты и запускает задачи FreeRTOS.
 * 
 * @author HydroNFT Team
 * @version 2.0
 * @date 2026
 */

#include <stdio.h>
#include <stdatomic.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "dht.h"
#include "ads1115.h"
#include "device_control.h"
#include "ota_client.h"
#include "hydro_mqtt_client.h"
#include "sntp_client.h"
#include "log_forwarder.h"
#include "espnow_receiver.h"
#include "main.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM 1  ///< Ядро 1 (APP_CPU) для задач
#endif

#ifndef PRO_CPU_NUM
#define PRO_CPU_NUM 0  ///< Ядро 0 (PRO_CPU) для задач
#endif

/// Тег для системы логирования ESP-IDF
static const char *TAG = "main";

/// @brief GPIO пин для датчика DHT (настраивается через menuconfig)
#define DHT_PIN CONFIG_EXAMPLE_DATA_GPIO

/// @brief Тип датчика DHT
#if defined(CONFIG_EXAMPLE_TYPE_DHT11)
#define DHT_TYPE DHT_TYPE_DHT11
#elif defined(CONFIG_EXAMPLE_TYPE_AM2301)
#define DHT_TYPE DHT_TYPE_AM2301
#elif defined(CONFIG_EXAMPLE_TYPE_SI7021)
#define DHT_TYPE DHT_TYPE_SI7021
#else
#define DHT_TYPE DHT_TYPE_DHT11
#endif

/// Час включения света по расписанию (настраивается через menuconfig)
#define LIGHT_ON_HOUR CONFIG_LIGHT_ON_HOUR

/// Время слива воды через клапан (5 минут) — используется esp_timer для монотонности
#define VALVE_DRAIN_DURATION_US (5LL * 60 * 1000000)  // 5 мин в микросекундах

// ============================================================================
// EVENT GROUP для мгновенной реакции задач на изменения режимов
// ============================================================================

/// EventGroup для синхронизации задач расписания
static EventGroupHandle_t s_schedule_event = NULL;

// Биты EventGroup определены в device_control.h — используются напрямую

// ============================================================================
// ЗАДАЧА ЧТЕНИЯ DHT ДАТЧИКА
// ============================================================================

/**
 * @brief Задача для чтения температуры и влажности с датчика DHT
 * 
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 1 (APP_CPU)
 * @note Приоритет: 3
 * @note Размер стека: 4096 байт
 * @note НЕ добавляем в watchdog — dht_read_float_data() может блокироваться
 */
void dht_task(void *pvParameters)
{
    float temperature, humidity;

#ifdef CONFIG_EXAMPLE_INTERNAL_PULLUP
    device_control_set_dht_pullup(true);
    ESP_LOGI(TAG, "DHT: Внутренний pull-up резистор включён");
#else
    device_control_set_dht_pullup(false);
    ESP_LOGI(TAG, "DHT: Внутренний pull-up резистор выключен (требуется внешний 10kΩ)");
#endif

    ESP_LOGI(TAG, "DHT датчик инициализирован (GPIO %d)", DHT_PIN);
    
    ESP_LOGI(TAG, "DHT: Ожидание прогрева датчика (2 сек)...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        esp_err_t res = dht_read_float_data(DHT_TYPE, DHT_PIN, &humidity, &temperature);

        if (res == ESP_OK) {
            device_control_set_dht_data(temperature, humidity, true);
            ESP_LOGI(TAG, "DHT: Влажность=%.1f%% Температура=%.1f°C", humidity, temperature);
        } else {
            // Сбрасываем valid чтобы не публиковать устаревшие данные
            device_control_set_dht_data(0.0f, 0.0f, false);
            ESP_LOGD(TAG, "Ошибка чтения DHT датчика: %d", res);
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// ============================================================================
// ЗАДАЧА ЧТЕНИЯ ADS1115
// ============================================================================

/**
 * @brief Задача для чтения измерений с ADS1115
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 1 (APP_CPU)
 * @note Приоритет: 4
 * @note Размер стека: 4096 байт
 */
void ads1115_task(void *pvParameters)
{
    esp_err_t res = ads1115_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось инициализировать ADS1115: %d", res);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "ADS1115 успешно инициализирован");

    ads1115_measurement_t measurements[4];

    while (1) {
        if (device_control_ads1115_is_stop_requested()) {
            ESP_LOGI(TAG, "ADS1115: остановлен для OTA, ожидание...");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        res = ads1115_measure_all_channels(measurements);

        if (res == ESP_OK) {
            for (int i = 0; i < 4; i++) {
                if (measurements[i].error == ESP_OK) {
                    ESP_LOGI(TAG, "Канал %u: напряжение = %.04f В",
                           measurements[i].channel,
                           measurements[i].voltage);
                } else {
                    ESP_LOGE(TAG, "Канал %u: ошибка = %d",
                           measurements[i].channel,
                           measurements[i].error);
                }
            }
        } else {
            ESP_LOGE(TAG, "Ошибка при измерениях: %d", res);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// ЗАДАЧА ПУБЛИКАЦИИ ДАННЫХ СЕНСОРОВ В MQTT
// ============================================================================

/**
 * @brief Задача для публикации данных сенсоров в MQTT
 * 
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU)
 * @note Приоритет: 4
 * @note Размер стека: 3072 байт
 */
void mqtt_sensor_task(void *pvParameters)
{
    while (1) {
        if (mqtt_client_is_connected()) {
            mqtt_client_publish_sensor_data();
        }
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ============================================================================
// ЗАДАЧА ПРОВЕРКИ ФЛАГА OTA ОБНОВЛЕНИЯ
// ============================================================================

/**
 * @brief Задача для проверки флага OTA обновления из MQTT
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU)
 * @note Приоритет: 4
 * @note Размер стека: 4096 байт
 */
void mqtt_ota_check_task(void *pvParameters)
{
    EventGroupHandle_t event = (EventGroupHandle_t)pvParameters;

    while (1) {
        if (mqtt_client_is_connected() && mqtt_client_is_ota_requested()) {
            ESP_LOGI(TAG, "Получена команда OTA обновления из MQTT...");

            // Сначала сбрасываем флаг, затем запускаем OTA
            mqtt_client_reset_ota_flag();
            start_ota_progress_task();
            ota_start_task();
        }

        xEventGroupWaitBits(event, DEVICE_CTRL_OTA_FLAG_BIT,
                           pdTRUE, pdFALSE,
                           pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// ЗАДАЧА УПРАВЛЕНИЯ СВЕТОМ ПО РАСПИСАНИЮ
// ============================================================================

/**
 * @brief Задача управления освещением по расписанию
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU)
 * @note Приоритет: 3
 * @note Размер стека: 3072 байт
 */
void light_schedule_task(void *pvParameters)
{
    EventGroupHandle_t event = (EventGroupHandle_t)pvParameters;

    ESP_LOGI(TAG, "Задача управления светом запущена (день=%d:00–23:59, ночь=00:00–%d:59)",
             LIGHT_ON_HOUR, LIGHT_ON_HOUR - 1);

    while (1) {
        if (!sntp_client_is_synced()) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        time_t now;
        time(&now);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);

        int current_hour = timeinfo.tm_hour;

        // Отслеживаем переход MANUAL → AUTO
        static device_mode_t prev_mode = DEVICE_MODE_MANUAL;
        device_mode_t current_mode = device_control_get_mode();

        if (prev_mode == DEVICE_MODE_MANUAL && current_mode == DEVICE_MODE_AUTO) {
            // Переход в AUTO — применяем расписание немедленно
            bool should_be_on = (current_hour >= LIGHT_ON_HOUR);
            device_control_set_light_state(should_be_on);
            ESP_LOGI(TAG, "Переход в AUTO: свет %s", should_be_on ? "ВКЛ (день)" : "ВЫКЛ (ночь)");
        }
        prev_mode = current_mode;

        if (current_mode == DEVICE_MODE_MANUAL) {
            xEventGroupWaitBits(event, DEVICE_CTRL_LIGHT_MODE_BIT,
                               pdTRUE, pdFALSE,
                               pdMS_TO_TICKS(60000));
            continue;
        }

        bool should_be_on = (current_hour >= LIGHT_ON_HOUR);
        bool currently_on = device_control_get_light_state();

        if (should_be_on && !currently_on) {
            ESP_LOGI(TAG, "Расписание: день (%d:00), включаю свет", current_hour);
            device_control_set_light_state(true);
        } else if (!should_be_on && currently_on) {
            ESP_LOGI(TAG, "Расписание: ночь (%d:00), выключаю свет", current_hour);
            device_control_set_light_state(false);
        }

        xEventGroupWaitBits(event, DEVICE_CTRL_LIGHT_MODE_BIT,
                           pdTRUE, pdFALSE,
                           pdMS_TO_TICKS(60000));
    }
}

// ============================================================================
// ЗАДАЧА УПРАВЛЕНИЯ НАСОСОМ И КЛАПАНОМ
// ============================================================================

/**
 * @brief Задача управления насосом и клапаном по циклу слива
 *
 * Автомат состояний:
 *
 * ДЕНЬ (8:00–23:59):
 *   CIRCULATING → DRAINING   : ESP-NOW false→true (верхний уровень)
 *   DRAINING → CIRCULATING   : таймер слива ≥ 5 мин
 *   CIRCULATING → NIGHT_STOP : наступила ночь, насос работает
 *
 * НОЧЬ (00:00–07:59):
 *   NIGHT_STOP → NIGHT_IDLE  : ESP-NOW true (насос остановлен)
 *   NIGHT_STOP → NIGHT_IDLE  : насос уже выключен
 *   DRAINING → NIGHT_IDLE    : наступила ночь во время слива
 *   NIGHT_IDLE → CIRCULATING : наступил день
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU)
 * @note Приоритет: 3
 * @note Размер стека: 3072 байт
 */
void pump_valve_schedule_task(void *pvParameters)
{
    EventGroupHandle_t event = (EventGroupHandle_t)pvParameters;

    typedef enum {
        STATE_CIRCULATING,   ///< День: насос ВКЛ, клапан ЗАКРЫТ, ждём ESP-NOW true
        STATE_DRAINING,      ///< День: насос ВЫКЛ, клапан ОТКРЫТ, таймер 5 мин
        STATE_NIGHT_STOP,    ///< Ночь: насос работал при закате, ждём ESP-NOW true
        STATE_NIGHT_IDLE,    ///< Ночь: насос ВЫКЛ, клапан ЗАКРЫТ, ждём рассвет
    } pump_valve_state_t;

    pump_valve_state_t state = STATE_CIRCULATING;
    int64_t drain_start_us = 0;   // Монотонное время (esp_timer)
    bool last_espnow_state = false;
    bool prev_daytime = false;  ///< Будет инициализировано при первом цикле

    ESP_LOGI(TAG, "Задача управления насосом/клапаном запущена");

    while (1) {
        if (!sntp_client_is_synced()) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        time_t now;
        time(&now);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        int current_hour = timeinfo.tm_hour;
        bool is_daytime = (current_hour >= LIGHT_ON_HOUR);

        bool espnow_high = espnow_receiver_get_touch_state();

        // Инициализируем prev_daytime при первом цикле (после SNTP-синхронизации)
        static bool first_run = true;
        if (first_run) {
            prev_daytime = is_daytime;
            last_espnow_state = espnow_high;  // Инициализируем состояние ESP-NOW
            first_run = false;
            ESP_LOGI(TAG, "Начальное состояние: %s, state=%s",
                     is_daytime ? "ДЕНЬ" : "НОЧЬ",
                     state == STATE_CIRCULATING ? "CIRCULATING" :
                     state == STATE_DRAINING ? "DRAINING" :
                     state == STATE_NIGHT_STOP ? "NIGHT_STOP" : "NIGHT_IDLE");
        }

        // Отслеживаем переход MANUAL → AUTO
        static device_mode_t prev_mode = DEVICE_MODE_MANUAL;
        device_mode_t current_mode = device_control_get_mode();

        if (prev_mode == DEVICE_MODE_MANUAL && current_mode == DEVICE_MODE_AUTO) {
            // Переход в AUTO — проверяем реальный уровень воды
            if (is_daytime) {
                if (espnow_high) {
                    // Верхний уровень достигнут — слив
                    ESP_LOGI(TAG, "Переход в AUTO: верхний уровень, начинаю слив");
                    device_control_set_pump_state(false);
                    device_control_set_valve_state(true);
                    drain_start_us = esp_timer_get_time();
                    state = STATE_DRAINING;
                } else {
                    // Нижний уровень — циркуляция
                    ESP_LOGI(TAG, "Переход в AUTO: нижний уровень, насос ВКЛ");
                    device_control_set_pump_state(true);
                    device_control_set_valve_state(false);
                    state = STATE_CIRCULATING;
                }
            } else {
                // Ночь — всё ВЫКЛ
                device_control_set_pump_state(false);
                device_control_set_valve_state(false);
                state = STATE_NIGHT_IDLE;
                ESP_LOGI(TAG, "Переход в AUTO: ночь, всё ВЫКЛ");
            }
            last_espnow_state = espnow_high;
        }
        prev_mode = current_mode;

        // В режиме MANUAL — пропускаем авто-управление
        if (current_mode == DEVICE_MODE_MANUAL) {
            last_espnow_state = espnow_high;
            prev_daytime = is_daytime;
            xEventGroupWaitBits(event, DEVICE_CTRL_PUMP_MODE_BIT,
                               pdTRUE, pdFALSE,
                               pdMS_TO_TICKS(10000));
            continue;
        }

        // ====================================================================
        // Детекция перехода день↔ночь
        // ====================================================================
        if (is_daytime != prev_daytime) {
            if (is_daytime) {
                // Рассвет: 07:59 → 08:00
                ESP_LOGI(TAG, "Рассвет: переход в дневной режим");
                if (state == STATE_NIGHT_IDLE || state == STATE_NIGHT_STOP) {
                    state = STATE_CIRCULATING;
                    ESP_LOGI(TAG, "День: насос ВКЛ, клапан ЗАКРЫТ (циркуляция)");
                    device_control_set_pump_state(true);
                    device_control_set_valve_state(false);
                }
            } else {
                // Закат: 23:59 → 00:00
                ESP_LOGI(TAG, "Закат: переход в ночной режим");
                if (state == STATE_CIRCULATING) {
                    // Насос работал — нужно дождаться ESP-NOW true
                    if (device_control_get_pump_state()) {
                        state = STATE_NIGHT_STOP;
                        ESP_LOGI(TAG, "Ночь: насос работал, ждём верхний уровень для остановки");
                    } else {
                        state = STATE_NIGHT_IDLE;
                        ESP_LOGI(TAG, "Ночь: насос уже выключен, переходим в ожидание");
                        device_control_set_pump_state(false);
                        device_control_set_valve_state(false);
                    }
                } else if (state == STATE_DRAINING) {
                    // Слив во время заката — закрываем клапан, насос ВЫКЛ
                    ESP_LOGI(TAG, "Ночь во время слива — клапан ЗАКРЫТ, насос ВЫКЛ");
                    device_control_set_valve_state(false);
                    device_control_set_pump_state(false);
                    state = STATE_NIGHT_IDLE;
                } else if (state == STATE_NIGHT_STOP || state == STATE_NIGHT_IDLE) {
                    // Уже в ночном состоянии — гарантируем ВЫКЛ
                    device_control_set_pump_state(false);
                    device_control_set_valve_state(false);
                }
            }
            prev_daytime = is_daytime;
        }

        // ====================================================================
        // Автомат состояний
        // ====================================================================
        switch (state) {
        case STATE_CIRCULATING:
            // День: насос работает, ждём ESP-NOW false→true (фронт)
            if (espnow_high && !last_espnow_state) {
                ESP_LOGI(TAG, "День: верхний уровень — насос ВЫКЛ, клапан ОТКРЫТ на 5 мин");
                device_control_set_pump_state(false);
                device_control_set_valve_state(true);
                drain_start_us = esp_timer_get_time();
                state = STATE_DRAINING;
            }
            break;

        case STATE_DRAINING:
            // День: ждём истечения 5 минут (монотонный таймер)
            if ((esp_timer_get_time() - drain_start_us) >= VALVE_DRAIN_DURATION_US) {
                ESP_LOGI(TAG, "Слив завершён (5 мин) — клапан ЗАКРЫТ, насос ВКЛ");
                device_control_set_valve_state(false);
                device_control_set_pump_state(true);
                state = STATE_CIRCULATING;
            }
            break;

        case STATE_NIGHT_STOP:
            // Ночь: насос работал при закате, ждём ESP-NOW true для остановки
            if (espnow_high) {
                ESP_LOGI(TAG, "Ночь: получен верхний уровень, насос остановлен");
                device_control_set_pump_state(false);
                state = STATE_NIGHT_IDLE;
            }
            break;

        case STATE_NIGHT_IDLE:
            // Ночь: всё выключено, ждём рассвет
            // Переход обрабатывается в блоке детекции день↔ночь
            break;

        default:
            state = STATE_CIRCULATING;
            break;
        }

        last_espnow_state = espnow_high;
        xEventGroupWaitBits(event, DEVICE_CTRL_PUMP_MODE_BIT,
                           pdTRUE, pdFALSE,
                           pdMS_TO_TICKS(10000));
    }
}

// ============================================================================
// ЗАДАЧА ПУБЛИКАЦИИ ПРОГРЕССА OTA В MQTT
// ============================================================================

static TaskHandle_t ota_progress_task_handle = NULL;
static _Atomic bool s_ota_progress_task_active = false;

/**
 * @brief Задача для публикации прогресса OTA обновления в MQTT
 * 
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU)
 * @note Приоритет: 4
 * @note Размер стека: 4096 байт
 */
void mqtt_ota_progress_task(void *pvParameters)
{
    int last_progress = -1;
    int no_change_count = 0;
    int total = 0;
    int downloaded = 0;

    ESP_LOGI(TAG, "Задача публикации прогресса OTA запущена");

    while (1) {
        int progress = ota_get_progress_percent();

        if (progress >= 0) {
            total = ota_get_total_size();
            downloaded = ota_get_downloaded_bytes();

            mqtt_client_publish_ota_progress(progress, downloaded, total);

            ESP_LOGI(TAG, "OTA прогресс: %d%% (%d/%d)", progress, downloaded, total);
            last_progress = progress;
            no_change_count = 0;
        } else if (ota_is_active()) {
            // OTA активна, но загрузка ещё не началась (HTTP подключение)
            // Не считаем это как "без изменений" — ждём реального прогресса
            no_change_count = 0;
        } else {
            no_change_count++;

            if (no_change_count >= 3) {
                ESP_LOGI(TAG, "OTA сессия завершена (прогресс: %d%%)", last_progress);

                if (last_progress == 100) {
                    mqtt_client_publish_ota_status("completed");
                }

                ESP_LOGI(TAG, "Задача публикации прогресса OTA завершается");

                // Сбрасываем флаг и обнуляем handle перед удалением задачи
                atomic_store(&s_ota_progress_task_active, false);
                ota_progress_task_handle = NULL;
                vTaskDelete(NULL);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Запуск задачи публикации прогресса OTA
 *
 * @note Использует атомарный флаг для отслеживания состояния задачи вместо eTaskGetState()
 */
void start_ota_progress_task(void)
{
    if (ota_progress_task_handle != NULL) {
        // Проверяем атомарный флаг вместо eTaskGetState()
        if (!atomic_load(&s_ota_progress_task_active)) {
            ESP_LOGI(TAG, "Задача прогресса OTA завершена, обнуляю handle");
            ota_progress_task_handle = NULL;
        } else {
            ESP_LOGW(TAG, "Задача прогресса OTA ещё активна");
            return;
        }
    }
    TaskHandle_t tmp = NULL;
    if (xTaskCreatePinnedToCore(
        mqtt_ota_progress_task,
        "mqtt_ota_progress_task",
        3076,
        NULL,
        4,
        &tmp,
        PRO_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'mqtt_ota_progress_task'");
        return;
    }
    ota_progress_task_handle = tmp;
    atomic_store(&s_ota_progress_task_active, true);
    ESP_LOGI(TAG, "Задача публикации прогресса OTA создана");
}

// ============================================================================
// ГЛАВНАЯ ФУНКЦИЯ ПРИЛОЖЕНИЯ
// ============================================================================

/**
 * @brief Главная функция приложения - точка входа
 *
 * @note После запуска задач управление передаётся планировщику FreeRTOS
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Запуск приложения HydroNFT");

    // ==========================================================================
    // 1. ИНИЦИАЛИЗАЦИЯ УПРАВЛЕНИЯ ОБОРУДОВАНИЕМ (GPIO, DHT mutex, manual override)
    // ==========================================================================
    
    esp_err_t err = device_control_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Критическая ошибка инициализации device_control: %s", esp_err_to_name(err));
        return;
    }

    // ==========================================================================
    // 2. ПРОВЕРКА СТАТУСА OTA ПРОШИВКИ
    // ==========================================================================
    
    ota_check_and_diagnose();

    // ==========================================================================
    // 3. ИНИЦИАЛИЗАЦИЯ OTA КОМПОНЕНТА (NVS, СЕТЬ)
    // ==========================================================================
    
    ota_init();

    // ==========================================================================
    // 4. ИНИЦИАЛИЗАЦИЯ MQTT КЛИЕНТА
    // ==========================================================================

    mqtt_client_init();

    // ==========================================================================
    // 4.1. ИНИЦИАЛИЗАЦИЯ ПЕРЕСЫЛКИ ЛОГОВ В MQTT
    // ==========================================================================

    log_forwarder_init();

    // ==========================================================================
    // 4.2. ИНИЦИАЛИЗАЦИЯ ESP-NOW ПРИЁМНИКА
    // ==========================================================================

    espnow_receiver_init();

    // ==========================================================================
    // 5. ИНИЦИАЛИЗАЦИЯ SNTP КЛИЕНТА
    // ==========================================================================

    sntp_client_init();

    // ==========================================================================
    // 5.1. СОЗДАНИЕ EVENT GROUP для синхронизации задач
    // ==========================================================================

    s_schedule_event = xEventGroupCreate();
    if (s_schedule_event == NULL) {
        ESP_LOGE(TAG, "Не удалось создать EventGroup");
        return;
    }

    // Передаём EventGroup в device_control для сигнализации задач
    device_control_set_schedule_event(s_schedule_event);

    ESP_LOGI(TAG, "EventGroup создан для синхронизации задач расписания");

    // ==========================================================================
    // 6. ЗАПУСК ЗАДАЧ FREERTOS
    // ==========================================================================

    if (xTaskCreatePinnedToCore(
        dht_task,
        "dht_task",
        4096,
        NULL,
        3,
        NULL,
        APP_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'dht_task'");
    }

    if (xTaskCreatePinnedToCore(
        ads1115_task,
        "ads1115_task",
        4096,
        NULL,
        4,
        NULL,
        APP_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'ads1115_task'");
    }

    if (xTaskCreatePinnedToCore(
        mqtt_sensor_task,
        "mqtt_sensor_task",
        3072,
        NULL,
        4,
        NULL,
        PRO_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'mqtt_sensor_task'");
    }

    if (xTaskCreatePinnedToCore(
        mqtt_ota_check_task,
        "mqtt_ota_check_task",
        2048,
        (void *)s_schedule_event,
        4,
        NULL,
        PRO_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'mqtt_ota_check_task'");
    }

    if (xTaskCreatePinnedToCore(
        light_schedule_task,
        "light_schedule_task",
        3072,
        (void *)s_schedule_event,
        3,
        NULL,
        PRO_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'light_schedule_task'");
    }

    if (xTaskCreatePinnedToCore(
        pump_valve_schedule_task,
        "pump_valve_schedule",
        3072,
        (void *)s_schedule_event,
        3,
        NULL,
        PRO_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'pump_valve_schedule_task'");
    }

    if (xTaskCreatePinnedToCore(
        espnow_receiver_task,
        "espnow_receiver_task",
        4096,
        NULL,
        4,
        NULL,
        PRO_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'espnow_receiver_task'");
    }

    ESP_LOGI(TAG, "Приложение запущено. Ожидание команд MQTT...");
    ESP_LOGI(TAG, "SNTP инициализирован. Время синхронизируется");
    ESP_LOGI(TAG, "Задачи распределены: ADS1115/DHT - ядро 1, MQTT/OTA/SNTP - ядро 0");

    // Удаляем задачу main — она больше не нужна, экономим ~3KB RAM
    vTaskDelete(NULL);
}
