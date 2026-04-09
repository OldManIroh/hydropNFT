/**
 * @file main.c
 * @brief Главный файл приложения HydroNFT
 *
 * Точка входа приложения. Инициализирует компоненты и запускает задачи FreeRTOS.
 *
 * @par Архитектура задач FreeRTOS
 * | Задача                  | Приоритет | Ядро | Стек | Назначение |
 * |-------------------------|-----------|------|------|------------|
 * | ads1115_task            | 4         | 1    | 4096 | Чтение ADC датчиков (pH, TDS, уровень, темп. воды) |
 * | dht_task                | 3         | 1    | 4096 | Чтение DHT датчика (темп/влажность воздуха) |
 * | light_schedule_task     | 3         | 1    | 3072 | Управление светом по расписанию |
 * | pump_valve_schedule_task| 3         | 1    | 3072 | Управление насосом/клапаном (автомат состояний) |
 * | mqtt_sensor_task        | 4         | 0    | 3072 | Публикация сенсоров в MQTT каждые 5 сек |
 * | mqtt_ota_check_task     | 4         | 0    | 4096 | Проверка флага OTA из MQTT |
 * | mqtt_ota_progress_task  | 4         | 0    | 3072 | Динамическая задача: публикация прогресса OTA |
 * | espnow_receiver_task    | 4         | 0    | 4096 | Приём ESP-NOW сигналов уровня воды |
 * | ota_task                | 5         | 0    | 8192 | Загрузка прошивки (создаётся динамически) |
 * | wifi_reconnect_task     | 2         | 0    | 2048 | Переподключение WiFi после сохранения настроек |
 *
 * @par Распределение по ядрам
 * - **Ядро 0 (PRO_CPU):** MQTT, OTA, ESP-NOW, WiFi reconnect
 * - **Ядро 1 (APP_CPU):** Сенсоры (ADS1115, DHT), расписания (свет, насос/клапан)
 *
 * @par EventGroup биты (синхронизация задач)
 * | Бит                         | Задача-получатель          | Кто устанавливает |
 * |-----------------------------|----------------------------|-------------------|
 * | DEVICE_CTRL_OTA_FLAG_BIT    | mqtt_ota_check_task        | device_control_notify_ota_flag() |
 * | DEVICE_CTRL_LIGHT_MODE_BIT  | light_schedule_task        | device_control_notify_mode_changed() |
 * | DEVICE_CTRL_PUMP_MODE_BIT   | pump_valve_schedule_task   | device_control_notify_mode_changed() |
 *
 * @note Задачи расписаний используют xEventGroupWaitBits с xClearOnExit=TRUE —
 *       каждая задача очищает только свой бит, не мешая другим.
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
#include "esp_wifi.h"
#include "esp_netif_sntp.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_task_wdt.h"
#include "mdns.h"
#include "dht.h"
#include "ads1115.h"
#include "device_control.h"
#include "ota_client.h"
#include "hydro_mqtt_client.h"
#include "sntp_client.h"
#include "log_forwarder.h"
#include "espnow_receiver.h"
#include "web_server.h"
#include "settings.h"
#include "main.h"

#ifndef APP_CPU_NUM
#if CONFIG_FREERTOS_UNICORE
// Одноядерный ESP32-S2/S3 — APP_CPU_NUM не определён в ESP-IDF, используем единственное ядро
#define APP_CPU_NUM 0
#else
// Двухъядерный ESP32 — ядро 1 (APP_CPU) для задач сенсоров, чтобы разгрузить PRO_CPU (MQTT/OTA)
#define APP_CPU_NUM 1
#endif
#endif

#ifndef PRO_CPU_NUM
#if CONFIG_FREERTOS_UNICORE
// Одноядерный — то же ядро что и APP_CPU_NUM
#define PRO_CPU_NUM 0
#else
// Двухъядерный — ядро 0 (PRO_CPU) для MQTT, OTA, расписаний
#define PRO_CPU_NUM 0
#endif
#endif

/// Тег для системы логирования ESP-IDF
static const char *TAG = "main";

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================================

/**
 * @brief Применить настройки WiFi STA из NVS
 * @return ESP_OK при успехе, код ошибки при неудаче
 *
 * Выносит дублирующийся код применения STA-конфигурации.
 * Использует обычные проверки вместо ESP_ERROR_CHECK — при повреждённых NVS
 * устройство продолжит работу в AP-режиме.
 */
static esp_err_t apply_wifi_sta_config(void)
{
    char ssid[SETTINGS_SSID_MAX_LEN] = {0};
    char pass[SETTINGS_PASS_MAX_LEN] = {0};

    esp_err_t ssid_err = settings_get_wifi_ssid(ssid, sizeof(ssid));
    if (ssid_err != ESP_OK || ssid[0] == '\0') {
        return ESP_ERR_NOT_FOUND;  // SSID не найден — STA не настроен
    }

    esp_err_t pass_err = settings_get_wifi_pass(pass, sizeof(pass));
    if (pass_err != ESP_OK || pass[0] == '\0') {
        ESP_LOGW(TAG, "WiFi STA: пароль не найден — возможно открытая сеть");
    }

    wifi_config_t sta_config = {0};
    sta_config.sta.threshold.authmode = pass[0] != '\0' ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;
    strncpy((char *)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid) - 1);
    sta_config.sta.ssid[sizeof(sta_config.sta.ssid) - 1] = '\0';
    strncpy((char *)sta_config.sta.password, pass, sizeof(sta_config.sta.password) - 1);
    sta_config.sta.password[sizeof(sta_config.sta.password) - 1] = '\0';

    esp_err_t cfg_err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
    if (cfg_err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка применения WiFi STA настроек: %s", esp_err_to_name(cfg_err));
        return cfg_err;
    }

    ESP_LOGI(TAG, "WiFi STA: подключение к '%s'...", ssid);
    return ESP_OK;
}

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

/// Время слива воды через клапан (настраивается через menuconfig)
#define VALVE_DRAIN_DURATION_US (CONFIG_VALVE_DRAIN_DURATION_MIN * 60LL * 1000000)

// Проверка корректности часа включения света на этапе компиляции
_Static_assert(LIGHT_ON_HOUR > 0 && LIGHT_ON_HOUR < 24,
    "LIGHT_ON_HOUR должен быть в диапазоне 1-23");

// ============================================================================
// EVENT GROUP для мгновенной реакции задач на изменения режимов
// ============================================================================
/**
 * EventGroup для синхронизации задач расписания.
 *
 * Используется вместо polling с фиксированным интервалом:
 * - При изменении AUTO↔MANUAL задачи просыпаются немедленно (< 10 мс)
 * - Без EventGroup задержка реакции достигала бы 60 сек (период опроса)
 *
 * Биты определены в device_control.h (единый источник истины):
 * - DEVICE_CTRL_OTA_FLAG_BIT    — сигнал mqtt_ota_check_task об OTA команде
 * - DEVICE_CTRL_LIGHT_MODE_BIT  — сигнал light_schedule_task об изменении режима
 * - DEVICE_CTRL_PUMP_MODE_BIT   — сигнал pump_valve_schedule_task об изменении режима
 *
 * Каждая задача вызывает xEventGroupWaitBits с xClearOnExit=pdTRUE —
 * бит очищается только для этой задачи, другие задачи видят свои биты.
 */
static EventGroupHandle_t s_schedule_event = NULL;

// ============================================================================
// ЗАДАЧА ЧТЕНИЯ DHT ДАТЧИКА
// ============================================================================

/**
 * @brief Задача для чтения температуры и влажности с датчика DHT
 *
 * @par Почему НЕ добавлена в TWDT (Task Watchdog Timer):
 * Функция dht_read_float_data() может блокироваться до 1 секунды при
 * неисправном датчике или проблемах с таймингами 1-Wire. Если задача
 * добавлена в TWDT, она гарантированно вызовет перезагрузку устройства
 * при первой же ошибке чтения. Вместо этого:
 * - При ошибке данные помечаются invalid (valid=false)
 * - MQTT публикация пропускает невалидные данные
 * - Следующее чтение через 3 секунды может восстановиться
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 1 (APP_CPU)
 * @note Приоритет: 3 — ниже чем ADS1115 (4), чтобы не задерживать ADC измерения
 * @note Размер стека: 4096 байт
 * @note Интервал чтения: 3 секунды (оптимально для DHT — минимальный интервал 2 сек)
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
    // DHT требует стабилизации питания после включения — без задержки первое чтение = ошибка.
    // Спецификация: минимум 1 сек после подачи питания перед первым запросом данных.
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
 * @par Почему инициализация из задачи а не из app_main():
 * ADS1115 использует I2C шину которая может быть ещё не готова к моменту
 * запуска app_main(). Инициализация из задачи позволяет корректно
 * обработать ошибку и удалить задачу без последствий для остальной системы.
 *
 * @par Управление жизненным циклом:
 * При получении команды OTA задача вызвает vTaskDelete(NULL) после
 * сброса g_ads1115_task_active. Это позволяет ads1115_deinit() безопасно
 * удалить мьютексы (ожидая 100 мс после сброса флага).
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 1 (APP_CPU) — отдельно от MQTT/OTA чтобы
 *       блокирующие I2C вызовы (~500 мс на 4 канала) не задерживали сетевой стек
 * @note Приоритет: 4 — выше чем расписания (3), чтобы данные сенсоров были свежими
 * @note Размер стека: 4096 байт
 * @note Интервал чтения: 1 секунда
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

    ads1115_measurement_t measurements[ADS1115_NUM_CHANNELS];

    while (1) {
        if (device_control_ads1115_is_stop_requested()) {
            ESP_LOGI(TAG, "ADS1115: остановлен для OTA, удаляю задачу");
            atomic_store(&g_ads1115_task_active, false);
            vTaskDelete(NULL);
        }

        res = ads1115_measure_all_channels(measurements);

        if (res == ESP_OK) {
            for (int i = 0; i < ADS1115_NUM_CHANNELS; i++) {
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
    // Добавляем задачу в TWDT — если задача зависнет, watchdog перезагрузит устройство
    // Без этого зависшая задача публикации сенсоров не будет обнаружена
    esp_task_wdt_add(NULL);
    while (1) {
        esp_task_wdt_reset();  // Сброс watchdog — задача жива
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
    int retry_count = 0;
    const int MAX_OTA_RETRIES = 3;

    while (1) {
        if (mqtt_client_is_connected() && mqtt_client_is_ota_requested()) {
            ESP_LOGI(TAG, "Получена команда OTA обновления из MQTT (попытка %d/%d)...",
                     retry_count + 1, MAX_OTA_RETRIES);

            // Сначала запускаем OTA, потом сбрасываем флаг — избегаем race condition
            esp_err_t res = ota_start_task();
            if (res == ESP_OK) {
                mqtt_client_reset_ota_flag();
                start_ota_progress_task();
                retry_count = 0;  // Сброс счётчика при успехе
            } else {
                ESP_LOGE(TAG, "Не удалось запустить OTA задачу: %s", esp_err_to_name(res));
                retry_count++;
                if (retry_count >= MAX_OTA_RETRIES) {
                    ESP_LOGE(TAG, "Превышено максимальное число попыток OTA (%d), сбрасываю флаг",
                             MAX_OTA_RETRIES);
                    mqtt_client_reset_ota_flag();
                    retry_count = 0;
                }
            }
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
 * @note Задача работает на ядре 1 (APP_CPU) — изолирована от OTA/MQTT на ядре 0.
 *       Зависание расписания на ядре 0 во время OTA могло бы задержать
 *       переключение света. Перенос на ядро 1 гарантирует своевременную реакцию.
 * @note Приоритет: 3
 * @note Размер стека: 3072 байт
 */
void light_schedule_task(void *pvParameters)
{
    EventGroupHandle_t event = (EventGroupHandle_t)pvParameters;
    int night_end = (LIGHT_ON_HOUR > 0) ? LIGHT_ON_HOUR - 1 : 23;

    // TWDT — задача управляет светом по расписанию, зависание = свет не переключится
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "Задача управления светом запущена (день=%d:00–23:59, ночь=00:00–%02d:59)",
             LIGHT_ON_HOUR, night_end);

    while (1) {
        esp_task_wdt_reset();  // Сброс watchdog
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
 * @note Задача работает на ядре 1 (APP_CPU) — изолирована от OTA/MQTT на ядре 0.
 *       Это критично для безопасности оборудования: если OTA займёт ядро 0,
 *       насос и клапан продолжат работать по расписанию без задержек.
 * @note Приоритет: 3
 * @note Размер стека: 3072 байт
 */
void pump_valve_schedule_task(void *pvParameters)
{
    EventGroupHandle_t event = (EventGroupHandle_t)pvParameters;

    // TWDT — задача управляет насосом/клапаном, зависание = затопление или пересыхание
    esp_task_wdt_add(NULL);

    typedef enum {
        STATE_CIRCULATING,   ///< День: насос ВКЛ, клапан ЗАКРЫТ, ждём ESP-NOW true
        STATE_DRAINING,      ///< День: насос ВЫКЛ, клапан ОТКРЫТ, таймер 5 мин
        STATE_NIGHT_STOP,    ///< Ночь: насос работал при закате, ждём ESP-NOW true
        STATE_NIGHT_IDLE,    ///< Ночь: насос ВЫКЛ, клапан ЗАКРЫТ, ждём рассвет
    } pump_valve_state_t;

    pump_valve_state_t state = STATE_CIRCULATING;
    // Монотонное время (esp_timer) — НЕ зависит от SNTP синхронизации.
    // esp_timer_get_time() считает микросекунды с момента boot, всегда monotonic.
    // xTaskGetTickCount() тоже монотонный, но с меньшим разрешением (1 тик = 1-10 мс).
    // Для 5-минутного таймера слива esp_timer даёт точность до микросекунд и не
    // подвержен wrap-around (int64_t = 292 года).
    int64_t drain_start_us = 0;
    bool last_espnow_state = false;
    // prev_daytime инициализируется при первом цикле (после SNTP-синхронизации)
    // чтобы избежать ложного детектирования перехода день→ночь при загрузке.
    bool prev_daytime = false;

    ESP_LOGI(TAG, "Задача управления насосом/клапаном запущена");

    while (1) {
        esp_task_wdt_reset();
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

        // Инициализируем prev_daytime при первом цикле — ТОЛЬКО после SNTP-синхронизации.
        // Если инициализировать до SNTP, current_hour будет 0 (1970-01-01) и prev_daytime
        // установится в false (ночь). При последующей синхронизации SNTP время может
        // прыгнуть на день → детекция перехода ночь→день → ложное включение насоса.
        // first_run = true гарантирует что первое определение day/night происходит
        // когда системное время уже корректно.
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
    // no_change_count считает циклы без прогресса загрузки.
    // Значение 3 = 3 секунды (цикл 1 сек) — достаточно чтобы HTTP успел
    // установить соединение и начать передачу данных. Если за 3 сек прогресса нет —
    // OTA сессия завершена (ошибка или загрузка уже закончилась).
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
 * Использует atomic_compare_exchange_strong для предотвращения race condition:
 * Если два потока (например, MQTT handler и веб-сервер) одновременно вызовут
 * эту функцию, только один CAS преуспеет (expected=false → true).
 * Второй получит expected=true и вернёт управление без создания задачи.
 *
 * Без CAS возможны две ситуации:
 * 1. Оба потока видят handle==NULL → обе создают задачу → дублирование OTA progress
 * 2. Оба потока проверяют флаг → оба создают задачу → memory leak старого handle
 *
 * @note Вызывается после ota_start_task() при успешном создании OTA задачи
 */
void start_ota_progress_task(void)
{
    // Атомарная попытка захватить право на создание задачи — предотвращает race condition
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_ota_progress_task_active, &expected, true)) {
        // Уже активна — задача либо создаётся, либо уже работает
        if (ota_progress_task_handle != NULL) {
            ESP_LOGW(TAG, "Задача прогресса OTA ещё активна");
        }
        return;
    }

    TaskHandle_t tmp = NULL;
    if (xTaskCreatePinnedToCore(
        mqtt_ota_progress_task,
        "mqtt_ota_progress_task",
        3072,
        NULL,
        4,
        &tmp,
        PRO_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'mqtt_ota_progress_task'");
        atomic_store(&s_ota_progress_task_active, false);  // Откат флага
        return;
    }
    ota_progress_task_handle = tmp;
    ESP_LOGI(TAG, "Задача публикации прогресса OTA создана");
}

// ============================================================================
// ЗАДАЧА ПЕРЕПОДКЛЮЧЕНИЯ WIFI (после сохранения настроек из веб-интерфейса)
// ============================================================================

/**
 * @brief Задача переподключения WiFi после сохранения настроек
 *
 * @par Почему приоритет 2:
 * Приоритет ниже чем у задач расписаний (3) и сенсоров (4) — переподключение
 * WiFi не критично по времени и не должно вытеснять управление оборудованием.
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU)
 * @note Приоритет: 2 — ниже всех остальных задач, WiFi reconnect не критичен
 * @note Размер стека: 2048 байт — задача只做 esp_wifi_disconnect/connect
 * @note Интервал опроса флага: 2 секунды
 */
void wifi_reconnect_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Задача переподключения WiFi запущена");

    // Счётчик ошибок — предотвращает бесконечный цикл при повреждённых NVS.
    // Без этого задача будет бесконечно пытаться переподключиться,
    // флудя логи и расходуя CPU/радио.
    int error_count = 0;
    const int MAX_RECONNECT_ERRORS = 3;

    while (1) {
        if (web_server_wifi_reconnect_requested()) {
            ESP_LOGI(TAG, "Запрошено переподключение WiFi...");

            // Применяем настройки через общую функцию
            esp_err_t res = apply_wifi_sta_config();
            if (res != ESP_OK) {
                error_count++;
                if (error_count >= MAX_RECONNECT_ERRORS) {
                    ESP_LOGE(TAG, "Превышен лимит ошибок WiFi reconnect (%d) — отмена. "
                             "Проверьте настройки WiFi в веб-интерфейсе.", MAX_RECONNECT_ERRORS);
                    error_count = 0;  // Сброс — пользователь может попробовать снова
                    continue;
                }
                ESP_LOGW(TAG, "WiFi SSID не найден или ошибка настроек (ошибка %d/%d)",
                         error_count, MAX_RECONNECT_ERRORS);
                // Восстанавливаем флаг — задача попробует снова через 2 сек
                web_server_set_wifi_reconnect_requested(true);
                continue;
            }

            // Успешное применение настроек — сброс счётчика ошибок
            error_count = 0;

            // Отключаем STA интерфейс - AP остаётся доступен
            // В ESP-IDF 6.0 esp_wifi_disconnect() без аргументов влияет только на STA
            esp_err_t dis_err = esp_wifi_disconnect();
            if (dis_err != ESP_OK) {
                ESP_LOGE(TAG, "Ошибка отключения WiFi STA: %s", esp_err_to_name(dis_err));
            }
            vTaskDelay(pdMS_TO_TICKS(1000));

            esp_err_t conn_err = esp_wifi_connect();
            if (conn_err != ESP_OK) {
                error_count++;
                if (error_count >= MAX_RECONNECT_ERRORS) {
                    ESP_LOGE(TAG, "Превышен лимит ошибок WiFi reconnect (%d) — отмена. "
                             "Проверьте SSID/пароль в веб-интерфейсе.", MAX_RECONNECT_ERRORS);
                    error_count = 0;
                } else {
                    ESP_LOGE(TAG, "Ошибка подключения WiFi STA: %s (ошибка %d/%d)",
                             esp_err_to_name(conn_err), error_count, MAX_RECONNECT_ERRORS);
                }
            } else {
                // Успешный запрос на подключение — сброс счётчика
                error_count = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ============================================================================
// ГЛАВНАЯ ФУНКЦИЯ ПРИЛОЖЕНИЯ
// ============================================================================

/**
 * @brief Главная функция приложения - точка входа
 *
 * @note После запуска задач управление передаётся планировщику FreeRTOS
 */

// ============================================================================
// WiFi EVENT HANDLER — автоматическое переподключение STA
// ============================================================================

/// Счётчик попыток reconnect (для exponential backoff)
static uint32_t g_wifi_reconnect_attempts = 0;

/// Таймер для отложенного reconnect (exponential backoff)
static esp_timer_handle_t s_reconnect_timer = NULL;

/**
 * @brief Callback таймера reconnect — вызывает esp_wifi_connect()
 *
 * Вызывается из контекста esp_timer task после истечения backoff задержки.
 * esp_wifi_connect() асинхронный — не блокирует выполнение.
 */
static void wifi_reconnect_timer_cb(void *arg)
{
    ESP_LOGW(TAG, "WiFi STA: попытка переподключения #%lu", (unsigned long)(g_wifi_reconnect_attempts));
    esp_wifi_connect();
}

/**
 * @brief Обработчик события отключения WiFi STA от роутера
 *
 * Вызывается автоматически при потере связи с AP (перезагрузка роутера,
 * временная помеха, выход из зоны покрытия). Без этого обработчика
 * устройство НЕ переподключится до ручного запроса через веб-интерфейс.
 *
 * @par Exponential backoff через esp_timer
 * Задержка: 1s → 2s → 4s → 8s → 16s → 30s (макс). Реализована через
 * one-shot esp_timer вместо блокировки в event handler:
 * - Event handler НЕ должен блокироваться (контекст системной задачи)
 * - Без esp_timer: при return без esp_wifi_connect() новый DISCONNECTED
 *   event НЕ будет сгенерирован → устройство никогда не переподключится
 * - С esp_timer: timer callback вызывает esp_wifi_connect() после задержки,
 *   при неудаче генерируется новый DISCONNECTED → цикл продолжается
 *
 * @note Регистрация обработчика происходит ДО цикла ожидания подключения
 */
static void wifi_reconnect_handler(void *arg, esp_event_base_t event_base,
                                    int32_t event_id, void *event_data)
{
    g_wifi_reconnect_attempts++;

    // Рассчитываем задержку: 2^(attempts-1), макс 30 сек
    uint32_t delay_sec = (1U << (g_wifi_reconnect_attempts - 1));
    if (delay_sec > 30) delay_sec = 30;

    ESP_LOGW(TAG, "WiFi STA отключён — переподключение через %lu сек (попытка %lu)",
             (unsigned long)delay_sec, (unsigned long)g_wifi_reconnect_attempts);

    // Создаём таймер при первом использовании (lazy init)
    if (s_reconnect_timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = wifi_reconnect_timer_cb,
            .name = "wifi_reconn",
        };
        esp_err_t err = esp_timer_create(&timer_args, &s_reconnect_timer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка создания таймера reconnect: %s — попытка без задержки",
                     esp_err_to_name(err));
            esp_wifi_connect();  // fallback: немедленная попытка
            return;
        }
    }

    // Останавливаем предыдущий таймер (если ещё активен) и запускаем новый
    esp_timer_stop(s_reconnect_timer);  // Игнорируем ошибку (может быть уже остановлен)
    esp_err_t err = esp_timer_start_once(s_reconnect_timer, (uint64_t)delay_sec * 1000000ULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка запуска таймера reconnect: %s — попытка без задержки",
                 esp_err_to_name(err));
        esp_wifi_connect();  // fallback
    }
}

/**
 * @brief Обработчик события получения IP адреса STA интерфейсом
 *
 * Сбрасывает счётчик попыток reconnect — соединение восстановлено.
 */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi STA получил IP: " IPSTR, IP2STR(&event->ip_info.ip));
        // Сброс счётчика reconnect — соединение успешно восстановлено
        g_wifi_reconnect_attempts = 0;
    }
}

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
    // 3.1. ИНИЦИАЛИЗАЦИЯ НАСТРОЕК (NVS)
    // ==========================================================================

    settings_init();

    // ==========================================================================
    // 3.2. ИНИЦИАЛИЗАЦИЯ WIFI В РЕЖИМЕ APSTA (с нуля)
    // ==========================================================================
    // APSTA = одновременно точка доступа (AP) + клиент (STA)
    // AP всегда доступен для настроек, STA подключается к роутеру

    // err уже объявлена выше

    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка esp_netif_init: %s — продолжаю", esp_err_to_name(err));
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка esp_event_loop_create_default: %s — продолжаю", esp_err_to_name(err));
    }

    // Создаём оба интерфейса
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    // Получаем MAC для уникального имени AP
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char ap_ssid[32];
    snprintf(ap_ssid, sizeof(ap_ssid), "HydroNFT_%02X%02X", mac[4], mac[5]);

    // Настраиваем AP интерфейс
    // В APSTA режиме ESP32 использует один радиомодуль — AP и STA работают на одном канале.
    // При подключении STA к роутеру, AP автоматически переключается на канал роутера.
    // channel=0 означает "авто" — канал определяется при подключении STA.
    // Если задать фиксированный канал (например, 1), а роутер на канале 6 —
    // ESP-IDF всё равно переключит AP на канал роутера, но в логах будет warning.
    wifi_config_t ap_config = {0};
    ap_config.ap.channel = 0;
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    memcpy(ap_config.ap.ssid, ap_ssid, strlen(ap_ssid));
    ap_config.ap.ssid_len = strlen(ap_ssid);
#ifdef CONFIG_AP_WIFI_PASSWORD
    strncpy((char *)ap_config.ap.password, CONFIG_AP_WIFI_PASSWORD, sizeof(ap_config.ap.password) - 1);
#else
    strncpy((char *)ap_config.ap.password, "hydronft12345678", sizeof(ap_config.ap.password) - 1);
#endif
    ap_config.ap.password[sizeof(ap_config.ap.password) - 1] = '\0';

    // Инициализируем WiFi и переключаем в APSTA
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Критическая ошибка esp_wifi_init: %s", esp_err_to_name(err));
        return;
    }
    err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка esp_wifi_set_mode(APSTA): %s — продолжаю", esp_err_to_name(err));
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка esp_wifi_set_config(AP): %s — продолжаю", esp_err_to_name(err));
    }

    // Пробуем подключиться к сохранённому WiFi (STA)
    {
        char ssid[SETTINGS_SSID_MAX_LEN] = {0};
        char pass[SETTINGS_PASS_MAX_LEN] = {0};
        if (settings_get_wifi_ssid(ssid, sizeof(ssid)) == ESP_OK && ssid[0] != '\0') {
            esp_err_t pass_err = settings_get_wifi_pass(pass, sizeof(pass));
            if (pass_err != ESP_OK || pass[0] == '\0') {
                ESP_LOGW(TAG, "WiFi STA: пароль не найден — возможно открытая сеть");
            }
            wifi_config_t sta_config = {0};
            sta_config.sta.threshold.authmode = pass[0] != '\0' ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;
            strncpy((char *)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid) - 1);
            sta_config.sta.ssid[sizeof(sta_config.sta.ssid) - 1] = '\0';
            strncpy((char *)sta_config.sta.password, pass, sizeof(sta_config.sta.password) - 1);
            sta_config.sta.password[sizeof(sta_config.sta.password) - 1] = '\0';

            // Обычная проверка вместо ESP_ERROR_CHECK — при повреждённых NVS-данных
            // WiFi STA не подключится, но устройство продолжит работу в AP-режиме
            // (веб-интерфейс настроек остаётся доступен по адресу AP)
            esp_err_t cfg_err = esp_wifi_set_config(WIFI_IF_STA, &sta_config);
            if (cfg_err != ESP_OK) {
                ESP_LOGE(TAG, "Ошибка применения WiFi STA настроек: %s — продолжаю в AP режиме",
                         esp_err_to_name(cfg_err));
            } else {
                ESP_LOGI(TAG, "WiFi STA: подключение к '%s'...", ssid);
            }
        } else {
            ESP_LOGI(TAG, "WiFi STA: настройки не найдены, только AP режим");
        }
    }

    // Отключаем энергосбережение WiFi для максимальной производительности OTA.
    // WIFI_PS_NONE — радиомодуль всегда активен, без задержек на пробуждение.
    // Это критично для OTA: при WIFI_PS_MIN_MODEM загрузка может прерываться
    // на периоды сна, что приводит к таймаутам HTTP клиента.
    // Потребление увеличивается на ~15-30 мА, но для стационарного устройства это приемлемо.
    err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Ошибка esp_wifi_set_ps: %s — продолжаю", esp_err_to_name(err));
    }

    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка esp_wifi_start: %s — продолжаю в fallback", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "WiFi APSTA запущен: AP='%s' (WPA2)", ap_ssid);
    }

    // Обработчик отключения STA — автоматическое переподключение
    // Без этого обработчика при перезагрузке роутера или временной потере сигнала
    // WiFi STA НЕ переподключится самостоятельно (только через ручной запрос из веб-интерфейса)
    err = esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
        wifi_reconnect_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Ошибка регистрации WiFi reconnect handler: %s", esp_err_to_name(err));
    }

    // Обработчик получения IP — сброс счётчика reconnect backoff
    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
        ip_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Ошибка регистрации IP_EVENT handler: %s", esp_err_to_name(err));
    }

    // Ждём подключения STA к роутеру (до 15 сек) — избегаем заведомо провальных попыток MQTT/SNTP.
    // Без ожидания MQTT клиент попытается подключиться без IP-адреса и получит ошибку.
    // 15 секунд — достаточно для типичного DHCP handshake (3-5 сек) + запас на слабый сигнал.
    // Если STA не подключится — продолжаем в AP-режиме (веб-сервер настроек доступен).
    {
        char ssid[SETTINGS_SSID_MAX_LEN] = {0};
        bool sta_configured = (settings_get_wifi_ssid(ssid, sizeof(ssid)) == ESP_OK && ssid[0] != '\0');

        if (sta_configured) {
            ESP_LOGI(TAG, "Ожидание подключения STA к '%s' (до 15 сек)...", ssid);

            // Используем существующий EventGroup (s_schedule_event) + отдельный таймаут
            int timeout_sec = 0;
            while (timeout_sec < 15) {
                esp_netif_ip_info_t ip_info;
                esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                if (sta_netif && esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
                    ESP_LOGI(TAG, "WiFi STA получил IP: " IPSTR, IP2STR(&ip_info.ip));
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1000));
                timeout_sec++;
            }
            if (timeout_sec >= 15) {
                ESP_LOGW(TAG, "Таймаут ожидания WiFi STA (15 сек) — продолжаем без STA");
            }
        } else {
            ESP_LOGI(TAG, "WiFi STA не настроен — только AP режим");
        }
    }

    // ==========================================================================
    // 3.3. ИНИЦИАЛИЗАЦИЯ mDNS (hydronft.local)
    // ==========================================================================

    if (mdns_init() == ESP_OK) {
        mdns_hostname_set("hydronft");
        mdns_instance_name_set("HydroNFT Controller");
        mdns_service_add(NULL, "_http", "_tcp", CONFIG_WEB_SERVER_PORT, NULL, 0);
        ESP_LOGI(TAG, "mDNS инициализирован: http://hydronft.local");
    } else {
        ESP_LOGW(TAG, "Ошибка инициализации mDNS");
    }

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
    // 5.2. ИНИЦИАЛИЗАЦИЯ ВЕБ-СЕРВЕРА
    // ==========================================================================

#ifdef CONFIG_WEB_SERVER_ENABLED
    web_server_init();
#endif

    // ==========================================================================
    // 6. ЗАПУСК ЗАДАЧ FREERTOS
    // ==========================================================================
    // Порядок запуска определён по зависимостям:
    //   1. Сенсоры (DHT, ADS1115) — независимы, запускаются первыми
    //   2. MQTT — зависит от WiFi, но не от сенсоров
    //   3. Расписания — зависят от SNTP (время) и MQTT (режим)
    //   4. ESP-NOW — зависит от WiFi
    //   5. WiFi reconnect — зависит от settings

    // DHT — ядро 1, приоритет 3 (низкий, блокирующий I/O)

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

    // ADS1115 — ядро 1, приоритет 4 (выше DHT, блокирующий I2C ~500 мс на 4 канала)
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

    // MQTT sensor — ядро 0, приоритет 4 (публикация данных каждые 5 сек)
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

    // MQTT OTA check — ядро 0, приоритет 4, стек 4096 (нужен для esp_err_to_name + логирование)
    if (xTaskCreatePinnedToCore(
        mqtt_ota_check_task,
        "mqtt_ota_check_task",
        4096,
        (void *)s_schedule_event,
        4,
        NULL,
        PRO_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'mqtt_ota_check_task'");
    }

    // Light schedule — ядро 1, приоритет 3 (расписание, зависит от SNTP)
    // Перенесено на ядро 1 — изоляция от OTA/MQTT, гарантия своевременного переключения света
    if (xTaskCreatePinnedToCore(
        light_schedule_task,
        "light_schedule_task",
        3072,
        (void *)s_schedule_event,
        3,
        NULL,
        APP_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'light_schedule_task'");
    }

    // Pump/valve schedule — ядро 1, приоритет 3 (критично для оборудования, автомат состояний)
    // Перенесено на ядро 1 — защита от OTA-задержек на ядре 0, затопление/пересыхание недопустимо
    if (xTaskCreatePinnedToCore(
        pump_valve_schedule_task,
        "pmp_vlv_sched",
        3072,
        (void *)s_schedule_event,
        3,
        NULL,
        APP_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'pump_valve_schedule_task'");
    }

    // ESP-NOW receiver — ядро 0, приоритет 4 (приём данных уровня воды, быстрый отклик)
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

    // WiFi reconnect — ядро 0, приоритет 2 (самый низкий, опрос каждые 2 сек)
    if (xTaskCreatePinnedToCore(
        wifi_reconnect_task,
        "wifi_reconnect_task",
        2048,
        NULL,
        2,
        NULL,
        PRO_CPU_NUM
    ) != pdPASS) {
        ESP_LOGE(TAG, "КРИТИЧЕСКАЯ ОШИБКА: не удалось создать задачу 'wifi_reconnect_task'");
    }

    ESP_LOGI(TAG, "Приложение запущено. Ожидание команд MQTT...");
    ESP_LOGI(TAG, "SNTP инициализирован. Время синхронизируется");
    ESP_LOGI(TAG, "Задачи распределены: ADS1115/DHT - ядро 1, MQTT/OTA/SNTP - ядро 0");

    // Удаляем задачу main — она больше не нужна, экономим ~3KB RAM.
    // FreeRTOS автоматически освобождает стек удалённой задачи (idle task cleanup).
    // После этого управление передаётся планировщику FreeRTOS.
    vTaskDelete(NULL);
}
