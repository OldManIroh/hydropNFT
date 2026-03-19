/**
 * @file main.c
 * @brief Главный файл приложения HydroNFT
 * 
 * Это основной файл приложения для контроллера гидропонной системы.
 * Реализует следующий функционал:
 * 
 * - Управление оборудованием (насос, свет, клапан) через GPIO
 * - Чтение данных с датчиков через ADS1115 (pH, TDS, температура)
 * - MQTT подключение к Home Assistant
 * - Поддержка OTA обновлений через MQTT команду
 * 
 * Архитектура приложения:
 * @verbatim
 * +-------------------+
 * |    app_main()     |
 * +-------------------+
 *         |
 *         v
 * +-------------------+
 * | Инициализация GPIO |
 * +-------------------+
 *         |
 *         v
 * +-------------------+
 * | OTA Check & Init  |
 * +-------------------+
 *         |
 *         v
 * +-------------------+
 * |   MQTT Init       |
 * +-------------------+
 *         |
 *         +---> ads1115_task()      [Чтение сенсоров]
 *         |
 *         +---> mqtt_sensor_task()  [Публикация в MQTT]
 *         |
 *         +---> mqtt_ota_check_task() [Проверка OTA флага]
 * @endverbatim
 * 
 * Задачи FreeRTOS:
 * | Задача                 | Приоритет | Стек (байты) | Ядро | Описание           |
 * |------------------------|-----------|--------------|------|--------------------|
 * | dht_task               | 3         | 2048         | 1    | Чтение DHT         |
 * | ads1115_task           | 3         | 4096         | 1    | Чтение ADS1115     |
 * | mqtt_sensor_task       | 4         | 2048         | 0    | Публикация сенсоров|
 * | mqtt_ota_check_task    | 4         | 3072         | 0    | Проверка OTA флага |
 * | mqtt_ota_progress_task | 4         | 2048         | 0    | Публикация OTA прогресса |
 * 
 * @author HydroNFT Team
 * @version 1.0
 * @date 2024
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "ads1115.h"
#include "dht.h"
#include "ota_client.h"
#include "hydro_mqtt_client.h"
#include "sntp_client.h"
#include "log_forwarder.h"
#include "main.h"

#ifndef APP_CPU_NUM
#define APP_CPU_NUM 1  ///< Ядро 1 (APP_CPU) для задач
#endif

// Дескрипторы задач для управления (приостановка при OTA)
TaskHandle_t ads1115_task_handle = NULL;  ///< Дескриптор задачи ADS1115

// Семафор для корректной остановки ADS1115 (пункт 1.8)
static SemaphoreHandle_t ads1115_running_sem = NULL;

/**
 * @brief Получить семафор задачи ADS1115
 * 
 * Используется компонентом OTA для корректной остановки задачи ADS1115
 * 
 * @return SemaphoreHandle_t Семафор или NULL если ещё не создан
 */
SemaphoreHandle_t get_ads1115_running_sem(void)
{
    return ads1115_running_sem;
}

// Объявления функций для работы с OTA прогрессом
void mqtt_ota_progress_task(void *pvParameters);
void start_ota_progress_task(void);
void stop_ota_progress_task(void);

// ============================================================================
// КОНФИГУРАЦИЯ GPIO
// ============================================================================

/// @brief GPIO пины для управления оборудованием
/// @{
#define PUMP_PIN  19  ///< Насос циркуляции воды
#define LIGHT_PIN 18  ///< Фитолампа освещения
#define VALVE_PIN 4   ///< Электроклапан слива/подачи
/// @}

/// @brief GPIO пин для датчика DHT (настраивается через menuconfig)
#define DHT_PIN CONFIG_EXAMPLE_DATA_GPIO

/// @brief GPIO пины для датчиков
/// @{
/// @note Настраивается через menuconfig: Example configuration → Data GPIO number
/// @}

/// @brief Тип датчика DHT
/// @{
/// @note Настраивается через menuconfig: Example configuration → Select chip type
#if defined(CONFIG_EXAMPLE_TYPE_DHT11)
#define DHT_TYPE DHT_TYPE_DHT11
#elif defined(CONFIG_EXAMPLE_TYPE_AM2301)
#define DHT_TYPE DHT_TYPE_AM2301
#elif defined(CONFIG_EXAMPLE_TYPE_SI7021)
#define DHT_TYPE DHT_TYPE_SI7021
#else
#define DHT_TYPE DHT_TYPE_DHT11  // По умолчанию
#endif
/// @}

// Переменные для хранения данных DHT
static float dht_temperature = 0.0f;  ///< Температура с DHT
static float dht_humidity = 0.0f;     ///< Влажность с DHT
static SemaphoreHandle_t dht_data_mutex = NULL;  ///< Мьютекс для защиты данных DHT
static bool dht_data_valid = false;   ///< Флаг: данные DHT получены успешно (ЛОГ-3)

/// Тег для системы логирования ESP-IDF
static const char *TAG = "main";

// ============================================================================
// ПЕРЕМЕННЫЕ ДЛЯ РУЧНОГО УПРАВЛЕНИЯ СВЕТОМ
// ============================================================================

/// Флаг ручного управления светом (приоритет над расписанием)
static bool light_manual_override = false;

/// Время последнего ручного переключения света
static time_t light_manual_time = 0;

/// Время авто-сброса ручного управления (2 часа)
#define LIGHT_MANUAL_TIMEOUT_SECONDS (2 * 3600)

/// Флаг ручного управления насосом (приоритет над автоматикой)
static bool pump_manual_override = false;

/// Время последнего ручного переключения насоса
static time_t pump_manual_time = 0;

/// Время авто-сброса ручного управления (5 минут)
#define PUMP_MANUAL_TIMEOUT_SECONDS (5 * 60)

/// Флаг ручного управления клапаном (приоритет над автоматикой)
static bool valve_manual_override = false;

/// Время последнего ручного переключения клапана
static time_t valve_manual_time = 0;

/// Время авто-сброса ручного управления (5 минут)
#define VALVE_MANUAL_TIMEOUT_SECONDS (5 * 60)

// ============================================================================
// ФУНКЦИИ ДЛЯ ПОЛУЧЕНИЯ ДАННЫХ DHT (используются в mqtt_client)
// ============================================================================

/**
 * @brief Получить температуру с DHT датчика
 * @return Температура в °C
 */
float get_dht_temperature(void)
{
    float temp = 0.0f;
    if (dht_data_mutex != NULL) {
        xSemaphoreTake(dht_data_mutex, portMAX_DELAY);
        temp = dht_temperature;
        xSemaphoreGive(dht_data_mutex);
    }
    return temp;
}

/**
 * @brief Получить влажность с DHT датчика
 * @return Влажность в %
 */
float get_dht_humidity(void)
{
    float hum = 0.0f;
    if (dht_data_mutex != NULL) {
        xSemaphoreTake(dht_data_mutex, portMAX_DELAY);
        hum = dht_humidity;
        xSemaphoreGive(dht_data_mutex);
    }
    return hum;
}

/**
 * @brief Проверить валидность данных DHT (ЛОГ-3)
 * @return true если данные были успешно получены хотя бы раз
 * @return false если данные ещё не получены
 */
bool is_dht_data_valid(void)
{
    bool valid = false;
    if (dht_data_mutex != NULL) {
        xSemaphoreTake(dht_data_mutex, portMAX_DELAY);
        valid = dht_data_valid;
        xSemaphoreGive(dht_data_mutex);
    }
    return valid;
}

// ============================================================================
// ФУНКЦИИ УПРАВЛЕНИЯ GPIO (насос, свет, клапан)
// ============================================================================

/**
 * @brief Установить состояние насоса
 * @param state true = включить, false = выключить
 * @param manual true = ручное управление (устанавливает флаг override на 5 минут)
 */
void set_pump_state(bool state, bool manual)
{
    gpio_set_level(PUMP_PIN, state ? 1 : 0);
    ESP_LOGI(TAG, "Насос: %s (%s)", state ? "ВКЛ" : "ВЫКЛ", manual ? "вручную" : "автоматически");

    // Если ручное управление - устанавливаем флаг override на 5 минут
    if (manual) {
        pump_manual_override = true;
        pump_manual_time = time(NULL);
        ESP_LOGI(TAG, "Ручное управление насосом, авто-режим отключён на 5 минут");
    }
}

/**
 * @brief Установить состояние света
 * @param state true = включить, false = выключить
 * @param manual true = ручное управление (устанавливает флаг override)
 */
void set_light_state(bool state, bool manual)
{
    gpio_set_level(LIGHT_PIN, state ? 1 : 0);
    ESP_LOGI(TAG, "Свет: %s (%s)", state ? "ВКЛ" : "ВЫКЛ", manual ? "вручную" : "расписание");

    // Если ручное управление - устанавливаем флаг override на 2 часа
    if (manual) {
        light_manual_override = true;
        light_manual_time = time(NULL);
        ESP_LOGI(TAG, "Ручное управление светом, авто-режим отключён на 2 часа");
    }
}

/**
 * @brief Установить состояние клапана
 * @param state true = открыть, false = закрыть
 * @param manual true = ручное управление (устанавливает флаг override на 5 минут)
 */
void set_valve_state(bool state, bool manual)
{
    gpio_set_level(VALVE_PIN, state ? 1 : 0);
    ESP_LOGI(TAG, "Клапан: %s (%s)", state ? "ОТКРЫТ" : "ЗАКРЫТ", manual ? "вручную" : "автоматически");

    // Если ручное управление - устанавливаем флаг override на 5 минут
    if (manual) {
        valve_manual_override = true;
        valve_manual_time = time(NULL);
        ESP_LOGI(TAG, "Ручное управление клапаном, авто-режим отключён на 5 минут");
    }
}

/**
 * @brief Получить состояние насоса
 * @return true если включен, false если выключен
 */
bool get_pump_state(void)
{
    return gpio_get_level(PUMP_PIN);
}

/**
 * @brief Получить состояние света
 * @return true если включен, false если выключен
 */
bool get_light_state(void)
{
    return gpio_get_level(LIGHT_PIN);
}

/**
 * @brief Получить состояние клапана
 * @return true если открыт, false если закрыт
 */
bool get_valve_state(void)
{
    return gpio_get_level(VALVE_PIN);
}

// ============================================================================
// ЗАДАЧА ЧТЕНИЯ DHT ДАТЧИКА
// ============================================================================

/**
 * @brief Задача для чтения температуры и влажности с датчика DHT
 * 
 * DHT11/DHT22 - цифровой датчик температуры и влажности.
 * Читает данные каждые 2 секунды (чаще не рекомендуется).
 * 
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 1 (APP_CPU)
 * @note Приоритет: 5 (выше среднего)
 * @note Размер стека: 2048 байт
 * @note Интервал чтения: 2 секунды
 *
 * @see dht_read_float_data()
 */
void dht_task(void *pvParameters)
{
    float temperature, humidity;

    // НЕ добавляем в watchdog — dht_read_float_data() может блокироваться
    // надолго при неисправном датчике, что вызовет ложное срабатывание WDT

    // Настраиваем pull-up резистор если включено в menuconfig
#ifdef CONFIG_EXAMPLE_INTERNAL_PULLUP
    gpio_set_pull_mode(DHT_PIN, GPIO_PULLUP_ONLY);
    ESP_LOGI(TAG, "DHT: Внутренний pull-up резистор включён");
#else
    ESP_LOGI(TAG, "DHT: Внутренний pull-up резистор выключен (требуется внешний 10kΩ)");
#endif

    ESP_LOGI(TAG, "DHT датчик инициализирован (GPIO %d)", DHT_PIN);
    
    // Пункт 2.3: Задержка на прогрев датчика после подачи питания
    ESP_LOGI(TAG, "DHT: Ожидание прогрева датчика (2 сек)...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        // Читаем температуру и влажность
        // DHT может блокироваться до 1 секунды при чтении
        esp_err_t res = dht_read_float_data(DHT_TYPE, DHT_PIN, &humidity, &temperature);

        if (res == ESP_OK) {
            // Сохраняем данные для публикации в MQTT (защищено мьютексом)
            xSemaphoreTake(dht_data_mutex, portMAX_DELAY);
            dht_temperature = temperature;
            dht_humidity = humidity;
            dht_data_valid = true;  // ЛОГ-3: флаг успешного получения данных
            xSemaphoreGive(dht_data_mutex);

            ESP_LOGI(TAG, "DHT: Влажность=%.1f%% Температура=%.1f°C", humidity, temperature);
        } else {
            ESP_LOGW(TAG, "Ошибка чтения DHT датчика: %d", res);
        }

        // Пауза 2 секунды (чаще читать не рекомендуется)
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ============================================================================
// ЗАДАЧА ЧТЕНИЯ ADS1115
// ============================================================================

/**
 * @brief Задача для чтения измерений с ADS1115
 *
 * ADS1115 - 4-канальный 16-битный АЦП с интерфейсом I2C.
 * Используется для чтения аналоговых датчиков:
 * - Канал 0: pH метр
 * - Канал 1: Температура
 * - Канал 2: TDS метр
 * - Канал 3: Резерв/другой датчик
 *
 * Процесс работы:
 * 1. Инициализация ADS1115
 * 2. Циклическое чтение всех 4 каналов
 * 3. Вывод результатов в лог
 * 4. Пауза 500 мс
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 1 (APP_CPU)
 * @note Приоритет: 5 (выше среднего)
 * @note Размер стека: 4096 байт
 *
 * @see ads1115_init()
 * @see ads1115_measure_all_channels()
 */
void ads1115_task(void *pvParameters)
{
    // Инициализация ADS1115
    esp_err_t res = ads1115_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось инициализировать ADS1115: %d", res);
        vTaskDelete(NULL);  // Удаляем задачу при ошибке
        return;
    }

    // Получаем семафор для корректной остановки (пункт 1.8)
    // Семафор создан в app_main() до запуска задач
    SemaphoreHandle_t sem = get_ads1115_running_sem();
    if (sem == NULL) {
        ESP_LOGE(TAG, "Семафор ADS1115 не создан");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "ADS1115 успешно инициализирован");

    // Массив для хранения измерений с 4 каналов
    ads1115_measurement_t measurements[4];
    /*
     * Структура measurements[i]:
     * - channel: номер канала (0-3)
     * - voltage: напряжение в вольтах
     * - raw_value: сырое значение ADC
     * - error: код ошибки (ESP_OK если успешно)
     */

    while (1) {
        // Пункт 1.8: Ждём разрешения на работу через семафор
        if (xSemaphoreTake(sem, pdMS_TO_TICKS(600)) == pdTRUE) {
            // Чтение всех 4 каналов одновременно
            res = ads1115_measure_all_channels(measurements);

            if (res == ESP_OK) {
                // Вывод результатов каждого канала
                for (int i = 0; i < 4; i++) {
                    if (measurements[i].error == ESP_OK) {
                        ESP_LOGI(TAG, "Канал %u: напряжение = %.04f В, raw = %d",
                               measurements[i].channel,
                               measurements[i].voltage,
                               measurements[i].raw_value);
                    } else {
                        ESP_LOGE(TAG, "Канал %u: ошибка = %d",
                               measurements[i].channel,
                               measurements[i].error);
                    }
                }
            } else {
                ESP_LOGE(TAG, "Ошибка при измерениях: %d", res);
            }

            // Возвращаем семафор (разрешаем остановку)
            xSemaphoreGive(sem);
        } else {
            // Таймаут истёк - семафор был взят другим (остановка для OTA)
            ESP_LOGI(TAG, "ADS1115: семафор недоступен, ожидание...");
        }

        // Ожидание 500 мс перед следующим измерением
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// ЗАДАЧА ПУБЛИКАЦИИ ДАННЫХ СЕНСОРОВ В MQTT
// ============================================================================

/**
 * @brief Задача для публикации данных сенсоров в MQTT
 * 
 * Периодически публикует данные с датчиков в MQTT топики:
 * - hydro/sensor/ph/state - pH значение
 * - hydro/sensor/tds/state - TDS значение (ppm)
 * - hydro/sensor/level/state - Уровень воды (%)
 * 
 * Процесс работы:
 * 1. Проверка подключения к MQTT
 * 2. Публикация данных сенсоров
 * 3. Пауза 5 секунд
 * 
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU) - сетевой стек
 * @note Приоритет: 4 (средний)
 * @note Размер стека: 2048 байт
 * @note Интервал публикации: 5 секунд
 *
 * @see mqtt_client_is_connected()
 * @see mqtt_client_publish_sensor_data()
 */
void mqtt_sensor_task(void *pvParameters)
{
    while (1) {
        // Публикуем только при активном подключении к MQTT
        if (mqtt_client_is_connected()) {
            mqtt_client_publish_sensor_data();
        }
        
        // Пауза 5 секунд между публикациями
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// ============================================================================
// ЗАДАЧА ПРОВЕРКИ ФЛАГА OTA ОБНОВЛЕНИЯ
// ============================================================================

/**
 * @brief Задача для проверки флага OTA обновления из MQTT
 *
 * Мониторит флаг ota_update_requested, который устанавливается
 * при получении MQTT команды из топика hydro/ota/update.
 *
 * Процесс работы:
 * 1. Проверка подключения к MQTT
 * 2. Проверка флага OTA обновления
 * 3. При установке флага:
 *    - Сброс флага
 *    - Остановка задачи ADS1115 через семафор (пункт 1.8)
 *    - Запуск задачи OTA обновления
 * 4. Пауза 1 секунда
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU) - сетевой стек
 * @note Приоритет: 4 (средний)
 * @note Размер стека: 3072 байт
 * @note Интервал проверки: 1 секунда
 *
 * @see mqtt_client_get_ota_flag()
 * @see mqtt_client_reset_ota_flag()
 * @see ota_start_task()
 */
void mqtt_ota_check_task(void *pvParameters)
{
    while (1) {
        // Проверяем подключение и флаг OTA
        if (mqtt_client_is_connected() && mqtt_client_get_ota_flag()) {
            ESP_LOGI(TAG, "Получена команда OTA обновления из MQTT...");

            // Сбрасываем флаг, чтобы не запустить OTA повторно
            mqtt_client_reset_ota_flag();

            // Запускаем задачу публикации прогресса OTA
            start_ota_progress_task();

            // Запускаем задачу OTA обновления
            // ota_task сама проверит версии и при необходимости:
            // - остановит оборудование (насос, свет, клапан)
            // - остановит ADS1115 через семафор
            // - загрузит и установит новую прошивку
            // Если версии совпадут — ничего не будет остановлено
            ota_start_task();
        }

        // Пауза 1 секунда перед следующей проверкой
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// ЗАДАЧА УПРАВЛЕНИЯ СВЕТОМ ПО РАСПИСАНИЮ
// ============================================================================

/**
 * @brief Задача управления освещением по расписанию
 *
 * Включает свет в CONFIG_LIGHT_ON_HOUR часов и выключает
 * в CONFIG_LIGHT_OFF_HOUR часов.
 *
 * Процесс работы:
 * 1. Ожидание синхронизации SNTP
 * 2. Проверка текущего часа
 * 3. Включение/выключение света по расписанию
 * 4. Пауза 1 минута
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU)
 * @note Приоритет: 3 (ниже среднего)
 * @note Размер стека: 2048 байт
 * @note Интервал проверки: 1 минута
 */
void light_schedule_task(void *pvParameters)
{
    const int on_hour = CONFIG_LIGHT_ON_HOUR;
    const int off_hour = CONFIG_LIGHT_OFF_HOUR;

    // ЛОГ-4: Валидация конфигурации - часы включения и выключения должны различаться
    if (on_hour == off_hour) {
        ESP_LOGE(TAG, "ОШИБКА: LIGHT_ON_HOUR (%d) равен LIGHT_OFF_HOUR (%d)!", on_hour, off_hour);
        ESP_LOGE(TAG, "Свет не будет управляться. Исправьте конфигурацию в menuconfig!");
        vTaskDelete(NULL);  // Завершаем задачу при некорректной конфигурации
        return;
    }

    ESP_LOGI(TAG, "Задача управления светом запущена (вкл=%d:00, выкл=%d:00)",
             on_hour, off_hour);

    int last_action_hour = -1;  // Последний час действия

    while (1) {
        // Ждём синхронизации времени
        if (!sntp_client_is_synced()) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Получаем текущее время
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        int current_hour = timeinfo.tm_hour;

        // Проверяем таймаут ручного управления (2 часа)
        if (light_manual_override && (now - light_manual_time) > LIGHT_MANUAL_TIMEOUT_SECONDS) {
            light_manual_override = false;
            ESP_LOGI(TAG, "Ручное управление отключено, возвращаем авто-режим");
        }

        // Если есть ручной режим - пропускаем авто-управление
        if (light_manual_override) {
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }

        // Проверяем условие включения (текущий час == on_hour и ещё не включали)
        if (current_hour == on_hour && last_action_hour != on_hour) {
            ESP_LOGI(TAG, "Время включать свет (%d:00)", on_hour);
            set_light_state(true, false);
            last_action_hour = current_hour;
        }
        // Проверяем условие выключения (текущий час == off_hour и ещё не выключали)
        else if (current_hour == off_hour && last_action_hour != off_hour) {
            ESP_LOGI(TAG, "Время выключать свет (%d:00)", off_hour);
            set_light_state(false, false);
            last_action_hour = current_hour;
        }
        // Сбрасываем last_action_hour если час сменился
        else if (current_hour != last_action_hour) {
            last_action_hour = -1;
        }

        // Пауза 1 минута
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

// ============================================================================
// ЗАДАЧА УПРАВЛЕНИЯ НАСОСОМ (ЗАГЛУШКА)
// ============================================================================

/**
 * @brief Задача управления насосом по расписанию
 *
 * @note ЗАГЛУШКА - логика управления будет добавлена позже
 */
void pump_schedule_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Задача управления насосом запущена (ЗАГЛУШКА)");

    while (1) {
        // Ждём синхронизации времени
        if (!sntp_client_is_synced()) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Проверяем таймаут ручного управления (5 минут)
        time_t now;
        if (pump_manual_override) {
            time(&now);
            if ((now - pump_manual_time) > PUMP_MANUAL_TIMEOUT_SECONDS) {
                pump_manual_override = false;
                ESP_LOGI(TAG, "Ручное управление насосом отключено, возвращаем авто-режим");
            }
        }

        // Пауза 1 минута
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

// ============================================================================
// ЗАДАЧА УПРАВЛЕНИЯ КЛАПАНОМ (ЗАГЛУШКА)
// ============================================================================

/**
 * @brief Задача управления клапаном по расписанию
 *
 * @note ЗАГЛУШКА - логика управления будет добавлена позже
 */
void valve_schedule_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Задача управления клапаном запущена (ЗАГЛУШКА)");

    while (1) {
        // Ждём синхронизации времени
        if (!sntp_client_is_synced()) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        // Проверяем таймаут ручного управления (5 минут)
        time_t now;
        if (valve_manual_override) {
            time(&now);
            if ((now - valve_manual_time) > VALVE_MANUAL_TIMEOUT_SECONDS) {
                valve_manual_override = false;
                ESP_LOGI(TAG, "Ручное управление клапаном отключено, возвращаем авто-режим");
            }
        }

        // Пауза 1 минута
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}


// ============================================================================
// ЗАДАЧА ПУБЛИКАЦИИ ПРОГРЕССА OTA В MQTT
// ============================================================================

// Дескриптор задачи публикации прогресса OTA
static TaskHandle_t ota_progress_task_handle = NULL;

/**
 * @brief Задача для публикации прогресса OTA обновления в MQTT
 * 
 * Периодически публикует текущий прогресс загрузки прошивки
 * в топик hydro/ota/progress в формате JSON.
 * 
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU)
 * @note Приоритет: 4 (средний)
 * @note Размер стека: 4096 байт (snprintf + ESP_LOGI + log_forwarder + запас)
 * @note Задача удаляет себя сама после завершения OTA
 */
void mqtt_ota_progress_task(void *pvParameters)
{
    int last_progress = -1;  // Последний отправленный прогресс
    int no_change_count = 0;  // Счётчик циклов без изменений
    int total = 0;
    int downloaded = 0;

    ESP_LOGI(TAG, "Задача публикации прогресса OTA запущена");

    while (1) {
        // Получаем текущий прогресс OTA
        int progress = ota_get_progress_percent();

        // Если OTA активна (прогресс >= 0)
        if (progress >= 0) {
            total = ota_get_total_size();
            downloaded = ota_get_downloaded_bytes();

            // Публикуем в MQTT топик через mqtt_client
            mqtt_client_publish_ota_progress(progress, downloaded, total);

            ESP_LOGI(TAG, "OTA прогресс: %d%% (%d/%d)", progress, downloaded, total);
            last_progress = progress;
            no_change_count = 0;
        }
        // Если OTA не активна - проверяем не завершилась ли она
        else {
            no_change_count++;
            
            // Если прогресс не меняется 3 секунды - OTA завершена
            if (no_change_count >= 3) {
                ESP_LOGI(TAG, "OTA сессия завершена (прогресс: %d%%)", last_progress);
                
                // Если прогресс был 100% - всё успешно
                if (last_progress == 100) {
                    mqtt_client_publish_ota_status("completed");
                }
                
                ESP_LOGI(TAG, "Задача публикации прогресса OTA завершается");
                
                // Сбрасываем дескриптор и удаляем задачу
                ota_progress_task_handle = NULL;
                vTaskDelete(NULL);
            }
        }
        
        // Пауза 1 секунда
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Запуск задачи публикации прогресса OTA
 * 
 * Создаёт задачу для мониторинга и публикации прогресса OTA обновления.
 * Задача автоматически удаляет себя после завершения OTA.
 * 
 * @note Вызывается при запуске OTA обновления
 * @note Задача работает на ядре 0 (PRO_CPU)
 */
void start_ota_progress_task(void)
{
    if (ota_progress_task_handle == NULL) {
        xTaskCreatePinnedToCore(
            mqtt_ota_progress_task,
            "mqtt_ota_progress_task",
            4096,  // 4096 байт (json_msg[128] + snprintf + ESP_LOGI + log_forwarder + запас)
            NULL,
            4,
            &ota_progress_task_handle,
            PRO_CPU_NUM  // Ядро 0
        );
        ESP_LOGI(TAG, "Задача публикации прогресса OTA создана");
    }
}

/**
 * @brief Остановка задачи публикации прогресса OTA
 * 
 * Принудительно удаляет задачу публикации прогресса.
 * Используется при ошибках OTA.
 */
void stop_ota_progress_task(void)
{
    if (ota_progress_task_handle != NULL) {
        vTaskDelete(ota_progress_task_handle);
        ota_progress_task_handle = NULL;
        ESP_LOGI(TAG, "Задача публикации прогресса OTA остановлена");
    }
}

// ============================================================================
// ГЛАВНАЯ ФУНКЦИЯ ПРИЛОЖЕНИЯ
// ============================================================================

/**
 * @brief Главная функция приложения - точка входа
 * 
 * Вызывается ESP-IDF после инициализации базовых систем.
 * Выполняет инициализацию всех подсистем и запуск задач.
 * 
 * Последовательность инициализации:
 * 1. Конфигурация GPIO пинов (насос, свет, клапан)
 * 2. Проверка статуса OTA прошивки (диагностика при необходимости)
 * 3. Инициализация OTA компонента (NVS, сеть, Wi-Fi)
 * 4. Инициализация MQTT клиента
 * 5. Запуск задач FreeRTOS
 * 
 * @note Функция не возвращает управление (работает в бесконечном цикле)
 * @note После запуска задач управление передаётся планировщику FreeRTOS
 * 
 * @see ads1115_task()
 * @see mqtt_sensor_task()
 * @see mqtt_ota_check_task()
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Запуск приложения HydroNFT");

    // ==========================================================================
    // 1. КОНФИГУРАЦИЯ GPIO ДЛЯ УПРАВЛЕНИЯ ОБОРУДОВАНИЕМ
    // ==========================================================================
    
    // Насос циркуляции воды (GPIO 19)
    gpio_config_t pump_io_config = {
        .intr_type = GPIO_INTR_DISABLE,  // Отключаем прерывания
        .mode = GPIO_MODE_OUTPUT,        // Режим выхода
        .pin_bit_mask = (1ULL << PUMP_PIN),  // Маска пина
        .pull_down_en = 0,               // Отключаем подтяжку к земле
        .pull_up_en = 0                  // Отключаем подтяжку к питанию
    };
    gpio_config(&pump_io_config);

    // Фитолампа (GPIO 18)
    gpio_config_t light_io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LIGHT_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&light_io_config);

    // Электроклапан (GPIO 5)
    gpio_config_t valve_io_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << VALVE_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&valve_io_config);

    // ПОТ-10: Устанавливаем GPIO в безопасное состояние (выключено)
    gpio_set_level(PUMP_PIN, 0);
    gpio_set_level(LIGHT_PIN, 0);
    gpio_set_level(VALVE_PIN, 0);
    ESP_LOGI(TAG, "GPIO установлены в безопасное состояние (насос, свет, клапан = ВЫКЛ)");

    // ==========================================================================
    // 2. ПРОВЕРКА СТАТУСА OTA ПРОШИВКИ
    // ==========================================================================
    
    /*
     * Проверяем, не требуется ли диагностика после OTA обновления.
     * Если устройство перезагрузилось с новой прошивкой, статус будет
     * ESP_OTA_IMG_PENDING_VERIFY и запустится диагностика.
     * 
     * Диагностика:
     * - Ждёт 5 секунд
     * - Проверяет GPIO диагностического пина
     * - При успехе: подтверждает прошивку
     * - При неудаче: откатывается к предыдущей версии
     */
    ota_check_and_diagnose();

    // ==========================================================================
    // 3. ИНИЦИАЛИЗАЦИЯ OTA КОМПОНЕНТА (NVS, СЕТЬ)
    // ==========================================================================
    
    /*
     * Инициализирует:
     * - NVS flash (хранение настроек, Wi-Fi кредов)
     * - Сетевой стек (esp_netif, esp_event_loop)
     * - Подключение к Wi-Fi/Ethernet
     * - Отключение энергосбережения Wi-Fi
     * 
     * Блокирует выполнение до успешного подключения к сети!
     */
    ota_init();

    // ==========================================================================
    // 4. ИНИЦИАЛИЗАЦИЯ MQTT КЛИЕНТА
    // ==========================================================================

    /*
     * Создаёт MQTT клиента и подключается к брокеру:
     * - Broker: mqtt://192.168.0.107:1883
     * - Username: hydroesp32
     * - Password: asda
     *
     * После подключения:
     * - Отправляет Home Assistant Discovery конфигурации
     * - Подписывается на топики команд
     * - Публикует текущие состояния
     *
     * НЕ блокирует выполнение - подключение асинхронное!
     */
    mqtt_client_init();

    // ==========================================================================
    // 4.1. ИНИЦИАЛИЗАЦИЯ ПЕРЕСЫЛКИ ЛОГОВ В MQTT
    // ==========================================================================

    /*
     * Инициализирует компонент log_forwarder для перехвата WARNING и ERROR
     * логов и отправки их в MQTT топик hydro/log для Home Assistant.
     *
     * Вызывается ПОСЛЕ mqtt_client_init() т.к. использует MQTT подключение.
     */
    log_forwarder_init();

    // ==========================================================================
    // 5. ИНИЦИАЛИЗАЦИЯ SNTP КЛИЕНТА
    // ==========================================================================

    /*
     * Инициализирует SNTP клиент для синхронизации времени с NTP серверами.
     * Использует серверы:
     * - ntp.msk-ix.ru (Московский IX)
     * - ru.pool.ntp.org (Российский pool)
     * - pool.ntp.org (Глобальный pool)
     * - time.google.com (Google)
     *
     * После синхронизации вызывается callback, который логирует время.
     * НЕ блокирует выполнение - синхронизация асинхронная!
     */
    sntp_client_init();

    // ==========================================================================
    // 6. ИНИЦИАЛИЗАЦИЯ СЕМАФОРА ДЛЯ ADS1115
    // ==========================================================================
    
    // Создаём семафор для корректной остановки ADS1115 до запуска задачи
    // Это исправляет race condition, когда mqtt_ota_check_task() пытается получить
    // семафор до его создания в ads1115_task()
    ads1115_running_sem = xSemaphoreCreateBinary();
    if (ads1115_running_sem == NULL) {
        ESP_LOGE(TAG, "Не удалось создать семафор ADS1115");
        return; // Прерываем инициализацию
    }
    // Начальное состояние: задача запущена (семафор доступен)
    xSemaphoreGive(ads1115_running_sem);
    ESP_LOGI(TAG, "Семафор ADS1115 создан и инициализирован");

    // Создаём мьютекс для защиты данных DHT (КРИТ-3)
    dht_data_mutex = xSemaphoreCreateMutex();
    if (dht_data_mutex == NULL) {
        ESP_LOGE(TAG, "Не удалось создать мьютекс DHT");
        return;
    }
    ESP_LOGI(TAG, "Мьютекс DHT создан");

    // ==========================================================================
    // 7. ЗАПУСК ЗАДАЧ FREERTOS
    // ==========================================================================

    // Задача чтения DHT (приоритет 3, ядро 1 - APP_CPU)
    // ПОТ-6: Стек увеличен до 4096 байт для запаса при логировании
    // DHT использует GPIO, работает на ядре 1
    // Приоритет 3
    // Стек: 4096 байт - dht_read_float_data (может блокироваться),
    // ESP_LOGI/ESP_LOGW, 2 float переменные (8 байт), esp_task_wdt, запас 3×
    xTaskCreatePinnedToCore(
        dht_task,
        "dht_task",
        4096,  // ПОТ-6: увеличено до 4096 байт (watchdog timeout)
        NULL,
        3,
        NULL,
        APP_CPU_NUM  // Ядро 1
    );

    // Задача чтения ADS1115 (приоритет 4, ядро 1 - APP_CPU)
    // ПОТ-1: Приоритет увеличен до 4 для уменьшения задержек I2C при чтении DHT
    // I2C драйвер работает на ядре 1, поэтому оставляем здесь
    // Стек: 4096 байт - measurements[4] (48 байт),
    // медианный фильтр sorted[5] (20 байт), ESP_LOGI в цикле, I2C драйвер, запас 3×
    xTaskCreatePinnedToCore(
        ads1115_task,
        "ads1115_task",
        4096,  // 4096 байт
        NULL,
        4,  // ПОТ-1: увеличено с 3 до 4
        &ads1115_task_handle,  // Сохраняем дескриптор для приостановки при OTA
        APP_CPU_NUM  // Ядро 1
    );

    // Задача публикации сенсоров в MQTT (приоритет 4, ядро 0 - PRO_CPU)
    // ПОТ-7: Стек увеличен до 3072 байт для запаса при логировании
    // Сетевой стек и Wi-Fi работают на ядре 0
    // Стек: 3072 байт - mqtt_client_publish_sensor_data вызывается,
    // 6 публикаций, msg[64] на стеке, запас 4×
    xTaskCreatePinnedToCore(
        mqtt_sensor_task,
        "mqtt_sensor_task",
        3072,  // ПОТ-7: увеличено с 2048 до 3072 байт
        NULL,
        4,
        NULL,
        PRO_CPU_NUM  // Ядро 0
    );

    // Задача проверки флага OTA из MQTT (приоритет 4, ядро 0 - PRO_CPU)
    // OTA использует HTTP и запись во flash - лучше на ядре 0
    // Стек: 4096 байт - ota_start_task, xSemaphoreTake,
    // start_ota_progress_task, ESP_LOGI, запас 3×
    xTaskCreatePinnedToCore(
        mqtt_ota_check_task,
        "mqtt_ota_check_task",
        4096,  // ПОТ-10: увеличено с 3072 до 4096 байт
        NULL,
        4,
        NULL,
        PRO_CPU_NUM  // Ядро 0
    );

    // Задача управления светом по расписанию (приоритет 3, ядро 0 - PRO_CPU)
    // Использует SNTP время для включения/выключения света
    // Стек: 3072 байт - set_light_state, ESP_LOGI, localtime_r (struct tm ~64 байт),
    // локальные переменные (time_t, int), запас 3×
    xTaskCreatePinnedToCore(
        light_schedule_task,
        "light_schedule_task",
        3072,  // ПОТ-9: увеличено с 2048 до 3072 байт (переполнение стека)
        NULL,
        3,
        NULL,
        PRO_CPU_NUM  // Ядро 0
    );

    // Задача управления насосом по расписанию (приоритет 3, ядро 0 - PRO_CPU)
    // ЗАГЛУШКА - логика управления будет добавлена позже
    // Стек: 3072 байт - ESP_LOGI, localtime_r, ESP_LOGI, запас
    xTaskCreatePinnedToCore(
        pump_schedule_task,
        "pump_schedule_task",
        3072,
        NULL,
        3,
        NULL,
        PRO_CPU_NUM  // Ядро 0
    );

    // Задача управления клапаном по расписанию (приоритет 3, ядро 0 - PRO_CPU)
    // ЗАГЛУШКА - логика управления будет добавлена позже
    // Стек: 3072 байт - ESP_LOGI, localtime_r, ESP_LOGI, запас
    xTaskCreatePinnedToCore(
        valve_schedule_task,
        "valve_schedule_task",
        3072,
        NULL,
        3,
        NULL,
        PRO_CPU_NUM  // Ядро 0
    );

    // Задача публикации прогресса OTA ОТКЛЮЧЕНА
    // Задача создаётся динамически при запуске OTA (start_ota_progress_task)
    // и удаляется после завершения обновления
    // xTaskCreatePinnedToCore(
    //     mqtt_ota_progress_task,
    //     "mqtt_ota_progress_task",
    //     configMINIMAL_STACK_SIZE * 3,
    //     NULL,
    //     4,
    //     NULL,
    //     PRO_CPU_NUM
    // );

    ESP_LOGI(TAG, "Приложение запущено. Ожидание команд MQTT...");
    ESP_LOGI(TAG, "SNTP инициализирован. Время синхронизируется");
    ESP_LOGI(TAG, "Задачи распределены: ADS1115 - ядро 1, MQTT/OTA/SNTP - ядро 0");

    // Управление передаётся планировщику FreeRTOS
    // app_main() завершается, задачи работают автономно
}
