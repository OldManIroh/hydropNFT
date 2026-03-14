/**
 * @file mqtt_client.c
 * @brief MQTT клиент для HydroNFT проекта
 * 
 * Этот модуль реализует MQTT клиент для подключения к Home Assistant
 * и управления устройством через MQTT протокол.
 * 
 * Функционал:
 * - Подключение к MQTT брокеру
 * - Home Assistant MQTT Discovery (автоматическое обнаружение устройств)
 * - Управление выключателями (насос, свет)
 * - Публикация данных сенсоров (pH, TDS, уровень воды)
 * - Поддержка OTA обновлений через MQTT команду
 * 
 * @author HydroNFT Team
 * @version 1.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "mqtt_client.h"           // Заголовочный файл ESP-IDF MQTT библиотеки
#include "hydro_mqtt_client.h"     // Локальный заголовочный файл компонента

/// Тег для системы логирования ESP-IDF
static const char *TAG = "mqtt_client";

/// Дескриптор MQTT клиента - используется для всех операций с MQTT
static esp_mqtt_client_handle_t mqtt_client;

/// Флаг состояния подключения к MQTT брокеру
static bool mqtt_connected = false;

/// Флаг запроса OTA обновления - устанавливается при получении команды из MQTT
/// Используется в main.c для запуска процесса обновления прошивки
static bool ota_update_requested = false;

/// @brief Состояния выключателей
/// @{
static bool pump_state = false;   ///< Насос циркуляции - выключен по умолчанию
static bool light_state = false;  ///< Фитолампа - выключена по умолчанию
static bool valve_state = false;  ///< Клапан - выключен по умолчанию
/// @}

// ============================================================================
// ЗАГЛУШКИ ДАТЧИКОВ (замените на реальные функции чтения)
// ============================================================================

/**
 * @brief Чтение значения pH датчика
 * @return Значение pH (6.8 - 7.7) - случайное значение для тестирования
 *
 * @note ЗАМЕНИТЕ на реальное чтение с ADC через ADS1115
 */
static float read_ph(void) { return 6.8f + (rand() % 10) / 10.0f; }

/**
 * @brief Чтение значения TDS датчика
 * @return Значение TDS в ppm (500 - 549) - случайное значение для тестирования
 *
 * @note ЗАМЕНИТЕ на реальное чтение с ADC через ADS1115
 */
static int read_tds(void) { return 500 + rand() % 50; }

/**
 * @brief Чтение значения уровня воды
 * @return Уровень воды в процентах (75 - 84) - случайное значение для тестирования
 *
 * @note ЗАМЕНИТЕ на реальное чтение с емкостного датчика
 */
static int read_level(void) { return 75 + rand() % 10; }

// ============================================================================
// ВНЕШНИЕ ПЕРЕМЕННЫЕ (из main.c)
// ============================================================================

/**
 * @brief Получить температуру с DHT датчика
 * @return Температура в °C
 */
extern float get_dht_temperature(void);

/**
 * @brief Получить влажность с DHT датчика
 * @return Влажность в %
 */
extern float get_dht_humidity(void);

// ============================================================================
// HOME ASSISTANT MQTT DISCOVERY
// ============================================================================

/**
 * @brief Отправка конфигурации сенсора для Home Assistant Discovery
 * 
 * Home Assistant использует MQTT Discovery для автоматического обнаружения
 * устройств. Эта функция отправляет JSON конфигурацию в специальный топик,
 * после чего HA автоматически создаст entity для данного сенсора.
 * 
 * @param client Дескриптор MQTT клиента
 * @param object_id Уникальный идентификатор объекта (например: "ph", "tds")
 * @param name Отображаемое имя сенсора в HA (например: "pH Level")
 * @param unit Единица измерения (например: "ppm", "%", "")
 * @param device_class Класс устройства для правильной иконки в HA
 *                     (например: "pH", "temperature", "humidity", "")
 * @param state_topic Топик, в который будут публиковаться значения сенсора
 * 
 * @note Формат топика: homeassistant/sensor/{device_name}/{object_id}/config
 * @note QoS = 1, Retain = 1 (сохранять конфигурацию на брокере)
 * 
 * @note Допустимые device_class для Home Assistant:
 *       - "pH" - для кислотности
 *       - "temperature" - для температуры
 *       - "humidity" - для влажности
 *       - "conductivity" - для проводимости/TDS
 *       - "" - пустое значение (без класса)
 */
static void send_discovery_sensor(esp_mqtt_client_handle_t client, const char *object_id,
                                  const char *name, const char *unit, const char *device_class,
                                  const char *state_topic)
{
    char topic[128];   ///< Буфер для MQTT топика
    char payload[1024]; ///< Буфер для JSON payload

    // Формируем топик конфигурации для Home Assistant
    snprintf(topic, sizeof(topic), "homeassistant/sensor/hydroesp32/%s/config", object_id);
    
    // Проверяем, есть ли device_class - если пустой, не включаем в JSON
    if (device_class && device_class[0] != '\0') {
        // С device_class
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"%s\","           // Отображаемое имя
                 "\"state_topic\":\"%s\","     // Топик для получения значений
                 "\"unit_of_measurement\":\"%s\"," // Единица измерения
                 "\"device_class\":\"%s\","    // Класс устройства
                 "\"unique_id\":\"esp32_hydro_%s\"," // Уникальный ID
                 "\"device\":{"                // Информация об устройстве
                 "\"identifiers\":[\"esp32_hydro_controller\"]," // ID устройства
                 "\"name\":\"Hydro Controller\","
                 "\"model\":\"ESP32\","
                 "\"manufacturer\":\"HydroNFT\""
                 "}}",
                 name, state_topic, unit ? unit : "", device_class, object_id);
    } else {
        // Без device_class (пустое значение)
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"%s\","
                 "\"state_topic\":\"%s\","
                 "\"unit_of_measurement\":\"%s\","
                 "\"unique_id\":\"esp32_hydro_%s\","
                 "\"device\":{"
                 "\"identifiers\":[\"esp32_hydro_controller\"],"
                 "\"name\":\"Hydro Controller\","
                 "\"model\":\"ESP32\","
                 "\"manufacturer\":\"HydroNFT\""
                 "}}",
                 name, state_topic, unit ? unit : "", object_id);
    }

    // Публикуем конфигурацию с QoS=1 и retain=1
    // retain=1 означает, что брокер сохранит сообщение для новых подписчиков
    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Sensor discovery sent for %s", object_id);
}

/**
 * @brief Отправка конфигурации выключателя для Home Assistant Discovery
 * 
 * Аналогично send_discovery_sensor, но для выключателей (switch).
 * Выключатели имеют два топика:
 * - command_topic - для получения команд от HA
 * - state_topic - для отправки текущего состояния обратно в HA
 * 
 * @param client Дескриптор MQTT клиента
 * @param object_id Уникальный идентификатор (например: "pump", "light")
 * @param name Отображаемое имя в HA
 * @param command_topic Топик для получения команд (HA → устройство)
 * @param state_topic Топик для отправки состояния (устройство → HA)
 * 
 * @note payload_on="ON", payload_off="OFF" - стандартные значения
 */
static void send_discovery_switch(esp_mqtt_client_handle_t client, const char *object_id,
                                  const char *name, const char *command_topic, const char *state_topic)
{
    char topic[128];
    char payload[1024];

    // Топик конфигурации выключателя
    snprintf(topic, sizeof(topic), "homeassistant/switch/hydroesp32/%s/config", object_id);
    
    // JSON конфигурации выключателя
    snprintf(payload, sizeof(payload),
             "{\"name\":\"%s\","
             "\"command_topic\":\"%s\","    // Топик для команд
             "\"state_topic\":\"%s\","      // Топик для состояния
             "\"payload_on\":\"ON\","       // Команда включения
             "\"payload_off\":\"OFF\","     // Команда выключения
             "\"unique_id\":\"esp32_hydro_%s\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydro Controller\","
             "\"model\":\"ESP32\","
             "\"manufacturer\":\"HydroNFT\""
             "}}",
             name, command_topic, state_topic, object_id);

    int msg_id = esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Switch discovery sent for %s, msg_id=%d", object_id, msg_id);
}

/**
 * @brief Отправка конфигурации кнопки для Home Assistant Discovery
 * 
 * Кнопки используются для одноразовых действий (например, запуск OTA).
 * В отличие от switch, кнопки не имеют состояния - только команда.
 * 
 * @param client Дескриптор MQTT клиента
 * @param object_id Уникальный идентификатор (например: "ota_update")
 * @param name Отображаемое имя в HA
 * @param command_topic Топик для получения команд
 * @param payload Значение, отправляемое при нажатии кнопки
 * 
 * @note Кнопки не имеют state_topic - только command_topic
 */
static void send_discovery_button(esp_mqtt_client_handle_t client, const char *object_id,
                                  const char *name, const char *command_topic, const char *payload)
{
    char topic[128];
    char payload_json[1024];

    // Топик конфигурации кнопки
    snprintf(topic, sizeof(topic), "homeassistant/button/hydroesp32/%s/config", object_id);
    
    // JSON конфигурации кнопки
    snprintf(payload_json, sizeof(payload_json),
             "{\"name\":\"%s\","
             "\"command_topic\":\"%s\","
             "\"payload_press\":\"%s\","
             "\"unique_id\":\"esp32_hydro_%s\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydro Controller\","
             "\"model\":\"ESP32\","
             "\"manufacturer\":\"HydroNFT\""
             "}}",
             name, command_topic, payload, object_id);

    int msg_id = esp_mqtt_client_publish(client, topic, payload_json, 0, 1, 1);
    ESP_LOGI(TAG, "Button discovery sent for %s, msg_id=%d", object_id, msg_id);
}

/**
 * @brief Отправка конфигурации сенсора версии прошивки для Home Assistant Discovery
 * 
 * @param client Дескриптор MQTT клиента
 */
static void send_discovery_firmware_version(esp_mqtt_client_handle_t client)
{
    char topic[128];
    char payload[1024];

    snprintf(topic, sizeof(topic), "homeassistant/sensor/hydroesp32/firmware_version/config");
    
    snprintf(payload, sizeof(payload),
             "{\"name\":\"Firmware Version\","
             "\"state_topic\":\"hydro/firmware/version\","
             "\"unique_id\":\"esp32_hydro_firmware_version\","
             "\"icon\":\"mdi:chip\","
             "\"entity_category\":\"diagnostic\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydro Controller\","
             "\"model\":\"ESP32\","
             "\"manufacturer\":\"HydroNFT\""
             "}}");

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Firmware version discovery sent");
}

/**
 * @brief Отправка конфигурации сенсора прогресса OTA для Home Assistant Discovery
 * 
 * @param client Дескриптор MQTT клиента
 */
static void send_discovery_ota_progress(esp_mqtt_client_handle_t client)
{
    char topic[128];
    char payload[1024];

    snprintf(topic, sizeof(topic), "homeassistant/sensor/hydroesp32/ota_progress/config");
    
    snprintf(payload, sizeof(payload),
             "{\"name\":\"OTA Progress\","
             "\"state_topic\":\"hydro/ota/progress\","
             "\"unique_id\":\"esp32_hydro_ota_progress\","
             "\"unit_of_measurement\":\"%%\","
             "\"icon\":\"mdi:progress-download\","
             "\"value_template\":\"{{ value_json.progress }}\","
             "\"entity_category\":\"diagnostic\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydro Controller\","
             "\"model\":\"ESP32\","
             "\"manufacturer\":\"HydroNFT\""
             "}}");

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "OTA progress discovery sent");
}

/**
 * @brief Отправка конфигурации сенсора статуса OTA для Home Assistant Discovery
 * 
 * @param client Дескриптор MQTT клиента
 */
static void send_discovery_ota_status(esp_mqtt_client_handle_t client)
{
    char topic[128];
    char payload[1024];

    snprintf(topic, sizeof(topic), "homeassistant/sensor/hydroesp32/ota_status/config");
    
    snprintf(payload, sizeof(payload),
             "{\"name\":\"OTA Status\","
             "\"state_topic\":\"hydro/ota/status\","
             "\"unique_id\":\"esp32_hydro_ota_status\","
             "\"icon\":\"mdi:state-machine\","
             "\"entity_category\":\"diagnostic\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydro Controller\","
             "\"model\":\"ESP32\","
             "\"manufacturer\":\"HydroNFT\""
             "}}");

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "OTA status discovery sent");
}

/**
 * @brief Публикация состояния выключателя
 * 
 * Отправляет текущее состояние выключателя (ON/OFF) в MQTT топик.
 * Используется для синхронизации состояния между устройством и HA.
 * 
 * @param state_topic Топик для публикации состояния
 * @param state Состояние: true = ON, false = OFF
 * 
 * @note retain=1 - сохранять последнее состояние на брокере
 */
static void publish_switch_state(const char *state_topic, bool state)
{
    const char *payload = state ? "ON" : "OFF";
    esp_mqtt_client_publish(mqtt_client, state_topic, payload, 0, 1, 0);
    ESP_LOGI(TAG, "Published switch state %s to %s", payload, state_topic);
}

// ============================================================================
// ОБРАБОТЧИК СОБЫТИЙ MQTT
// ============================================================================

/**
 * @brief Главный обработчик событий MQTT клиента
 * 
 * Эта функция вызывается автоматически при наступлении любого MQTT события:
 * - Подключение/отключение от брокера
 * - Получение данных из подписанных топиков
 * - Ошибки соединения
 * 
 * @param handler_args Аргументы обработчика (не используются)
 * @param base Базовый тип события (ESP_EVENT)
 * @param event_id ID события (MQTT_EVENT_*)
 * @param event_data Данные события (esp_mqtt_event_handle_t)
 * 
 * @note Обработчик выполняется в контексте задачи MQTT клиента
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;  // Приводим к типу MQTT события
    (void)handler_args;
    (void)base;

    // Обрабатываем различные типы событий
    switch (event->event_id) {
        
        // ================================================================
        // СОБЫТИЕ: УСПЕШНОЕ ПОДКЛЮЧЕНИЕ К БРОКЕРУ
        // ================================================================
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            mqtt_connected = true;

            // Отправляем конфигурации для Home Assistant Discovery
            // После этого HA автоматически обнаружит наши сенсоры и выключатели

            // Сенсоры
            // pH - используем device_class "pH"
            send_discovery_sensor(mqtt_client, "ph", "pH Level", "", "pH", "hydro/sensor/ph/state");
            // TDS - используем device_class "conductivity" (проводимость)
            send_discovery_sensor(mqtt_client, "tds", "TDS", "ppm", "conductivity", "hydro/sensor/tds/state");
            // Уровень воды - без device_class (пустая строка)
            send_discovery_sensor(mqtt_client, "water_level", "Water Level", "%", "", "hydro/sensor/level/state");
            // Температура (DHT) - используем device_class "temperature"
            send_discovery_sensor(mqtt_client, "temperature", "Temperature", "°C", "temperature", "hydro/sensor/temperature/state");
            // Влажность (DHT) - используем device_class "humidity"
            send_discovery_sensor(mqtt_client, "humidity", "Humidity", "%", "humidity", "hydro/sensor/humidity/state");
            
            // Сенсоры OTA
            // Версия прошивки
            send_discovery_firmware_version(mqtt_client);
            // Прогресс OTA
            send_discovery_ota_progress(mqtt_client);
            // Статус OTA
            send_discovery_ota_status(mqtt_client);

            // Выключатели
            send_discovery_switch(mqtt_client, "pump", "Circulation Pump",
                                  "hydro/switch/pump/set", "hydro/switch/pump/state");
            send_discovery_switch(mqtt_client, "light", "Grow Light",
                                  "hydro/switch/light/set", "hydro/switch/light/state");
            send_discovery_switch(mqtt_client, "valve", "Water Valve",
                                  "hydro/switch/valve/set", "hydro/switch/valve/state");

            // Кнопка OTA обновления
            send_discovery_button(mqtt_client, "ota_update", "Start OTA Update",
                                  "hydro/ota/update", "START");

            // Подписываемся на топики команд
            // QoS=1 гарантирует доставку сообщений
            esp_mqtt_client_subscribe(mqtt_client, "hydro/switch/pump/set", 1);
            esp_mqtt_client_subscribe(mqtt_client, "hydro/switch/light/set", 1);
            esp_mqtt_client_subscribe(mqtt_client, "hydro/switch/valve/set", 1);

            // Подписка на топик OTA обновлений
            esp_mqtt_client_subscribe(mqtt_client, "hydro/ota/update", 1);

            // Публикуем текущее состояние выключателей для синхронизации с HA
            publish_switch_state("hydro/switch/pump/state", pump_state);
            publish_switch_state("hydro/switch/light/state", light_state);
            publish_switch_state("hydro/switch/valve/state", valve_state);
            
            // Публикуем статус OTA
            mqtt_client_publish_ota_status("idle");
            break;

        // ================================================================
        // СОБЫТИЕ: ОТКЛЮЧЕНИЕ ОТ БРОКЕРА
        // ================================================================
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected");
            mqtt_connected = false;
            break;

        // ================================================================
        // СОБЫТИЕ: ПОЛУЧЕНЫ ДАННЫЕ ИЗ ПОДПИСАННОГО ТОПИКА
        // ================================================================
        case MQTT_EVENT_DATA: {
            char topic[64];   // Буфер для имени топика
            char data[64];    // Буфер для данных сообщения
            
            // Копируем данные события в строки (с учётом длины, без null-терминатора)
            snprintf(topic, sizeof(topic), "%.*s", event->topic_len, event->topic);
            snprintf(data, sizeof(data), "%.*s", event->data_len, event->data);

            ESP_LOGI(TAG, "Received on %s: %s", topic, data);

            // -------------------------------------------------------------
            // Обработка команды для насоса
            // Топик: hydro/switch/pump/set
            // Данные: "ON" или "OFF"
            // -------------------------------------------------------------
            if (strcmp(topic, "hydro/switch/pump/set") == 0) {
                if (strcmp(data, "ON") == 0) {
                    pump_state = true;
                    ESP_LOGI(TAG, "Pump: ON");
                    // Здесь будет реальное включение насоса через GPIO
                    // gpio_set_level(PUMP_PIN, 1);
                } else if (strcmp(data, "OFF") == 0) {
                    pump_state = false;
                    ESP_LOGI(TAG, "Pump: OFF");
                    // gpio_set_level(PUMP_PIN, 0);
                }
                // Отправляем подтверждение состояния обратно в HA
                publish_switch_state("hydro/switch/pump/state", pump_state);
            }
            
            // -------------------------------------------------------------
            // Обработка команды для света
            // Топик: hydro/switch/light/set
            // Данные: "ON" или "OFF"
            // -------------------------------------------------------------
            else if (strcmp(topic, "hydro/switch/light/set") == 0) {
                if (strcmp(data, "ON") == 0) {
                    light_state = true;
                    ESP_LOGI(TAG, "Light: ON");
                    // gpio_set_level(LIGHT_PIN, 1);
                } else if (strcmp(data, "OFF") == 0) {
                    light_state = false;
                    ESP_LOGI(TAG, "Light: OFF");
                    // gpio_set_level(LIGHT_PIN, 0);
                }
                publish_switch_state("hydro/switch/light/state", light_state);
            }

            // -------------------------------------------------------------
            // Обработка команды для клапана
            // Топик: hydro/switch/valve/set
            // Данные: "ON" или "OFF"
            // -------------------------------------------------------------
            else if (strcmp(topic, "hydro/switch/valve/set") == 0) {
                if (strcmp(data, "ON") == 0) {
                    valve_state = true;
                    ESP_LOGI(TAG, "Valve: ON");
                    set_valve_state(true);  // Открываем клапан
                } else if (strcmp(data, "OFF") == 0) {
                    valve_state = false;
                    ESP_LOGI(TAG, "Valve: OFF");
                    set_valve_state(false);  // Закрываем клапан
                }
                publish_switch_state("hydro/switch/valve/state", valve_state);
            }

            // -------------------------------------------------------------
            // Обработка команды OTA обновления
            // Топик: hydro/ota/update
            // Данные: "START"
            // -------------------------------------------------------------
            else if (strcmp(topic, "hydro/ota/update") == 0) {
                if (strcmp(data, "START") == 0) {
                    ota_update_requested = true;  // Устанавливаем флаг для main.c
                    ESP_LOGI(TAG, "Получена команда OTA START!");
                    mqtt_client_publish_ota_status("started");  // Сообщаем о начале
                }
            }
            break;
        }

        // Остальные события игнорируем
        default:
            break;
    }
}

// ============================================================================
// ПУБЛИЧНЫЕ ФУНКЦИИ КОМПОНЕНТА
// ============================================================================

/**
 * @brief Инициализация и запуск MQTT клиента
 * 
 * Создаёт MQTT клиента, регистрирует обработчик событий и подключается к брокеру.
 * Вызывается один раз в app_main().
 * 
 * Конфигурация подключения:
 * - Broker URI: mqtt://192.168.0.107:1883
 * - Username: hydroesp32
 * - Password: asda
 * 
 * @note Функция не блокирующая - подключение происходит в фоне
 * @note Проверка mqtt_client_is_connected() покажет результат подключения
 */
void mqtt_client_init(void)
{
    ESP_LOGI(TAG, "Initializing MQTT client");

    // Конфигурация MQTT клиента
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.0.107:1883",  // Адрес брокера
        .credentials.username = "hydroesp32",               // Имя пользователя
        .credentials.authentication.password = "asda",      // Пароль
    };

    // Инициализация клиента
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    
    // Регистрация обработчика событий на все события (ESP_EVENT_ANY_ID)
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    // Запуск клиента (начнётся подключение к брокеру)
    esp_mqtt_client_start(mqtt_client);
}

/**
 * @brief Проверка состояния подключения к MQTT
 * @return true если подключено к брокеру
 * @return false если не подключено
 * 
 * @note Используйте в циклах задач для проверки перед публикацией
 */
bool mqtt_client_is_connected(void)
{
    return mqtt_connected;
}

/**
 * @brief Получение флага запроса OTA обновления
 * @return true если получена команда "START" из топика hydro/ota/update
 * @return false если команда не получена
 * 
 * @note Вызывается в mqtt_ota_check_task() в main.c
 * @note После получения флага необходимо вызвать mqtt_client_reset_ota_flag()
 */
bool mqtt_client_get_ota_flag(void)
{
    return ota_update_requested;
}

/**
 * @brief Сброс флага OTA обновления
 * 
 * Вызывается после обработки флага в main.c для предотвращения
 * повторного запуска OTA.
 */
void mqtt_client_reset_ota_flag(void)
{
    ota_update_requested = false;
}

/**
 * @brief Публикация статуса OTA обновления
 * 
 * Отправляет текущий статус процесса OTA в топик hydro/ota/status
 * для информирования пользователя через HA.
 * 
 * @param status Строка статуса:
 *               - "started" - обновление началось
 *               - "downloading" - загрузка прошивки
 *               - "completed" - успешно завершено
 *               - "failed" - ошибка обновления
 */
void mqtt_client_publish_ota_status(const char *status)
{
    if (mqtt_connected && mqtt_client) {
        esp_mqtt_client_publish(mqtt_client, "hydro/ota/status", status, 0, 0, 0);
        ESP_LOGI(TAG, "Published OTA status: %s", status);
    }
}

/**
 * @brief Публикация данных сенсоров в MQTT
 * 
 * Читает значения с датчиков и публикует их в соответствующие топики:
 * - hydro/sensor/ph/state - pH значение
 * - hydro/sensor/tds/state - TDS значение (ppm)
 * - hydro/sensor/level/state - Уровень воды (%)
 * 
 * @note Вызывается периодически из mqtt_sensor_task() в main.c
 * @note Рекомендуется интервал вызова: 5-10 секунд
 * @note ЗАМЕНИТЕ заглушки read_*() на реальные функции чтения с ADC
 */
void mqtt_client_publish_sensor_data(void)
{
    if (mqtt_connected && mqtt_client) {
        char msg[64];  // Буфер для строкового представления значения

        // Публикация pH
        snprintf(msg, sizeof(msg), "%.2f", read_ph());
        esp_mqtt_client_publish(mqtt_client, "hydro/sensor/ph/state", msg, 0, 0, 0);

        // Публикация TDS
        snprintf(msg, sizeof(msg), "%d", read_tds());
        esp_mqtt_client_publish(mqtt_client, "hydro/sensor/tds/state", msg, 0, 0, 0);

        // Публикация уровня воды
        snprintf(msg, sizeof(msg), "%d", read_level());
        esp_mqtt_client_publish(mqtt_client, "hydro/sensor/level/state", msg, 0, 0, 0);

        // Публикация данных DHT (температура и влажность)
        float temp = get_dht_temperature();
        float hum = get_dht_humidity();
        
        if (temp != 0.0f || hum != 0.0f) {
            // Температура
            snprintf(msg, sizeof(msg), "%.1f", temp);
            esp_mqtt_client_publish(mqtt_client, "hydro/sensor/temperature/state", msg, 0, 0, 0);
            
            // Влажность
            snprintf(msg, sizeof(msg), "%.1f", hum);
            esp_mqtt_client_publish(mqtt_client, "hydro/sensor/humidity/state", msg, 0, 0, 0);
        }
    }
}

// ============================================================================
// ПУБЛИКАЦИЯ ВРЕМЕНИ В MQTT - ОТКЛЮЧЕНА
// ============================================================================
// Эта функция больше не используется - время синхронизируется через SNTP,
// но не публикуется в MQTT.
//

// ============================================================================
// ПУБЛИКАЦИЯ ПРОГРЕССА OTA
// ============================================================================

/**
 * @brief Публикация прогресса OTA обновления в MQTT
 * 
 * Отправляет текущий прогресс загрузки прошивки в топик hydro/ota/progress
 * в формате JSON.
 * 
 * @param progress Прогресс в процентах (0-100)
 * @param downloaded Количество загруженных байт
 * @param total Общий размер файла в байтах
 * 
 * @note Топик: hydro/ota/progress
 * @note Формат: {"progress":50,"downloaded":493752,"total":987504}
 */
void mqtt_client_publish_ota_progress(int progress, int downloaded, int total)
{
    if (mqtt_connected && mqtt_client) {
        char json_msg[128];
        snprintf(json_msg, sizeof(json_msg),
                "{\"progress\":%d,\"downloaded\":%d,\"total\":%d}",
                progress, downloaded, total);
        
        esp_mqtt_client_publish(mqtt_client, "hydro/ota/progress", json_msg, 0, 1, 0);
    }
}

/**
 * @brief Публикация версии прошивки в MQTT
 * 
 * Отправляет текущую версию прошивки в топик hydro/firmware/version
 * 
 * @param version Строка версии прошивки
 * 
 * @note Топик: hydro/firmware/version
 * @note Пример: "0.2"
 */
void mqtt_client_publish_firmware_version(const char *version)
{
    if (mqtt_connected && mqtt_client && version) {
        esp_mqtt_client_publish(mqtt_client, "hydro/firmware/version", version, 0, 1, 1);
        ESP_LOGI(TAG, "Published firmware version: %s", version);
    }
}
// /**
//  * @brief Публикация текущего времени в MQTT
//  *
//  * Отправляет строку с текущим временем в топик hydro/system/time
//  * для отображения в Home Assistant.
//  *
//  * @param time_str Строка времени в формате "YYYY-MM-DD HH:MM:SS"
//  *
//  * @note QoS=1 (гарантированная доставка), retain=0 (не сохранять)
//  */
/*
void mqtt_client_publish_time(const char *time_str)
{
    if (mqtt_connected && mqtt_client) {
        esp_mqtt_client_publish(mqtt_client, "hydro/system/time", time_str, 0, 1, 0);
    }
}
*/
