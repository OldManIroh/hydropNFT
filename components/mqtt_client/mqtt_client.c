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
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "mqtt_client.h"           // Заголовочный файл ESP-IDF MQTT библиотеки
#include "hydro_mqtt_client.h"     // Локальный заголовочный файл компонента
#include "esp_ota_ops.h"           // Для получения версии прошивки
#include "esp_mac.h"               // Для получения MAC адреса
#include "main.h"                  // Для функций управления GPIO и получения данных DHT
#include "ads1115.h"               // Для получения данных с ADS1115

/// Тег для системы логирования ESP-IDF
static const char *TAG = "mqtt_client";

/// Максимальное напряжение TDS датчика при 1000 ppm
static const float TDS_MAX_VOLTAGE_V = 2.3f;

/// Дескриптор MQTT клиента - используется для всех операций с MQTT
static esp_mqtt_client_handle_t mqtt_client;

/// Флаг состояния подключения к MQTT брокеру
static _Atomic bool mqtt_connected = false;

/// Флаг запроса OTA обновления - устанавливается при получении команды из MQTT
/// Используется в main.c для запуска процесса обновления прошивки
static _Atomic bool ota_update_requested = false;

/// @brief Состояния выключателей
/// @{
static _Atomic bool pump_state = false;   ///< Насос циркуляции - выключен по умолчанию
static _Atomic bool light_state = false;  ///< Фитолампа - выключена по умолчанию
static _Atomic bool valve_state = false;  ///< Клапан - выключен по умолчанию
/// @}


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
                 "\"name\":\"Hydroponic system\","
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
                 "\"name\":\"Hydroponic system\","
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
             "\"name\":\"Hydroponic system\","
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
             "\"device_class\":\"restart\","
             "\"entity_category\":\"diagnostic\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydroponic system\","
             "\"model\":\"ESP32\","
             "\"manufacturer\":\"HydroNFT\""
             "}}",
             name, command_topic, payload, object_id);

    // WDT-1: esp_mqtt_client_publish может блокироваться - не используем в критичных местах
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
             "{\"name\":\"Версия прошивки\","
             "\"state_topic\":\"hydro/firmware/version\","
             "\"unique_id\":\"esp32_hydro_firmware_version\","
             "\"icon\":\"mdi:chip\","
             "\"entity_category\":\"diagnostic\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydroponic system\","
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
             "{\"name\":\"Прогресс обновления\","
             "\"state_topic\":\"hydro/ota/progress\","
             "\"unique_id\":\"esp32_hydro_ota_progress\","
             "\"unit_of_measurement\":\"%%\","
             "\"icon\":\"mdi:progress-download\","
             "\"value_template\":\"{{ value_json.progress }}\","
             "\"entity_category\":\"diagnostic\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydroponic system\","
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
             "{\"name\":\"Статус обновления\","
             "\"state_topic\":\"hydro/ota/status\","
             "\"unique_id\":\"esp32_hydro_ota_status\","
             "\"icon\":\"mdi:state-machine\","
             "\"entity_category\":\"diagnostic\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydroponic system\","
             "\"model\":\"ESP32\","
             "\"manufacturer\":\"HydroNFT\""
             "}}");

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "OTA status discovery sent");
}

/**
 * @brief Отправка конфигурации сенсора MAC адреса для Home Assistant Discovery
 * 
 * @param client Дескриптор MQTT клиента
 */
static void send_discovery_mac_address(esp_mqtt_client_handle_t client)
{
    char topic[128];
    char payload[1024];

    snprintf(topic, sizeof(topic), "homeassistant/sensor/hydroesp32/mac_address/config");
    
    snprintf(payload, sizeof(payload),
             "{\"name\":\"MAC адрес\","
             "\"state_topic\":\"hydro/system/mac_address\","
             "\"unique_id\":\"esp32_hydro_mac_address\","
             "\"icon\":\"mdi:network\","
             "\"entity_category\":\"diagnostic\","
             "\"device\":{"
             "\"identifiers\":[\"esp32_hydro_controller\"],"
             "\"name\":\"Hydroponic system\","
             "\"model\":\"ESP32\","
             "\"manufacturer\":\"HydroNFT\""
             "}}");

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "MAC address discovery sent");
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

            // === Сенсоры (в порядке отображения в HA) ===
            // 1. Уровень воды
            send_discovery_sensor(mqtt_client, "water_level", "Уровень воды", "%", "", "hydro/sensor/level/state");
            // 2. Уровень pH
            send_discovery_sensor(mqtt_client, "ph", "Уровень pH", "", "pH", "hydro/sensor/ph/state");
            // 3. Температура воды (ADS1115 канал 1)
            send_discovery_sensor(mqtt_client, "water_temp", "Температура воды", "°C", "temperature", "hydro/sensor/water_temp/state");
            // 4. Солесодержание (TDS)
            send_discovery_sensor(mqtt_client, "tds", "Солесодержание (TDS)", "ppm", "", "hydro/sensor/tds/state");
            // 5. Влажность (DHT)
            send_discovery_sensor(mqtt_client, "humidity", "Влажность", "%", "humidity", "hydro/sensor/humidity/state");
            // 6. Температура помещения (DHT)
            send_discovery_sensor(mqtt_client, "temperature", "Температура помещения", "°C", "temperature", "hydro/sensor/temperature/state");

            // === Выключатели ===
            // 7. Насос циркуляции
            send_discovery_switch(mqtt_client, "pump", "Насос циркуляции",
                                  "hydro/switch/pump/set", "hydro/switch/pump/state");
            // 8. Фитолампа
            send_discovery_switch(mqtt_client, "light", "Фитолампа",
                                  "hydro/switch/light/set", "hydro/switch/light/state");
            // 9. Клапан воды
            send_discovery_switch(mqtt_client, "valve", "Клапан воды",
                                  "hydro/switch/valve/set", "hydro/switch/valve/state");

            // === Диагностика ===
            // 10. Версия прошивки
            send_discovery_firmware_version(mqtt_client);
            // 11. Статус обновления
            send_discovery_ota_status(mqtt_client);
            // 12. Прогресс обновления
            send_discovery_ota_progress(mqtt_client);
            // 13. Кнопка обновления прошивки
            send_discovery_button(mqtt_client, "ota_update", "Обновление прошивки",
                                  "hydro/ota/update", "START");
            // 14. MAC адрес устройства
            send_discovery_mac_address(mqtt_client);

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

            // Публикуем текущую версию прошивки при каждом подключении к MQTT
            // Это гарантирует, что после OTA обновления и перезагрузки
            // новая версия будет отображена в Home Assistant
            {
                esp_app_desc_t app_info;
                if (esp_ota_get_partition_description(esp_ota_get_running_partition(), &app_info) == ESP_OK) {
                    mqtt_client_publish_firmware_version(app_info.version);
                    ESP_LOGI(TAG, "Published firmware version on connect: %s", app_info.version);
                }
            }

            // Публикуем MAC адрес устройства в диагностический раздел
            mqtt_client_publish_mac_address();
            break;

        // ================================================================
        // СОБЫТИЕ: ОТКЛЮЧЕНИЕ ОТ БРОКЕРА
        // ================================================================
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected. ESP-MQTT автоматически пытается переподключиться...");
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
                    ESP_LOGI(TAG, "Pump: ON (ручное управление)");
                    // Включаем насос через GPIO (ручное управление на 5 минут)
                    set_pump_state(true, true);
                } else if (strcmp(data, "OFF") == 0) {
                    pump_state = false;
                    ESP_LOGI(TAG, "Pump: OFF (ручное управление)");
                    set_pump_state(false, true);
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
                    ESP_LOGI(TAG, "Light: ON (ручное управление)");
                    set_light_state(true, true);  // true = ручное управление
                } else if (strcmp(data, "OFF") == 0) {
                    light_state = false;
                    ESP_LOGI(TAG, "Light: OFF (ручное управление)");
                    set_light_state(false, true);  // true = ручное управление
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
                    ESP_LOGI(TAG, "Valve: ON (ручное управление)");
                    set_valve_state(true, true);  // Открываем клапан (ручное управление на 5 минут)
                } else if (strcmp(data, "OFF") == 0) {
                    valve_state = false;
                    ESP_LOGI(TAG, "Valve: OFF (ручное управление)");
                    set_valve_state(false, true);  // Закрываем клапан (ручное управление на 5 минут)
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

        // ================================================================
        // СОБЫТИЕ: ОШИБКА MQTT
        // ================================================================
        case MQTT_EVENT_ERROR:
            if (event->error_handle != NULL) {
                ESP_LOGE(TAG, "MQTT error: type=%d, code=%d", 
                        event->error_handle->error_type, 
                        event->error_handle->connect_return_code);
            } else {
                ESP_LOGE(TAG, "MQTT error: unknown");
            }
            break;

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
 * Конфигурация подключения (через menuconfig):
 * - Broker URI: CONFIG_MQTT_BROKER_URI
 * - Username: CONFIG_MQTT_USERNAME
 * - Password: CONFIG_MQTT_PASSWORD
 *
 * @note Функция не блокирующая - подключение происходит в фоне
 * @note Проверка mqtt_client_is_connected() покажет результат подключения
 */
void mqtt_client_init(void)
{
    ESP_LOGI(TAG, "Initializing MQTT client");

    // Конфигурация MQTT клиента (учётные данные из Kconfig)
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,           // Адрес брокера из menuconfig
        .credentials.username = CONFIG_MQTT_USERNAME,           // Имя пользователя из menuconfig
        .credentials.authentication.password = CONFIG_MQTT_PASSWORD,  // Пароль из menuconfig
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
 * Читает реальные значения напряжений с ADS1115 и публикует в топики:
 * - hydro/sensor/ph/state - Напряжение pH датчика (канал 0), В
 * - hydro/sensor/tds/state - TDS значение (канал 2), ppm
 * - hydro/sensor/level/state - Напряжение датчика уровня (канал 3), В
 * - hydro/sensor/water_temp/state - Напряжение датчика температуры воды (канал 1), В
 *
 * Каналы ADS1115:
 * - 0: pH датчик
 * - 1: Температура воды
 * - 2: TDS датчик
 * - 3: Уровень воды
 *
 * @note Вызывается периодически из mqtt_sensor_task() в main.c
 * @note TDS датчик преобразуется из напряжения в ppm (0-2.3В = 0-1000 ppm)
 */
void mqtt_client_publish_sensor_data(void)
{
    if (mqtt_connected && mqtt_client) {
        char msg[64];  // Буфер для строкового представления значения

        const ads1115_sensor_data_t *sensor_data = ads1115_get_latest_data();

        if (sensor_data->valid) {
            // Публикация напряжения pH датчика (канал 0)
            snprintf(msg, sizeof(msg), "%.4f", sensor_data->voltage[0]);
            esp_mqtt_client_publish(mqtt_client, "hydro/sensor/ph/state", msg, 0, 0, 0);

            // Публикация TDS датчика (канал 2) - преобразование из напряжения в ppm
            // 0-TDS_MAX_VOLTAGE_V = 0-1000 ppm
            float tds_ppm = (sensor_data->voltage[2] / TDS_MAX_VOLTAGE_V) * 1000.0f;
            if (tds_ppm < 0.0f) tds_ppm = 0.0f;
            if (tds_ppm > 1000.0f) tds_ppm = 1000.0f;
            snprintf(msg, sizeof(msg), "%.0f", tds_ppm);
            esp_mqtt_client_publish(mqtt_client, "hydro/sensor/tds/state", msg, 0, 0, 0);

            // Публикация напряжения датчика уровня воды (канал 3)
            snprintf(msg, sizeof(msg), "%.4f", sensor_data->voltage[3]);
            esp_mqtt_client_publish(mqtt_client, "hydro/sensor/level/state", msg, 0, 0, 0);

            // Публикация напряжения датчика температуры воды (канал 1)
            snprintf(msg, sizeof(msg), "%.4f", sensor_data->voltage[1]);
            esp_mqtt_client_publish(mqtt_client, "hydro/sensor/water_temp/state", msg, 0, 0, 0);
        }

        // Публикация данных DHT (температура и влажность)
        if (is_dht_data_valid()) {
            float temp = get_dht_temperature();
            float hum = get_dht_humidity();

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

/**
 * @brief Публикация MAC адреса устройства в MQTT
 * 
 * Отправляет MAC адрес устройства в топик hydro/system/mac_address
 * 
 * @note Топик: hydro/system/mac_address
 * @note Формат: "XX:XX:XX:XX:XX:XX"
 */
void mqtt_client_publish_mac_address(void)
{
    if (mqtt_connected && mqtt_client) {
        uint8_t mac_addr[6];
        char mac_str[18];
        
        // Получаем MAC адрес устройства
        esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
        
        // Форматируем MAC адрес в строку
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        
        esp_mqtt_client_publish(mqtt_client, "hydro/system/mac_address", mac_str, 0, 1, 1);
        ESP_LOGI(TAG, "Published MAC address: %s", mac_str);
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

/**
 * @brief Получить дескриптор MQTT клиента
 */
void *mqtt_client_get_handle(void)
{
    return mqtt_client;
}
