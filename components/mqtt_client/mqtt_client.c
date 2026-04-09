/**
 * @file mqtt_client.c
 * @brief MQTT клиент для HydroNFT
 *
 * @par Архитектура
 * - Один esp_mqtt_client с обработчиком событий (MQTT_EVENT_*)
 * - HA Discovery отправляется ОДИН РАЗ при первом подключении (retain=1 на брокере)
 * - При reconnect — только подписки и состояния (не дублируем discovery)
 * - s_discovery_sent сбрасывается при disconnect — HA может потерять конфигурацию
 *   если брокер не сохранил retain=1 сообщения
 *
 * @par Поток OTA из MQTT
 * 1. HA шлёт "START" в hydro/ota/update
 * 2. MQTT handler ставит ota_update_requested = true
 * 3. mqtt_ota_check_task (main.c) проверяет флаг, запускает ota_start_task()
 * 4. OTA задача грузит прошивку, перезагружает устройство
 *
 * @par Безопасность
 * - Буферы uri/user/pass — static (не стек) — esp_mqtt_client_init копирует строки,
 *   но static защищает от будущих изменений ESP-IDF и от use-after-free при restart
 * - Discovery payload проверяется на truncation — не публикуем невалидный JSON
 *
 * @author HydroNFT Team
 * @version 2.0
 * @date 2026
 */

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"           // Заголовочный файл ESP-IDF MQTT библиотеки
#include "hydro_mqtt_client.h"     // Локальный заголовочный файл компонента
#include "esp_ota_ops.h"           // Для получения версии прошивки
#include "esp_mac.h"               // Для получения MAC адреса
#include "device_control.h"        // Для функций управления GPIO и получения данных DHT
#include "ads1115.h"               // Для получения данных с ADS1115
#include "sntp_client.h"           // Для проверки синхронизации времени
#include "log_forwarder.h"         // Для discovery сенсора логов
#include "settings.h"              // Для настроек из NVS

/// Тег для системы логирования ESP-IDF
static const char *TAG = "mqtt_client";

/// Device block для Home Assistant MQTT Discovery
static const char *HA_DEVICE_BLOCK =
    "\"device\":{"
    "\"identifiers\":[\"esp32_hydro_controller\"],"
    "\"name\":\"Hydroponic system\","
    "\"model\":\"ESP32\","
    "\"manufacturer\":\"HydroNFT\""
    "}";

/// Дескриптор MQTT клиента - используется для всех операций с MQTT
static esp_mqtt_client_handle_t mqtt_client;

/// Флаг состояния подключения к MQTT брокеру
static _Atomic bool mqtt_connected = false;

/// Флаг запроса OTA обновления - устанавливается при получении команды из MQTT
/// Используется в main.c для запуска процесса обновления прошивки
static _Atomic bool ota_update_requested = false;

/// Флаг однократной публикации времени синхронизации (после SNTP + MQTT)
static bool s_time_published = false;

/// Флаг однократной отправки HA discovery конфигураций (retain=1, не нужно повторять при reconnect)
static _Atomic bool s_discovery_sent = false;


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
 * @param value_template Jinja2-шаблон для извлечения значения из JSON
 *                       (например: "{{ value_json.ph }}")
 * @param state_topic Топик, в который будут публиковаться значения сенсора
 *
 * @note Формат топика: homeassistant/sensor/{device_name}/{object_id}/config
 * @note QoS = 1, Retain = 1 (сохранять конфигурацию на брокере)
 */
static void send_discovery_sensor(esp_mqtt_client_handle_t client, const char *object_id,
                                  const char *name, const char *unit, const char *device_class,
                                  const char *value_template, const char *state_topic)
{
    char topic[128];   ///< Буфер для MQTT топика
    char payload[512]; ///< Буфер для JSON payload

    // Формируем топик конфигурации для Home Assistant
    snprintf(topic, sizeof(topic), "homeassistant/sensor/hydroesp32/%s/config", object_id);

    // Проверяем, есть ли device_class - если пустой, не включаем в JSON
    if (device_class && device_class[0] != '\0') {
        // С device_class
        int ret = snprintf(payload, sizeof(payload),
                 "{\"name\":\"%s\","
                 "\"state_topic\":\"%s\","
                 "\"unit_of_measurement\":\"%s\","
                 "\"device_class\":\"%s\","
                 "\"unique_id\":\"esp32_hydro_%s\","
                 "\"value_template\":\"%s\","
                 "%s}",
                 name, state_topic, unit ? unit : "", device_class, object_id,
                 value_template, HA_DEVICE_BLOCK);
        if (ret < 0 || (size_t)ret >= sizeof(payload)) {
            ESP_LOGW(TAG, "Обрезан discovery payload для %s (%d/%zu байт) — пропускаю публикацию",
                     object_id, ret, sizeof(payload));
            return;
        }
    } else {
        // Без device_class (пустое значение)
        int ret = snprintf(payload, sizeof(payload),
                 "{\"name\":\"%s\","
                 "\"state_topic\":\"%s\","
                 "\"unit_of_measurement\":\"%s\","
                 "\"unique_id\":\"esp32_hydro_%s\","
                 "\"value_template\":\"%s\","
                 "%s}",
                 name, state_topic, unit ? unit : "", object_id,
                 value_template, HA_DEVICE_BLOCK);
        if (ret < 0 || (size_t)ret >= sizeof(payload)) {
            ESP_LOGW(TAG, "Обрезан discovery payload для %s (%d/%zu байт) — пропускаю публикацию",
                     object_id, ret, sizeof(payload));
            return;  // Не публикуем невалидный JSON
        }
    }

    // Публикуем конфигурацию с QoS=1 и retain=1
    // retain=1 означает, что брокер сохранит сообщение для новых подписчиков
    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Отправлена конфигурация сенсора %s", object_id);
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
    char payload[512];

    snprintf(topic, sizeof(topic), "homeassistant/switch/hydroesp32/%s/config", object_id);

    int ret = snprintf(payload, sizeof(payload),
             "{\"name\":\"%s\","
             "\"command_topic\":\"%s\","
             "\"state_topic\":\"%s\","
             "\"payload_on\":\"ON\","
             "\"payload_off\":\"OFF\","
             "\"unique_id\":\"esp32_hydro_%s\","
             "%s}",
             name, command_topic, state_topic, object_id, HA_DEVICE_BLOCK);
    if (ret < 0 || (size_t)ret >= sizeof(payload)) {
        ESP_LOGW(TAG, "Обрезан discovery payload для switch %s — пропускаю", object_id);
        return;
    }

    int msg_id = esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Отправлена конфигурация выключателя %s, msg_id=%d", object_id, msg_id);
}

/**
 * @brief Отправка конфигурации селектора режима для Home Assistant Discovery
 *
 * Селекторы (select) используются для переключения режимов AUTO/MANUAL.
 *
 * @param client Дескриптор MQTT клиента
 * @param object_id Уникальный идентификатор (например: "pump_mode")
 * @param name Отображаемое имя в HA
 * @param command_topic Топик для получения команд
 * @param state_topic Топик для отправки состояния
 *
 * @note Опции: "auto", "manual"
 */
static void send_discovery_select(esp_mqtt_client_handle_t client, const char *object_id,
                                  const char *name, const char *command_topic, const char *state_topic)
{
    char topic[128];
    char payload[512];

    snprintf(topic, sizeof(topic), "homeassistant/select/hydroesp32/%s/config", object_id);

    int ret = snprintf(payload, sizeof(payload),
             "{\"name\":\"%s\","
             "\"command_topic\":\"%s\","
             "\"state_topic\":\"%s\","
             "\"options\":[\"auto\",\"manual\"],"
             "\"unique_id\":\"esp32_hydro_%s\","
             "\"icon\":\"mdi:cog\","
             "%s}",
             name, command_topic, state_topic, object_id, HA_DEVICE_BLOCK);
    if (ret < 0 || (size_t)ret >= sizeof(payload)) {
        ESP_LOGW(TAG, "Обрезан discovery payload для select %s — пропускаю", object_id);
        return;
    }

    int msg_id = esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Отправлена конфигурация селектора %s, msg_id=%d", object_id, msg_id);
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
    char payload_json[512];

    // Топик конфигурации кнопки
    snprintf(topic, sizeof(topic), "homeassistant/button/hydroesp32/%s/config", object_id);

    // JSON конфигурации кнопки
    int ret = snprintf(payload_json, sizeof(payload_json),
             "{\"name\":\"%s\","
             "\"command_topic\":\"%s\","
             "\"payload_press\":\"%s\","
             "\"unique_id\":\"esp32_hydro_%s\","
             "\"device_class\":\"restart\","
             "\"entity_category\":\"diagnostic\","
             "%s}",
             name, command_topic, payload, object_id, HA_DEVICE_BLOCK);
    if (ret < 0 || (size_t)ret >= sizeof(payload_json)) {
        ESP_LOGW(TAG, "Обрезан discovery payload для кнопки %s — пропускаю", object_id);
        return;
    }

    // WDT-1: esp_mqtt_client_publish может блокироваться - не используем в критичных местах
    int msg_id = esp_mqtt_client_publish(client, topic, payload_json, 0, 1, 1);
    ESP_LOGI(TAG, "Отправлена конфигурация кнопки %s, msg_id=%d", object_id, msg_id);
}

/**
 * @brief Отправка конфигурации сенсора версии прошивки для Home Assistant Discovery
 * 
 * @param client Дескриптор MQTT клиента
 */
static void send_discovery_firmware_version(esp_mqtt_client_handle_t client)
{
    const char *topic = "homeassistant/sensor/hydroesp32/firmware_version/config";
    char payload[512];
    
    snprintf(payload, sizeof(payload),
             "{\"name\":\"Версия прошивки\","
             "\"state_topic\":\"hydro/firmware/version\","
             "\"unique_id\":\"esp32_hydro_firmware_version\","
             "\"icon\":\"mdi:chip\","
             "\"entity_category\":\"diagnostic\","
             "%s}",
             HA_DEVICE_BLOCK);

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Отправлена конфигурация версии прошивки");
}

/**
 * @brief Отправка конфигурации сенсора прогресса OTA для Home Assistant Discovery
 * 
 * @param client Дескриптор MQTT клиента
 */
static void send_discovery_ota_progress(esp_mqtt_client_handle_t client)
{
    const char *topic = "homeassistant/sensor/hydroesp32/ota_progress/config";
    char payload[512];

    snprintf(payload, sizeof(payload),
             "{\"name\":\"Прогресс обновления\","
             "\"state_topic\":\"hydro/ota/progress\","
             "\"unique_id\":\"esp32_hydro_ota_progress\","
             "\"unit_of_measurement\":\"%%\","
             "\"icon\":\"mdi:progress-download\","
             "\"value_template\":\"{{ value_json.progress }}\","
             "\"entity_category\":\"diagnostic\","
             "%s}",
             HA_DEVICE_BLOCK);

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Отправлена конфигурация прогресса OTA");
}

/**
 * @brief Отправка конфигурации сенсора статуса OTA для Home Assistant Discovery
 * 
 * @param client Дескриптор MQTT клиента
 */
static void send_discovery_ota_status(esp_mqtt_client_handle_t client)
{
    const char *topic = "homeassistant/sensor/hydroesp32/ota_status/config";
    char payload[512];

    snprintf(payload, sizeof(payload),
             "{\"name\":\"Статус обновления\","
             "\"state_topic\":\"hydro/ota/status\","
             "\"unique_id\":\"esp32_hydro_ota_status\","
             "\"icon\":\"mdi:state-machine\","
             "\"entity_category\":\"diagnostic\","
             "%s}",
             HA_DEVICE_BLOCK);

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Отправлена конфигурация статуса OTA");
}

/**
 * @brief Отправка конфигурации сенсора MAC адреса для Home Assistant Discovery
 * 
 * @param client Дескриптор MQTT клиента
 */
static void send_discovery_mac_address(esp_mqtt_client_handle_t client)
{
    const char *topic = "homeassistant/sensor/hydroesp32/mac_address/config";
    char payload[512];

    snprintf(payload, sizeof(payload),
             "{\"name\":\"MAC адрес\","
             "\"state_topic\":\"hydro/system/mac_address\","
             "\"unique_id\":\"esp32_hydro_mac_address\","
             "\"icon\":\"mdi:network\","
             "\"entity_category\":\"diagnostic\","
             "%s}",
             HA_DEVICE_BLOCK);

    esp_mqtt_client_publish(client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "Отправлена конфигурация MAC адреса");
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
    if (!atomic_load(&mqtt_connected) || !mqtt_client) {
        ESP_LOGW(TAG, "Публикация состояния %s пропущена: MQTT не подключён", state_topic);
        return;
    }
    const char *payload = state ? "ON" : "OFF";
    int msg_id = esp_mqtt_client_publish(mqtt_client, state_topic, payload, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Ошибка публикации состояния %s (msg_id=%d)", state_topic, msg_id);
    } else {
        ESP_LOGD(TAG, "Опубликовано состояние: %s → %s", state_topic, payload);
    }
}

// ============================================================================
// CALLBACK ИЗМЕНЕНИЯ СОСТОЯНИЯ GPIO
// ============================================================================

/**
 * @brief Callback для публикации состояния GPIO в MQTT
 * Вызывается из device_control при каждом изменении gpio_set_level().
 */
static void on_device_state_changed(const char *topic, bool state)
{
    if (atomic_load(&mqtt_connected) && mqtt_client) {
        publish_switch_state(topic, state);
    }
}

// ============================================================================
// HELPER ДЛЯ ОБРАБОТКИ MQTT КОМАНД УСТРОЙСТВ
// ============================================================================

/**
 * @brief Обработать команду вкл/выкл устройства из MQTT
 *
 * Общая логика для насоса, света и клапана:
 * - ON/OFF → вызов device_control_set_*_state
 * - Переключение в MANUAL режим
 * - Публикация состояния режима
 */
static void handle_switch_command(const char *data, const char *device_name,
                                  void (*set_state)(bool))
{
    if (set_state == NULL) {
        ESP_LOGE(TAG, "handle_switch_command: set_state callback NULL для %s", device_name);
        return;
    }

    if (strcmp(data, "ON") == 0) {
        ESP_LOGI(TAG, "%s: ВКЛ", device_name);
        set_state(true);
    } else if (strcmp(data, "OFF") == 0) {
        ESP_LOGI(TAG, "%s: ВЫКЛ", device_name);
        set_state(false);
    } else {
        return;  // Неизвестная команда — игнорируем
    }

    // Переключаем в MANUAL только если ещё не в нём — избегаем избыточных публикаций и wakeup'ов
    if (device_control_get_mode() != DEVICE_MODE_MANUAL) {
        device_control_set_mode(DEVICE_MODE_MANUAL);
        if (mqtt_client != NULL) {
            esp_mqtt_client_publish(mqtt_client, "hydro/switch/mode/state", "manual", 0, 1, 0);
        }
        device_control_notify_mode_changed();
    }
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
            ESP_LOGI(TAG, "MQTT подключен");
            atomic_store(&mqtt_connected, true);

            // HA Discovery отправляем только один раз (конфигурации с retain=1 сохраняются на брокере)
            if (!atomic_load(&s_discovery_sent)) {
                // === Сенсоры (единый JSON-топик hydro/sensor/data, value_template для извлечения полей) ===
                // 1. Уровень воды
                send_discovery_sensor(mqtt_client, "water_level", "Уровень воды", "%", "",
                        "{{ value_json.level }}", "hydro/sensor/data");
                // 2. Уровень pH
                send_discovery_sensor(mqtt_client, "ph", "Уровень pH", "", "pH",
                        "{{ value_json.ph }}", "hydro/sensor/data");
                // 3. Температура воды
                send_discovery_sensor(mqtt_client, "water_temp", "Температура воды", "°C", "temperature",
                        "{{ value_json.water_temp }}", "hydro/sensor/data");
                // 4. Солесодержание (TDS)
                send_discovery_sensor(mqtt_client, "tds", "Солесодержание (TDS)", "ppm", "",
                        "{{ value_json.tds }}", "hydro/sensor/data");
                // 5. Влажность (DHT)
                send_discovery_sensor(mqtt_client, "humidity", "Влажность", "%", "humidity",
                        "{{ value_json.humidity }}", "hydro/sensor/data");
                // 6. Температура помещения (DHT)
                send_discovery_sensor(mqtt_client, "temperature", "Температура помещения", "°C", "temperature",
                        "{{ value_json.temp }}", "hydro/sensor/data");

                // === Выключатели ===
                send_discovery_switch(mqtt_client, "pump", "Насос циркуляции",
                                      "hydro/switch/pump/set", "hydro/switch/pump/state");
                send_discovery_switch(mqtt_client, "light", "Фитолампа",
                                      "hydro/switch/light/set", "hydro/switch/light/state");
                send_discovery_switch(mqtt_client, "valve", "Клапан воды",
                                      "hydro/switch/valve/set", "hydro/switch/valve/state");

                // === Селектор режима (AUTO/MANUAL) — общий для всех устройств ===
                send_discovery_select(mqtt_client, "system_mode", "Режим системы",
                                      "hydro/switch/mode/set", "hydro/switch/mode/state");

                // === Диагностика ===
                send_discovery_firmware_version(mqtt_client);
                send_discovery_ota_status(mqtt_client);
                send_discovery_ota_progress(mqtt_client);
                send_discovery_button(mqtt_client, "ota_update", "Обновление прошивки",
                                      "hydro/ota/update", "START");
                send_discovery_mac_address(mqtt_client);

                // === Touch-сенсор (верхний уровень воды) ===
                {
                    const char *topic = "homeassistant/binary_sensor/hydroesp32/touch/config";
                    char payload[512];
                    snprintf(payload, sizeof(payload),
                             "{\"name\":\"Верхний уровень воды\","
                             "\"state_topic\":\"hydro/control/touch\","
                             "\"payload_on\":\"true\","
                             "\"payload_off\":\"false\","
                             "\"unique_id\":\"esp32_hydro_touch\","
                             "\"device_class\":\"moisture\","
                             "%s}",
                             HA_DEVICE_BLOCK);
                    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, 1);
                    ESP_LOGI(TAG, "Отправлена конфигурация touch-сенсора");
                }

                atomic_store(&s_discovery_sent, true);
                ESP_LOGI(TAG, "HA Discovery конфигурации отправлены (больше не повторяются при reconnect)");

                // Лог-сенсор из log_forwarder
                log_forwarder_send_discovery();
            }

            // Подписываемся на топики команд (QoS=1) — необходимо при каждом reconnect
            int sub_pump = esp_mqtt_client_subscribe(mqtt_client, "hydro/switch/pump/set", 1);
            int sub_light = esp_mqtt_client_subscribe(mqtt_client, "hydro/switch/light/set", 1);
            int sub_valve = esp_mqtt_client_subscribe(mqtt_client, "hydro/switch/valve/set", 1);
            int sub_mode = esp_mqtt_client_subscribe(mqtt_client, "hydro/switch/mode/set", 1);
            int sub_ota = esp_mqtt_client_subscribe(mqtt_client, "hydro/ota/update", 1);

            if (sub_pump < 0 || sub_light < 0 || sub_valve < 0 || sub_mode < 0 || sub_ota < 0) {
                ESP_LOGE(TAG, "Ошибка подписки на MQTT топики (будет повторено при reconnect): pump=%d, light=%d, valve=%d, mode=%d, ota=%d",
                        sub_pump, sub_light, sub_valve, sub_mode, sub_ota);
                // Не прерываем инициализацию — подписки будут восстановлены при следующем MQTT_EVENT_CONNECTED
            }

            // Публикуем текущий режим работы
            {
                const char *mode_str = device_control_mode_to_string(device_control_get_mode());
                esp_mqtt_client_publish(mqtt_client, "hydro/switch/mode/state", mode_str, 0, 1, 0);
            }

            // Регистрируем callback для автоматической публикации состояния GPIO
            device_control_register_state_cb(on_device_state_changed);

            // Публикуем текущее состояние выключателей (callback не вызовется т.к. GPIO не меняется)
            publish_switch_state("hydro/switch/pump/state", device_control_get_pump_state());
            publish_switch_state("hydro/switch/light/state", device_control_get_light_state());
            publish_switch_state("hydro/switch/valve/state", device_control_get_valve_state());

            // Публикуем статус OTA
            mqtt_client_publish_ota_status("idle");

            // Публикуем текущую версию прошивки при каждом подключении к MQTT
            // Это гарантирует, что после OTA обновления и перезагрузки
            // новая версия будет отображена в Home Assistant
            {
                esp_app_desc_t app_info;
                if (esp_ota_get_partition_description(esp_ota_get_running_partition(), &app_info) == ESP_OK) {
                    mqtt_client_publish_firmware_version(app_info.version);
                    ESP_LOGI(TAG, "Опубликована версия прошивки при подключении: %s", app_info.version);
                } else {
                    ESP_LOGE(TAG, "Ошибка получения версии прошивки");
                }
            }

            // Публикуем MAC адрес устройства в диагностический раздел
            mqtt_client_publish_mac_address();

            // Публикуем локальное время один раз (только после SNTP синхронизации, лог через log_forwarder в MQTT)
            if (!s_time_published && sntp_client_is_synced()) {
                time_t now;
                time(&now);
                struct tm timeinfo;
                localtime_r(&now, &timeinfo);
                char time_buf[64];
                strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
                ESP_LOGW(TAG, "Локальное время (NTP синхронизировано): %s", time_buf);
                s_time_published = true;
            }
            break;

        // ================================================================
        // СОБЫТИЕ: ОТКЛЮЧЕНИЕ ОТ БРОКЕРА
        // ================================================================
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT отключён. Автоматическое переподключение...");
            atomic_store(&mqtt_connected, false);
            // Сбрасываем флаг discovery — при следующем подключении HA может не иметь конфигурации
            atomic_store(&s_discovery_sent, false);
            // Сбрасываем флаг публикации времени — при повторной синхронизации NTP нужно опубликовать
            s_time_published = false;
            break;

        // ================================================================
        // СОБЫТИЕ: ПОЛУЧЕНЫ ДАННЫЕ ИЗ ПОДПИСАННОГО ТОПИКА
        // ================================================================
        case MQTT_EVENT_DATA: {
            char topic[128];  // Буфер для имени топика
            char data[512];   // Буфер для данных сообщения (увеличен для безопасности)

            // Проверяем усечение топика
            if (event->topic_len >= (int)sizeof(topic)) {
                ESP_LOGE(TAG, "Топик слишком длинный (%d байт) — игнорируем сообщение", event->topic_len);
                break;
            }

            // Проверяем усечение payload
            int data_len = event->data_len;
            if (data_len >= (int)sizeof(data)) {
                ESP_LOGW(TAG, "MQTT payload усечён: %d байт (макс %zu) — обрезано", data_len, sizeof(data) - 1);
                data_len = sizeof(data) - 1;
            }

            // Копируем данные события в строки
            snprintf(topic, sizeof(topic), "%.*s", event->topic_len, event->topic);
            snprintf(data, sizeof(data), "%.*s", data_len, event->data);

            // Понижено до DEBUG — при частых MQTT-командах INFO засоряет лог.
            // Для production достаточно логирования в handle_switch_command
            ESP_LOGD(TAG, "Получено в %s: %s", topic, data);

            // -------------------------------------------------------------
            // Обработка команд для устройств (насос, свет, клапан)
            // -------------------------------------------------------------
            if (strcmp(topic, "hydro/switch/pump/set") == 0) {
                handle_switch_command(data, "Насос", device_control_set_pump_state);
            }
            else if (strcmp(topic, "hydro/switch/light/set") == 0) {
                handle_switch_command(data, "Свет", device_control_set_light_state);
            }
            else if (strcmp(topic, "hydro/switch/valve/set") == 0) {
                handle_switch_command(data, "Клапан", device_control_set_valve_state);
            }

            // -------------------------------------------------------------
            // Обработка команды режима системы (AUTO/MANUAL)
            // Топик: hydro/switch/mode/set
            // Данные: "auto" или "manual"
            // -------------------------------------------------------------
            else if (strcmp(topic, "hydro/switch/mode/set") == 0) {
                device_mode_t mode = device_control_mode_from_string(data);
                device_control_set_mode(mode);
                esp_mqtt_client_publish(mqtt_client, "hydro/switch/mode/state", data, 0, 1, 0);
                device_control_notify_mode_changed();
                ESP_LOGI(TAG, "Режим системы: %s", data);
            }

            // -------------------------------------------------------------
            // Обработка команды OTA обновления
            // Топик: hydro/ota/update
            // Данные: "START"
            // -------------------------------------------------------------
            else if (strcmp(topic, "hydro/ota/update") == 0) {
                if (strcmp(data, "START") == 0) {
                    if (atomic_load(&ota_update_requested)) {
                        ESP_LOGW(TAG, "OTA уже запрошена — игнорирую повторную команду START");
                        mqtt_client_publish_ota_status("already_in_progress");
                        break;
                    }
                    atomic_store(&ota_update_requested, true);
                    device_control_notify_ota_flag();
                    ESP_LOGI(TAG, "Получена команда OTA START!");
                    mqtt_client_publish_ota_status("started");
                }
            }
            break;
        }

        // ================================================================
        // СОБЫТИЕ: ОШИБКА MQTT
        // ================================================================
        case MQTT_EVENT_ERROR:
            if (event->error_handle != NULL) {
                ESP_LOGE(TAG, "Ошибка MQTT: тип=%d, код=%d",
                        event->error_handle->error_type,
                        event->error_handle->connect_return_code);
            } else {
                ESP_LOGE(TAG, "Ошибка MQTT: неизвестная");
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
    ESP_LOGI(TAG, "Инициализация MQTT клиента");

    // Берём настройки из NVS, fallback на Kconfig defaults
    // Статические буферы — esp_mqtt_client_init копирует строки внутрь конфигурации клиента,
    // но static защищает от будущих изменений в ESP-IDF (если вдруг перейдут на сохранение указателя)
    // и подавляет предупреждения статических анализаторов о возврате указателей на стек.
    // Переиспользование буферов при mqtt_client_restart() безопасно — старый клиент уже уничтожен
    // к моменту вызова esp_mqtt_client_init() с новыми данными.
    static char uri[SETTINGS_URI_MAX_LEN];
    static char user[SETTINGS_SSID_MAX_LEN];
    static char pass[SETTINGS_PASS_MAX_LEN];

    settings_get_mqtt_uri(uri, sizeof(uri));
    settings_get_mqtt_user(user, sizeof(user));
    settings_get_mqtt_pass(pass, sizeof(pass));

    // Fallback на Kconfig если NVS пуст
    if (uri[0] == '\0') {
        strncpy(uri, CONFIG_MQTT_BROKER_URI, sizeof(uri) - 1);
        uri[sizeof(uri) - 1] = '\0';
    }
    if (user[0] == '\0') {
        strncpy(user, CONFIG_MQTT_USERNAME, sizeof(user) - 1);
        user[sizeof(user) - 1] = '\0';
    }
    if (pass[0] == '\0') {
        strncpy(pass, CONFIG_MQTT_PASSWORD, sizeof(pass) - 1);
        pass[sizeof(pass) - 1] = '\0';
    }

    ESP_LOGI(TAG, "MQTT URI: %s", uri);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = uri,
        .credentials.username = user,
        .credentials.authentication.password = pass,
    };

    // Инициализация клиента
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Не удалось инициализировать MQTT клиент");
        return;
    }

    // Регистрация обработчика событий на все события (ESP_EVENT_ANY_ID)
    esp_err_t err = esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось зарегистрировать MQTT обработчик: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return;
    }

    // Запуск клиента (начнётся подключение к брокеру)
    err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось запустить MQTT клиент: %s", esp_err_to_name(err));
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        return;
    }

    ESP_LOGI(TAG, "MQTT клиент запущен, подключение к брокеру...");
}

/**
 * @brief Перезапуск MQTT клиента с новыми настройками
 *
 * Останавливает текущий клиент, перечитывает настройки из NVS
 * и создаёт новый клиент.
 *
 * @note Вызывается асинхронно — не блокирует вызывающую задачу
 */
void mqtt_client_restart(void)
{
    if (mqtt_client != NULL) {
        ESP_LOGI(TAG, "Запрошен перезапуск MQTT клиента...");
        esp_mqtt_client_stop(mqtt_client);
        esp_mqtt_client_destroy(mqtt_client);
        mqtt_client = NULL;
        atomic_store(&mqtt_connected, false);
    }

    // Сбрасываем флаг discovery — нужно отправить заново
    atomic_store(&s_discovery_sent, false);

    ESP_LOGI(TAG, "Перезапуск MQTT клиента с новыми настройками");
    mqtt_client_init();
}

/**
 * @brief Установить флаг запроса OTA обновления
 */
void mqtt_client_set_ota_flag(void)
{
    atomic_store(&ota_update_requested, true);
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
    return atomic_load(&mqtt_connected);
}

/**
 * @brief Сброс флага OTA обновления
 * 
 * Вызывается после обработки флага в main.c для предотвращения
 * повторного запуска OTA.
 */
void mqtt_client_reset_ota_flag(void)
{
    atomic_store(&ota_update_requested, false);
}

bool mqtt_client_is_ota_requested(void)
{
    return atomic_load(&ota_update_requested);
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
    if (atomic_load(&mqtt_connected) && mqtt_client) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, "hydro/ota/status", status, 0, 1, 0);
        if (msg_id >= 0) {
            ESP_LOGI(TAG, "Опубликован статус OTA: %s (msg_id=%d)", status, msg_id);
        } else {
            ESP_LOGE(TAG, "Ошибка публикации статуса OTA: msg_id=%d", msg_id);
        }
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
    if (!atomic_load(&mqtt_connected) || !mqtt_client) {
        ESP_LOGD(TAG, "Публикация сенсоров пропущена: connected=%d, client=%p", atomic_load(&mqtt_connected), (void *)mqtt_client);
        return;
    }

    char json_msg[512];
    int offset = 0;

    ads1115_sensor_data_t sensor_data = ads1115_get_latest_data();

    // Формируем единый JSON-пакет со всеми сенсорами
    if (sensor_data.valid) {
        float tds_ppm = ads1115_voltage_to_tds(sensor_data.voltage[2]);

        offset += snprintf(json_msg + offset, sizeof(json_msg) - offset,
                "{\"ph\":%.4f,\"tds\":%.0f,\"level\":%.4f,\"water_temp\":%.4f",
                sensor_data.voltage[0], tds_ppm,
                sensor_data.voltage[3], sensor_data.voltage[1]);
    } else {
        offset += snprintf(json_msg + offset, sizeof(json_msg) - offset,
                "{\"ph\":null,\"tds\":null,\"level\":null,\"water_temp\":null");
    }
    // После каждого snprintf проверяем переполнение — если offset >= sizeof(json_msg),
    // то sizeof(json_msg) - offset подтечёт (unsigned wrap-around) и следующий snprintf
    // получит огромный size, что приведёт к записи за пределы буфера.
    if (offset >= sizeof(json_msg)) {
        ESP_LOGE(TAG, "Переполнение JSON буфера сенсоров (offset=%d)", offset);
        return;
    }

    // DHT данные
    if (device_control_is_dht_data_valid()) {
        float temp = device_control_get_dht_temperature();
        float hum = device_control_get_dht_humidity();
        offset += snprintf(json_msg + offset, sizeof(json_msg) - offset,
                ",\"temp\":%.1f,\"humidity\":%.1f}", temp, hum);
    } else {
        offset += snprintf(json_msg + offset, sizeof(json_msg) - offset,
                ",\"temp\":null,\"humidity\":null}");
    }
    if (offset >= sizeof(json_msg)) {
        ESP_LOGE(TAG, "Переполнение JSON буфера сенсоров (DHT, offset=%d)", offset);
        return;
    }

    // Одна публикация вместо 6 — экономия ~83% MQTT-трафика
    int msg_id = esp_mqtt_client_publish(mqtt_client, "hydro/sensor/data", json_msg, 0, 0, 0);
    if (msg_id >= 0) {
        ESP_LOGD(TAG, "Опубликованы сенсоры: %s (msg_id=%d)", json_msg, msg_id);
    } else {
        ESP_LOGE(TAG, "Ошибка публикации сенсоров: msg_id=%d", msg_id);
    }
}

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
    if (atomic_load(&mqtt_connected) && mqtt_client) {
        char json_msg[128];
        int bytes_written = snprintf(json_msg, sizeof(json_msg),
                "{\"progress\":%d,\"downloaded\":%d,\"total\":%d}",
                progress, downloaded, total);

        // bytes_written < 0 — ошибка snprintf (не должно произойти)
        // (size_t)bytes_written >= sizeof(json_msg) — буфер переполнен, JSON невалидный
        if (bytes_written < 0 || (size_t)bytes_written >= sizeof(json_msg)) {
            ESP_LOGE(TAG, "Ошибка формирования JSON прогресса OTA: буфер переполнен (%d байт)", bytes_written);
            return;
        }
        
        int msg_id = esp_mqtt_client_publish(mqtt_client, "hydro/ota/progress", json_msg, 0, 1, 0);
        if (msg_id >= 0) {
            ESP_LOGD(TAG, "Опубликован прогресс OTA: %d%% (%d/%d) (msg_id=%d)", progress, downloaded, total, msg_id);
        } else {
            ESP_LOGE(TAG, "Ошибка публикации прогресса OTA: msg_id=%d", msg_id);
        }
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
    if (atomic_load(&mqtt_connected) && mqtt_client && version) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, "hydro/firmware/version", version, 0, 1, 1);
        if (msg_id >= 0) {
            ESP_LOGI(TAG, "Опубликована версия прошивки: %s (msg_id=%d)", version, msg_id);
        } else {
            ESP_LOGE(TAG, "Ошибка публикации версии прошивки: msg_id=%d", msg_id);
        }
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
    if (atomic_load(&mqtt_connected) && mqtt_client) {
        uint8_t mac_addr[6];
        char mac_str[18];
        
        esp_read_mac(mac_addr, ESP_MAC_WIFI_STA);
        
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
        
        esp_mqtt_client_publish(mqtt_client, "hydro/system/mac_address", mac_str, 0, 1, 1);
        ESP_LOGI(TAG, "Опубликован MAC адрес: %s", mac_str);
    }
}

/**
 * @brief Публикация состояния touch-сенсора (верхний уровень воды)
 *
 * Отправляет состояние внешнего touch-сенсора, полученного по ESP-NOW,
 * в топик hydro/control/touch.
 *
 * @param state true = вода достигла верхнего уровня, false = ниже
 *
 * @note Вызывается из espnow_receiver_task при изменении состояния
 * @note Топик: hydro/control/touch, payload: "true"/"false"
 */
esp_err_t mqtt_client_publish_touch_state(bool state)
{
    if (atomic_load(&mqtt_connected) && mqtt_client) {
        const char *payload = state ? "true" : "false";
        int msg_id = esp_mqtt_client_publish(mqtt_client, "hydro/control/touch", payload, 0, 0, 0);
        if (msg_id >= 0) {
            ESP_LOGI(TAG, "Опубликовано состояние touch: %s (msg_id=%d)", payload, msg_id);
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Ошибка публикации touch-состояния: msg_id=%d", msg_id);
            return ESP_FAIL;
        }
    } else {
        ESP_LOGW(TAG, "Публикация touch-состояния пропущена: connected=%d, client=%p", atomic_load(&mqtt_connected), (void *)mqtt_client);
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief Получить дескриптор MQTT клиента
 */
void *mqtt_client_get_handle(void)
{
    return mqtt_client;
}
