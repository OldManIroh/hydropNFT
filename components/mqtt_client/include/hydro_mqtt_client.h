/**
 * @file hydro_mqtt_client.h
 * @brief Заголовочный файл MQTT клиента для HydroNFT проекта
 * 
 * Этот модуль предоставляет API для работы с MQTT в проекте HydroNFT.
 * 
 * Основные возможности:
 * - Автоматическое подключение к MQTT брокеру
 * - Home Assistant MQTT Discovery (автообнаружение устройств)
 * - Управление выключателями (насос, свет) через MQTT команды
 * - Публикация данных сенсоров (pH, TDS, уровень воды)
 * - Поддержка OTA обновлений через MQTT команду
 * 
 * Пример использования в main.c:
 * @code
 * void app_main(void) {
 *     // Инициализация MQTT клиента
 *     mqtt_client_init();
 *     
 *     // В цикле задачи проверяем подключение и публикуем данные
 *     while (1) {
 *         if (mqtt_client_is_connected()) {
 *             mqtt_client_publish_sensor_data();
 *         }
 *         vTaskDelay(pdMS_TO_TICKS(5000));
 *     }
 * }
 * @endcode
 * 
 * @author HydroNFT Team
 * @version 2.0
 * @date 2026
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация MQTT клиента
 * 
 * Создаёт MQTT клиента, регистрирует обработчик событий
 * и инициирует подключение к брокеру.
 * 
 * Конфигурация брокера (настраивается в mqtt_client.c):
 * - URI: mqtt://<broker_address>:1883
 * - Username: <your_username>
 * - Password: <your_password>
 * 
 * После успешного подключения:
 * 1. Отправляются конфигурации для Home Assistant Discovery
 * 2. Подписывается на топики команд hydro/switch/pump/set и hydro/switch/light/set
 * 3. Публикует текущие состояния выключателей
 * 
 * @note Функция не блокирующая - подключение происходит асинхронно
 * @note Для проверки результата используйте mqtt_client_is_connected()
 * @note Вызывается один раз в app_main()
 * 
 * @see mqtt_client_is_connected()
 * @see mqtt_client_publish_sensor_data()
 */
void mqtt_client_init(void);

/**
 * @brief Проверка состояния подключения к MQTT брокеру
 * 
 * Возвращает текущий статус подключения к MQTT брокеру.
 * 
 * @return true если успешно подключено к брокеру
 * @return false если не подключено или идёт переподключение
 * 
 * @note Используйте перед публикацией данных для избежания ошибок
 * 
 * Пример:
 * @code
 * if (mqtt_client_is_connected()) {
 *     mqtt_client_publish_sensor_data();
 * }
 * @endcode
 */
bool mqtt_client_is_connected(void);

/**
 * @brief Включить/отключить MQTT клиент
 * @param enabled true — включить, false — отключить
 *
 * @note При отключении клиент не разрывает текущее соединение,
 *       но предотвращает автоматическое переподключение
 * @note При повторном включении вызывается mqtt_client_restart()
 */
void mqtt_client_set_enabled(bool enabled);

/**
 * @brief Проверить, включён ли MQTT клиент
 * @return true если MQTT включён (по умолчанию true)
 */
bool mqtt_client_is_enabled(void);

/**
 * @brief Сброс флага OTA обновления
 *
 * Сбрасывает флаг ota_update_requested в false.
 * Вызывается после обработки флага для предотвращения
 * повторного запуска OTA обновления.
 *
 * @note Вызывается в mqtt_ota_check_task() после получения флага
 */
void mqtt_client_reset_ota_flag(void);

/**
 * @brief Проверка, была ли уже запрошена OTA
 * @return true если OTA уже запрошена (даже если ещё не запущена задача)
 */
bool mqtt_client_is_ota_requested(void);

/**
 * @brief Публикация статуса OTA обновления
 * 
 * Отправляет текущий статус процесса OTA в топик hydro/ota/status
 * для информирования пользователя через Home Assistant.
 * 
 * @param status Строка статуса обновления:
 *               - "started" - обновление началось
 *               - "downloading" - загрузка прошивки с сервера
 *               - "writing" - запись прошивки во flash
 *               - "completed" - успешно завершено, перезагрузка
 *               - "failed" - ошибка обновления
 * 
 * @note Статус публикуется только при активном подключении к MQTT
 * @note QoS=0, retain=0 (не сохранять статус на брокере)
 * 
 * Пример:
 * @code
 * mqtt_client_publish_ota_status("started");
 * @endcode
 */
void mqtt_client_publish_ota_status(const char *status);

/**
 * @brief Публикация данных сенсоров в MQTT
 * 
 * Считывает текущие значения с датчиков и публикует их
 * в соответствующие MQTT топики:
 * 
 * | Датчик      | Топик                        | Формат |
 * |-------------|------------------------------|--------|
 * | pH          | hydro/sensor/ph/state        | float  |
 * | TDS         | hydro/sensor/tds/state       | int    |
 * | Уровень воды| hydro/sensor/level/state     | int %  |
 * 
 * @note Функция проверяет подключение перед публикацией
 * @note Рекомендуется вызывать каждые 5-10 секунд
 * @note Данные читаются из ADS1115 (каналы 0-3) и DHT датчика
 * 
 * Пример использования в задаче FreeRTOS:
 * @code
 * void mqtt_sensor_task(void *pvParameters) {
 *     while (1) {
 *         if (mqtt_client_is_connected()) {
 *             mqtt_client_publish_sensor_data();
 *         }
 *         vTaskDelay(pdMS_TO_TICKS(5000));
 *     }
 * }
 * @endcode
 */
void mqtt_client_publish_sensor_data(void);

/**
 * @brief Публикация прогресса OTA обновления в MQTT
 * 
 * @param progress Прогресс в процентах (0-100)
 * @param downloaded Количество загруженных байт
 * @param total Общий размер файла в байтах
 * 
 * @note Топик: hydro/ota/progress
 * @note Формат: {"progress":50,"downloaded":493752,"total":987504}
 */
void mqtt_client_publish_ota_progress(int progress, int downloaded, int total);

/**
 * @brief Публикация версии прошивки в MQTT
 *
 * @param version Строка версии прошивки
 *
 * @note Топик: hydro/firmware/version
 * @note Пример: "0.2"
 */
void mqtt_client_publish_firmware_version(const char *version);

/**
 * @brief Получить дескриптор MQTT клиента
 *
 * Используется внутренними компонентами для прямой публикации
 * в MQTT (например, log_forwarder).
 *
 * @return Дескриптор MQTT клиента
 *
 * @note Используйте с осторожностью - только для внутренней публикации
 */
void *mqtt_client_get_handle(void);

/**
 * @brief Публикация MAC адреса устройства в MQTT
 *
 * Отправляет MAC адрес устройства в топик hydro/system/mac_address
 *
 * @note Топик: hydro/system/mac_address
 * @note Формат: "XX:XX:XX:XX:XX:XX"
 */
void mqtt_client_publish_mac_address(void);

/**
 * @brief Публикация состояния touch-сигнала в MQTT
 *
 * Отправляет состояние touch-датчика (полученное по ESP-NOW)
 * в топик hydro/control/touch.
 *
 * @param state true = касание (верхний уровень), false = нет касания (нижний уровень)
 *
 * @note Топик: hydro/control/touch
 * @note Формат: "true" или "false"
 * @note QoS=0, retain=0
 */
esp_err_t mqtt_client_publish_touch_state(bool state);

/**
 * @brief Перезапуск MQTT клиента с новыми настройками
 *
 * Останавливает текущий MQTT клиент, перечитывает настройки из NVS
 * и создаёт новый клиент с обновлёнными параметрами.
 *
 * @note Вызывается из веб-сервера после сохранения настроек MQTT
 */
void mqtt_client_restart(void);

/**
 * @brief Установить флаг запроса OTA обновления
 *
 * Вызывается из веб-интерфейса для предотвращения гонки
 * между OTA из MQTT и OTA из веб-сервера.
 */
void mqtt_client_set_ota_flag(void);

#ifdef __cplusplus
}
#endif
