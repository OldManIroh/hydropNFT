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
 * @version 1.0
 * @date 2024
 */

#ifndef HYDRO_MQTT_CLIENT_H
#define HYDRO_MQTT_CLIENT_H

#include <stdbool.h>  // Для типа bool

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
 * - URI: mqtt://192.168.0.105:1883
 * - Username: hydroesp32
 * - Password: asda
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
 * @brief Проверка флага запроса OTA обновления
 * 
 * Возвращает состояние флага, который устанавливается при получении
 * MQTT команды из топика hydro/ota/update со значением "START".
 * 
 * @return true если получена команда на OTA обновление через MQTT
 * @return false если команда не получена
 * 
 * @note Флаг устанавливается в обработчике MQTT событий
 * @note После получения флага необходимо вызвать mqtt_client_reset_ota_flag()
 * 
 * Пример использования в main.c:
 * @code
 * void mqtt_ota_check_task(void *pvParameters) {
 *     while (1) {
 *         if (mqtt_client_is_connected() && mqtt_client_get_ota_flag()) {
 *             mqtt_client_reset_ota_flag();
 *             ota_start_task();  // Запуск OTA обновления
 *         }
 *         vTaskDelay(pdMS_TO_TICKS(1000));
 *     }
 * }
 * @endcode
 * 
 * @see mqtt_client_reset_ota_flag()
 * @see ota_start_task()
 */
bool mqtt_client_get_ota_flag(void);

/**
 * @brief Сброс флага OTA обновления
 * 
 * Сбрасывает флаг ota_update_requested в false.
 * Вызывается после обработки флага для предотвращения
 * повторного запуска OTA обновления.
 * 
 * @note Вызывается в mqtt_ota_check_task() после получения флага
 * 
 * @see mqtt_client_get_ota_flag()
 */
void mqtt_client_reset_ota_flag(void);

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
 * @note В текущей версии используются заглушки read_*()
 * @note ЗАМЕНИТЕ на реальные функции чтения с ADC/ADS1115
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

// Функция mqtt_client_publish_time() отключена - время не публикуется в MQTT

// ============================================================================
// ФУНКЦИИ УПРАВЛЕНИЯ КЛАПАНОМ (объявления в main.c)
// ============================================================================

/**
 * @brief Установить состояние клапана
 * @param state Состояние: true = открыть, false = закрыть
 */
void set_valve_state(bool state);

/**
 * @brief Получить состояние клапана
 * @return true если открыт, false если закрыт
 */
bool get_valve_state(void);

#ifdef __cplusplus
}
#endif

#endif // HYDRO_MQTT_CLIENT_H
