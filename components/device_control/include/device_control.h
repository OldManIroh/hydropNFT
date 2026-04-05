/**
 * @file device_control.h
 * @brief Управление оборудованием HydroNFT
 *
 * Компонент для управления GPIO (насос, свет, клапан),
 * хранения данных DHT, режимов работы (авто/ручной) и остановки ADS1115 для OTA.
 *
 * Режимы работы каждого устройства:
 * - AUTO   — автоматическое управление (расписание, датчики)
 * - MANUAL — ручное управление через MQTT, без ограничений по времени
 *
 * @author HydroNFT Team
 * @version 2.0
 * @date 2026
 */

#pragma once

#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/// @brief GPIO пины для управления оборудованием
/// @{
#define DEVICE_CONTROL_PUMP_PIN   19  ///< Насос циркуляции воды
#define DEVICE_CONTROL_LIGHT_PIN  18  ///< Фитолампа освещения
#define DEVICE_CONTROL_VALVE_PIN  4   ///< Электроклапан слива/подачи
/// @}

/**
 * @brief Режим работы устройства
 */
typedef enum {
    DEVICE_MODE_AUTO,    ///< Автоматическое управление (расписание/датчики)
    DEVICE_MODE_MANUAL,  ///< Ручное управление через MQTT (без таймаута)
} device_mode_t;

/**
 * @brief Инициализация компонента управления оборудованием
 *
 * Настраивает GPIO пины (насос, свет, клапан) как выходы,
 * устанавливает безопасное состояние (LOW), создаёт мьютекс DHT.
 * Все устройства по умолчанию в режиме MANUAL (безопасный режим после перезагрузки).
 *
 * @return ESP_OK при успехе, код ошибки в противном случае
 *
 * @note Вызывается один раз в app_main() перед запуском задач
 * @note Повторный вызов возвращает ESP_OK (защита от дублирования)
 */
esp_err_t device_control_init(void);

// ============================================================================
// УПРАВЛЕНИЕ GPIO
// ============================================================================

/**
 * @brief Установить состояние насоса
 * @param state true = включить, false = выключить
 */
void device_control_set_pump_state(bool state);

/**
 * @brief Установить состояние света
 * @param state true = включить, false = выключить
 */
void device_control_set_light_state(bool state);

/**
 * @brief Установить состояние клапана
 * @param state true = открыть, false = закрыть
 */
void device_control_set_valve_state(bool state);

/**
 * @brief Получить состояние насоса
 * @return true если включен, false если выключен
 */
bool device_control_get_pump_state(void);

/**
 * @brief Получить состояние света
 * @return true если включен, false если выключен
 */
bool device_control_get_light_state(void);

/**
 * @brief Получить состояние клапана
 * @return true если открыт, false если закрыт
 */
bool device_control_get_valve_state(void);

// ============================================================================
// РЕЖИМЫ РАБОТЫ (AUTO / MANUAL) — общий для всех устройств
// ============================================================================

/**
 * @brief Установить режим работы системы
 * @param mode DEVICE_MODE_AUTO или DEVICE_MODE_MANUAL
 */
void device_control_set_mode(device_mode_t mode);

/**
 * @brief Получить текущий режим работы системы
 * @return DEVICE_MODE_AUTO или DEVICE_MODE_MANUAL
 */
device_mode_t device_control_get_mode(void);

/**
 * @brief Преобразовать режим в строку
 * @param mode Режим устройства
 * @return "auto" или "manual"
 */
const char *device_control_mode_to_string(device_mode_t mode);

/**
 * @brief Преобразовать строку в режим
 * @param str "auto" или "manual"
 * @return DEVICE_MODE_AUTO или DEVICE_MODE_MANUAL
 * @note При неизвестной строке возвращает DEVICE_MODE_AUTO
 */
device_mode_t device_control_mode_from_string(const char *str);

// ============================================================================
// CALLBACK ИЗМЕНЕНИЯ СОСТОЯНИЯ GPIO
// ============================================================================

/**
 * @brief Тип callback-функции при изменении состояния GPIO
 * @param topic MQTT топик для публикации состояния
 * @param state true = включено, false = выключено
 *
 * @note Вызывается после каждого gpio_set_level() в device_control
 * @note Callback вызывается в контексте вызывающей задачи (не ISR)
 */
typedef void (*device_state_change_cb_t)(const char *topic, bool state);

/**
 * @brief Зарегистрировать callback для уведомления об изменении GPIO
 * @param cb Callback-функция или NULL для отмены
 *
 * @note Вызывается из mqtt_client при подключении к MQTT
 * @note Только один callback может быть зарегистрирован одновременно
 */
void device_control_register_state_cb(device_state_change_cb_t cb);

// ============================================================================
// ДАННЫЕ DHT
// ============================================================================

/**
 * @brief Установить данные DHT датчика
 * @param temperature Температура в °C
 * @param humidity Влажность в %
 * @param valid true если данные корректны
 *
 * @note Вызывается из dht_task() после успешного чтения
 */
void device_control_set_dht_data(float temperature, float humidity, bool valid);

/**
 * @brief Получить температуру с DHT датчика
 * @return Температура в °C
 */
float device_control_get_dht_temperature(void);

/**
 * @brief Получить влажность с DHT датчика
 * @return Влажность в %
 */
float device_control_get_dht_humidity(void);

/**
 * @brief Проверить валидность данных DHT
 * @return true если данные были успешно получены хотя бы раз
 */
bool device_control_is_dht_data_valid(void);

// ============================================================================
// ОСТАНОВКА ADS1115 ДЛЯ OTA
// ============================================================================

/**
 * @brief Запросить остановку задачи ADS1115 для OTA обновления
 *
 * @note Вызывается из ota_client перед началом OTA
 */
void device_control_ads1115_stop_for_ota(void);

/**
 * @brief Проверить, запрошена ли остановка ADS1115
 * @return true если OTA запросила остановку
 *
 * @note Вызывается из ads1115_task() в цикле опроса
 */
bool device_control_ads1115_is_stop_requested(void);

// ============================================================================
// СИГНАЛИЗАЦИЯ ЗАДАЧ РАСПИСАНИЯ (EventGroup)
// ============================================================================

/**
 * @brief Установить EventGroup для синхронизации задач расписания
 * @param event Handle EventGroup (создаётся в main.c)
 *
 * @note Вызывается один раз в app_main() после создания EventGroup
 */
void device_control_set_schedule_event(void *event);

/**
 * @brief Сигнал задачам расписания об изменении режима (AUTO/MANUAL)
 *
 * Вызывается из mqtt_client при получении команды mode/set или ON/OFF.
 * Мгновенно пробуждает light_schedule_task и pump_valve_schedule_task.
 */
void device_control_notify_mode_changed(void);

/**
 * @brief Сигнал задаче OTA об изменении флага
 *
 * Вызывается из mqtt_client при получении команды OTA START.
 */
void device_control_notify_ota_flag(void);

// ============================================================================
// CALLBACK ИЗМЕНЕНИЯ СОСТОЯНИЯ GPIO
// ============================================================================

/**
 * @brief Тип callback-функции при изменении состояния GPIO
 * @param topic MQTT топик для публикации состояния
 * @param state true = включено, false = выключено
 *
 * @note Вызывается после каждого gpio_set_level() в device_control
 * @note Callback вызывается в контексте вызывающей задачи (не ISR)
 */
typedef void (*device_state_change_cb_t)(const char *topic, bool state);

/**
 * @brief Зарегистрировать callback для уведомления об изменении GPIO
 * @param cb Callback-функция или NULL для отмены
 *
 * @note Вызывается из mqtt_client при подключении к MQTT
 * @note Только один callback может быть зарегистрирован одновременно
 */
void device_control_register_state_cb(device_state_change_cb_t cb);

#ifdef __cplusplus
}
#endif
