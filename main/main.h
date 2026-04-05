/**
 * @file main.h
 * @brief Заголовочный файл приложения HydroNFT
 *
 * Объявления задач FreeRTOS и вспомогательных функций OTA прогресса.
 * Для управления оборудованием используйте device_control.h.
 *
 * @author HydroNFT Team
 * @version 2.0
 * @date 2026
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// ОБЪЯВЛЕНИЯ ЗАДАЧ FREERTOS
// ============================================================================

/**
 * @brief Задача чтения DHT датчика
 * @param pvParameters Параметры задачи (не используются)
 */
void dht_task(void *pvParameters);

/**
 * @brief Задача чтения ADS1115
 * @param pvParameters Параметры задачи (не используются)
 */
void ads1115_task(void *pvParameters);

/**
 * @brief Задача публикации сенсоров в MQTT
 * @param pvParameters Параметры задачи (не используются)
 */
void mqtt_sensor_task(void *pvParameters);

/**
 * @brief Задача проверки флага OTA из MQTT
 * @param pvParameters Параметры задачи (не используются)
 */
void mqtt_ota_check_task(void *pvParameters);

/**
 * @brief Задача управления светом по расписанию
 * @param pvParameters Параметры задачи (не используются)
 */
void light_schedule_task(void *pvParameters);

/**
 * @brief Задача управления насосом и клапаном
 * @param pvParameters Параметры задачи (не используются)
 */
void pump_valve_schedule_task(void *pvParameters);

/**
 * @brief Задача приёма ESP-NOW
 * @param pvParameters Параметры задачи (не используются)
 */
void espnow_receiver_task(void *pvParameters);

/**
 * @brief Задача публикации прогресса OTA в MQTT
 * @param pvParameters Параметры задачи (не используются)
 */
void mqtt_ota_progress_task(void *pvParameters);

// ============================================================================
// УПРАВЛЕНИЕ ЗАДАЧЕЙ ПРОГРЕССА OTA
// ============================================================================

/**
 * @brief Запуск задачи публикации прогресса OTA
 */
void start_ota_progress_task(void);

/**
 * @brief Остановка задачи публикации прогресса OTA
 */
void stop_ota_progress_task(void);

#ifdef __cplusplus
}
#endif
