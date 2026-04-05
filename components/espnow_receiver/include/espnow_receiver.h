#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация ESP-NOW приёмника
 *
 * Инициализирует Wi-Fi (STA mode), ESP-NOW и регистрирует callback
 * для приёма данных. Ожидает 1-байтовые сообщения (true/false).
 *
 * @return ESP_OK при успехе, код ошибки в противном случае
 *
 * @note Вызывать ПОСЛЕ ota_init() — нужен инициализированный Wi-Fi стек
 * @note Wi-Fi не подключается к AP, ESP-NOW работает напрямую
 */
esp_err_t espnow_receiver_init(void);

/**
 * @brief Задача обработки полученных ESP-NOW данных
 *
 * Периодически проверяет состояние touch-сигнала и публикует
 * в MQTT топик hydro/control/touch при изменении.
 *
 * @param pvParameters Параметры задачи (не используются)
 *
 * @note Задача работает на ядре 0 (PRO_CPU)
 * @note Приоритет: 4 (средний)
 * @note Размер стека: 4096 байт
 * @note Интервал проверки: 100 мс
 */
void espnow_receiver_task(void *pvParameters);

/**
 * @brief Получить текущее состояние touch-сигнала
 *
 * Возвращает последнее принятое по ESP-NOW состояние.
 *
 * @return true = касание (верхний уровень), false = нет касания
 */
bool espnow_receiver_get_touch_state(void);

#ifdef __cplusplus
}
#endif
