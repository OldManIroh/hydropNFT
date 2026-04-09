/**
 * @file web_server.h
 * @brief Встроенный HTTP сервер для локального управления HydroNFT
 *
 * Предоставляет:
 * - Веб-интерфейс (dashboard)
 * - REST API для сенсоров и управления
 *
 * @author HydroNFT Team
 * @version 1.0
 * @date 2026
 */

#pragma once

#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация и запуск HTTP сервера
 *
 * Регистрирует обработчики для главной страницы и REST API.
 * Запускает сервер на порту из CONFIG_WEB_SERVER_PORT.
 *
 * @return ESP_OK при успехе, код ошибки в противном случае
 */
esp_err_t web_server_init(void);

/**
 * @brief Остановка HTTP сервера
 *
 * @return ESP_OK при успехе
 */
esp_err_t web_server_stop(void);

/**
 * @brief Проверить, запрошено ли переподключение WiFi из веб-интерфейса
 * @return true если нужно переподключить WiFi
 *
 * @note Сбрасывает флаг при чтении (одноразовый сигнал)
 */
bool web_server_wifi_reconnect_requested(void);

/**
 * @brief Установить флаг переподключения WiFi
 * @param value true — запросить переподключение, false — отменить
 *
 * @note Используется для восстановления флага при ошибке в wifi_reconnect_task
 */
void web_server_set_wifi_reconnect_requested(bool value);

#ifdef __cplusplus
}
#endif
