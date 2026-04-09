/**
 * @file settings.h
 * @brief Хранение настроек в NVS для HydroNFT
 *
 * Сохраняет конфигурацию WiFi и MQTT между перезагрузками.
 * Приоритет: NVS → menuconfig defaults.
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

/// Максимальная длина SSID
#define SETTINGS_SSID_MAX_LEN 32
/// Максимальная длина пароля
#define SETTINGS_PASS_MAX_LEN 64
/// Максимальная длина URI
#define SETTINGS_URI_MAX_LEN 128

/**
 * @brief Инициализация NVS
 *
 * Открывает namespace "hydrocfg" для хранения настроек.
 *
 * @return ESP_OK при успехе
 */
esp_err_t settings_init(void);

// ============================================================================
// WiFi НАСТРОЙКИ
// ============================================================================

/**
 * @brief Получить SSID WiFi из NVS
 * @param buf Буфер для SSID (мин. SETTINGS_SSID_MAX_LEN)
 * @param buf_size Размер буфера
 * @return ESP_OK при успехе, ESP_ERR_NOT_FOUND если не сохранён
 */
esp_err_t settings_get_wifi_ssid(char *buf, size_t buf_size);

/**
 * @brief Сохранить SSID WiFi в NVS
 * @param ssid Строка SSID
 * @return ESP_OK при успехе
 */
esp_err_t settings_set_wifi_ssid(const char *ssid);

/**
 * @brief Получить пароль WiFi из NVS
 * @param buf Буфер для пароля (мин. SETTINGS_PASS_MAX_LEN)
 * @param buf_size Размер буфера
 * @return ESP_OK при успехе, ESP_ERR_NOT_FOUND если не сохранён
 */
esp_err_t settings_get_wifi_pass(char *buf, size_t buf_size);

/**
 * @brief Сохранить пароль WiFi в NVS
 * @param pass Строка пароля
 * @return ESP_OK при успехе
 */
esp_err_t settings_set_wifi_pass(const char *pass);

// ============================================================================
// MQTT НАСТРОЙКИ
// ============================================================================

/**
 * @brief Получить URI MQTT брокера из NVS
 * @param buf Буфер для URI (мин. SETTINGS_URI_MAX_LEN)
 * @param buf_size Размер буфера
 * @return ESP_OK при успехе, ESP_ERR_NOT_FOUND если не сохранён
 */
esp_err_t settings_get_mqtt_uri(char *buf, size_t buf_size);

/**
 * @brief Сохранить URI MQTT брокера в NVS
 * @param uri Строка URI
 * @return ESP_OK при успехе
 */
esp_err_t settings_set_mqtt_uri(const char *uri);

/**
 * @brief Получить MQTT username из NVS
 * @param buf Буфер (мин. SETTINGS_SSID_MAX_LEN)
 * @param buf_size Размер буфера
 * @return ESP_OK при успехе, ESP_ERR_NOT_FOUND если не сохранён
 */
esp_err_t settings_get_mqtt_user(char *buf, size_t buf_size);

/**
 * @brief Сохранить MQTT username в NVS
 * @param user Строка username
 * @return ESP_OK при успехе
 */
esp_err_t settings_set_mqtt_user(const char *user);

/**
 * @brief Получить MQTT password из NVS
 * @param buf Буфер (мин. SETTINGS_PASS_MAX_LEN)
 * @param buf_size Размер буфера
 * @return ESP_OK при успехе, ESP_ERR_NOT_FOUND если не сохранён
 */
esp_err_t settings_get_mqtt_pass(char *buf, size_t buf_size);

/**
 * @brief Сохранить MQTT password в NVS
 * @param pass Строка password
 * @return ESP_OK при успехе
 */
esp_err_t settings_set_mqtt_pass(const char *pass);

#ifdef __cplusplus
}
#endif
