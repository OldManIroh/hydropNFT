/**
 * @file settings.c
 * @brief Хранение настроек в NVS (Non-Volatile Storage)
 *
 * @par Namespace: "hydrocfg"
 * Хранит WiFi SSID/pass, MQTT URI/user/pass — сохраняется между перезагрузками.
 *
 * @par Безопасность буферов
 * nvs_get_str_safe() гарантирует null-termination даже при ошибке NVS:
 * - buf[0] = '\0' ДО чтения — пустая строка если NVS недоступен
 * - buf[buf_size-1] = '\0' ПОСЛЕ чтения — защита от ESP_ERR_NVS_INVALID_LENGTH
 * - caller проверяет return code перед использованием данных
 *
 * @par Потокобезопасность
 * NVS API ESP-IDF потокобезопасен для одного handle (внутренний мьютекс).
 * Настройки читаются из разных задач (wifi_reconnect_task, web_server, main)
 * без дополнительного мьютекса — ESP-IDF гарантирует атомарность nvs_get_str.
 *
 * @author HydroNFT Team
 * @version 1.0
 * @date 2026
 */

#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "settings.h"

static const char *TAG = "settings";
static const char *NVS_NAMESPACE = "hydrocfg";

static nvs_handle_t s_nvs_handle = 0;

esp_err_t settings_init(void)
{
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &s_nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка открытия NVS (%s): %s", NVS_NAMESPACE, esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "NVS инициализирован (%s)", NVS_NAMESPACE);
    return ESP_OK;
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================================

static esp_err_t nvs_get_str_safe(const char *key, char *buf, size_t buf_size)
{
    // Гарантируем null-termination даже при ошибке
    if (buf == NULL || buf_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    buf[0] = '\0';  // Очищаем буфер заранее

    size_t required_size = buf_size;
    esp_err_t err = nvs_get_str(s_nvs_handle, key, buf, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "Ключ '%s' не найден в NVS", key);
        return ESP_ERR_NOT_FOUND;
    }
    if (err == ESP_ERR_NVS_INVALID_LENGTH) {
        // Строка длиннее буфера — данные обрезаны, но null-termination гарантирована
        ESP_LOGW(TAG, "Ключ '%s' усечён: требуется %zu, буфер %zu", key, required_size, buf_size);
        buf[buf_size - 1] = '\0';
        return err;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка чтения '%s' из NVS: %s", key, esp_err_to_name(err));
        return err;
    }

    // Дополнительная гарантия null-termination
    buf[buf_size - 1] = '\0';
    return ESP_OK;
}

static esp_err_t nvs_set_str_safe(const char *key, const char *value)
{
    if (value == NULL || value[0] == '\0') {
        return nvs_erase_key(s_nvs_handle, key);
    }
    esp_err_t err = nvs_set_str(s_nvs_handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(s_nvs_handle);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка записи '%s' в NVS: %s", key, esp_err_to_name(err));
    }
    return err;
}

// ============================================================================
// WiFi НАСТРОЙКИ
// ============================================================================

esp_err_t settings_get_wifi_ssid(char *buf, size_t buf_size)
{
    return nvs_get_str_safe("wifi_ssid", buf, buf_size);
}

esp_err_t settings_set_wifi_ssid(const char *ssid)
{
    esp_err_t err = nvs_set_str_safe("wifi_ssid", ssid);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi SSID сохранён в NVS");
    }
    return err;
}

esp_err_t settings_get_wifi_pass(char *buf, size_t buf_size)
{
    return nvs_get_str_safe("wifi_pass", buf, buf_size);
}

esp_err_t settings_set_wifi_pass(const char *pass)
{
    esp_err_t err = nvs_set_str_safe("wifi_pass", pass);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "WiFi пароль сохранён в NVS");
    }
    return err;
}

// ============================================================================
// MQTT НАСТРОЙКИ
// ============================================================================

esp_err_t settings_get_mqtt_uri(char *buf, size_t buf_size)
{
    return nvs_get_str_safe("mqtt_uri", buf, buf_size);
}

esp_err_t settings_set_mqtt_uri(const char *uri)
{
    esp_err_t err = nvs_set_str_safe("mqtt_uri", uri);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MQTT URI сохранён в NVS");
    }
    return err;
}

esp_err_t settings_get_mqtt_user(char *buf, size_t buf_size)
{
    return nvs_get_str_safe("mqtt_user", buf, buf_size);
}

esp_err_t settings_set_mqtt_user(const char *user)
{
    esp_err_t err = nvs_set_str_safe("mqtt_user", user);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MQTT user сохранён в NVS");
    }
    return err;
}

esp_err_t settings_get_mqtt_pass(char *buf, size_t buf_size)
{
    return nvs_get_str_safe("mqtt_pass", buf, buf_size);
}

esp_err_t settings_set_mqtt_pass(const char *pass)
{
    esp_err_t err = nvs_set_str_safe("mqtt_pass", pass);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MQTT пароль сохранён в NVS");
    }
    return err;
}
