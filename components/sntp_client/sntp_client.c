/**
 * @file sntp_client.c
 * @brief Реализация SNTP клиента для синхронизации времени
 *
 * Компонент использует ESP-IDF SNTP библиотеку для получения
 * точного времени с публичных NTP серверов.
 *
 * @author HydroNFT Team
 * @version 1.0
 * @date 2024
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
#include "esp_netif_sntp.h"
#include "sntp_client.h"

/// Тег для системы логирования
static const char *TAG = "sntp_client";

/// Флаг состояния синхронизации времени (atomic — пишется из callback, читается из задач)
static _Atomic bool s_time_synced = false;

/**
 * @brief Callback функция уведомления о синхронизации времени
 *
 * Вызывается автоматически SNTP библиотекой при успешном получении времени.
 * Логирует текущее время и устанавливает флаг синхронизации.
 *
 * @param tv Указатель на структуру timeval с текущим временем
 */
void time_sync_notification_cb(struct timeval *tv)
{
    // Получаем текущее время для форматирования
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Форматируем время в строку для красивого вывода
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);

    // ESP_LOGI — чтобы log_forwarder НЕ отправлял в MQTT (только W и E)
    // Публикация в MQTT в момент SNTP sync может повредить сетевой стек
    ESP_LOGI(TAG, "SNTP синхронизировано: %s", strftime_buf);

    // Устанавливаем флаг синхронизации
    s_time_synced = true;
}

/**
 * @brief Инициализация SNTP клиента
 *
 * Конфигурирует SNTP с серверами и callback функцией.
 * Использует настройки из Kconfig для NTP серверов.
 *
 * @note Требуется активное подключение к WiFi перед вызовом
 */
void sntp_client_init(void)
{
    ESP_LOGI(TAG, "Инициализация SNTP клиента");

    // Создаём конфигурацию SNTP с настройками по умолчанию
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG();

    // Настраиваем NTP серверы из Kconfig (в порядке приоритета)
    config.servers[0] = CONFIG_SNTP_SERVER_1;

    // Устанавливаем callback для уведомления о синхронизации
    config.sync_cb = time_sync_notification_cb;

    // Инициализируем SNTP с конфигурацией
    esp_netif_sntp_init(&config);

    ESP_LOGI(TAG, "SNTP запущен, ожидание синхронизации...");
}

/**
 * @brief Синхронизация времени через SNTP
 *
 * Ожидает синхронизации времени с NTP серверами.
 * Блокируется до успешной синхронизации или исчерпания попыток.
 *
 * @return true если синхронизация успешна
 * @return false если не удалось синхронизировать время
 *
 * @note Требуется активное подключение к WiFi перед вызовом
 */
bool sntp_time_sync(void)
{
    ESP_LOGI(TAG, "Синхронизация времени через SNTP");

    int retry = 0;
    const int retry_count = CONFIG_SNTP_SYNC_RETRY_COUNT;

    while (esp_netif_sntp_sync_wait(5000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT &&
           ++retry < retry_count) {
        ESP_LOGI(TAG, "Ожидание синхронизации NTP... (%d/%d)", retry, retry_count);
    }

    if (retry < retry_count) {
        ESP_LOGI(TAG, "Время успешно синхронизировано");
        return true;
    } else {
        ESP_LOGE(TAG, "Не удалось синхронизировать время");
        return false;
    }
}

/**
 * @brief Проверка состояния синхронизации
 */
bool sntp_client_is_synced(void)
{
    return s_time_synced;
}

/**
 * @brief Получение строки текущего времени
 */
bool sntp_client_get_time_string(char *buffer, size_t buffer_size)
{
    if (!s_time_synced) {
        return false;
    }
    
    time_t now;
    struct tm timeinfo;
    
    // Получаем текущее время
    time(&now);
    localtime_r(&now, &timeinfo);
    
    // Форматируем в строку "YYYY-MM-DD HH:MM:SS"
    strftime(buffer, buffer_size, "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    return true;
}

/**
 * @brief Получение Unix timestamp
 */
time_t sntp_client_get_timestamp(void)
{
    if (!s_time_synced) {
        return 0;
    }
    
    time_t now;
    time(&now);
    return now;
}