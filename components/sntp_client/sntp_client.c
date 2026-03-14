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
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "sntp_client.h"

/// Тег для системы логирования
static const char *TAG = "sntp_client";

/// Флаг состояния синхронизации времени
static bool s_time_synced = false;

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
    ESP_LOGI(TAG, "Получено время с NTP сервера");
    
    // Получаем текущее время для форматирования
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    
    // Форматируем время в строку для красивого вывода
    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    ESP_LOGI(TAG, "Текущее время: %s", strftime_buf);
    
    // Устанавливаем флаг синхронизации
    s_time_synced = true;
}

/**
 * @brief Инициализация SNTP клиента
 * 
 * Конфигурирует SNTP с серверами и callback функцией.
 */
void sntp_client_init(void)
{
    ESP_LOGI(TAG, "Инициализация SNTP клиента");
    
    // Создаём конфигурацию SNTP с настройками по умолчанию
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG();
    
    // Настраиваем NTP серверы (в порядке приоритета)
    config.servers[0] = "ntp.msk-ix.ru";      // Московский IX
    config.servers[1] = "ru.pool.ntp.org";    // Российский pool
    config.servers[2] = "pool.ntp.org";       // Глобальный pool
    config.servers[3] = "time.google.com";    // Google NTP
    
    // Устанавливаем callback для уведомления о синхронизации
    config.sync_cb = time_sync_notification_cb;
    
    // Инициализируем SNTP с конфигурацией
    esp_netif_sntp_init(&config);
    
    ESP_LOGI(TAG, "SNTP запущен, ожидание синхронизации...");
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
