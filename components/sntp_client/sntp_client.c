/**
 * @file sntp_client.c
 * @brief SNTP клиент для синхронизации времени
 *
 * @par Архитектура
 * - ESP-IDF SNTP работает в фоновом режиме (асинхронно)
 * - Callback time_sync_notification_cb() вызывается при успешной синхронизации
 * - Флаг s_time_synced (_Atomic bool) — задачи проверяют перед использованием времени
 *
 * @par Часовой пояс
 * - CONFIG_SNTP_TIMEZONE (Kconfig, по умолч. MSK-5 = UTC+5 Екатеринбург)
 * - POSIX TZ строка: <offset> = UTC - local, т.е. MSK-5 означает UTC+5
 * - tzset() применяет пояс после setenv("TZ", ...)
 *
 * @par sntp_time_sync() — блокирующая синхронизация
 * Функция существует для сценариев где нужно дождаться времени ПЕРЕД продолжением.
 * В основном проекте НЕ используется — SNTP работает асинхронно, задачи проверяют
 * sntp_client_is_synced() перед чтением времени.
 *
 * @author HydroNFT Team
 * @version 2.0
 * @date 2026
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
#include "hydro_mqtt_client.h"     // Для проверки MQTT подключения
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
    // Устанавливаем часовой пояс UTC+5 (Екатеринбург)
    // POSIX: UTC-5 означает "впереди UTC на 5 часов" (минус = east)
#ifdef CONFIG_SNTP_TIMEZONE
    setenv("TZ", CONFIG_SNTP_TIMEZONE, 1);
#else
    setenv("TZ", "UTC-5", 1);
#endif
    tzset();

    // Форматируем время — уже с учётом часового пояса
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    char strftime_buf[64];
    strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);

    // ESP_LOGI — только в UART терминал
    ESP_LOGI(TAG, "SNTP синхронизировано: %s", strftime_buf);

    // Устанавливаем флаг синхронизации
    atomic_store(&s_time_synced, true);
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

    // Инициализация SNTP с несколькими серверами для отказоустойчивости.
    // Количество серверов задаётся в menuconfig → SNTP Client Configuration → Number of NTP servers.
#if CONFIG_SNTP_SERVER_COUNT_4
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(
        4, ESP_SNTP_SERVER_LIST(CONFIG_SNTP_SERVER_1, CONFIG_SNTP_SERVER_2,
                                CONFIG_SNTP_SERVER_3, CONFIG_SNTP_SERVER_4));
    ESP_LOGI(TAG, "SNTP: 4 сервера для отказоустойчивости");
#elif CONFIG_SNTP_SERVER_COUNT_3
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(
        3, ESP_SNTP_SERVER_LIST(CONFIG_SNTP_SERVER_1, CONFIG_SNTP_SERVER_2,
                                CONFIG_SNTP_SERVER_3));
    ESP_LOGI(TAG, "SNTP: 3 сервера для отказоустойчивости");
#elif CONFIG_SNTP_SERVER_COUNT_2
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(
        2, ESP_SNTP_SERVER_LIST(CONFIG_SNTP_SERVER_1, CONFIG_SNTP_SERVER_2));
    ESP_LOGI(TAG, "SNTP: 2 сервера для отказоустойчивости");
#else
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_SERVER_1);
    ESP_LOGI(TAG, "SNTP: 1 сервер");
#endif

    // Устанавливаем callback для уведомления о синхронизации
    config.sync_cb = time_sync_notification_cb;

    // Плавная синхронизация — корректирует время постепенно (adjtime()),
    // а не скачком (settimeofday()). Предотвращает:
    // - Разрыв таймеров слива (esp_timer_get_time не затрагивается, но time() может скакнуть)
    // - Ложные срабатывания переходов день↔ночь в расписаниях
    // - Артефакты в логах (время прыгает назад)
    config.smooth_sync = true;

    // Инициализируем SNTP с конфигурацией
    esp_err_t ret = esp_netif_sntp_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка инициализации SNTP: %s", esp_err_to_name(ret));
        return;
    }

    // Если LWIP_MAX_SERVERS меньше выбранного — часть серверов будет проигнорирована
#if CONFIG_SNTP_SERVER_COUNT_4 && CONFIG_LWIP_SNTP_MAX_SERVERS < 4
    ESP_LOGW(TAG, "CONFIG_LWIP_SNTP_MAX_SERVERS=%d < 4 — увеличьте в menuconfig → LWIP → SNTP",
             CONFIG_LWIP_SNTP_MAX_SERVERS);
#elif CONFIG_SNTP_SERVER_COUNT_3 && CONFIG_LWIP_SNTP_MAX_SERVERS < 3
    ESP_LOGW(TAG, "CONFIG_LWIP_SNTP_MAX_SERVERS=%d < 3 — увеличьте в menuconfig → LWIP → SNTP",
             CONFIG_LWIP_SNTP_MAX_SERVERS);
#elif CONFIG_SNTP_SERVER_COUNT_2 && CONFIG_LWIP_SNTP_MAX_SERVERS < 2
    ESP_LOGW(TAG, "CONFIG_LWIP_SNTP_MAX_SERVERS=%d < 2 — увеличьте в menuconfig → LWIP → SNTP",
             CONFIG_LWIP_SNTP_MAX_SERVERS);
#endif

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
    esp_err_t sync_result;

    while ((sync_result = esp_netif_sntp_sync_wait(5000 / portTICK_PERIOD_MS)) == ESP_ERR_TIMEOUT &&
           ++retry < retry_count) {
        ESP_LOGI(TAG, "Ожидание синхронизации NTP... (%d/%d)", retry, retry_count);
    }

    // Проверяем результат — не только TIMEOUT, но и другие ошибки
    if (sync_result != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка синхронизации SNTP: %s (%d попыток)",
                 esp_err_to_name(sync_result), retry_count);
        atomic_store(&s_time_synced, false);
        return false;
    }

    if (retry >= retry_count) {
        ESP_LOGE(TAG, "Не удалось синхронизировать время после %d попыток", retry_count);
        atomic_store(&s_time_synced, false);
        return false;
    }

    ESP_LOGI(TAG, "Время успешно синхронизировано");
    return true;
}

/**
 * @brief Проверка состояния синхронизации
 */
bool sntp_client_is_synced(void)
{
    return atomic_load(&s_time_synced);
}

/**
 * @brief Получение строки текущего времени
 */
bool sntp_client_get_time_string(char *buffer, size_t buffer_size)
{
    if (!atomic_load(&s_time_synced)) {
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
    if (!atomic_load(&s_time_synced)) {
        return 0;
    }
    
    time_t now;
    time(&now);
    return now;
}