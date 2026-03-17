/**
 * @file log_forwarder.c
 * @brief Компонент пересылки WARNING и ERROR логов в MQTT
 *
 * Перехватывает логи ESP-IDF через esp_log_set_vprintf()
 * и отправляет только WARNING и ERROR уровни в Home Assistant.
 *
 * ВАЖНО: esp_log_set_vprintf() получает строку, которая уже содержит
 * ANSI escape-коды цветов (если они включены в menuconfig).
 * Формат с цветами:  "\033[0;33mW (12345) tag: message\033[0m\n"
 * Формат без цветов: "W (12345) tag: message\n"
 *
 * Для корректной работы рекомендуется отключить цвета в menuconfig:
 *   Component config → Log output → Use ANSI terminal colors → OFF
 * Или использовать парсинг с учётом escape-кодов (реализовано ниже).
 *
 * @author HydroNFT Team
 * @version 2.0
 * @date 2024
 */

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "log_forwarder.h"
#include "hydro_mqtt_client.h"

/// Тег для внутреннего логирования компонента
static const char *TAG = "log_forwarder";

/// Флаг включения/выключения пересылки логов
static bool s_log_forwarding_enabled = true;

/// Флаг защиты от рекурсии (когда наш код сам генерирует логи)
static bool s_inside_log_handler = false;

/// Буфер для формирования MQTT сообщений (критическая секция)
static StaticSemaphore_t s_buffer_mutex_storage;
static SemaphoreHandle_t s_buffer_mutex;

/// Буфер для формирования сообщения (защищён мьютексом)
static char s_log_buffer[512];

/// Время последней отправки лога (для ограничения частоты)
static int64_t s_last_log_time = 0;

/// Минимальный интервал между отправками логов (мс)
#define LOG_THROTTLE_INTERVAL_MS 200

/// Топик для отправки логов
static const char *LOG_MQTT_TOPIC = "hydro/log";

/// Топик для MQTT Discovery конфигурации
static const char *LOG_DISCOVERY_TOPIC = "homeassistant/sensor/hydroesp32/log/config";

/**
 * @brief Извлечь символ уровня лога из строки
 *
 * ESP-IDF может форматировать логи двумя способами:
 * 1. С ANSI цветами: "\033[0;33mW (12345) tag: message\033[0m\n"
 *    - Escape-последовательность: \033[ + параметры + m
 *    - После 'm' идёт символ уровня (W, E, I, D, V)
 * 2. Без цветов: "W (12345) tag: message\n"
 *    - Первый символ — уровень лога
 *
 * @param str Строка лога
 * @return Символ уровня ('E', 'W', 'I', 'D', 'V') или '\0' если не удалось определить
 */
static char extract_log_level(const char *str)
{
    if (str == NULL || str[0] == '\0') {
        return '\0';
    }

    // Случай 1: Строка начинается с ESC (\033 = 0x1B) — есть ANSI цвета
    if (str[0] == '\033' && str[1] == '[') {
        // Пропускаем escape-последовательность: \033[...m
        const char *p = str + 2;
        while (*p != '\0' && *p != 'm') {
            p++;
        }
        if (*p == 'm') {
            p++; // Пропускаем 'm'
            // Теперь p указывает на символ уровня лога
            return *p;
        }
        return '\0';
    }

    // Случай 2: Нет escape-кодов — первый символ это уровень
    // Проверяем что это валидный символ уровня
    if (str[0] == 'E' || str[0] == 'W' || str[0] == 'I' ||
        str[0] == 'D' || str[0] == 'V') {
        return str[0];
    }

    return '\0';
}

/**
 * @brief Очистить ANSI escape-коды из строки
 *
 * Удаляет все ANSI escape-последовательности вида \033[...m
 * для чистого текста в MQTT сообщении.
 *
 * @param dst Буфер назначения
 * @param dst_size Размер буфера назначения
 * @param src Исходная строка с escape-кодами
 */
static void strip_ansi_codes(char *dst, size_t dst_size, const char *src)
{
    size_t di = 0;
    const char *p = src;

    while (*p != '\0' && di < dst_size - 1) {
        if (*p == '\033') {
            // Пропускаем escape-последовательность: \033[...m
            p++;
            if (*p == '[') {
                p++;
                while (*p != '\0' && *p != 'm') {
                    p++;
                }
                if (*p == 'm') {
                    p++;
                }
            }
        } else {
            dst[di++] = *p++;
        }
    }
    dst[di] = '\0';

    // Убираем символы перевода строки в конце
    while (di > 0 && (dst[di - 1] == '\n' || dst[di - 1] == '\r')) {
        dst[--di] = '\0';
    }
}

/**
 * @brief Кастомная функция вывода логов
 *
 * Перехватывает все логи ESP-IDF и фильтрует их:
 * - WARNING (W) - отправляем в MQTT и UART
 * - ERROR (E) - отправляем в MQTT и UART
 * - Остальные - только в UART
 *
 * @param fmt Форматная строка лога
 * @param args Список аргументов
 * @return Количество отправленных символов
 */
static int log_vprintf(const char *fmt, va_list args)
{
    // Защита от рекурсии: если мы уже внутри обработчика,
    // просто выводим в UART и выходим
    if (s_inside_log_handler) {
        return vprintf(fmt, args);
    }

    // ВАЖНО: va_list можно использовать только ОДИН раз!
    // Для второго использования нужен va_copy()
    va_list args_copy;
    va_copy(args_copy, args);

    // Выводим в UART (консоль) — используем оригинальный args
    int ret = vprintf(fmt, args);

    // Если пересылка выключена — освобождаем копию и выходим
    if (!s_log_forwarding_enabled) {
        va_end(args_copy);
        return ret;
    }

    // Форматируем строку во временный буфер для анализа уровня
    // Используем args_copy (т.к. args уже использован в vprintf)
    char temp_buf[256];
    vsnprintf(temp_buf, sizeof(temp_buf), fmt, args_copy);
    va_end(args_copy);

    // Определяем уровень лога
    char level = extract_log_level(temp_buf);

    // Пропускаем всё кроме WARNING и ERROR
    if (level != 'W' && level != 'E') {
        return ret;
    }

    // Ограничение частоты отправки логов (throttling)
    int64_t now = esp_timer_get_time() / 1000; // мс
    if (now - s_last_log_time < LOG_THROTTLE_INTERVAL_MS) {
        return ret;
    }
    s_last_log_time = now;

    // Проверяем подключение к MQTT
    if (!mqtt_client_is_connected()) {
        return ret;
    }

    // Формируем сообщение для MQTT (защищаем мьютексом)
    // Используем xSemaphoreTake с таймаутом 0 — не блокируемся
    if (xSemaphoreTake(s_buffer_mutex, 0) == pdTRUE) {
        // Устанавливаем флаг рекурсии
        s_inside_log_handler = true;

        // Очищаем ANSI escape-коды для чистого текста в MQTT
        strip_ansi_codes(s_log_buffer, sizeof(s_log_buffer), temp_buf);

        // Отправляем в MQTT топик hydro/log
        esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)mqtt_client_get_handle();
        if (client != NULL) {
            esp_mqtt_client_publish(client, LOG_MQTT_TOPIC, s_log_buffer, 0, 1, 0);
        }

        // Снимаем флаг рекурсии
        s_inside_log_handler = false;

        xSemaphoreGive(s_buffer_mutex);
    }

    return ret;
}

/**
 * @brief Отправка MQTT Discovery конфигурации для сенсора логов
 *
 * Регистрирует сенсор логов в Home Assistant через MQTT Discovery.
 * После этого HA автоматически создаст entity sensor.hydronft_log.
 *
 * @note Вызывается при подключении к MQTT
 */
static void log_forwarder_send_discovery(void)
{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)mqtt_client_get_handle();
    if (client == NULL) {
        return;
    }

    const char *payload =
        "{"
        "\"name\":\"Логи устройства\","
        "\"state_topic\":\"hydro/log\","
        "\"unique_id\":\"esp32_hydro_log\","
        "\"icon\":\"mdi:math-log\","
        "\"entity_category\":\"diagnostic\","
        "\"device\":{"
        "\"identifiers\":[\"esp32_hydro_controller\"],"
        "\"name\":\"Hydroponic system\","
        "\"model\":\"ESP32\","
        "\"manufacturer\":\"HydroNFT\""
        "}"
        "}";

    esp_mqtt_client_publish(client, LOG_DISCOVERY_TOPIC, payload, 0, 1, 1);
}

/**
 * @brief Задача ожидания подключения MQTT и отправки Discovery
 *
 * Ждёт подключения к MQTT и отправляет Discovery конфигурацию.
 * После отправки задача удаляет себя.
 *
 * @param pvParameters Параметры задачи (не используются)
 */
static void log_forwarder_discovery_task(void *pvParameters)
{
    // Ждём подключения к MQTT (максимум 60 секунд)
    for (int i = 0; i < 60; i++) {
        if (mqtt_client_is_connected()) {
            log_forwarder_send_discovery();
            ESP_LOGI(TAG, "MQTT Discovery для логов отправлен");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Задача больше не нужна — удаляем
    vTaskDelete(NULL);
}

/**
 * @brief Инициализация компонента пересылки логов
 *
 * 1. Создаёт мьютекс для защиты буфера
 * 2. Устанавливает кастомную функцию вывода логов
 * 3. Запускает задачу отправки MQTT Discovery
 *
 * @note Вызывать ПОСЛЕ mqtt_client_init()
 */
void log_forwarder_init(void)
{
    ESP_LOGI(TAG, "Инициализация пересылки логов в MQTT");

    // Создаём мьютекс для защиты буфера
    s_buffer_mutex = xSemaphoreCreateMutexStatic(&s_buffer_mutex_storage);
    if (s_buffer_mutex == NULL) {
        ESP_LOGE(TAG, "Не удалось создать мьютекс буфера");
        return;
    }

    // Устанавливаем кастомную функцию вывода логов
    esp_log_set_vprintf(log_vprintf);

    ESP_LOGI(TAG, "Пересылка WARNING и ERROR логов в MQTT включена");
    ESP_LOGI(TAG, "Топик: %s", LOG_MQTT_TOPIC);

    // Запускаем задачу для отправки MQTT Discovery конфигурации
    // (нужно дождаться подключения к MQTT)
    xTaskCreate(
        log_forwarder_discovery_task,
        "log_disc_task",
        2048,
        NULL,
        2,  // Низкий приоритет
        NULL
    );
}

/**
 * @brief Включить/выключить пересылку логов
 */
void log_forwarder_enable(bool enable)
{
    s_log_forwarding_enabled = enable;
    ESP_LOGI(TAG, "Пересылка логов %s", enable ? "включена" : "выключена");
}

/**
 * @brief Проверка состояния пересылки логов
 */
bool log_forwarder_is_enabled(void)
{
    return s_log_forwarding_enabled;
}
