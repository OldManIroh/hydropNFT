/**
 * @file espnow_receiver.c
 * @brief ESP-NOW приёмник для HydroNFT
 *
 * Принимает бинарные пакеты состояния от espnow_binary_state устройства
 * и публикует изменения в MQTT топик hydro/control/touch.
 *
 * Формат пакета (отправитель espnow_binary_state):
 * - uint8_t  state     (1 = касание, 0 = нет касания)
 * - uint16_t seq_num   (порядковый номер)
 * - uint16_t crc       (CRC16 контроль суммы)
 *
 * Архитектура:
 * - ESP-NOW callback сохраняет состояние в _Atomic bool (минимум работы в IRQ)
 * - FreeRTOS задача читает состояние и публикует в MQTT при изменении
 *
 * @author HydroNFT Team
 * @version 1.0
 * @date 2026
 */

#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "esp_crc.h"
#include "hydro_mqtt_client.h"
#include "espnow_receiver.h"

static const char *TAG = "espnow_receiver";

// ============================================================================
/// СТРУКТУРА ДАННЫХ (должна совпадать с отправителем)
// ============================================================================

typedef struct {
    uint8_t state;
    uint16_t seq_num;
    uint16_t crc;
} __attribute__((packed)) state_packet_t;

/// Текущее состояние touch-сигнала (защита от race condition)
static _Atomic bool s_touch_state = false;

/// Флаг изменения состояния (сигнал для задачи)
static _Atomic bool s_state_changed = false;

/// MAC-адрес последнего отправителя (для отладки)
/// @note Записывается в ISR, читается в задаче — без синхронизации.
///       Допустимо т.к. используется только для логирования.
static uint8_t s_last_sender_mac[6] = {0};

/// Счётчик принятых пакетов
static _Atomic uint32_t s_packet_count = 0;

/// Счётчик отброшенных пакетов (CRC ошибка)
static _Atomic uint32_t s_crc_errors = 0;

// ============================================================================
/// ESP-NOW CALLBACK
// ============================================================================

/**
 * @brief Callback приёма ESP-NOW данных
 *
 * Вызывается в контексте прерывания Wi-Fi. Валидирует CRC пакета,
 * извлекает состояние и сохраняет в _Atomic bool.
 */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len)
{
    if (data == NULL || data_len < (int)sizeof(state_packet_t)) {
        ESP_LOGW(TAG, "Получены некорректные данные ESP-NOW (len=%d, expected>=%d)",
                 data_len, (int)sizeof(state_packet_t));
        return;
    }

    // Копируем пакет
    state_packet_t packet;
    memcpy(&packet, data, sizeof(state_packet_t));

    // Валидация CRC
    uint16_t expected_crc = packet.crc;
    packet.crc = 0;
    uint16_t calculated_crc = esp_crc16_le(UINT16_MAX, (const uint8_t *)&packet, sizeof(state_packet_t));

    if (calculated_crc != expected_crc) {
        atomic_fetch_add(&s_crc_errors, 1);
        ESP_LOGW(TAG, "CRC ошибка: expected=0x%04X, got=0x%04X, seq=%u",
                 expected_crc, calculated_crc, packet.seq_num);
        return;
    }

    // Сохраняем MAC-адрес отправителя
    if (recv_info->src_addr != NULL) {
        memcpy(s_last_sender_mac, recv_info->src_addr, 6);
    }

    // Извлекаем состояние (0=false, ненулевое=true)
    bool new_state = (packet.state != 0);
    atomic_store(&s_touch_state, new_state);
    atomic_store(&s_state_changed, true);
    atomic_fetch_add(&s_packet_count, 1);

    // Логирование вынесено в espnow_receiver_task() — ESP_LOGI небезопасен в ISR
}

// ============================================================================
/// ИНИЦИАЛИЗАЦИЯ
// ============================================================================

esp_err_t espnow_receiver_init(void)
{
    esp_err_t ret;

    // Wi-Fi уже инициализирован через ota_init(), проверяем
    wifi_mode_t mode;
    ret = esp_wifi_get_mode(&mode);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi не инициализирован — вызовите ota_init() перед espnow_receiver_init()");
        return ret;
    }

    // Убеждаемся что Wi-Fi в STA или APSTA режиме
    if (mode != WIFI_MODE_STA && mode != WIFI_MODE_APSTA) {
        ESP_LOGW(TAG, "Wi-Fi режим %d, переключаем на APSTA для ESP-NOW", mode);
        ret = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Не удалось установить Wi-Fi режим APSTA: %d", ret);
            return ret;
        }
    }

    // Проверяем текущий канал Wi-Fi
    uint8_t current_primary;
    wifi_second_chan_t current_second;
    ret = esp_wifi_get_channel(&current_primary, &current_second);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Текущий Wi-Fi канал: %d", current_primary);
    }

    // Устанавливаем канал только если он отличается от нужного
    if (ret != ESP_OK || current_primary != CONFIG_ESPNOW_CHANNEL) {
        ret = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Не удалось установить канал %d: %d, продолжаем на текущем",
                     CONFIG_ESPNOW_CHANNEL, ret);
        } else {
            ESP_LOGI(TAG, "Wi-Fi канал установлен: %d", CONFIG_ESPNOW_CHANNEL);
        }
    } else {
        ESP_LOGI(TAG, "Канал %d уже установлен, менять не нужно", current_primary);
    }

    // Инициализация ESP-NOW (должна быть ДО esp_now_set_pmk)
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось инициализировать ESP-NOW: %d", ret);
        return ret;
    }

    // Устанавливаем PMK (должен совпадать с отправителем)
    ret = esp_now_set_pmk((uint8_t *)CONFIG_ESPNOW_PMK);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось установить PMK: %d", ret);
        esp_now_deinit();
        return ret;
    }

    // Регистрация callback приёма
    ret = esp_now_register_recv_cb(espnow_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось зарегистрировать ESP-NOW callback: %d", ret);
        esp_now_deinit();
        return ret;
    }

    ESP_LOGI(TAG, "ESP-NOW приёмник инициализирован (ожидание touch-сигналов)");
    return ESP_OK;
}

// ============================================================================
/// ЗАДАЧА ОБРАБОТКИ
// ============================================================================

void espnow_receiver_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Задача ESP-NOW receiver запущена");

    while (1) {
        // Проверяем было ли изменение состояния
        if (atomic_exchange(&s_state_changed, false)) {
            bool current_state = atomic_load(&s_touch_state);

            // Формируем строку MAC-адреса для логирования
            char mac_str[18];
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     s_last_sender_mac[0], s_last_sender_mac[1], s_last_sender_mac[2],
                     s_last_sender_mac[3], s_last_sender_mac[4], s_last_sender_mac[5]);

            ESP_LOGI(TAG, "ESP-NOW: touch=%s (от %s) [принято=%u, CRC ошибок=%u]",
                     current_state ? "true" : "false", mac_str,
                     atomic_load(&s_packet_count), atomic_load(&s_crc_errors));

            // Публикуем в MQTT если подключены
            if (mqtt_client_is_connected()) {
                mqtt_client_publish_touch_state(current_state);
            } else {
                ESP_LOGW(TAG, "MQTT не подключён, состояние touch=%s не опубликовано",
                         current_state ? "true" : "false");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool espnow_receiver_get_touch_state(void)
{
    return atomic_load(&s_touch_state);
}
