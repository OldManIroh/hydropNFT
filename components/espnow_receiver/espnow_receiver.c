/**
 * @file espnow_receiver.c
 * @brief ESP-NOW приёмник сигналов уровня воды
 *
 * @par Архитектура
 * Принимает бинарные пакеты от внешнего ESP32-передатчика (уровень воды в баке):
 * - uint8_t  state     (1 = верхний уровень, 0 = нижний)
 * - uint16_t seq_num   (порядковый номер для отладки)
 * - uint16_t crc       (CRC16 для защиты от помех)
 *
 * @par Поток данных
 * 1. ESP-NOW callback (WiFi task context) → валидация CRC → атомарная запись
 * 2. espnow_receiver_task (FreeRTOS задача) → чтение состояния → MQTT публикация
 *
 * @par Безопасность
 * - CRC16 проверяется в callback — пакеты с ошибками отбрасываются до записи
 * - MAC-адрес хранится в _Atomic uint64_t (48-bit MAC + 16-bit padding) —
 *   атомарный доступ на ESP32, torn read невозможен
 * - Приёмник НЕ меняет канал WiFi — работает на канале роутера (STA/APSTA режим)
 *   Передатчик должен быть на том же канале.
 *
 * @par ESP-NOW инициализация
 * - PMK ключ должен совпадать с передатчиком (CONFIG_ESPNOW_PMK)
 * - Wi-Fi должен быть в STA или APSTA режиме (проверяется при init)
 * - Канал НЕ меняется — esp_wifi_set_channel() не вызывается
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

/// Флаг ожидающей публикации: не восстанавливаем s_state_changed при отключённом MQTT,
/// чтобы избежать лавины логов. Публикуем при следующей возможности.
static _Atomic bool s_pending_publish = false;

/// MAC-адрес последнего отправителя (для отладки)
/// @note Атомарный доступ через 64-bit packed (48-bit MAC + padding)
static _Atomic uint64_t s_last_sender_mac = 0;

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
 * Вызывается в контексте задачи Wi-Fi (не ISR). Валидирует CRC пакета,
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

    // Сохраняем MAC-адрес отправителя атомарно (64-bit write = атомарно на ESP32)
    if (recv_info->src_addr != NULL) {
        uint64_t mac64 = 0;
        memcpy(&mac64, recv_info->src_addr, 6);  // 6 байт MAC → uint64_t
        atomic_store(&s_last_sender_mac, mac64);
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

    // Приёмник ESP-NOW остаётся на канале роутера — НЕ меняем канал.
    // STA уже подключён к AP и работает на его канале.
    // ESP-NOW использует тот же радиомодуль, поэтому автоматически
    // слушает на канале роутера. Передатчик должен быть настроен
    // на тот же канал (фиксированный канал роутера).
    {
        uint8_t current_primary;
        wifi_second_chan_t current_second;
        ret = esp_wifi_get_channel(&current_primary, &current_second);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "ESP-NOW слушает на канале Wi-Fi: %d (канал роутера)", current_primary);
        }
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

    bool last_published_state = false;  // Дедупликация состояний

    while (1) {
        // Проверяем было ли изменение состояния
        if (atomic_exchange(&s_state_changed, false)) {
            bool current_state = atomic_load(&s_touch_state);

            // Публикуем только если состояние реально изменилось
            if (current_state != last_published_state) {
                // Читаем MAC-адрес атомарно (64-bit read = атомарно на ESP32)
                uint64_t mac64 = atomic_load(&s_last_sender_mac);
                uint8_t local_mac[6];
                memcpy(local_mac, &mac64, 6);

                // Формируем строку MAC-адреса для логирования
                char mac_str[18];
                snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                         local_mac[0], local_mac[1], local_mac[2],
                         local_mac[3], local_mac[4], local_mac[5]);

                ESP_LOGI(TAG, "ESP-NOW: touch=%s (от %s) [принято=%u, CRC ошибок=%u]",
                         current_state ? "true" : "false", mac_str,
                         atomic_load(&s_packet_count), atomic_load(&s_crc_errors));

                // Публикуем в MQTT если подключены
                esp_err_t pub_res = mqtt_client_publish_touch_state(current_state);
                if (pub_res == ESP_OK) {
                    atomic_store(&s_pending_publish, false);
                    last_published_state = current_state;
                } else {
                    // Ошибка публикации — установим pending для повторной попытки
                    if (!atomic_load(&s_pending_publish)) {
                        atomic_store(&s_pending_publish, true);
                    }
                }
            }
            // Состояние не изменилось — ничего публиковать не нужно
        }

        // Публикуем отложенное состояние как только MQTT восстановится
        if (atomic_load(&s_pending_publish) && mqtt_client_is_connected()) {
            bool pending_state = atomic_load(&s_touch_state);
            esp_err_t pub_res = mqtt_client_publish_touch_state(pending_state);
            if (pub_res == ESP_OK) {
                atomic_store(&s_pending_publish, false);
                last_published_state = pending_state;
                ESP_LOGI(TAG, "ESP-NOW: отложенная публикация touch=%s выполнена",
                         pending_state ? "true" : "false");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool espnow_receiver_get_touch_state(void)
{
    return atomic_load(&s_touch_state);
}
