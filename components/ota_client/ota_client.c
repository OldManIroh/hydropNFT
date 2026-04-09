/*
 * ============================================================================
 * OTA Client Component — Обновление прошивки "по воздуху" для ESP32
 * ============================================================================
 *
 * @par Архитектура разделов flash
 * +------------------+
 * | Bootloader       | 0x1000
 * +------------------+
 * | Partition Table  | 0x8000
 * +------------------+
 * | OTA_0 (app)      | 0x10000  <- Текущая прошивка
 * +------------------+
 * | OTA_1 (app)      |          <- Резервная/обновляемая
 * +------------------+
 * | NVS (data)       |          <- Настройки, Wi-Fi креды
 * +------------------+
 *
 * @par Процесс OTA обновления
 * 1. MQTT/Web шлёт команду → ota_start_task() создаёт задачу
 * 2. s_ota_active = true (ДО создания задачи — предотвращает повторный запуск)
 * 3. Загрузка прошивки порциями по 1024 байт через HTTPS
 * 4. Проверка заголовка — если версия совпадает → abort без перезагрузки
 * 5. Проверка на invalid partition → откат если та же версия что и последняя неудачная
 * 6. esp_ota_begin → esp_ota_write → esp_ota_end → esp_ota_set_boot_partition
 * 7. esp_restart() → bootloader грузит новый раздел
 * 8. ota_check_and_diagnose() проверяет ESP_OTA_IMG_PENDING_VERIFY
 * 9. Диагностика GPIO (5 сек) → esp_ota_mark_app_valid_cancel_rollback()
 *    ИЛИ esp_ota_mark_app_invalid_rollback_and_reboot()
 *
 * @par Безопасность при OTA
 * Перед записью во flash:
 * - Отключаются насос, свет, клапан (реле могут быть в неопределённом состоянии)
 * - ADS1115 задача удаляется (I2C шина может конфликтовать с SPI flash записью)
 *
 * @author HydroNFT Team
 * @version 2.0
 * @date 2026
 * ============================================================================
 */

#include <string.h>
#include <inttypes.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_http_client.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "errno.h"
#include "ota_client.h"
#include "hydro_mqtt_client.h"  // Для публикации статуса OTA и версии прошивки
#include "esp_task_wdt.h"

#include "device_control.h"    // Для управления оборудованием и остановки ADS1115

#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"
#endif

#define BUFFSIZE 1024
#define HASH_LEN 32

static const char *TAG = "ota_client";

// Переменные для отслеживания прогресса OTA
static _Atomic int s_ota_downloaded_bytes = 0;     ///< Количество загруженных байт
static _Atomic int s_ota_total_bytes = 0;          ///< Общий размер файла
static _Atomic bool s_ota_active = false;          ///< Флаг активной OTA сессии

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

/**
 * @brief Вспомогательная функция перезагрузки с задержкой
 *
 * Используется при фатальных и нефатальных ошибках OTA.
 * Публикует статус (если указан), ждёт 5 секунд, удаляет задачу
 * из TWDT и перезагружает устройство.
 *
 * @param ota_status Строка статуса для MQTT или NULL (не публиковать)
 *
 * @note Функция НЕ ВОЗВРАЩАЕТ управление — вызов esp_restart() завершает процесс
 * @note __attribute__((noreturn)) подсказывает компилятору что код после вызова недостижим
 */
__attribute__((noreturn)) static void ota_restart_helper(const char *ota_status)
{
    if (ota_status) {
        mqtt_client_publish_ota_status(ota_status);
    }
    ESP_LOGW(TAG, "Перезагрузка через 5 секунд...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_task_wdt_delete(NULL);
    esp_restart();
    // esp_restart() не возвращает — код ниже недостижим
}

/**
 * @brief Обработчик фатальных ошибок — перезагружает устройство
 *
 * Вызывается при критических ошибках OTA:
 * - Ошибка сети/SSL
 * - Ошибка записи во flash
 * - Повреждение данных
 *
 * Перед перезагрузкой публикует статус ошибки в MQTT.
 *
 * @note Функция не возвращает управление - устройство будет перезапущено
 */
__attribute__((noreturn)) static void task_fatal_error(void)
{
    ESP_LOGE(TAG, "Фатальная ошибка OTA - перезагрузка устройства...");
    ota_restart_helper("fatal_error");
}

/**
 * @brief Перезагрузка при ошибке OTA (не фатальная)
 *
 * Вызывается при незавершённой загрузке или ошибке валидации.
 */
static void ota_restart_with_delay(void)
{
    ota_restart_helper(NULL);
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    static const char hex[] = "0123456789abcdef";
    for (int i = 0; i < HASH_LEN; ++i) {
        hash_print[i * 2]     = hex[image_hash[i] >> 4];
        hash_print[i * 2 + 1] = hex[image_hash[i] & 0xf];
    }
    hash_print[HASH_LEN * 2] = '\0';
    ESP_LOGI(TAG, "%s: %s", label, hash_print);
}

/**
 * @brief Завершение OTA задачи без обновления
 *
 * Вызывается когда версия прошивки совпадает с уже установленной.
 * Просто завершаем задачу OTA - устройство продолжает работу.
 *
 * @param client HTTP клиент для очистки
 * @param update_handle Дескриптор OTA для прерывания
 */
static void ota_exit_no_update(esp_http_client_handle_t client, esp_ota_handle_t update_handle)
{
    ESP_LOGW(TAG, "Обновление не требуется - версия прошивки совпадает");
    ESP_LOGI(TAG, "Для загрузки новой прошивки разместите актуальный файл на сервере");

    // Сбрасываем флаг активной OTA, чтобы задача прогресса корректно завершилась
    atomic_store(&s_ota_active, false);
    atomic_store(&s_ota_downloaded_bytes, 0);
    atomic_store(&s_ota_total_bytes, 0);

    // Очищаем ресурсы
    if (update_handle != 0) {
        esp_ota_abort(update_handle);
    }
    http_cleanup(client);

    // Публикуем статус что обновление не требуется
    mqtt_client_publish_ota_status("no_update");

    // Удаляем задачу из TWDT перед удалением, чтобы WDT не ждал сброс от несуществующей задачи
    esp_task_wdt_delete(NULL);

    // Завершаем задачу OTA - устройство продолжает работу
    // Оборудование и ADS1115 НЕ были остановлены (версии совпали — ничего не трогали)
    vTaskDelete(NULL);
}

static void ota_task(void *pvParameter)
{
    esp_err_t err;
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;
    char ota_write_data[BUFFSIZE + 1];  ///< Буфер загрузки (на стеке — экономит .bss)

    ESP_LOGI(TAG, "Запуск задачи OTA-обновления");

    // Добавляем эту задачу в TWDT для предотвращения перезагрузки по watchdog
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "Не удалось добавить OTA задачу в watchdog");
    }

    // Публикуем текущую версию прошивки
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(esp_ota_get_running_partition(), &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Текущая версия прошивки: %s", running_app_info.version);
        mqtt_client_publish_firmware_version(running_app_info.version);
    }
    
    // Инициализируем переменные прогресса
    atomic_store(&s_ota_downloaded_bytes, 0);
    atomic_store(&s_ota_total_bytes, 0);
    // s_ota_active уже установлен в ota_start_task()

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running) {
        ESP_LOGW(TAG, "Настроенный OTA-раздел по смещению 0x%08" PRIx32 ", но работает с 0x%08" PRIx32,
                 configured->address, running->address);
    }
    ESP_LOGI(TAG, "Тип раздела %d подтип %d (смещение 0x%08" PRIx32 ")",
             running->type, running->subtype, running->address);

    esp_http_client_config_t config = {
        .url = CONFIG_EXAMPLE_FIRMWARE_UPG_URL,
        .cert_pem = (char *)server_cert_pem_start,
        .timeout_ms = CONFIG_EXAMPLE_OTA_RECV_TIMEOUT,
        .keep_alive_enable = true,
    };

#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    config.skip_cert_common_name_check = true;
#endif

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Не удалось инициализировать HTTP соединение");
        task_fatal_error();
    }

    err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось открыть HTTP соединение: %s", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
    }
    
    // Получаем длину файла из HTTP заголовков.
    // esp_http_client_fetch_headers() может вернуть -1 при ошибке (нет заголовка Content-Length).
    // НЕ сохраняем отрицательное значение в s_ota_total_bytes — иначе прогресс
    // (downloaded * 100) / (size_t)-1 даст garbage (огромное число).
    int total = esp_http_client_fetch_headers(client);
    if (total <= 0) {
        ESP_LOGE(TAG, "Не удалось определить размер прошивки (total=%d) — отмена OTA", total);
        http_cleanup(client);
        task_fatal_error();
    }
    atomic_store(&s_ota_total_bytes, total);
    ESP_LOGI(TAG, "Размер прошивки: %d байт (%.2f KB)", total, total / 1024.0);

    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "Не найден OTA раздел для обновления");
        http_cleanup(client);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Запись в раздел подтипа %d по смещению 0x%" PRIx32,
             update_partition->subtype, update_partition->address);

    size_t binary_file_length = 0;
    bool image_header_was_checked = false;
    int last_progress = 0;

    while (1) {
        // Сброс watchdog в каждом цикле
        esp_task_wdt_reset();

        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);

        if (data_read < 0) {
            ESP_LOGE(TAG, "Ошибка: ошибка чтения SSL данных");
            http_cleanup(client);
            if (update_handle != 0) {
                esp_ota_abort(update_handle);
            }
            task_fatal_error();
        }
        else if (data_read > 0) {
            if (image_header_was_checked == false) {
                esp_app_desc_t new_app_info;

                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "Версия новой прошивки: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Версия текущей прошивки: %s", running_app_info.version);
                    }

                    const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Версия последней невалидной прошивки: %s", invalid_app_info.version);
                    }

                    if (last_invalid_app != NULL) {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                            ESP_LOGW(TAG, "Новая версия совпадает с невалидной версией.");
                            ESP_LOGE(TAG, "Ранее была попытка запустить прошивку версии %s, но она не удалась", invalid_app_info.version);
                            ota_exit_no_update(client, update_handle);
                        }
                    }

#ifndef CONFIG_EXAMPLE_SKIP_VERSION_CHECK
                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0) {
                        ESP_LOGW(TAG, "Текущая версия совпадает с новой. Обновление не будет продолжено.");
                        ESP_LOGI(TAG, "Текущая версия: %s, Новая версия: %s", running_app_info.version, new_app_info.version);
                        ota_exit_no_update(client, update_handle);
                    }
#endif

                    image_header_was_checked = true;

                    // ============================================================
                    // Версии различаются — обновление будет выполнено!
                    // Теперь безопасно остановить оборудование и ADS1115
                    // ============================================================

                    // БЕЗОПАСНОСТЬ: Отключаем насос, свет и клапан перед записью во flash
                    ESP_LOGW(TAG, "БЕЗОПАСНОСТЬ: Отключение насоса, света и клапана перед OTA...");
                    device_control_set_pump_state(false);
                    device_control_set_light_state(false);
                    device_control_set_valve_state(false);
                    vTaskDelay(pdMS_TO_TICKS(100));  // Даём время на отключение реле

                    // Корректная остановка ADS1115 через флаг
                    ESP_LOGI(TAG, "Останавливаем ADS1115...");
                    device_control_ads1115_stop_for_ota();
                    
                    // Ждём небольшую задержку чтобы задача ADS1115 успела обработать флаг
                    vTaskDelay(pdMS_TO_TICKS(200));
                    
                    ESP_LOGI(TAG, "ADS1115 остановлен, начинаем OTA обновление...");

                    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Ошибка начала OTA (%s)", esp_err_to_name(err));
                        http_cleanup(client);
                        // esp_ota_abort(update_handle) - не вызываем, хендл ещё не создан
                        task_fatal_error();
                    }
                    ESP_LOGI(TAG, "Начало OTA-обновления прошло успешно");
                }
                else {
                    ESP_LOGE(TAG, "Полученный пакет не соответствует ожидаемому размеру");
                    http_cleanup(client);
                    task_fatal_error();
                    // task_fatal_error() вызывает esp_restart() и не возвращается
                }
            }

            // update_handle гарантированно != 0 здесь (установлен в esp_ota_begin выше)
            err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK) {
                http_cleanup(client);
                esp_ota_abort(update_handle);
                task_fatal_error();
            }
            binary_file_length += data_read;
            atomic_store(&s_ota_downloaded_bytes, (int)binary_file_length);

            // Выводим прогресс каждые 10%
            int current_total = atomic_load(&s_ota_total_bytes);
            if (current_total > 0) {
                int progress = (int)((binary_file_length * 100) / (size_t)current_total);
                if (progress != last_progress) {
                    ESP_LOGI(TAG, "Прогресс загрузки: %d%% (%zu/%d байт)",
                            progress, binary_file_length, current_total);
                    last_progress = progress;
                }
            }
            ESP_LOGD(TAG, "Длина записанного образа %zu", binary_file_length);
        }
        else {
            if (errno == ECONNRESET || errno == ENOTCONN) {
                ESP_LOGE(TAG, "Соединение закрыто, errno = %d", errno);
                break;
            }
            if (esp_http_client_is_complete_data_received(client) == true) {
                ESP_LOGI(TAG, "Соединение закрыто");
                break;
            }
            // Уступаем CPU если данных нет — предотвращаем busy-wait
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    ESP_LOGI(TAG, "Общая длина записанных бинарных данных: %zu", binary_file_length);

    if (esp_http_client_is_complete_data_received(client) != true) {
        ESP_LOGE(TAG, "Ошибка при получении полного файла");
        ESP_LOGE(TAG, "Загружено: %zu байт из %d", binary_file_length, atomic_load(&s_ota_total_bytes));
        
        // Публикуем ошибку
        mqtt_client_publish_ota_status("error_incomplete");
        
        http_cleanup(client);
        esp_ota_abort(update_handle);

        ota_restart_with_delay();
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Ошибка валидации образа, образ поврежден");
            mqtt_client_publish_ota_status("error_validation");
        }
        else {
            ESP_LOGE(TAG, "Ошибка завершения OTA (%s)!", esp_err_to_name(err));
            mqtt_client_publish_ota_status("error_finalize");
        }
        
        http_cleanup(client);

        ota_restart_with_delay();
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки загрузочного раздела (%s)!", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Подготовка к перезагрузке системы!");
    
    // Сбрасываем флаг активной OTA
    atomic_store(&s_ota_active, false);
    atomic_store(&s_ota_downloaded_bytes, 0);
    atomic_store(&s_ota_total_bytes, 0);
    
    esp_restart();
}

// ============================================================================
// ФУНКЦИИ ДЛЯ ПОЛУЧЕНИЯ ПРОГРЕССА OTA
// ============================================================================

/**
 * @brief Получить текущий прогресс OTA обновления в процентах
 * @return Прогресс от 0 до 100, или -1 если OTA не активна
 */
int ota_get_progress_percent(void)
{
    if (!atomic_load(&s_ota_active) || atomic_load(&s_ota_total_bytes) <= 0) {
        return -1;  // OTA не активна
    }
    return (atomic_load(&s_ota_downloaded_bytes) * 100) / atomic_load(&s_ota_total_bytes);
}

int ota_get_total_size(void)
{
    return atomic_load(&s_ota_total_bytes);
}

int ota_get_downloaded_bytes(void)
{
    return atomic_load(&s_ota_downloaded_bytes);
}

bool ota_is_active(void)
{
    return atomic_load(&s_ota_active);
}

bool ota_diagnostic(void)
{
    // ПРИМЕЧАНИЕ: GPIO 34-39 на ESP32 — input-only и не имеют внутренних подтяжек.
    // Установлено GPIO_PULLUP_DISABLE — требуется внешний подтягивающий резистор (10kΩ к VCC).
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    // Проверяем результат gpio_config — ошибка означает что диагностику доверять нельзя
    esp_err_t gpio_err = gpio_config(&io_conf);
    if (gpio_err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка конфигурации GPIO диагностики (%d): %s — диагностика провалена",
                 CONFIG_EXAMPLE_GPIO_DIAGNOSTIC, esp_err_to_name(gpio_err));
        return false;  // Безопасный fallback — откат прошивки
    }

    ESP_LOGI(TAG, "Диагностика (5 сек)...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    bool diagnostic_is_ok = gpio_get_level(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    gpio_reset_pin(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    return diagnostic_is_ok;
}

void ota_check_and_diagnose(void)
{
    ESP_LOGI(TAG, "Проверка состояния прошивки");

    uint8_t sha_256[HASH_LEN] = {0};

    // SHA-256 таблицы разделов — используем esp_partition_find_first
    const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
    if (part != NULL) {
        esp_partition_get_sha256(part, sha_256);
        print_sha256(sha_256, "SHA-256 для раздела NVS");
    }

    // SHA-256 загрузчика
    part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (part == NULL) {
        // Factory может отсутствовать — ищем первый app
        part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
    }
    if (part != NULL) {
        esp_partition_get_sha256(part, sha_256);
        print_sha256(sha_256, "SHA-256 для загрузочного раздела");
    }

    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 для текущей прошивки");

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            bool diagnostic_is_ok = ota_diagnostic();
            if (diagnostic_is_ok) {
                ESP_LOGI(TAG, "Диагностика завершена успешно! Подтверждаем прошивку...");
                esp_ota_mark_app_valid_cancel_rollback();
            }
            else {
                ESP_LOGE(TAG, "Диагностика не удалась! Откат к предыдущей версии...");
                esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }
}

void ota_init(void)
{
    ESP_LOGI(TAG, "Инициализация NVS");

    // Инициализация NVS с восстановлением при ошибках
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS требует очистки (переполнение или новая версия). Предыдущие данные будут потеряны.");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Критическая ошибка NVS: %s. Попытка восстановления...", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_LOGI(TAG, "NVS восстановлен");
    }
}

esp_err_t ota_start_task(void)
{
    // Атомарная попытка захватить право на запуск OTA — предотвращает race condition
    // между двумя одновременными вызовами (например, MQTT + веб-интерфейс)
    bool expected = false;
    if (!atomic_compare_exchange_strong(&s_ota_active, &expected, true)) {
        ESP_LOGW(TAG, "OTA уже выполняется, игнорирую команду");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t res = xTaskCreatePinnedToCore(&ota_task, "ota_task", 8192, NULL, 5, NULL, PRO_CPU_NUM);
    if (res != pdPASS) {
        ESP_LOGE(TAG, "Не удалось создать задачу OTA: %d", res);
        atomic_store(&s_ota_active, false);
        mqtt_client_publish_ota_status("failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Задача OTA создана успешно");
    return ESP_OK;
}
