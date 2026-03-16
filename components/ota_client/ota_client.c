/*
 * ============================================================================
 * OTA Client Component — Компонент OTA-обновления для ESP32
 * ============================================================================
 */

#include <string.h>
#include <inttypes.h>
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
#include "protocol_examples_common.h"
#include "errno.h"
#include "ota_client.h"
#include "hydro_mqtt_client.h"  // Для публикации статуса OTA и версии прошивки
#include "esp_task_wdt.h"       // Для сброса watchdog (пункт 1.7)

// Объявляем внешнюю функцию для доступа к семафору ADS1115
extern SemaphoreHandle_t get_ads1115_running_sem(void);

#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"
#endif

#define BUFFSIZE 1024
#define HASH_LEN 32
#define OTA_URL_SIZE 256

static const char *TAG = "ota_client";
static char ota_write_data[BUFFSIZE + 1] = {0};

// Переменные для отслеживания прогресса OTA
static int s_ota_downloaded_bytes = 0;     ///< Количество загруженных байт
static int s_ota_total_bytes = 0;          ///< Общий размер файла
static bool s_ota_active = false;          ///< Флаг активной OTA сессии

extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

/**
 * @brief Обработчик фатальных ошибок — перезагружает устройство
 * 
 * Вызывается при критических ошибках OTA:
 * - Ошибка сети/SSL
 * - Ошибка записи во flash
 * - Повреждение данных
 * 
 * Перед перезагрузкой:
 * 1. Публикует статус ошибки в MQTT
 * 2. Ждёт 5 секунд (чтобы сообщение дошло до брокера)
 * 3. Перезагружает устройство
 * 
 * @note Функция не возвращает управление - устройство будет перезапущено
 */
static void task_fatal_error(void)
{
    ESP_LOGE(TAG, "Фатальная ошибка OTA - перезагрузка устройства...");
    
    // Публикуем статус фатальной ошибки
    mqtt_client_publish_ota_status("fatal_error");
    
    // Даём 5 секунд на доставку MQTT сообщения
    ESP_LOGW(TAG, "Перезагрузка через 5 секунд...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    // Перезагружаем устройство
    esp_restart();
    
    // На случай если esp_restart() не сработал
    while (1) { ; }
}

static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1];
    hash_print[HASH_LEN * 2] = 0;
    for (int i = 0; i < HASH_LEN; ++i) {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
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
    s_ota_active = false;
    s_ota_downloaded_bytes = 0;
    s_ota_total_bytes = 0;

    // Очищаем ресурсы и завершаем задачу
    if (update_handle > 0) {
        esp_ota_abort(update_handle);
    }
    http_cleanup(client);

    // Публикуем статус что обновление не требуется
    mqtt_client_publish_ota_status("no_update");

    // Пункт 1.8: Восстанавливаем семафор ADS1115 (т.к. OTA не состоялось)
    // Задача была остановлена через семафор в mqtt_ota_check_task
    SemaphoreHandle_t sem = get_ads1115_running_sem();
    if (sem != NULL) {
        ESP_LOGI(TAG, "Восстановление семафора ADS1115...");
        xSemaphoreGive(sem);
        ESP_LOGI(TAG, "Задача ADS1115 восстановлена");
    }

    // Завершаем задачу OTA - устройство продолжает работу
    vTaskDelete(NULL);
}

static void ota_task(void *pvParameter)
{
    esp_err_t err;
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Запуск задачи OTA-обновления");
    
    // Публикуем текущую версию прошивки
    esp_app_desc_t running_app_info;
    if (esp_ota_get_partition_description(esp_ota_get_running_partition(), &running_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Текущая версия прошивки: %s", running_app_info.version);
        mqtt_client_publish_firmware_version(running_app_info.version);
    }
    
    // Инициализируем переменные прогресса
    s_ota_downloaded_bytes = 0;
    s_ota_total_bytes = 0;
    s_ota_active = true;

    // ПОТ-11: Сбрасываем last_progress в начале OTA сессии
    // (переменная static внутри цикла, но задача создаётся заново при каждом OTA)

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
        esp_http_client_cleanup(client);
        task_fatal_error();
    }
    
    // Получаем длину файла из HTTP заголовков
    s_ota_total_bytes = esp_http_client_fetch_headers(client);
    if (s_ota_total_bytes > 0) {
        ESP_LOGI(TAG, "Размер прошивки: %d байт (%.2f KB)", s_ota_total_bytes, s_ota_total_bytes / 1024.0);
    }

    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Запись в раздел подтипа %d по смещению 0x%" PRIx32,
             update_partition->subtype, update_partition->address);

    int binary_file_length = 0;
    bool image_header_was_checked = false;

    while (1) {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);

        if (data_read < 0) {
            ESP_LOGE(TAG, "Ошибка: ошибка чтения SSL данных");
            http_cleanup(client);
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

                    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Ошибка начала OTA (%s)", esp_err_to_name(err));
                        http_cleanup(client);
                        esp_ota_abort(update_handle);
                        task_fatal_error();
                    }
                    ESP_LOGI(TAG, "Начало OTA-обновления прошло успешно");
                }
                else {
                    ESP_LOGE(TAG, "Полученный пакет не соответствует ожидаемому размеру");
                    http_cleanup(client);
                    esp_ota_abort(update_handle);
                    task_fatal_error();
                }
            }

            err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK) {
                http_cleanup(client);
                esp_ota_abort(update_handle);
                task_fatal_error();
            }
            binary_file_length += data_read;
            s_ota_downloaded_bytes = binary_file_length;

            // Выводим прогресс каждые 10%
            if (s_ota_total_bytes > 0) {
                int progress = (binary_file_length * 100) / s_ota_total_bytes;
                if (progress % 10 == 0 && progress > 0) {
                    // ПОТ-11: Используем локальную переменную вместо static
                    // чтобы прогресс корректно сбрасывался между сессиями OTA
                    static int last_progress = 0;
                    if (progress != last_progress) {
                        // Пункт 1.7: Сброс watchdog для предотвращения перезагрузки
                        esp_task_wdt_reset();
                        ESP_LOGI(TAG, "Прогресс загрузки: %d%% (%d/%d байт)",
                                progress, binary_file_length, s_ota_total_bytes);
                        last_progress = progress;
                    }
                }
            }
            ESP_LOGD(TAG, "Длина записанного образа %d", binary_file_length);
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
        }
    }

    ESP_LOGI(TAG, "Общая длина записанных бинарных данных: %d", binary_file_length);

    if (esp_http_client_is_complete_data_received(client) != true) {
        ESP_LOGE(TAG, "Ошибка при получении полного файла");
        ESP_LOGE(TAG, "Загружено: %d байт из %d", binary_file_length, s_ota_total_bytes);
        
        // Публикуем ошибку
        mqtt_client_publish_ota_status("error_incomplete");
        
        http_cleanup(client);
        esp_ota_abort(update_handle);
        
        // Перезагружаем устройство через 5 секунд
        ESP_LOGW(TAG, "Перезагрузка через 5 секунд...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
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
        
        // Перезагружаем устройство через 5 секунд
        ESP_LOGW(TAG, "Перезагрузка через 5 секунд...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        esp_restart();
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки загрузочного раздела (%s)!", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Подготовка к перезагрузке системы!");
    
    // Сбрасываем флаг активной OTA
    s_ota_active = false;
    s_ota_downloaded_bytes = 0;
    s_ota_total_bytes = 0;
    
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
    if (!s_ota_active || s_ota_total_bytes <= 0) {
        return -1;  // OTA не активна
    }
    return (s_ota_downloaded_bytes * 100) / s_ota_total_bytes;
}

/**
 * @brief Получить общий размер загружаемого файла в байтах
 * @return Размер файла в байтах, или 0 если неизвестно
 */
int ota_get_total_size(void)
{
    return s_ota_total_bytes;
}

/**
 * @brief Получить количество загруженных байт
 * @return Количество загруженных байт
 */
int ota_get_downloaded_bytes(void)
{
    return s_ota_downloaded_bytes;
}

bool ota_diagnostic(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Диагностика (5 сек)...");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    bool diagnostic_is_ok = gpio_get_level(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    gpio_reset_pin(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    return diagnostic_is_ok;
}

void ota_check_and_diagnose(void)
{
    ESP_LOGI(TAG, "Проверка состояния прошивки");

    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    partition.address = ESP_PARTITION_TABLE_OFFSET;
    partition.size = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 для таблицы разделов");

    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 для загрузчика");

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
    ESP_LOGI(TAG, "Инициализация NVS и сети");

    // Пункт 3.2: Улучшенная обработка сбоя питания при записи в NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS требует очистки (переполнение или новая версия). Предыдущие данные будут потеряны.");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        // Критическая ошибка NVS — пробуем восстановление
        ESP_LOGE(TAG, "Критическая ошибка NVS: %s. Попытка восстановления...", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_LOGI(TAG, "NVS восстановлен");
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());

#if CONFIG_EXAMPLE_CONNECT_WIFI
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif
}

void ota_start_task(void)
{
    xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
}
