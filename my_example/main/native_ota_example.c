/*
 * ============================================================================
 * Native OTA Example — Пример OTA-обновления для ESP32
 * ============================================================================
 *
 * Этот код реализует безопасное обновление прошивки "по воздуху" (OTA — Over-The-Air).
 *
 * Как это работает:
 * 1. Устройство подключается к Wi-Fi/Ethernet
 * 2. Загружает новую прошивку с HTTPS-сервера
 * 3. Записывает её во второй раздел flash-памяти (OTA partition)
 * 4. Перезагружается с новой прошивкой
 * 5. Проверяет работоспособность (диагностика 5 сек)
 * 6. Если всё хорошо — подтверждает прошивку, иначе — откатывается назад
 *
 * Архитектура разделов flash-памяти:
 * - Загрузчик (Bootloader)
 * - Таблица разделов (Partition Table)
 * - OTA Partition 0 (текущая прошивка)
 * - OTA Partition 1 (резервная/обновляемая прошивка)
 * - NVS (Non-Volatile Storage — энергонезависимое хранилище)
 *
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

#if CONFIG_EXAMPLE_CONNECT_WIFI
#include "esp_wifi.h"
#endif

/*
 * Размер буфера для чтения данных из сети.
 * Чем больше буфер — тем быстрее загрузка, но больше потребление RAM.
 * 1024 байта — безопасное значение для ESP32.
 */
#define BUFFSIZE 1024

/* Длина SHA-256 хеша в байтах (256 бит = 32 байта) */
#define HASH_LEN 32

/* Тег для логирования — отображается в начале каждого лога */
static const char *TAG = "native_ota_example";

/*
 * Буфер для временного хранения данных, полученных от сервера.
 * Данные читаются порциями по BUFFSIZE байт и записываются во flash.
 * +1 байт для возможного нуль-терминатора (для безопасности).
 */
static char ota_write_data[BUFFSIZE + 1] = {0};

/*
 * SSL-сертификат сервера обновлений.
 * Встроен в бинарник как внешние символы из файла сертификата.
 * Используется для проверки подлинности HTTPS-сервера.
 *
 * Файл сертификата должен быть скомпилирован в проект как binary data.
 */
extern const uint8_t server_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_ca_cert_pem_end");

/* Максимальный размер URL для загрузки прошивки */
#define OTA_URL_SIZE 256

/**
 * @brief Освобождает ресурсы HTTP-клиента
 *
 * Закрывает TCP-соединение и освобождает память, выделенную для клиента.
 * Вызывается при завершении загрузки или при ошибке.
 *
 * @param client Дескриптор HTTP-клиента для очистки
 */
static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);   // Закрываем TCP-соединение
    esp_http_client_cleanup(client); // Освобождаем память клиента
}

/**
 * @brief Обработчик фатальных ошибок — останавливает задачу
 *
 * Вызывается при критических ошибках, которые делают невозможным
 * продолжение работы (ошибка сети, повреждение данных, ошибка записи).
 *
 * Почему не перезагрузка? — Чтобы не войти в цикл бесконечных перезагрузок
 * при постоянной ошибке. Устройство "зависает", требуя ручного вмешательства.
 *
 * Атрибут noreturn подсказывает компилятору, что функция не возвращает управление.
 */
static void __attribute__((noreturn)) task_fatal_error(void)
{
    ESP_LOGE(TAG, "Выход из задачи из-за фатальной ошибки...");
    vTaskDelete(NULL); // Удаляем текущую задачу из планировщика FreeRTOS

    // Бесконечный цикл — процессор останавливается здесь
    // Требуется физическая перезагрузка или сброс
    while (1)
    {
        ;
    }
}

/**
 * @brief Выводит SHA-256 хеш в консоль в шестнадцатеричном формате
 *
 * SHA-256 используется для проверки целостности:
 * - таблицы разделов
 * - загрузчика
 * - текущей прошивки
 *
 * @param image_hash Массив из 32 байт с бинарным хешем
 * @param label Текстовая метка для вывода (например, "SHA-256 для прошивки")
 */
static void print_sha256(const uint8_t *image_hash, const char *label)
{
    char hash_print[HASH_LEN * 2 + 1]; // 64 символа hex + нуль-терминатор
    hash_print[HASH_LEN * 2] = 0;

    // Преобразуем каждый байт в 2 hex-символа (0x00 -> "00")
    for (int i = 0; i < HASH_LEN; ++i)
    {
        sprintf(&hash_print[i * 2], "%02x", image_hash[i]);
    }
    ESP_LOGI(TAG, "%s: %s", label, hash_print);
}

/**
 * @brief Бесконечный цикл ожидания — используется при обнаружении проблем
 *
 * Вызывается в двух случаях:
 * 1. Версия прошивки совпадает с уже установленной (обновление не нужно)
 * 2. Версия совпадает с последней "невалидной" (защита от цикла откатов)
 *
 * Устройство продолжает работать, но не обновляется.
 * Пользователь должен сбросить устройство вручную для повторной попытки.
 */
static void infinite_loop(void)
{
    int i = 0;
    ESP_LOGI(TAG, "Когда новая прошивка доступна на сервере, нажмите кнопку сброса для загрузки");
    while (1)
    {
        ESP_LOGI(TAG, "Ожидание новой прошивки ... %d", ++i);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Пауза 2 секунды между сообщениями
    }
}

/**
 * @brief Основная задача OTA-обновления
 *
 * Это главная функция, которая выполняет весь процесс обновления:
 * 1. Подключается к серверу обновлений по HTTPS
 * 2. Загружает бинарный файл прошивки порциями
 * 3. Проверяет заголовок прошивки (версия, совместимость)
 * 4. Записывает данные во flash-память (в OTA partition)
 * 5. Проверяет целостность записанного образа
 * 6. Устанавливает новый раздел как загрузочный
 * 7. Перезагружает устройство
 *
 * Задача работает в отдельном потоке FreeRTOS с приоритетом 5.
 * После завершения обновления задача завершается.
 *
 * @param pvParameter Параметры задачи (не используются, NULL)
 */
static void ota_example_task(void *pvParameter)
{
    esp_err_t err;                                  // Код ошибки ESP-IDF
    esp_ota_handle_t update_handle = 0;             // Дескриптор сессии OTA-записи
    const esp_partition_t *update_partition = NULL; // Указатель на раздел для записи

    ESP_LOGI(TAG, "Запуск задачи OTA-обновления");

    /*
     * Получаем информацию о разделах:
     * - configured: раздел, который выбран для загрузки в NVRAM
     * - running: раздел, с которого устройство загрузилось фактически
     *
     * В норме они должны совпадать. Если нет — возможно, данные NVRAM
     * повреждены или был сбой питания во время предыдущего обновления.
     */
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running)
    {
        ESP_LOGW(TAG, "Настроенный OTA-раздел загрузки по смещению 0x%08" PRIx32 ", но работает с 0x%08" PRIx32,
                 configured->address, running->address);
        ESP_LOGW(TAG, "(Это может произойти, если данные OTA-загрузки или предпочтительный образ загрузки каким-то образом повреждены.)");
    }
    ESP_LOGI(TAG, "Тип раздела %d подтип %d (смещение 0x%08" PRIx32 ")",
             running->type, running->subtype, running->address);

    /*
     * Конфигурация HTTP-клиента для загрузки прошивки:
     * - url: адрес сервера с бинарником прошивки (из menuconfig)
     * - cert_pem: SSL-сертификат для проверки сервера
     * - timeout_ms: максимальное время ожидания ответа
     * - keep_alive_enable: держать TCP-соединение открытым
     */
    esp_http_client_config_t config = {
        .url = CONFIG_EXAMPLE_FIRMWARE_UPG_URL,
        .cert_pem = (char *)server_cert_pem_start,
        .timeout_ms = CONFIG_EXAMPLE_OTA_RECV_TIMEOUT,
        .keep_alive_enable = true,
    };

#ifdef CONFIG_EXAMPLE_FIRMWARE_UPGRADE_URL_FROM_STDIN
    /*
     * Опция для чтения URL из стандартного ввода (stdin).
     * Используется редко — например, для тестирования с консоли.
     *
     * Если URL = "FROM_STDIN", читаем реальный URL из UART.
     */
    char url_buf[OTA_URL_SIZE];
    if (strcmp(config.url, "FROM_STDIN") == 0)
    {
        example_configure_stdin_stdout();
        fgets(url_buf, OTA_URL_SIZE, stdin);
        int len = strlen(url_buf);
        url_buf[len - 1] = '\0'; // Удаляем символ новой строки
        config.url = url_buf;
    }
    else
    {
        ESP_LOGE(TAG, "Несоответствие конфигурации: неверный URL образа прошивки для обновления");
        abort();
    }
#endif

#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
    /*
     * Опция для отключения проверки CN (Common Name) в SSL-сертификате.
     *
     * В продакшене НЕ рекомендуется отключать — это снижает безопасность.
     * Используйте только для тестирования с самоподписанными сертификатами.
     */
    config.skip_cert_common_name_check = true;
#endif

    /*
     * Инициализация HTTP-клиента.
     * Выделяется память для буферов и структуры клиента.
     */
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        ESP_LOGE(TAG, "Не удалось инициализировать HTTP соединение");
        task_fatal_error();
    }

    /*
     * Открываем TCP-соединение с сервером.
     * Выполняется SSL/TLS рукопожатие для HTTPS.
     */
    err = esp_http_client_open(client, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Не удалось открыть HTTP соединение: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        task_fatal_error();
    }
    esp_http_client_fetch_headers(client); // Читаем HTTP-заголовки ответа

    /*
     * Получаем следующий доступный OTA-раздел для записи.
     *
     * ESP32 поддерживает 2 OTA-раздела:
     * - OTA_0 (factory или текущий)
     * - OTA_1 (альтернативный для обновления)
     *
     * esp_ota_get_next_update_partition() возвращает раздел,
     * отличный от текущего запущенного.
     */
    update_partition = esp_ota_get_next_update_partition(NULL);
    assert(update_partition != NULL);
    ESP_LOGI(TAG, "Запись в раздел подтипа %d по смещению 0x%" PRIx32,
             update_partition->subtype, update_partition->address);

    int binary_file_length = 0;            // Счётчик записанных байт
    bool image_header_was_checked = false; // Флаг: проверен ли заголовок прошивки

    /*
     * Основной цикл загрузки прошивки.
     *
     * Работает по принципу потоковой загрузки:
     * - Читаем порцию данных из HTTP-ответа (до 1024 байт)
     * - Сразу записываем во flash-память
     * - Повторяем, пока не получим весь файл
     *
     * Преимущества такого подхода:
     * - Не требуется много RAM для хранения всего файла
     * - Можно обновлять прошивки любого размера
     * - Ошибка на любом этапе безопасно прерывает процесс
     */
    while (1)
    {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);

        if (data_read < 0)
        {
            /*
             * Отрицательное значение = ошибка сети/SSL.
             *
             * Возможные причины:
             * - Обрыв соединения
             * - Ошибка SSL-рукопожатия
             * - Таймаут ответа сервера
             */
            ESP_LOGE(TAG, "Ошибка: ошибка чтения SSL данных");
            http_cleanup(client);
            task_fatal_error();
        }
        else if (data_read > 0)
        {
            /*
             * Положительное значение = получены данные для записи.
             *
             * ПЕРВЫЙ ПАКЕТ: нужно проверить заголовок прошивки
             * ПОСЛЕДУЮЩИЕ ПАКЕТЫ: просто записываем данные
             */
            if (image_header_was_checked == false)
            {
                esp_app_desc_t new_app_info; // Структура с информацией о прошивке

                /*
                 * Проверяем, что получили достаточно данных для анализа заголовка.
                 *
                 * Структура бинарника прошивки:
                 * [Image Header (8 байт)] [Segment Header (8 байт)] [App Description (256 байт)]
                 *
                 * Нужно минимум 272 байта для чтения esp_app_desc_t
                 */
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                {
                    /*
                     * Извлекаем esp_app_desc_t из потока данных.
                     * Эта структура содержит:
                     * - version: строка версии прошивки
                     * - secure_version: версия безопасности
                     * - app_elf_sha256: хеш ELF-файла
                     * - и другие метаданные
                     */
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "Версия новой прошивки: %s", new_app_info.version);

                    /*
                     * Получаем информацию о текущей запущенной прошивке для сравнения.
                     */
                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Версия текущей прошивки: %s", running_app_info.version);
                    }

                    /*
                     * Получаем информацию о последней "невалидной" прошивке.
                     *
                     * Невалидная = та, которая не прошла диагностику после обновления.
                     * ESP32 хранит её версию для защиты от "цикла откатов".
                     */
                    const esp_partition_t *last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK)
                    {
                        ESP_LOGI(TAG, "Версия последней невалидной прошивки: %s", invalid_app_info.version);
                    }

                    /*
                     * ПРОВЕРКА 1: Не пытаемся ли загрузить ту же версию, что уже сломалась?
                     *
                     * Если версия совпадает с последней невалидной — отказываемся обновлять.
                     * Это защищает от бесконечного цикла: загрузка → сбой → откат → загрузка...
                     */
                    if (last_invalid_app != NULL)
                    {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0)
                        {
                            ESP_LOGW(TAG, "Новая версия совпадает с невалидной версией.");
                            ESP_LOGW(TAG, "Ранее была попытка запустить прошивку версии %s, но она не удалась.", invalid_app_info.version);
                            ESP_LOGW(TAG, "Прошивка была откачена к предыдущей версии.");
                            http_cleanup(client);
                            infinite_loop(); // Зависаем в ожидании ручного сброса
                        }
                    }

#ifndef CONFIG_EXAMPLE_SKIP_VERSION_CHECK
                    /*
                     * ПРОВЕРКА 2: Не пытаемся ли загрузить ту же версию, что уже работает?
                     *
                     * Бессмысленно обновлять на ту же версию — трата времени и ресурса flash.
                     *
                     * Можно отключить через CONFIG_EXAMPLE_SKIP_VERSION_CHECK, если нужно
                     * принудительное обновление (например, для перепрошивки с теми же версиями).
                     */
                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0)
                    {
                        ESP_LOGW(TAG, "Текущая работающая версия совпадает с новой. Обновление не будет продолжено.");
                        http_cleanup(client);
                        infinite_loop();
                    }
#endif

                    /* Заголовок проверен — можно начинать запись */
                    image_header_was_checked = true;

                    /*
                     * Инициализируем сессию OTA-записи.
                     *
                     * OTA_WITH_SEQUENTIAL_WRITES — оптимизация для последовательной записи.
                     * update_handle — дескриптор, который нужно передать в esp_ota_write().
                     */
                    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Ошибка начала OTA (%s)", esp_err_to_name(err));
                        http_cleanup(client);
                        esp_ota_abort(update_handle);
                        task_fatal_error();
                    }
                    ESP_LOGI(TAG, "Начало OTA-обновления прошло успешно");
                }
                else
                {
                    /*
                     * Первый пакет слишком маленький — не хватает данных заголовка.
                     * Это аномалия — возможно, повреждён файл или ошибка сети.
                     */
                    ESP_LOGE(TAG, "Полученный пакет не соответствует ожидаемому размеру");
                    http_cleanup(client);
                    esp_ota_abort(update_handle);
                    task_fatal_error();
                }
            }

            /*
             * ЗАПИСЬ ДАННЫХ ВО FLASH
             *
             * esp_ota_write() принимает буфер и размер, записывает в текущую
             * позицию OTA-раздела. Внутренне ведётся подсчёт контрольной суммы.
             */
            err = esp_ota_write(update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK)
            {
                http_cleanup(client);
                esp_ota_abort(update_handle);
                task_fatal_error();
            }
            binary_file_length += data_read;
            ESP_LOGD(TAG, "Длина записанного образа %d", binary_file_length);
        }
        else
        {
            /*
             * data_read == 0: данных нет.
             *
             * Возможные причины:
             * 1. Соединение разорвано (errno = ECONNRESET/ENOTCONN)
             * 2. Сервер завершил передачу (is_complete_data_received = true)
             * 3. Временная пауза в передаче (ожидание данных)
             */
            if (errno == ECONNRESET || errno == ENOTCONN)
            {
                ESP_LOGE(TAG, "Соединение закрыто, errno = %d", errno);
                break;
            }
            if (esp_http_client_is_complete_data_received(client) == true)
            {
                ESP_LOGI(TAG, "Соединение закрыто");
                break;
            }
        }
    }

    ESP_LOGI(TAG, "Общая длина записанных бинарных данных: %d", binary_file_length);

    /*
     * ПРОВЕРКА: весь ли файл получен?
     *
     * esp_http_client_is_complete_data_received() возвращает true,
     * если сервер передал все данные и соединение закрыто корректно.
     */
    if (esp_http_client_is_complete_data_received(client) != true)
    {
        ESP_LOGE(TAG, "Ошибка при получении полного файла");
        http_cleanup(client);
        esp_ota_abort(update_handle);
        task_fatal_error();
    }

    /*
     * ЗАВЕРШЕНИЕ OTA-СЕССИИ И ВАЛИДАЦИЯ
     *
     * esp_ota_end() выполняет:
     * 1. Закрывает дескриптор записи
     * 2. Вычисляет и проверяет контрольную сумму всего образа
     * 3. Проверяет структуру заголовков
     *
     * Если образ повреждён — вернёт ESP_ERR_OTA_VALIDATE_FAILED
     */
    err = esp_ota_end(update_handle);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED)
        {
            ESP_LOGE(TAG, "Ошибка валидации образа, образ поврежден");
        }
        else
        {
            ESP_LOGE(TAG, "Ошибка завершения OTA (%s)!", esp_err_to_name(err));
        }
        http_cleanup(client);
        task_fatal_error();
    }

    /*
     * УСТАНОВКА НОВОЙ ПРОШИВКИ КАК ЗАГРУЗОЧНОЙ
     *
     * esp_ota_set_boot_partition() записывает в NVRAM информацию о том,
     * с какого раздела загружаться при следующей перезагрузке.
     *
     * После этого вызова раздел помечается как "pending_verify" —
     * при следующей загрузке система будет ждать подтверждения работоспособности.
     */
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Ошибка установки загрузочного раздела (%s)!", esp_err_to_name(err));
        http_cleanup(client);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Подготовка к перезагрузке системы!");
    esp_restart(); // Перезагрузка устройства
    return;
}

/**
 * @brief Функция диагностики работоспособности после обновления
 *
 * Вызывается после первого запуска новой прошивки.
 * Даёт 5 секунд на проверку аппаратной части через GPIO-пин.
 *
 * Как работает:
 * 1. Настраивает диагностический GPIO на вход с подтяжкой к VCC
 * 2. Ждёт 5 секунд
 * 3. Читает уровень сигнала
 *
 * Если GPIO = HIGH (1) — диагностика успешна
 * Если GPIO = LOW (0) — проблема, нужен откат
 *
 * На практике можно подключить:
 * - Кнопку для ручного подтверждения
 * - Датчик для проверки работы устройства
 * - Сигнал от другой платы о состоянии системы
 *
 * @return true если диагностика успешна, false иначе
 */
static bool diagnostic(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;                           // Отключаем прерывания
    io_conf.mode = GPIO_MODE_INPUT;                                  // Режим входа
    io_conf.pin_bit_mask = (1ULL << CONFIG_EXAMPLE_GPIO_DIAGNOSTIC); // Используем GPIO для диагностики
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                    // Отключаем подтяжку к земле
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                         // Включаем подтяжку к питанию
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "Диагностика (5 сек)...");
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Ожидание 5 секунд для диагностики

    // Читаем уровень сигнала с диагностического GPIO
    bool diagnostic_is_ok = gpio_get_level(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);

    // Сбрасываем настройки GPIO
    gpio_reset_pin(CONFIG_EXAMPLE_GPIO_DIAGNOSTIC);
    return diagnostic_is_ok;
}

/**
 * @brief Главная функция приложения — точка входа
 *
 * Вызывается SDK после инициализации базовых систем.
 * Выполняет:
 * 1. Вывод хешей для отладки (таблица разделов, загрузчик, прошивка)
 * 2. Проверку состояния OTA (не требуется ли диагностика после обновления)
 * 3. Инициализацию NVS, сети, Wi-Fi
 * 4. Запуск задачи OTA-обновления
 */
void app_main(void)
{
    ESP_LOGI(TAG, "OTA пример app_main запуск");

    uint8_t sha_256[HASH_LEN] = {0};
    esp_partition_t partition;

    /*
     * Вычисляем и выводим SHA-256 хеши для отладки:
     *
     * Это помогает определить:
     * - Изменилась ли прошивка после обновления
     * - Не повреждена ли таблица разделов
     * - Соответствует ли загрузчик ожидаемому
     */

    // Хеш таблицы разделов
    partition.address = ESP_PARTITION_TABLE_OFFSET;
    partition.size = ESP_PARTITION_TABLE_MAX_LEN;
    partition.type = ESP_PARTITION_TYPE_DATA;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 для таблицы разделов: ");

    // Хеш загрузчика
    partition.address = ESP_BOOTLOADER_OFFSET;
    partition.size = ESP_PARTITION_TABLE_OFFSET;
    partition.type = ESP_PARTITION_TYPE_APP;
    esp_partition_get_sha256(&partition, sha_256);
    print_sha256(sha_256, "SHA-256 для загрузчика: ");

    // Хеш текущей прошивки
    esp_partition_get_sha256(esp_ota_get_running_partition(), sha_256);
    print_sha256(sha_256, "SHA-256 для текущей прошивки: ");

    /*
     * ПРОВЕРКА СОСТОЯНИЯ OTA-ОБНОВЛЕНИЯ
     *
     * После перезагрузки с новой прошивкой её статус = ESP_OTA_IMG_PENDING_VERIFY
     * Это значит: "Прошивка новая, нужно подтвердить её работоспособность"
     *
     * Если статус другой (ESP_OTA_IMG_VALID, ESP_OTA_IMG_INVALID) —
     * прошивка уже проверена ранее, диагностика не нужна.
     */
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK)
    {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            /*
             * Запускаем диагностику для проверки работоспособности.
             *
             * Возможны 2 исхода:
             * 1. diagnostic_is_ok = true → подтверждаем прошивку, отменяем откат
             * 2. diagnostic_is_ok = false → откатываемся к предыдущей версии
             */
            bool diagnostic_is_ok = diagnostic();
            if (diagnostic_is_ok)
            {
                ESP_LOGI(TAG, "Диагностика завершена успешно! Продолжаем выполнение ...");
                esp_ota_mark_app_valid_cancel_rollback(); // Подтверждаем валидность прошивки
            }
            else
            {
                ESP_LOGE(TAG, "Диагностика не удалась! Начинаем откат к предыдущей версии ...");
                esp_ota_mark_app_invalid_rollback_and_reboot(); // Откатываемся к предыдущей версии
            }
        }
    }

    /*
     * ИНИЦИАЛИЗАЦИЯ NVS (Non-Volatile Storage)
     *
     * NVS используется для:
     * - Хранения Wi-Fi кредов (SSID, пароль)
     * - Сохранения состояния OTA (какой раздел загрузочный)
     * - Пользовательских данных между перезагрузками
     *
     * При OTA размер NVS-раздела может отличаться от factory-прошивки.
     * Если формат не совпадает — очищаем и пересоздаём NVS.
     */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /*
     * ИНИЦИАЛИЗАЦИЯ СЕТЕВОГО СТЕКА
     *
     * esp_netif — абстрактный сетевой интерфейс ESP-IDF
     * esp_event_loop — цикл обработки сетевых событий (подключение, отключение, IP и т.д.)
     */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /*
     * ПОДКЛЮЧЕНИЕ К СЕТИ
     *
     * example_connect() — вспомогательная функция из protocol_examples_common.
     * Автоматически подключает Wi-Fi или Ethernet в зависимости от конфигурации.
     *
     * Блокирует выполнение до успешного получения IP-адреса.
     */
    ESP_ERROR_CHECK(example_connect());

#if CONFIG_EXAMPLE_CONNECT_WIFI
    /*
     * ОТКЛЮЧЕНИЕ ЭНЕРГОСБЕРЕЖЕНИЯ WI-FI
     *
     * WIFI_PS_NONE — максимальная производительность, минимальная задержка.
     *
     * Важно для OTA:
     * - Стабильное соединение без пауз на сон
     * - Максимальная скорость загрузки
     * - Меньше риск обрыва соединения
     */
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif // CONFIG_EXAMPLE_CONNECT_WIFI

    /*
     * ЗАПУСК ЗАДАЧИ OTA-ОБНОВЛЕНИЯ
     *
     * Создаём отдельную задачу FreeRTOS:
     * - ota_example_task — функция задачи
     * - "ota_example_task" — имя для отладки
     * - 8192 — размер стека в байтах (32 бита = 8 KB)
     * - приоритет 5 — выше среднего, но ниже системных
     *
     * Почему отдельная задача?
     * - OTA может длиться долго (секунды/минуты)
     * - Не блокирует основной поток
     * - Можно добавить мониторинг прогресса
     */
    xTaskCreate(&ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
}
