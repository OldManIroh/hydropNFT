/**
 * @file sntp_client.h
 * @brief Клиент SNTP для синхронизации времени с NTP серверами
 *
 * Этот компонент реализует получение и синхронизацию системного времени
 * через протокол SNTP (Simple Network Time Protocol).
 *
 * Функционал:
 * - Подключение к публичным NTP серверам (настраиваются через Kconfig)
 * - Автоматическая повторная попытка при ошибке
 * - Уведомление об успешной синхронизации через callback
 * - Получение отформатированной строки времени
 *
 * Пример использования:
 * @code
 * void app_main(void) {
 *     // Предполагается, что WiFi уже подключён (например, через mqtt_client)
 *
 *     // Инициализация SNTP
 *     sntp_client_init();
 *
 *     // Синхронизация времени (блокируется до успеха или таймаута)
 *     if (!sntp_time_sync()) {
 *         ESP_LOGE(TAG, "Не удалось синхронизировать время");
 *         return;
 *     }
 *
 *     // Теперь можно использовать время
 *     char time_str[32];
 *     if (sntp_client_get_time_string(time_str, sizeof(time_str))) {
 *         printf("Текущее время: %s\n", time_str);
 *     }
 * }
 * @endcode
 *
 * @author HydroNFT Team
 * @version 1.0
 * @date 2024
 */

#ifndef SNTP_CLIENT_H
#define SNTP_CLIENT_H

#include <stdbool.h>
#include <stddef.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация SNTP клиента
 *
 * Настраивает и запускает SNTP клиент для синхронизации времени.
 * Использует следующие NTP серверы (из Kconfig):
 * - CONFIG_SNTP_SERVER_1 (основной)
 * - CONFIG_SNTP_SERVER_2
 * - CONFIG_SNTP_SERVER_3
 * - CONFIG_SNTP_SERVER_4
 *
 * При успешной синхронизации вызывается callback функция,
 * которая логирует текущее время.
 *
 * @note Функция не блокирующая - синхронизация происходит в фоне
 * @note Требуется активное подключение к интернету (Wi-Fi/Ethernet)
 * @note Вызывается один раз после инициализации сети
 *
 * @see sntp_time_sync()
 * @see sntp_client_get_time_string()
 * @see sntp_client_is_synced()
 */
void sntp_client_init(void);

/**
 * @brief Синхронизация времени через SNTP
 *
 * Ожидает синхронизации времени с NTP серверами, настроенными в Kconfig:
 * - CONFIG_SNTP_SERVER_1 (основной)
 * - CONFIG_SNTP_SERVER_2
 * - CONFIG_SNTP_SERVER_3
 * - CONFIG_SNTP_SERVER_4
 *
 * Функция блокируется до успешной синхронизации или исчерпания попыток.
 *
 * @return true если синхронизация успешна
 * @return false если не удалось синхронизировать время
 *
 * @note Требуется активное подключение к WiFi перед вызовом
 * @note Использует CONFIG_SNTP_SYNC_RETRY_COUNT для максимального числа попыток
 *
 * Пример:
 * @code
 * if (sntp_time_sync()) {
 *     // Время синхронизировано
 * }
 * @endcode
 *
 * @see sntp_client_init()
 */
bool sntp_time_sync(void);

/**
 * @brief Проверка состояния синхронизации времени
 * 
 * Возвращает текущий статус синхронизации с NTP сервером.
 * 
 * @return true если время успешно синхронизировано
 * @return false если синхронизация ещё не завершена или не удалась
 * 
 * @note После sntp_client_init() требуется время на синхронизацию (обычно 1-5 сек)
 * @note При потере сети может потребоваться повторная инициализация
 * 
 * Пример:
 * @code
 * if (sntp_client_is_synced()) {
 *     // Время синхронизировано, можно использовать
 *     sntp_client_get_time_string(buf, sizeof(buf));
 * }
 * @endcode
 */
bool sntp_client_is_synced(void);

/**
 * @brief Получение строки текущего времени
 * 
 * Форматирует текущее системное время в строку формата:
 * "YYYY-MM-DD HH:MM:SS" (например: "2024-03-13 15:30:45")
 * 
 * @param buffer Буфер для хранения строки времени
 * @param buffer_size Размер буфера (рекомендуется минимум 32 байта)
 * @return true если время получено успешно
 * @return false если время ещё не синхронизировано
 * 
 * @note Требуется предварительная синхронизация через sntp_client_init()
 * @note Используйте sntp_client_is_synced() для проверки перед вызовом
 * 
 * Пример:
 * @code
 * char time_str[32];
 * if (sntp_client_get_time_string(time_str, sizeof(time_str))) {
 *     ESP_LOGI(TAG, "Время: %s", time_str);
 * }
 * @endcode
 * 
 * @see sntp_client_is_synced()
 */
bool sntp_client_get_time_string(char *buffer, size_t buffer_size);

/**
 * @brief Получение текущего времени в секундах (Unix timestamp)
 * 
 * Возвращает текущее системное время как количество секунд,
 * прошедших с 1 января 1970 года (Unix epoch).
 * 
 * @return time_t Текущее время в секундах, или 0 если не синхронизировано
 * 
 * @note Для преобразования в читаемый формат используйте localtime()
 * 
 * Пример:
 * @code
 * time_t now = sntp_client_get_timestamp();
 * if (now > 0) {
 *     struct tm timeinfo;
 *     localtime_r(&now, &timeinfo);
 *     printf("Год: %d\n", timeinfo.tm_year + 1900);
 * }
 * @endcode
 */
time_t sntp_client_get_timestamp(void);

#ifdef __cplusplus
}
#endif

#endif // SNTP_CLIENT_H
