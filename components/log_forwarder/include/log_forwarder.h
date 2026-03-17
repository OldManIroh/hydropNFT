/**
 * @file log_forwarder.h
 * @brief Заголовочный файл компонента пересылки логов в MQTT
 *
 * Компонент перехватывает логи ESP-IDF уровней WARNING и ERROR
 * и отправляет их в MQTT топик для отображения в Home Assistant.
 *
 * @author HydroNFT Team
 * @version 1.0
 * @date 2024
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация пересылки логов в MQTT
 *
 * Устанавливает кастомную функцию вывода логов, которая:
 * 1. Перехватывает все логи ESP-IDF
 * 2. Фильтрует только WARNING и ERROR уровни
 * 3. Отправляет их в MQTT топик hydro/log
 * 4. Дублирует логи в UART (консоль)
 *
 * @note Вызывать после инициализации MQTT клиента
 * @note Требуется активное WiFi подключение
 */
void log_forwarder_init(void);

/**
 * @brief Включить/выключить пересылку логов
 *
 * @param enable true = включить пересылку, false = выключить
 *
 * @note По умолчанию пересылка включена
 */
void log_forwarder_enable(bool enable);

/**
 * @brief Проверка состояния пересылки логов
 *
 * @return true если пересылка включена
 * @return false если пересылка выключена
 */
bool log_forwarder_is_enabled(void);

#ifdef __cplusplus
}
#endif
