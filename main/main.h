#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Установить состояние насоса
 * @param state Состояние: true = включить, false = выключить
 * @param manual Ручное управление: true = ручное (из MQTT, 5 мин), false = авто
 */
void set_pump_state(bool state, bool manual);

/**
 * @brief Установить состояние света
 * @param state Состояние: true = включить, false = выключить
 * @param manual Ручное управление: true = ручное (из MQTT, 2 часа), false = авто (расписание)
 */
void set_light_state(bool state, bool manual);

/**
 * @brief Установить состояние клапана
 * @param state Состояние: true = открыть, false = закрыть
 * @param manual Ручное управление: true = ручное (из MQTT, 5 мин), false = авто
 */
void set_valve_state(bool state, bool manual);

/**
 * @brief Получить состояние насоса
 * @return true если включен, false если выключен
 */
bool get_pump_state(void);

/**
 * @brief Получить состояние света
 * @return true если включен, false если выключен
 */
bool get_light_state(void);

/**
 * @brief Получить состояние клапана
 * @return true если открыт, false если закрыт
 */
bool get_valve_state(void);

/**
 * @brief Получить температуру с DHT датчика
 * @return Температура в °C
 */
float get_dht_temperature(void);

/**
 * @brief Получить влажность с DHT датчика
 * @return Влажность в %
 */
float get_dht_humidity(void);

/**
 * @brief Проверить валидность данных DHT
 * @return true если данные были успешно получены хотя бы раз
 * @return false если данные ещё не получены
 */
bool is_dht_data_valid(void);

/**
 * @brief Остановить задачу ADS1115 для OTA обновления
 * 
 * Устанавливает флаг остановки, после чего задача ADS1115
 * перестанет опрашивать датчики и будет ждать в цикле.
 * 
 * @note Вызывается из ota_client перед началом OTA
 */
void ads1115_stop_for_ota(void);

#ifdef __cplusplus
}
#endif
