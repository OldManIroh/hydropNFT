#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Конфигурация датчика уровня воды
 */
typedef struct {
    uint8_t touch_channel;              ///< Канал касания (например, TOUCH_MIN_CHAN_ID)
    uint16_t baseline_samples;          ///< Количество измерений для усреднения базовой линии
    float threshold_ratio;              ///< Коэффициент для расчёта порога (обычно 0.015f)
    uint32_t measurement_interval_ms;   ///< Интервал измерений в миллисекундах
} water_level_config_t;

/**
 * @brief Структура состояния датчика уровня воды
 */
typedef struct {
    uint32_t baseline;                  ///< Базовая линия (значение без воды)
    uint32_t current_value;             ///< Текущее измеренное значение
    int32_t delta;                      ///< Разница между текущим значением и базовой линией
    bool is_initialized;                ///< Флаг инициализации
} water_level_state_t;

/**
 * @brief Инициализация датчика уровня воды
 * 
 * @param config Конфигурация датчика
 * @param state Указатель на структуру состояния
 * @return true в случае успеха, false в случае ошибки
 */
bool water_level_init(const water_level_config_t *config, water_level_state_t *state);

/**
 * @brief Собрать базовую линию (значение без воды)
 * 
 * @param state Указатель на структуру состояния
 * @return true в случае успеха, false в случае ошибки
 */
bool water_level_calibrate_baseline(water_level_state_t *state);

/**
 * @brief Получить текущее измерение уровня воды
 * 
 * @param state Указатель на структуру состояния
 * @return true в случае успеха, false в случае ошибки
 */
bool water_level_read(water_level_state_t *state);

/**
 * @brief Получить текущее значение дельты (разница от базовой линии)
 * 
 * @param state Указатель на структуру состояния
 * @return int32_t Текущая дельта
 */
int32_t water_level_get_delta(const water_level_state_t *state);

/**
 * @brief Получить текущее измеренное значение
 * 
 * @param state Указатель на структуру состояния
 * @return uint32_t Текущее измеренное значение
 */
uint32_t water_level_get_value(const water_level_state_t *state);

/**
 * @brief Проверить, есть ли вода (дельта превышает порог)
 * 
 * @param state Указатель на структуру состояния
 * @param threshold Порог для определения наличия воды
 * @return true если вода обнаружена, false если нет
 */
bool water_level_is_water_detected(const water_level_state_t *state, int32_t threshold);

/**
 * @brief Деинициализация датчика уровня воды
 * 
 * @param state Указатель на структуру состояния
 * @return true в случае успеха, false в случае ошибки
 */
bool water_level_deinit(water_level_state_t *state);

#ifdef __cplusplus
}
#endif
