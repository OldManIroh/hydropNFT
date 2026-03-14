#pragma once

#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Инициализация компонента ADS1115
 * @return ESP_OK при успехе, код ошибки в противном случае
 */
esp_err_t ads1115_init(void);

/**
 * @brief Чтение напряжения с определенного канала
 * @param channel Номер канала (0-3)
 * @param voltage Указатель для хранения значения напряжения
 * @return ESP_OK при успехе, код ошибки в противном случае
 */
esp_err_t ads1115_read_voltage(uint8_t channel, float *voltage);

/**
 * @brief Чтение сырых ADC значений с определенного канала
 * @param channel Номер канала (0-3)
 * @param raw_value Указатель для хранения сырых ADC значений
 * @return ESP_OK при успехе, код ошибки в противном случае
 */
esp_err_t ads1115_read_raw(uint8_t channel, int16_t *raw_value);

/**
 * @brief Структура для хранения измерения канала
 */
typedef struct {
    uint8_t channel;    // Номер канала (0-3)
    float voltage;      // Измеренное напряжение
    int16_t raw_value;  // Сырое значение ADC
    esp_err_t error;    // Код ошибки измерения
} ads1115_measurement_t;

/**
 * @brief Измерение напряжения со всех каналов и возврат результатов
 * @param measurements Массив для хранения результатов измерений (должен быть размером не менее 4 элементов)
 * @return ESP_OK при успехе, код ошибки в противном случае
 */
esp_err_t ads1115_measure_all_channels(ads1115_measurement_t *measurements);

/**
 * @brief Непрерывное измерение и вывод напряжения со всех каналов
 * @note Эта функция работает в цикле и должна вызываться из задачи
 * @deprecated Используйте ads1115_measure_all_channels() для получения массива измерений
 */
void ads1115_measure_all_channels_legacy(void);

#ifdef __cplusplus
}
#endif
