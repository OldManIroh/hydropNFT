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
 * @brief Деинициализация I2C драйвера
 * 
 * Вызывается перед OTA обновлением для освобождения I2C шины.
 * 
 * @note Вызывать только перед OTA обновлением (когда версии НЕ совпали)
 * @note НЕ вызывать если OTA не требуется (версии совпали)
 */
void ads1115_deinit(void);

/**
 * @brief Сброс состояния ADS1115
 * 
 * Полностью сбрасывает состояние компонента.
 * Используется при ошибках инициализации.
 */
void ads1115_reset(void);

/**
 * @brief Структура для хранения последних данных сенсоров ADS1115
 *
 * Каналы:
 * - 0: pH датчик (напряжение)
 * - 1: Температура воды (напряжение)
 * - 2: TDS датчик (напряжение)
 * - 3: Уровень воды (напряжение)
 */
typedef struct {
    float voltage[4];   ///< Напряжение каждого канала в вольтах
    bool valid;         ///< true если хотя бы один канал успешно прочитан
    uint8_t valid_channels;  ///< Битовая маска валидных каналов (бит 0 = канал 0)
} ads1115_sensor_data_t;

/**
 * @brief Получить последние измеренные данные с ADS1115
 * 
 * Возвращает указатель на структуру с последними данными.
 * Данные обновляются в ads1115_task() каждые 500 мс.
 * 
 * @return Указатель на структуру с данными (никогда не NULL)
 */
const ads1115_sensor_data_t* ads1115_get_latest_data(void);

#ifdef __cplusplus
}
#endif
