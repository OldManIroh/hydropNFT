#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ads111x.h>
#include <string.h>
#include <i2cdev.h>
#include "ads1115.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_PORT 0
#define GAIN ADS111X_GAIN_2V048 // +-4.096V

static const char *TAG = "ADS1115";

// Адрес I2C
static const uint8_t addr = ADS111X_ADDR_GND;

// Дескрипторы
static i2c_dev_t device;

// Значение усиления
static float gain_val;

// Флаг инициализации
static bool ads1115_initialized = false;

// Массив настроек мультиплексора для всех 4 каналов
static const ads111x_mux_t mux_settings[4] = {
    ADS111X_MUX_0_GND,
    ADS111X_MUX_1_GND,
    ADS111X_MUX_2_GND,
    ADS111X_MUX_3_GND};

esp_err_t ads1115_init(void)
{
    // Проверяем, не была ли уже инициализирована
    if (ads1115_initialized) {
        ESP_LOGW(TAG, "ADS1115 уже инициализирован");
        return ESP_OK;
    }

    // Инициализация библиотеки
    esp_err_t res = i2cdev_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка инициализации I2C библиотеки: %d", res);
        return res;
    }

    // Очистка дескрипторов устройства
    memset(&device, 0, sizeof(device));

    // Инициализация ADS1115
    res = ads111x_init_desc(&device, addr, I2C_PORT, CONFIG_I2CDEV_DEFAULT_SDA_PIN, CONFIG_I2CDEV_DEFAULT_SCL_PIN);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка инициализации ADS1115: %d", res);
        return res;
    }

    gain_val = ads111x_gain_values[GAIN];

    // Настройка ADS1115
    res = ads111x_set_mode(&device, ADS111X_MODE_SINGLE_SHOT);   // Режим одиночного измерения
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки режима ADS1115: %d", res);
        return res;
    }

    res = ads111x_set_data_rate(&device, ADS111X_DATA_RATE_8); // 32 выборки в секунду
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки частоты дискретизации ADS1115: %d", res);
        return res;
    }

    res = ads111x_set_gain(&device, GAIN);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки усиления ADS1115: %d", res);
        return res;
    }

    ads1115_initialized = true;
    ESP_LOGI(TAG, "ADS1115 успешно инициализирован");
    return ESP_OK;
}

esp_err_t ads1115_read_voltage(uint8_t channel, float *voltage)
{
    if (channel > 3 || voltage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t raw = 0;
    esp_err_t res;

    // Установка входного мультиплексора для канала
    res = ads111x_set_input_mux(&device, mux_settings[channel]);
    if (res != ESP_OK) {
        return res;
    }

    // Запуск преобразования
    res = ads111x_start_conversion(&device);
    if (res != ESP_OK) {
        return res;
    }

    // Ожидание завершения преобразования
    bool busy;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));   // освобождаем процессор
        res = ads111x_is_busy(&device, &busy);
        if (res != ESP_OK) {
            return res;
        }
    } while (busy);                     // busy == true, пока идёт преобразование

    // Чтение результата
    res = ads111x_get_value(&device, &raw);
    if (res == ESP_OK) {
        *voltage = gain_val / ADS111X_MAX_VALUE * raw;
    }

    return res;
}

esp_err_t ads1115_read_raw(uint8_t channel, int16_t *raw_value)
{
    if (channel > 3 || raw_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t res;

    // Установка входного мультиплексора для канала
    res = ads111x_set_input_mux(&device, mux_settings[channel]);
    if (res != ESP_OK) {
        return res;
    }

    // Запуск преобразования
    res = ads111x_start_conversion(&device);
    if (res != ESP_OK) {
        return res;
    }

    // Ожидание завершения преобразования
    bool busy;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));   // освобождаем процессор
        res = ads111x_is_busy(&device, &busy);
        if (res != ESP_OK) {
            return res;
        }
    } while (busy);                     // busy == true, пока идёт преобразование

    // Чтение результата
    return ads111x_get_value(&device, raw_value);
}

esp_err_t ads1115_measure_all_channels(ads1115_measurement_t *measurements)
{
    if (measurements == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t overall_result = ESP_OK;

    for (size_t ch = 0; ch < 4; ch++)
    {
        // Инициализация структуры измерения
        measurements[ch].channel = ch;
        measurements[ch].voltage = 0.0f;
        measurements[ch].raw_value = 0;
        measurements[ch].error = ESP_OK;

        // Чтение напряжения
        esp_err_t voltage_res = ads1115_read_voltage(ch, &measurements[ch].voltage);
        
        // Чтение сырого значения
        esp_err_t raw_res = ads1115_read_raw(ch, &measurements[ch].raw_value);

        // Определяем общий результат для канала
        if (voltage_res != ESP_OK) {
            measurements[ch].error = voltage_res;
            overall_result = voltage_res;
        } else if (raw_res != ESP_OK) {
            measurements[ch].error = raw_res;
            overall_result = raw_res;
        }
    }

    return overall_result;
}

void ads1115_measure_all_channels_legacy(void)
{
    ads1115_measurement_t measurements[4];
    esp_err_t res = ads1115_measure_all_channels(measurements);

    if (res == ESP_OK) {
        for (size_t ch = 0; ch < 4; ch++) {
            ESP_LOGI(TAG, "Канал %u: напряжение = %.04f В", measurements[ch].channel, measurements[ch].voltage);
        }
    } else {
        ESP_LOGE(TAG, "Ошибка при измерениях: %d", res);
    }
}

/**
 * @brief Деинициализация I2C драйвера для освобождения шины
 *
 * Вызывается перед OTA обновлением для предотвращения конфликтов I2C.
 * После перезагрузки при успешном OTA инициализация произойдёт заново.
 *
 * @note Вызывать только перед OTA обновлением (когда версии НЕ совпали)
 * @note НЕ вызывать если OTA не требуется (версии совпали)
 */
void ads1115_deinit(void)
{
    ESP_LOGI(TAG, "Деинициализация ADS1115 для OTA...");
    
    // Сбрасываем флаг инициализации
    ads1115_initialized = false;
    
    // Очищаем дескриптор устройства
    memset(&device, 0, sizeof(device));
    
    ESP_LOGI(TAG, "ADS1115 деинициализирован");
}

/**
 * @brief Сброс состояния ADS1115
 *
 * Полностью сбрасывает состояние компонента.
 * Используется при ошибках инициализации.
 */
void ads1115_reset(void)
{
    ESP_LOGI(TAG, "Сброс состояния ADS1115...");
    
    ads1115_initialized = false;
    memset(&device, 0, sizeof(device));
    gain_val = 0.0f;
    
    ESP_LOGI(TAG, "Состояние ADS1115 сброшено");
}
