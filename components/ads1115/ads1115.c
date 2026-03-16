#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <ads111x.h>
#include <string.h>
#include <i2cdev.h>
#include "ads1115.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_PORT 0
#define GAIN ADS111X_GAIN_4V096 // +-4.096V

static const char *TAG = "ADS1115";

// Адрес I2C
static const uint8_t addr = ADS111X_ADDR_GND;

// Дескрипторы
static i2c_dev_t device;

// Значение усиления
static float gain_val;

// Флаг инициализации
static bool ads1115_initialized = false;

// Мьютекс для защиты доступа к ADS1115 (пункт 1.2)
static SemaphoreHandle_t ads1115_mutex = NULL;

// Конфигурация фильтра (пункт 1.3)
#define ADS1115_FILTER_WINDOW_SIZE 5

// Состояние медианного фильтра
typedef struct {
    float voltage_buffer[4][ADS1115_FILTER_WINDOW_SIZE];
    int16_t raw_buffer[4][ADS1115_FILTER_WINDOW_SIZE];
    uint8_t buffer_index;
    bool buffer_filled;
} ads1115_filter_state_t;

static ads1115_filter_state_t filter_state = {0};

// Последние данные сенсоров для доступа из других компонентов (ЛОГ-1)
static ads1115_sensor_data_t latest_sensor_data = {0};

// Таймауты для операций I2C (пункт 1.1)
#define ADS1115_I2C_TIMEOUT_MS 100
#define ADS1115_CONVERSION_TIMEOUT_MS 150

// Массив настроек мультиплексора для всех 4 каналов
static const ads111x_mux_t mux_settings[4] = {
    ADS111X_MUX_0_GND,
    ADS111X_MUX_1_GND,
    ADS111X_MUX_2_GND,
    ADS111X_MUX_3_GND};

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (медианный фильтр - пункт 1.3)
// ============================================================================

/**
 * @brief Вычисление медианы массива float
 * @param buffer Буфер с данными
 * @param size Размер буфера
 * @return Медианное значение
 */
static float median_filter_float(const float *buffer, uint8_t size)
{
    float sorted[ADS1115_FILTER_WINDOW_SIZE];
    memcpy(sorted, buffer, size * sizeof(float));
    
    // Сортировка пузырьком для малого размера
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    return sorted[size / 2];  // Возвращаем медиану
}

/**
 * @brief Вычисление медианы массива int16_t
 * @param buffer Буфер с данными
 * @param size Размер буфера
 * @return Медианное значение
 */
static int16_t median_filter_int16(const int16_t *buffer, uint8_t size)
{
    int16_t sorted[ADS1115_FILTER_WINDOW_SIZE];
    memcpy(sorted, buffer, size * sizeof(int16_t));
    
    // Сортировка пузырьком для малого размера
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                int16_t temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    return sorted[size / 2];  // Возвращаем медиану
}

esp_err_t ads1115_init(void)
{
    // Проверяем, не была ли уже инициализирована
    if (ads1115_initialized) {
        ESP_LOGW(TAG, "ADS1115 уже инициализирован");
        return ESP_OK;
    }

    // Создаём мьютекс для защиты доступа (пункт 1.2)
    if (ads1115_mutex == NULL) {
        ads1115_mutex = xSemaphoreCreateMutex();
        if (ads1115_mutex == NULL) {
            ESP_LOGE(TAG, "Не удалось создать мьютекс");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "Мьютекс ADS1115 создан");
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

/**
 * @brief Чтение напряжения с определенного канала
 * @param channel Номер канала (0-3)
 * @param voltage Указатель для хранения значения напряжения
 * @return ESP_OK при успехе, код ошибки в противном случае
 * 
 * @note Пункт 1.1: Добавлена обработка ошибок I2C и таймауты
 * @note Пункт 1.2: Добавлена защита мьютексом
 * @note Пункт 2.2: Исправлен расчёт напряжения (переполнение)
 */
esp_err_t ads1115_read_voltage(uint8_t channel, float *voltage)
{
    if (channel > 3 || voltage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Захватываем мьютекс (пункт 1.2)
    if (xSemaphoreTake(ads1115_mutex, pdMS_TO_TICKS(ADS1115_I2C_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Не удалось захватить мьютекс ADS1115");
        return ESP_ERR_TIMEOUT;
    }

    int16_t raw = 0;
    esp_err_t res;

    // Установка входного мультиплексора для канала
    res = ads111x_set_input_mux(&device, mux_settings[channel]);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки мультиплексора канала %u: %s", 
                 channel, esp_err_to_name(res));
        xSemaphoreGive(ads1115_mutex);
        return res;
    }

    // Запуск преобразования
    res = ads111x_start_conversion(&device);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка запуска конвертации канала %u: %s", 
                 channel, esp_err_to_name(res));
        xSemaphoreGive(ads1115_mutex);
        return res;
    }

    // Ожидание завершения преобразования с таймаутом (пункт 1.1)
    bool busy;
    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(ADS1115_CONVERSION_TIMEOUT_MS);
    
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        res = ads111x_is_busy(&device, &busy);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка проверки статуса канала %u: %s", 
                     channel, esp_err_to_name(res));
            xSemaphoreGive(ads1115_mutex);
            return res;
        }
        
        // Проверка таймаута
        if (xTaskGetTickCount() - start_tick > timeout_ticks) {
            ESP_LOGE(TAG, "Таймаут ожидания конвертации канала %u", channel);
            xSemaphoreGive(ads1115_mutex);
            return ESP_ERR_TIMEOUT;
        }
    } while (busy);

    // Чтение результата
    res = ads111x_get_value(&device, &raw);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка чтения значения канала %u: %s", 
                 channel, esp_err_to_name(res));
        xSemaphoreGive(ads1115_mutex);
        return res;
    }

    // Освобождаем мьютекс
    xSemaphoreGive(ads1115_mutex);

    // Расчёт напряжения (пункт 2.2 - исправлено переполнение)
    *voltage = (gain_val * (float)raw) / (float)ADS111X_MAX_VALUE;
    
    return ESP_OK;
}

/**
 * @brief Чтение сырых ADC значений с определенного канала
 * @param channel Номер канала (0-3)
 * @param raw_value Указатель для хранения сырых ADC значений
 * @return ESP_OK при успехе, код ошибки в противном случае
 * 
 * @note Пункт 1.1: Добавлена обработка ошибок I2C и таймауты
 * @note Пункт 1.2: Добавлена защита мьютексом
 */
esp_err_t ads1115_read_raw(uint8_t channel, int16_t *raw_value)
{
    if (channel > 3 || raw_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Захватываем мьютекс (пункт 1.2)
    if (xSemaphoreTake(ads1115_mutex, pdMS_TO_TICKS(ADS1115_I2C_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Не удалось захватить мьютекс ADS1115");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t res;

    // Установка входного мультиплексора для канала
    res = ads111x_set_input_mux(&device, mux_settings[channel]);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки мультиплексора канала %u: %s", 
                 channel, esp_err_to_name(res));
        xSemaphoreGive(ads1115_mutex);
        return res;
    }

    // Запуск преобразования
    res = ads111x_start_conversion(&device);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка запуска конвертации канала %u: %s", 
                 channel, esp_err_to_name(res));
        xSemaphoreGive(ads1115_mutex);
        return res;
    }

    // Ожидание завершения преобразования с таймаутом (пункт 1.1)
    bool busy;
    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(ADS1115_CONVERSION_TIMEOUT_MS);
    
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        res = ads111x_is_busy(&device, &busy);
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Ошибка проверки статуса канала %u: %s", 
                     channel, esp_err_to_name(res));
            xSemaphoreGive(ads1115_mutex);
            return res;
        }
        
        // Проверка таймаута
        if (xTaskGetTickCount() - start_tick > timeout_ticks) {
            ESP_LOGE(TAG, "Таймаут ожидания конвертации канала %u", channel);
            xSemaphoreGive(ads1115_mutex);
            return ESP_ERR_TIMEOUT;
        }
    } while (busy);

    // Чтение результата
    res = ads111x_get_value(&device, raw_value);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка чтения значения канала %u: %s", 
                 channel, esp_err_to_name(res));
    }

    // Освобождаем мьютекс
    xSemaphoreGive(ads1115_mutex);
    
    return res;
}

/**
 * @brief Измерение напряжения со всех каналов и возврат результатов
 * @param measurements Массив для хранения результатов измерений (должен быть размером не менее 4 элементов)
 * @return ESP_OK при успехе, код ошибки в противном случае
 * 
 * @note Пункт 1.3: Добавлен медианный фильтр для подавления шумов
 */
esp_err_t ads1115_measure_all_channels(ads1115_measurement_t *measurements)
{
    if (measurements == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t overall_result = ESP_OK;

    for (size_t ch = 0; ch < 4; ch++) {
        // Инициализация структуры измерения
        measurements[ch].channel = ch;
        measurements[ch].voltage = 0.0f;
        measurements[ch].raw_value = 0;
        measurements[ch].error = ESP_OK;

        // КРИТ-1: ОДНО чтение напряжения на канал (вместо двух: voltage + raw)
        float voltage = 0.0f;
        esp_err_t res = ads1115_read_voltage(ch, &voltage);

        if (res != ESP_OK) {
            measurements[ch].error = res;
            overall_result = res;
            continue;
        }

        // Добавляем в буфер фильтра (пункт 1.3)
        filter_state.voltage_buffer[ch][filter_state.buffer_index] = voltage;

        // Применяем медианный фильтр только если буфер заполнен хотя бы один раз
        if (filter_state.buffer_filled) {
            measurements[ch].voltage = median_filter_float(
                filter_state.voltage_buffer[ch], ADS1115_FILTER_WINDOW_SIZE);
        } else {
            measurements[ch].voltage = voltage;
        }
    }

    // Циклический буфер
    filter_state.buffer_index = (filter_state.buffer_index + 1) % ADS1115_FILTER_WINDOW_SIZE;
    if (filter_state.buffer_index == 0) {
        filter_state.buffer_filled = true;
    }

    // ЛОГ-1: Обновляем последние данные для доступа из MQTT
    for (size_t ch = 0; ch < 4; ch++) {
        if (measurements[ch].error == ESP_OK) {
            latest_sensor_data.voltage[ch] = measurements[ch].voltage;
        }
    }
    if (overall_result == ESP_OK) {
        latest_sensor_data.valid = true;
    }

    return overall_result;
}

/**
 * @brief Получить последние измеренные данные с ADS1115
 * @return Указатель на структуру с данными (никогда не NULL)
 */
const ads1115_sensor_data_t* ads1115_get_latest_data(void)
{
    return &latest_sensor_data;
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

    // Освобождаем мьютекс
    if (ads1115_mutex != NULL) {
        vSemaphoreDelete(ads1115_mutex);
        ads1115_mutex = NULL;
        ESP_LOGI(TAG, "Мьютекс ADS1115 удалён");
    }

    // Освобождаем ресурсы I2C устройства (ПОТ-8)
    ads111x_free_desc(&device);

    // Сбрасываем флаг инициализации
    ads1115_initialized = false;

    // Очищаем дескриптор устройства
    memset(&device, 0, sizeof(device));

    // Сбрасываем состояние фильтра
    memset(&filter_state, 0, sizeof(filter_state));

    // Освобождаем ресурсы I2C библиотеки (если другие компоненты не используют)
    // i2cdev_done();  // Закомментировано - может использоваться другими компонентами

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

    // Освобождаем мьютекс если существует
    if (ads1115_mutex != NULL) {
        vSemaphoreDelete(ads1115_mutex);
        ads1115_mutex = NULL;
    }

    ads1115_initialized = false;
    memset(&device, 0, sizeof(device));
    gain_val = 0.0f;
    
    // Сбрасываем состояние фильтра
    memset(&filter_state, 0, sizeof(filter_state));

    ESP_LOGI(TAG, "Состояние ADS1115 сброшено");
}
