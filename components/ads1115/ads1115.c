/**
 * @file ads1115.c
 * @brief Драйвер ADS1115 для HydroNFT
 *
 * Компонент для работы с 4-канальным 16-битным АЦП ADS1115 через I2C.
 * Используется для чтения аналоговых датчиков:
 * - Канал 0: pH метр
 * - Канал 1: Температура воды
 * - Канал 2: TDS метр
 * - Канал 3: Уровень воды
 *
 * Особенности реализации:
 * - Мьютекс для защиты от одновременного доступа
 * - Медианный фильтр (окно 5 отсчётов) для подавления шумов
 * - Обработка таймаутов I2C
 *
 * @author HydroNFT Team
 * @version 2.0
 * @date 2024
 */

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
#define GAIN ADS111X_GAIN_4V096  ///< Коэффициент усиления: ±4.096В

static const char *TAG = "ADS1115";

/// Адрес I2C (зависит от подключения ADDR: GND=0x48, VDD=0x49, SDA=0x4A, SCL=0x4B)
static const uint8_t addr = ADS111X_ADDR_GND;

/// Дескриптор устройства I2C
static i2c_dev_t device;

/// Значение усиления в вольтах (из таблицы ads111x_gain_values)
static float gain_val;

/// Флаг инициализации (защита от повторной инициализации)
static bool ads1115_initialized = false;

/// Мьютекс для защиты доступа к I2C шине
static SemaphoreHandle_t ads1115_mutex = NULL;

/// Размер окна медианного фильтра
#define ADS1115_FILTER_WINDOW_SIZE 5

/// Состояние медианного фильтра для всех 4 каналов
typedef struct {
    float voltage_buffer[4][ADS1115_FILTER_WINDOW_SIZE];
    uint8_t buffer_index;
    bool buffer_filled;
} ads1115_filter_state_t;

static ads1115_filter_state_t filter_state = {0};

/// Последние данные сенсоров для доступа из других компонентов
static ads1115_sensor_data_t latest_sensor_data = {0};

/// Таймаут ожидания конвертации (250 мс для 8 SPS)
#define ADS1115_CONVERSION_TIMEOUT_MS 250

/// Таймаут захвата мьютекса
#define ADS1115_I2C_TIMEOUT_MS 200

/// Настройки мультиплексора для каналов 0-3 (каждый канал к GND)
static const ads111x_mux_t mux_settings[4] = {
    ADS111X_MUX_0_GND,  ///< Канал 0: AIN0 к GND
    ADS111X_MUX_1_GND,  ///< Канал 1: AIN1 к GND
    ADS111X_MUX_2_GND,  ///< Канал 2: AIN2 к GND
    ADS111X_MUX_3_GND   ///< Канал 3: AIN3 к GND
};

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ (медианный фильтр)
// ============================================================================

/**
 * @brief Вычисление медианы массива float
 * @param buffer Буфер с данными
 * @param size Размер буфера
 * @return Медианное значение
 *
 * @note Использует сортировку пузырьком — эффективно для малых размеров (≤5)
 * @note Возвращает средний элемент отсортированного массива
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
 * @brief Инициализация ADS1115
 * @return ESP_OK при успехе, код ошибки в противном случае
 *
 * Последовательность инициализации:
 * 1. Создание мьютекса для защиты I2C шины
 * 2. Инициализация I2C библиотеки (i2cdev)
 * 3. Настройка дескриптора устройства
 * 4. Конфигурация ADS1115: режим, скорость, усиление
 *
 * @note Вызывается один раз в app_main() перед запуском задач
 * @note Повторный вызов возвращает ESP_OK (защита от дублирования)
 * @note При ошибке инициализации используйте ads1115_reset() для сброса
 */
esp_err_t ads1115_init(void)
{
    // Проверяем, не была ли уже инициализирована
    if (ads1115_initialized) {
        ESP_LOGW(TAG, "ADS1115 уже инициализирован");
        return ESP_OK;
    }

    // Создаём мьютекс для защиты доступа
    if (ads1115_mutex == NULL) {
        ads1115_mutex = xSemaphoreCreateMutex();
        if (ads1115_mutex == NULL) {
            ESP_LOGE(TAG, "Не удалось создать мьютекс");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "Мьютекс ADS1115 создан");
    }

    // Инициализация библиотеки I2C
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
    res = ads111x_set_mode(&device, ADS111X_MODE_SINGLE_SHOT);   ///< Режим одиночного измерения
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка установки режима ADS1115: %d", res);
        return res;
    }

    res = ads111x_set_data_rate(&device, ADS111X_DATA_RATE_8); ///< 8 выборок в секунду (период 125 мс)
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
 * Выполняет полный цикл измерения:
 * 1. Установка мультиплексора на нужный канал
 * 2. Запуск конвертации
 * 3. Ожидание завершения (с таймаутом)
 * 4. Чтение результата
 * 5. Преобразование в напряжение
 *
 * @note Функция захватывает мьютекс на время всего измерения (до 250 мс)
 */
esp_err_t ads1115_read_voltage(uint8_t channel, float *voltage)
{
    if (channel > 3 || voltage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Захватываем мьютекс
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

    // Ожидание завершения преобразования с таймаутом
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

    // Расчёт напряжения: V = (gain × raw) / MAX_VALUE
    // gain_val = 4.096В, ADS111X_MAX_VALUE = 32767 (2^15-1)
    *voltage = (gain_val * (float)raw) / (float)ADS111X_MAX_VALUE;

    return ESP_OK;
}

/**
 * @brief Измерение напряжения со всех каналов и возврат результатов
 * @param measurements Массив для хранения результатов измерений (должен быть размером не менее 4 элементов)
 * @return ESP_OK при успехе, код ошибки в противном случае
 *
 * Измеряет все 4 канала ADS1115 последовательно:
 * 1. Чтение напряжения (ads1115_read_voltage)
 * 2. Применение медианного фильтра
 * 3. Сохранение в структуру measurements
 * 4. Обновление latest_sensor_data с маской валидных каналов
 *
 * @note При ошибке одного канала остальные продолжают работать
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
        measurements[ch].error = ESP_OK;

        // Читаем напряжение с канала
        float voltage = 0.0f;
        esp_err_t res = ads1115_read_voltage(ch, &voltage);

        if (res != ESP_OK) {
            measurements[ch].error = res;
            overall_result = res;
            continue;
        }

        // Инициализация буфера при первом измерении
        if (!filter_state.buffer_filled) {
            for (int i = 0; i < ADS1115_FILTER_WINDOW_SIZE; i++) {
                filter_state.voltage_buffer[ch][i] = voltage;
            }
        }

        // Добавляем в буфер фильтра
        filter_state.voltage_buffer[ch][filter_state.buffer_index] = voltage;

        // Применяем медианный фильтр (всегда, т.к. буфер инициализирован)
        measurements[ch].voltage = median_filter_float(
            filter_state.voltage_buffer[ch], ADS1115_FILTER_WINDOW_SIZE);
    }

    // Циклический буфер: индекс 0→1→2→3→4→0...
    filter_state.buffer_index = (filter_state.buffer_index + 1) % ADS1115_FILTER_WINDOW_SIZE;
    if (filter_state.buffer_index == 0) {
        filter_state.buffer_filled = true;  // Буфер заполнен хотя бы один раз
    }

    // Обновляем последние данные для доступа из MQTT
    latest_sensor_data.valid = false;
    latest_sensor_data.valid_channels = 0;

    for (size_t ch = 0; ch < 4; ch++) {
        if (measurements[ch].error == ESP_OK) {
            latest_sensor_data.voltage[ch] = measurements[ch].voltage;
            latest_sensor_data.valid = true;
            latest_sensor_data.valid_channels |= (1 << ch);
        }
    }

    return overall_result;
}

/**
 * @brief Получить последние измеренные данные с ADS1115
 * @return Указатель на структуру с данными (никогда не NULL)
 *
 * Возвращает указатель на статическую структуру latest_sensor_data,
 * которая обновляется в функции ads1115_measure_all_channels().
 *
 * @note Данные обновляются в ads1115_task() каждые 1 секунду
 * @note Поле valid_channels содержит битовую маску валидных каналов:
 *       - бит 0 установлен = канал 0 валиден
 *       - бит 1 установлен = канал 1 валиден, и т.д.
 * @note Функция не требует мьютекса — чтение bool и float атомарно на ESP32
 */
const ads1115_sensor_data_t* ads1115_get_latest_data(void)
{
    return &latest_sensor_data;
}

/**
 * @brief Деинициализация ADS1115 для OTA обновления
 *
 * Вызывается перед OTA обновлением для предотвращения конфликтов I2C.
 * Освобождает все ресурсы:
 * 1. Удаляет мьютекс
 * 2. Освобождает дескриптор устройства
 * 3. Сбрасывает флаг инициализации
 * 4. Очищает буферы фильтра
 * 5. Освобождает ресурсы I2C библиотеки (i2cdev_done)
 *
 * @note Вызывать ТОЛЬКО перед OTA обновлением (когда версии НЕ совпали)
 * @note НЕ вызывать если OTA не требуется (версии совпали) — см. ota_exit_no_update()
 * @note После перезагрузки при успешном OTA инициализация произойдёт заново
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

    // Освобождаем ресурсы I2C устройства
    ads111x_free_desc(&device);

    // Сбрасываем флаг инициализации
    ads1115_initialized = false;

    // Очищаем дескриптор устройства
    memset(&device, 0, sizeof(device));

    // Сбрасываем состояние фильтра
    memset(&filter_state, 0, sizeof(filter_state));

    // Освобождаем ресурсы I2C библиотеки
    i2cdev_done();

    ESP_LOGI(TAG, "ADS1115 деинициализирован");
}

/**
 * @brief Сброс состояния ADS1115
 *
 * Полностью сбрасывает состояние компонента:
 * 1. Удаляет мьютекс
 * 2. Сбрасывает флаг инициализации
 * 3. Очищает дескриптор устройства
 * 4. Сбрасывает буферы фильтра
 *
 * Используется при ошибках инициализации для возможности повторной инициализации.
 *
 * @note В отличие от ads1115_deinit(), НЕ вызывает i2cdev_done()
 * @note После сброса необходимо вызвать ads1115_init() для повторной инициализации
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
