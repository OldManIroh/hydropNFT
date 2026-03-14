#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ads1115.h"
#include "esp_log.h"

static const char *TAG = "SINGLE_CHANNEL";

// Функция для опроса одного канала
void read_single_channel(uint8_t channel)
{
    float voltage = 0.0f;
    int16_t raw_value = 0;
    esp_err_t res;
    
    ESP_LOGI(TAG, "=== Опрос канала %u ===", channel);
    
    // Проверка номера канала
    if (channel > 3) {
        ESP_LOGE(TAG, "Неверный номер канала: %u. Допустимые значения: 0-3", channel);
        return;
    }
    
    // Чтение напряжения с канала
    res = ads1115_read_voltage(channel, &voltage);
    if (res == ESP_OK) {
        ESP_LOGI(TAG, "Канал %u: напряжение = %.04f В", channel, voltage);
    } else {
        ESP_LOGE(TAG, "Ошибка чтения напряжения с канала %u: %d", channel, res);
        return;
    }
    
    // Чтение сырого значения с канала
    res = ads1115_read_raw(channel, &raw_value);
    if (res == ESP_OK) {
        ESP_LOGI(TAG, "Канал %u: сырое значение = %d", channel, raw_value);
    } else {
        ESP_LOGE(TAG, "Ошибка чтения сырого значения с канала %u: %d", channel, res);
        return;
    }
    
    // Дополнительная информация
    ESP_LOGI(TAG, "Канал %u: значение в процентах от диапазона = %.02f%%", 
             channel, (voltage / 4.096f) * 100.0f);
}

// Задача для периодического опроса канала
void single_channel_task(void *pvParameters)
{
    uint8_t channel = (uint8_t)(uintptr_t)pvParameters;
    
    // Инициализация ADS1115
    esp_err_t res = ads1115_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Не удалось инициализировать ADS1115: %d", res);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "ADS1115 успешно инициализирован");
    
    while (1) {
        // Опрос выбранного канала
        read_single_channel(channel);
        
        // Пауза между измерениями
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 секунда
    }
}

// Пример использования в app_main
void app_main_example(void)
{
    // Запуск задачи для опроса канала 0
    xTaskCreatePinnedToCore(single_channel_task, "channel_0_task", 
                           configMINIMAL_STACK_SIZE * 4, 
                           (void*)0, 5, NULL, APP_CPU_NUM);
    
    // Можно запустить задачи для других каналов
    // xTaskCreatePinnedToCore(single_channel_task, "channel_1_task", 
    //                        configMINIMAL_STACK_SIZE * 4, 
    //                        (void*)1, 5, NULL, APP_CPU_NUM);
}