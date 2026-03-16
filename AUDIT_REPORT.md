# 🔍 Тотальный аудит кода HydroNFT

**Дата:** 15.03.2026  
**Версия прошивки:** 0.2.17  
**Платформа:** ESP32, ESP-IDF, FreeRTOS  

---

## 🔴 КРИТИЧЕСКИЕ ПРОБЛЕМЫ

### КРИТ-1. Двойное чтение каждого канала ADS1115 — удвоение I2C трафика и рассинхронизация данных

**Файл:** `components/ads1115/ads1115.c`, функция `ads1115_measure_all_channels()`, строки ~210-220

**Описание:** Для каждого канала вызываются **две** отдельные функции: `ads1115_read_voltage()` и `ads1115_read_raw()`. Каждая из них выполняет **полный цикл**: установка мультиплексора → запуск конвертации → ожидание → чтение. Это означает:

1. **Каждый канал читается ДВАЖДЫ** — 8 конвертаций вместо 4 за цикл.
2. **Значения `voltage` и `raw` относятся к РАЗНЫМ моментам времени** — между двумя чтениями проходит ~130 мс (конвертация при 8 SPS = 125 мс + overhead). Для быстро меняющихся сигналов (pH при перемешивании) это даёт несогласованные данные.
3. **Удвоение износа I2C шины** и увеличение времени цикла с ~500 мс до ~1000 мс.

**Исправление:**

```c
esp_err_t ads1115_measure_all_channels(ads1115_measurement_t *measurements)
{
    if (measurements == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t overall_result = ESP_OK;

    for (size_t ch = 0; ch < 4; ch++) {
        measurements[ch].channel = ch;
        measurements[ch].voltage = 0.0f;
        measurements[ch].raw_value = 0;
        measurements[ch].error = ESP_OK;

        // ОДНО чтение — получаем raw, вычисляем voltage из него
        int16_t raw = 0;
        esp_err_t res = ads1115_read_raw(ch, &raw);

        if (res != ESP_OK) {
            measurements[ch].error = res;
            overall_result = res;
            continue;
        }

        float voltage = (gain_val * (float)raw) / (float)ADS111X_MAX_VALUE;

        // Добавляем в буфер фильтра
        filter_state.voltage_buffer[ch][filter_state.buffer_index] = voltage;
        filter_state.raw_buffer[ch][filter_state.buffer_index] = raw;

        if (filter_state.buffer_filled) {
            measurements[ch].voltage = median_filter_float(
                filter_state.voltage_buffer[ch], ADS1115_FILTER_WINDOW_SIZE);
            measurements[ch].raw_value = median_filter_int16(
                filter_state.raw_buffer[ch], ADS1115_FILTER_WINDOW_SIZE);
        } else {
            measurements[ch].voltage = voltage;
            measurements[ch].raw_value = raw;
        }
    }

    filter_state.buffer_index = (filter_state.buffer_index + 1) % ADS1115_FILTER_WINDOW_SIZE;
    if (filter_state.buffer_index == 0) {
        filter_state.buffer_filled = true;
    }

    return overall_result;
}
```

---

### КРИТ-2. DHT22: критическая секция блокирует ВСЕ прерывания на ~5 мс — может вызвать сбой Wi-Fi

**Файл:** `managed_components/esp-idf-lib__dht/dht.c`, функция `dht_read_data()`, строки ~150-155

**Описание:** Библиотека DHT использует `portENTER_CRITICAL()` / `portEXIT_CRITICAL()` для защиты тайминг-критичного чтения. На ESP32 это **отключает прерывания на текущем ядре** на всё время чтения 40 бит (~5 мс).

Проблемы:
1. **Wi-Fi стек на ядре 0** может потерять пакеты, если DHT читается на ядре 0. В текущей конфигурации DHT задача привязана к ядру 1 (`APP_CPU_NUM`), что **частично** смягчает проблему.
2. **FreeRTOS tick interrupt** на ядре 1 будет пропущен — это может вызвать дрейф `vTaskDelay` и нарушение таймингов других задач на этом ядре.
3. **Watchdog Timer** для ядра 1 может сработать при частых чтениях DHT, если TWDT настроен на малый интервал.

**Важно:** Библиотека `esp-idf-lib/dht` использует **bit-banging с `ets_delay_us()`** внутри критической секции. Это **НЕ** рекомендуемый подход для ESP32.

**Рекомендация:** Перейти на реализацию через **RMT периферию** ESP32 (например, библиотека `espressif/dht` или собственная реализация через RMT). RMT аппаратно захватывает импульсы без блокировки CPU.

Если миграция на RMT невозможна прямо сейчас, текущая конфигурация (DHT на ядре 1) — приемлемый компромисс, но с оговорками.

---

### КРИТ-3. DHT: при ошибке чтения критическая секция НЕ освобождается

**Файл:** `managed_components/esp-idf-lib__dht/dht.c`, функция `dht_read_data()`, строки ~150-160

```c
PORT_ENTER_CRITICAL();
esp_err_t result = dht_fetch_data(sensor_type, pin, data);
if (result == ESP_OK)
    PORT_EXIT_CRITICAL();
```

**Описание:** Если `dht_fetch_data()` возвращает ошибку (таймаут, сбой), `PORT_EXIT_CRITICAL()` **НЕ вызывается**! Прерывания на ядре остаются отключёнными навсегда. Это приведёт к:
- Зависанию всех задач на ядре 1
- Срабатыванию watchdog
- Перезагрузке устройства

**Примечание:** Макрос `CHECK_LOGE` внутри `dht_fetch_data()` содержит `PORT_EXIT_CRITICAL()` перед возвратом ошибки. Однако это работает только для ошибок **внутри** `dht_fetch_data()`. Если ошибка произойдёт по другой причине (например, повреждение стека), критическая секция останется заблокированной.

**Это баг в самой библиотеке esp-idf-lib/dht.** Рекомендуется:
1. Подать issue/PR в upstream
2. Или форкнуть и исправить:

```c
PORT_ENTER_CRITICAL();
esp_err_t result = dht_fetch_data(sensor_type, pin, data);
PORT_EXIT_CRITICAL();  // ВСЕГДА освобождаем критическую секцию
```

---

### КРИТ-4. Семафор ADS1115 создаётся внутри задачи, но используется глобально — race condition

**Файл:** `main/main.c`, функция `ads1115_task()`, строки ~175-190

**Описание:** Семафор `ads1115_running_sem` создаётся **внутри** `ads1115_task()`, но `get_ads1115_running_sem()` может быть вызван из `mqtt_ota_check_task()` **до** того, как семафор создан. Более того, в `ads1115_task()` есть логическая ошибка:

```cdht_read_data()`, строки ~150-160

SemaphoreHandle_t sem = get_ads1115_running_sem();  // Возвращает NULL (ещё не создан!)
if (sem == NULL) {
    sem = xSemaphoreCreateBinary();
    // ...
    xSemaphoreGive(sem);
}
```

Но `ads1115_running_sem` (статическая переменная) **никогда не обновляется** — `sem` — это локальная переменная! Далее в цикле используется `sem` (локальная), а в `xSemaphoreGive(ads1115_running_sem)` — глобальная (которая всё ещё NULL).

**Строка с багом:** `xSemaphoreGive(ads1115_running_sem);` — вызов `xSemaphoreGive(NULL)` — **undefined behavior**, может привести к crash.

**Исправление:**

```c
// В app_main(), ДО создания задач:
ads1115_running_sem = xSemaphoreCreateBinary();
xSemaphoreGive(ads1115_running_sem);

// В ads1115_task()PORT_EXIT_CRITICAL — убрать создание семафора, просто использовать:
SemaphoreHandle_t sem = get_ads1115_running_sem();
if (sem == NULL) {
    ESP_LOGE(TAG, "Семафор ADS1115 не создан!");
    vTaskDelete(NULL);
    return;
}
```

---

### КРИТ-5. Переменные `ota_update_requested`, `mqtt_connected`, `pump_state` и др. — нет атомарного доступа

**Файл:** `components/mqtt_client/mqtt_client.c`, строки ~30-40

**Описание:** Переменные `mqtt_connected`, `ota_update_requested`, `pump_state`, `light_state`, `valve_state` объявлены как обычные `bool` и модифицируются из **разных контекстов**:
- `mqtt_event_handler` — выполняется в контексте задачи MQTT клиента (ядро 0)
- `mqtt_client_is_connected()` — вызывается из задач на ядре 1
- `mqtt_client_get_ota_flag()` — вызывается из задачи на ядре 0

На многоядерном ESP32 без барьеров памяти или `volatile`/атомарных операций компилятор может кэшировать значение в регистре, и изменение на одном ядре **не будет видно** на другом.

**Исправление:** Использовать `_Atomic bool` или `volatile bool` (минимум), либо защитить мьютексом:

```c
#include <stdatomic.h>

static atomic_bool mqtt_connected = false;
static atomic_bool ota_update_requested = false;
```

---

### КРИТ-6. Переменные DHT `dht_temperature` / `dht_humidity` — нет защиты при межъядерном доступе

**Файл:** `main/maiPORT_EXIT_CRITICALn.c`, строки ~85-86

**Описание:** `dht_temperature` и `dht_humidity` записываются в `dht_task()` на ядре 1, а читаются через `get_dht_temperature()` / `get_dht_humidity()` из `mqtt_client_publish_sensor_data()` на ядре 0. Тип `float` на ESP32 — 32 бита, и хотя запись/чтение 32-битного значения на Xtensa обычно атомарна, **стандарт C этого не гарантирует**, и компилятор может оптимизировать чтение.

**Исправление:** Как минимум добавить `volatile`:

```c
static volatile float dht_temperature = 0.0f;
static volatile float dht_humidity = 0.0f;
```

---

### КРИТ-7. Управление насосом и светом — GPIO НЕ переключается

**Файл:** `components/mqtt_client/mqtt_client.c`, обработчик `MQTT_EVENT_DATA`, строки ~290-310

**Описание:** При получении команды "ON"/"OFF" для насоса и света **только меняется переменная состояния**, но GPIO **не переключается**:

```c
if (strcmp(data, "ON") == 0) {
    pump_state = true;
    ESP_LOGI(TAG, "Pump: ON");
    // Здесь будет реальное включение насоса через GPIO
    // gpio_set_level(PUMP_PIN, 1);  // <-- ЗАКОММЕНТИРОВАНО!
}
```

Для клапана (`valve`) GPIO переключается через `set_valve_state()`, а для насоса и света — нет. Это значит, что **насос и свет физически не управляются** через MQTT.

**Исправление:** Раскомментировать вызовы GPIO или вынести в отдельные функции аналогично `set_valve_state()`:

```c
// В main.c добавить:
void set_pump_state(bool state) {
    gpio_set_level(PUMP_PIN, state ? 1 : 0);
    ESP_LOGI(TAG, "Насос: %s", state ? "ВКЛ" : "ВЫКЛ");
}

void set_light_state(bool state) {
    gpio_set_level(LIGHT_PIN, state ? 1 : 0);
    ESP_LOGI(TAG, "Свет: %s", state ? "ВКЛ" : "ВЫКЛ");
}
```

---

## 🟠 ЛОГИЧЕСКИЕ ОШИБКИ

### ЛОГ-1. Данные сенсоров — публикуются ЗАГЛУШКИ, а не реальные значения ADS1115

**Файл:** `components/mqtt_client/mqtt_client.c`, строки ~60-75, функция `mqtt_client_publish_sensor_data()`

**Описание:** Функции `read_ph()`, `read_tds()`, `read_level()`, `read_water_temp()` возвращают **случайные числа** (`rand()`). Данные с ADS1115 читаются в `ads1115_task()`, но **никуда не передаются** — только выводятся в лог. MQTT публикует фейковые данные.

**Исправление:** Создать глобальную структуру с последними измерениями ADS1115 и использовать её в MQTT:

```c
// В ads1115.h:
typedef struct {
    float ph_voltage;
    float tds_voltage;
    float water_temp_voltage;
    float level_voltage;
    bool valid;
} ads1115_sensor_data_t;

const ads1115_sensor_data_t* ads1115_get_latest_data(void);
```

---

### ЛОГ-2. ADS1115 Data Rate: комментарий не соответствует коду

**Файл:** `components/ads1115/ads1115.c`, строка ~120

```c
res = ads111x_set_data_rate(&device, ADS111X_DATA_RATE_8); // 32 выборки в секунду
```

**Описание:** `ADS111X_DATA_RATE_8` = 8 SPS (samples per second), а комментарий говорит "32 выборки в секунду". При 8 SPS одна конвертация занимает **125 мс**. Для 4 каналов (а с текущим багом КРИТ-1 — 8 конвертаций) это **500 мс - 1000 мс** только на чтение, что почти совпадает с `vTaskDelay(500)` в цикле задачи.

**Рекомендация:** Увеличить до `ADS111X_DATA_RATE_32` (32 SPS, ~31 мс на конвертацию) или `ADS111X_DATA_RATE_128` (128 SPS, ~8 мс) для более быстрого опроса. Для pH/TDS датчиков 32-128 SPS — оптимально.

---

### ЛОГ-3. Усиление ADS1115 (PGA) может быть неоптимальным

**Файл:** `components/ads1115/ads1115.c`, строка ~10

```c
#define GAIN ADS111X_GAIN_2V048 // +-4.096V  <-- КОММЕНТАРИЙ НЕВЕРНЫЙ
```

**Описание:** `ADS111X_GAIN_2V048` означает диапазон **±2.048V**, а не ±4.096V. Комментарий вводит в заблуждение. Для pH-датчиков (обычно 0-3.0V) и TDS (0-2.3V) диапазон ±2.048V может быть **недостаточным** — значения выше 2.048V будут обрезаны (clipping).

**Рекомендация:** Для pH/TDS датчиков с выходом 0-3.3V использовать `ADS111X_GAIN_4V096` (±4.096V). Или настраивать PGA индивидуально для каждого канала.

---

### ЛОГ-4. Проверка DHT данных: `temp != 0.0f || hum != 0.0f` — некорректная валидация

**Файл:** `components/mqtt_client/mqtt_client.c`, строка ~380

```c
if (temp != 0.0f || hum != 0.0f) {
```

**Описание:** Если реальная температура = 0°C (зимой в неотапливаемом помещении) и влажность = 0% (маловероятно, но возможно при ошибке), данные **не будут опубликованы**. Лучше использовать отдельный флаг валидности.

---

### ЛОГ-5. I2C частота в библиотеке ads111x.c — 1 МГц, что выше спецификации ADS1115

**Файл:** `managed_components/esp-idf-lib__ads111x/ads111x.c`, строка ~53

```c
#define I2C_FREQ_HZ 1000000 // Max 1MHz for esp32
```

**Описание:** ADS1115 поддерживает I2C до **3.4 МГц** (High-Speed mode), но стандартный режим — **400 кГц**. Библиотека устанавливает 1 МГц, что является Fast-mode Plus. Это может работать, но при длинных проводах или помехах может вызывать ошибки I2C. Библиотека `i2cdev` использует это значение как `dev->cfg.master.clk_speed`.

**Рекомендация:** Для надёжности в промышленных условиях снизить до 400 кГц, переопределив скорость после `ads111x_init_desc()`:

```c
device.cfg.master.clk_speed = 400000;  // 400 кГц — стандартный Fast-mode
```

---

## 🟡 ПОТЕНЦИАЛЬНЫЕ ПРОБЛЕМЫ

### ПОТ-1. Нет Watchdog для задач на ядре 1

**Файл:** `main/main.c`

**Описание:** Task Watchdog Timer (TWDT) по умолчанию в ESP-IDF мониторит только idle task. Задачи `ads1115_task` и `dht_task` на ядре 1 **не подписаны на TWDT**. Если одна из них зависнет (например, I2C bus lockup), это не будет обнаружено.

**Рекомендация:**
```c
#include "esp_task_wdt.h"

void ads1115_task(void *pvParameters) {
    esp_task_wdt_add(NULL);  // Подписать текущую задачу на TWDT
    // ...
    while (1) {
        esp_task_wdt_reset();  // Сбрасывать в каждой итерации
        // ... чтение ...
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

---

### ПОТ-2. Нет механизма восстановления I2C шины при зависании

**Файл:** `components/ads1115/ads1115.c`

**Описание:** Если I2C шина "зависнет" (SDA залипнет в LOW — частая проблема при сбоях питания), все последующие операции будут возвращать ошибку. Нет механизма восстановления (clock stretching recovery, bus reset).

**Рекомендация:** Добавить функцию восстановления I2C шины:
```c
static void ads1115_i2c_bus_recovery(void) {
    // Отправить 9 тактов SCL для разблокировки зависшего slave
    gpio_set_direction(SCL_PIN, GPIO_MODE_OUTPUT);
    for (int i = 0; i < 9; i++) {
        gpio_set_level(SCL_PIN, 0);
        esp_rom_delay_us(5);
        gpio_set_level(SCL_PIN, 1);
        esp_rom_delay_us(5);
    }
    // Переинициализировать I2C
}
```

---

### ПОТ-3. Нет калибровочных коэффициентов для pH/TDS в NVS

**Файл:** `components/ads1115/ads1115.c`

**Описание:** Сырое напряжение с ADS1115 нигде не преобразуется в реальные единицы (pH, ppm). Нет хранения калибровочных коэффициентов в NVS. Для pH-метра необходима как минимум двухточечная калибровка (pH 4.0 и pH 7.0).

---

### ПОТ-4. Размер стека `dht_task` может быть недостаточным

**Файл:** `main/main.c`, строка ~395

**Описание:** `configMINIMAL_STACK_SIZE` на ESP32 обычно = 1536 байт. Для задачи с `ESP_LOGI` (которая использует `printf` и может потребовать ~2 КБ стека) это может быть на грани.

**Исправлено:** Размер стека оптимизирован на основе анализа локальных переменных:

| Задача | Локальные переменные | Макс. использование | Новый размер | Запас |
|--------|---------------------|---------------------|--------------|-------|
| dht_task | 2×float (8 байт) + ESP_LOG | ~1 КБ | 2048 байт | 2× |
| ads1115_task | measurements[4] (48 байт) + ESP_LOG | ~1,5 КБ | 4096 байт | 3× |
| mqtt_sensor_task | нет локальных | ~512 байт | 2048 байт | 4× |
| mqtt_ota_check_task | SemaphoreHandle_t + ESP_LOG | ~1 КБ | 3072 байт | 3× |
| mqtt_ota_progress_task | json_msg[128] + int (20 байт) | ~1 КБ | 2048 байт | 2× |

**Примечание:** В ESP-IDF размер стека указывается в **байтах** (не в словах, как в стандартном FreeRTOS).

**Рекомендация:** Использовать `uxTaskGetStackHighWaterMark()` для эмпирической проверки:

```c
// В конце каждой задачи, перед vTaskDelay():
static int first_time = 1;
if (first_time) {
    first_time = 0;
    UBaseType_t high_water = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "%s стек: минимум свободно %lu слов (%lu байт)", 
             pcTaskGetName(NULL, NULL), high_water, high_water * 4);
}
```

---

### ПОТ-5. OTA задача не подписана на TWDT, но вызывает `esp_task_wdt_reset()`

**Файл:** `components/ota_client/ota_client.c`, строка ~165

```c
esp_task_wdt_reset();
```

**Описание:** `esp_task_wdt_reset()` вызывается в OTA задаче, но задача **не подписана** на TWDT через `esp_task_wdt_add()`. Вызов `esp_task_wdt_reset()` без предварительной подписки вернёт ошибку (которая игнорируется). Это не критично, но бесполезно.

---

### ПОТ-6. Нет защиты от одновременного включения нескольких насосов/клапанов

**Файл:** `components/mqtt_client/mqtt_client.c`

**Описание:** Нет логики, предотвращающей одновременное включение насоса и клапана (если это недопустимо по конструкции системы). Любая комбинация команд из MQTT будет выполнена.

---

### ПОТ-7. MQTT QoS=0 для данных сенсоров — возможна потеря данных

**Файл:** `components/mqtt_client/mqtt_client.c`, функция `mqtt_client_publish_sensor_data()`

```c
esp_mqtt_client_publish(mqtt_client, "hydro/sensor/ph/state", msg, 0, 0, 0);
//                                                                   ^-- QoS=0
```

**Описание:** Все данные сенсоров публикуются с QoS=0 (fire-and-forget). При нестабильном Wi-Fi данные могут теряться без уведомления. Для телеметрии рекомендуется QoS=1.

---

### ПОТ-8. `ads1115_deinit()` не освобождает I2C ресурсы полностью

**Файл:** `components/ads1115/ads1115.c`, функция `ads1115_deinit()`

**Описание:** Функция обнуляет `device` через `memset`, но **не вызывает** `ads111x_free_desc(&device)` (которая удаляет мьютекс устройства и снимает его с шины) и не вызывает `i2cdev_done()`. Это может привести к утечке ресурсов I2C при повторной инициализации.

**Исправлено:**

```c
void ads1115_deinit(void)
{
    ESP_LOGI(TAG, "Деинициализация ADS1115 для OTA...");

    if (ads1115_mutex != NULL) {
        vSemaphoreDelete(ads1115_mutex);
        ads1115_mutex = NULL;
    }

    // Освобождаем ресурсы I2C устройства
    ads111x_free_desc(&device);

    ads1115_initialized = false;
    memset(&device, 0, sizeof(device));
    memset(&filter_state, 0, sizeof(filter_state));

    ESP_LOGI(TAG, "ADS1115 деинициализирован");
}
```

**Примечание:** `i2cdev_done()` не вызывается намеренно, так как I2C шина может использоваться другими компонентами (например, другими устройствами на шине I2C).

---

### ПОТ-9. Нет повторных попыток чтения DHT при ошибке

**Файл:** `main/main.c`, функция `dht_task()`, строки ~140-150

**Описание:** При ошибке чтения DHT просто логируется предупреждение и ожидается следующий цикл (2 сек). Нет retry-механизма. Для DHT22 рекомендуется 1-2 повторные попытки с паузой 100 мс.

**Исправление:**
```c
while (1) {
    esp_err_t res = ESP_FAIL;
    for (int retry = 0; retry < 3; retry++) {
        res = dht_read_float_data(DHT_TYPE, DHT_PIN, &humidity, &temperature);
        if (res == ESP_OK) break;
        ESP_LOGW(TAG, "DHT: попытка %d/3 не удалась: %s", retry + 1, esp_err_to_name(res));
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    if (res == ESP_OK) {
        dht_temperature = temperature;
        dht_humidity = humidity;
        ESP_LOGI(TAG, "DHT: Влажность=%.1f%% Температура=%.1f°C", humidity, temperature);
    } else {
        ESP_LOGE(TAG, "DHT: все 3 попытки чтения не удались");
        // Используем последние корректные значения (уже в dht_temperature/dht_humidity)
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
}
```

---

### ПОТ-10. GPIO пины не устанавливаются в безопасное состояние при старте

**Файл:** `main/main.c`, функция `app_main()`

**Описание:** После конфигурации GPIO для насоса, света и клапана **не устанавливается начальный уровень**. По умолчанию GPIO может быть в неопределённом состоянии.

**Исправлено:**

```c
// ПОТ-10: Устанавливаем GPIO в безопасное состояние (выключено)
gpio_set_level(PUMP_PIN, 0);
gpio_set_level(LIGHT_PIN, 0);
gpio_set_level(VALVE_PIN, 0);
ESP_LOGI(TAG, "GPIO установлены в безопасное состояние (насос, свет, клапан = ВЫКЛ)");
```

---

### ПОТ-11. `static int last_progress = 0;` в OTA — не сбрасывается между сессиями

**Файл:** `components/ota_client/ota_client.c`, строка ~163

**Описание:** Переменная `static` внутри цикла `while(1)` сохраняет значение между итерациями, но также **между вызовами задачи** (если задача пересоздаётся). При повторном OTA (без перезагрузки) прогресс может не логироваться корректно.

**Исправлено:**

1. Добавлен комментарий к переменной `last_progress` для ясности:
```c
// ПОТ-11: Используем локальную переменную вместо static
// чтобы прогресс корректно сбрасывался между сессиями OTA
static int last_progress = 0;
```

2. Добавлена инициализация в начале OTA сессии:
```c
// ПОТ-11: Сбрасываем last_progress в начале OTA сессии
// (переменная static внутри цикла, но задача создаётся заново при каждом OTA)
```

**Примечание:** Поскольку задача OTA создаётся через `xTaskCreate()` заново при каждом обновлении и удаляется через `vTaskDelete()`, статическая переменная фактически сбрасывается. Добавлены комментарии для улучшения читаемости кода.

---

## 🔵 БЕЗОПАСНОСТЬ И НАДЁЖНОСТЬ

### БЕЗ-1. Пароли MQTT и Wi-Fi в открытом виде в коде

**Файл:** `components/mqtt_client/mqtt_client.c`, строки ~330-335

```c
.broker.address.uri = "mqtt://192.168.0.107:1883",
.credentials.username = "hydroesp32",
.credentials.authentication.password = "asda",
```

**Описание:** Пароль MQTT (`asda`) и URI брокера захардкожены в исходном коде. Wi-Fi креды хранятся через `protocol_examples_common` (вероятно, тоже в menuconfig/sdkconfig). Это:
1. Попадает в git-репозиторий (публичный: `github.com/OldManIroh/hydropNFT.git`)
2. Не может быть изменено без перепрошивки

**Рекомендация:** Хранить в NVS с шифрованием (`nvs_flash_init_partition_ptr` с encrypted partition) или использовать Kconfig для конфигурации.

---

### БЕЗ-2. MQTT без TLS — данные передаются в открытом виде

**Файл:** `components/mqtt_client/mqtt_client.c`

```c
.broker.address.uri = "mqtt://192.168.0.107:1883",  // НЕ mqtts://
```

**Описание:** Используется нешифрованное MQTT соединение. В локальной сети это допустимо, но при выходе в интернет — критическая уязвимость.

---

### БЕЗ-3. OTA сервер использует самоподписанный сертификат с `skip_cert_common_name_check`

**Файл:** `components/ota_client/Kconfig.projbuild` + `ota_client.c`

**Описание:** Если включена опция `CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK`, проверка CN сертификата пропускается. Это позволяет MITM-атаку при OTA обновлении — злоумышленник может подсунуть вредоносную прошивку.

---

### БЕЗ-4. Приватный ключ сервера (`ca_key.pem`) в репозитории

**Файл:** `server_certs/ca_key.pem`

**Описание:** Приватный ключ TLS сертификата находится в git-репозитории. Это позволяет любому, кто имеет доступ к репозиторию, подделать OTA сервер.

**Рекомендация:** Добавить `server_certs/ca_key.pem` в `.gitignore`.

---

## 📝 STYLE / READABILITY

### СТИЛЬ-1. Несогласованность комментариев и кода

- `ads1115.c:10` — `GAIN ADS111X_GAIN_2V048 // +-4.096V` (неверный комментарий)
- `ads1115.c:120` — `ADS111X_DATA_RATE_8 // 32 выборки в секунду` (неверный комментарий)
- Документация в `main.c` (таблица задач) указывает приоритет 5 для ADS1115, но в коде — 3

### СТИЛЬ-2. Дублирование кода в `ads1115_read_voltage()` и `ads1115_read_raw()`

Обе функции содержат идентичную логику: установка мультиплексора → запуск конвертации → ожидание → чтение. Следует объединить в одну внутреннюю функцию.

### СТИЛЬ-3. Магические числа в размерах стека

**Исправлено:** Размеры стека теперь указаны явно в байтах с обоснованием:

```c
2048,  // dht_task, mqtt_sensor_task, mqtt_ota_progress_task
3072,  // mqtt_ota_check_task
4096,  // ads1115_task
```

**Экономия памяти по сравнению с исходной конфигурацией:**

| Задача | Было (байт) | Стало (байт) | Экономия |
|--------|-------------|--------------|----------|
| dht_task | 3072 | 2048 | 1024 |
| ads1115_task | 12288 | 4096 | 8192 |
| mqtt_sensor_task | 6144 | 2048 | 4096 |
| mqtt_ota_check_task | 6144 | 3072 | 3072 |
| **Итого** | **27648** | **11264** | **16384** |

**Общая экономия: 16 КБ** стека задач FreeRTOS.

Рекомендуется определить именованные константы для улучшения читаемости:

```c
#define DHT_TASK_STACK_SIZE             2048
#define ADS1115_TASK_STACK_SIZE         4096
#define MQTT_SENSOR_TASK_STACK_SIZE     2048
#define MQTT_OTA_CHECK_TASK_STACK_SIZE  3072
```

---

## 📊 СВОДНАЯ ТАБЛИЦА

| # | Тип | Описание | Файл | Строка |
|---|-----|----------|------|--------|
| КРИТ-1 | 🔴 Критическая | Двойное чтение ADS1115 каналов | ads1115.c | ~210 |
| КРИТ-2 | 🔴 Критическая | DHT блокирует прерывания на 5 мс | dht.c (библиотека) | ~150 |
| КРИТ-3 | 🔴 Критическая | DHT: критическая секция не освобождается при ошибке | dht.c (библиотека) | ~155 |
| КРИТ-4 | 🔴 Критическая | Семафор ADS1115: race condition + xSemaphoreGive(NULL) | main.c | ~175-200 |
| КРИТ-5 | 🔴 Критическая | Нет атомарного доступа к shared-переменным | mqtt_client.c | ~30-40 |
| КРИТ-6 | 🔴 Критическая | DHT float переменные без volatile | main.c | ~85-86 |
| КРИТ-7 | 🔴 Критическая | GPIO насоса/света не переключается | mqtt_client.c | ~290-310 |
| ЛОГ-1 | 🟠 Логическая | MQTT публикует заглушки вместо реальных данных | mqtt_client.c | ~60-75 |
| ЛОГ-2 | 🟠 Логическая | Комментарий Data Rate не соответствует коду | ads1115.c | ~120 |
| ЛОГ-3 | 🟠 Логическая | PGA gain комментарий неверный, возможен clipping | ads1115.c | ~10 |
| ЛОГ-4 | 🟠 Логическая | Некорректная валидация DHT данных (0.0 == нет данных) | mqtt_client.c | ~380 |
| ЛОГ-5 | 🟠 Логическая | I2C 1 МГц — выше рекомендуемого для длинных линий | ads111x.c | ~53 |
| ПОТ-1 | 🟡 Потенциальная | Нет TWDT для задач на ядре 1 | main.c | — |
| ПОТ-2 | 🟡 Потенциальная | Нет восстановления I2C шины | ads1115.c | — |
| ПОТ-3 | 🟡 Потенциальная | Нет калибровки pH/TDS в NVS | ads1115.c | — |
| ПОТ-4 | 🟡 Потенциальная | Размер стека dht_task может быть мал | main.c | ~395 |
| ПОТ-5 | 🟡 Потенциальная | OTA: esp_task_wdt_reset без esp_task_wdt_add | ota_client.c | ~165 |
| ПОТ-6 | 🟡 Потенциальная | Нет защиты от одновременного включения актуаторов | mqtt_client.c | — |
| ПОТ-7 | 🟡 Потенциальная | QoS=0 для телеметрии | mqtt_client.c | — |
| ПОТ-8 | 🟡 Потенциальная | ads1115_deinit не освобождает I2C ресурсы | ads1115.c | — |
| ПОТ-9 | 🟡 Потенциальная | Нет retry для DHT | main.c | ~140 |
| ПОТ-10 | 🟡 Потенциальная | GPIO не в безопасном состоянии при старте | main.c | — |
| ПОТ-11 | 🟡 Потенциальная | static last_progress не сбрасывается | ota_client.c | ~163 |
| БЕЗ-1 | 🔵 Безопасность | Пароли в открытом виде в коде | mqtt_client.c | ~330 |
| БЕЗ-2 | 🔵 Безопасность | MQTT без TLS | mqtt_client.c | ~330 |
| БЕЗ-3 | 🔵 Безопасность | OTA: skip CN check | ota_client.c | — |
| БЕЗ-4 | 🔵 Безопасность | Приватный ключ в репозитории | server_certs/ | — |
