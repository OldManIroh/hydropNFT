# Настройка пересылки логов в Home Assistant

## Что делает компонент `log_forwarder`

Компонент перехватывает логи ESP-IDF уровней **WARNING** и **ERROR** и отправляет их в MQTT топик `hydro/log` для отображения в Home Assistant.

### Возможности:
- Перехват логов через `esp_log_set_vprintf()`
- Фильтрация: только WARNING (W) и ERROR (E)
- Корректный парсинг ANSI escape-кодов цветов (работает с включёнными и выключенными цветами)
- Очистка escape-кодов перед отправкой в MQTT (чистый текст)
- Защита от рекурсии (когда сам компонент генерирует логи)
- Throttling — ограничение частоты отправки (200 мс)
- Мьютекс для потокобезопасности буфера
- MQTT Discovery — автоматическая регистрация сенсора в Home Assistant
- Дублирование логов в UART (консоль не теряется)

## Настройка в menuconfig

### 1. Уровень логирования

Установите уровень логирования Warning, чтобы оставить только WARNING и ERROR:

```
Component config → Log output → Default log verbosity → Warning
```

### 2. ANSI цвета (опционально)

Компонент корректно работает **с включёнными и выключенными** цветами.
Но для экономии трафика можно отключить:

```
Component config → Log output → Use ANSI terminal colors → OFF
```

## Как это работает

### Архитектура

```
ESP_LOGW/ESP_LOGE → esp_log_set_vprintf() → log_vprintf()
                                                  │
                                    ┌─────────────┼─────────────┐
                                    │             │             │
                                    v             v             v
                                 vprintf()   extract_log_level()  mqtt_publish()
                                 (UART)      (фильтрация W/E)    (hydro/log)
```

### Формат строки лога ESP-IDF

С ANSI цветами (по умолчанию):
```
\033[0;31mE (12345) mqtt_client: Connection lost\033[0m\n
│         │                                      │
│         └─ Символ уровня (E/W/I/D/V)          └─ Сброс цвета
└─ Escape-последовательность цвета
```

Без цветов:
```
E (12345) mqtt_client: Connection lost\n
│
└─ Символ уровня
```

### Важные детали реализации

1. **`va_copy()`** — `va_list` можно использовать только один раз. Для `vprintf()` и `vsnprintf()` нужна копия через `va_copy()`.

2. **Парсинг ANSI** — `extract_log_level()` пропускает escape-последовательность `\033[...m` и находит символ уровня после неё.

3. **Очистка ANSI** — `strip_ansi_codes()` удаляет все escape-коды перед отправкой в MQTT.

4. **Защита от рекурсии** — флаг `s_inside_log_handler` предотвращает бесконечный цикл, когда `esp_mqtt_client_publish()` внутри обработчика генерирует свои логи.

5. **MQTT Discovery** — при старте запускается задача, которая ждёт подключения к MQTT и отправляет конфигурацию Discovery.

## Настройка Home Assistant

### Автоматическое обнаружение (MQTT Discovery)

Компонент автоматически отправляет Discovery конфигурацию при подключении к MQTT.
Сенсор `sensor.hydroponic_system_logi_ustrojstva` появится автоматически в HA.

### Ручная настройка через configuration.yaml

Если Discovery не работает, добавьте в `configuration.yaml`:

```yaml
mqtt:
  sensor:
    - name: "HydroNFT Logs"
      state_topic: "hydro/log"
      icon: "mdi:math-log"
      entity_category: "diagnostic"
```

### Dashboard (Lovelace UI)

Добавьте карточку для отображения последнего лога:

```yaml
type: entities
entities:
  - entity: sensor.hydroponic_system_logi_ustrojstva
    name: "Последний лог"
    icon: "mdi:math-log"
```

Или используйте карточку Markdown для более красивого отображения:

```yaml
type: markdown
title: "Логи устройства"
content: "{{ states('sensor.hydroponic_system_logi_ustrojstva') }}"
```

### Автоматизация уведомлений об ошибках

Создайте автоматизацию для получения уведомлений при ошибках:

```yaml
automation:
  - alias: "HydroNFT Error Notification"
    trigger:
      - platform: state
        entity_id: sensor.hydroponic_system_logi_ustrojstva
    condition:
      - condition: template
        value_template: "{{ trigger.to_state.state.startswith('E ') }}"
    action:
      - service: notify.mobile_app_your_phone
        data:
          title: "HydroNFT Ошибка!"
          message: "{{ trigger.to_state.state }}"
```

## Проверка работы

1. **Прошейте устройство:**
   ```bash
   idf.py build flash monitor
   ```

2. **Проверьте MQTT через mosquitto_sub:**
   ```bash
   mosquitto_sub -h 192.168.0.107 -u hydroesp32 -P asda -t "hydro/log" -v
   ```

3. **Проверьте Discovery:**
   ```bash
   mosquitto_sub -h 192.168.0.107 -u hydroesp32 -P asda -t "homeassistant/sensor/hydroesp32/log/config" -v
   ```

4. **В Home Assistant:**
   - Developer Tools → States → найдите `sensor.hydroponic_system_logi_ustrojstva`

## Топики MQTT

| Топик | Описание | QoS | Retain |
|-------|----------|-----|--------|
| `hydro/log` | Логи WARNING и ERROR | 1 | 0 |
| `homeassistant/sensor/hydroesp32/log/config` | MQTT Discovery конфигурация | 1 | 1 |

## API компонента

```c
// Инициализация (вызывается в app_main ПОСЛЕ mqtt_client_init)
log_forwarder_init();

// Включить/выключить пересылку
log_forwarder_enable(bool enable);

// Проверка состояния
log_forwarder_is_enabled();
```

## Структура компонента

```
components/log_forwarder/
├── CMakeLists.txt          # Конфигурация сборки (зависимости: mqtt_client, mqtt, esp_timer)
├── log_forwarder.c         # Реализация
└── include/
    └── log_forwarder.h     # Заголовочный файл
```

## Исправленные проблемы (v2.0)

| Проблема | Описание | Решение |
|----------|----------|---------|
| Двойное использование va_list | UB — va_list невалиден после первого использования | `va_copy()` |
| Парсинг уровня лога | `fmt[0]` не работает с ANSI цветами | `extract_log_level()` с парсингом escape-кодов |
| ANSI коды в MQTT | Escape-коды мусорят в HA | `strip_ansi_codes()` |
| Рекурсия | `esp_mqtt_client_publish()` генерирует логи → бесконечный цикл | Флаг `s_inside_log_handler` |
| Нет MQTT Discovery | Сенсор не появляется автоматически в HA | `log_forwarder_send_discovery()` |
| CMakeLists.txt | Неправильные зависимости | Добавлен `PRIV_REQUIRES mqtt` |
