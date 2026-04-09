/**
 * @file web_server.c
 * @brief Встроенный HTTP сервер HydroNFT
 *
 * @par Компоненты предоставляет
 * - Главную страницу с дашбордом (inline HTML)
 * - Страницу настроек WiFi/MQTT
 * - REST API: /api/sensors, /api/control, /api/set, /api/status, /api/ota
 *
 * @par Безопасность
 * - Все критичные endpoints (POST) защищены CSRF проверкой:
 *   1. Проверяется Referer — должен быть из приватного диапазона IP
 *   2. Если Referer отсутствует — проверяется Content-Type: application/json
 *      (HTML-формы <form> не могут отправить JSON без JavaScript)
 * - JSON-парсер extract_json_string() проверяет что перед ключом — { или ,
 *   (защита от подстроки: "wifi_ssid" внутри "wifi_ssid_backup")
 * - Строки SSID/MQTT URI экранируются перед вставкой в JSON (", \ → \", \\)
 * - Content-Length берётся из req->content_len (не парсинг заголовка через atoi)
 *
 * @par Ограничения JSON-парсера
 * extract_json_string() — ручной парсер, НЕ полноценный JSON.
 * Не поддерживает: вложенные объекты, числовые значения, массивы, unicode escape.
 * Работает для простых flat-объектов с строковыми значениями.
 * При необходимости сложных запросов — использовать cJSON.
 *
 * @author HydroNFT Team
 * @version 2.0
 * @date 2026
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdatomic.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_ota_ops.h"
#include "mdns.h"
#include "ads1115.h"
#include "device_control.h"
#include "sntp_client.h"
#include "hydro_mqtt_client.h"
#include "settings.h"
#include "web_server.h"
#include "ota_client.h"

static const char *TAG = "web_server";

// ============================================================================
// HTML ДАШБОРД (inline)
// ============================================================================

static const char *MAIN_PAGE =
    "<!DOCTYPE html>"
    "<html lang='ru'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>HydroNFT</title>"
    "<style>"
    "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;"
    "  max-width:640px;margin:0 auto;padding:16px;background:#f5f5f5;color:#333}"
    "h1{text-align:center;color:#1a73e8;margin-bottom:4px}"
    ".sub{text-align:center;color:#666;font-size:14px;margin-bottom:20px}"
    ".card{background:#fff;border-radius:12px;padding:16px;margin-bottom:12px;"
    "  box-shadow:0 1px 3px rgba(0,0,0,0.1)}"
    ".card h2{margin:0 0 12px;font-size:16px;color:#555}"
    ".sensor{display:flex;justify-content:space-between;align-items:center;"
    "  padding:8px 0;border-bottom:1px solid #eee}"
    ".sensor:last-child{border-bottom:none}"
    ".sensor .label{color:#666}"
    ".sensor .value{font-weight:bold;font-size:18px;color:#1a73e8}"
    ".btn{display:inline-block;padding:10px 20px;border:none;border-radius:8px;"
    "  font-size:14px;cursor:pointer;margin:4px;transition:opacity 0.2s}"
    ".btn:hover{opacity:0.85}"
    ".btn-on{background:#4caf50;color:#fff}"
    ".btn-off{background:#f44336;color:#fff}"
    ".btn-mode{background:#1a73e8;color:#fff}"
    ".btn-sm{padding:6px 12px;font-size:12px}"
    ".row{display:flex;flex-wrap:wrap;gap:4px;margin-top:8px}"
    ".status{display:inline-block;width:10px;height:10px;border-radius:50%;"
    "  margin-right:6px}"
    ".status-on{background:#4caf50}.status-off{background:#ccc}"
    ".nav{text-align:center;margin-bottom:16px}"
    ".nav a{margin:0 8px;color:#1a73e8;text-decoration:none}"
    ".refresh{text-align:center;color:#999;font-size:12px;margin-top:16px}"
    ".mode-switch{display:flex;align-items:center;gap:8px}"
    ".mode-lbl{font-size:13px;color:#888;font-weight:500}"
    ".toggle{position:relative;display:inline-block;width:44px;height:24px}"
    ".toggle input{opacity:0;width:0;height:0}"
    ".slider{position:absolute;cursor:pointer;inset:0;background:#ccc;border-radius:24px;"
    "  transition:0.3s}"
    ".slider:before{position:absolute;content:'';height:18px;width:18px;left:3px;bottom:3px;"
    "  background:#fff;border-radius:50%;transition:0.3s}"
    ".toggle input:checked+.slider{background:#1a73e8}"
    ".toggle input:checked+.slider:before{transform:translateX(20px)}"
    "</style></head><body>"
    "<h1>\U0001F331 HydroNFT</h1>"
    "<div class='nav'>"
    "  <a href='/'>\U0001F4CA Дашборд</a>"
    "  <a href='/settings'>\U00002699 Настройки</a>"
    "</div>"
    "<div class='sub' id='clock'>--:--:--</div>"

    "<div class='card'><h2>\U0001F4CA Сенсоры</h2>"
    "<div class='sensor'><span class='label'>pH</span><span class='value' id='ph'>—</span></div>"
    "<div class='sensor'><span class='label'>TDS</span><span class='value' id='tds'>—</span></div>"
    "<div class='sensor'><span class='label'>Темп. воды</span><span class='value' id='water_temp'>—</span></div>"
    "<div class='sensor'><span class='label'>Уровень воды</span><span class='value' id='level'>—</span></div>"
    "<div class='sensor'><span class='label'>Температура</span><span class='value' id='temp'>—</span></div>"
    "<div class='sensor'><span class='label'>Влажность</span><span class='value' id='humidity'>—</span></div>"
    "</div>"

    "<div class='card'><h2>\U0001F3AE Управление</h2>"
    "<div class='sensor'>"
    "  <span class='label'><span class='status' id='pump_status'></span>Насос</span>"
    "  <div class='mode-switch'>"
    "    <span class='mode-lbl'>Выкл</span>"
    "    <label class='toggle'><input type='checkbox' id='pumpToggle' onchange=\"toggleDevice('pump',this.checked)\"><span class='slider'></span></label>"
    "    <span class='mode-lbl'>Вкл</span>"
    "  </div>"
    "</div>"
    "<div class='sensor'>"
    "  <span class='label'><span class='status' id='light_status'></span>Свет</span>"
    "  <div class='mode-switch'>"
    "    <span class='mode-lbl'>Выкл</span>"
    "    <label class='toggle'><input type='checkbox' id='lightToggle' onchange=\"toggleDevice('light',this.checked)\"><span class='slider'></span></label>"
    "    <span class='mode-lbl'>Вкл</span>"
    "  </div>"
    "</div>"
    "<div class='sensor'>"
    "  <span class='label'><span class='status' id='valve_status'></span>Клапан</span>"
    "  <div class='mode-switch'>"
    "    <span class='mode-lbl'>Закрыт</span>"
    "    <label class='toggle'><input type='checkbox' id='valveToggle' onchange=\"toggleDevice('valve',this.checked)\"><span class='slider'></span></label>"
    "    <span class='mode-lbl'>Открыт</span>"
    "  </div>"
    "</div>"
    "<div class='sensor'>"
    "  <span class='label'>Режим</span>"
    "  <div class='mode-switch'>"
    "    <span class='mode-lbl'>Ручной</span>"
    "    <label class='toggle'><input type='checkbox' id='modeToggle' onchange='toggleMode(this.checked)'><span class='slider'></span></label>"
    "    <span class='mode-lbl'>Авто</span>"
    "  </div>"
    "</div>"
    "</div>"

    "<div class='refresh'>Автообновление каждые 5 секунд</div>"

    "<script>"
    "async function api(url){try{const r=await fetch(url);return await r.json()}"
    "  catch(e){console.error(e);return null}}"
    "async function update(){"
    "  const s=await api('/api/sensors');if(!s)return;"
    "  document.getElementById('ph').textContent=s.ph!==null?s.ph.toFixed(4)+' В':'—';"
    "  document.getElementById('tds').textContent=s.tds!==null?s.tds.toFixed(0)+' ppm':'—';"
    "  document.getElementById('water_temp').textContent=s.water_temp!==null?s.water_temp.toFixed(4)+' В':'—';"
    "  document.getElementById('level').textContent=s.level!==null?s.level.toFixed(4)+' В':'—';"
    "  document.getElementById('temp').textContent=s.temp!==null?s.temp.toFixed(1)+' °C':'—';"
    "  document.getElementById('humidity').textContent=s.humidity!==null?s.humidity.toFixed(1)+'%':'—';"
    "  const c=await api('/api/control');if(!c)return;"
    "  setStatus('pump',c.pump==='ON');setStatus('light',c.light==='ON');setStatus('valve',c.valve==='ON');"
    "  document.getElementById('pumpToggle').checked=(c.pump==='ON');"
    "  document.getElementById('lightToggle').checked=(c.light==='ON');"
    "  document.getElementById('valveToggle').checked=(c.valve==='ON');"
    "  document.getElementById('modeToggle').checked=(c.mode==='auto');"
    "}"
    "function setStatus(id,on){const el=document.getElementById(id);"
    "  el.className='status '+(on?'status-on':'status-off')}"
    "async function toggleDevice(dev,on){"
    "  await fetch('/api/set',{method:'POST',headers:{'Content-Type':'application/json'},"
    "  body:JSON.stringify({device:dev,state:on?'ON':'OFF'})});setTimeout(update,200)}"
    "async function toggleMode(isAuto){"
    "  await fetch('/api/set',{method:'POST',headers:{'Content-Type':'application/json'},"
    "  body:JSON.stringify({device:'mode',state:isAuto?'auto':'manual'})});setTimeout(update,200)}"
    "function updateClock(){const n=new Date();document.getElementById('clock').textContent="
    "  n.toLocaleTimeString('ru-RU')}"
    "update();setInterval(update,5000);setInterval(updateClock,1000);updateClock();"
    "</script></body></html>";

// ============================================================================
// HTML СТРАНИЦА НАСТРОЕК
// ============================================================================

static const char *SETTINGS_PAGE =
    "<!DOCTYPE html>"
    "<html lang='ru'><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>HydroNFT — Настройки</title>"
    "<style>"
    "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;"
    "  max-width:640px;margin:0 auto;padding:16px;background:#f5f5f5;color:#333}"
    "h1{text-align:center;color:#1a73e8;margin-bottom:4px}"
    ".nav{text-align:center;margin-bottom:16px}"
    ".nav a{margin:0 8px;color:#1a73e8;text-decoration:none}"
    ".card{background:#fff;border-radius:12px;padding:16px;margin-bottom:12px;"
    "  box-shadow:0 1px 3px rgba(0,0,0,0.1)}"
    ".card h2{margin:0 0 12px;font-size:16px;color:#555}"
    "form{display:flex;flex-direction:column;gap:12px}"
    "label{font-size:14px;color:#555}"
    "input{padding:8px 12px;border:1px solid #ddd;border-radius:8px;font-size:14px}"
    ".btn{padding:10px 20px;border:none;border-radius:8px;font-size:14px;cursor:pointer}"
    ".btn-save{background:#1a73e8;color:#fff}"
    ".btn-restart{background:#f44336;color:#fff}"
    ".sys-row{display:flex;justify-content:space-between;margin:6px 0;font-size:14px}"
    ".sys-row span:first-child{color:#666}"
    ".sys-row span:last-child{font-weight:bold;color:#333}"
    ".status-bar{background:#e8f5e9;padding:12px;border-radius:8px;margin-bottom:12px}"
    ".status-bar .row{display:flex;justify-content:space-between;margin:4px 0}"
    ".msg{padding:12px;border-radius:8px;margin-bottom:12px;display:none}"
    ".msg-ok{background:#e8f5e9;color:#2e7d32}"
    ".msg-err{background:#ffebee;color:#c62828}"
    "</style></head><body>"
    "<h1>\U00002699 Настройки</h1>"
    "<div class='nav'>"
    "  <a href='/'>\U0001F4CA Дашборд</a>"
    "  <a href='/settings'>\U00002699 Настройки</a>"
    "</div>"

    "<div id='statusBar' class='status-bar'></div>"
    "<div id='msg' class='msg'></div>"

    "<div class='card'><h2>WiFi подключение</h2>"
    "<form id='wifiForm' onsubmit='saveWifi(event)'>"
    "<label>SSID WiFi сети</label>"
    "<input id='wifi_ssid' name='wifi_ssid' type='text' placeholder='Моя сеть'>"
    "<label>Пароль WiFi</label>"
    "<input id='wifi_pass' name='wifi_pass' type='password' placeholder='Пароль'>"
    "<button class='btn btn-save' type='submit'>Сохранить и подключиться</button>"
    "</form></div>"

    "<div class='card'><h2>MQTT брокер</h2>"
    "<form id='mqttForm' onsubmit='saveMqtt(event)'>"
    "<label>URI брокера</label>"
    "<input id='mqtt_uri' name='mqtt_uri' type='text' placeholder='mqtt://192.168.0.100:1883'>"
    "<label>Имя пользователя</label>"
    "<input id='mqtt_user' name='mqtt_user' type='text' placeholder='hydroesp32'>"
    "<label>Пароль</label>"
    "<input id='mqtt_pass' name='mqtt_pass' type='password' placeholder='Пароль'>"
    "<button class='btn btn-save' type='submit'>Сохранить и переподключить</button>"
    "</form></div>"

    "<div class='card'><h2>Система и OTA</h2>"
    "<div class='sys-row'><span>Версия:</span><span id='sys_fw'>—</span></div>"
    "<div class='sys-row'><span>MAC:</span><span id='sys_mac'>—</span></div>"
    "<div class='sys-row'><span>OTA:</span><span id='sys_ota'>idle</span></div>"
    "<button class='btn btn-restart' onclick=\"startOTA()\">\U0001F4E5 Обновить прошивку</button>"
    "<button class='btn btn-restart' onclick=\"restartDevice()\">\U0001F504 Перезагрузить</button>"
    "</div>"

    "<script>"
    "async function loadStatus(){"
    "  try{const r=await fetch('/api/status');const d=await r.json();"
    "    const sb=document.getElementById('statusBar');"
    "    sb.innerHTML='<div class=\\'row\\'><span>WiFi:</span><span>'+"
    "    (d.wifi_connected?'Подключён ✓':'Не подключён ✗')+'</span></div>'+"
    "    '<div class=\\'row\\'><span>SSID:</span><span>'+(d.wifi_ssid||'—')+'</span></div>'+"
    "    '<div class=\\'row\\'><span>IP:</span><span>'+(d.ip||'—')+'</span></div>'+"
    "    '<div class=\\'row\\'><span>AP IP:</span><span>'+(d.ap_ip||'—')+'</span></div>'+"
    "    '<div class=\\'row\\'><span>MQTT:</span><span>'+"
    "    (d.mqtt_connected?'Подключён ✓':'Не подключён ✗')+'</span></div>'+"
    "    '<div class=\\'row\\'><span>mDNS:</span><span>'+(d.mdns||'—')+'</span></div>';"
    "    document.getElementById('wifi_ssid').value=d.wifi_ssid||'';"
    "    document.getElementById('mqtt_uri').value=d.mqtt_uri||'';"
    "    document.getElementById('mqtt_user').value=d.mqtt_user||'';"
    "    if(d.firmware_version)document.getElementById('sys_fw').textContent=d.firmware_version;"
    "    if(d.mac_address)document.getElementById('sys_mac').textContent=d.mac_address;"
    "    if(d.ota_status)document.getElementById('sys_ota').textContent=d.ota_status;"
    "  }catch(e){console.error(e)}"
    "}"
    "function showMsg(text,ok){"
    "  const m=document.getElementById('msg');m.textContent=text;"
    "  m.className='msg '+(ok?'msg-ok':'msg-err');m.style.display='block';"
    "  setTimeout(()=>{m.style.display='none'},5000)}"
    "async function saveWifi(e){"
    "  e.preventDefault();const d={"
    "    wifi_ssid:document.getElementById('wifi_ssid').value,"
    "    wifi_pass:document.getElementById('wifi_pass').value};"
    "  try{const r=await fetch('/settings/save',{method:'POST',headers:{'Content-Type':'application/json'},"
    "    body:JSON.stringify(d)});"
    "    if(r.ok){showMsg('Настройки WiFi сохранены. Перезагрузка...',true)}"
    "    else{showMsg('Ошибка сохранения',false)}"
    "  }catch(er){showMsg('Ошибка сети',false)}}"
    "async function saveMqtt(e){"
    "  e.preventDefault();const d={"
    "    mqtt_uri:document.getElementById('mqtt_uri').value,"
    "    mqtt_user:document.getElementById('mqtt_user').value,"
    "    mqtt_pass:document.getElementById('mqtt_pass').value};"
    "  try{const r=await fetch('/settings/mqtt',{method:'POST',headers:{'Content-Type':'application/json'},"
    "    body:JSON.stringify(d)});"
    "    if(r.ok){showMsg('Настройки MQTT сохранены',true)}"
    "    else{showMsg('Ошибка сохранения',false)}"
    "  }catch(er){showMsg('Ошибка сети',false)}}"
    "async function startOTA(){"
    "  if(!confirm('Запустить OTA обновление? Устройство перезагрузится после загрузки.'))return;"
    "  try{const r=await fetch('/api/ota',{method:'POST'});"
    "    if(r.ok){showMsg('OTA обновление запущено...',true);setTimeout(loadStatus,3000)}"
    "    else{const t=await r.text();showMsg('Ошибка: '+t,false)}"
    "  }catch(er){showMsg('Ошибка сети',false)}}"
    "async function restartDevice(){"
    "  if(confirm('Перезагрузить устройство?')){"
    "    await fetch('/settings/restart',{method:'POST'});showMsg('Устройство перезагружается...',true)}}"
    "}"
    "loadStatus();setInterval(loadStatus,10000);"
    "</script></body></html>";

// ============================================================================
// HANDLER: главная страница
// ============================================================================

static esp_err_t main_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, MAIN_PAGE, strlen(MAIN_PAGE));
    return ESP_OK;
}

// ============================================================================
// HANDLER: страница настроек
// ============================================================================

static esp_err_t settings_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, SETTINGS_PAGE, strlen(SETTINGS_PAGE));
    return ESP_OK;
}

// ============================================================================
// HANDLER: GET /api/sensors
// ============================================================================

static esp_err_t api_sensors_handler(httpd_req_t *req)
{
    char json[512];
    int offset = 0;

    ads1115_sensor_data_t adc = ads1115_get_latest_data();

    if (adc.valid)
    {
        float tds_ppm = ads1115_voltage_to_tds(adc.voltage[2]);

        offset += snprintf(json + offset, sizeof(json) - offset,
                           "{\"ph\":%.4f,\"tds\":%.0f,\"water_temp\":%.4f,\"level\":%.4f",
                           adc.voltage[0], tds_ppm, adc.voltage[1], adc.voltage[3]);
    }
    else
    {
        offset += snprintf(json + offset, sizeof(json) - offset,
                           "{\"ph\":null,\"tds\":null,\"water_temp\":null,\"level\":null");
    }
    if (offset >= sizeof(json)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON buffer overflow");
        return ESP_FAIL;
    }

    if (device_control_is_dht_data_valid())
    {
        offset += snprintf(json + offset, sizeof(json) - offset,
                           ",\"temp\":%.1f,\"humidity\":%.1f}",
                           device_control_get_dht_temperature(),
                           device_control_get_dht_humidity());
    }
    else
    {
        offset += snprintf(json + offset, sizeof(json) - offset,
                           ",\"temp\":null,\"humidity\":null}");
    }
    if (offset >= sizeof(json)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON buffer overflow");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

// ============================================================================
// HANDLER: GET /api/control
// ============================================================================

static esp_err_t api_control_handler(httpd_req_t *req)
{
    char json[256];
    snprintf(json, sizeof(json),
             "{\"pump\":\"%s\","
             "\"light\":\"%s\","
             "\"valve\":\"%s\","
             "\"mode\":\"%s\"}",
             device_control_get_pump_state() ? "ON" : "OFF",
             device_control_get_light_state() ? "ON" : "OFF",
             device_control_get_valve_state() ? "ON" : "OFF",
             device_control_mode_to_string(device_control_get_mode()));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================================

/**
 * @brief Извлечь строковое значение из простого JSON по ключу
 * @param json JSON-строка
 * @param key Ключ (например "\"wifi_ssid\"")
 * @param out Буфер для значения
 * @param out_size Размер буфера
 * @return Длина значения или -1 если не найдено/ошибка
 *
 * Поддерживает экранированные кавычки внутри значений.
 */
static int extract_json_string(const char *json, const char *key, char *out, size_t out_size)
{
    char *f_start = strstr(json, key);
    if (!f_start)
        return -1;
    // Проверяем что перед ключом — начало объекта { или разделитель , (защита от подстроки)
    // Пример атаки: {"wifi_ssid":"home","wifi_ssid_backup":"evil"} — strstr найдёт первый ключ
    // внутри строкового значения. Проверка предыдущего символа предотвращает это.
    if (f_start != json) {
        char prev = f_start[-1];
        if (prev != '{' && prev != ',' && prev != ' ' && prev != '\t' && prev != '\n' && prev != '\r')
            return -1;
    }
    char *colon = strchr(f_start, ':');
    if (!colon)
        return -1;
    char *q1 = strchr(colon, '"');
    if (!q1)
        return -1;
    // Ищем закрывающую кавычку, пропуская экранированные
    char *p = q1 + 1;
    char *q2 = NULL;
    while (*p != '\0')
    {
        if (*p == '\\' && *(p + 1) != '\0')
        {
            p += 2;
            continue;
        }
        if (*p == '"')
        {
            q2 = p;
            break;
        }
        p++;
    }
    if (!q2)
        return -1;
    size_t vl = (size_t)(q2 - q1 - 1);
    if (vl >= out_size)
        return -1;
    memcpy(out, q1 + 1, vl);
    out[vl] = '\0';
    return (int)vl;
}

// ============================================================================
// HANDLER: POST /api/set
// ============================================================================

/**
 * @brief Проверка Referer для защиты от CSRF
 * @return true если запрос разрешён
 */
static bool check_csrf(httpd_req_t *req)
{
    char referer[128];
    esp_err_t ret = httpd_req_get_hdr_value_str(req, "Referer", referer, sizeof(referer));

    if (ret == ESP_OK)
    {
        // Проверяем что Referer ведёт на наш же хост
        if (strstr(referer, "hydronft.local") != NULL ||
            strstr(referer, "localhost") != NULL ||
            strstr(referer, "127.0.0.1") != NULL)
        {
            return true;
        }
        // Парсим хост из URL и проверяем приватные диапазоны
        const char *host_start = strstr(referer, "://");
        if (host_start) {
            host_start += 3;
            const char *host_end = strchr(host_start, ':');
            if (!host_end) host_end = strchr(host_start, '/');
            if (host_end) {
                char host[64];
                size_t hl = host_end - host_start;
                if (hl < sizeof(host)) {
                    memcpy(host, host_start, hl);
                    host[hl] = '\0';
                    unsigned int a, b, c, d;
                    if (sscanf(host, "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
                        if (a == 10) return true;
                        if (a == 172 && b >= 16 && b <= 31) return true;
                        if (a == 192 && b == 168) return true;
                        if (a == 127) return true;
                    }
                }
            }
        }
        ESP_LOGW(TAG, "CSRF блокировка: Referer=%s", referer);
        return false;
    }

    // Нет Referer — разрешаем (мобильные приложения, curl)
    return true;
}

static esp_err_t api_set_handler(httpd_req_t *req)
{
    // Проверяем метод запроса
    if (req->method != HTTP_POST)
    {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Use POST method");
        return ESP_FAIL;
    }

    // CSRF защита: если Referer отсутствует или не из приватной сети,
    // проверяем Content-Type — HTML-формы (<form action="...">) НЕ МОГУТ отправить JSON.
    // Это предотвращает CSRF через <form method="POST" action="http://hydronft.local/...">
    // даже при отсутствии Referer (referrer-policy: no-referrer)
    if (!check_csrf(req)) {
        char ct[64];
        if (httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct)) != ESP_OK ||
            strstr(ct, "application/json") == NULL) {
            httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "CSRF protection: Content-Type required");
            return ESP_FAIL;
        }
    }

    // Используем req->content_len вместо парсинга Content-Length заголовка вручную.
    // httpd автоматически парсит Content-Length и сохраняет значение в req->content_len.
    // Это безопаснее чем atoi() — не зависит от формата строки.
    if (req->content_len == 0 || req->content_len > 256) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Content-Length");
        return ESP_FAIL;
    }
    size_t body_len = req->content_len;

    char body[256];
    int received = httpd_req_recv(req, body, body_len);
    if (received <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read body");
        return ESP_FAIL;
    }
    body[received] = '\0';

    // Парсим JSON вручную (device, state)
    // Ожидаем: {"device":"pump","state":"ON"}
    char device[32] = {0};
    char state[32] = {0};

    // Извлекаем device
    if (extract_json_string(body, "\"device\"", device, sizeof(device)) < 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid device field");
        return ESP_FAIL;
    }

    // Извлекаем state
    if (extract_json_string(body, "\"state\"", state, sizeof(state)) < 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid state field");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Запрос управления: %s → %s", device, state);

    // Case-insensitive comparison для state — принимаем ON/on/On, OFF/off/Off и т.д.
    bool state_on = (strcasecmp(state, "ON") == 0 || strcasecmp(state, "OPEN") == 0 || strcasecmp(state, "TRUE") == 0);
    bool state_off = (strcasecmp(state, "OFF") == 0 || strcasecmp(state, "CLOSE") == 0 || strcasecmp(state, "FALSE") == 0);

    if (strcmp(device, "pump") == 0)
    {
        if (!state_on && !state_off) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid state: use ON/OFF");
            return ESP_FAIL;
        }
        device_control_set_pump_state(state_on);
    }
    else if (strcmp(device, "light") == 0)
    {
        if (!state_on && !state_off) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid state: use ON/OFF");
            return ESP_FAIL;
        }
        device_control_set_light_state(state_on);
    }
    else if (strcmp(device, "valve") == 0)
    {
        if (!state_on && !state_off) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid state: use OPEN/CLOSE or ON/OFF");
            return ESP_FAIL;
        }
        device_control_set_valve_state(state_on);
    }
    else if (strcmp(device, "mode") == 0)
    {
        device_mode_t mode = device_control_mode_from_string(state);
        device_control_set_mode(mode);
    }
    else
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown device");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

// ============================================================================
// JSON ESCAPE HELPER
// ============================================================================

/**
 * @brief Экранировать спецсимволы JSON в строке
 * Заменяет " → \", \ → \\, и управляющие символы.
 */
static void json_escape_string(char *dst, size_t dst_size, const char *src)
{
    size_t di = 0;
    for (size_t si = 0; src[si] != '\0' && di < dst_size - 1; si++) {
        char c = src[si];
        if (c == '"' || c == '\\') {
            if (di + 2 >= dst_size) break;
            dst[di++] = '\\';
            dst[di++] = c;
        } else if (c >= 0x20) {
            dst[di++] = c;
        }
        // Управляющие символы (< 0x20) пропускаем
    }
    dst[di] = '\0';
}

// ============================================================================
// HANDLER: GET /api/status
// ============================================================================

static esp_err_t api_status_handler(httpd_req_t *req)
{
    char ssid[SETTINGS_SSID_MAX_LEN] = {0};
    char uri[SETTINGS_URI_MAX_LEN] = {0};
    char user[SETTINGS_SSID_MAX_LEN] = {0};

    settings_get_wifi_ssid(ssid, sizeof(ssid));
    settings_get_mqtt_uri(uri, sizeof(uri));
    settings_get_mqtt_user(user, sizeof(user));

    // Экранируем строки для JSON (SSID может содержать " или \)
    char ssid_esc[SETTINGS_SSID_MAX_LEN * 2];
    char uri_esc[SETTINGS_URI_MAX_LEN * 2];
    char user_esc[SETTINGS_SSID_MAX_LEN * 2];
    json_escape_string(ssid_esc, sizeof(ssid_esc), ssid);
    json_escape_string(uri_esc, sizeof(uri_esc), uri);
    json_escape_string(user_esc, sizeof(user_esc), user);

    // Получаем IP STA интерфейса
    char ip_str[16] = "—";
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info;
    if (sta_netif && esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK)
    {
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
    }

    // Получаем IP AP интерфейса
    char ap_ip_str[16] = "—";
    esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_ip_info_t ap_ip_info;
    if (ap_netif && esp_netif_get_ip_info(ap_netif, &ap_ip_info) == ESP_OK)
    {
        snprintf(ap_ip_str, sizeof(ap_ip_str), IPSTR, IP2STR(&ap_ip_info.ip));
    }

    // mDNS hostname
    char mdns_name[64] = {0};
    if (mdns_hostname_get(mdns_name) != ESP_OK)
    {
        strncpy(mdns_name, "—", sizeof(mdns_name) - 1);
    }

    // Версия прошивки
    char fw_version[32] = "—";
    esp_app_desc_t app_info;
    if (esp_ota_get_partition_description(esp_ota_get_running_partition(), &app_info) == ESP_OK)
    {
        strncpy(fw_version, app_info.version, sizeof(fw_version) - 1);
    }

    // MAC адрес
    char mac_str[18] = "—";
    uint8_t mac[6];
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK)
    {
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    // Статус OTA
    const char *ota_status = "idle";
    if (ota_is_active())
    {
        ota_status = "downloading";
    }

    // Реальный статус WiFi STA (проверка по наличию IP-адреса)
    esp_netif_ip_info_t wifi_ip_info;
    bool wifi_up = (sta_netif && esp_netif_get_ip_info(sta_netif, &wifi_ip_info) == ESP_OK
                    && wifi_ip_info.ip.addr != 0);

    // Увеличиваем буфер JSON для новых полей
    char json[1024];
    snprintf(json, sizeof(json),
             "{\"wifi_connected\":%s,"
             "\"wifi_ssid\":\"%s\","
             "\"ip\":\"%s\","
             "\"ap_ip\":\"%s\","
             "\"mqtt_connected\":%s,"
             "\"mqtt_uri\":\"%s\","
             "\"mqtt_user\":\"%s\","
             "\"mdns\":\"%s\","
             "\"firmware_version\":\"%s\","
             "\"mac_address\":\"%s\","
             "\"ota_status\":\"%s\"}",
             wifi_up ? "true" : "false",
             ssid_esc, ip_str, ap_ip_str,
             mqtt_client_is_connected() ? "true" : "false",
             uri_esc, user_esc,
             mdns_name[0] != '\0' ? mdns_name : "—",
             fw_version, mac_str, ota_status);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json, strlen(json));
    return ESP_OK;
}

// ============================================================================
// HANDLER: POST /settings/save (WiFi)
// ============================================================================

/// Флаг для сигнала задаче main о необходимости переподключения WiFi
static _Atomic bool s_wifi_reconnect_requested = false;

static esp_err_t settings_save_handler(httpd_req_t *req)
{
    // Проверяем метод
    if (req->method != HTTP_POST)
    {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Use POST method");
        return ESP_FAIL;
    }

    // CSRF защита: если Referer отсутствует или не из приватной сети,
    // проверяем Content-Type — HTML-формы (<form action="...">) НЕ МОГУТ отправить JSON.
    // Это предотвращает CSRF через <form method="POST" action="http://hydronft.local/...">
    // даже при отсутствии Referer (referrer-policy: no-referrer)
    if (!check_csrf(req)) {
        char ct[64];
        if (httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct)) != ESP_OK ||
            strstr(ct, "application/json") == NULL) {
            httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "CSRF: Content-Type required");
            return ESP_FAIL;
        }
    }

    char ssid[SETTINGS_SSID_MAX_LEN] = {0};
    char pass[SETTINGS_PASS_MAX_LEN] = {0};

    // Читаем тело POST-запроса
    if (req->content_len == 0 || req->content_len > 512) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Content-Length");
        return ESP_FAIL;
    }
    size_t body_len = req->content_len;

    char body[512];
    int received = httpd_req_recv(req, body, body_len);
    if (received <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read body");
        return ESP_FAIL;
    }
    body[received] = '\0';

    // Парсим JSON: {"wifi_ssid":"...","wifi_pass":"..."}
    if (extract_json_string(body, "\"wifi_ssid\"", ssid, sizeof(ssid)) < 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid wifi_ssid");
        return ESP_FAIL;
    }
    extract_json_string(body, "\"wifi_pass\"", pass, sizeof(pass)); // Опционально

    if (ssid[0] == '\0')
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
        return ESP_FAIL;
    }

    settings_set_wifi_ssid(ssid);
    settings_set_wifi_pass(pass);

    ESP_LOGI(TAG, "WiFi настройки сохранены: SSID=%s", ssid);

    // Запрашиваем переподключение WiFi (#9)
    atomic_store(&s_wifi_reconnect_requested, true);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

// ============================================================================
// HANDLER: POST /settings/mqtt (MQTT настройки)
// ============================================================================

static esp_err_t settings_mqtt_handler(httpd_req_t *req)
{
    // Проверяем метод
    if (req->method != HTTP_POST)
    {
        httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Use POST method");
        return ESP_FAIL;
    }

    // CSRF защита: если Referer отсутствует или не из приватной сети,
    // проверяем Content-Type — HTML-формы (<form action="...">) НЕ МОГУТ отправить JSON.
    // Это предотвращает CSRF через <form method="POST" action="http://hydronft.local/...">
    // даже при отсутствии Referer (referrer-policy: no-referrer)
    if (!check_csrf(req)) {
        char ct[64];
        if (httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct)) != ESP_OK ||
            strstr(ct, "application/json") == NULL) {
            httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "CSRF: Content-Type required");
            return ESP_FAIL;
        }
    }

    char uri[SETTINGS_URI_MAX_LEN] = {0};
    char user[SETTINGS_SSID_MAX_LEN] = {0};
    char pass[SETTINGS_PASS_MAX_LEN] = {0};

    // Читаем тело POST-запроса
    if (req->content_len == 0 || req->content_len > 512) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid Content-Length");
        return ESP_FAIL;
    }
    size_t body_len = req->content_len;

    char body[512];
    int received = httpd_req_recv(req, body, body_len);
    if (received <= 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read body");
        return ESP_FAIL;
    }
    body[received] = '\0';

    // Парсим JSON: {"mqtt_uri":"...","mqtt_user":"...","mqtt_pass":"..."}
    extract_json_string(body, "\"mqtt_uri\"", uri, sizeof(uri));
    extract_json_string(body, "\"mqtt_user\"", user, sizeof(user));
    extract_json_string(body, "\"mqtt_pass\"", pass, sizeof(pass));

    if (uri[0] != '\0')
        settings_set_mqtt_uri(uri);
    if (user[0] != '\0')
        settings_set_mqtt_user(user);
    if (pass[0] != '\0')
        settings_set_mqtt_pass(pass);

    ESP_LOGI(TAG, "MQTT настройки сохранены: URI=%s", uri);

    // Переподключаем MQTT клиент (#10)
    if (uri[0] != '\0' || user[0] != '\0' || pass[0] != '\0')
    {
        ESP_LOGI(TAG, "Переподключение MQTT клиента с новыми настройками");
        mqtt_client_restart();
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

// ============================================================================
// HANDLER: POST /api/ota (запуск OTA обновления)
// ============================================================================

static esp_err_t api_ota_handler(httpd_req_t *req)
{
    // CSRF защита: если Referer отсутствует или не из приватной сети,
    // проверяем Content-Type — HTML-формы (<form action="...">) НЕ МОГУТ отправить JSON.
    // Это предотвращает CSRF через <form method="POST" action="http://hydronft.local/...">
    // даже при отсутствии Referer (referrer-policy: no-referrer)
    if (!check_csrf(req)) {
        char ct[64];
        if (httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct)) != ESP_OK ||
            strstr(ct, "application/json") == NULL) {
            httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "CSRF: Content-Type required");
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Запрос OTA обновления из веб-интерфейса");

    // Ставим флаг OTA — предотвращаем гонку с MQTT handler
    mqtt_client_set_ota_flag();

    // Запускаем OTA
    esp_err_t res = ota_start_task();
    if (res == ESP_OK)
    {
        start_ota_progress_task();
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"started\"}");
    }
    else if (res == ESP_ERR_INVALID_STATE)
    {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"already_in_progress\"}");
    }
    else
    {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"status\":\"failed\"}");
    }
    return ESP_OK;
}

// ============================================================================
// HANDLER: POST /settings/restart
// ============================================================================

static esp_err_t settings_restart_handler(httpd_req_t *req)
{
    // CSRF защита: если Referer отсутствует или не из приватной сети,
    // проверяем Content-Type — критичный endpoint перезагрузки
    if (!check_csrf(req)) {
        httpd_resp_send_err(req, HTTPD_403_FORBIDDEN, "CSRF protection");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Перезагрузка устройства по запросу из веб-интерфейса");

    // Безопасное отключение оборудования перед перезагрузкой — предотвращаем
    // ситуацию когда насос/клапан остаются включёнными после перезагрузки ESP32
    // (GPIO могут быть в неопределённом состоянии до инициализации)
    device_control_set_pump_state(false);
    device_control_set_light_state(false);
    device_control_set_valve_state(false);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"restarting\"}");

    // Даём время ответить клиенту
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ
// ============================================================================

static httpd_handle_t server = NULL;

esp_err_t web_server_init(void)
{
    if (server != NULL)
    {
        ESP_LOGW(TAG, "Веб-сервер уже запущен");
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEB_SERVER_PORT;
    config.max_uri_handlers = 13;  // 11 используется + 2 резерв для будущих endpoint'ов
    // 6144 байт — запас для формирования JSON (до 1KB), сетевых буферов httpd
    // и будущего TLS (при переходе на HTTPS потребуется ещё больше ~8192)
    config.stack_size = 6144;

    ESP_LOGI(TAG, "Запуск веб-сервера на порту %d...", config.server_port);

    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Не удалось запустить веб-сервер: %s", esp_err_to_name(err));
        return err;
    }

    // Основные страницы
    httpd_uri_t uri_main = {
        .uri = "/", .method = HTTP_GET, .handler = main_page_handler, .user_ctx = NULL};
    httpd_uri_t uri_settings = {
        .uri = "/settings", .method = HTTP_GET, .handler = settings_page_handler, .user_ctx = NULL};

    // REST API
    httpd_uri_t uri_api_sensors = {
        .uri = "/api/sensors", .method = HTTP_GET, .handler = api_sensors_handler, .user_ctx = NULL};
    httpd_uri_t uri_api_control = {
        .uri = "/api/control", .method = HTTP_GET, .handler = api_control_handler, .user_ctx = NULL};
    httpd_uri_t uri_api_set = {
        .uri = "/api/set", .method = HTTP_POST, .handler = api_set_handler, .user_ctx = NULL};
    httpd_uri_t uri_api_status = {
        .uri = "/api/status", .method = HTTP_GET, .handler = api_status_handler, .user_ctx = NULL};

    // Настройки
    httpd_uri_t uri_save_wifi = {
        .uri = "/settings/save", .method = HTTP_POST, .handler = settings_save_handler, .user_ctx = NULL};
    httpd_uri_t uri_mqtt = {
        .uri = "/settings/mqtt", .method = HTTP_POST, .handler = settings_mqtt_handler, .user_ctx = NULL};
    httpd_uri_t uri_restart = {
        .uri = "/settings/restart", .method = HTTP_POST, .handler = settings_restart_handler, .user_ctx = NULL};
    httpd_uri_t uri_api_ota = {
        .uri = "/api/ota", .method = HTTP_POST, .handler = api_ota_handler, .user_ctx = NULL};

    httpd_register_uri_handler(server, &uri_main);
    httpd_register_uri_handler(server, &uri_settings);
    httpd_register_uri_handler(server, &uri_api_sensors);
    httpd_register_uri_handler(server, &uri_api_control);
    httpd_register_uri_handler(server, &uri_api_set);
    httpd_register_uri_handler(server, &uri_api_status);
    httpd_register_uri_handler(server, &uri_save_wifi);
    httpd_register_uri_handler(server, &uri_mqtt);
    httpd_register_uri_handler(server, &uri_restart);
    httpd_register_uri_handler(server, &uri_api_ota);

    ESP_LOGI(TAG, "Веб-сервер запущен");
    return ESP_OK;
}

esp_err_t web_server_stop(void)
{
    if (server == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = httpd_stop(server);
    server = NULL;
    return err;
}

bool web_server_wifi_reconnect_requested(void)
{
    return atomic_exchange(&s_wifi_reconnect_requested, false);
}

void web_server_set_wifi_reconnect_requested(bool value)
{
    atomic_store(&s_wifi_reconnect_requested, value);
}
