#!/usr/bin/env python3
"""
OTA HTTPS сервер для ESP32
Запускает HTTPS сервер для OTA обновления прошивки

==============================================================================
ЗАПУСК СЕРВЕРА:
==============================================================================

# 1. Перейдите в директорию с бинарником прошивки:
cd /home/iroh/Документы/Project/hydropNFT/build

# 2. Запустите сервер:
python3 /home/iroh/Документы/Project/hydropNFT/ota_server.py

# Или из любой директории:
python3 ota_server.py

==============================================================================
ПРОВЕРКА РАБОТЫ:
==============================================================================

# Откройте в браузере (должно начаться скачивание):
https://192.168.0.103:8070/hydropNFT.bin

# Или через curl:
curl -k https://192.168.0.103:8070/hydropNFT.bin -o test.bin

# Проверка magic byte (должно быть e904 03d8):
curl -k https://192.168.0.103:8070/hydropNFT.bin | head -c 4 | xxd

==============================================================================
НАСТРОЙКА ESP-IDF:
==============================================================================

# Откройте menuconfig:
idf.py menuconfig

# Перейдите в:
Example Configuration → Firmware Upgrade URL

# Укажите URL:
https://192.168.0.103:8070/hydropNFT.bin

# Сохраните и пересоберите:
idf.py build
idf.py flash

==============================================================================
СОЗДАНИЕ СЕРТИФИКАТОВ (если нет):
==============================================================================

cd /home/iroh/Документы/Project/hydropNFT/server_certs

# Создать приватный ключ:
openssl genrsa -out ca_key.pem 2048

# Создать самоподписанный сертификат:
openssl req -new -x509 -key ca_key.pem -out ca_cert.pem -days 365 \
    -subj "/C=RU/ST=Moscow/L=Moscow/O=HydroNFT/CN=192.168.0.103"

==============================================================================
ОТКЛЮЧЕНИЕ БРАНДМАУЭРА (если нужно):
==============================================================================

# Разрешить порт 8070:
sudo ufw allow 8070/tcp

# Или временно отключить брандмауэр:
sudo ufw disable

==============================================================================
"""

import http.server
import ssl
import os
import sys

# Адрес и порт сервера
server_address = ('0.0.0.0', 8070)

# Создаём HTTP сервер
httpd = http.server.HTTPServer(server_address, http.server.SimpleHTTPRequestHandler)

# Пути к сертификатам (используем ca_cert и ca_key)
certfile = '/home/iroh/Документы/Project/hydropNFT/server_certs/ca_cert.pem'
keyfile = '/home/iroh/Документы/Project/hydropNFT/server_certs/ca_key.pem'

# Проверяем наличие сертификатов
if not os.path.exists(certfile):
    print(f"❌ Ошибка: Сертификат не найден: {certfile}")
    print()
    print("Создайте сертификаты командой:")
    print()
    print("  cd /home/iroh/Документы/Project/hydropNFT/server_certs")
    print("  openssl genrsa -out ca_key.pem 2048")
    print("  openssl req -new -x509 -key ca_key.pem -out ca_cert.pem -days 365")
    print()
    sys.exit(1)

if not os.path.exists(keyfile):
    print(f"❌ Ошибка: Приватный ключ не найден: {keyfile}")
    sys.exit(1)

print("✅ Сертификаты найдены:")
print(f"   Сертификат: {certfile}")
print(f"   Ключ: {keyfile}")
print()

# Создаём SSL контекст (новый способ для Python 3.12+)
context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
context.load_cert_chain(certfile=certfile, keyfile=keyfile)

# Применяем SSL к сокету
httpd.socket = context.wrap_socket(httpd.socket, server_side=True)

# Получаем текущую директорию безопасным способом
try:
    current_dir = os.getcwd()
except FileNotFoundError:
    current_dir = os.path.dirname(os.path.abspath(__file__))

print("=" * 60)
print("OTA HTTPS сервер запущен!")
print("=" * 60)
print(f"Порт: {8070}")
print(f"URL для OTA: https://192.168.0.103:8070/hydropNFT.bin")
print(f"Директория: {current_dir}")
print(f"Файл прошивки: {current_dir}/hydropNFT.bin")
print("=" * 60)

# Проверяем наличие файла прошивки
bin_file = os.path.join(current_dir, 'hydropNFT.bin')
if os.path.exists(bin_file):
    print(f"✅ Файл прошивки найден: {bin_file}")
    print(f"   Размер: {os.path.getsize(bin_file)} байт")
else:
    print(f"❌ Файл прошивки НЕ найден: {bin_file}")
    print("   Убедитесь, что вы запустили сервер из папки build/")
    print("   и что проект собран (idf.py build)")

print("=" * 60)
print("Нажмите Ctrl+C для остановки")
print("=" * 60)

try:
    httpd.serve_forever()
except KeyboardInterrupt:
    print("\nОстановка сервера...")
    httpd.server_close()
    print("Сервер остановлен")
