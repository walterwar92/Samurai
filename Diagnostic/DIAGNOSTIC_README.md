# Диагностика робота Гоша

## Файлы

- `diagnostic.py` — скрипт диагностики
- `diagnostic_requirements.txt` — зависимости Python

## Установка зависимостей

```bash
# Системные пакеты (если ещё не установлены)
sudo apt update
sudo apt install -y python3-pip python3-smbus i2c-tools python3-pyaudio portaudio19-dev libopencv-dev

# Python-пакеты
sudo pip3 install -r diagnostic_requirements.txt
```

## Запуск

```bash
sudo python3 diagnostic.py
```

`sudo` обязателен — WS2812 LED работает только от root.

## Результаты

Всё сохраняется в `~/samurai_diagnostics/`:

```
~/samurai_diagnostics/
├── diagnostic_20260216_193045.log   # полный текстовый лог
├── diagnostic_20260216_193045.json  # JSON-отчёт со всеми данными
├── camera_snapshot_20260216_193045.jpg  # фото с камеры
└── mic_test_20260216_193045.wav     # 2с запись микрофона
```

## Что делает робот при запуске

1. Едет **вперёд 1с** → **назад 1с**
2. Поворачивает **влево 1с** → **вправо 1с**
3. Каждый мотор отдельно: вперёд 0.5с, назад 0.5с
4. Сервоприводы: init → тестовые углы → init
5. WS2812: красный → зелёный → синий → бегущий огонь → выкл

