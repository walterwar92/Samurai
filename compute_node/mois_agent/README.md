# MOIS HTTP-агент Samurai

Polling-based мост между **сайтом** (Supabase Edge Function `robot-gateway`)
и **роботом**. Запускается **только на ноутбуке** (Compute) и работает как
тонкая прослойка: получает команды с сайта → дёргает локальный dashboard
FastAPI `:5000` → возвращает результат + телеметрию.

```
┌─────────────┐   HTTPS poll    ┌─────────────────┐  HTTP REST  ┌─────────────┐
│   Сайт      │ ←─────────────→ │  MOIS-агент     │ ──────────→ │ Dashboard   │
│ (Supabase   │   commands /    │ (на ноутбуке)   │  /api/v1/*  │ FastAPI     │
│  Edge Fn)   │   results /     │                 │             │ :5000       │
│             │   telemetry     │  python -m      │             │             │
└─────────────┘                 │  compute_node   │             └──────┬──────┘
                                │  .mois_agent    │                    │ MQTT
                                └─────────────────┘                    ↓
                                                              ┌─────────────┐
                                                              │ Pi (robot1) │
                                                              └─────────────┘
```

## Зачем

- **Сайт не лезет напрямую в локальную сеть.** Не нужен публичный IP, проброс
  портов или VPN. Агент инициирует связь сам (HTTPS-poll наружу).
- **Все endpoints на сайте.** Cайт держит реестр команд, очередь, журнал
  результатов и телеметрии. Агент их только исполняет.
- **Единая точка управления роботом.** Все 54 команды агента уходят в
  локальный dashboard `:5000`, который уже знает MQTT (Pi) и Samcan bridge
  (Arduino). Логика управления не дублируется.

## Быстрый старт

```bash
# 1. Создать конфиг
cp compute_node/mois_agent/config.json.example \
   compute_node/mois_agent/config.json
# отредактируй api_url и api_token

# 2. Убедись что dashboard запущен (на том же ноуте)
./samurai.sh compute

# 3. В отдельной консоли — агент
./samurai.sh agent

# или одной строкой через ENV
MOIS_API_URL="https://xxx.supabase.co/functions/v1/robot-gateway" \
MOIS_API_TOKEN="rt_xxxxxxxx" \
./samurai.sh agent

# Посмотреть какие команды поддерживаются
./samurai.sh agent --list-commands
```

Production-режим:

```bash
sudo ./scripts/systemd/install.sh agent
sudo systemctl enable --now samurai-agent
journalctl -u samurai-agent -f
```

## Конфиг

Источники в порядке убывания приоритета:
1. CLI-флаги (`--api-url`, `--api-token`, …)
2. ENV-переменные (`MOIS_API_URL`, `MOIS_API_TOKEN`, `MOIS_DASHBOARD_URL`,
   `MOIS_SAMCAN_URL`, `MOIS_ROBOT_ID`, `MOIS_POLL_INTERVAL`,
   `MOIS_TELEMETRY_INTERVAL`, `MOIS_AGENT_NAME`)
3. `compute_node/mois_agent/config.json`
4. Дефолты

`config.json` структура:

```json
{
  "api_url": "https://YOUR-PROJECT.supabase.co/functions/v1/robot-gateway",
  "api_token": "rt_xxxxxxxx",
  "dashboard_url": "http://127.0.0.1:5000",
  "samcan_url": null,
  "robot_id": "robot1",
  "poll_interval": 10,
  "telemetry_interval": 30,
  "agent_name": "samurai-mois"
}
```

`samcan_url` оставь `null` — агент тогда ходит к Samcan через прокси
dashboard (`/api/v1/samcan/*`). Заполняй только если хочешь обойти dashboard
и стучать прямо к Samcan bridge `:5005`.

## Edge Function API (что агент ожидает от сайта)

Все запросы идут на `api_url` с разными `?action=` и Bearer-токеном в
заголовке. Реализация на стороне сайта (Supabase Edge Function на Deno)
должна поддержать четыре действия:

### POST `?action=capabilities`

Регистрация команд при старте агента. Тело:

```json
{
  "commands": [
    { "name": "drive", "description": "...", "params_schema": { "type": "object", ... } },
    { "name": "stop",  "description": "..." }
  ]
}
```

Сайт сохраняет это (например, в таблицу `agent_capabilities`) и UI рисует
кнопки/формы из `params_schema` (JSON Schema).

### GET `?action=poll`

Ответ — следующая порция команд для исполнения и (опционально) новый
интервал:

```json
{
  "poll_interval": 10,
  "commands": [
    { "id": "uuid-1", "command": "drive", "params": { "linear": 0.2, "angular": 0 } },
    { "id": "uuid-2", "command": "claw",  "params": { "state": "open" } }
  ]
}
```

`id` потом возвращается в `cmd-result`. Если очередь пуста —
`{"commands": []}`. `poll_interval` (если есть) клампится агентом в
[1, 300] секунд.

### POST `?action=cmd-result`

Результат выполнения каждой команды:

```json
{
  "command_id": "uuid-1",
  "exit_code": 0,
  "stdout": "...",
  "stderr": "",
  "exec_time_ms": 142
}
```

`exit_code=0` — успех. Если dashboard вернул ошибку (например, MQTT
отвалился), `exit_code=1` и `stderr` содержит читаемое описание.

### POST `?action=telemetry`

Раз в `telemetry_interval` секунд агент дёргает `/api/v1/status` локального
dashboard и шлёт компактный snapshot:

```json
{
  "custom": {
    "online": true,
    "ts": 1733930000,
    "robot_id": "robot1",
    "battery_pct": 87,
    "temperature_c": 52.4,
    "pose": { "x": 1.23, "y": 0.45, "yaw": 1.57 },
    "velocity": { "linear": 0.0, "angular": 0.0 },
    "fsm_state": "idle",
    "speed_profile": "normal",
    "mqtt_connected": true,
    "ultrasonic_m": 1.2,
    "imu_yaw_deg": 89.5,
    "detection_enabled": true,
    "detections_count": 0
  }
}
```

Если dashboard недоступен — отправляется `{"online": false, "error": "..."}`.

## Реестр команд

Полный список — `./samurai.sh agent --list-commands`. На момент написания
**54 команды**, сгруппированы:

| Категория  | Команды |
|------------|---------|
| basic      | `ping`, `info`, `dashboard_health` |
| motion     | `drive`, `stop`, `emergency_stop`, `reset_position`, `set_speed_profile`, `get_speed_profile` |
| state      | `get_status`, `get_pose`, `get_velocity`, `get_battery`, `get_temperature`, `get_sensors`, `get_ultrasonic`, `get_imu`, `get_fsm`, `get_detections`, `get_actuators`, `get_mqtt`, `get_log` |
| actuators  | `claw`, `head`, `arm`, `arm_presets`, `head_presets`, `led` |
| voice/fsm  | `voice`, `fsm_transition`, `tts_speak`, `tts_toggle` |
| control    | `patrol`, `patrol_waypoints`, `follow_me`, `path_recorder`, `path_recorder_status`, `path_recorder_list`, `detection_toggle`, `obstacle_avoidance_toggle`, `collision_guard_toggle` |
| maps       | `map_list`, `map_info`, `map_save`, `map_load`, `zones_list`, `zones_clear` |
| samcan     | `samcan_cmd`, `samcan_scenario`, `samcan_state`, `samcan_log`, `samcan_scenarios`, `samcan_diag`, `samcan_preset_apply` |

Каждая команда документирована в файле `commands/<категория>.py` и
экспортирует `params_schema` (JSON Schema) — сайт может строить UI
автоматически.

## Как добавить новую команду

1. Открой подходящий `commands/<категория>.py`.
2. Напиши handler `(params: dict, ctx) -> HandlerResult`. Используй
   `from._utils.from_http(ctx.client.post(...))` для типичного REST-вызова.
3. Зарегистрируй в `COMMANDS = {...}` с описанием и (опционально)
   `params_schema`.
4. Перезапусти агента — на старте он отправит обновлённые capabilities
   на сайт.

Пример (взять из commands/motion.py):

```python
def handle_drive(params, ctx):
    linear = float(params.get("linear", 0.0))
    angular = float(params.get("angular", 0.0))
    return from_http(ctx.client.post(
        "/api/v1/robot/velocity",
        json_body={"linear": linear, "angular": angular},
    ))

COMMANDS = {
    "drive": {
        "description": "Задать скорость робота: linear (м/с), angular (рад/с)",
        "params_schema": {
            "type": "object",
            "properties": {
                "linear":  {"type": "number"},
                "angular": {"type": "number"},
            },
        },
        "handler": handle_drive,
    },
}
```

## Файлы

```
compute_node/mois_agent/
├── README.md               (этот файл)
├── __init__.py
├── __main__.py             python -m compute_node.mois_agent
├── agent.py                основной poll/exec/telemetry цикл
├── client.py               HTTP-обёртка к dashboard FastAPI
├── config.py               загрузка конфига (file/env/cli)
├── config.json.example     шаблон config.json
├── requirements.txt        httpx
├── telemetry.py            collect_telemetry()
└── commands/
    ├── __init__.py         склейка реестра + list_capabilities()
    ├── _utils.py           HandlerResult, ok/err/from_http
    ├── basic.py            ping, info, dashboard_health
    ├── motion.py           drive, stop, ...
    ├── state.py            get_pose, get_battery, ...
    ├── actuators.py        claw, head, arm, led
    ├── voice.py            voice, fsm_transition, tts
    ├── control.py          patrol, follow_me, path_recorder, ...
    ├── maps.py             map_list/save/load, zones
    └── samcan.py           samcan_cmd, samcan_scenario, ...
```

## CLI

```bash
samurai agent [опции]

  --api-url URL          Supabase Edge Function URL
  --api-token TOKEN      Bearer-токен (rt_...)
  --dashboard URL        Dashboard URL (default: http://127.0.0.1:5000)
  --samcan URL           Прямой URL к Samcan bridge (без — через dashboard)
  --robot-id ID          MQTT robot_id (default: robot1)
  --poll SEC             Poll-интервал
  --telemetry SEC        Интервал телеметрии
  --config PATH          Альт. путь к config.json
  --log-level LVL        DEBUG|INFO|WARNING|ERROR
  --list-commands        Распечатать команды и выйти
```

## Безопасность

- `api_token` хранится в `config.json` или ENV. **Не коммить файл с настоящим
  токеном.** В `.gitignore` уже добавлено `compute_node/mois_agent/config.json`.
- Агент шлёт только Bearer-токен и не предъявляет клиентских сертификатов —
  ротируй токены на стороне сайта.
- На стороне сайта валидируй `params` каждой команды по `params_schema`,
  присланному в `capabilities`. Агент не доверяет тому, что params валидны,
  и сам проверяет ключевые поля, но первая линия защиты — сайт.

## Troubleshooting

- **«Dashboard :5000 не отвечает»** — запусти `./samurai.sh compute` (или
  как минимум `python -m compute_node.dashboard`).
- **«нужны api_url и api_token»** — нет config.json и нет ENV. См. раздел
  «Конфиг».
- **`HTTP 401`** — токен не подходит. Сгенерируй новый на сайте.
- **`exit_code=1, stderr=…HTTP 503`** — dashboard вернул ошибку (часто
  значит что Pi/MQTT отвалился). Проверь `./samurai.sh status`.
