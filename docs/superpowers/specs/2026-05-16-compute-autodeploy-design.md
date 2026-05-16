# Compute autodeploy на Pi — дизайн

**Дата:** 2026-05-16
**Предлагаемая ветка реализации:** `feat/compute-autodeploy`
**Авторы:** @walterwar92 + brainstorming session

## Проблема

Сейчас обновление кода на роботе (Raspberry Pi 4) требует:
1. Подключить Pi к интернету (Wi-Fi с роутером, имеющим выход в сеть).
2. SSH на Pi → `git pull` → ручной перезапуск `samurai-robot.service`.

Это неудобно в полевых условиях: Pi часто работает в hotspot-режиме от ноутбука (без интернета), и каждое обновление превращается в шаги «подключи к домашнему Wi-Fi → подожди → отключи → подключи обратно к ноуту».

## Цель

Юзер на ноутбуке выполняет **одну** команду:
```
./samurai.sh compute --pi 192.168.x.x
```
и получает:
- Свежий код на Pi (rsync рабочего дерева ноута).
- Перезапущенный `samurai-robot.service`.
- Поднятый compute-стек на ноуте (как сейчас).

Pi не нуждается в интернете — только в LAN-видимости с ноутом.

## Решения принятые на этапе брейншторма

| Развилка | Выбор | Почему |
|---|---|---|
| Триггер деплоя | `compute` автодеплоит | Одна команда — всё работает |
| Состояние кода | Рабочее дерево как есть (с uncommitted) | Быстрые итерации, никаких «закоммить чтобы задеплоить» |
| Объём синка | Чёрный список (`.deployignore`) | Гибче белого списка, не сломается при добавлении новой папки |
| Механизм рестарта | `sudo systemctl restart samurai-robot` | Логи в journalctl, авто-рестарт при падении, prod-ready |
| Failure recovery | Только показ логов, ручной fix | Минимум кода, нет двойного места на SD-карте |
| Где живёт логика | Inline в `compute.sh` | Минимум новых файлов; deploy не нужен сам по себе |

## Архитектура

### Поток исполнения `./samurai.sh compute --pi IP`

```
1. Парсинг флагов (новые: --no-deploy, --pi-user, --pi-path, --ssh-key)
2. discover_pi $PI_IP                       ── уже есть, проверяет MQTT :1883
3. deploy_to_pi $PI_IP $PI_USER $PI_PATH    ── НОВОЕ (если не --no-deploy)
   a. pre-flight: ssh ... true              ── проверка доступности
   b. pre-flight: systemctl list-unit-files ── проверка что samurai-robot.service установлен
   c. rsync -az --delete --exclude-from=.deployignore  работа → Pi
   d. ssh ... sudo systemctl restart samurai-robot
   e. sleep 2 → ssh ... systemctl is-active samurai-robot
      ├─ active   → log_ok, продолжаем
      └─ inactive → journalctl tail → die
4. build_frontend                           ── уже есть
5. ROS2 / dashboard стек                    ── уже есть
```

При фейле на шаге 3 ноутбучный стек **не поднимается** — нет смысла поднимать compute, если робот не отвечает.

### Компоненты

#### Изменения в существующих файлах

**`scripts/cmds/compute.sh`** — добавить:
- Новые флаги: `--no-deploy`, `--pi-user USER`, `--pi-path PATH`, `--ssh-key FILE`.
- Функция `deploy_to_pi()` (~80 строк):
  - Pre-flight SSH (`ConnectTimeout=5`).
  - Pre-flight: проверка наличия `samurai-robot.service`.
  - `rsync -az --delete --exclude-from="$SAMURAI_ROOT/.deployignore" \
      -e "ssh ${ssh_opts[*]}" "$SAMURAI_ROOT/" "$user@$ip:$path/"`.
  - `ssh ... 'sudo systemctl restart samurai-robot'`.
  - Verify через `systemctl is-active`, при не-active дамп `journalctl -u samurai-robot -n 30 --no-pager`.
- Точка вызова: после `discover_pi`, до `build_frontend`. Условие: `[[ -n "$PI_IP" && -z "$NO_DEPLOY" ]]`.

**`scripts/systemd/install.sh`** — добавить флаг `--with-remote-deploy`:
- Генерация `/etc/sudoers.d/samurai-robot` с NOPASSWD для четырёх команд. Юзер берётся тот же, под которого ставится systemd-юнит (переменная `$user` в `install.sh`, та же что подставляется в `__SAMURAI_USER__`):
  ```
  <user> ALL=(root) NOPASSWD: /bin/systemctl restart samurai-robot, \
                             /bin/systemctl is-active samurai-robot, \
                             /bin/systemctl status samurai-robot, \
                             /bin/journalctl -u samurai-robot *
  ```
- Запись через временный файл + `visudo -cf <tmpfile>` валидация + `install -m 0440 -o root -g root <tmpfile> /etc/sudoers.d/samurai-robot`. Битый sudoers может залочить sudo — `visudo -cf` обязателен.

#### Новые файлы

**`.deployignore`** в корне репо. Содержимое:
```gitignore
# Что НЕ синхронизировать на Pi через ./samurai.sh compute --pi.
# Используется rsync --exclude-from. Синтаксис gitignore-подобный.

# Git / CI
.git/
.github/

# Зоны которые крутятся ТОЛЬКО на ноуте
compute_node/
ros_ws/
android_app/
matlab/
docs/

# Локальные артефакты
node_modules/
__pycache__/
*.pyc
*.pyo
.venv/
venv/
.mypy_cache/
.pytest_cache/
.ruff_cache/
dist/
build/
*.egg-info/

# IDE / OS мусор
.idea/
.vscode/
.DS_Store
Thumbs.db

# MATLAB
slprj/
*.slxc
*.asv

# Логи и временные
*.log
*.tmp
```

**`scripts/bootstrap_pi.sh`** — первичная настройка Pi одним вызовом. Идемпотентный (можно гонять повторно). Этапы:
Скрипт должен определить целевого юзера: если запущен через sudo — берётся `$SUDO_USER`, иначе `$USER`. Все `pip install --user` выполняются через `sudo -u "$TARGET_USER"` чтобы не засорять корневой site-packages.

Этапы:
1. `apt update && apt install -y mosquitto mosquitto-clients python3-pip python3-picamera2 i2c-tools avahi-daemon rsync`.
2. `sudo -u "$TARGET_USER" pip install --user -r requirements.txt`.
3. `usermod -aG gpio,i2c,spi "$TARGET_USER"`.
4. `raspi-config nonint do_i2c 0 && raspi-config nonint do_camera 0`.
5. `systemctl enable --now mosquitto avahi-daemon ssh`.
6. `./scripts/systemd/install.sh --with-remote-deploy` (использует тот же `$TARGET_USER`).
7. Финальная инструкция в stdout: «Теперь на ноуте: `ssh-copy-id <TARGET_USER>@raspberrypi.local`».

Запуск (один раз на свежем Pi):
```
ssh <user>@raspberrypi.local 'cd Samurai && sudo ./scripts/bootstrap_pi.sh'
```

### Конфигурация

#### Приоритет источников для `--pi-user` / `--pi-path`
1. CLI-аргументы (`--pi-user`, `--pi-path`).
2. Env-переменные (`SAMURAI_PI_USER`, `SAMURAI_PI_PATH`).
3. Файл `~/.samurai/deploy.conf` (опционально, `KEY=value` пары).
4. Хардкод: `pi` / `~/Samurai`.

Файл `deploy.conf` — на случай если юзер не хочет каждый раз тянуть флаги. Не обязателен, не коммитится.

Пример `~/.samurai/deploy.conf`:
```
SAMURAI_PI_USER=pi
SAMURAI_PI_PATH=/home/pi/Samurai
SAMURAI_PI_SSH_KEY=/home/walter/.ssh/samurai_ed25519
```

## Edge cases

| Сценарий | Поведение |
|---|---|
| `--hotspot` + `--pi` | Pi подключается к точке доступа ноута; `discover_pi` находит его на стандартных IP хотспота, rsync/ssh работают без изменений. |
| `--no-deploy` | Деплой пропущен, ноутбучный стек поднимается без Pi-синка. Для отладки compute локально или с симулятором. |
| Robot уже `active` | `systemctl restart` нормально кладёт и поднимает заново, FSM начинается с чистого состояния. |
| Расхождение часов Pi и ноута | rsync `-az --delete` по timestamp может пропустить файлы при больших расхождениях. Опциональный флаг `--force-resync` добавляет `--checksum` к rsync. По умолчанию не включён (медленнее). |
| Конкурентный деплой с двух машин | `flock` на Pi через ssh: `ssh ... 'flock -n /var/lock/samurai-deploy.lock -c "<rsync_remote_cmd>"'` — второй деплой получит ошибку lock contention. Реализация: lock берётся на стороне Pi через ssh-команду перед rsync receiver-сайдом. **Примечание:** для простоты v1 можно опустить (один разработчик в полевых условиях), добавить позже если станет нужно. |
| Прямые правки на Pi внутри `pi_nodes/` | Затрутся `--delete`. By design: источник истины — ноут. |
| `compute --pi` без `--no-deploy` но robot не установлен | Pre-flight упадёт с подсказкой запустить `bootstrap_pi.sh`. |

## Failure mode

| Когда | Что показываем | Exit |
|---|---|---|
| SSH недоступен | `SSH до pi@... недоступен (ключ настроен? Pi включён?)` | non-zero |
| `samurai-robot.service` не установлен | `samurai-robot.service не установлен на Pi. Запусти: ssh ... 'cd Samurai && sudo ./scripts/bootstrap_pi.sh'` | non-zero |
| `rsync` упал | stderr rsync + код возврата | non-zero |
| `systemctl restart` упал (sudo пароль / NOPASSWD не настроен) | подсказка `sudo ./scripts/systemd/install.sh --with-remote-deploy` | non-zero |
| После restart юнит `failed`/`inactive` | `journalctl -u samurai-robot -n 30 --no-pager` дамп в stderr | non-zero |

Во всех случаях `compute.sh` `die`-ит до запуска ноутбучного стека.

## Pre-conditions на Pi (для справки)

Что должно быть однократно настроено на Pi для работы `compute --pi`:

| Требование | Закрывается |
|---|---|
| Python ≥ 3.9 | OS (Debian 13 = Python 3.13) |
| apt-пакеты (mosquitto, picamera2, i2c-tools, avahi, rsync) | `bootstrap_pi.sh` |
| pip-deps из `requirements.txt` | `bootstrap_pi.sh` |
| I2C / camera в `raspi-config` | `bootstrap_pi.sh` |
| User в `gpio,i2c,spi` | `bootstrap_pi.sh` |
| `samurai-robot.service` | `install.sh` (вызывается из `bootstrap_pi.sh`) |
| `/var/lib/samurai/` | `install.sh` |
| `mosquitto.service` enabled+active | `bootstrap_pi.sh` |
| SSH-ключ ноута в `~/.ssh/authorized_keys` Pi | вручную, `ssh-copy-id` |
| `/etc/sudoers.d/samurai-robot` (NOPASSWD) | `install.sh --with-remote-deploy` |

Что **НЕ** нужно на Pi:
- Интернет.
- Git (на Pi `.git/` исключён через `.deployignore`).
- Модели Vosk/YOLO (Vosk — на ноуте/Android, YOLO — на compute).

## Тестирование

### Юнит-тесты (`tests/test_deploy.sh`, bats или bash)
1. **Аргументы `deploy_to_pi`** — PATH override для `ssh` и `rsync` (фейки в `tests/fixtures/bin/` пишут argv в stdout), проверка корректности команд: `ssh -o ConnectTimeout=5 user@host true`, `rsync -az --delete --exclude-from=... user@host:path/` и т.п.
2. **`.deployignore` корректность** — `rsync -n -av --exclude-from=.deployignore .` (dry-run) против временной директории; assertion что список НЕ содержит `compute_node/`, `.git/`, `node_modules/`, `__pycache__/`.
3. **Pre-flight отказы** — фейковый ssh возвращающий exit 1 → проверка что `deploy_to_pi` падает с правильным сообщением.

### Integration smoke (не в CI)
Скрипт-чеклист `tests/integration/test_compute_deploy.sh` для запуска против реального Pi:
- Создать тестовый файл `pi_nodes/__deploy_test__.py` локально.
- `./samurai.sh compute --pi <real-ip> --no-rebuild`.
- `ssh pi 'test -f ~/Samurai/pi_nodes/__deploy_test__.py'` — должен быть.
- `ssh pi 'systemctl is-active samurai-robot'` — должен быть `active`.
- Cleanup.

Запускается вручную разработчиком, не в CI (требует физический Pi).

### CI
Юнит-тесты прогоняются в существующем GitHub Actions workflow вместе с остальными bash-тестами.

## Что НЕ входит в скоуп

- Деплой на Samcan (Arduino Uno) — там прошивка через PlatformIO/Arduino IDE, другой механизм.
- Деплой на ESP32 — то же.
- Развёртывание с нуля (clone репо на чистый Pi) — `bootstrap_pi.sh` предполагает что репо уже клонирован вручную. Можно автоматизировать позже.
- Rollback на предыдущую версию — сознательно отказались на этапе брейншторма (минимум кода).
- Web UI для триггера деплоя — это CLI-инструмент.
- Шифрование/подписи (типа cosign) — деплой по локальной сети между доверенными машинами.

## План реализации (high-level)

1. `.deployignore` в корне репо — отдельный коммит, валидируется dry-run rsync.
2. `scripts/systemd/install.sh` — флаг `--with-remote-deploy` + генерация sudoers.d.
3. `scripts/bootstrap_pi.sh` — новый скрипт первичной настройки.
4. `scripts/cmds/compute.sh` — функция `deploy_to_pi()` + новые флаги + точка вызова.
5. `tests/test_deploy.sh` — юнит-тесты.
6. `docs/` — обновить README/CLAUDE.md с пользовательским сценарием.

Детальный разбивка по шагам — в плане реализации (см. `writing-plans`).
