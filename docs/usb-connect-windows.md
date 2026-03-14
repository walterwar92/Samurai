# Подключение к Raspberry Pi по USB с Windows

Два способа: **USB OTG** (только кабель) и **UART Serial** (нужен адаптер).

---

## Способ 1 — USB OTG через USB-C (рекомендуется)

Pi 4 можно подключить к ноутбуку обычным USB-C кабелем — Pi появится как сетевой адаптер.

### Шаг 1: Настройка на Pi (один раз)

Подключить SD-карту к ноутбуку или зайти на Pi пока он ещё доступен.

Открыть `/boot/firmware/config.txt`, добавить в конец:
```
dtoverlay=dwc2
```

Открыть `/boot/firmware/cmdline.txt`, найти строку и добавить после `rootwait`:
```
modules-load=dwc2,g_ether
```

> Важно: `cmdline.txt` — одна строка, не переносить.

Пример до:
```
console=serial0,115200 console=tty1 root=PARTUUID=... rootfstype=ext4 rootwait quiet splash
```

Пример после:
```
console=serial0,115200 console=tty1 root=PARTUUID=... rootfstype=ext4 rootwait modules-load=dwc2,g_ether quiet splash
```

### Шаг 2: Подключение

1. Подключить USB-C кабель: **Pi USB-C порт → USB-A/C порт ноутбука**
   > Использовать кабель передачи данных, не зарядный
2. Подождать ~30 секунд пока Pi загрузится
3. Открыть **Диспетчер устройств** (`Win+X → Диспетчер устройств`)
4. Убедиться что появился раздел **"Сетевые адаптеры"** → `USB Ethernet/RNDIS Gadget`

### Шаг 3: SSH

```cmd
ssh gosha@raspberrypi.local
```

Если `raspberrypi.local` не резолвится:
```cmd
# Найти IP адреса новых интерфейсов
arp -a

# Попробовать типичный IP OTG-адаптера
ssh gosha@169.254.x.x
```

### Возможные проблемы

| Проблема | Решение |
|----------|---------|
| Устройство не появляется в диспетчере | Проверить кабель — нужен data-кабель, не только зарядный |
| Драйвер не установился | Скачать [RNDIS драйвер](https://modclouddownloadprod.blob.core.windows.net/shared/mod-rndis-driver-windows.zip) |
| `raspberrypi.local` не работает | Установить [Bonjour Print Services](https://support.apple.com/kb/DL999) от Apple |

---

## Способ 2 — UART Serial (USB-to-TTL адаптер)

Работает всегда, даже если ОС Pi не загрузилась полностью. Нужен адаптер **CP2102**, **CH340** или **FTDI** (~200–500₽).

### Шаг 1: Настройка Pi (один раз)

В `/boot/firmware/config.txt` добавить:
```
enable_uart=1
```

Перезагрузить Pi.

### Шаг 2: Подключение проводов

```
Адаптер  →  Raspberry Pi 4 (GPIO)
GND      →  Pin 6  (GND)
TX       →  Pin 10 (GPIO15 / RXD)
RX       →  Pin 8  (GPIO14 / TXD)
```

> **Не подключать VCC** — Pi питается отдельно.

Распиновка Pi 4 (вид сверху, USB-порты слева):
```
 [USB] [USB]
  ...
  Pin 1  (3.3V)   Pin 2  (5V)
  Pin 3  (SDA)    Pin 4  (5V)
  Pin 5  (SCL)    Pin 6  (GND)  ← GND
  Pin 7  (GPIO4)  Pin 8  (TXD)  ← к RX адаптера
  Pin 9  (GND)    Pin 10 (RXD)  ← к TX адаптера
```

### Шаг 3: Драйвер адаптера на Windows

- **CH340**: [Драйвер CH341SER](https://www.wch-ic.com/downloads/CH341SER_EXE.html)
- **CP2102**: [Драйвер Silicon Labs](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)
- **FTDI**: [Драйвер FTDI](https://ftdichip.com/drivers/vcp-drivers/)

После установки — порт появится в Диспетчере устройств как `COM3` (номер может отличаться).

### Шаг 4: Подключение через PuTTY

1. Скачать [PuTTY](https://www.putty.org/)
2. Открыть PuTTY:
   - Connection type: **Serial**
   - Serial line: `COM3` (смотреть в Диспетчере устройств)
   - Speed: `115200`
3. Нажать **Open**
4. Нажать **Enter** — появится приглашение логина Pi

```
raspberrypi login: gosha
Password: ****
```

### Через Windows Terminal / PowerShell

```powershell
# Установить PuTTY и запустить из терминала:
putty -serial COM3 -sercfg 115200,8,n,1,N
```

---

## Что делать после подключения (восстановление Tailscale)

```bash
# Проверить статус Tailscale
sudo systemctl status tailscaled

# Перезапустить если упал
sudo systemctl restart tailscaled

# Проверить что подключён
tailscale status

# Добавить автоперезапуск на будущее
sudo systemctl edit tailscaled
```

Добавить в редакторе:
```ini
[Service]
Restart=always
RestartSec=10
```

```bash
sudo systemctl daemon-reload
```

---

## Добавить cron-watchdog (рекомендуется)

```bash
crontab -e
```

Добавить строку:
```
*/5 * * * * systemctl is-active --quiet tailscaled || systemctl restart tailscaled
```

Теперь Tailscale будет автоматически перезапускаться каждые 5 минут при падении.
