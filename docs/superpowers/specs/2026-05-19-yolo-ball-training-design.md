# YOLO Ball Training — Fine-tune yolo11n на красный/зелёный/синий мячи

**Дата:** 2026-05-19
**Статус:** Design approved, ждёт implementation plan
**Owner:** walterwarda@gmail.com
**Связано с:** существующая FSM-охота за мячами (`pi_nodes/nodes/fsm_node.py`),
`compute_node/detector.py`, `compute_node/detectors/yolo_backend.py`.

## Контекст

В проекте уже:
- Pi `camera_node.py` стримит H.264 по TCP с `:8554`
- `compute_node/detector.py` запускает YOLO (yolo11n.pt, 80 COCO-классов) +
  HSV-классификатор цвета как post-process
- FSM на Pi (`fsm_node.py`) подписан на `samurai/{id}/ball_detection`,
  переходит SEARCHING → TARGETING → APPROACHING → GRABBING при появлении
  мяча нужного цвета

Текущая проблема: HSV-классификатор цвета **ненадёжен в плохом освещении**
— почти все мячи метятся `colour='black'`, FSM не может найти красный/синий.
Перекалибровка HSV под конкретный свет хрупка (изменилось освещение →
снова не работает).

## Цель

Заменить HSV-post-process на **обученную YOLO-модель с тремя классами**:
`ball_red`, `ball_green`, `ball_blue`. Цвет — часть класса, не отдельный
этап. Это делает определение цвета light-invariant в той мере, в которой
обучающий датасет покрывает разнообразие освещений.

**Не цель:** добавить новые цвета (жёлтый/оранжевый); поменять FSM-логику;
заменить общий YOLO детектор для других объектов.

## Архитектура

### Поток данных (без изменений по транспорту)

```
Pi camera → H.264 TCP → detector.py
                            → YoloBackend(model=samurai_balls.pt)
                            → DetectionPipeline (с class→colour mapper)
                            → MQTTPublisher
                                → samurai/{id}/ball_detection (closest)
                                → samurai/{id}/detections (all)
                                → samurai/{id}/yolo/annotated (jpeg)
                            → FSM на Pi (без изменений)
```

### 3 класса вместо 1

- Class 0: `ball_red`
- Class 1: `ball_green`
- Class 2: `ball_blue`

`yolo_backend.py` после inference выдаёт `class` name. Новый mapper
конвертирует в формат, который FSM ожидает:

```python
{
  "class": "ball",        # унифицировано для FSM
  "colour": "red",        # из имени класса YOLO
  "bbox": [x, y, w, h],
  "conf": 0.87,
  ...
}
```

Это позволяет FSM продолжать работать без изменений — он смотрит только
на `colour` и `class`.

## Компоненты

### 1. `tools/collect_frames.py` (новый, ~150 строк)

CLI-инструмент сбора датасета.

**Использование:**
```
python tools/collect_frames.py --pi 192.168.4.1 --out dataset/raw
```

**Поведение:**
- Подключается к Pi через `H264TCPFrameSource` (тот же что в детекторе)
- Открывает OpenCV окно с live preview
- Hotkeys:
  - `r` — сохранить текущий кадр в `dataset/raw/red/img_NNNNN.jpg`
  - `g` — `dataset/raw/green/...`
  - `b` — `dataset/raw/blue/...`
  - `q` / ESC — выход
- Счётчики на overlay показывают сколько собрано по каждому классу
- Имена файлов с timestamp + auto-increment, дубли не перезаписываются

**Зависимости:** `cv2`, `av` (через H264TCPFrameSource), уже в проекте.

### 2. `dataset/samurai_balls.yaml` (новый)

Ultralytics dataset config:

```yaml
path: ../dataset           # относительно ultralytics cache (или абсолютный)
train: images/train
val: images/val
nc: 3
names:
  0: ball_red
  1: ball_green
  2: ball_blue
```

Структура папок после разметки:
```
dataset/
├── samurai_balls.yaml
├── raw/                    # сырые JPEG из collect_frames
│   ├── red/
│   ├── green/
│   └── blue/
├── images/
│   ├── train/              # 80% сырых
│   └── val/                # 20% сырых
└── labels/
    ├── train/              # .txt в YOLO формате (class cx cy w h, нормализованные)
    └── val/
```

Скрипт деления train/val — часть `train_balls.py` (см. ниже) или отдельный
маленький скрипт `tools/split_dataset.py`. Решение: вшить в `train_balls.py`,
не плодить файлов.

### 3. `tools/train_balls.py` (новый, ~100 строк)

Обёртка над `ultralytics.YOLO.train`.

**Использование:**
```
python tools/train_balls.py --epochs 50 --imgsz 416
```

**Поведение:**
1. Проверяет наличие `dataset/raw/{red,green,blue}/*.jpg` и
   `dataset/labels/raw/*.txt` (один-в-один с изображениями).
2. Делит на train/val 80/20 (с фиксированным seed для повторяемости),
   создаёт симлинки/копии в `dataset/images/{train,val}` и
   `dataset/labels/{train,val}`.
3. Запускает `YOLO('yolo11n.pt').train(data='dataset/samurai_balls.yaml',
   epochs=50, imgsz=416, batch=8, optimizer='AdamW', patience=10,
   project='runs', name='balls_train')`.
4. По завершении копирует `runs/balls_train/weights/best.pt` →
   `samurai_balls.pt` в корне проекта.
5. Печатает метрики (mAP50, precision, recall по классам).

**Зависимости:** `ultralytics` (уже в проекте), на CPU работает.

### 4. Class→colour mapper в `yolo_backend.py` или `DetectionPipeline`

**Где:** правка в `compute_node/detectors/yolo_backend.py` функции `_do_infer`
(или в `DetectionPipeline.process`).

**Логика:**

```python
BALL_CLASS_TO_COLOUR = {
    'ball_red': 'red',
    'ball_green': 'green',
    'ball_blue': 'blue',
}

# После inference, для каждого Detection:
if cls_name in BALL_CLASS_TO_COLOUR:
    detection.colour = BALL_CLASS_TO_COLOUR[cls_name]
    detection.cls = 'ball'      # унифицируем для FSM
```

Если модель `yolo11n.pt` (COCO) — mapping не срабатывает, всё работает
по-старому (`sports ball` остаётся `sports ball`, HSV postprocess
вычисляет colour).

Mapper срабатывает только для классов, начинающихся с `ball_*` →
безопасно для других моделей.

## Объём датасета

Минимум жизнеспособный: **100 фото каждого цвета** = 300 фото total.
Рекомендуемый: **200 каждого** = 600 фото total.

Разнообразие:
- Дистанция: близко (мяч занимает 30-40% кадра), средне, далеко
  (мяч 20-50 px — нижняя граница работы yolo11n)
- Освещение: дневной свет, искусственный, тусклый вечерний (важно!)
- Окружение: разные поверхности (пол, ковёр, стол), на руках, в углу
- Несколько мячей в одном кадре (mixed examples)
- Без мяча в кадре — 10-20 фото фона (negative samples) — снижает false
  positive на похожих объектах

## Этапы работы

| Этап | Кто делает | Время |
|---|---|---|
| 1. Написать `collect_frames.py` | Claude | 10 мин |
| 2. Собрать 600 фото | пользователь | ~30 мин |
| 3. Разметить (Roboflow или LabelImg) | пользователь | 1-2 часа |
| 4. Написать `train_balls.py` + dataset yaml | Claude | 15 мин |
| 5. Тренировка (CPU) | автоматом | ~30-60 мин |
| 6. Class→colour mapper | Claude | 10 мин |
| 7. Тест end-to-end на Pi | оба | 30 мин |

**Total: 2-4 часа.** Тренировка идёт в фоне.

## Риски и митигации

- **R1: Датасет не покрывает реальное освещение** → модель не работает
  в темноте. **М1:** собирать кадры в разное время суток, в разном свете.
- **R2: Дисбаланс классов** (200 красных, 50 синих) → модель путает.
  **М2:** в `collect_frames.py` overlay показывает счётчики по классам,
  пользователь следит за балансом.
- **R3: Маленькие мячи** (<20 px) → yolo11n плохо детектит малое.
  **М3:** в датасете включаем близкие дистанции; если нужно
  больше дальности — переходим на yolo11s (4× больше параметров, всё
  ещё на CPU терпимо).
- **R4: Overfitting** (особенно на CPU где batch маленький).
  **М4:** patience=10 (early stopping), augmentation default ultralytics
  (mosaic, hsv jitter — что особенно полезно для color robustness).
- **R5: Разметка скучная, пользователь сдастся** на 200-м фото.
  **М5:** Roboflow auto-annotation с SAM делает 80% работы автоматом —
  пользователь только подтверждает класс.

## Что НЕ входит в этот спек

- Дополнительные классы (жёлтый/оранжевый/другие объекты) — добавим
  отдельным спеком если потребуется.
- Замена HSV-калибратора (он остаётся для других моделей).
- Изменения FSM-логики (она нативно работает по colour=red/green/blue).
- GPU-тренировка (если у пользователя появится NVIDIA — добавим
  опционально через CUDA_VISIBLE_DEVICES, не блокер).
- Active learning / continuous learning (out of scope).

## Тестирование

- **Smoke test после тренировки:** `yolo predict model=samurai_balls.pt
  source=dataset/images/val` — визуально проверить bbox'ы на validation
  set.
- **End-to-end:** `./samurai.sh detector --pi 192.168.4.1 --model samurai_balls.pt`
  → положить мяч в кадр → дашборд показывает детекцию с правильным цветом.
- **FSM test:** голосовая команда «найди красный мяч» → робот должен
  ехать к красному.
