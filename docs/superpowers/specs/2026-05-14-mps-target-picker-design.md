# МПС — пред-прогонный пикер цели (3D-окно выбора точки)

> При запуске МПС-сценария на роботе (`source=robot`) вместо немедленного
> прогона всплывает модальное 3D-окно: модель робота в центре, вокруг —
> окружность радиуса N (= заданная дистанция). Пользователь кликает точку
> на окружности → выбирает **направление** φ → жмёт «Старт». Робот
> разворачивается на месте к выбранной точке, затем едет прямо N метров.
>
> **Базируется на:** ветка `feat/mps-target-picker` (от
> `fix/mps-heading-scenario-relative`, где θ сделан scenario-relative).
> Переиспользует существующую 3D-инфраструктуру МПС (`RobotModel`,
> `Mps3DScene`, `Mps3DOverlay`-паттерн) и каноническую модель
> `[s, v, θ, ω, e_int]` без переписывания.
>
> **Скоуп:** фронтенд (`compute_node/frontend/src/`) + контракт
> (`MpsScenarioRequest`) + Pi-нода (`pi_nodes/nodes/mps_node.py`) +
> `config.yaml` + `docs/mps/api.md`. Симулятор (`mps_runner.py`) НЕ
> трогается.

## 1. Контекст и проблема

Сценарий МПС — «проехать D метров вперёд». «Вперёд» определяется через
курс θ в каноническом векторе состояния `[s, v, θ, ω, e_int]` с
`x_ref[θ] = 0`. Из-за того что θ приходил из одометрии в абсолютных
координатах, робот доворачивал к фиксированному курсу одометрии вместо
«вперёд куда смотрит» — это починено в `fix/mps-heading-scenario-relative`
(θ снимается относительно старта, `θ_start`).

Эта фича меняет **подход**: вместо «всегда вперёд» (φ=0) пользователь
получает явный контроль над направлением — кликает точку на окружности
радиуса N, и робот сам разворачивается туда. φ=0 остаётся валидным
выбором («прямо») и даёт ровно сегодняшнее поведение.

## 2. Решения брейншторма (зафиксировано 2026-05-14)

| # | Вопрос | Решение |
|---|---|---|
| 1 | Что робот физически делает, чтобы достичь точки? | **Развернуться на месте к точке, затем ехать прямо.** Две фазы. Переиспользует каноническую модель. |
| 2 | Система отсчёта выбранного угла | **Относительно текущего курса робота.** Робот в окне всегда смотрит «вперёд», φ — это насколько довернуть. Pi-нода: цель поворота = `θ_start + φ`. Фронтенду НЕ нужна live-одометрия. |
| 3 | Границы фичи | **Только робот** (`source=robot`). Симулятор сохраняет текущий поток «вперёд D». |
| 4 | Как выбирается точка и запускается прогон | **Клик по краю окружности → маркер + readout φ° → кнопка «Старт».** Закрытие окна без «Старт» = прогон не запускается. |
| 5 | Как строить 3D-окно | **Вариант A — переиспользовать 3D-инфраструктуру Mps3D** (`RobotModel`/GLB, сетка, свет; паттерн модалки `Mps3DOverlay`). Камера наклонно-сверху. |

## 3. Цели и не-цели

### Цели

1. **Явный контроль направления.** Пользователь выбирает, куда поедет
   робот, кликом по окружности — а не полагается на неоднозначное
   «вперёд».
2. **3D-наглядность.** Модель робота `Samurai.glb` в 3D-сцене, окружность
   радиуса N вокруг неё — визуально согласовано с пост-прогонным
   3D-оверлеем.
3. **Безопасность для железа.** Явный шаг «Старт»; закрытие окна =
   ничего не запускается; предвыбранный φ=0 — безопасный дефолт «прямо».
4. **Обратная совместимость.** `target_heading=0.0` по умолчанию = ровно
   сегодняшнее поведение. Старые клиенты, симулятор, существующие тесты
   не ломаются.

### Не-цели

- Не для симулятора — пикер появляется только при `source=robot`.
- Не трогаем пост-прогонный 3D-оверлей (`Mps3DProvider/Scene/Overlay/Toast`)
  — он про **воспроизведение** траектории, это про **выбор** цели. Два
  независимых компонента.
- Не делаем навигацию к произвольной (x, y) — только точка на окружности
  фиксированного радиуса N, т.е. выбор **направления** при фиксированной
  дистанции.
- Не делаем дугу/кривую траекторию — только «развернуться → ехать прямо».
- Не переписываем каноническую модель `[s, v, θ, ω, e_int]` — обе фазы
  выражаются через `x_ref` существующей модели.
- Не добавляем live-одометрию во фронтенд — угол относительный, робот в
  окне рисуется фиксированно «вперёд».
- Не трогаем `mps_runner.py` — sim игнорирует `target_heading` (поле есть
  в запросе, но идеальный симулятор его не читает).

## 4. UX-сценарий

```
1. Пользователь на /mps настроил матрицы, в ScenarioControls выбрал
   source=robot, ввёл distance=N и v_target=V, нажал «Запустить».
2. source=robot → прогон НЕ летит сразу. MpsPageInner открывает
   модалку <MpsTargetPicker distance=N vTarget=V>.
3. Модалка (portal в document.body, тёмный backdrop):
   • Шапка: «Куда ехать роботу» + крестик ✕.
   • Тело: 3D-сцена — модель робота в центре смотрит «вперёд» (вверх
     экрана), вокруг кольцо радиуса N на полу, сетка, свет. Маркер
     предвыбран на φ=0 (прямо перед роботом).
   • Подвал: readout курса («прямо» при φ≈0, иначе «+35°» / «−40°») +
     кнопка «Старт» (активна сразу — φ=0 валиден).
4. Пользователь кликает рядом с кольцом → точка прилипает к краю кольца,
   маркер (конус) и тонкая линия робот→маркер перемещаются, readout
   обновляется. Можно кликать сколько угодно раз до «Старт».
5а. Пользователь жмёт ✕ (или клик по backdrop) → модалка закрывается,
    прогон НЕ запускается, ScenarioControls в исходном состоянии.
5б. Пользователь жмёт «Старт»:
    • Модалка закрывается.
    • runHook.run({ distance:N, v_target:V, source:'robot',
      target_heading:φ }) → POST /api/v1/mps/scenario/run.
    • Дальше — существующий robot-поток: run_id, WS-телеметрия,
      пост-прогонный 3D-оверлей (без изменений).
6. Робот: фаза TURN (разворот на месте к θ_start+φ) → фаза DRIVE
   (едет N метров, удерживая курс φ) → finished.
```

## 5. Архитектура

### Слои изменений

```
Фронтенд        MpsPageInner ветвит handleRun по source.
                robot → <MpsTargetPicker> (модалка) → <MpsTargetScene> (R3F).
                Чистая математика угла — lib/targetAngle.ts.
                  │  target_heading: φ (рад, относительный)
                  ▼
Контракт        MpsScenarioRequest += target_heading: float = 0.0.
                routers/mps.py — без правок (model_dump прокидывает поле).
                  │  MQTT mps/scenario/run { request: {..., target_heading} }
                  ▼
Pi-нода         mps_node: _RunState += target_heading, phase.
                _tick — двухфазный FSM: TURN → DRIVE.
                config.yaml — пороги turn-фазы.
```

### Сквозной поток данных

```
ScenarioControls «Запустить» (source=robot, distance=N, v_target=V)
  → MpsPageInner.handleRun(req):
      req.source === 'robot'
        ? setPicker({ distance:N, vTarget:V })   // открыть модалку
        : runHook.run(req)                       // sim — как сейчас
  → <MpsTargetPicker> рендерится
  → клик по сцене → raycast в пол → targetAngle.groundPointToAngle()
      → φ → setState(pickedAngle)
  → «Старт» → onConfirm(φ):
      setPicker(null)
      runHook.run({ distance:N, v_target:V, source:'robot',
                    target_heading:φ })
  → POST /api/v1/mps/scenario/run
  → routers/mps.py: publish MQTT mps/scenario/run
      { run_id, request: {distance, v_target, source, target_heading},
        schema_version }
  → mps_node._on_scenario_run: сохраняет target_heading=φ, phase='turn',
      снимает theta_start (уже делается)
  → _tick фаза TURN:  x_ref = [0, 0, φ, 0, 0]
      |normalize(x[θ] − φ)| < turn_tol → phase='drive', drive_t=0
  → _tick фаза DRIVE: x_ref = [s_ref, V, φ, 0, 0]   (θ_ref = φ!)
      x[s] ≥ N − reach_eps → finished('reached')
```

### Где живёт состояние пикера

Пикер — **отдельное, простое** состояние, НЕ переиспользует
`Mps3DProvider` (тот — FSM `idle|toasting|overlay` для пост-прогонного
воспроизведения). В `MpsPageInner` добавляется:

```ts
const [picker, setPicker] = useState<{ distance: number; vTarget: number } | null>(null)
```

`picker !== null` ⇒ модалка открыта. Так пред-прогонный пикер и
пост-прогонный оверлей остаются полностью независимыми.

## 6. Компоненты и интерфейсы (фронтенд)

### `lib/targetAngle.ts` (~40 строк) — новый

Чистые функции, юнит-тестятся без WebGL.

```ts
/** Точка на полу 3D-сцены (Three-координаты x, z) → относительный курс φ.
 *  Мировое: (wx, wy) = (x, -z); φ = atan2(wy, wx). Результат в (-π, π]. */
export function groundPointToAngle(x: number, z: number): number

/** Угол φ + радиус N → позиция маркера в Three-координатах [x, h, z].
 *  [N·cos φ, MARKER_H, -N·sin φ]. */
export function angleToMarkerPosition(angle: number, radius: number): [number, number, number]

/** φ → подпись для readout: «прямо» при |φ| < ~3°, иначе «+35°» / «−40°».
 *  Знак: + влево (CCW), − вправо (CW). */
export function formatHeadingLabel(angle: number): string
```

### `components/mps/MpsTargetScene.tsx` (~150 строк) — новый

R3F `<Canvas>`. Переиспользует `RobotModel` и паттерн сетки/света из
`Mps3DScene.tsx`. Для кольца — рассмотреть `components/3d/DistanceRings.tsx`
(если параметризуется радиусом) либо собственный `<torusGeometry>`.

```ts
interface MpsTargetSceneProps {
  distance: number              // радиус кольца N
  pickedAngle: number           // текущий φ (рад)
  onPick: (angle: number) => void
}
```

Содержимое сцены:
- `<RobotModel yaw={0} pitch={0} roll={0} posX={0} posY={0} stationary noSmooth/>`
  — робот в центре смотрит «вперёд» (+X мировое).
- Сетка + свет — паттерн из `Mps3DScene` (ambient + directional + hemisphere).
- Кольцо: `torusGeometry` радиуса `distance` в плоскости пола.
- Невидимая плоскость пола (`planeGeometry`, `visible={false}` или
  прозрачный материал) — мишень для raycast по клику.
- Маркер: `<coneGeometry>` на `angleToMarkerPosition(pickedAngle, distance)`,
  тонкая линия от `[0,h,0]` к маркеру.
- Камера: наклонно-сверху, **сзади-сверху от робота**, так чтобы ось
  «вперёд» (+X) робота смотрела к верху вьюпорта; точные координаты
  подбираются при реализации. `OrbitControls` — опционально, с
  ограниченным `maxPolarAngle` (вид остаётся близким к top-down, чтобы
  выбор края оставался точным).
- Клик по Canvas: R3F `onClick` (или `onPointerDown`) по плоскости пола →
  `event.point` (Three x, y, z) → `groundPointToAngle(point.x, point.z)`
  → `onPick(φ)`.

Если WebGL недоступен — `<Canvas>` не рендерит, но модалка показывает
fallback-текст; «Старт» с предвыбранным φ=0 всё равно работает.

### `components/mps/MpsTargetPicker.tsx` (~120 строк) — новый

Модалка-обёртка. Паттерн — `Mps3DOverlay.tsx` (portal в `document.body`,
backdrop, анимация появления).

```ts
interface MpsTargetPickerProps {
  distance: number
  vTarget: number
  onConfirm: (targetHeading: number) => void   // φ в радианах
  onCancel: () => void
}
```

- `createPortal(..., document.body)`, backdrop `fixed inset-0 bg-black/70`.
  Клик по backdrop и ✕ → `onCancel()`. (В отличие от `Mps3DOverlay`,
  где закрытие только по ✕ — здесь backdrop-клик безопасен, т.к. это
  отмена выбора, а не прерывание прогона.)
- Владеет состоянием `const [pickedAngle, setPickedAngle] = useState(0)`.
- Шапка: «Куда ехать роботу» + ✕.
- Тело: `<MpsTargetScene distance={distance} pickedAngle={pickedAngle}
  onPick={setPickedAngle}/>`.
- Подвал: `formatHeadingLabel(pickedAngle)` + дистанция (`N м`) + кнопка
  «Старт» → `onConfirm(pickedAngle)`. «Старт» активна всегда (φ=0 валиден).
- Размер: «мини-окно» — например `w-[640px] h-[520px]` по центру (не
  fullscreen, в отличие от пост-прогонного оверлея).

### Изменения `pages/MpsPage.tsx` (`MpsPageInner`)

1. Состояние: `const [picker, setPicker] = useState<{distance, vTarget} | null>(null)`.
2. `handleRun(req)`:
   ```ts
   function handleRun(req: MpsScenarioRequest) {
     if (req.source === 'robot') {
       setPicker({ distance: req.distance, vTarget: req.v_target })
     } else {
       runHook.run(req)            // sim — без изменений
     }
   }
   ```
3. Рендер модалки:
   ```tsx
   {picker && (
     <MpsTargetPicker
       distance={picker.distance}
       vTarget={picker.vTarget}
       onConfirm={(targetHeading) => {
         setPicker(null)
         runHook.run({
           distance: picker.distance,
           v_target: picker.vTarget,
           source: 'robot',
           target_heading: targetHeading,
         })
       }}
       onCancel={() => setPicker(null)}
     />
   )}
   ```

`ScenarioControls.tsx` — **не трогаем**: ветвление целиком в
`MpsPageInner.handleRun`. (Опциональный полиш — менять подпись кнопки на
«Выбрать цель…» при `source=robot` — в скоуп НЕ входит.)

### Изменения `lib/mpsApi.ts` / `types/mps.ts`

`MpsScenarioRequest` += `target_heading?: number` (рад, относительный
курс; опционально для обратной совместимости, бэкенд дефолтит в 0.0).
`mpsApi.runScenario` уже сериализует весь объект запроса — отдельная
правка не нужна.

## 7. Контракт API

### `compute_node/dashboard/schemas/mps.py`

`MpsScenarioRequest` получает поле:

```python
target_heading: float = Field(
    default=0.0,
    ge=-math.pi, le=math.pi,
    description='Относительный целевой курс (рад) от курса на старте '
                'сценария. 0.0 = ехать прямо вперёд (поведение по '
                'умолчанию). Используется только при source="robot".',
)
```

Дефолт `0.0` ⇒ существующие клиенты, симулятор и тесты не затронуты.

### `compute_node/dashboard/routers/mps.py`

**Без изменений кода.** Robot-путь уже делает
`request.model_dump()` → MQTT-payload, новое поле прокидывается
автоматически. (Допустимо добавить поясняющий комментарий.)

### MQTT-топик `mps/scenario/run`

Payload расширяется автоматически:
```json
{
  "run_id": "...",
  "request": { "distance": 2.0, "v_target": 0.15,
               "source": "robot", "target_heading": 0.61 },
  "schema_version": "1.0"
}
```

### `docs/mps/api.md`

Описать новое поле `target_heading` и двухфазный сценарий robot-прогона
(TURN → DRIVE).

## 8. Pi-нода — двухфазный FSM (`pi_nodes/nodes/mps_node.py`)

### `_RunState`

`__slots__` += `target_heading`, `phase`, `turn_t`, `drive_t`.

```python
def __init__(self, run_id, distance, v_target,
             s_start=0.0, theta_start=0.0, target_heading=0.0):
    ...
    self.target_heading = target_heading   # φ, рад, относительный
    self.phase = 'turn'                    # 'turn' | 'drive'
    self.turn_t = 0.0                      # часы фазы TURN
    self.drive_t = 0.0                     # часы фазы DRIVE (s_ref ramp, timeout)
```

Существующий `run.t` заменяется на два раздельных таймера: `turn_t` и
`drive_t`. `drive_t` стартует с 0 при переходе TURN→DRIVE — чтобы
`s_ref` ramp и drive-timeout считались от начала движения, а не от старта
сценария.

### `_on_scenario_run`

Читает `target_heading` из `request` (дефолт 0.0), валидирует диапазон
`[−π, π]` (precondition-ошибка при выходе за границы), передаёт в
`_RunState`. `phase` стартует с `'turn'`. `theta_start` снимается как
сейчас.

### `_tick` — фаза TURN

```
x = scenario-relative state (x[_S] -= s_start; x[_THETA] = normalize(θ - theta_start))
x_ref = [0.0, 0.0, φ, 0.0, 0.0]          # развернуться к φ, стоя на месте
u = mpc.step(x, x_ref)
u[0] = 0.0                                # жёстко: чистое вращение, без хода
u[1] = clip(u[1], ±omega_max_in_turn)     # отдельный, более высокий кап
publish cmd_vel, publish telemetry

if turn_t > turn_timeout_s:  finish('timeout')
if |normalize(x[_THETA] - φ)| < turn_tolerance_rad:
    phase = 'drive'; drive_t = 0.0        # переход
turn_t += tick_dt
```

### `_tick` — фаза DRIVE

Существующая логика «вперёд», с двумя отличиями:
- `s_ref = min(distance, drive_t * v_target)` (по `drive_t`, не `run.t`).
- `x_ref = [s_ref, v_target, φ, 0.0, 0.0]` — **`θ_ref = φ`**, не 0:
  робот удерживает выбранный курс, а не доворачивает обратно к 0.
- `u[1] = clip(u[1], ±omega_max_in_forward)` — как сейчас.
- `reached` / drive-`timeout` — как сейчас, но timeout по `drive_t`.

При φ=0 фаза TURN завершается на первом же тике (`|x[θ] − 0| < tol`,
робот стоит ровно) и DRIVE = ровно сегодняшнее поведение.

### `config.yaml` — секция `mps.scenario`

Добавить (рядом с существующими `distance_max`, `v_target_max`,
`omega_max_in_forward`):

```yaml
mps:
  scenario:
    turn_tolerance_rad: 0.05        # |θ - φ| < этого ⇒ TURN завершён (~3°)
    turn_timeout_s: 10.0            # TURN не сошёлся за это ⇒ timeout
    omega_max_in_turn: 1.0          # кап ω в TURN (выше omega_max_in_forward=0.5)
```

Читаются через тот же `self._cfg(...)` с дефолтами (нода не падает без
config_loader).

### Знак угла φ (sign convention) — критично

Единое соглашение **во всех трёх местах**:
- φ = 0 — «вперёд» (ось +X мировая / `RobotModel` yaw=0).
- φ > 0 — поворот **влево** (CCW), φ < 0 — **вправо** (CW).
- Пикер: `groundPointToAngle` = `atan2(worldY, worldX)` = `atan2(-z, x)`.
- Pi: `x[_THETA]` из одометрии — CCW-положительный, scenario-relative
  (после фикса бага). Цель TURN = довести `x[_THETA]` до `φ`.

Расхождение знака = робот поедет зеркально. Выносится в раздел рисков и
покрывается тестом «φ>0 ⇒ робот крутится в сторону +θ».

### Телеметрия и метрики

`MpsTelemetryPoint` (`t, x, u, y, s_remaining`) — **схема не меняется**.
Во время TURN `x[_S] ≈ 0`, `x[_THETA]` растёт к φ; во время DRIVE
`x[_S]` растёт вдоль φ. Пост-прогонный 3D-оверлей рисует траекторию как
`(s·cos θ, s·sin θ)` — естественно покажет «робот развернулся на месте,
потом уехал под углом φ». Поле `t` в точке = `turn_t` во время TURN и
`turn_total + drive_t` во время DRIVE (монотонно). Метрики в `_finish_run`
считаются по всей телеметрии; `settling_time` = полное время (turn +
drive).

## 9. Граничные случаи

| Случай | Поведение |
|---|---|
| φ ≈ 0 (выбрали «прямо») | TURN завершается на первом тике → чистый DRIVE = сегодняшнее поведение. Регрессия-якорь. |
| φ ≈ ±π (выбрали «назад») | Робот разворачивается на ~180°, потом едет. Направление разворота — по знаку `normalize(x[θ] − φ)`. На точном ±π — вырожденный случай, любая сторона приемлема. |
| Закрыли пикер (✕ или backdrop) без «Старт» | `onCancel` → `setPicker(null)`. Прогон НЕ запускается. |
| Робот оффлайн | «Старт» → POST → 503 (как сейчас). Ошибка всплывает существующим механизмом. Пред-проверки онлайна нет — консистентно с текущим потоком. |
| TURN не сходится (робот застрял) | `turn_t > turn_timeout_s` → `_finish_run('timeout')`, cmd_vel=0. |
| `distance` невалидна | Пикер не открыть — `ScenarioControls` уже валидирует диапазон 0.1–5.0 до вызова `onRun`. |
| WebGL недоступен | `MpsTargetScene` показывает fallback-текст; «Старт» с φ=0 работает (деградация — едет прямо). |
| Несколько кликов до «Старт» | `pickedAngle` просто перезаписывается; летит последнее значение. |
| sim-прогон | `handleRun` видит `source=sim` → `runHook.run` напрямую, пикер не открывается. `target_heading` в запросе отсутствует/0.0, `mps_runner` его не читает. |
| Старый клиент / отсутствует `target_heading` | Pydantic-дефолт `0.0` → поведение «вперёд». |

## 10. Тестирование

### Фронтенд (vitest + @testing-library/react)

| Файл | Покрытие |
|---|---|
| `lib/targetAngle.test.ts` (новый) | `groundPointToAngle`: точка спереди → φ≈0; слева → φ>0; справа → φ<0; сзади → φ≈±π. `angleToMarkerPosition`: радиус сохраняется, маппинг знака. `formatHeadingLabel`: «прямо» в дедзоне, «+»/«−» вне. |
| `components/mps/MpsTargetPicker.test.tsx` (новый) | Рендерит шапку/подвал; «Старт» зовёт `onConfirm` с текущим `pickedAngle`; ✕ и backdrop-клик зовут `onCancel`; readout обновляется при смене угла (через мок `onPick`). |
| `pages/MpsPage.test.tsx` (правка) | `handleRun` с `source=robot` открывает `<MpsTargetPicker>`, прогон НЕ летит; `source=sim` — `runHook.run` вызван сразу, пикер не открыт; `onConfirm` закрывает пикер и зовёт `runHook.run` с `target_heading`. |

`MpsTargetScene` (R3F Canvas) — **не** юнит-тестим (нет jsdom-поддержки
WebGL, как зафиксировано в спеке 2026-05-11). Логика угла вынесена в
`targetAngle.ts` именно ради тестируемости.

### Бэкенд (pytest)

| Файл | Покрытие |
|---|---|
| `tests/test_mps_router.py` (правка) | `MpsScenarioRequest` принимает `target_heading`; валидация `[−π, π]` (за границей → 422/ошибка); robot-прогон кладёт `target_heading` в MQTT-payload; дефолт 0.0 при отсутствии поля. |

### Pi-нода (pytest, TDD — `tests/test_mps_node.py`, правка)

- TURN: при φ>0 первый `cmd_vel.angular_z` > 0 (крутится к +θ); при φ<0 — < 0.
- TURN→DRIVE: подать odom с `x[θ] ≈ φ` → `phase` становится `'drive'`.
- DRIVE: после перехода `x_ref[θ] == φ` (робот держит курс, не доворачивает к 0).
- **φ=0 → регрессия:** существующие тесты `test_tick_*` остаются зелёными;
  TURN завершается мгновенно, DRIVE идентичен сегодняшнему.
- TURN timeout: `turn_t` превысил `turn_timeout_s` без схождения →
  `mps/scenario/finished` со `status='timeout'`.
- `target_heading` вне `[−π, π]` в `scenario/run` → `mps/error`
  `precondition`.

### Ручная проверка (нужно железо — см. ниже)

`npm run dev` → `/mps`, source=robot, distance=2, «Запустить» → пикер
открылся, модель робота видна, кольцо радиуса 2; клик слева → маркер
прилип, readout «+N°»; «Старт» → робот развернулся влево, поехал;
повторить с φ=0 → робот едет прямо без разворота.

## 11. Список файлов

### Новые

| Файл | Назначение | ~строк |
|---|---|---|
| `compute_node/frontend/src/lib/targetAngle.ts` | Чистая математика: точка пола↔угол, подпись курса | ~40 |
| `compute_node/frontend/src/lib/targetAngle.test.ts` | Юнит-тесты математики угла | ~70 |
| `compute_node/frontend/src/components/mps/MpsTargetScene.tsx` | R3F-сцена: робот, кольцо N, маркер, raycast-клик | ~150 |
| `compute_node/frontend/src/components/mps/MpsTargetPicker.tsx` | Модалка-обёртка, состояние `pickedAngle`, «Старт»/✕ | ~120 |
| `compute_node/frontend/src/components/mps/MpsTargetPicker.test.tsx` | Юнит-тесты модалки | ~80 |
| `docs/superpowers/plans/2026-05-14-mps-target-picker.md` | План реализации (создаётся скиллом writing-plans) | — |

### Изменяемые

| Файл | Правки |
|---|---|
| `compute_node/frontend/src/pages/MpsPage.tsx` | `picker`-состояние, ветвление `handleRun` по `source`, рендер `<MpsTargetPicker>` |
| `compute_node/frontend/src/pages/MpsPage.test.tsx` | Кейсы: robot открывает пикер, sim бежит сразу, `onConfirm` |
| `compute_node/frontend/src/types/mps.ts` | `MpsScenarioRequest += target_heading?: number` |
| `compute_node/dashboard/schemas/mps.py` | `MpsScenarioRequest += target_heading: float = 0.0`, валидация `[−π, π]` |
| `pi_nodes/nodes/mps_node.py` | `_RunState` (поля turn/drive), `_on_scenario_run` (читает `target_heading`), `_tick` (двухфазный FSM TURN→DRIVE), `_finish_run` (метрики и `settling_time` по `turn_t + drive_t` вместо `run.t`) |
| `tests/test_mps_node.py` | Тесты двухфазного FSM (см. §10) |
| `tests/test_mps_router.py` | Тесты контракта `target_heading` |
| `config.yaml` | `mps.scenario.turn_tolerance_rad`, `turn_timeout_s`, `omega_max_in_turn` |
| `docs/mps/api.md` | Описание `target_heading` + двухфазный robot-сценарий |

### Не затрагиваем

- `compute_node/mps_runner.py` — симулятор игнорирует `target_heading`.
- Пост-прогонные 3D-компоненты (`Mps3DProvider/Scene/Overlay/Toast`) —
  независимы от пикера.
- `compute_node/frontend/src/components/mps/ScenarioControls.tsx` —
  ветвление целиком в `MpsPageInner`.
- Каноническая модель `pi_nodes/control/{mpc_controller,state_space_model}.py`
  — переиспользуется как есть.

## 12. Допущения и риски

1. **Знак угла φ.** Самый вероятный баг — расхождение CCW/CW между
   пикером (`atan2`) и Pi-нодой (`x[θ]` из одометрии). Митигация: явное
   соглашение в §8 + тест «φ>0 ⇒ angular_z>0».
2. **Точность raycast-клика с наклонной камеры.** Нам нужен только
   **угол** (маркер всё равно прилипает к кольцу радиуса N), а угол
   устойчив к погрешности проекции. Если на практике выбор у самого
   центра даёт скачущий угол — добавить «мёртвую зону»: клики ближе
   `0.3·N` к центру игнорировать.
3. **TURN-фаза на реальном железе.** Разворот на месте дифференциального
   робота зависит от трения/люфта. `omega_max_in_turn` и
   `turn_tolerance_rad` — стартовые значения; финальная настройка — на
   роботе (см. ниже). Если робот «не доворачивает» — увеличить кап или
   ослабить толеранс.
4. **`RobotModel` reuse.** Компонент уже принимает `posX/posY/yaw/
   stationary/noSmooth` — для статичного робота в центре пропсы
   `yaw=0, posX=0, posY=0, stationary, noSmooth` достаточны, правок
   `RobotModel` не требуется.
5. **`DistanceRings` reuse.** `components/3d/DistanceRings.tsx` может
   подойти для кольца, если параметризуется радиусом; иначе — свой
   `torusGeometry` (тривиально). Решение — при реализации.
6. **Камера.** Точные координаты камеры (наклон, дистанция) подбираются
   при реализации под читаемость сцены; в спеке зафиксирован только
   инвариант «вид близок к top-down, +X робота смотрит вверх вьюпорта».

## 13. Физическая верификация (нужно железо)

Логика покрывается тест-сьютом, но **поведение на роботе** проверяется
только на железе:
- TURN-фаза реально разворачивает робота к φ и не «недокручивает».
- Пороги `turn_tolerance_rad` / `omega_max_in_turn` / `turn_timeout_s`
  адекватны реальному роботу.
- Знак φ совпадает (выбрал влево — поехал влево).
- Переход TURN→DRIVE плавный, без рывка.

Это продолжение уже отложенной robot-верификации МПС (см.
`memory/physical_tests_pending.md`). До железа фича считается
«реализована + покрыта тестами», но не «проверена на роботе».
