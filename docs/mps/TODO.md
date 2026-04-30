# МПС — TODO

Живой список задач по модулю. Двигаем пункты в `## Done` после
мерджа в `dev`.

## Now

- [ ] Описать формальный интерфейс модуля (входы / выходы / FSM).
- [ ] Решить: отдельный процесс или подключаемый плагин в
      `pi_nodes/` / `compute_node/`.
- [ ] Подобрать тестовый рантайм (pytest на ноуте vs Pi-hardware-loop).

## Next

- [ ] MQTT-схема для `samurai/{id}/mps/...` — топики, payload
      (по аналогии с `pi_nodes/schemas.py`).
- [ ] REST-роутер `/api/v1/mps/*` на dashboard, если нужен внешний
      контроль из UI.
- [ ] Конфиг-секция `mps:` в `config.yaml` + Pydantic-схема.
- [ ] Базовая визуализация в дашборде (страница / виджет).

## Backlog / parking lot

- [ ] Интеграционные тесты с Mosquitto в testcontainers
      (по примеру `tests/test_mqtt_integration.py`).
- [ ] Документ в `docs/mps/architecture.md` с диаграммой.

## Done

_(пусто — наполняется по факту merge)_
