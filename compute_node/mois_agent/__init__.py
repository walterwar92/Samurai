"""MOIS HTTP-агент Samurai — мост сайт ↔ робот.

Запускается на ноутбуке (Compute). Polling-based:
  GET  api_url?action=poll          — забрать команды
  POST api_url?action=cmd-result    — отдать результат
  POST api_url?action=capabilities  — зарегистрироваться при старте
  POST api_url?action=telemetry     — отправить телеметрию

Все handlers команд транслируют запрос в локальный dashboard FastAPI
(:5000), который уже знает как достучаться до Pi (MQTT) и Samcan
(прокси на :5005). Это даёт единую точку входа и не дублирует логику.
"""
