#!/usr/bin/env bash
# scripts/cmds/voice-llm.sh — LLM voice intent parser на ноутбуке (#2, 2026-04)
#
# Подписывается на samurai/{robot_id}/voice_command, парсит через
# Ollama (Qwen 2.5 7B) → publish samurai/{robot_id}/voice/intent.
# fsm_node на Pi подхватывает intent и выполняет structured action.
# При confidence < 0.5 — fsm fallback'ит на regex-парсинг.
#
# Использование:
#   samurai voice-llm                              # auto-discover Pi, Ollama default
#   samurai voice-llm --pi 192.168.1.50            # явный IP MQTT-брокера
#   samurai voice-llm --backend mock               # для dev без Ollama
#   samurai voice-llm --model qwen2.5:1.5b         # быстрая модель на CPU
#   samurai voice-llm --ollama-host http://gpu-rig:11434  # remote Ollama

set -euo pipefail

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../lib" && pwd)"
# shellcheck source=../lib/common.sh
source "$LIB_DIR/common.sh"
# shellcheck source=../lib/checks.sh
source "$LIB_DIR/checks.sh"
# shellcheck source=../lib/discover.sh
source "$LIB_DIR/discover.sh"
# shellcheck source=../lib/locking.sh
source "$LIB_DIR/locking.sh"

main() {
    local pi_ip_arg=""
    local robot_id="robot1"
    local backend="ollama"
    local model="qwen2.5:7b"
    local ollama_host="http://localhost:11434"

    while [[ $# -gt 0 ]]; do
        case "$1" in
            --pi)            pi_ip_arg="$2"; shift 2 ;;
            --robot-id)      robot_id="$2"; shift 2 ;;
            --backend)       backend="$2"; shift 2 ;;
            --model)         model="$2"; shift 2 ;;
            --ollama-host)   ollama_host="$2"; shift 2 ;;
            -h|--help)
                cat <<EOF
Использование: samurai voice-llm [опции]

Запускает compute_node/llm_voice — LLM-парсер голосовых команд.
Подписывается на samurai/{robot}/voice_command (raw text от Vosk),
публикует samurai/{robot}/voice/intent (structured JSON для fsm).

Опции:
  --pi IP            IP Raspberry Pi (без — mDNS auto-discovery)
  --robot-id ID      MQTT robot_id (default: robot1)
  --backend ollama|mock  default: ollama
  --model NAME       Ollama model tag (default: qwen2.5:7b)
                     Альтернативы: qwen2.5:1.5b (быстрая), llama3.2:3b
  --ollama-host URL  default: http://localhost:11434
  -h, --help         Эта справка

Setup Ollama (один раз на ноуте):
  curl -fsSL https://ollama.com/install.sh | sh
  ollama pull qwen2.5:7b   # ~4.7 GB
  ollama serve              # listening на :11434

Примеры:
  samurai voice-llm                              # auto-discover, qwen 7b
  samurai voice-llm --backend mock               # dev без Ollama
  samurai voice-llm --model qwen2.5:1.5b         # быстрая модель на CPU
EOF
                exit 0
                ;;
            *) die "Неизвестный аргумент: $1 (см. samurai voice-llm --help)" ;;
        esac
    done

    acquire_lock voice-llm

    log_step "Поиск Raspberry Pi (MQTT broker)"
    local pi_ip
    pi_ip=$(discover_pi_interactive "$pi_ip_arg")

    print_banner "L L M   V O I C E" "Qwen 2.5 7B  -  Ollama  -  MQTT"

    check_python 9
    check_pip_packages \
        "paho.mqtt.client:paho-mqtt" \
        "httpx:httpx" \
        "pydantic:pydantic"

    log_ok "Pi: ${BOLD}$pi_ip${NC}"
    if ! check_tcp_port "$pi_ip" 1883; then
        log_warn "Порт 1883 не отвечает — убедись что mosquitto на Pi"
    fi

    if [[ "$backend" == "ollama" ]]; then
        # Проверим что Ollama жив (если localhost) — иначе всё рухнет
        local ollama_check_url="${ollama_host}/api/tags"
        if [[ "$ollama_host" == *"localhost"* || "$ollama_host" == *"127.0.0.1"* ]]; then
            if ! curl -sf --max-time 2 "$ollama_check_url" >/dev/null 2>&1; then
                log_warn "Ollama на $ollama_host не отвечает."
                log_warn "  Запусти: ollama serve  (и убедись что pull qwen2.5:7b сделан)"
            else
                log_ok "Ollama: ${GREEN}${ollama_host}${NC} (model=${model})"
            fi
        fi
    fi

    if load_mqtt_creds; then
        log_ok "MQTT auth: user=${BOLD}${SAMURAI_MQTT_USER}${NC}"
    fi

    log_step "Запуск парсера"
    echo ""
    echo -e "  ${BOLD}┌──────────────────────────────────────────┐${NC}"
    echo -e "  ${BOLD}│${NC}  Pi MQTT: ${CYAN}${pi_ip}:1883${NC}"
    echo -e "  ${BOLD}│${NC}  Robot:   ${GREEN}${robot_id}${NC}"
    echo -e "  ${BOLD}│${NC}  Backend: ${GREEN}${backend}${NC}"
    [[ "$backend" == "ollama" ]] && \
    echo -e "  ${BOLD}│${NC}  Model:   ${GREEN}${model}${NC} @ ${ollama_host}"
    echo -e "  ${BOLD}│${NC}  Подписки: ${GREEN}voice_command${NC}"
    echo -e "  ${BOLD}│${NC}  Топики:  ${GREEN}samurai/${robot_id}/voice/{intent,llm/status}${NC}"
    echo -e "  ${BOLD}└──────────────────────────────────────────┘${NC}"
    echo ""
    echo -e "${YELLOW}  ── Ctrl+C для остановки ──${NC}"
    echo ""

    local args=(
        --broker "$pi_ip"
        --port 1883
        --robot-id "$robot_id"
        --backend "$backend"
        --model "$model"
        --ollama-host "$ollama_host"
    )

    cd "$SAMURAI_ROOT"
    exec python3 -m compute_node.llm_voice "${args[@]}"
}

main "$@"
