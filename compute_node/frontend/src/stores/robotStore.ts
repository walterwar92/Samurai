/**
 * Zustand store для состояния робота (#6, 2026-04).
 *
 * Заменяет SocketProvider + useState/Context для shared state. Преимущества:
 *   - Гранулярные селекторы (useRobotStore(s => s.battery_percent)) →
 *     re-render только тех компонентов, чьё значение изменилось.
 *   - Глобальный singleton без provider'а — Header.tsx и debug-модал могут
 *     читать состояние без обёртки в Provider.
 *   - Проще тестировать и проще мигрировать на server-state library
 *     (TanStack Query) если потребуется.
 *
 * Public хуки:
 *   useRobotStore               — полный state + actions (избегайте, лучше селектор)
 *   useRobotState() (legacy)    — selector(s => s.state) для обратной совместимости
 *
 * Этот файл (Z1) — пустой скелет: state-данные, типы actions. SocketIO
 * подключается в Z2 (action `connect`/`disconnect`). Гранулярные селекторы
 * добавляются в Z4.
 */
import { create } from 'zustand'

import type { RobotState } from '@/types/robot'

// ── Initial state ──────────────────────────────────────────────────────
//
// `state` — последний `state_update` от backend через SocketIO. null если
// ещё не пришёл (на старте приложения / при reconnect).
//
// `connected` — статус сокета (для индикатора в Header).

interface RobotStoreState {
  /** Последний snapshot от backend (через SocketIO state_update). */
  state: RobotState | null
  /** Подключение к SocketIO */
  connected: boolean

  // ── Actions ────────────────────────────────────────────────────────
  /** Инициализировать SocketIO (одно подключение на жизнь приложения). */
  connect: () => void
  /** Закрыть SocketIO (вызывается при unmount App.tsx). */
  disconnect: () => void
  /**
   * Послать voice-подобную команду на бэк (`send_command` event).
   * Тонкая обёртка над socket.emit — backend интерпретирует как голосовую.
   */
  send: (text: string) => void
  /** Reset симулятора (no-op в robot-mode). */
  resetSim: () => void
}

export const useRobotStore = create<RobotStoreState>((set, _get) => ({
  state: null,
  connected: false,

  // ── Stub actions (заполнятся в Z2) ─────────────────────────────────
  connect: () => {
    // SocketIO connection created in Z2. На текущей фазе только обнуляем
    // connected в false при двойном вызове.
    set({ connected: false })
  },

  disconnect: () => {
    set({ state: null, connected: false })
  },

  send: (_text: string) => {
    // Будет publish в socket.emit('send_command', { text }) в Z2
  },

  resetSim: () => {
    // Будет socket.emit('reset_sim', {}) в Z2
  },
}))
