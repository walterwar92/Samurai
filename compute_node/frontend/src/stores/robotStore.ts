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
 * SocketIO: один singleton-сокет на жизнь приложения, создаётся в connect().
 * Backend (compute_node/dashboard) emit'ит 'state_update' каждые 100 ms.
 */
import { io, type Socket } from 'socket.io-client'
import { create } from 'zustand'

import type { RobotState } from '@/types/robot'

// ── Module-level singleton socket ──────────────────────────────────────
//
// SocketIO держим вне store: store хранит только данные/статус, а сокет —
// IO-ресурс. Это упрощает SSR и тестирование (mock IO без mock store).
let socket: Socket | null = null

interface RobotStoreState {
  /** Последний snapshot от backend (через SocketIO state_update). */
  state: RobotState | null
  /** Подключён ли SocketIO к серверу. */
  connected: boolean

  // ── Actions ────────────────────────────────────────────────────────
  /**
   * Инициализировать SocketIO. Идемпотентно: повторные вызовы no-op
   * (нужно чтобы StrictMode double-invoke не открывал второй коннект).
   */
  connect: () => void
  /** Закрыть SocketIO (вызывается при unmount App.tsx). */
  disconnect: () => void
  /** Послать voice-подобную команду (`send_command` event). */
  send: (text: string) => void
  /** Reset симулятора (no-op в robot-mode). */
  resetSim: () => void
}

export const useRobotStore = create<RobotStoreState>((set) => ({
  state: null,
  connected: false,

  connect: () => {
    if (socket !== null) return  // Idempotent
    const s = io({ transports: ['websocket', 'polling'] })
    s.on('connect', () => set({ connected: true }))
    s.on('disconnect', () => set({ connected: false }))
    s.on('state_update', (data: RobotState) => set({ state: data }))
    socket = s
  },

  disconnect: () => {
    if (socket === null) return
    socket.disconnect()
    socket = null
    set({ state: null, connected: false })
  },

  send: (text: string) => {
    socket?.emit('send_command', { text })
  },

  resetSim: () => {
    socket?.emit('reset_sim', {})
  },
}))

/**
 * Удобные actions без хука. Используем когда нужно вызвать action
 * вне React-компонента (например в module-level helper).
 */
export const robotActions = {
  connect: () => useRobotStore.getState().connect(),
  disconnect: () => useRobotStore.getState().disconnect(),
  send: (text: string) => useRobotStore.getState().send(text),
  resetSim: () => useRobotStore.getState().resetSim(),
}
