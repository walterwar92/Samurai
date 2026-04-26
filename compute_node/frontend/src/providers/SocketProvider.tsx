/**
 * SocketProvider — backward-compat shim над Zustand store (#6, 2026-04).
 *
 * Раньше провайдер сам держал Socket.IO соединение и выставлял его через
 * useState/Context. Теперь сокет живёт в `src/stores/robotStore.ts` как
 * singleton, а этот компонент остался как:
 *
 *   1. Wrapper для backward-compat: компоненты, использующие useSocket(),
 *      продолжают работать без изменений (мигрируем на selectors в Z5).
 *   2. Lifecycle-driver: вызывает store.connect() при монтировании,
 *      store.disconnect() при unmount. App.tsx ставит этот провайдер
 *      на корне.
 *
 * После Z5/Z6 SocketProvider можно удалить — connect()/disconnect()
 * перенести в App.tsx, useSocket() либо удалить, либо оставить как алиас.
 */
import { createContext, useContext, useEffect, type ReactNode } from 'react'

import { useRobotStore } from '@/stores/robotStore'
import type { RobotState } from '@/types/robot'

interface SocketContextValue {
  state: RobotState | null
  connected: boolean
  sendCommand: (text: string) => void
  resetSim: () => void
}

const SocketContext = createContext<SocketContextValue>({
  state: null,
  connected: false,
  sendCommand: () => {},
  resetSim: () => {},
})

export function SocketProvider({ children }: { children: ReactNode }) {
  const state = useRobotStore((s) => s.state)
  const connected = useRobotStore((s) => s.connected)
  const send = useRobotStore((s) => s.send)
  const resetSim = useRobotStore((s) => s.resetSim)
  const connect = useRobotStore((s) => s.connect)
  const disconnect = useRobotStore((s) => s.disconnect)

  // Lifecycle: один коннект на жизнь приложения.
  useEffect(() => {
    connect()
    return () => disconnect()
  }, [connect, disconnect])

  return (
    <SocketContext.Provider
      value={{ state, connected, sendCommand: send, resetSim }}
    >
      {children}
    </SocketContext.Provider>
  )
}

export function useSocket() {
  return useContext(SocketContext)
}
