import { createContext, useContext, useEffect, useState, useCallback, type ReactNode } from 'react'
import { io, type Socket } from 'socket.io-client'
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
  const [state, setState] = useState<RobotState | null>(null)
  const [connected, setConnected] = useState(false)
  const [socket, setSocket] = useState<Socket | null>(null)

  useEffect(() => {
    const s = io({ transports: ['websocket', 'polling'] })

    s.on('connect', () => setConnected(true))
    s.on('disconnect', () => setConnected(false))
    s.on('state_update', (data: RobotState) => setState(data))

    setSocket(s)

    return () => {
      s.disconnect()
    }
  }, [])

  const sendCommand = useCallback(
    (text: string) => {
      socket?.emit('send_command', { text })
    },
    [socket]
  )

  const resetSim = useCallback(() => {
    socket?.emit('reset_sim', {})
  }, [socket])

  return (
    <SocketContext.Provider value={{ state, connected, sendCommand, resetSim }}>
      {children}
    </SocketContext.Provider>
  )
}

export function useSocket() {
  return useContext(SocketContext)
}
