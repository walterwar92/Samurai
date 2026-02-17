import { useSocket } from '@/providers/SocketProvider'

export function useRobotState() {
  const { state } = useSocket()
  return state
}
