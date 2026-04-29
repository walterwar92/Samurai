/**
 * useRobotState — селектор полного state робота из Zustand store.
 *
 * До #6 жил в SocketProvider context, теперь — тонкий wrapper над
 * useRobotStore. Сохранён ради backward-compat: 8+ компонентов уже
 * импортят `useRobotState`. Новые компоненты должны использовать
 * гранулярные селекторы (Z4): usePose(), useBattery(), useFsm() и т.д.
 */
import { useRobotStore } from '@/stores/robotStore'

export function useRobotState() {
  return useRobotStore((s) => s.state)
}
