import type { MpsTelemetryPoint, ScenarioStatus } from '@/types/mps'

interface Mps3DSceneProps {
  telemetry: MpsTelemetryPoint[]
  distance: number
  status: ScenarioStatus
}

// Заглушка — полная реализация в Task 5
export function Mps3DScene(_props: Mps3DSceneProps) {
  return <div data-scene-placeholder="true" className="w-full h-full bg-[#1a1a2e]" />
}
