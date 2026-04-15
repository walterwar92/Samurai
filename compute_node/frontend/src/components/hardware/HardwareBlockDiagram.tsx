import type { HardwarePreset } from '@/types/hardware'

type BlockId = 'motors' | 'servos' | 'camera' | 'imu' | 'range' | 'leds'

interface HardwareBlockDiagramProps {
  preset: HardwarePreset
  activeBlock: BlockId | null
  onBlockClick: (id: BlockId) => void
}

interface BlockInfo {
  id: BlockId
  label: string
  icon: string
  summary: (p: HardwarePreset) => string
  enabled: (p: HardwarePreset) => boolean
}

const BLOCKS: BlockInfo[] = [
  {
    id: 'motors',
    label: 'Моторы',
    icon: '⚙',
    summary: (p) => {
      const count = Object.keys(p.motors.channels).length
      return `${p.motors.driver} / ${count} шт`
    },
    enabled: () => true,
  },
  {
    id: 'servos',
    label: 'Сервоприводы',
    icon: '🔧',
    summary: (p) => {
      const total = 1 + p.servos.arm.channels.length
      return `${p.servos.driver} / ${total} шт`
    },
    enabled: () => true,
  },
  {
    id: 'camera',
    label: 'Камера',
    icon: '📷',
    summary: (p) => p.camera.type === 'none' ? 'Отключена' : `${p.camera.type} ${p.camera.width}x${p.camera.height}`,
    enabled: (p) => p.camera.type !== 'none',
  },
  {
    id: 'imu',
    label: 'IMU',
    icon: '🧭',
    summary: (p) => p.imu.type === 'none' ? 'Отключен' : `${p.imu.type.toUpperCase()} @ ${p.imu.address}`,
    enabled: (p) => p.imu.type !== 'none',
  },
  {
    id: 'range',
    label: 'Дальномер',
    icon: '📡',
    summary: (p) => {
      if (p.range_sensor.type === 'none') return 'Отключен'
      if (p.range_sensor.type === 'hc_sr04')
        return `HC-SR04 T:${p.range_sensor.trigger_pin} E:${p.range_sensor.echo_pin}`
      return p.range_sensor.type
    },
    enabled: (p) => p.range_sensor.type !== 'none',
  },
  {
    id: 'leds',
    label: 'LED',
    icon: '💡',
    summary: (p) => p.leds.type === 'none' ? 'Отключены' : `${p.leds.type} x${p.leds.count} GPIO${p.leds.gpio_pin}`,
    enabled: (p) => p.leds.type !== 'none',
  },
]

export function HardwareBlockDiagram({ preset, activeBlock, onBlockClick }: HardwareBlockDiagramProps) {
  return (
    <div className="grid grid-cols-2 md:grid-cols-3 gap-2">
      {BLOCKS.map((b) => {
        const active = activeBlock === b.id
        const enabled = b.enabled(preset)
        return (
          <button
            key={b.id}
            onClick={() => onBlockClick(b.id)}
            className={`
              relative p-3 rounded-lg border text-left transition-all
              ${active
                ? 'border-primary bg-primary/10 ring-1 ring-primary/30'
                : enabled
                  ? 'border-zinc-700 bg-zinc-800/50 hover:border-zinc-500 hover:bg-zinc-800'
                  : 'border-zinc-800 bg-zinc-900/30 opacity-50 hover:opacity-70'
              }
            `}
          >
            {/* Status dot */}
            <div className={`absolute top-2 right-2 w-2 h-2 rounded-full ${
              enabled ? 'bg-emerald-500' : 'bg-zinc-600'
            }`} />

            <div className="text-lg mb-1">{b.icon}</div>
            <div className="text-xs font-medium text-zinc-200 mb-0.5">{b.label}</div>
            <div className="text-[10px] text-zinc-500 font-mono leading-tight">
              {b.summary(preset)}
            </div>
          </button>
        )
      })}
    </div>
  )
}

export type { BlockId }
