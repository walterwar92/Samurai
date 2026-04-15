import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { HardwarePreset, MotorDriver } from '@/types/hardware'

const DRIVER_OPTIONS: { id: MotorDriver; label: string }[] = [
  { id: 'pca9685', label: 'PCA9685' },
  { id: 'direct_gpio', label: 'GPIO' },
  { id: 'custom', label: 'Custom' },
]

interface MotorBlockProps {
  motors: HardwarePreset['motors']
  onChange: (motors: HardwarePreset['motors']) => void
}

const inputCls = 'w-full bg-zinc-900 border border-zinc-700 rounded px-2 py-1 text-xs font-mono text-zinc-200 focus:outline-none focus:border-amber-500'

export function MotorBlock({ motors, onChange }: MotorBlockProps) {
  const updateChannel = (key: string, field: 'in1' | 'in2', val: string) => {
    const num = parseInt(val)
    if (isNaN(num)) return
    const updated = { ...motors, channels: { ...motors.channels } }
    updated.channels[key] = { ...updated.channels[key], [field]: num }
    onChange(updated)
  }

  const updateLabel = (key: string, val: string) => {
    const updated = { ...motors, channels: { ...motors.channels } }
    updated.channels[key] = { ...updated.channels[key], label: val }
    onChange(updated)
  }

  const updateDriver = (driver: MotorDriver) => {
    onChange({ ...motors, driver })
  }

  const updateCount = (val: string) => {
    const num = parseInt(val)
    if (num >= 1 && num <= 8) {
      onChange({ ...motors, count: num })
    }
  }

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Моторы DC
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2">
        {/* Driver type */}
        <div className="flex items-center gap-2">
          <span className="text-[10px] text-zinc-500 w-16 shrink-0">Драйвер</span>
          <div className="flex gap-1 flex-1">
            {DRIVER_OPTIONS.map(d => (
              <button
                key={d.id}
                onClick={() => updateDriver(d.id)}
                className={`text-[10px] px-2 py-0.5 rounded flex-1 ${
                  motors.driver === d.id
                    ? 'bg-primary text-primary-foreground'
                    : 'bg-zinc-800 text-zinc-400 hover:bg-zinc-700'
                }`}
              >
                {d.label}
              </button>
            ))}
          </div>
        </div>

        {/* Motor count */}
        <div className="flex items-center gap-2">
          <span className="text-[10px] text-zinc-500 w-16 shrink-0">Кол-во</span>
          <input
            className={inputCls}
            type="number"
            min={1}
            max={8}
            value={motors.count}
            onChange={e => updateCount(e.target.value)}
          />
        </div>

        {/* Channel mapping */}
        <div className="space-y-1.5 mt-1">
          <div className="grid grid-cols-[60px_1fr_50px_50px] gap-1 text-[10px] text-zinc-500">
            <span>Мотор</span>
            <span>Метка</span>
            <span>IN1</span>
            <span>IN2</span>
          </div>
          {Object.entries(motors.channels).map(([key, ch]) => (
            <div key={key} className="grid grid-cols-[60px_1fr_50px_50px] gap-1 items-center">
              <span className="text-[10px] font-mono text-amber-300">{key}</span>
              <input
                className={inputCls}
                value={ch.label}
                onChange={e => updateLabel(key, e.target.value)}
              />
              <input
                className={inputCls}
                type="number"
                min={0}
                value={ch.in1}
                onChange={e => updateChannel(key, 'in1', e.target.value)}
              />
              <input
                className={inputCls}
                type="number"
                min={0}
                value={ch.in2}
                onChange={e => updateChannel(key, 'in2', e.target.value)}
              />
            </div>
          ))}
        </div>
      </CardContent>
    </Card>
  )
}
