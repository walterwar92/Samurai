import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Separator } from '@/components/ui/separator'
import type { HardwarePreset, ServoDriver } from '@/types/hardware'

const DRIVER_OPTIONS: { id: ServoDriver; label: string }[] = [
  { id: 'pca9685', label: 'PCA9685' },
  { id: 'direct_gpio', label: 'GPIO' },
  { id: 'custom', label: 'Custom' },
]

interface ServoBlockProps {
  servos: HardwarePreset['servos']
  onChange: (servos: HardwarePreset['servos']) => void
}

const inputCls = 'w-full bg-zinc-900 border border-zinc-700 rounded px-2 py-1 text-xs font-mono text-zinc-200 focus:outline-none focus:border-amber-500'

export function ServoBlock({ servos, onChange }: ServoBlockProps) {
  const updateHead = (field: string, val: string) => {
    const num = parseInt(val)
    if (isNaN(num)) return
    onChange({
      ...servos,
      head: { ...servos.head, [field]: num },
    })
  }

  const updateArmChannel = (idx: number, val: string) => {
    const num = parseInt(val)
    if (isNaN(num)) return
    const channels = [...servos.arm.channels]
    channels[idx] = num
    onChange({
      ...servos,
      arm: { ...servos.arm, channels },
    })
  }

  const updateArmAngles = (field: 'home_angles' | 'min_angles' | 'max_angles', idx: number, val: string) => {
    const num = parseInt(val)
    if (isNaN(num)) return
    const arr = [...servos.arm[field]]
    arr[idx] = num
    onChange({
      ...servos,
      arm: { ...servos.arm, [field]: arr },
    })
  }

  const updateArmLabel = (idx: number, val: string) => {
    const labels = [...servos.arm.labels]
    labels[idx] = val
    onChange({
      ...servos,
      arm: { ...servos.arm, labels },
    })
  }

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Сервоприводы
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2.5">
        {/* Driver */}
        <div className="flex items-center gap-2">
          <span className="text-[10px] text-zinc-500 w-16 shrink-0">Драйвер</span>
          <div className="flex gap-1 flex-1">
            {DRIVER_OPTIONS.map(d => (
              <button
                key={d.id}
                onClick={() => onChange({ ...servos, driver: d.id })}
                className={`text-[10px] px-2 py-0.5 rounded flex-1 ${
                  servos.driver === d.id
                    ? 'bg-primary text-primary-foreground'
                    : 'bg-zinc-800 text-zinc-400 hover:bg-zinc-700'
                }`}
              >
                {d.label}
              </button>
            ))}
          </div>
        </div>

        {/* Head servo */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1">
            Голова (камера)
          </span>
          <div className="grid grid-cols-4 gap-1.5">
            {(['channel', 'home', 'min', 'max'] as const).map(f => (
              <div key={f}>
                <span className="text-[9px] text-zinc-500 block mb-0.5">
                  {f === 'channel' ? 'CH' : f === 'home' ? 'Home' : f === 'min' ? 'Min' : 'Max'}
                </span>
                <input
                  className={inputCls}
                  type="number"
                  value={servos.head[f]}
                  onChange={e => updateHead(f, e.target.value)}
                />
              </div>
            ))}
          </div>
        </div>

        <Separator />

        {/* Arm servos */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1">
            Рука ({servos.arm.channels.length} сустава)
          </span>
          <div className="space-y-1">
            <div className="grid grid-cols-[1fr_40px_40px_40px_40px] gap-1 text-[9px] text-zinc-500">
              <span>Метка</span>
              <span>CH</span>
              <span>Home</span>
              <span>Min</span>
              <span>Max</span>
            </div>
            {servos.arm.channels.map((ch, i) => (
              <div key={i} className="grid grid-cols-[1fr_40px_40px_40px_40px] gap-1 items-center">
                <input
                  className={inputCls}
                  value={servos.arm.labels[i] || ''}
                  onChange={e => updateArmLabel(i, e.target.value)}
                />
                <input
                  className={inputCls}
                  type="number"
                  value={ch}
                  onChange={e => updateArmChannel(i, e.target.value)}
                />
                <input
                  className={inputCls}
                  type="number"
                  value={servos.arm.home_angles[i]}
                  onChange={e => updateArmAngles('home_angles', i, e.target.value)}
                />
                <input
                  className={inputCls}
                  type="number"
                  value={servos.arm.min_angles[i]}
                  onChange={e => updateArmAngles('min_angles', i, e.target.value)}
                />
                <input
                  className={inputCls}
                  type="number"
                  value={servos.arm.max_angles[i]}
                  onChange={e => updateArmAngles('max_angles', i, e.target.value)}
                />
              </div>
            ))}
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
