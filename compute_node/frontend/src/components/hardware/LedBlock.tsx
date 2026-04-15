import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { HardwarePreset, LedType } from '@/types/hardware'

const inputCls = 'w-full bg-zinc-900 border border-zinc-700 rounded px-2 py-1 text-xs font-mono text-zinc-200 focus:outline-none focus:border-amber-500'

interface LedBlockProps {
  leds: HardwarePreset['leds']
  onChange: (leds: HardwarePreset['leds']) => void
}

const LED_TYPES: { id: LedType; label: string }[] = [
  { id: 'ws2812b', label: 'WS2812B' },
  { id: 'apa102', label: 'APA102' },
  { id: 'none', label: 'Нет' },
]

export function LedBlock({ leds, onChange }: LedBlockProps) {
  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Светодиоды
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2">
        <div className="flex items-center gap-2">
          <span className="text-[10px] text-zinc-500 w-14 shrink-0">Тип</span>
          <div className="flex gap-1 flex-1">
            {LED_TYPES.map(t => (
              <button
                key={t.id}
                onClick={() => onChange({ ...leds, type: t.id })}
                className={`text-[10px] px-2 py-0.5 rounded flex-1 ${
                  leds.type === t.id
                    ? 'bg-primary text-primary-foreground'
                    : 'bg-zinc-800 text-zinc-400 hover:bg-zinc-700'
                }`}
              >
                {t.label}
              </button>
            ))}
          </div>
        </div>

        {leds.type !== 'none' && (
          <div className="grid grid-cols-3 gap-1.5">
            <div>
              <span className="text-[9px] text-zinc-500">GPIO</span>
              <input
                className={inputCls}
                type="number"
                value={leds.gpio_pin}
                onChange={e => onChange({ ...leds, gpio_pin: parseInt(e.target.value) || 0 })}
              />
            </div>
            <div>
              <span className="text-[9px] text-zinc-500">Кол-во</span>
              <input
                className={inputCls}
                type="number"
                value={leds.count}
                onChange={e => onChange({ ...leds, count: parseInt(e.target.value) || 1 })}
              />
            </div>
            <div>
              <span className="text-[9px] text-zinc-500">Яркость</span>
              <input
                className={inputCls}
                type="number"
                step={0.05}
                min={0}
                max={1}
                value={leds.brightness}
                onChange={e => onChange({ ...leds, brightness: parseFloat(e.target.value) || 0.3 })}
              />
            </div>
          </div>
        )}
      </CardContent>
    </Card>
  )
}
