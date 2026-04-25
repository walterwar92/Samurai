import { useEffect, useState, useCallback } from 'react'
import { Bookmark, Download, Play, Save } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { cn } from '@/lib/utils'
import type { SamcanApi, SamcanPresets } from '@/hooks/useSamcanState'

interface PresetManagerProps {
  api: SamcanApi
  /** Live telemetry (текущие углы из Arduino). */
  liveArm?: number
  liveBase?: number
  liveClaw?: number
  /** Текущие значения слайдеров — если «live» не достоверен. */
  sliderArm: number
  sliderBase: number
  disabled?: boolean
}

/**
 * Панель управления пресетами: park / forward / claw_open / claw_closed.
 * Сохраняет в JSON файл на сервере через /api/samcan/preset/save.
 * Кнопка «Apply» применяет через /api/samcan/preset/apply.
 */
export function PresetManager({
  api, liveArm, liveBase, liveClaw, sliderArm, sliderBase, disabled,
}: PresetManagerProps) {
  const [presets, setPresets] = useState<SamcanPresets>({})
  const [status, setStatus] = useState<string>('')

  const refresh = useCallback(async () => {
    setPresets(await api.getPresets())
  }, [api])

  useEffect(() => { refresh() }, [refresh])

  const currentArm  = liveArm  ?? sliderArm
  const currentBase = liveBase ?? sliderBase
  const currentClaw = liveClaw ?? 90

  const flash = (msg: string) => {
    setStatus(msg)
    setTimeout(() => setStatus(''), 1800)
  }

  const savePose = async (name: 'park' | 'forward', withClaw: boolean) => {
    const fields: { base: number; arm: number; claw?: number } = {
      base: currentBase, arm: currentArm,
    }
    if (withClaw) fields.claw = currentClaw
    await api.savePreset(name, fields)
    flash(`Сохранено: ${name.toUpperCase()}`)
    refresh()
  }

  const saveClaw = async (name: 'claw_open' | 'claw_closed') => {
    await api.savePreset(name, { claw: currentClaw })
    flash(`Сохранено: ${name === 'claw_open' ? 'OPEN' : 'CLOSED'}`)
    refresh()
  }

  const apply = async (name: 'park' | 'forward' | 'grab') => {
    await api.applyPreset(name)
    flash(`Применено: ${name.toUpperCase()}`)
  }

  const park = presets.park || {}
  const fwd  = presets.forward || {}

  return (
    <div className="space-y-2.5">
      <div className="flex items-center gap-2">
        <Bookmark className="w-3 h-3 text-primary" />
        <span className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Сохранённые позы
        </span>
        {status && (
          <span className="text-[10px] text-emerald-400 ml-auto">{status}</span>
        )}
      </div>

      <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
        <PresetCard
          title="PARK"
          subtitle="безопасная поза (рука поднята)"
          values={[
            { k: 'base', v: park.base },
            { k: 'arm',  v: park.arm },
            { k: 'claw', v: park.claw },
          ]}
          current={{ base: currentBase, arm: currentArm, claw: currentClaw }}
          onSave={() => savePose('park', true)}
          onApply={() => apply('park')}
          disabled={disabled}
          accent="sky"
        />
        <PresetCard
          title="FORWARD"
          subtitle="рука вперёд для захвата"
          values={[
            { k: 'base', v: fwd.base },
            { k: 'arm',  v: fwd.arm },
          ]}
          current={{ base: currentBase, arm: currentArm }}
          onSave={() => savePose('forward', false)}
          onApply={() => apply('forward')}
          disabled={disabled}
          accent="emerald"
        />
      </div>

      <div className="grid grid-cols-2 gap-2">
        <ClawCard
          label="CLAW_OPEN"
          current={presets.claw_open}
          onSave={() => saveClaw('claw_open')}
          disabled={disabled}
          hintCurrent={currentClaw}
          colour="rgb(34 197 94)"
        />
        <ClawCard
          label="CLAW_CLOSED"
          current={presets.claw_closed}
          onSave={() => saveClaw('claw_closed')}
          disabled={disabled}
          hintCurrent={currentClaw}
          colour="rgb(234 179 8)"
        />
      </div>

      <Button
        size="sm"
        variant="default"
        className="w-full text-xs bg-primary/90 hover:bg-primary"
        disabled={disabled}
        onClick={() => apply('grab')}
      >
        <Play className="w-3 h-3 mr-1" />
        GRAB с текущими пресетами
      </Button>

      <div className="text-[9px] text-muted-foreground">
        Пресеты хранятся на сервере в <code>samcan_presets.json</code> —
        не сбрасываются при перезапуске Arduino или bridge.
      </div>
    </div>
  )
}

/* ─── helpers ─── */

interface KV { k: string; v: number | undefined }

function PresetCard({
  title, subtitle, values, current, onSave, onApply, disabled, accent,
}: {
  title: string
  subtitle: string
  values: KV[]
  current: Record<string, number>
  onSave: () => void
  onApply: () => void
  disabled?: boolean
  accent: 'sky' | 'emerald'
}) {
  const accentRing = accent === 'sky' ? 'border-sky-500/40' : 'border-emerald-500/40'
  const accentText = accent === 'sky' ? 'text-sky-300'       : 'text-emerald-300'
  return (
    <div className={cn('rounded-md border bg-zinc-900/40 p-2.5 space-y-2', accentRing)}>
      <div>
        <div className={cn('text-xs font-bold tracking-wider', accentText)}>{title}</div>
        <div className="text-[10px] text-muted-foreground">{subtitle}</div>
      </div>
      <div className="grid grid-cols-3 gap-1 text-[10px] font-mono">
        {values.map(({ k, v }) => (
          <div key={k} className="flex flex-col items-center bg-zinc-950/60 rounded px-1 py-1">
            <span className="text-[8px] uppercase text-muted-foreground">{k}</span>
            <span className="text-zinc-200">{v ?? '—'}°</span>
            {v !== undefined && current[k] !== undefined && v !== current[k] && (
              <span className="text-[8px] text-amber-400">live {current[k]}°</span>
            )}
          </div>
        ))}
      </div>
      <div className="flex gap-1.5">
        <Button
          size="sm" variant="outline" className="flex-1 h-7 text-[10px]"
          disabled={disabled}
          onClick={onSave}
        >
          <Save className="w-3 h-3 mr-1" />
          Сохранить
        </Button>
        <Button
          size="sm" variant="default" className="flex-1 h-7 text-[10px]"
          disabled={disabled}
          onClick={onApply}
        >
          <Download className="w-3 h-3 mr-1" />
          Применить
        </Button>
      </div>
    </div>
  )
}

function ClawCard({
  label, current, onSave, disabled, hintCurrent, colour,
}: {
  label: string
  current: number | undefined
  onSave: () => void
  disabled?: boolean
  hintCurrent: number
  colour: string
}) {
  return (
    <div className="rounded-md border border-border/60 bg-zinc-900/40 p-2 flex items-center gap-2">
      <div className="w-2.5 h-2.5 rounded-full flex-none" style={{ background: colour }} />
      <div className="flex-1 min-w-0">
        <div className="text-[10px] font-semibold text-zinc-200 tracking-wider">{label}</div>
        <div className="text-[9px] font-mono text-muted-foreground">
          сохр: {current ?? '—'}° · live: {hintCurrent}°
        </div>
      </div>
      <Button
        size="sm" variant="outline" className="h-7 text-[10px] px-2"
        disabled={disabled}
        onClick={onSave}
      >
        <Save className="w-3 h-3 mr-1" /> Save
      </Button>
    </div>
  )
}
