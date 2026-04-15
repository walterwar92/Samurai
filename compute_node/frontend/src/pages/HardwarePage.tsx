import { useState, useEffect, useCallback } from 'react'
import { Header } from '@/components/layout/Header'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Separator } from '@/components/ui/separator'
import { api } from '@/lib/api'
import { emptyPreset } from '@/types/hardware'
import type { HardwarePreset, PresetSummary, Platform } from '@/types/hardware'
import { HardwareBlockDiagram, type BlockId } from '@/components/hardware/HardwareBlockDiagram'
import { PlatformSelector } from '@/components/hardware/PlatformSelector'
import { PresetPanel } from '@/components/hardware/PresetPanel'
import { MotorBlock } from '@/components/hardware/MotorBlock'
import { ServoBlock } from '@/components/hardware/ServoBlock'
import { SensorBlock } from '@/components/hardware/SensorBlock'
import { LedBlock } from '@/components/hardware/LedBlock'

export function HardwarePage() {
  const [preset, setPreset] = useState<HardwarePreset>(emptyPreset())
  const [presets, setPresets] = useState<PresetSummary[]>([])
  const [active, setActive] = useState('')
  const [activeBlock, setActiveBlock] = useState<BlockId | null>(null)
  const [dirty, setDirty] = useState(false)
  const [loadedName, setLoadedName] = useState('')

  // ── Fetch presets list + active ──────────────────────────────────
  const refresh = useCallback(async () => {
    try {
      const data = await api.listHardwarePresets()
      setPresets(data.presets || [])
      setActive(data.active || '')
    } catch { /* backend offline */ }
  }, [])

  // Initial load
  useEffect(() => {
    refresh()
  }, [refresh])

  // Auto-load active preset on first load
  useEffect(() => {
    if (active && !loadedName) {
      loadPreset(active)
    }
  }, [active]) // eslint-disable-line react-hooks/exhaustive-deps

  // ── Load a specific preset ──────────────────────────────────────
  const loadPreset = async (name: string) => {
    try {
      const data = await api.getHardwarePreset(name)
      if (data.preset) {
        // Merge with empty to ensure all fields exist
        setPreset({ ...emptyPreset(), ...data.preset })
        setLoadedName(name)
        setDirty(false)
      }
    } catch { /* */ }
  }

  // ── Save preset (via custom event from PresetPanel) ─────────────
  useEffect(() => {
    const handler = async (e: Event) => {
      const { name, description } = (e as CustomEvent).detail
      const toSave = {
        ...preset,
        name,
        description,
      }
      await api.saveHardwarePreset(toSave)
      setLoadedName(name)
      setDirty(false)
      refresh()
    }
    window.addEventListener('hw-preset-save', handler)
    return () => window.removeEventListener('hw-preset-save', handler)
  }, [preset, refresh])

  // ── Update helpers (mark dirty) ─────────────────────────────────
  const update = <K extends keyof HardwarePreset>(key: K, val: HardwarePreset[K]) => {
    setPreset(prev => ({ ...prev, [key]: val }))
    setDirty(true)
  }

  // ── Render active block editor ──────────────────────────────────
  const renderEditor = () => {
    switch (activeBlock) {
      case 'motors':
        return (
          <MotorBlock
            motors={preset.motors}
            onChange={v => update('motors', v)}
          />
        )
      case 'servos':
        return (
          <ServoBlock
            servos={preset.servos}
            onChange={v => update('servos', v)}
          />
        )
      case 'camera':
      case 'imu':
      case 'range':
        return (
          <SensorBlock
            imu={preset.imu}
            camera={preset.camera}
            range_sensor={preset.range_sensor}
            i2c={preset.i2c}
            onImuChange={v => update('imu', v)}
            onCameraChange={v => update('camera', v)}
            onRangeChange={v => update('range_sensor', v)}
            onI2cChange={v => update('i2c', v)}
          />
        )
      case 'leds':
        return (
          <LedBlock
            leds={preset.leds}
            onChange={v => update('leds', v)}
          />
        )
      default:
        return (
          <Card>
            <CardContent className="p-6 text-center text-xs text-zinc-500">
              Выберите блок на схеме для редактирования
            </CardContent>
          </Card>
        )
    }
  }

  return (
    <div className="min-h-screen">
      <Header />

      <div className="grid grid-cols-1 lg:grid-cols-[1fr_300px] gap-2.5 p-2.5 max-w-[1600px] mx-auto min-h-[calc(100vh-48px)]">
        {/* ── Left: Main content ── */}
        <div className="space-y-2.5">
          {/* Platform selector */}
          <Card>
            <CardHeader className="py-2 px-3">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Платформа
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3">
              <PlatformSelector
                value={preset.platform}
                onChange={p => update('platform', p)}
              />
            </CardContent>
          </Card>

          {/* Block diagram */}
          <Card>
            <CardHeader className="py-2 px-3">
              <div className="flex items-center justify-between">
                <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                  Блок-схема оборудования
                </CardTitle>
                {loadedName && (
                  <span className="text-[10px] text-zinc-500">
                    {loadedName}{dirty && ' *'}
                  </span>
                )}
              </div>
            </CardHeader>
            <CardContent className="p-3">
              <HardwareBlockDiagram
                preset={preset}
                activeBlock={activeBlock}
                onBlockClick={(id) => setActiveBlock(activeBlock === id ? null : id)}
              />
            </CardContent>
          </Card>

          {/* Block editor (expanded) */}
          {renderEditor()}
        </div>

        {/* ── Right sidebar: Presets ── */}
        <div className="lg:row-span-3">
          <ScrollArea className="h-[calc(100vh-60px)]">
            <div className="space-y-2.5 pr-2">
              <PresetPanel
                active={active}
                presets={presets}
                dirty={dirty}
                currentName={loadedName}
                onLoad={loadPreset}
                onRefresh={refresh}
              />

              <Separator />

              {/* Quick summary */}
              <Card>
                <CardHeader className="py-2 px-3">
                  <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                    Сводка
                  </CardTitle>
                </CardHeader>
                <CardContent className="p-3">
                  <div className="space-y-1 text-[10px]">
                    <Row label="Платформа" value={PLATFORM_NAMES[preset.platform] || preset.platform} />
                    <Row label="Моторы" value={`${preset.motors.count} (${preset.motors.driver})`} />
                    <Row label="Серво" value={`${1 + preset.servos.arm.channels.length} шт`} />
                    <Row label="IMU" value={preset.imu.type === 'none' ? 'Нет' : preset.imu.type.toUpperCase()} />
                    <Row label="Камера" value={preset.camera.type === 'none' ? 'Нет' : preset.camera.type} />
                    <Row label="Дальномер" value={preset.range_sensor.type === 'none' ? 'Нет' : preset.range_sensor.type} />
                    <Row label="LED" value={preset.leds.type === 'none' ? 'Нет' : `${preset.leds.count}x ${preset.leds.type}`} />
                    <Row label="PCA9685" value={`${preset.i2c.pca9685_address} @ ${preset.i2c.pca9685_frequency}Hz`} />
                  </div>
                </CardContent>
              </Card>
            </div>
          </ScrollArea>
        </div>
      </div>
    </div>
  )
}

/* ── Helpers ── */

const PLATFORM_NAMES: Record<string, string> = {
  raspberry_pi_pca9685: 'Raspberry Pi + PCA9685',
  arduino: 'Arduino',
  esp32: 'ESP32',
  custom: 'Custom',
}

function Row({ label, value }: { label: string; value: string }) {
  return (
    <div className="flex justify-between">
      <span className="text-zinc-500">{label}</span>
      <span className="text-zinc-300 font-mono">{value}</span>
    </div>
  )
}
