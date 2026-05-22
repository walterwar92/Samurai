import { memo, useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { api } from '@/lib/api'
import type { OdometrySources, OdometrySourceMode, RobotPose } from '@/types/robot'

const MODES: OdometrySourceMode[] = ['wheel', 'imu', 'complementary', 'ekf']

const MODE_LABEL: Record<OdometrySourceMode, string> = {
  wheel: 'Wheel',
  imu: 'IMU',
  complementary: 'α·W+(1-α)·I',
  ekf: 'EKF',
}

const MODE_HINT: Record<OdometrySourceMode, string> = {
  wheel: 'Только колёсная (cmd_vel × scale)',
  imu: 'Только IMU (AccelPositionEstimator)',
  complementary: 'Взвешенная сумма wheel + IMU',
  ekf: 'Калман: wheel + IMU как два измерения',
}

interface Props {
  sources: OdometrySources | null
  pose: RobotPose | null
}

export const OdometryComparePanel = memo(function OdometryComparePanel({
  sources,
  pose,
}: Props) {
  const [pendingMode, setPendingMode] = useState<OdometrySourceMode | null>(null)
  const [alpha, setAlpha] = useState(0.7)
  const [error, setError] = useState<string | null>(null)

  const activeMode: OdometrySourceMode = sources?.source ?? 'wheel'

  const handleSetMode = async (mode: OdometrySourceMode) => {
    setPendingMode(mode)
    setError(null)
    try {
      await api.setOdometrySource(mode)
    } catch (e) {
      setError(`Не удалось переключить: ${(e as Error).message}`)
    } finally {
      setPendingMode(null)
    }
  }

  const handleAlphaChange = async (newAlpha: number) => {
    setAlpha(newAlpha)
    try {
      await api.setOdometryAlpha(newAlpha)
    } catch (e) {
      setError(`Не удалось обновить alpha: ${(e as Error).message}`)
    }
  }

  const fmt = (v: number | undefined | null) =>
    v === undefined || v === null || Number.isNaN(v) ? '—' : v.toFixed(3)

  const xW = sources?.x_wheel ?? 0
  const yW = sources?.y_wheel ?? 0
  const xI = sources?.x_imu ?? 0
  const yI = sources?.y_imu ?? 0
  const xF = pose?.x ?? 0
  const yF = pose?.y ?? 0

  const dx = xW - xI
  const dy = yW - yI
  const delta = Math.sqrt(dx * dx + dy * dy)

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold flex items-center justify-between">
          <span>Одометрия — сравнение источников</span>
          <span className="text-[10px] normal-case tracking-normal text-muted-foreground/80">
            активный: <span className="text-foreground font-medium">{MODE_LABEL[activeMode]}</span>
          </span>
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3 px-3 pb-3">
        {/* Mode switcher */}
        <div className="flex gap-1 flex-wrap">
          {MODES.map((m) => {
            const isActive = activeMode === m
            const isPending = pendingMode === m
            return (
              <Button
                key={m}
                size="sm"
                variant={isActive ? 'default' : 'outline'}
                disabled={isPending}
                onClick={() => handleSetMode(m)}
                title={MODE_HINT[m]}
                className="text-[11px] px-2 h-7"
              >
                {isPending ? '…' : MODE_LABEL[m]}
              </Button>
            )
          })}
        </div>

        {/* Alpha slider — only for complementary mode */}
        {activeMode === 'complementary' && (
          <div className="space-y-1">
            <div className="flex items-center justify-between text-[11px]">
              <span className="text-muted-foreground">α (wheel weight)</span>
              <span className="font-mono">{alpha.toFixed(2)}</span>
            </div>
            <input
              type="range"
              min={0}
              max={1}
              step={0.05}
              value={alpha}
              onChange={(e) => handleAlphaChange(parseFloat(e.target.value))}
              className="w-full"
            />
            <div className="flex justify-between text-[9px] text-muted-foreground/70">
              <span>IMU</span>
              <span>50/50</span>
              <span>Wheel</span>
            </div>
          </div>
        )}

        {/* Per-source readings */}
        <div className="grid grid-cols-[repeat(3,minmax(0,1fr))] gap-2 text-[11px] font-mono">
          <SourceCol label="WHEEL" x={xW} y={yW} accent="text-amber-300" />
          <SourceCol label="IMU" x={xI} y={yI} accent="text-cyan-300" />
          <SourceCol label="FUSED" x={xF} y={yF} accent="text-emerald-300" />
        </div>

        {/* Delta — pure diagnostic, the "ошибка" between estimators */}
        <div className="text-[11px] text-muted-foreground flex items-center justify-between border-t border-border/50 pt-2">
          <span>|wheel − imu|</span>
          <span className={`font-mono ${delta > 0.10 ? 'text-rose-400' : delta > 0.03 ? 'text-amber-300' : 'text-emerald-300'}`}>
            {fmt(delta)} m
          </span>
        </div>

        {sources?.stationary_imu !== undefined && (
          <div className="text-[10px] text-muted-foreground flex items-center justify-between">
            <span>ZUPT (IMU)</span>
            <span>{sources.stationary_imu ? 'STATIONARY' : 'moving'}</span>
          </div>
        )}

        {error && (
          <div className="text-[10px] text-rose-400 break-words">{error}</div>
        )}
      </CardContent>
    </Card>
  )
})


function SourceCol({
  label, x, y, accent,
}: {
  label: string
  x: number
  y: number
  accent: string
}) {
  return (
    <div className="space-y-0.5">
      <div className={`text-[9px] uppercase tracking-wider ${accent}`}>{label}</div>
      <div>x: {x.toFixed(3)}</div>
      <div>y: {y.toFixed(3)}</div>
    </div>
  )
}
