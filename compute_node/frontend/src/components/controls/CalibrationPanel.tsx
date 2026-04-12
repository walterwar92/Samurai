import { useState, useEffect } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Separator } from '@/components/ui/separator'
import { api } from '@/lib/api'

interface CalibrationCoeffs {
  profile: string
  scale_fwd: number
  scale_bwd: number
  motor_trim: number
}

interface ProfileEntry {
  scale_fwd: number
  scale_bwd: number
  motor_trim: number
  description: string
}

interface CalibrationPanelProps {
  coeffs: CalibrationCoeffs | null
  profiles: {
    profiles: Record<string, ProfileEntry>
    active: string
  } | null
}

export function CalibrationPanel({ coeffs, profiles }: CalibrationPanelProps) {
  const [fwd, setFwd] = useState('')
  const [bwd, setBwd] = useState('')
  const [trim, setTrim] = useState('')
  const [saveName, setSaveName] = useState('')
  const [saveDesc, setSaveDesc] = useState('')
  const [editing, setEditing] = useState(false)
  const [showSave, setShowSave] = useState(false)

  // Sync inputs from live coefficients
  useEffect(() => {
    if (coeffs && !editing) {
      setFwd(coeffs.scale_fwd.toFixed(4))
      setBwd(coeffs.scale_bwd.toFixed(4))
      setTrim(coeffs.motor_trim.toFixed(3))
    }
  }, [coeffs, editing])

  // Request profile list on mount
  useEffect(() => {
    api.listCalibrationProfiles()
  }, [])

  const handleApply = () => {
    const f = parseFloat(fwd)
    const b = parseFloat(bwd)
    const t = parseFloat(trim)
    if (!isNaN(f) && !isNaN(b) && !isNaN(t)) {
      api.setCalibration(f, b, t)
      setEditing(false)
    }
  }

  const handleSave = () => {
    if (saveName.trim()) {
      api.saveCalibrationProfile(saveName.trim(), saveDesc.trim())
      setSaveName('')
      setSaveDesc('')
      setShowSave(false)
      // Refresh profile list
      setTimeout(() => api.listCalibrationProfiles(), 500)
    }
  }

  const handleLoad = (name: string) => {
    api.loadCalibrationProfile(name)
    setEditing(false)
    setTimeout(() => api.listCalibrationProfiles(), 500)
  }

  const handleDelete = (name: string) => {
    api.deleteCalibrationProfile(name)
    setTimeout(() => api.listCalibrationProfiles(), 500)
  }

  const profileList = profiles?.profiles ?? {}
  const activeName = coeffs?.profile ?? profiles?.active ?? ''
  const inputCls = 'w-full bg-zinc-900 border border-zinc-700 rounded px-2 py-1 text-xs font-mono text-zinc-200 focus:outline-none focus:border-amber-500'

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Калибровка колёс
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2.5">
        {/* Active profile indicator */}
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-1.5">
            <div className="w-2 h-2 rounded-full bg-emerald-500" />
            <span className="text-xs text-zinc-300">
              Профиль: <span className="text-amber-300 font-medium">{activeName || '—'}</span>
            </span>
          </div>
          <button
            onClick={() => api.listCalibrationProfiles()}
            className="text-[10px] text-zinc-500 hover:text-zinc-300"
            title="Обновить"
          >
            ↻
          </button>
        </div>

        {/* Coefficients display/edit */}
        <div className="space-y-1.5">
          <div className="flex items-center gap-2">
            <span className="text-[10px] text-zinc-500 w-16 shrink-0">FWD</span>
            <input
              className={inputCls}
              value={fwd}
              onChange={e => { setFwd(e.target.value); setEditing(true) }}
              placeholder="1.235"
            />
          </div>
          <div className="flex items-center gap-2">
            <span className="text-[10px] text-zinc-500 w-16 shrink-0">BWD</span>
            <input
              className={inputCls}
              value={bwd}
              onChange={e => { setBwd(e.target.value); setEditing(true) }}
              placeholder="0.988"
            />
          </div>
          <div className="flex items-center gap-2">
            <span className="text-[10px] text-zinc-500 w-16 shrink-0">TRIM %</span>
            <input
              className={inputCls}
              value={trim}
              onChange={e => { setTrim(e.target.value); setEditing(true) }}
              placeholder="-12.003"
            />
          </div>
        </div>

        {/* Apply / Save buttons */}
        <div className="flex gap-1.5">
          <Button
            size="sm"
            variant={editing ? 'default' : 'outline'}
            className="text-xs h-7 px-2.5 flex-1"
            onClick={handleApply}
            disabled={!editing}
          >
            Применить
          </Button>
          <Button
            size="sm"
            variant="outline"
            className="text-xs h-7 px-2.5 flex-1"
            onClick={() => setShowSave(!showSave)}
          >
            {showSave ? 'Отмена' : 'Сохранить как...'}
          </Button>
        </div>

        {/* Save dialog */}
        {showSave && (
          <div className="bg-zinc-800/60 rounded p-2 space-y-1.5">
            <input
              className={inputCls}
              value={saveName}
              onChange={e => setSaveName(e.target.value)}
              placeholder="Имя профиля (напр. плитка)"
            />
            <input
              className={inputCls}
              value={saveDesc}
              onChange={e => setSaveDesc(e.target.value)}
              placeholder="Описание (необязательно)"
            />
            <Button
              size="sm"
              className="text-xs h-7 w-full"
              onClick={handleSave}
              disabled={!saveName.trim()}
            >
              Сохранить профиль
            </Button>
          </div>
        )}

        <Separator />

        {/* Profile list */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
            Профили
          </span>
          {Object.keys(profileList).length === 0 ? (
            <span className="text-[10px] text-zinc-600">Нет сохранённых профилей</span>
          ) : (
            <div className="space-y-1">
              {Object.entries(profileList).map(([name, p]) => (
                <div
                  key={name}
                  className={`flex items-center justify-between p-1.5 rounded text-xs ${
                    name === activeName
                      ? 'bg-amber-900/30 border border-amber-700/50'
                      : 'bg-zinc-800/40 border border-zinc-700/30'
                  }`}
                >
                  <div className="min-w-0 flex-1">
                    <div className="flex items-center gap-1">
                      {name === activeName && (
                        <span className="text-[8px] text-amber-400">●</span>
                      )}
                      <span className="font-medium text-zinc-200 truncate">{name}</span>
                    </div>
                    <div className="text-[10px] text-zinc-500 font-mono">
                      F:{p.scale_fwd.toFixed(3)} B:{p.scale_bwd.toFixed(3)} T:{p.motor_trim.toFixed(1)}%
                    </div>
                    {p.description && (
                      <div className="text-[10px] text-zinc-600 truncate">{p.description}</div>
                    )}
                  </div>
                  <div className="flex gap-1 ml-1.5 shrink-0">
                    {name !== activeName && (
                      <button
                        onClick={() => handleLoad(name)}
                        className="text-[10px] px-1.5 py-0.5 rounded bg-zinc-700 text-zinc-300 hover:bg-zinc-600"
                      >
                        Загр.
                      </button>
                    )}
                    <button
                      onClick={() => handleDelete(name)}
                      className="text-[10px] px-1.5 py-0.5 rounded bg-red-900/40 text-red-400 hover:bg-red-800/50"
                    >
                      ✕
                    </button>
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  )
}
