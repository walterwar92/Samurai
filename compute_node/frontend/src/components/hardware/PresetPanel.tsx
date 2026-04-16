import { useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Separator } from '@/components/ui/separator'
import { api } from '@/lib/api'
import type { PresetSummary } from '@/types/hardware'

interface PresetPanelProps {
  active: string
  onLoad: (name: string) => void
  onRefresh: () => void
  presets: PresetSummary[]
  dirty: boolean
  currentName: string
}

export function PresetPanel({
  active, onLoad, onRefresh, presets, dirty, currentName,
}: PresetPanelProps) {
  const [saveName, setSaveName] = useState('')
  const [saveDesc, setSaveDesc] = useState('')
  const [showSave, setShowSave] = useState(false)

  const inputCls =
    'w-full bg-zinc-900 border border-zinc-700 rounded px-2 py-1 text-xs font-mono text-zinc-200 focus:outline-none focus:border-amber-500'

  const emitSave = (name: string, description: string) => {
    window.dispatchEvent(
      new CustomEvent('hw-preset-save', {
        detail: { name, description },
      }),
    )
  }

  const handleSaveAs = () => {
    if (!saveName.trim()) return
    emitSave(saveName.trim(), saveDesc.trim())
    setSaveName('')
    setSaveDesc('')
    setShowSave(false)
  }

  const handleQuickSave = () => {
    // Re-save the currently loaded preset under the same name.
    if (!currentName) {
      setShowSave(true)
      return
    }
    emitSave(currentName, '')
  }

  const handleDelete = async (name: string) => {
    if (!confirm(`Удалить пресет «${name}»?`)) return
    await api.deleteHardwarePreset(name)
    onRefresh()
  }

  const handleApply = async (name: string) => {
    await api.applyHardwarePreset(name)
    onRefresh()
  }

  const nameExists = presets.some((p) => p.name === saveName.trim())

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Пресеты оборудования
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2.5">
        {/* Active indicator */}
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-1.5">
            <div
              className={`w-2 h-2 rounded-full ${
                active ? 'bg-emerald-500' : 'bg-zinc-600'
              }`}
            />
            <span className="text-xs text-zinc-300">
              Активный:{' '}
              <span className="text-amber-300 font-medium">
                {active || '—'}
              </span>
            </span>
          </div>
          <button
            onClick={onRefresh}
            className="text-[10px] text-zinc-500 hover:text-zinc-300"
            title="Обновить список"
          >
            ↻
          </button>
        </div>

        {dirty && (
          <div className="text-[10px] text-amber-400 bg-amber-900/20 rounded px-2 py-1">
            Есть несохранённые изменения
          </div>
        )}

        {/* Save buttons */}
        <div className="flex gap-1.5">
          <Button
            size="sm"
            variant="outline"
            className="text-xs h-7 px-2.5 flex-1"
            onClick={handleQuickSave}
            disabled={!currentName || !dirty}
            title={currentName ? `Сохранить в «${currentName}»` : 'Сначала выберите имя'}
          >
            💾 Сохранить
          </Button>
          <Button
            size="sm"
            variant="outline"
            className="text-xs h-7 px-2.5 flex-1"
            onClick={() => {
              setShowSave(!showSave)
              if (!showSave) setSaveName(currentName || '')
            }}
          >
            {showSave ? 'Отмена' : 'Сохранить как...'}
          </Button>
        </div>

        {showSave && (
          <div className="bg-zinc-800/60 rounded p-2 space-y-1.5">
            <input
              className={inputCls}
              value={saveName}
              onChange={(e) => setSaveName(e.target.value)}
              placeholder="Имя пресета"
              autoFocus
              onKeyDown={(e) => e.key === 'Enter' && handleSaveAs()}
            />
            <input
              className={inputCls}
              value={saveDesc}
              onChange={(e) => setSaveDesc(e.target.value)}
              placeholder="Описание (необязательно)"
              onKeyDown={(e) => e.key === 'Enter' && handleSaveAs()}
            />
            {nameExists && (
              <div className="text-[9px] text-amber-400">
                Пресет с таким именем существует — будет перезаписан
              </div>
            )}
            <Button
              size="sm"
              className="text-xs h-7 w-full"
              onClick={handleSaveAs}
              disabled={!saveName.trim()}
            >
              {nameExists ? 'Перезаписать' : 'Создать пресет'}
            </Button>
          </div>
        )}

        <Separator />

        {/* Preset list */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1.5">
            Все пресеты ({presets.length})
          </span>
          {presets.length === 0 ? (
            <span className="text-[10px] text-zinc-600">
              Нет сохранённых пресетов
            </span>
          ) : (
            <div className="space-y-1">
              {presets.map((p) => (
                <div
                  key={p.name}
                  className={`p-1.5 rounded text-xs ${
                    p.name === active
                      ? 'bg-amber-900/30 border border-amber-700/50'
                      : 'bg-zinc-800/40 border border-zinc-700/30'
                  }`}
                >
                  <div className="flex items-center justify-between mb-0.5">
                    <div className="flex items-center gap-1 min-w-0 flex-1">
                      {p.name === active && (
                        <span className="text-[8px] text-amber-400">●</span>
                      )}
                      <span className="font-medium text-zinc-200 truncate">
                        {p.name}
                      </span>
                    </div>
                  </div>
                  <div className="text-[10px] text-zinc-500 mb-1">
                    {p.platform} {p.description && `— ${p.description}`}
                  </div>
                  <div className="flex gap-1">
                    <button
                      onClick={() => onLoad(p.name)}
                      className="text-[10px] px-1.5 py-0.5 rounded bg-zinc-700 text-zinc-300 hover:bg-zinc-600"
                    >
                      Загрузить
                    </button>
                    {p.name !== active && (
                      <button
                        onClick={() => handleApply(p.name)}
                        className="text-[10px] px-1.5 py-0.5 rounded bg-emerald-900/40 text-emerald-400 hover:bg-emerald-800/50"
                      >
                        Применить
                      </button>
                    )}
                    {p.name === active && (
                      <span className="text-[10px] px-1.5 py-0.5 text-amber-400">
                        Активен
                      </span>
                    )}
                    <button
                      onClick={() => handleDelete(p.name)}
                      className="text-[10px] px-1.5 py-0.5 rounded bg-red-900/40 text-red-400 hover:bg-red-800/50 ml-auto"
                      title="Удалить"
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
