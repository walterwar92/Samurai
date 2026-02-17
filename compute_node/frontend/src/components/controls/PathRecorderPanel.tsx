import { useState, useEffect } from 'react'
import { Circle, Square, Play, Save, FolderOpen } from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { api } from '@/lib/api'

interface PathRecorderPanelProps {
  pathRecorder: { state: string; points_count: number; current_idx: number } | null
}

export function PathRecorderPanel({ pathRecorder }: PathRecorderPanelProps) {
  const state = pathRecorder?.state ?? 'idle'
  const [pathName, setPathName] = useState('')
  const [paths, setPaths] = useState<string[]>([])

  const loadPathList = async () => {
    try {
      const data = await api.listPaths()
      if (data?.paths) setPaths(data.paths)
    } catch {}
  }

  useEffect(() => {
    loadPathList()
  }, [])

  const handleSave = () => {
    if (pathName.trim()) {
      api.pathRecorderCommand(`save:${pathName.trim()}`)
      setPathName('')
      setTimeout(loadPathList, 500)
    }
  }

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Запись пути
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2">
        <div className="flex gap-1.5">
          <Button
            size="sm"
            variant={state === 'recording' ? 'destructive' : 'outline'}
            className="flex-1 h-7 text-[10px]"
            onClick={() => api.pathRecorderCommand(state === 'recording' ? 'stop' : 'record')}
          >
            {state === 'recording' ? <Square size={10} className="mr-1" /> : <Circle size={10} className="mr-1" />}
            {state === 'recording' ? 'Стоп' : 'Запись'}
          </Button>
          <Button
            size="sm"
            variant={state === 'replaying' ? 'default' : 'outline'}
            className="flex-1 h-7 text-[10px]"
            onClick={() => api.pathRecorderCommand('replay')}
            disabled={state === 'recording'}
          >
            <Play size={10} className="mr-1" /> Воспроизвести
          </Button>
        </div>

        {pathRecorder && (
          <div className="text-xs text-muted-foreground">
            {state === 'recording' && `Запись... ${pathRecorder.points_count} точек`}
            {state === 'replaying' && `Воспроизведение ${pathRecorder.current_idx + 1}/${pathRecorder.points_count}`}
            {state === 'idle' && pathRecorder.points_count > 0 && `${pathRecorder.points_count} точек в буфере`}
          </div>
        )}

        <div className="flex gap-1.5">
          <Input
            placeholder="Имя пути..."
            value={pathName}
            onChange={(e) => setPathName(e.target.value)}
            className="h-7 text-xs"
            onKeyDown={(e) => e.key === 'Enter' && handleSave()}
          />
          <Button size="sm" className="h-7 px-2" onClick={handleSave}>
            <Save size={12} />
          </Button>
        </div>

        {paths.length > 0 && (
          <div className="space-y-1">
            <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
              Сохранённые ({paths.length})
            </span>
            {paths.map((name) => (
              <div key={name} className="flex items-center gap-1.5 text-[10px]">
                <span className="font-mono truncate flex-1">{name}</span>
                <button
                  className="text-muted-foreground hover:text-cyan-400"
                  onClick={() => api.pathRecorderCommand(`load:${name}`)}
                  title="Загрузить"
                >
                  <FolderOpen size={10} />
                </button>
              </div>
            ))}
          </div>
        )}
      </CardContent>
    </Card>
  )
}
