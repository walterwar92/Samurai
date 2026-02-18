import { useState, useEffect } from 'react'
import { Save, FolderOpen, Trash2 } from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { api } from '@/lib/api'

export function MapManagerPanel() {
  const [mapName, setMapName] = useState('')
  const [maps, setMaps] = useState<string[]>([])

  const loadMapList = async () => {
    try {
      const data = await api.listMaps()
      if (data?.maps) setMaps(data.maps)
    } catch {}
  }

  useEffect(() => {
    loadMapList()
  }, [])

  const handleSave = () => {
    if (mapName.trim()) {
      api.saveMap(mapName.trim())
      setMapName('')
      setTimeout(loadMapList, 500)
    }
  }

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Карты
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2">
        <div className="flex gap-1.5">
          <Input
            placeholder="Имя карты..."
            value={mapName}
            onChange={(e) => setMapName(e.target.value)}
            className="h-7 text-xs"
            onKeyDown={(e) => e.key === 'Enter' && handleSave()}
          />
          <Button size="sm" className="h-7 px-2" onClick={handleSave}>
            <Save size={12} />
          </Button>
        </div>

        {maps.length > 0 && (
          <div className="space-y-1">
            <span className="text-[10px] uppercase text-muted-foreground tracking-wider">
              Сохранённые ({maps.length})
            </span>
            {maps.map((name) => (
              <div key={name} className="flex items-center gap-1.5 text-[10px]">
                <span className="font-mono truncate flex-1">{name}</span>
                <button
                  className="text-muted-foreground hover:text-cyan-400"
                  onClick={() => api.loadMap(name)}
                  title="Загрузить"
                >
                  <FolderOpen size={10} />
                </button>
              </div>
            ))}
          </div>
        )}

        <Button size="sm" variant="outline" className="w-full h-7 text-[10px]" onClick={loadMapList}>
          Обновить список
        </Button>
      </CardContent>
    </Card>
  )
}
