import { useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

export function CameraFeed() {
  const [hasError, setHasError] = useState(false)

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Камера + YOLO
        </CardTitle>
      </CardHeader>
      <CardContent className="p-0">
        {hasError ? (
          <div className="flex items-center justify-center h-48 bg-black/50 text-muted-foreground text-sm">
            Нет видеопотока
          </div>
        ) : (
          <img
            src="/video_feed"
            alt="Camera Feed"
            className="w-full h-auto block bg-black min-h-[200px]"
            onError={() => setHasError(true)}
          />
        )}
      </CardContent>
    </Card>
  )
}
