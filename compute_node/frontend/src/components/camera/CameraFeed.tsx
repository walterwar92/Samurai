import { useState, useEffect, useRef, useCallback } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

type StreamMode = 'ws' | 'mjpeg'

export function CameraFeed() {
  const [mode, setMode] = useState<StreamMode>('ws')
  const [hasError, setHasError] = useState(false)
  const [fps, setFps] = useState(0)
  const imgRef = useRef<HTMLImageElement>(null)
  const wsRef = useRef<WebSocket | null>(null)
  const frameCountRef = useRef(0)
  const blobUrlRef = useRef<string | null>(null)

  const cleanup = useCallback(() => {
    if (wsRef.current) {
      wsRef.current.close()
      wsRef.current = null
    }
    if (blobUrlRef.current) {
      URL.revokeObjectURL(blobUrlRef.current)
      blobUrlRef.current = null
    }
  }, [])

  // WebSocket binary frame receiver
  useEffect(() => {
    if (mode !== 'ws') {
      cleanup()
      return
    }

    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const wsUrl = `${proto}//${window.location.host}/ws/camera`
    let ws: WebSocket

    try {
      ws = new WebSocket(wsUrl)
      ws.binaryType = 'arraybuffer'
    } catch {
      setMode('mjpeg')
      return
    }

    wsRef.current = ws

    ws.onopen = () => {
      setHasError(false)
    }

    ws.onmessage = (event) => {
      if (!imgRef.current) return
      // Revoke previous blob URL to prevent memory leak
      if (blobUrlRef.current) {
        URL.revokeObjectURL(blobUrlRef.current)
      }
      const blob = new Blob([event.data], { type: 'image/jpeg' })
      const url = URL.createObjectURL(blob)
      blobUrlRef.current = url
      imgRef.current.src = url
      frameCountRef.current++
    }

    ws.onerror = () => {
      // Fallback to MJPEG
      setMode('mjpeg')
    }

    ws.onclose = () => {
      // Try reconnect after 2s
      setTimeout(() => {
        if (mode === 'ws') {
          setMode('mjpeg') // fallback if WS keeps failing
        }
      }, 2000)
    }

    // FPS counter
    const fpsInterval = setInterval(() => {
      setFps(frameCountRef.current)
      frameCountRef.current = 0
    }, 1000)

    return () => {
      clearInterval(fpsInterval)
      cleanup()
    }
  }, [mode, cleanup])

  const toggleMode = () => {
    setHasError(false)
    setMode(prev => prev === 'ws' ? 'mjpeg' : 'ws')
  }

  return (
    <Card>
      <CardHeader className="py-2 px-3 flex flex-row items-center justify-between">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Камера + YOLO
        </CardTitle>
        <div className="flex items-center gap-2">
          {mode === 'ws' && fps > 0 && (
            <span className="text-[10px] text-zinc-500 font-mono">{fps} fps</span>
          )}
          <button
            onClick={toggleMode}
            className="text-[10px] px-1.5 py-0.5 rounded bg-zinc-800 border border-zinc-600 text-zinc-400 hover:text-zinc-200 transition-colors"
          >
            {mode === 'ws' ? 'WS' : 'MJPEG'}
          </button>
        </div>
      </CardHeader>
      <CardContent className="p-0">
        {hasError ? (
          <div className="flex items-center justify-center h-48 bg-black/50 text-muted-foreground text-sm">
            Нет видеопотока
          </div>
        ) : mode === 'ws' ? (
          <img
            ref={imgRef}
            alt="Camera Feed (WS)"
            className="w-full h-auto block bg-black min-h-[200px]"
            onError={() => setHasError(true)}
          />
        ) : (
          <img
            src="/video_feed"
            alt="Camera Feed (MJPEG)"
            className="w-full h-auto block bg-black min-h-[200px]"
            onError={() => setHasError(true)}
          />
        )}
      </CardContent>
    </Card>
  )
}
