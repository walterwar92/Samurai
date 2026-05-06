import { useState, useEffect, useRef, useCallback } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

/**
 * CameraFeed — H.264 поток с робота через WebCodecs API.
 *
 * Pipeline (с 2026-04, заменил MJPEG/JPEG-WS):
 *   WebSocket /ws/h264 (dashboard прокси)
 *     → первое сообщение JSON {codec, width, height, fps}  (только для display)
 *     → бинарные NAL units (Annex B) от Pi camera_node
 *     → NAL parser → парсинг SPS → построение точного avc1.PPCCLL
 *     → VideoDecoder.configure → EncodedVideoChunk → VideoFrame
 *     → canvas drawImage
 *
 * BUGFIX 2026-05: codec НЕ хардкодим. picamera2 H264Encoder на Pi 4
 * (V4L2 M2M / vc4-hw-encode) по умолчанию выдаёт High profile (avc1.640028),
 * а не Main 4.0. Если сконфигурировать decoder под Main, а получить High,
 * Chrome тихо дропает кадры (без error callback). Парсим первый SPS NAL
 * (байты profile_idc / constraint_set / level_idc) и собираем точный
 * codec string динамически.
 *
 * Требования: Chrome 94+, Edge 94+, Android Chrome 94+.
 * Safari/iOS — WebCodecs только в Technology Preview, fallback показывает ошибку.
 */

interface EndpointInfo {
  type: 'endpoint'
  codec: string  // 'h264'
  format: string // 'annex-b'
  width?: number
  height?: number
  fps?: number
}

// Fallback codec string если SPS почему-то не распарсился (Main 4.0).
// В нормальной ситуации мы строим codec из реального SPS, см. parseSpsCodec().
const FALLBACK_CODEC = 'avc1.4D0028'

/**
 * Извлекает avc1.PPCCLL из SPS NAL unit (Annex B).
 *
 * SPS layout (после start code + NAL header byte 0x67):
 *   byte 0: profile_idc            (0x42=Baseline, 0x4D=Main, 0x64=High, ...)
 *   byte 1: constraint_set_flags   (8 bits packed)
 *   byte 2: level_idc              (0x1F=3.1, 0x28=4.0, 0x29=4.1, ...)
 */
function parseSpsCodec(nalWithStart: Uint8Array): string | null {
  let scLen = 3
  if (nalWithStart[0] === 0 && nalWithStart[1] === 0
      && nalWithStart[2] === 0 && nalWithStart[3] === 1) {
    scLen = 4
  }
  // start code (3-4) + NAL header byte (1) + 3 байта SPS = scLen + 4
  if (nalWithStart.byteLength < scLen + 4) return null
  const profile = nalWithStart[scLen + 1]
  const constraint = nalWithStart[scLen + 2]
  const level = nalWithStart[scLen + 3]
  const hex = (n: number) => n.toString(16).padStart(2, '0').toUpperCase()
  return `avc1.${hex(profile)}${hex(constraint)}${hex(level)}`
}

export function CameraFeed() {
  const [hasError, setHasError] = useState(false)
  const [errorText, setErrorText] = useState<string>('')
  const [fps, setFps] = useState(0)
  const [resolution, setResolution] = useState<{ w: number; h: number } | null>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const wsRef = useRef<WebSocket | null>(null)
  const decoderRef = useRef<VideoDecoder | null>(null)
  const frameCountRef = useRef(0)

  // Проверка доступности WebCodecs API (Safari iOS — нет)
  const webCodecsSupported = typeof window !== 'undefined' && 'VideoDecoder' in window

  const cleanup = useCallback(() => {
    if (decoderRef.current) {
      try {
        if (decoderRef.current.state !== 'closed') decoderRef.current.close()
      } catch (_e) { /* ignore */ }
      decoderRef.current = null
    }
    if (wsRef.current) {
      try { wsRef.current.close() } catch (_e) { /* ignore */ }
      wsRef.current = null
    }
  }, [])

  useEffect(() => {
    if (!webCodecsSupported) {
      setHasError(true)
      setErrorText('Браузер не поддерживает WebCodecs (нужен Chrome 94+ / Edge 94+)')
      return
    }

    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) {
      setHasError(true)
      setErrorText('Canvas 2D context недоступен')
      return
    }

    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const wsUrl = `${proto}//${window.location.host}/ws/h264`

    let pendingAU: Uint8Array[] = []  // накопленные NAL units текущего access unit
    let auHasIDR = false              // содержит ли текущий AU IDR slice
    let configured = false             // decoder.configure() вызван
    let chunkTimestamp = 0
    // Интервал между фреймами в микросекундах. По умолчанию ~30fps,
    // обновляется из JSON header (ep.fps) когда дашборд его пришлёт.
    let frameIntervalUs = 33333

    const ws = new WebSocket(wsUrl)
    ws.binaryType = 'arraybuffer'
    wsRef.current = ws

    const decoder = new VideoDecoder({
      output: (frame: VideoFrame) => {
        if (!canvas || !ctx) {
          frame.close()
          return
        }
        if (canvas.width !== frame.displayWidth || canvas.height !== frame.displayHeight) {
          canvas.width = frame.displayWidth
          canvas.height = frame.displayHeight
          setResolution({ w: frame.displayWidth, h: frame.displayHeight })
        }
        ctx.drawImage(frame, 0, 0)
        frame.close()
        frameCountRef.current++
      },
      error: (e: DOMException) => {
        console.error('[CameraFeed] decode error:', e.message)
        setHasError(true)
        setErrorText(`Decode error: ${e.message}`)
      },
    })
    decoderRef.current = decoder

    const flushAU = (forceKey = false) => {
      if (pendingAU.length === 0) return
      // До первого SPS decoder ещё не настроен — буферизуем (но если AU
      // не содержит IDR, его всё равно бесполезно держать). Дроп.
      if (!configured) {
        pendingAU = []
        auHasIDR = false
        return
      }
      const totalLen = pendingAU.reduce((s, n) => s + n.byteLength, 0)
      const buf = new Uint8Array(totalLen)
      let off = 0
      for (const nal of pendingAU) {
        buf.set(nal, off)
        off += nal.byteLength
      }
      const chunkType: 'key' | 'delta' = (auHasIDR || forceKey) ? 'key' : 'delta'
      try {
        decoder.decode(new EncodedVideoChunk({
          type: chunkType,
          timestamp: chunkTimestamp,
          data: buf,
        }))
        chunkTimestamp += frameIntervalUs
      } catch (e) {
        console.error('[CameraFeed] EncodedVideoChunk error:', e)
      }
      pendingAU = []
      auHasIDR = false
    }

    const handleNAL = (nalWithStart: Uint8Array) => {
      // nalWithStart содержит start code + NAL byte + payload (Annex B).
      // Парсим NAL type из первого байта payload (после start code).
      // Start code: 0x000001 (3) или 0x00000001 (4).
      let scLen = 3
      if (nalWithStart[0] === 0 && nalWithStart[1] === 0
          && nalWithStart[2] === 0 && nalWithStart[3] === 1) {
        scLen = 4
      }
      if (nalWithStart.byteLength <= scLen) return
      const nalHeader = nalWithStart[scLen]
      const nalType = nalHeader & 0x1F  // bit 0..4 = NAL unit type

      // 1 = non-IDR slice, 5 = IDR slice → boundary access unit
      // 9 = AUD → начало нового AU
      // 7 = SPS, 8 = PPS, 6 = SEI → служебные, накапливаем в текущем AU
      const isSlice = (nalType === 1 || nalType === 5)
      const isAUD = (nalType === 9)

      // Configure decoder при первом SPS — единственный надёжный способ
      // узнать profile/level потока. JSON header от dashboard несёт только
      // 'h264' без точного codec string (см. routers/camera.py).
      if (nalType === 7 && !configured) {
        const codec = parseSpsCodec(nalWithStart) ?? FALLBACK_CODEC
        try {
          decoder.configure({ codec, optimizeForLatency: true })
          configured = true
          console.info('[CameraFeed] decoder configured:', codec)
        } catch (e) {
          // SPS не дал валидной строки — пробуем fallback (Main 4.0)
          console.warn('[CameraFeed] configure failed for', codec, '→ fallback', e)
          try {
            decoder.configure({ codec: FALLBACK_CODEC, optimizeForLatency: true })
            configured = true
          } catch (e2) {
            console.error('[CameraFeed] fallback configure failed:', e2)
            setHasError(true)
            setErrorText('VideoDecoder: codec не поддерживается')
            return
          }
        }
      }

      if (isAUD || (isSlice && pendingAU.some(_isSlice))) {
        // Это начало нового access unit — flush предыдущий
        flushAU()
      }

      pendingAU.push(nalWithStart)
      if (nalType === 5) auHasIDR = true

      // Если это slice — flush сразу (для low-latency, не ждём AUD следующего AU)
      if (isSlice) {
        flushAU()
      }
    }

    // Helper: проверяет, является ли уже накопленный NAL unit slice'ом
    const _isSlice = (nal: Uint8Array): boolean => {
      const sc = (nal[0] === 0 && nal[1] === 0 && nal[2] === 0 && nal[3] === 1) ? 4 : 3
      if (nal.byteLength <= sc) return false
      const t = nal[sc] & 0x1F
      return t === 1 || t === 5
    }

    // Накопительный буфер для NAL parsing (поток фрагментирован по WS msg)
    let streamBuf = new Uint8Array(0)

    const findStartCode = (buf: Uint8Array, from: number): number => {
      // Ищем 0x000001 или 0x00000001 начиная с offset `from`
      for (let i = from; i < buf.byteLength - 2; i++) {
        if (buf[i] === 0 && buf[i + 1] === 0) {
          if (buf[i + 2] === 1) return i
          if (i + 3 < buf.byteLength && buf[i + 2] === 0 && buf[i + 3] === 1) return i
        }
      }
      return -1
    }

    ws.onopen = () => {
      setHasError(false)
      setErrorText('')
    }

    ws.onmessage = (event) => {
      const data = event.data
      if (typeof data === 'string') {
        // JSON header — только display metadata. decoder.configure() переехал
        // в handleNAL() (на первый SPS), потому что точный codec string мы
        // знаем только из SPS, а не из dashboard endpoint info.
        try {
          const ep = JSON.parse(data) as EndpointInfo
          if (ep.type === 'endpoint') {
            if (ep.width && ep.height) {
              setResolution({ w: ep.width, h: ep.height })
            }
            if (ep.fps && ep.fps > 0) {
              frameIntervalUs = Math.round(1_000_000 / ep.fps)
            }
          }
        } catch (e) {
          console.warn('[CameraFeed] bad JSON header:', e)
        }
        return
      }

      // Binary chunk — добавляем к streamBuf и парсим NAL units.
      // SPS триггерит configure() внутри handleNAL — так что НЕ ждём JSON,
      // парсим всегда (JSON header может прийти из proxy, может не прийти).
      const newBytes = new Uint8Array(data as ArrayBuffer)
      const merged = new Uint8Array(streamBuf.byteLength + newBytes.byteLength)
      merged.set(streamBuf, 0)
      merged.set(newBytes, streamBuf.byteLength)
      streamBuf = merged

      // Найти все NAL units в streamBuf
      let pos = findStartCode(streamBuf, 0)
      if (pos < 0) return  // ни одного start code

      while (true) {
        const nextPos = findStartCode(streamBuf, pos + 3)
        if (nextPos < 0) {
          // Сохраняем хвост от текущего start code до конца — ждём следующего chunk
          streamBuf = streamBuf.slice(pos)
          break
        }
        const nal = streamBuf.slice(pos, nextPos)
        handleNAL(nal)
        pos = nextPos
      }
    }

    ws.onerror = (e) => {
      console.error('[CameraFeed] WebSocket error:', e)
      setHasError(true)
      setErrorText('WebSocket ошибка — Pi camera offline?')
    }

    ws.onclose = (e) => {
      if (e.code === 1011) {
        setHasError(true)
        setErrorText(`Камера недоступна: ${e.reason || 'unknown'}`)
      }
    }

    const fpsInterval = setInterval(() => {
      setFps(frameCountRef.current)
      frameCountRef.current = 0
    }, 1000)

    return () => {
      clearInterval(fpsInterval)
      cleanup()
    }
  }, [webCodecsSupported, cleanup])

  return (
    <Card>
      <CardHeader className="py-2 px-3 flex flex-row items-center justify-between">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Камера + YOLO
        </CardTitle>
        <div className="flex items-center gap-2">
          {!hasError && fps > 0 && (
            <span className="text-[10px] text-zinc-500 font-mono">{fps} fps</span>
          )}
          {!hasError && resolution && (
            <span className="text-[10px] text-zinc-600 font-mono">
              {resolution.w}×{resolution.h}
            </span>
          )}
          <span className="text-[10px] px-1.5 py-0.5 rounded bg-zinc-800 border border-zinc-600 text-zinc-400">
            H.264
          </span>
        </div>
      </CardHeader>
      <CardContent className="p-0">
        {hasError ? (
          <div className="flex flex-col items-center justify-center h-48 bg-black/50 text-muted-foreground text-sm gap-2 p-4 text-center">
            <span>Нет видеопотока</span>
            {errorText && (
              <span className="text-[10px] text-zinc-600 font-mono">{errorText}</span>
            )}
          </div>
        ) : (
          <canvas
            ref={canvasRef}
            className="w-full h-auto block bg-black min-h-[200px]"
          />
        )}
      </CardContent>
    </Card>
  )
}
