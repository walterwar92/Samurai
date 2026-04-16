import { useState, useRef, useCallback, useEffect } from 'react'
import type { HardwarePreset, LayoutMap, BlockPosition } from '@/types/hardware'

/* =================================================================
 * Canvas-based hardware block diagram (Simulink / LabVIEW style)
 *
 * Features:
 *   • Draggable blocks on a dotted-grid canvas
 *   • Bezier wires between typed I/O ports
 *   • Connections derived from current preset
 *   • Click block → select (opens properties panel)
 *   • Layout persisted in preset.layout
 * ================================================================= */

type BlockId =
  | 'controller'
  | 'pca9685'
  | 'motors'
  | 'servos'
  | 'imu'
  | 'camera'
  | 'range'
  | 'leds'

// A single port on a block (input OR output)
interface PortDef {
  id: string
  label: string
}

interface BlockDef {
  id: BlockId
  label: string
  icon: string
  color: string
  w: number
  h: number
  inputs: PortDef[]   // on the left side
  outputs: PortDef[]  // on the right side
  defaultPos: BlockPosition
  summary: (p: HardwarePreset) => string[]
  visible: (p: HardwarePreset) => boolean
  // Maps to one of the editor block ids used by HardwarePage
  editorKey: 'motors' | 'servos' | 'camera' | 'imu' | 'range' | 'leds' | null
}

// ---------------------------------------------------------------
// Block library
// ---------------------------------------------------------------
const BLOCK_DEFS: BlockDef[] = [
  {
    id: 'controller',
    label: 'Контроллер',
    icon: '▣',
    color: '#0ea5e9',
    w: 180,
    h: 120,
    inputs: [],
    outputs: [
      { id: 'i2c', label: 'I²C' },
      { id: 'gpio', label: 'GPIO' },
      { id: 'usb', label: 'USB' },
    ],
    defaultPos: { x: 40, y: 180 },
    editorKey: null,
    visible: () => true,
    summary: (p) => [
      p.platform === 'raspberry_pi_pca9685' ? 'Raspberry Pi 4' : p.platform,
      'CPU · GPIO · USB',
    ],
  },
  {
    id: 'pca9685',
    label: 'PCA9685',
    icon: '◈',
    color: '#f59e0b',
    w: 180,
    h: 120,
    inputs: [{ id: 'i2c', label: 'I²C' }],
    outputs: [
      { id: 'pwm_m', label: 'PWM·M' },
      { id: 'pwm_s', label: 'PWM·S' },
    ],
    defaultPos: { x: 320, y: 40 },
    editorKey: null,
    visible: (p) =>
      p.motors.driver === 'pca9685' || p.servos.driver === 'pca9685',
    summary: (p) => [
      `Adr: ${p.i2c.pca9685_address}`,
      `Freq: ${p.i2c.pca9685_frequency} Hz`,
      '16 каналов PWM',
    ],
  },
  {
    id: 'motors',
    label: 'Моторы DC',
    icon: '⚙',
    color: '#ef4444',
    w: 200,
    h: 150,
    inputs: [{ id: 'pwm', label: 'PWM' }],
    outputs: [],
    defaultPos: { x: 600, y: 20 },
    editorKey: 'motors',
    visible: () => true,
    summary: (p) => {
      const count = Object.keys(p.motors.channels).length
      const chans = Object.entries(p.motors.channels)
        .map(([k, ch]) => `${k}: ${ch.in1}/${ch.in2}`)
      return [`${count} мотора (${p.motors.driver})`, ...chans]
    },
  },
  {
    id: 'servos',
    label: 'Сервоприводы',
    icon: '⚒',
    color: '#8b5cf6',
    w: 200,
    h: 160,
    inputs: [{ id: 'pwm', label: 'PWM' }],
    outputs: [],
    defaultPos: { x: 600, y: 200 },
    editorKey: 'servos',
    visible: () => true,
    summary: (p) => {
      const lines = [
        `Голова: ch${p.servos.head.channel}`,
        `Рука: ${p.servos.arm.channels.length} сустава`,
      ]
      p.servos.arm.labels.slice(0, 3).forEach((l, i) => {
        lines.push(`· ${l}: ch${p.servos.arm.channels[i]}`)
      })
      return lines
    },
  },
  {
    id: 'imu',
    label: 'IMU',
    icon: '⌖',
    color: '#10b981',
    w: 170,
    h: 90,
    inputs: [{ id: 'i2c', label: 'I²C' }],
    outputs: [],
    defaultPos: { x: 320, y: 200 },
    editorKey: 'imu',
    visible: (p) => p.imu.type !== 'none',
    summary: (p) => [p.imu.type.toUpperCase(), `Adr: ${p.imu.address}`],
  },
  {
    id: 'range',
    label: 'Дальномер',
    icon: '◥',
    color: '#06b6d4',
    w: 180,
    h: 100,
    inputs: [{ id: 'gpio', label: 'GPIO' }],
    outputs: [],
    defaultPos: { x: 320, y: 340 },
    editorKey: 'range',
    visible: (p) => p.range_sensor.type !== 'none',
    summary: (p) => {
      if (p.range_sensor.type === 'hc_sr04')
        return [
          'HC-SR04',
          `Trig: GPIO${p.range_sensor.trigger_pin}`,
          `Echo: GPIO${p.range_sensor.echo_pin}`,
        ]
      return [p.range_sensor.type.toUpperCase()]
    },
  },
  {
    id: 'leds',
    label: 'LED',
    icon: '✦',
    color: '#ec4899',
    w: 170,
    h: 90,
    inputs: [{ id: 'gpio', label: 'GPIO' }],
    outputs: [],
    defaultPos: { x: 40, y: 340 },
    editorKey: 'leds',
    visible: (p) => p.leds.type !== 'none',
    summary: (p) => [
      `${p.leds.type.toUpperCase()} x${p.leds.count}`,
      `GPIO ${p.leds.gpio_pin}`,
    ],
  },
  {
    id: 'camera',
    label: 'Камера',
    icon: '◉',
    color: '#f97316',
    w: 170,
    h: 90,
    inputs: [{ id: 'usb', label: 'USB' }],
    outputs: [],
    defaultPos: { x: 40, y: 40 },
    editorKey: 'camera',
    visible: (p) => p.camera.type !== 'none',
    summary: (p) => [
      p.camera.type.toUpperCase(),
      `${p.camera.width}×${p.camera.height}`,
    ],
  },
]

// ---------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------
interface Connection {
  fromBlock: BlockId
  fromPort: string
  toBlock: BlockId
  toPort: string
}

function deriveConnections(preset: HardwarePreset): Connection[] {
  const conns: Connection[] = []
  const usesPca =
    preset.motors.driver === 'pca9685' || preset.servos.driver === 'pca9685'

  if (usesPca) {
    conns.push({
      fromBlock: 'controller',
      fromPort: 'i2c',
      toBlock: 'pca9685',
      toPort: 'i2c',
    })
  }

  if (preset.motors.driver === 'pca9685') {
    conns.push({
      fromBlock: 'pca9685',
      fromPort: 'pwm_m',
      toBlock: 'motors',
      toPort: 'pwm',
    })
  } else {
    conns.push({
      fromBlock: 'controller',
      fromPort: 'gpio',
      toBlock: 'motors',
      toPort: 'pwm',
    })
  }

  if (preset.servos.driver === 'pca9685') {
    conns.push({
      fromBlock: 'pca9685',
      fromPort: 'pwm_s',
      toBlock: 'servos',
      toPort: 'pwm',
    })
  } else {
    conns.push({
      fromBlock: 'controller',
      fromPort: 'gpio',
      toBlock: 'servos',
      toPort: 'pwm',
    })
  }

  if (preset.imu.type !== 'none') {
    conns.push({
      fromBlock: 'controller',
      fromPort: 'i2c',
      toBlock: 'imu',
      toPort: 'i2c',
    })
  }

  if (preset.range_sensor.type !== 'none') {
    conns.push({
      fromBlock: 'controller',
      fromPort: 'gpio',
      toBlock: 'range',
      toPort: 'gpio',
    })
  }

  if (preset.leds.type !== 'none') {
    conns.push({
      fromBlock: 'controller',
      fromPort: 'gpio',
      toBlock: 'leds',
      toPort: 'gpio',
    })
  }

  if (preset.camera.type !== 'none') {
    conns.push({
      fromBlock: 'controller',
      fromPort: 'usb',
      toBlock: 'camera',
      toPort: 'usb',
    })
  }

  return conns
}

function portPosition(
  block: BlockDef,
  pos: BlockPosition,
  portId: string,
  side: 'input' | 'output',
): { x: number; y: number } {
  const ports = side === 'input' ? block.inputs : block.outputs
  const idx = ports.findIndex((p) => p.id === portId)
  if (idx < 0) return { x: pos.x + block.w / 2, y: pos.y + block.h / 2 }
  const step = block.h / (ports.length + 1)
  const y = pos.y + step * (idx + 1)
  const x = side === 'input' ? pos.x : pos.x + block.w
  return { x, y }
}

function bezierPath(
  from: { x: number; y: number },
  to: { x: number; y: number },
): string {
  const dx = Math.max(40, Math.abs(to.x - from.x) / 2)
  return `M ${from.x} ${from.y} C ${from.x + dx} ${from.y}, ${to.x - dx} ${to.y}, ${to.x} ${to.y}`
}

// ---------------------------------------------------------------
// Component props
// ---------------------------------------------------------------
interface HardwareBlockDiagramProps {
  preset: HardwarePreset
  layout: LayoutMap
  activeBlock: BlockId | null
  onBlockClick: (id: BlockId) => void
  onLayoutChange: (layout: LayoutMap) => void
}

// ---------------------------------------------------------------
// Main component
// ---------------------------------------------------------------
export function HardwareBlockDiagram({
  preset,
  layout,
  activeBlock,
  onBlockClick,
  onLayoutChange,
}: HardwareBlockDiagramProps) {
  const canvasRef = useRef<HTMLDivElement>(null)
  const [dragging, setDragging] = useState<{
    id: BlockId
    offX: number
    offY: number
    moved: boolean
  } | null>(null)

  const visible = BLOCK_DEFS.filter((b) => b.visible(preset))
  const connections = deriveConnections(preset).filter(
    (c) =>
      visible.some((b) => b.id === c.fromBlock) &&
      visible.some((b) => b.id === c.toBlock),
  )

  const getPos = useCallback(
    (b: BlockDef): BlockPosition => layout[b.id] || b.defaultPos,
    [layout],
  )

  // Canvas bounds (auto-grow)
  const maxX =
    Math.max(...visible.map((b) => getPos(b).x + b.w), 900) + 60
  const maxY =
    Math.max(...visible.map((b) => getPos(b).y + b.h), 460) + 60
  const width = maxX
  const height = maxY

  // ── Drag handling ────────────────────────────────────────────
  const handlePointerDown = (
    e: React.PointerEvent<HTMLDivElement>,
    block: BlockDef,
  ) => {
    if (!canvasRef.current) return
    const scroll = canvasRef.current
    const rect = scroll.getBoundingClientRect()
    const pos = getPos(block)
    const clientX = e.clientX + scroll.scrollLeft - rect.left
    const clientY = e.clientY + scroll.scrollTop - rect.top
    setDragging({
      id: block.id,
      offX: clientX - pos.x,
      offY: clientY - pos.y,
      moved: false,
    })
    ;(e.target as Element).setPointerCapture?.(e.pointerId)
    e.stopPropagation()
  }

  const handlePointerMove = useCallback(
    (e: PointerEvent) => {
      if (!dragging || !canvasRef.current) return
      const scroll = canvasRef.current
      const rect = scroll.getBoundingClientRect()
      const clientX = e.clientX + scroll.scrollLeft - rect.left
      const clientY = e.clientY + scroll.scrollTop - rect.top
      const newX = Math.max(0, clientX - dragging.offX)
      const newY = Math.max(0, clientY - dragging.offY)
      onLayoutChange({
        ...layout,
        [dragging.id]: { x: newX, y: newY },
      })
      if (!dragging.moved) {
        setDragging({ ...dragging, moved: true })
      }
    },
    [dragging, layout, onLayoutChange],
  )

  const handlePointerUp = useCallback(() => {
    setDragging(null)
  }, [])

  useEffect(() => {
    if (dragging) {
      window.addEventListener('pointermove', handlePointerMove)
      window.addEventListener('pointerup', handlePointerUp)
      return () => {
        window.removeEventListener('pointermove', handlePointerMove)
        window.removeEventListener('pointerup', handlePointerUp)
      }
    }
  }, [dragging, handlePointerMove, handlePointerUp])

  const resetLayout = () => {
    onLayoutChange({})
  }

  return (
    <div className="space-y-2">
      {/* Toolbar */}
      <div className="flex items-center justify-between text-[10px] text-zinc-500">
        <span>
          {visible.length} блоков · {connections.length} соединений
        </span>
        <button
          className="px-2 py-0.5 rounded border border-zinc-700 hover:border-zinc-500 hover:text-zinc-300"
          onClick={resetLayout}
          title="Сбросить позиции блоков"
        >
          ↻ Сбросить раскладку
        </button>
      </div>

      {/* Scrollable canvas */}
      <div
        ref={canvasRef}
        className="relative overflow-auto rounded-md border border-zinc-800 bg-zinc-950"
        style={{ minHeight: '500px', maxHeight: '72vh' }}
      >
        <div
          className="relative"
          style={{
            width: `${width}px`,
            height: `${height}px`,
            backgroundImage:
              'radial-gradient(circle, rgba(63, 63, 70, 0.45) 1px, transparent 1px)',
            backgroundSize: '22px 22px',
          }}
        >
          {/* SVG wires */}
          <svg
            className="absolute top-0 left-0 pointer-events-none"
            width={width}
            height={height}
            style={{ zIndex: 1 }}
          >
            <defs>
              <marker
                id="hw-arrowhead"
                markerWidth="10"
                markerHeight="10"
                refX="9"
                refY="5"
                orient="auto"
              >
                <polygon points="0 0, 10 5, 0 10" fill="#a1a1aa" />
              </marker>
              <linearGradient id="hw-wire" x1="0%" y1="0%" x2="100%" y2="0%">
                <stop offset="0%" stopColor="#71717a" />
                <stop offset="100%" stopColor="#a1a1aa" />
              </linearGradient>
            </defs>
            {connections.map((c, i) => {
              const fromBlock = BLOCK_DEFS.find((b) => b.id === c.fromBlock)!
              const toBlock = BLOCK_DEFS.find((b) => b.id === c.toBlock)!
              const fromPos = portPosition(
                fromBlock,
                getPos(fromBlock),
                c.fromPort,
                'output',
              )
              const toPos = portPosition(
                toBlock,
                getPos(toBlock),
                c.toPort,
                'input',
              )
              return (
                <g key={i}>
                  {/* glow */}
                  <path
                    d={bezierPath(fromPos, toPos)}
                    stroke={fromBlock.color}
                    strokeWidth="6"
                    fill="none"
                    opacity="0.12"
                  />
                  {/* wire */}
                  <path
                    d={bezierPath(fromPos, toPos)}
                    stroke="url(#hw-wire)"
                    strokeWidth="2"
                    fill="none"
                    markerEnd="url(#hw-arrowhead)"
                  />
                </g>
              )
            })}
          </svg>

          {/* Blocks */}
          {visible.map((b) => {
            const pos = getPos(b)
            const isActive = activeBlock === b.id
            const isDrag = dragging?.id === b.id
            return (
              <div
                key={b.id}
                className="absolute rounded-lg border-2 select-none transition-shadow"
                style={{
                  left: `${pos.x}px`,
                  top: `${pos.y}px`,
                  width: `${b.w}px`,
                  minHeight: `${b.h}px`,
                  borderColor: isActive ? '#fbbf24' : b.color + '88',
                  background: 'rgba(24, 24, 27, 0.97)',
                  boxShadow: isActive
                    ? '0 0 0 2px rgba(251, 191, 36, 0.4), 0 8px 24px rgba(0,0,0,0.5)'
                    : isDrag
                      ? '0 8px 24px rgba(0,0,0,0.7)'
                      : '0 2px 8px rgba(0,0,0,0.4)',
                  zIndex: isDrag ? 10 : isActive ? 5 : 2,
                  cursor: 'pointer',
                }}
                onClick={(e) => {
                  e.stopPropagation()
                  // Only register click if not a drag
                  if (!dragging?.moved) onBlockClick(b.id)
                }}
              >
                {/* Header (also drag handle) */}
                <div
                  className="flex items-center gap-1.5 px-2 py-1 rounded-t-md"
                  style={{
                    background: `linear-gradient(90deg, ${b.color}33, ${b.color}0a)`,
                    borderBottom: `1px solid ${b.color}55`,
                    cursor: isDrag ? 'grabbing' : 'grab',
                  }}
                  onPointerDown={(e) => handlePointerDown(e, b)}
                >
                  <span className="text-sm" style={{ color: b.color }}>
                    {b.icon}
                  </span>
                  <span className="text-[11px] font-semibold text-zinc-100 tracking-wide">
                    {b.label}
                  </span>
                </div>

                {/* Body */}
                <div className="p-2 space-y-0.5">
                  {b.summary(preset).map((line, i) => (
                    <div
                      key={i}
                      className="text-[9px] font-mono text-zinc-400 leading-tight truncate"
                    >
                      {line}
                    </div>
                  ))}
                </div>

                {/* Inputs (left side) */}
                {b.inputs.map((p, idx) => {
                  const step = b.h / (b.inputs.length + 1)
                  const y = step * (idx + 1) - 6
                  return (
                    <div
                      key={`in-${p.id}`}
                      className="absolute flex items-center pointer-events-none"
                      style={{ left: '-6px', top: `${y}px` }}
                    >
                      <div
                        className="w-3 h-3 rounded-full border-2"
                        style={{
                          background: '#0b0b0e',
                          borderColor: b.color,
                        }}
                      />
                      <span
                        className="ml-1 text-[8px] font-mono text-zinc-500 whitespace-nowrap"
                        style={{ textShadow: '0 0 4px #000' }}
                      >
                        {p.label}
                      </span>
                    </div>
                  )
                })}

                {/* Outputs (right side) */}
                {b.outputs.map((p, idx) => {
                  const step = b.h / (b.outputs.length + 1)
                  const y = step * (idx + 1) - 6
                  return (
                    <div
                      key={`out-${p.id}`}
                      className="absolute flex items-center pointer-events-none"
                      style={{ right: '-6px', top: `${y}px` }}
                    >
                      <span
                        className="mr-1 text-[8px] font-mono text-zinc-500 whitespace-nowrap"
                        style={{ textShadow: '0 0 4px #000' }}
                      >
                        {p.label}
                      </span>
                      <div
                        className="w-3 h-3 rounded-full border-2"
                        style={{
                          background: '#0b0b0e',
                          borderColor: b.color,
                        }}
                      />
                    </div>
                  )
                })}
              </div>
            )
          })}
        </div>
      </div>

      {/* Legend */}
      <div className="flex flex-wrap gap-x-3 gap-y-1 text-[9px] text-zinc-500">
        <span>🟢 Потяни блок за заголовок</span>
        <span>· Клик — свойства</span>
        <span>· Стрелки = аппаратная связь</span>
      </div>
    </div>
  )
}

export type { BlockId }
