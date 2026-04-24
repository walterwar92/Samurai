interface ArmVisualizerProps {
  arm?: number    // 0..180 — наклон плеча (0=опущено, 90=нейтраль, 180=вверх)
  base?: number   // 0..180 — поворот базы (90=прямо, 0=влево, 180=вправо)
  claw?: number   // угол серво (CLAW_OPEN..CLAW_CLOSED)
  clawOpen?: number
  clawClosed?: number
}

/**
 * SVG-схема руки — вид сбоку. Плечо (ARM) поворачивается от своей оси,
 * клешня раскрывается/закрывается. База показана сверху как стрелка.
 */
export function ArmVisualizer({
  arm = 90,
  base = 90,
  claw = 90,
  clawOpen = 70,
  clawClosed = 150,
}: ArmVisualizerProps) {
  // ARM: 90 = горизонтально вперёд, 0 = вниз, 180 = вверх
  // SVG y возрастает вниз, поэтому инвертируем
  const armRad = ((arm - 90) * Math.PI) / 180  // -90..+90
  const upperLen = 60
  const baseX = 80
  const baseY = 110
  const elbowX = baseX + Math.cos(armRad) * upperLen
  const elbowY = baseY - Math.sin(armRad) * upperLen
  const handLen = 36
  const handX = elbowX + Math.cos(armRad) * handLen
  const handY = elbowY - Math.sin(armRad) * handLen

  // Клешня раскрывается симметрично от продольной оси руки.
  // Нормализуем к 0..1 (1 = полностью открыта)
  const clawRange = Math.abs(clawClosed - clawOpen)
  const openness = clawRange === 0 ? 0.5 :
    Math.max(0, Math.min(1, Math.abs(claw - clawClosed) / clawRange))
  const fingerSpread = (15 + openness * 30) * Math.PI / 180  // 15°..45° от оси
  const fingerLen = 22
  const finger1X = handX + Math.cos(armRad + fingerSpread) * fingerLen
  const finger1Y = handY - Math.sin(armRad + fingerSpread) * fingerLen
  const finger2X = handX + Math.cos(armRad - fingerSpread) * fingerLen
  const finger2Y = handY - Math.sin(armRad - fingerSpread) * fingerLen

  // База — поворот сверху (стрелка)
  const baseAngle = base - 90  // -90..+90 от прямого вперёд
  const baseRad = (baseAngle * Math.PI) / 180

  return (
    <div className="flex flex-col items-center gap-2">
      <svg width={200} height={170} viewBox="0 0 200 170" className="block">
        {/* ground */}
        <line x1={20} y1={130} x2={180} y2={130} stroke="rgb(71 85 105)" strokeDasharray="3 4" />
        {/* base mount */}
        <rect x={baseX - 12} y={baseY - 4} width={24} height={20} rx={3} fill="rgb(51 65 85)" stroke="rgb(100 116 139)" />
        {/* upper arm */}
        <line
          x1={baseX} y1={baseY}
          x2={elbowX} y2={elbowY}
          stroke="rgb(56 189 248)"
          strokeWidth={6}
          strokeLinecap="round"
        />
        {/* hand pivot */}
        <circle cx={elbowX} cy={elbowY} r={4} fill="rgb(15 23 42)" stroke="rgb(56 189 248)" strokeWidth={2} />
        {/* hand link */}
        <line
          x1={elbowX} y1={elbowY}
          x2={handX} y2={handY}
          stroke="rgb(125 211 252)"
          strokeWidth={5}
          strokeLinecap="round"
        />
        {/* claw fingers */}
        <line
          x1={handX} y1={handY}
          x2={finger1X} y2={finger1Y}
          stroke={openness > 0.6 ? 'rgb(34 197 94)' : 'rgb(234 179 8)'}
          strokeWidth={4}
          strokeLinecap="round"
        />
        <line
          x1={handX} y1={handY}
          x2={finger2X} y2={finger2Y}
          stroke={openness > 0.6 ? 'rgb(34 197 94)' : 'rgb(234 179 8)'}
          strokeWidth={4}
          strokeLinecap="round"
        />
        <circle cx={handX} cy={handY} r={3} fill="rgb(56 189 248)" />
      </svg>

      {/* Base direction (top-down arrow) */}
      <div className="flex items-center gap-3 text-[10px] text-muted-foreground">
        <span>BASE</span>
        <svg width={56} height={32} viewBox="-28 -16 56 32">
          <circle cx={0} cy={0} r={14} fill="none" stroke="rgb(71 85 105)" strokeDasharray="2 3" />
          <line
            x1={0} y1={0}
            x2={Math.sin(baseRad) * 12}
            y2={-Math.cos(baseRad) * 12}
            stroke="rgb(56 189 248)"
            strokeWidth={2}
            strokeLinecap="round"
          />
          <circle cx={Math.sin(baseRad) * 12} cy={-Math.cos(baseRad) * 12} r={3} fill="rgb(56 189 248)" />
        </svg>
        <span className="font-mono text-zinc-400">{base}°</span>
      </div>

      <div className="grid grid-cols-3 gap-3 text-[10px] font-mono w-full">
        <Stat label="ARM"  value={`${arm}°`}  color="rgb(56 189 248)" />
        <Stat label="BASE" value={`${base}°`} color="rgb(56 189 248)" />
        <Stat label="CLAW" value={`${claw}°`} color={openness > 0.6 ? 'rgb(34 197 94)' : 'rgb(234 179 8)'} />
      </div>
    </div>
  )
}

function Stat({ label, value, color }: { label: string; value: string; color: string }) {
  return (
    <div className="flex flex-col items-center bg-zinc-900/50 rounded p-1.5 border border-border">
      <span className="text-[8px] uppercase tracking-wider text-muted-foreground">{label}</span>
      <span style={{ color }} className="font-bold tabular-nums">{value}</span>
    </div>
  )
}
