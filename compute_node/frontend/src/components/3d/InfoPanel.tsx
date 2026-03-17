interface InfoPanelProps {
  yaw: number
  pitch: number
  roll: number
  accel: [number, number, number]
  gyro: [number, number, number]
  posX: number
  posY: number
  linearVel: number
  angularVel: number
  onClearPath: () => void
}

function V({ label, value, unit }: { label: string; value: string; unit?: string }) {
  return (
    <div className="flex justify-between gap-2 text-xs">
      <span className="text-zinc-400">{label}</span>
      <span className="text-zinc-100 font-mono">
        {value}
        {unit && <span className="text-zinc-500 ml-0.5">{unit}</span>}
      </span>
    </div>
  )
}

export function InfoPanel({
  yaw, pitch, roll,
  accel, gyro,
  posX, posY,
  linearVel, angularVel,
  onClearPath,
}: InfoPanelProps) {
  return (
    <div className="absolute top-3 left-3 bg-zinc-900/85 backdrop-blur-sm border border-zinc-700 rounded-lg p-3 min-w-[200px] select-none pointer-events-auto">
      <div className="text-xs font-semibold text-zinc-300 mb-2 border-b border-zinc-700 pb-1">
        IMU / Одометрия
      </div>

      {/* Orientation */}
      <div className="mb-2">
        <div className="text-[10px] text-zinc-500 uppercase tracking-wider mb-0.5">Ориентация</div>
        <V label="Yaw" value={yaw.toFixed(1)} unit="°" />
        <V label="Pitch" value={pitch.toFixed(1)} unit="°" />
        <V label="Roll" value={roll.toFixed(1)} unit="°" />
      </div>

      {/* Accelerometer */}
      <div className="mb-2">
        <div className="text-[10px] text-zinc-500 uppercase tracking-wider mb-0.5">
          Акселерометр <span className="text-red-400">●</span>
        </div>
        <V label="X" value={accel[0].toFixed(3)} unit="м/с²" />
        <V label="Y" value={accel[1].toFixed(3)} unit="м/с²" />
        <V label="Z" value={accel[2].toFixed(3)} unit="м/с²" />
        <V
          label="|a|"
          value={Math.sqrt(accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2).toFixed(3)}
          unit="м/с²"
        />
      </div>

      {/* Gyroscope */}
      <div className="mb-2">
        <div className="text-[10px] text-zinc-500 uppercase tracking-wider mb-0.5">Гироскоп</div>
        <V label="X" value={gyro[0].toFixed(3)} unit="рад/с" />
        <V label="Y" value={gyro[1].toFixed(3)} unit="рад/с" />
        <V label="Z" value={gyro[2].toFixed(3)} unit="рад/с" />
      </div>

      {/* Odometry */}
      <div className="mb-2">
        <div className="text-[10px] text-zinc-500 uppercase tracking-wider mb-0.5">Позиция</div>
        <V label="X" value={posX.toFixed(3)} unit="м" />
        <V label="Y" value={posY.toFixed(3)} unit="м" />
        <V label="V lin" value={linearVel.toFixed(3)} unit="м/с" />
        <V label="V ang" value={angularVel.toFixed(3)} unit="рад/с" />
      </div>

      <button
        onClick={onClearPath}
        className="w-full mt-1 px-2 py-1 text-xs bg-zinc-800 hover:bg-zinc-700 text-zinc-300 rounded border border-zinc-600 transition-colors"
      >
        Очистить путь
      </button>
    </div>
  )
}
