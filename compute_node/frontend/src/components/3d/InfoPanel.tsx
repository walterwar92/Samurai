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
  // EKF toggle
  useEkf: boolean
  hasEkf: boolean
  onToggleEkf: () => void
  ekfBias: [number, number, number] | null
  yprRaw: [number, number, number] | null
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
  useEkf, hasEkf, onToggleEkf,
  ekfBias, yprRaw,
}: InfoPanelProps) {
  return (
    <div className="absolute top-3 left-3 bg-zinc-900/85 backdrop-blur-sm border border-zinc-700 rounded-lg p-3 min-w-[220px] select-none pointer-events-auto">
      <div className="text-xs font-semibold text-zinc-300 mb-2 border-b border-zinc-700 pb-1">
        IMU / Одометрия
      </div>

      {/* Orientation + EKF toggle */}
      <div className="mb-2">
        <div className="flex items-center justify-between mb-0.5">
          <div className="text-[10px] text-zinc-500 uppercase tracking-wider">Ориентация</div>
          {hasEkf && (
            <button
              onClick={onToggleEkf}
              className={`px-1.5 py-0.5 text-[10px] rounded border transition-colors font-mono ${
                useEkf
                  ? 'bg-emerald-900/80 border-emerald-600 text-emerald-300'
                  : 'bg-zinc-800 border-zinc-600 text-zinc-400'
              }`}
            >
              {useEkf ? 'EKF' : 'RAW'}
            </button>
          )}
        </div>
        <V label="Yaw" value={yaw.toFixed(1)} unit="°" />
        <V label="Pitch" value={pitch.toFixed(1)} unit="°" />
        <V label="Roll" value={roll.toFixed(1)} unit="°" />

        {/* Show raw values for comparison when EKF is active */}
        {useEkf && hasEkf && yprRaw && (
          <div className="mt-1 pt-1 border-t border-zinc-800">
            <div className="text-[10px] text-zinc-600 mb-0.5">RAW (сравнение)</div>
            <div className="flex justify-between gap-2 text-[10px]">
              <span className="text-zinc-600">Y/P/R</span>
              <span className="text-zinc-500 font-mono">
                {yprRaw[0].toFixed(1)} / {yprRaw[1].toFixed(1)} / {yprRaw[2].toFixed(1)}°
              </span>
            </div>
          </div>
        )}
      </div>

      {/* Gyro bias (EKF) */}
      {useEkf && hasEkf && ekfBias && (
        <div className="mb-2">
          <div className="text-[10px] text-emerald-600 uppercase tracking-wider mb-0.5">
            Гиро bias (EKF)
          </div>
          <V label="bX" value={ekfBias[0].toFixed(5)} unit="рад/с" />
          <V label="bY" value={ekfBias[1].toFixed(5)} unit="рад/с" />
          <V label="bZ" value={ekfBias[2].toFixed(5)} unit="рад/с" />
        </div>
      )}

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
