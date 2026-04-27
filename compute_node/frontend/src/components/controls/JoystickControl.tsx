import { useJoystick } from '@/hooks/useJoystick'

export function JoystickControl() {
  const { containerRef, state, startJoy, moveJoy, endJoy } = useJoystick()

  return (
    <div className="space-y-2">
      <div
        ref={containerRef}
        className="joystick-area relative w-40 h-40 rounded-full bg-secondary/50 border-2 border-border mx-auto cursor-pointer select-none"
        onPointerDown={startJoy}
        onPointerMove={moveJoy}
        onPointerUp={endJoy}
        onPointerCancel={endJoy}
      >
        {/* Center dot */}
        <div className="absolute top-1/2 left-1/2 w-2 h-2 rounded-full bg-muted-foreground/30 -translate-x-1/2 -translate-y-1/2" />

        {/* Knob */}
        <div
          className="absolute w-11 h-11 rounded-full bg-primary/85 border-2 border-primary shadow-lg shadow-primary/20 -translate-x-1/2 -translate-y-1/2 transition-none"
          style={{
            left: `${state.knobX}%`,
            top: `${state.knobY}%`,
          }}
        />
      </div>

      {/* Velocity display */}
      <div className="flex justify-center gap-4 text-xs text-muted-foreground">
        <span>
          Лин: <span className="font-mono text-foreground">{state.linear.toFixed(2)}</span> м/с
        </span>
        <span>
          Угл: <span className="font-mono text-foreground">{state.angular.toFixed(2)}</span> рад/с
        </span>
      </div>
    </div>
  )
}
