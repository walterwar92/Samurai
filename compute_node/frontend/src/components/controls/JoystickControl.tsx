import { useJoystick } from '@/hooks/useJoystick'
import { cn } from '@/lib/utils'

/**
 * Технический джойстик с heading-vector и цифровым readout.
 * Spring-feel при release — через CSS easing `ease-spring` (cubic-bezier с
 * overshoot). Никаких JS animation libs.
 */
export function JoystickControl() {
  const { containerRef, state, startJoy, moveJoy, endJoy } = useJoystick()
  const { knobX, knobY, linear, angular, active } = state

  return (
    <div className="flex flex-col items-center gap-3 select-none">
      <div
        ref={containerRef}
        onPointerDown={startJoy}
        onPointerMove={moveJoy}
        onPointerUp={endJoy}
        onPointerCancel={endJoy}
        className="joystick-area relative h-44 w-44 rounded-full border border-subtle bg-surface-2 cursor-grab active:cursor-grabbing"
      >
        {/* Кресты и кольца через SVG */}
        <svg viewBox="0 0 176 176" className="absolute inset-0 pointer-events-none">
          <line x1="88" y1="20" x2="88" y2="156" stroke="hsl(var(--surface-3))" strokeWidth="1" />
          <line x1="20" y1="88" x2="156" y2="88" stroke="hsl(var(--surface-3))" strokeWidth="1" />
          <circle
            cx="88"
            cy="88"
            r="56"
            fill="none"
            stroke="hsl(var(--surface-3))"
            strokeWidth="1"
            strokeDasharray="3 4"
          />
          <circle
            cx="88"
            cy="88"
            r="28"
            fill="none"
            stroke="hsl(var(--surface-3))"
            strokeWidth="1"
            strokeDasharray="2 3"
          />
          {active && (
            <line
              x1="88"
              y1="88"
              x2={(knobX / 100) * 176}
              y2={(knobY / 100) * 176}
              stroke="hsl(var(--accent))"
              strokeWidth="2"
              strokeLinecap="round"
            />
          )}
        </svg>

        {/* Knob */}
        <div
          className={cn(
            'absolute h-11 w-11 rounded-full bg-surface-3 border-2 border-accent',
            'transition-transform',
            active ? 'duration-fast ease-standard' : 'duration-standard ease-spring',
          )}
          style={{
            left: `${knobX}%`,
            top: `${knobY}%`,
            transform: 'translate(-50%, -50%)',
          }}
        />
      </div>

      {/* Readout */}
      <div className="font-mono text-small text-foreground-muted tabular-nums flex items-center gap-1">
        <span className={cn(linear !== 0 && 'text-accent')}>{linear.toFixed(2)}</span>
        <span className="text-foreground-faint">m/s</span>
        <span className="mx-2 text-foreground-faint">·</span>
        <span className={cn(angular !== 0 && 'text-accent')}>{angular.toFixed(2)}</span>
        <span className="text-foreground-faint">rad/s</span>
      </div>
    </div>
  )
}
