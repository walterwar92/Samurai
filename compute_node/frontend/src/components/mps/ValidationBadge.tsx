import type { MpsValidateResult } from '@/types/mps'

interface ValidationBadgeProps {
  result: MpsValidateResult | null
}

export function ValidationBadge({ result }: ValidationBadgeProps) {
  if (!result) {
    return (
      <div className="text-xs text-muted-foreground">
        Validate ещё не запущен.
      </div>
    )
  }

  const maxAbsOpen = Math.max(
    ...result.eigenvalues_ad.map((z) => Math.hypot(z.re, z.im)),
    0,
  )
  const maxAbsClosed = Math.max(
    ...result.eigenvalues_closed.map((z) => Math.hypot(z.re, z.im)),
    0,
  )

  return (
    <div className="text-xs space-y-1">
      <div>
        Plant:{' '}
        <strong className={result.is_plant_stable ? 'text-green-600' : 'text-red-600'}>
          {result.is_plant_stable ? 'STABLE' : 'UNSTABLE'}
        </strong>
        {' '}|λ|max = {maxAbsOpen.toFixed(3)}
      </div>
      <div>
        Closed-loop:{' '}
        <strong className={result.is_closed_loop_stable ? 'text-green-600' : 'text-red-600'}>
          {result.is_closed_loop_stable ? 'STABLE' : 'UNSTABLE'}
        </strong>
        {' '}|λ|max = {maxAbsClosed.toFixed(3)}
      </div>
      {result.warnings.length > 0 && (
        <ul className="list-disc pl-4 text-amber-700">
          {result.warnings.map((w, i) => (
            <li key={i}>{w}</li>
          ))}
        </ul>
      )}
    </div>
  )
}
