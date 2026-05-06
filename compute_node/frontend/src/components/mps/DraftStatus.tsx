import type { MpsMatrices } from '@/types/mps'
import type { CanonicalStatus } from '@/lib/mps/canonical'

interface DraftStatusProps {
  applied: MpsMatrices | null
  draft: MpsMatrices | null
  isStable?: boolean
  isValid?: boolean
  canonicalStatus?: CanonicalStatus
}

const BADGE = 'inline-flex items-center px-2 py-0.5 rounded-full text-xs font-medium'

export function DraftStatus({
  applied,
  draft,
  isStable = true,
  isValid = true,
  canonicalStatus,
}: DraftStatusProps) {
  if (!isValid) {
    return (
      <span className={`${BADGE} bg-red-500/20 text-red-700`} role="status">
        invalid
      </span>
    )
  }
  if (!isStable) {
    return (
      <span className={`${BADGE} bg-orange-500/20 text-orange-700`} role="status">
        unstable
      </span>
    )
  }
  if (canonicalStatus === 'non_canonical') {
    return (
      <span className={`${BADGE} bg-orange-500/20 text-orange-700`} role="status">
        non-canonical
      </span>
    )
  }
  if (canonicalStatus === 'incoherent') {
    return (
      <span className={`${BADGE} bg-amber-500/20 text-amber-700`} role="status">
        incoherent
      </span>
    )
  }
  if (draft && JSON.stringify(draft) !== JSON.stringify(applied)) {
    return (
      <span className={`${BADGE} bg-amber-500/20 text-amber-700`} role="status">
        draft (unsaved)
      </span>
    )
  }
  return (
    <span className={`${BADGE} bg-green-500/20 text-green-700`} role="status">
      applied
    </span>
  )
}
