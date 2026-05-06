import { Input } from '@/components/ui/input'
import { useMpsHighlight } from '@/hooks/useMpsHighlight'
import { CANONICAL_PATTERN_A, CANONICAL_PATTERN_B } from '@/lib/mps/canonical'
import type { MatrixName, EquationIndex } from './HighlightContext'

interface MatrixGridProps {
  matrix: MatrixName
  values: number[][]
  applied?: number[][]
  rowLabels: string[]
  colLabels: string[]
  onCell: (row: number, col: number, value: string) => void
}

function isValidNumber(value: number): boolean {
  return Number.isFinite(value)
}

function isCanonicalCell(matrix: MatrixName, row: number, col: number): boolean {
  if (matrix === 'A') {
    return CANONICAL_PATTERN_A.some((p) => p.row === row && p.col === col)
  }
  if (matrix === 'B') {
    return CANONICAL_PATTERN_B.some((p) => p.row === row && p.col === col)
  }
  return false
}

function isPatternZero(matrix: MatrixName, row: number, col: number): boolean {
  if (matrix !== 'A' && matrix !== 'B') return false
  return !isCanonicalCell(matrix, row, col)
}

export function MatrixGrid({
  matrix,
  values,
  applied,
  rowLabels,
  colLabels,
  onCell,
}: MatrixGridProps) {
  const { hovered, setCell, setEquation } = useMpsHighlight()
  const cols = values[0]?.length ?? 0
  const showCanonicalLegend = matrix === 'A' || matrix === 'B'

  return (
    <div className="space-y-2">
      <div
        className="grid gap-1"
        style={{
          gridTemplateColumns: `auto repeat(${cols}, minmax(0, 1fr))`,
        }}
      >
        <div />
        {colLabels.map((lbl, j) => (
          <div
            key={`col-${j}`}
            className={[
              'text-xs font-mono text-center py-1 select-none',
              hovered.cell?.matrix === matrix && hovered.cell.col === j
                ? 'text-foreground font-semibold'
                : 'text-muted-foreground',
            ].join(' ')}
          >
            {lbl}
          </div>
        ))}

        {values.map((row, i) => (
          <FragmentRow
            key={`row-${i}`}
            matrix={matrix}
            rowIndex={i}
            rowLabel={rowLabels[i]}
            row={row}
            appliedRow={applied?.[i]}
            highlightedEquation={hovered.equation}
            highlightedCell={hovered.cell}
            onCell={onCell}
            setCell={setCell}
            setEquation={setEquation}
          />
        ))}
      </div>
      {showCanonicalLegend && (
        <div className="text-xs text-muted-foreground">
          <span className="font-mono">¹</span> — каноническая ячейка (привязана к τ_v / τ_ω)
        </div>
      )}
    </div>
  )
}

interface FragmentRowProps {
  matrix: MatrixName
  rowIndex: number
  rowLabel: string
  row: number[]
  appliedRow: number[] | undefined
  highlightedEquation: number | null
  highlightedCell: { matrix: MatrixName; row: number; col: number } | null
  onCell: (row: number, col: number, value: string) => void
  setCell: (cell: { matrix: MatrixName; row: number; col: number } | null) => void
  setEquation: (i: EquationIndex | null) => void
}

function FragmentRow({
  matrix,
  rowIndex,
  rowLabel,
  row,
  appliedRow,
  highlightedEquation,
  highlightedCell,
  onCell,
  setCell,
  setEquation,
}: FragmentRowProps) {
  const rowHighlighted =
    highlightedEquation === rowIndex ||
    (highlightedCell?.matrix === matrix && highlightedCell.row === rowIndex)

  const canSetEquation = (matrix === 'A' || matrix === 'B') && rowIndex >= 0 && rowIndex <= 4

  return (
    <>
      <div
        className={[
          'text-xs font-mono py-1 pr-2 select-none cursor-default',
          rowHighlighted ? 'text-foreground font-semibold' : 'text-muted-foreground',
        ].join(' ')}
        onMouseEnter={() => {
          if (canSetEquation) setEquation(rowIndex as EquationIndex)
        }}
        onMouseLeave={() => {
          if (canSetEquation) setEquation(null)
        }}
      >
        {rowLabel}
      </div>
      {row.map((cell, j) => {
        const text = String(cell)
        const valid = isValidNumber(cell)
        const dirty = appliedRow !== undefined && appliedRow[j] !== cell
        const canonical = isCanonicalCell(matrix, rowIndex, j)
        const deviation =
          (matrix === 'A' || matrix === 'B') &&
          isPatternZero(matrix, rowIndex, j) &&
          isValidNumber(cell) &&
          Math.abs(cell) > 1e-9
        const cellHighlighted =
          highlightedCell?.matrix === matrix &&
          highlightedCell.row === rowIndex &&
          highlightedCell.col === j
        const colHighlighted =
          highlightedCell?.matrix === matrix && highlightedCell.col === j

        return (
          <div
            key={`cell-${rowIndex}-${j}`}
            className="relative"
            data-canonical={canonical || undefined}
            data-dirty={dirty || undefined}
            data-invalid={!valid || undefined}
            data-deviation={deviation || undefined}
            onMouseEnter={() => setCell({ matrix, row: rowIndex, col: j })}
            onMouseLeave={() => setCell(null)}
          >
            <Input
              value={text}
              onChange={(e) => onCell(rowIndex, j, e.target.value)}
              className={[
                'h-7 text-xs font-mono px-1',
                !valid ? 'border-red-500' : '',
                dirty && valid ? 'bg-amber-500/10' : '',
                deviation ? 'border-orange-400 border-dashed' : '',
                cellHighlighted ? 'ring-2 ring-primary/60' : '',
                !cellHighlighted && (rowHighlighted || colHighlighted)
                  ? 'bg-primary/5'
                  : '',
              ].join(' ')}
              aria-invalid={!valid}
              aria-label={`${matrix}[${rowIndex},${j}]`}
            />
            {canonical && (
              <span className="absolute -top-0.5 -right-0.5 text-[10px] font-mono text-primary/60 pointer-events-none">
                ¹
              </span>
            )}
          </div>
        )
      })}
    </>
  )
}
