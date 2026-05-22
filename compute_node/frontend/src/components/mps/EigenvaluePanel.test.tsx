import { describe, it, expect } from 'vitest'
import { render, screen } from '@testing-library/react'
import { EigenvaluePanel } from './EigenvaluePanel'
import type { ComplexNumber } from '@/types/mps'

// Pydantic v2 в режиме JSON сериализует NaN/Inf как null, поэтому фронт
// должен корректно отрендерить eigenvalues с null-полями (вместо креша на
// `null.toFixed`). См. mps_runner.closed_loop_eigenvalues: при падении
// MPCController возвращает [complex('nan')] * N.
//
// Жёсткое требование: рендер EigenvaluePanel НИКОГДА не должен бросать
// TypeError, даже если бэкенд прислал re/im = null или NaN.

const finite: ComplexNumber = { re: 0.5, im: 0.1 }
// JSON.parse сохраняет null как null; через тип ComplexNumber приходится
// явно «прокидывать» null — это симулирует то, что реально приходит из API.
const nullRe = { re: null as unknown as number, im: 0.0 } as ComplexNumber
const bothNull = {
  re: null as unknown as number,
  im: null as unknown as number,
} as ComplexNumber
const nanZ: ComplexNumber = { re: NaN, im: NaN }

describe('EigenvaluePanel — non-finite eigenvalues', () => {
  it('renders without throwing when an eigenvalue has re=null', () => {
    expect(() =>
      render(
        <EigenvaluePanel
          open={[finite]}
          closed={[nullRe]}
          isPlantStable
          isClosedLoopStable={false}
        />,
      ),
    ).not.toThrow()
  })

  it('renders without throwing when both re and im are null', () => {
    expect(() =>
      render(
        <EigenvaluePanel
          open={[bothNull]}
          closed={[bothNull]}
          isPlantStable={false}
          isClosedLoopStable={false}
        />,
      ),
    ).not.toThrow()
  })

  it('renders without throwing when re/im are NaN', () => {
    expect(() =>
      render(
        <EigenvaluePanel
          open={[nanZ]}
          closed={[nanZ]}
          isPlantStable={false}
          isClosedLoopStable={false}
        />,
      ),
    ).not.toThrow()
  })

  it('shows a "не вычислено" marker for non-finite eigenvalues', () => {
    render(
      <EigenvaluePanel
        open={[finite]}
        closed={[bothNull]}
        isPlantStable
        isClosedLoopStable={false}
      />,
    )
    // Есть и хотя бы одна валидная запись (open) и одна нерасчётная (closed)
    expect(screen.getByText(/не вычислено/i)).toBeInTheDocument()
  })

  it('still renders finite eigenvalues correctly alongside non-finite ones', () => {
    render(
      <EigenvaluePanel
        open={[finite, bothNull]}
        closed={[]}
        isPlantStable={false}
        isClosedLoopStable={false}
      />,
    )
    // formatComplex(finite) → "0.500 + 0.100i"
    expect(screen.getByText(/0\.500/)).toBeInTheDocument()
  })
})
