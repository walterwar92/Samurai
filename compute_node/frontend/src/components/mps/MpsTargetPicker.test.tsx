import { describe, it, expect, vi } from 'vitest'
import { render, screen, fireEvent, act } from '@testing-library/react'
import { MpsTargetPicker } from './MpsTargetPicker'

// R3F/WebGL в jsdom не работает — подменяем сцену заглушкой, которая
// умеет вызвать onPick (имитация клика по полу под углом π/4).
vi.mock('./MpsTargetScene', () => ({
  MpsTargetScene: ({ onPick }: { onPick: (a: number) => void }) => (
    <button data-testid="scene-stub" onClick={() => onPick(Math.PI / 4)}>
      scene
    </button>
  ),
}))

describe('MpsTargetPicker', () => {
  it('рендерит шапку и кнопку «Старт»', () => {
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={vi.fn()} onCancel={vi.fn()} />,
    )
    expect(screen.getByText(/Финальный курс после прибытия/i)).toBeInTheDocument()
    expect(screen.getByRole('button', { name: /Старт/i })).toBeInTheDocument()
  })

  it('«Старт» с предвыбранным φ=0 зовёт onConfirm(0)', () => {
    const onConfirm = vi.fn()
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={onConfirm} onCancel={vi.fn()} />,
    )
    act(() => { screen.getByRole('button', { name: /Старт/i }).click() })
    expect(onConfirm).toHaveBeenCalledWith(0)
  })

  it('после выбора точки «Старт» зовёт onConfirm с этим углом', () => {
    const onConfirm = vi.fn()
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={onConfirm} onCancel={vi.fn()} />,
    )
    act(() => { screen.getByTestId('scene-stub').click() })   // onPick(π/4)
    act(() => { screen.getByRole('button', { name: /Старт/i }).click() })
    expect(onConfirm).toHaveBeenCalledWith(Math.PI / 4)
  })

  it('readout курса обновляется после выбора точки', () => {
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={vi.fn()} onCancel={vi.fn()} />,
    )
    expect(screen.getByText('прямо')).toBeInTheDocument()
    act(() => { screen.getByTestId('scene-stub').click() })   // onPick(π/4)
    expect(screen.getByText('+45°')).toBeInTheDocument()
  })

  it('✕ зовёт onCancel', () => {
    const onCancel = vi.fn()
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={vi.fn()} onCancel={onCancel} />,
    )
    act(() => { screen.getByRole('button', { name: /Закрыть выбор цели/i }).click() })
    expect(onCancel).toHaveBeenCalled()
  })

  it('клик по backdrop зовёт onCancel', () => {
    const onCancel = vi.fn()
    render(
      <MpsTargetPicker distance={2} vTarget={0.15} onConfirm={vi.fn()} onCancel={onCancel} />,
    )
    act(() => { fireEvent.click(screen.getByTestId('mps-target-backdrop')) })
    expect(onCancel).toHaveBeenCalled()
  })
})
