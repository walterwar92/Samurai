import { describe, it, expect } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { vi, beforeEach } from 'vitest'
import { computeNewScale } from './CalibrationPanel'
import { CalibrationPanel } from './CalibrationPanel'

// Мокаем '@/lib/api' — нам нужно проверить setCalibration call.
vi.mock('@/lib/api', () => ({
  api: {
    setCalibration: vi.fn(),
    listCalibrationProfiles: vi.fn(),
    saveCalibrationProfile: vi.fn(),
    loadCalibrationProfile: vi.fn(),
    deleteCalibrationProfile: vi.fn(),
  },
}))

import { api } from '@/lib/api'

describe('computeNewScale', () => {
  it('масштабирует scale пропорционально измеренному', () => {
    const result = computeNewScale(1.235, 2.0, 2.18)
    expect(result).not.toBeNull()
    expect(result!).toBeCloseTo(1.3461, 3)
  })

  it('возвращает null при D_target = 0', () => {
    expect(computeNewScale(1.235, 0, 2.18)).toBeNull()
  })

  it('возвращает null при D_measured = 0', () => {
    expect(computeNewScale(1.235, 2.0, 0)).toBeNull()
  })

  it('возвращает null при отрицательном D_target', () => {
    expect(computeNewScale(1.235, -2.0, 2.18)).toBeNull()
  })

  it('возвращает null при NaN', () => {
    expect(computeNewScale(1.235, NaN, 2.18)).toBeNull()
    expect(computeNewScale(1.235, 2.0, NaN)).toBeNull()
    expect(computeNewScale(NaN, 2.0, 2.18)).toBeNull()
  })

  it('возвращает null при Infinity', () => {
    expect(computeNewScale(1.235, Infinity, 2.18)).toBeNull()
    expect(computeNewScale(1.235, 2.0, Infinity)).toBeNull()
  })

  it('точно повторяет старый scale при равных D', () => {
    expect(computeNewScale(1.235, 2.0, 2.0)).toBeCloseTo(1.235, 6)
  })
})

const coeffs = {
  profile: 'default',
  scale_fwd: 1.235,
  scale_bwd: 0.988,
  motor_trim: -12.003,
}

describe('CalibrationPanel — калькулятор', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('рендерит подсказку "только в режиме Robot"', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    expect(screen.getByText(/Применяется только в режиме Robot/i)).toBeInTheDocument()
  })

  it('калькулятор показывает превью при валидном вводе', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '2.0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '2.18' } })
    // Превью отображает текущий 1.235 → новый ~1.346
    expect(screen.getByTestId('calc-new-scale')).toHaveTextContent(/1\.346/)
    expect(screen.getByTestId('calc-current-scale')).toHaveTextContent(/1\.235/)
  })

  it('калькулятор подставляет в FWD при direction=fwd', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '2.0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '2.18' } })
    fireEvent.click(screen.getByRole('button', { name: /Подставить в FWD/i }))
    const fwdInput = screen.getByLabelText('FWD') as HTMLInputElement
    expect(parseFloat(fwdInput.value)).toBeCloseTo(1.3461, 3)
  })

  it('калькулятор подставляет в BWD при direction=bwd', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.click(screen.getByLabelText(/Назад/i))
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '1.0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '0.9' } })
    fireEvent.click(screen.getByRole('button', { name: /Подставить в BWD/i }))
    const bwdInput = screen.getByLabelText('BWD') as HTMLInputElement
    expect(parseFloat(bwdInput.value)).toBeCloseTo(0.8892, 3)
  })

  it('кнопка Подставить disabled при невалидном вводе', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '2.18' } })
    const btn = screen.getByRole('button', { name: /Подставить в FWD/i })
    expect(btn).toBeDisabled()
  })

  it('после Подставить → Применить вызывает api.setCalibration с правильными аргументами', () => {
    render(<CalibrationPanel coeffs={coeffs} profiles={null} />)
    fireEvent.change(screen.getByLabelText('D заданное, м'), { target: { value: '2.0' } })
    fireEvent.change(screen.getByLabelText('D измеренное, м'), { target: { value: '2.18' } })
    fireEvent.click(screen.getByRole('button', { name: /Подставить в FWD/i }))
    fireEvent.click(screen.getByRole('button', { name: /Применить/i }))
    expect(api.setCalibration).toHaveBeenCalledTimes(1)
    const call = (api.setCalibration as ReturnType<typeof vi.fn>).mock.calls[0]
    expect(call[0]).toBeCloseTo(1.3461, 3)        // fwd
    expect(call[1]).toBeCloseTo(0.988, 3)         // bwd (не менялся)
    expect(call[2]).toBeCloseTo(-12.003, 3)       // trim (не менялся)
  })
})
