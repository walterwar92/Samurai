import { describe, it, expect } from 'vitest'
import { computeNewScale } from './CalibrationPanel'

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
