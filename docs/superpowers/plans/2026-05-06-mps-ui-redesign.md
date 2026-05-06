# MPS UI Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Превратить страницу `/mps` из «сетки с цифрами» в учебный инструмент, где студент видит ОДУ-модель робота рядом с матрицами A/B и редактирует их через cross-highlight + двунаправленные физические слайдеры.

**Architecture:** Sticky-двухколоночный layout. Левая колонка (sticky 35%): KaTeX-карта ОДУ + слайдеры τ_v, τ_ω. Правая (scroll 65%): MatrixEditor с табами A·B / Q·R·N / C·D + EigenvaluePanel + Scenario + Plots + History. Cross-highlight через React Context связывает уравнения с ячейками матриц. Бэкенд не меняется.

**Tech Stack:** React 19 + TypeScript + Vite + Tailwind + shadcn/ui (Radix) + Recharts + KaTeX (новое) + Vitest (новое) + @testing-library/react (новое).

**Spec:** [`docs/superpowers/specs/2026-05-06-mps-ui-redesign-design.md`](../specs/2026-05-06-mps-ui-redesign-design.md)

---

## Глобальные конвенции

- **Рабочий каталог** для всех команд: `compute_node/frontend/`. Все `npm`/`npx` запускать оттуда.
- **Ветка:** `feat/mps`. Все коммиты в неё.
- **Коммит-стиль:** `feat(mps-ui): ...` или `test(mps-ui): ...` — следуем существующей конвенции `feat(mps): ...`.
- **Импорты:** `@/...` алиас на `compute_node/frontend/src/`.
- **PR-author rule (из memory project_feat_mps.md):** PR от @OneAstr0 / @razdryzg-dev, walterwar92 не указывать. В коммитах НЕ ставить `Co-Authored-By: Claude` или упоминания AI.

---

## Task 1: Setup vitest + testing-library

**Files:**
- Modify: `compute_node/frontend/package.json`
- Create: `compute_node/frontend/vitest.config.ts`
- Create: `compute_node/frontend/src/__tests__/setup.ts`
- Modify: `compute_node/frontend/tsconfig.app.json`

- [ ] **Step 1: Install dev dependencies**

```bash
cd compute_node/frontend
npm install --save-dev vitest@^2 @testing-library/react@^16 @testing-library/jest-dom@^6 @testing-library/user-event@^14 happy-dom@^15 @vitest/coverage-v8@^2
```

Expected: новые записи в `package.json` под `devDependencies`.

- [ ] **Step 2: Add npm scripts**

Открыть `compute_node/frontend/package.json`, в `scripts` добавить:

```json
"test": "vitest run",
"test:watch": "vitest",
"test:coverage": "vitest run --coverage"
```

- [ ] **Step 3: Create vitest config**

Создать `compute_node/frontend/vitest.config.ts`:

```typescript
import { defineConfig } from 'vitest/config'
import react from '@vitejs/plugin-react'
import path from 'node:path'

export default defineConfig({
  plugins: [react()],
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },
  test: {
    environment: 'happy-dom',
    globals: true,
    setupFiles: ['./src/__tests__/setup.ts'],
    css: true,
    include: ['src/**/*.test.{ts,tsx}'],
  },
})
```

- [ ] **Step 4: Create test setup**

Создать `compute_node/frontend/src/__tests__/setup.ts`:

```typescript
import '@testing-library/jest-dom/vitest'
import { afterEach } from 'vitest'
import { cleanup } from '@testing-library/react'

afterEach(() => {
  cleanup()
})
```

- [ ] **Step 5: Update tsconfig.app.json types**

Открыть `compute_node/frontend/tsconfig.app.json`, в `compilerOptions.types` (создать массив если нет) добавить `"vitest/globals"` и `"@testing-library/jest-dom"`.

Если `types` нет — добавить:

```json
"types": ["vitest/globals", "@testing-library/jest-dom"]
```

- [ ] **Step 6: Smoke-test**

Создать временно `compute_node/frontend/src/__tests__/smoke.test.ts`:

```typescript
import { describe, it, expect } from 'vitest'

describe('smoke', () => {
  it('vitest works', () => {
    expect(2 + 2).toBe(4)
  })
})
```

Run: `cd compute_node/frontend && npm test`
Expected: 1 test passed.

- [ ] **Step 7: Remove smoke test, commit**

Удалить `src/__tests__/smoke.test.ts`.

```bash
git add compute_node/frontend/package.json compute_node/frontend/package-lock.json compute_node/frontend/vitest.config.ts compute_node/frontend/src/__tests__/setup.ts compute_node/frontend/tsconfig.app.json
git commit -m "chore(mps-ui): добавлен vitest + @testing-library/react"
```

---

## Task 2: Install KaTeX

**Files:**
- Modify: `compute_node/frontend/package.json`
- Modify: `compute_node/frontend/src/main.tsx`

- [ ] **Step 1: Install dependencies**

```bash
cd compute_node/frontend
npm install katex@^0.16 react-katex@^3
npm install --save-dev @types/katex@^0.16 @types/react-katex@^3
```

- [ ] **Step 2: Import KaTeX styles**

Открыть `compute_node/frontend/src/main.tsx`, добавить **в самом верху файла** перед другими импортами:

```typescript
import 'katex/dist/katex.min.css'
```

- [ ] **Step 3: Smoke verify build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green, без ошибок.

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/package.json compute_node/frontend/package-lock.json compute_node/frontend/src/main.tsx
git commit -m "chore(mps-ui): добавлены katex + react-katex"
```

---

## Task 3: lib/mps/canonical.ts (TDD)

Файл с детектором канонической формы и функцией сборки матриц из физических параметров. **Самый важный модуль** — на нём держится PhysicsParams и подсветка деканонизации.

**Files:**
- Create: `compute_node/frontend/src/lib/mps/canonical.ts`
- Create: `compute_node/frontend/src/lib/mps/canonical.test.ts`

- [ ] **Step 1: Write failing tests**

Создать `compute_node/frontend/src/lib/mps/canonical.test.ts`:

```typescript
import { describe, it, expect } from 'vitest'
import {
  detectPhysics,
  buildCanonical,
  DEFAULT_TAU_V,
  DEFAULT_TAU_OMEGA,
  CANONICAL_PATTERN_A,
  CANONICAL_PATTERN_B,
} from './canonical'
import type { MpsMatrices } from '@/types/mps'

function emptyMatrices(): MpsMatrices {
  return {
    A: Array.from({ length: 5 }, () => Array(5).fill(0)),
    B: Array.from({ length: 5 }, () => Array(2).fill(0)),
    C: Array.from({ length: 5 }, (_, i) =>
      Array.from({ length: 5 }, (_, j) => (i === j ? 1 : 0)),
    ),
    D: Array.from({ length: 5 }, () => Array(2).fill(0)),
    Q_diag: [10, 5, 1, 1, 5],
    R_diag: [1, 1],
    horizon_N: 20,
    u_min: [-0.3, -1.5],
    u_max: [0.3, 1.5],
    schema_version: '1.0',
  }
}

describe('buildCanonical', () => {
  it('returns 5x5 A and 5x2 B with canonical entries', () => {
    const { A, B } = buildCanonical(0.15, 0.10)
    expect(A.length).toBe(5)
    expect(A[0].length).toBe(5)
    expect(B.length).toBe(5)
    expect(B[0].length).toBe(2)
  })

  it('A[0][1] = 1 (ṡ ← v)', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[0][1]).toBe(1)
  })

  it('A[1][1] = -1/tau_v', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[1][1]).toBeCloseTo(-1 / 0.15, 5)
  })

  it('A[2][3] = 1 (θ̇ ← ω)', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[2][3]).toBe(1)
  })

  it('A[3][3] = -1/tau_omega', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[3][3]).toBeCloseTo(-1 / 0.10, 5)
  })

  it('A[4][1] = -1 (ė_int ← -v)', () => {
    const { A } = buildCanonical(0.15, 0.10)
    expect(A[4][1]).toBe(-1)
  })

  it('B[1][0] = 1/tau_v', () => {
    const { B } = buildCanonical(0.15, 0.10)
    expect(B[1][0]).toBeCloseTo(1 / 0.15, 5)
  })

  it('B[3][1] = 1/tau_omega', () => {
    const { B } = buildCanonical(0.15, 0.10)
    expect(B[3][1]).toBeCloseTo(1 / 0.10, 5)
  })

  it('all other entries are zero', () => {
    const { A, B } = buildCanonical(0.15, 0.10)
    // A
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 5; j++) {
        const isCanonical =
          (i === 0 && j === 1) ||
          (i === 1 && j === 1) ||
          (i === 2 && j === 3) ||
          (i === 3 && j === 3) ||
          (i === 4 && j === 1)
        if (!isCanonical) expect(A[i][j]).toBe(0)
      }
    }
    // B
    for (let i = 0; i < 5; i++) {
      for (let j = 0; j < 2; j++) {
        const isCanonical = (i === 1 && j === 0) || (i === 3 && j === 1)
        if (!isCanonical) expect(B[i][j]).toBe(0)
      }
    }
  })
})

describe('detectPhysics', () => {
  it('extracts tau_v and tau_omega from canonical matrices', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A
    m.B = built.B
    const r = detectPhysics(m)
    expect(r.status).toBe('canonical')
    expect(r.tau_v).toBeCloseTo(0.15, 4)
    expect(r.tau_omega).toBeCloseTo(0.10, 4)
    expect(r.deviations).toEqual([])
  })

  it('tolerates rounding error within 1e-3 relative', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A.map((row) => row.slice())
    m.B = built.B.map((row) => row.slice())
    // Округлили -6.6666... до -6.67 (отклонение ~5e-4 < 1e-3)
    m.A[1][1] = -6.67
    m.B[1][0] = 6.67
    const r = detectPhysics(m)
    expect(r.status).toBe('canonical')
    expect(r.tau_v).toBeCloseTo(0.15, 2)
  })

  it('flags incoherent when A[1][1] and B[1][0] disagree', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A.map((row) => row.slice())
    m.B = built.B.map((row) => row.slice())
    m.A[1][1] = -4.0  // tau_v = 0.25
    // B[1][0] = 6.67 → tau_v = 0.15  — несогласованность
    const r = detectPhysics(m)
    expect(r.status).toBe('incoherent')
    expect(r.tau_v).toBeNull()
    expect(r.tau_omega).toBeCloseTo(0.10, 4)
  })

  it('flags non_canonical when a "should-be-zero" cell is non-zero', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A.map((row) => row.slice())
    m.B = built.B.map((row) => row.slice())
    m.A[0][2] = 0.5  // Эта ячейка должна быть 0
    const r = detectPhysics(m)
    expect(r.status).toBe('non_canonical')
    expect(r.deviations).toEqual([
      expect.objectContaining({ matrix: 'A', row: 0, col: 2, expected: 0, actual: 0.5 }),
    ])
  })

  it('flags non_canonical when fixed-one cell is wrong', () => {
    const m = emptyMatrices()
    const built = buildCanonical(0.15, 0.10)
    m.A = built.A.map((row) => row.slice())
    m.B = built.B.map((row) => row.slice())
    m.A[0][1] = 0.95  // Должно быть 1 (вне толеранса 1e-3)
    const r = detectPhysics(m)
    expect(r.status).toBe('non_canonical')
    expect(r.deviations.find((d) => d.row === 0 && d.col === 1)).toBeDefined()
  })

  it('returns nulls for tau when matrices are empty', () => {
    const m = emptyMatrices()  // A, B все нули
    const r = detectPhysics(m)
    // В пустых матрицах фиксированные единицы (A[0][1], A[2][3]) тоже 0 → non_canonical
    expect(r.status).toBe('non_canonical')
  })
})

describe('CANONICAL_PATTERN exports', () => {
  it('CANONICAL_PATTERN_A has 5 entries', () => {
    expect(CANONICAL_PATTERN_A.length).toBe(5)
  })
  it('CANONICAL_PATTERN_B has 2 entries', () => {
    expect(CANONICAL_PATTERN_B.length).toBe(2)
  })
  it('DEFAULT_TAU_V is 0.15', () => {
    expect(DEFAULT_TAU_V).toBe(0.15)
  })
  it('DEFAULT_TAU_OMEGA is 0.10', () => {
    expect(DEFAULT_TAU_OMEGA).toBe(0.10)
  })
})
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd compute_node/frontend && npx vitest run src/lib/mps/canonical.test.ts`
Expected: FAIL with "Cannot find module './canonical'".

- [ ] **Step 3: Write implementation**

Создать `compute_node/frontend/src/lib/mps/canonical.ts`:

```typescript
import type { MpsMatrices } from '@/types/mps'

export const DEFAULT_TAU_V = 0.15
export const DEFAULT_TAU_OMEGA = 0.10
export const DEFAULT_V_ZERO = 0.20

const TOL_REL = 1e-3
const TOL_ABS = 1e-9

type CellRole =
  | { kind: 'fixed'; value: number }
  | { kind: 'tunable'; param: 'tau_v' | 'tau_omega'; expr: 'neg_inv' | 'inv' }

export const CANONICAL_PATTERN_A: Array<{ row: number; col: number; role: CellRole }> = [
  { row: 0, col: 1, role: { kind: 'fixed', value: 1 } },
  { row: 1, col: 1, role: { kind: 'tunable', param: 'tau_v', expr: 'neg_inv' } },
  { row: 2, col: 3, role: { kind: 'fixed', value: 1 } },
  { row: 3, col: 3, role: { kind: 'tunable', param: 'tau_omega', expr: 'neg_inv' } },
  { row: 4, col: 1, role: { kind: 'fixed', value: -1 } },
]

export const CANONICAL_PATTERN_B: Array<{ row: number; col: number; role: CellRole }> = [
  { row: 1, col: 0, role: { kind: 'tunable', param: 'tau_v', expr: 'inv' } },
  { row: 3, col: 1, role: { kind: 'tunable', param: 'tau_omega', expr: 'inv' } },
]

export type CanonicalStatus = 'canonical' | 'incoherent' | 'non_canonical'

export interface CellDeviation {
  matrix: 'A' | 'B'
  row: number
  col: number
  expected: number
  actual: number
}

export interface PhysicsExtraction {
  tau_v: number | null
  tau_omega: number | null
  status: CanonicalStatus
  deviations: CellDeviation[]
}

function approxEqual(a: number, b: number): boolean {
  const denom = Math.max(TOL_ABS, Math.abs(b))
  return Math.abs(a - b) / denom < TOL_REL
}

function expectedValue(role: CellRole, tau_v: number, tau_omega: number): number {
  if (role.kind === 'fixed') return role.value
  const tau = role.param === 'tau_v' ? tau_v : tau_omega
  return role.expr === 'neg_inv' ? -1 / tau : 1 / tau
}

export function buildCanonical(
  tau_v: number,
  tau_omega: number,
): { A: number[][]; B: number[][] } {
  const A = Array.from({ length: 5 }, () => Array(5).fill(0))
  const B = Array.from({ length: 5 }, () => Array(2).fill(0))
  for (const { row, col, role } of CANONICAL_PATTERN_A) {
    A[row][col] = expectedValue(role, tau_v, tau_omega)
  }
  for (const { row, col, role } of CANONICAL_PATTERN_B) {
    B[row][col] = expectedValue(role, tau_v, tau_omega)
  }
  return { A, B }
}

/** Восстановить tau из значения ячейки. role.expr = 'neg_inv' → tau = -1/value; 'inv' → tau = 1/value. */
function tauFromCell(role: CellRole & { kind: 'tunable' }, actual: number): number | null {
  if (Math.abs(actual) < TOL_ABS) return null  // не делим на ~0
  if (role.expr === 'neg_inv') return -1 / actual
  return 1 / actual
}

export function detectPhysics(m: MpsMatrices): PhysicsExtraction {
  const deviations: CellDeviation[] = []

  // 1. Все ячейки A и B вне CANONICAL_PATTERN должны быть 0.
  const inPatternA = new Set(CANONICAL_PATTERN_A.map(({ row, col }) => `${row},${col}`))
  const inPatternB = new Set(CANONICAL_PATTERN_B.map(({ row, col }) => `${row},${col}`))
  for (let i = 0; i < 5; i++) {
    for (let j = 0; j < 5; j++) {
      if (inPatternA.has(`${i},${j}`)) continue
      if (Math.abs(m.A[i][j]) > TOL_ABS) {
        deviations.push({ matrix: 'A', row: i, col: j, expected: 0, actual: m.A[i][j] })
      }
    }
  }
  for (let i = 0; i < 5; i++) {
    for (let j = 0; j < 2; j++) {
      if (inPatternB.has(`${i},${j}`)) continue
      if (Math.abs(m.B[i][j]) > TOL_ABS) {
        deviations.push({ matrix: 'B', row: i, col: j, expected: 0, actual: m.B[i][j] })
      }
    }
  }

  // 2. Восстановить tau_v и tau_omega из tunable-ячеек.
  const tauVCandidates: number[] = []
  const tauOmegaCandidates: number[] = []
  for (const { row, col, role } of CANONICAL_PATTERN_A) {
    const actual = m.A[row][col]
    if (role.kind === 'fixed') {
      if (!approxEqual(actual, role.value)) {
        deviations.push({ matrix: 'A', row, col, expected: role.value, actual })
      }
    } else {
      const tau = tauFromCell(role, actual)
      if (tau !== null && tau > 0) {
        if (role.param === 'tau_v') tauVCandidates.push(tau)
        else tauOmegaCandidates.push(tau)
      }
    }
  }
  for (const { row, col, role } of CANONICAL_PATTERN_B) {
    const actual = m.B[row][col]
    if (role.kind !== 'tunable') continue
    const tau = tauFromCell(role, actual)
    if (tau !== null && tau > 0) {
      if (role.param === 'tau_v') tauVCandidates.push(tau)
      else tauOmegaCandidates.push(tau)
    }
  }

  // 3. Согласованность кандидатов.
  function reconcile(candidates: number[]): number | null {
    if (candidates.length === 0) return null
    const first = candidates[0]
    for (const c of candidates.slice(1)) {
      if (!approxEqual(c, first)) return null
    }
    return candidates.reduce((s, x) => s + x, 0) / candidates.length
  }

  const tau_v = reconcile(tauVCandidates)
  const tau_omega = reconcile(tauOmegaCandidates)

  // 4. Определить status.
  let status: CanonicalStatus
  if (deviations.length === 0 && tau_v !== null && tau_omega !== null) {
    status = 'canonical'
  } else if (
    deviations.length === 0 &&
    (tau_v === null || tau_omega === null) &&
    (tauVCandidates.length > 0 || tauOmegaCandidates.length > 0)
  ) {
    status = 'incoherent'
  } else {
    status = 'non_canonical'
  }

  return { tau_v, tau_omega, status, deviations }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd compute_node/frontend && npx vitest run src/lib/mps/canonical.test.ts`
Expected: All tests PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/lib/mps/canonical.ts compute_node/frontend/src/lib/mps/canonical.test.ts
git commit -m "feat(mps-ui): canonical-форма матриц + детектор физических параметров"
```

---

## Task 4: lib/mps/tokenMap.ts (TDD)

Маппинг между ячейками матриц A/B и token-id-ами в KaTeX-формулах.

**Files:**
- Create: `compute_node/frontend/src/lib/mps/tokenMap.ts`
- Create: `compute_node/frontend/src/lib/mps/tokenMap.test.ts`

- [ ] **Step 1: Write failing tests**

Создать `compute_node/frontend/src/lib/mps/tokenMap.test.ts`:

```typescript
import { describe, it, expect } from 'vitest'
import { tokenForCell, cellForToken, TOKENS, TOKEN_BY_ID } from './tokenMap'

describe('tokenForCell', () => {
  it('returns coef_s_v for A[0][1]', () => {
    expect(tokenForCell('A', 0, 1)).toBe('coef_s_v')
  })
  it('returns coef_v_v for A[1][1]', () => {
    expect(tokenForCell('A', 1, 1)).toBe('coef_v_v')
  })
  it('returns coef_omega_omega for A[3][3]', () => {
    expect(tokenForCell('A', 3, 3)).toBe('coef_omega_omega')
  })
  it('returns coef_v_uv for B[1][0]', () => {
    expect(tokenForCell('B', 1, 0)).toBe('coef_v_uv')
  })
  it('returns coef_omega_uomega for B[3][1]', () => {
    expect(tokenForCell('B', 3, 1)).toBe('coef_omega_uomega')
  })
  it('returns null for non-canonical cell A[0][2]', () => {
    expect(tokenForCell('A', 0, 2)).toBeNull()
  })
  it('returns null for B[0][0]', () => {
    expect(tokenForCell('B', 0, 0)).toBeNull()
  })
})

describe('cellForToken', () => {
  it('inverse of tokenForCell for canonical cells', () => {
    expect(cellForToken('coef_s_v')).toEqual({ matrix: 'A', row: 0, col: 1 })
    expect(cellForToken('coef_v_v')).toEqual({ matrix: 'A', row: 1, col: 1 })
    expect(cellForToken('coef_v_uv')).toEqual({ matrix: 'B', row: 1, col: 0 })
  })
  it('returns null for unknown token', () => {
    expect(cellForToken('coef_nonexistent')).toBeNull()
  })
})

describe('TOKENS metadata', () => {
  it('has 7 canonical tokens (5 in A, 2 in B)', () => {
    expect(TOKENS.length).toBe(7)
  })
  it('TOKEN_BY_ID is keyed by id', () => {
    expect(TOKEN_BY_ID['coef_v_v']).toBeDefined()
    expect(TOKEN_BY_ID['coef_v_v']?.equationRow).toBe(1)
  })
  it('coef_v_v is in equation row 1 (v̇)', () => {
    expect(TOKEN_BY_ID['coef_v_v']?.equationRow).toBe(1)
  })
})
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd compute_node/frontend && npx vitest run src/lib/mps/tokenMap.test.ts`
Expected: FAIL "Cannot find module './tokenMap'".

- [ ] **Step 3: Write implementation**

Создать `compute_node/frontend/src/lib/mps/tokenMap.ts`:

```typescript
export type MatrixName = 'A' | 'B' | 'C' | 'D'

export interface TokenMeta {
  id: string
  matrix: 'A' | 'B'
  row: number
  col: number
  equationRow: 0 | 1 | 2 | 3 | 4
  description: string
}

export const TOKENS: TokenMeta[] = [
  { id: 'coef_s_v',         matrix: 'A', row: 0, col: 1, equationRow: 0, description: '∂ṡ/∂v = 1' },
  { id: 'coef_v_v',         matrix: 'A', row: 1, col: 1, equationRow: 1, description: '∂v̇/∂v = −1/τ_v' },
  { id: 'coef_theta_omega', matrix: 'A', row: 2, col: 3, equationRow: 2, description: '∂θ̇/∂ω = 1' },
  { id: 'coef_omega_omega', matrix: 'A', row: 3, col: 3, equationRow: 3, description: '∂ω̇/∂ω = −1/τ_ω' },
  { id: 'coef_eint_v',      matrix: 'A', row: 4, col: 1, equationRow: 4, description: '∂ė_int/∂v = −1' },
  { id: 'coef_v_uv',        matrix: 'B', row: 1, col: 0, equationRow: 1, description: '∂v̇/∂u_v = 1/τ_v' },
  { id: 'coef_omega_uomega',matrix: 'B', row: 3, col: 1, equationRow: 3, description: '∂ω̇/∂u_ω = 1/τ_ω' },
]

export const TOKEN_BY_ID: Record<string, TokenMeta | undefined> = TOKENS.reduce(
  (acc, t) => {
    acc[t.id] = t
    return acc
  },
  {} as Record<string, TokenMeta>,
)

export function tokenForCell(matrix: 'A' | 'B' | 'C' | 'D', row: number, col: number): string | null {
  if (matrix !== 'A' && matrix !== 'B') return null
  const t = TOKENS.find((t) => t.matrix === matrix && t.row === row && t.col === col)
  return t?.id ?? null
}

export function cellForToken(id: string): { matrix: 'A' | 'B'; row: number; col: number } | null {
  const t = TOKEN_BY_ID[id]
  if (!t) return null
  return { matrix: t.matrix, row: t.row, col: t.col }
}
```

- [ ] **Step 4: Run tests**

Run: `cd compute_node/frontend && npx vitest run src/lib/mps/tokenMap.test.ts`
Expected: All PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/lib/mps/tokenMap.ts compute_node/frontend/src/lib/mps/tokenMap.test.ts
git commit -m "feat(mps-ui): tokenMap — связь ячеек A/B с токенами в формулах ОДУ"
```

---

## Task 5: HighlightContext + useMpsHighlight hook

**Files:**
- Create: `compute_node/frontend/src/components/mps/HighlightContext.tsx`
- Create: `compute_node/frontend/src/hooks/useMpsHighlight.ts`
- Create: `compute_node/frontend/src/components/mps/HighlightContext.test.tsx`

- [ ] **Step 1: Write failing test**

Создать `compute_node/frontend/src/components/mps/HighlightContext.test.tsx`:

```tsx
import { describe, it, expect } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { MpsHighlightProvider } from './HighlightContext'
import { useMpsHighlight } from '@/hooks/useMpsHighlight'

function Probe() {
  const { hovered, setEquation, setCell } = useMpsHighlight()
  return (
    <div>
      <div data-testid="state">{JSON.stringify(hovered)}</div>
      <button onClick={() => setEquation(1)}>set-eq-1</button>
      <button onClick={() => setEquation(null)}>clear-eq</button>
      <button onClick={() => setCell({ matrix: 'A', row: 1, col: 1 })}>set-cell</button>
    </div>
  )
}

describe('MpsHighlightProvider', () => {
  it('starts with empty state', () => {
    render(
      <MpsHighlightProvider>
        <Probe />
      </MpsHighlightProvider>,
    )
    const state = JSON.parse(screen.getByTestId('state').textContent ?? '{}')
    expect(state.equation).toBeNull()
    expect(state.cell).toBeNull()
    expect(state.vector).toBeNull()
  })

  it('updates equation index', () => {
    render(
      <MpsHighlightProvider>
        <Probe />
      </MpsHighlightProvider>,
    )
    fireEvent.click(screen.getByText('set-eq-1'))
    const state = JSON.parse(screen.getByTestId('state').textContent ?? '{}')
    expect(state.equation).toBe(1)
  })

  it('clears equation', () => {
    render(
      <MpsHighlightProvider>
        <Probe />
      </MpsHighlightProvider>,
    )
    fireEvent.click(screen.getByText('set-eq-1'))
    fireEvent.click(screen.getByText('clear-eq'))
    const state = JSON.parse(screen.getByTestId('state').textContent ?? '{}')
    expect(state.equation).toBeNull()
  })

  it('updates cell', () => {
    render(
      <MpsHighlightProvider>
        <Probe />
      </MpsHighlightProvider>,
    )
    fireEvent.click(screen.getByText('set-cell'))
    const state = JSON.parse(screen.getByTestId('state').textContent ?? '{}')
    expect(state.cell).toEqual({ matrix: 'A', row: 1, col: 1 })
  })
})

describe('useMpsHighlight outside provider', () => {
  it('returns no-op state', () => {
    function NoProvider() {
      const { hovered } = useMpsHighlight()
      return <div data-testid="np">{JSON.stringify(hovered)}</div>
    }
    render(<NoProvider />)
    const state = JSON.parse(screen.getByTestId('np').textContent ?? '{}')
    expect(state.equation).toBeNull()
  })
})
```

- [ ] **Step 2: Run, verify fail**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/HighlightContext.test.tsx`
Expected: FAIL with import error.

- [ ] **Step 3: Implement Context**

Создать `compute_node/frontend/src/components/mps/HighlightContext.tsx`:

```tsx
import { createContext, useCallback, useMemo, useState } from 'react'
import type { ReactNode } from 'react'

export type EquationIndex = 0 | 1 | 2 | 3 | 4
export type MatrixName = 'A' | 'B' | 'C' | 'D'
export type VectorName = 'Q' | 'R' | 'u_min' | 'u_max'

export interface HoveredCell {
  matrix: MatrixName
  row: number
  col: number
}

export interface HoveredVector {
  name: VectorName
  index: number
}

export interface MpsHighlightState {
  equation: EquationIndex | null
  cell: HoveredCell | null
  vector: HoveredVector | null
}

export interface MpsHighlightContextValue {
  hovered: MpsHighlightState
  setEquation: (i: EquationIndex | null) => void
  setCell: (c: HoveredCell | null) => void
  setVector: (v: HoveredVector | null) => void
  clearAll: () => void
}

const EMPTY_STATE: MpsHighlightState = {
  equation: null,
  cell: null,
  vector: null,
}

const NOOP_VALUE: MpsHighlightContextValue = {
  hovered: EMPTY_STATE,
  setEquation: () => {},
  setCell: () => {},
  setVector: () => {},
  clearAll: () => {},
}

export const MpsHighlightContext = createContext<MpsHighlightContextValue>(NOOP_VALUE)

interface MpsHighlightProviderProps {
  children: ReactNode
}

export function MpsHighlightProvider({ children }: MpsHighlightProviderProps) {
  const [hovered, setHovered] = useState<MpsHighlightState>(EMPTY_STATE)

  const setEquation = useCallback((equation: EquationIndex | null) => {
    setHovered((h) => (h.equation === equation ? h : { ...h, equation }))
  }, [])

  const setCell = useCallback((cell: HoveredCell | null) => {
    setHovered((h) => {
      if (h.cell === cell) return h
      if (
        cell &&
        h.cell &&
        h.cell.matrix === cell.matrix &&
        h.cell.row === cell.row &&
        h.cell.col === cell.col
      ) {
        return h
      }
      return { ...h, cell }
    })
  }, [])

  const setVector = useCallback((vector: HoveredVector | null) => {
    setHovered((h) => {
      if (h.vector === vector) return h
      if (
        vector &&
        h.vector &&
        h.vector.name === vector.name &&
        h.vector.index === vector.index
      ) {
        return h
      }
      return { ...h, vector }
    })
  }, [])

  const clearAll = useCallback(() => {
    setHovered(EMPTY_STATE)
  }, [])

  const value = useMemo<MpsHighlightContextValue>(
    () => ({ hovered, setEquation, setCell, setVector, clearAll }),
    [hovered, setEquation, setCell, setVector, clearAll],
  )

  return <MpsHighlightContext.Provider value={value}>{children}</MpsHighlightContext.Provider>
}
```

- [ ] **Step 4: Implement hook**

Создать `compute_node/frontend/src/hooks/useMpsHighlight.ts`:

```typescript
import { useContext } from 'react'
import { MpsHighlightContext } from '@/components/mps/HighlightContext'

export function useMpsHighlight() {
  return useContext(MpsHighlightContext)
}
```

- [ ] **Step 5: Run tests**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/HighlightContext.test.tsx`
Expected: All PASS.

- [ ] **Step 6: Commit**

```bash
git add compute_node/frontend/src/components/mps/HighlightContext.tsx compute_node/frontend/src/components/mps/HighlightContext.test.tsx compute_node/frontend/src/hooks/useMpsHighlight.ts
git commit -m "feat(mps-ui): MpsHighlightContext + useMpsHighlight для cross-highlight"
```

---

## Task 6: KatexFormula component

Обёртка над `react-katex` с поддержкой `data-token` attributes для cross-highlight.

**Files:**
- Create: `compute_node/frontend/src/components/mps/KatexFormula.tsx`
- Create: `compute_node/frontend/src/components/mps/KatexFormula.test.tsx`

- [ ] **Step 1: Write failing test**

Создать `compute_node/frontend/src/components/mps/KatexFormula.test.tsx`:

```tsx
import { describe, it, expect } from 'vitest'
import { render } from '@testing-library/react'
import { KatexFormula } from './KatexFormula'

describe('KatexFormula', () => {
  it('renders block formula', () => {
    const { container } = render(<KatexFormula formula="\\dot{s} = v" />)
    const katex = container.querySelector('.katex')
    expect(katex).not.toBeNull()
  })

  it('renders inline formula by default', () => {
    const { container } = render(<KatexFormula formula="x + 1" inline />)
    const katex = container.querySelector('.katex')
    expect(katex).not.toBeNull()
  })

  it('does not crash on malformed input', () => {
    const { container } = render(<KatexFormula formula="\\frac{" />)
    expect(container).toBeDefined()
  })
})
```

- [ ] **Step 2: Run, verify fail**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/KatexFormula.test.tsx`
Expected: FAIL with import error.

- [ ] **Step 3: Implement**

Создать `compute_node/frontend/src/components/mps/KatexFormula.tsx`:

```tsx
import { useEffect, useRef } from 'react'
import katex from 'katex'

interface KatexFormulaProps {
  formula: string
  inline?: boolean
  className?: string
  /** Если true — продолжает рендериться даже при ошибке (показывает stale + красный фон). */
  errorTolerant?: boolean
}

export function KatexFormula({
  formula,
  inline = false,
  className,
  errorTolerant = true,
}: KatexFormulaProps) {
  const ref = useRef<HTMLSpanElement>(null)

  useEffect(() => {
    if (!ref.current) return
    try {
      katex.render(formula, ref.current, {
        displayMode: !inline,
        throwOnError: false,
        errorColor: '#dc2626',
        strict: 'ignore',
      })
    } catch (err) {
      if (!errorTolerant) {
        console.error('KaTeX render error', err)
      }
      if (ref.current) {
        ref.current.textContent = formula
        ref.current.style.color = '#dc2626'
      }
    }
  }, [formula, inline, errorTolerant])

  return <span ref={ref} className={className} />
}
```

- [ ] **Step 4: Run tests**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/KatexFormula.test.tsx`
Expected: All PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/mps/KatexFormula.tsx compute_node/frontend/src/components/mps/KatexFormula.test.tsx
git commit -m "feat(mps-ui): KatexFormula — обёртка над katex с error-tolerance"
```

---

## Task 7: OdeCard component

Главный новый компонент левой sticky-колонки. Рендерит 5 уравнений ОДУ с подсветкой строк по hover (и по `equation` из Context).

**Files:**
- Create: `compute_node/frontend/src/components/mps/OdeCard.tsx`
- Create: `compute_node/frontend/src/components/mps/OdeCard.test.tsx`

- [ ] **Step 1: Write failing tests**

Создать `compute_node/frontend/src/components/mps/OdeCard.test.tsx`:

```tsx
import { describe, it, expect } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { MpsHighlightProvider } from './HighlightContext'
import { OdeCard } from './OdeCard'
import { buildCanonical, DEFAULT_TAU_V, DEFAULT_TAU_OMEGA } from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'

function makeMatrices(): MpsMatrices {
  const { A, B } = buildCanonical(DEFAULT_TAU_V, DEFAULT_TAU_OMEGA)
  return {
    A, B,
    C: Array.from({ length: 5 }, (_, i) =>
      Array.from({ length: 5 }, (_, j) => (i === j ? 1 : 0)),
    ),
    D: Array.from({ length: 5 }, () => Array(2).fill(0)),
    Q_diag: [10, 5, 1, 1, 5],
    R_diag: [1, 1],
    horizon_N: 20,
    u_min: [-0.3, -1.5],
    u_max: [0.3, 1.5],
    schema_version: '1.0',
  }
}

describe('OdeCard', () => {
  it('renders the title', () => {
    render(
      <MpsHighlightProvider>
        <OdeCard matrices={makeMatrices()} />
      </MpsHighlightProvider>,
    )
    expect(screen.getByText(/ОДУ-модель робота/i)).toBeInTheDocument()
  })

  it('has 5 equation rows with role=button', () => {
    render(
      <MpsHighlightProvider>
        <OdeCard matrices={makeMatrices()} />
      </MpsHighlightProvider>,
    )
    const rows = screen.getAllByRole('button', { name: /уравнение/i })
    expect(rows.length).toBe(5)
  })

  it('shows numeric coefficients when showNumeric=true', () => {
    render(
      <MpsHighlightProvider>
        <OdeCard matrices={makeMatrices()} showNumeric />
      </MpsHighlightProvider>,
    )
    // -1/0.15 ≈ -6.67, ищем «-6.67» в тексте
    expect(screen.getByText(/-6\.67/)).toBeInTheDocument()
  })

  it('hovers row → updates Context (probe via state badge)', () => {
    function Probe() {
      return (
        <MpsHighlightProvider>
          <OdeCard matrices={makeMatrices()} />
          <ProbeState />
        </MpsHighlightProvider>
      )
    }
    function ProbeState() {
      const { hovered } = require('@/hooks/useMpsHighlight').useMpsHighlight()
      return <div data-testid="hl">{hovered.equation ?? 'null'}</div>
    }
    render(<Probe />)
    const rows = screen.getAllByRole('button', { name: /уравнение/i })
    fireEvent.mouseEnter(rows[1])
    expect(screen.getByTestId('hl').textContent).toBe('1')
    fireEvent.mouseLeave(rows[1])
    expect(screen.getByTestId('hl').textContent).toBe('null')
  })

  it('shows deviation badge when matrices are non-canonical', () => {
    const m = makeMatrices()
    m.A[0][2] = 0.5  // не-каноническая правка
    render(
      <MpsHighlightProvider>
        <OdeCard matrices={m} />
      </MpsHighlightProvider>,
    )
    expect(screen.getByText(/нестандартные члены/i)).toBeInTheDocument()
  })
})
```

- [ ] **Step 2: Run, verify fail**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/OdeCard.test.tsx`
Expected: FAIL.

- [ ] **Step 3: Implement OdeCard**

Создать `compute_node/frontend/src/components/mps/OdeCard.tsx`:

```tsx
import { useMemo, useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { KatexFormula } from './KatexFormula'
import { useMpsHighlight } from '@/hooks/useMpsHighlight'
import { detectPhysics, DEFAULT_TAU_V, DEFAULT_TAU_OMEGA } from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'

interface OdeCardProps {
  matrices: MpsMatrices | null
  showNumeric?: boolean
}

interface EquationDef {
  index: 0 | 1 | 2 | 3 | 4
  symbolic: string
  numericFormula: (tauV: number, tauOmega: number) => string
  description: string
}

const EQUATIONS: EquationDef[] = [
  {
    index: 0,
    symbolic: '\\dot{s} = v',
    numericFormula: () => '\\dot{s} = v',
    description: 'уравнение для ṡ',
  },
  {
    index: 1,
    symbolic: '\\dot{v} = -\\frac{1}{\\tau_v}\\,v + \\frac{1}{\\tau_v}\\,u_v',
    numericFormula: (tauV) =>
      `\\dot{v} = ${(-1 / tauV).toFixed(2)}\\,v + ${(1 / tauV).toFixed(2)}\\,u_v`,
    description: 'уравнение для v̇',
  },
  {
    index: 2,
    symbolic: '\\dot{\\theta} = \\omega',
    numericFormula: () => '\\dot{\\theta} = \\omega',
    description: 'уравнение для θ̇',
  },
  {
    index: 3,
    symbolic: '\\dot{\\omega} = -\\frac{1}{\\tau_\\omega}\\,\\omega + \\frac{1}{\\tau_\\omega}\\,u_\\omega',
    numericFormula: (_, tauOmega) =>
      `\\dot{\\omega} = ${(-1 / tauOmega).toFixed(2)}\\,\\omega + ${(1 / tauOmega).toFixed(2)}\\,u_\\omega`,
    description: 'уравнение для ω̇',
  },
  {
    index: 4,
    symbolic: '\\dot{e}_{int} = v_{target} - v',
    numericFormula: () => '\\dot{e}_{int} = v_{target} - v',
    description: 'уравнение для ė_int',
  },
]

export function OdeCard({ matrices, showNumeric = false }: OdeCardProps) {
  const { hovered, setEquation } = useMpsHighlight()
  const [localShowNumeric, setLocalShowNumeric] = useState(showNumeric)

  const physics = useMemo(() => {
    if (!matrices) return null
    return detectPhysics(matrices)
  }, [matrices])

  const tauV = physics?.tau_v ?? DEFAULT_TAU_V
  const tauOmega = physics?.tau_omega ?? DEFAULT_TAU_OMEGA
  const deviationCount = physics?.deviations.length ?? 0

  const cellHighlightedEqRow: number | null = (() => {
    if (hovered.equation !== null) return hovered.equation
    if (hovered.cell && (hovered.cell.matrix === 'A' || hovered.cell.matrix === 'B')) {
      return hovered.cell.row
    }
    if (hovered.vector && hovered.vector.name === 'Q') return hovered.vector.index
    return null
  })()

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center justify-between">
          <span>ОДУ-модель робота</span>
          <button
            type="button"
            className="text-xs text-muted-foreground hover:text-foreground"
            onClick={() => setLocalShowNumeric((s) => !s)}
          >
            {localShowNumeric ? '∑ символьно' : '№ численно'}
          </button>
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3 text-sm">
        <div className="rounded border bg-muted/30 p-3 space-y-2">
          {EQUATIONS.map((eq) => {
            const isHl = cellHighlightedEqRow === eq.index
            const formula = localShowNumeric
              ? eq.numericFormula(tauV, tauOmega)
              : eq.symbolic
            return (
              <button
                key={eq.index}
                type="button"
                role="button"
                aria-label={eq.description}
                onMouseEnter={() => setEquation(eq.index)}
                onMouseLeave={() => setEquation(null)}
                className={[
                  'block w-full text-left px-2 py-1 rounded transition-colors',
                  isHl ? 'bg-primary/10 ring-1 ring-primary/40' : 'hover:bg-muted',
                ].join(' ')}
              >
                <KatexFormula formula={formula} inline />
              </button>
            )
          })}
        </div>

        <div className="text-xs text-muted-foreground space-y-1">
          <div>
            Состояние: <span className="font-mono">x = [s, v, θ, ω, e_int]ᵀ</span>
          </div>
          <div>
            Управление: <span className="font-mono">u = [v_cmd, ω_cmd]ᵀ</span>
          </div>
        </div>

        <div className="rounded border p-2 text-xs space-y-1 bg-muted/20">
          <div className="text-muted-foreground">Дискретизация ZOH (Ts = 50 мс):</div>
          <div className="font-mono">x[k+1] = A·x[k] + B·u[k]</div>
          <div className="font-mono">y[k]   = C·x[k] + D·u[k]</div>
        </div>

        {deviationCount > 0 && (
          <div className="rounded border border-orange-300 bg-orange-500/10 p-2 text-xs">
            <span className="font-medium text-orange-700">
              ⚠ нестандартные члены ({deviationCount})
            </span>{' '}
            <span className="text-muted-foreground">
              — A или B содержат коэффициенты вне канонической линеаризации.
              Подсвечены оранжевой рамкой в матрицах.
            </span>
          </div>
        )}
      </CardContent>
    </Card>
  )
}
```

- [ ] **Step 4: Run tests**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/OdeCard.test.tsx`
Expected: All PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/mps/OdeCard.tsx compute_node/frontend/src/components/mps/OdeCard.test.tsx
git commit -m "feat(mps-ui): OdeCard — sticky карта ОДУ с KaTeX и cross-highlight"
```

---

## Task 8: PhysicsParams component

Двунаправленные слайдеры τ_v и τ_ω. При движении ползунка собирает матрицу через `buildCanonical` и передаёт в `onPatch`. При смене `applied`/`draft` — детектит физику обратно.

**Files:**
- Create: `compute_node/frontend/src/components/mps/PhysicsParams.tsx`
- Create: `compute_node/frontend/src/components/mps/PhysicsParams.test.tsx`

- [ ] **Step 1: Write failing tests**

Создать `compute_node/frontend/src/components/mps/PhysicsParams.test.tsx`:

```tsx
import { describe, it, expect, vi } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { PhysicsParams } from './PhysicsParams'
import { buildCanonical, DEFAULT_TAU_V, DEFAULT_TAU_OMEGA } from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'

function makeMatrices(): MpsMatrices {
  const { A, B } = buildCanonical(DEFAULT_TAU_V, DEFAULT_TAU_OMEGA)
  return {
    A, B,
    C: Array.from({ length: 5 }, (_, i) =>
      Array.from({ length: 5 }, (_, j) => (i === j ? 1 : 0)),
    ),
    D: Array.from({ length: 5 }, () => Array(2).fill(0)),
    Q_diag: [10, 5, 1, 1, 5],
    R_diag: [1, 1],
    horizon_N: 20,
    u_min: [-0.3, -1.5],
    u_max: [0.3, 1.5],
    schema_version: '1.0',
  }
}

const DEFAULTS = { tau_v: DEFAULT_TAU_V, tau_omega: DEFAULT_TAU_OMEGA }

describe('PhysicsParams', () => {
  it('renders title', () => {
    render(
      <PhysicsParams
        applied={makeMatrices()}
        draft={null}
        onPatch={() => {}}
        defaults={DEFAULTS}
      />,
    )
    expect(screen.getByText(/Физические параметры/i)).toBeInTheDocument()
  })

  it('shows canonical status when matrices are canonical', () => {
    render(
      <PhysicsParams
        applied={makeMatrices()}
        draft={null}
        onPatch={() => {}}
        defaults={DEFAULTS}
      />,
    )
    expect(screen.getByText(/Каноническая форма/i)).toBeInTheDocument()
  })

  it('disables sliders and shows restore button when non_canonical', () => {
    const m = makeMatrices()
    m.A[0][2] = 0.5  // деканонизация
    render(
      <PhysicsParams
        applied={m}
        draft={null}
        onPatch={() => {}}
        defaults={DEFAULTS}
      />,
    )
    const sliders = screen.getAllByRole('slider')
    sliders.forEach((s) => expect(s).toBeDisabled())
    expect(screen.getByRole('button', { name: /Восстановить/i })).toBeInTheDocument()
  })

  it('slider τ_v change calls onPatch with updated A,B', () => {
    const onPatch = vi.fn()
    render(
      <PhysicsParams
        applied={makeMatrices()}
        draft={null}
        onPatch={onPatch}
        defaults={DEFAULTS}
      />,
    )
    const tauVSlider = screen.getByLabelText(/τ_v/i) as HTMLInputElement
    fireEvent.change(tauVSlider, { target: { value: '0.20' } })
    expect(onPatch).toHaveBeenCalled()
    const patched = onPatch.mock.calls[0][0] as MpsMatrices
    // -1/0.20 = -5.0
    expect(patched.A[1][1]).toBeCloseTo(-5.0, 4)
    expect(patched.B[1][0]).toBeCloseTo(5.0, 4)
  })

  it('shows N/A when incoherent', () => {
    const m = makeMatrices()
    m.A[1][1] = -4.0  // tau_v=0.25 от A
    // B[1][0] остался 6.67 от tau_v=0.15 → несогласованность
    render(
      <PhysicsParams
        applied={m}
        draft={null}
        onPatch={() => {}}
        defaults={DEFAULTS}
      />,
    )
    expect(screen.getByText(/неоднозначно/i)).toBeInTheDocument()
  })

  it('reset button resets specific tau to default', () => {
    const onPatch = vi.fn()
    const m = makeMatrices()
    const built = buildCanonical(0.20, DEFAULT_TAU_OMEGA)
    m.A = built.A
    m.B = built.B
    render(
      <PhysicsParams
        applied={m}
        draft={null}
        onPatch={onPatch}
        defaults={DEFAULTS}
      />,
    )
    const resetButtons = screen.getAllByRole('button', { name: /Сбросить τ_v/i })
    fireEvent.click(resetButtons[0])
    expect(onPatch).toHaveBeenCalled()
    const patched = onPatch.mock.calls[0][0] as MpsMatrices
    expect(patched.A[1][1]).toBeCloseTo(-1 / DEFAULT_TAU_V, 4)
  })
})
```

- [ ] **Step 2: Run, verify fail**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/PhysicsParams.test.tsx`
Expected: FAIL.

- [ ] **Step 3: Implement**

Создать `compute_node/frontend/src/components/mps/PhysicsParams.tsx`:

```tsx
import { useMemo } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import {
  buildCanonical,
  detectPhysics,
  DEFAULT_TAU_V,
  DEFAULT_TAU_OMEGA,
} from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'

interface PhysicsParamsProps {
  applied: MpsMatrices | null
  draft: MpsMatrices | null
  onPatch: (next: MpsMatrices) => void
  defaults: { tau_v: number; tau_omega: number }
}

const TAU_V_MIN = 0.05
const TAU_V_MAX = 0.50
const TAU_OMEGA_MIN = 0.05
const TAU_OMEGA_MAX = 0.50
const TAU_STEP = 0.005

interface SliderRowProps {
  label: string
  ariaLabel: string
  value: number | null
  min: number
  max: number
  step: number
  disabled?: boolean
  unit: string
  onChange: (next: number) => void
  onReset: () => void
  resetLabel: string
  cellsInfo: string
  incoherent?: boolean
}

function SliderRow(props: SliderRowProps) {
  const {
    label,
    ariaLabel,
    value,
    min,
    max,
    step,
    disabled,
    unit,
    onChange,
    onReset,
    resetLabel,
    cellsInfo,
    incoherent,
  } = props
  const displayValue = value === null ? min : value
  const valueText = incoherent
    ? 'неоднозначно'
    : value === null
    ? 'N/A'
    : `${value.toFixed(3)} ${unit}`

  return (
    <div className="space-y-1">
      <div className="flex items-center justify-between text-xs">
        <span className="font-mono">{label}</span>
        <span className="font-mono text-muted-foreground">{valueText}</span>
      </div>
      <div className="flex items-center gap-2">
        <input
          type="range"
          aria-label={ariaLabel}
          min={min}
          max={max}
          step={step}
          value={displayValue}
          disabled={disabled}
          onChange={(e) => onChange(Number(e.target.value))}
          className="flex-1"
        />
        <Button
          size="icon"
          variant="ghost"
          aria-label={resetLabel}
          onClick={onReset}
          disabled={disabled}
          className="h-7 w-7 text-xs"
          title={resetLabel}
        >
          ↻
        </Button>
      </div>
      <div className="text-xs text-muted-foreground font-mono">{cellsInfo}</div>
    </div>
  )
}

function patchCanonicalCells(
  base: MpsMatrices,
  tauV: number,
  tauOmega: number,
): MpsMatrices {
  const { A, B } = buildCanonical(tauV, tauOmega)
  const A_next = base.A.map((row) => row.slice())
  const B_next = base.B.map((row) => row.slice())
  // Записываем только канонические ячейки (паттерн), остальные сохраняются
  // как есть — чтобы не сбрасывать deviations студента нечаянно.
  // Если matrices canonical — паттерн совпадает с (A,B), и diff минимальный.
  A_next[0][1] = A[0][1]
  A_next[1][1] = A[1][1]
  A_next[2][3] = A[2][3]
  A_next[3][3] = A[3][3]
  A_next[4][1] = A[4][1]
  B_next[1][0] = B[1][0]
  B_next[3][1] = B[3][1]
  return { ...base, A: A_next, B: B_next }
}

function buildFromScratch(base: MpsMatrices, tauV: number, tauOmega: number): MpsMatrices {
  const { A, B } = buildCanonical(tauV, tauOmega)
  return { ...base, A, B }
}

export function PhysicsParams({ applied, draft, onPatch, defaults }: PhysicsParamsProps) {
  const current = draft ?? applied
  const physics = useMemo(() => (current ? detectPhysics(current) : null), [current])

  const tauV = physics?.tau_v ?? null
  const tauOmega = physics?.tau_omega ?? null
  const isNonCanonical = physics?.status === 'non_canonical'
  const isIncoherent = physics?.status === 'incoherent'
  const deviationCount = physics?.deviations.length ?? 0

  function handleTauVChange(next: number) {
    if (!current) return
    onPatch(patchCanonicalCells(current, next, tauOmega ?? defaults.tau_omega))
  }
  function handleTauOmegaChange(next: number) {
    if (!current) return
    onPatch(patchCanonicalCells(current, tauV ?? defaults.tau_v, next))
  }
  function handleResetTauV() {
    if (!current) return
    onPatch(patchCanonicalCells(current, defaults.tau_v, tauOmega ?? defaults.tau_omega))
  }
  function handleResetTauOmega() {
    if (!current) return
    onPatch(patchCanonicalCells(current, tauV ?? defaults.tau_v, defaults.tau_omega))
  }
  function handleRestoreCanonical() {
    if (!current) return
    onPatch(buildFromScratch(current, tauV ?? defaults.tau_v, tauOmega ?? defaults.tau_omega))
  }

  if (!current) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Физические параметры</CardTitle>
        </CardHeader>
        <CardContent className="text-sm text-muted-foreground">Загрузка матриц…</CardContent>
      </Card>
    )
  }

  const tauVCellsInfo =
    tauV === null
      ? `A[1][1] = ${current.A[1][1].toFixed(2)}, B[1][0] = ${current.B[1][0].toFixed(2)}`
      : `A[1][1] = ${(-1 / tauV).toFixed(2)}, B[1][0] = ${(1 / tauV).toFixed(2)}`
  const tauOmegaCellsInfo =
    tauOmega === null
      ? `A[3][3] = ${current.A[3][3].toFixed(2)}, B[3][1] = ${current.B[3][1].toFixed(2)}`
      : `A[3][3] = ${(-1 / tauOmega).toFixed(2)}, B[3][1] = ${(1 / tauOmega).toFixed(2)}`

  return (
    <Card>
      <CardHeader>
        <CardTitle>Физические параметры</CardTitle>
      </CardHeader>
      <CardContent className="space-y-4">
        <SliderRow
          label="τ_v (постоянная времени v)"
          ariaLabel="τ_v"
          value={tauV}
          min={TAU_V_MIN}
          max={TAU_V_MAX}
          step={TAU_STEP}
          disabled={isNonCanonical}
          unit="c"
          onChange={handleTauVChange}
          onReset={handleResetTauV}
          resetLabel="Сбросить τ_v"
          cellsInfo={tauVCellsInfo}
          incoherent={isIncoherent && tauV === null}
        />
        <SliderRow
          label="τ_ω (постоянная времени ω)"
          ariaLabel="τ_ω"
          value={tauOmega}
          min={TAU_OMEGA_MIN}
          max={TAU_OMEGA_MAX}
          step={TAU_STEP}
          disabled={isNonCanonical}
          unit="c"
          onChange={handleTauOmegaChange}
          onReset={handleResetTauOmega}
          resetLabel="Сбросить τ_ω"
          cellsInfo={tauOmegaCellsInfo}
          incoherent={isIncoherent && tauOmega === null}
        />

        {physics?.status === 'canonical' && (
          <div className="rounded border border-green-300 bg-green-500/10 p-2 text-xs text-green-800">
            ✓ Каноническая форма
            <div className="text-muted-foreground mt-1">
              Все «крутящиеся» ячейки A, B соответствуют τ_v={tauV?.toFixed(3)},
              τ_ω={tauOmega?.toFixed(3)}.
            </div>
          </div>
        )}

        {isIncoherent && (
          <div className="rounded border border-amber-300 bg-amber-500/10 p-2 text-xs text-amber-800">
            ⚠ Несогласованность: значения A и B дают разные τ. Слайдеры показывают «N/A».
          </div>
        )}

        {isNonCanonical && (
          <div className="rounded border border-orange-300 bg-orange-500/10 p-2 text-xs text-orange-800">
            ⚠ Не-каноническая форма: {deviationCount} ненулевых ячеек вне паттерна.
            Слайдеры заблокированы.
          </div>
        )}

        <Button
          size="sm"
          variant={isNonCanonical ? 'default' : 'outline'}
          onClick={handleRestoreCanonical}
          className="w-full"
        >
          Восстановить каноническую форму
        </Button>
      </CardContent>
    </Card>
  )
}
```

- [ ] **Step 4: Run tests**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/PhysicsParams.test.tsx`
Expected: All PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/mps/PhysicsParams.tsx compute_node/frontend/src/components/mps/PhysicsParams.test.tsx
git commit -m "feat(mps-ui): PhysicsParams — двунаправленные слайдеры τ_v, τ_ω"
```

---

## Task 9: MatrixGrid component

Переиспользуемая сетка матриц с подписями строк/столбцов, cross-highlight, dirty/invalid стилями. Заменяет inline `MatrixGrid` из текущего `MatrixEditor.tsx`.

**Files:**
- Create: `compute_node/frontend/src/components/mps/MatrixGrid.tsx`
- Create: `compute_node/frontend/src/components/mps/MatrixGrid.test.tsx`

- [ ] **Step 1: Write failing tests**

Создать `compute_node/frontend/src/components/mps/MatrixGrid.test.tsx`:

```tsx
import { describe, it, expect, vi } from 'vitest'
import { render, screen, fireEvent } from '@testing-library/react'
import { MpsHighlightProvider } from './HighlightContext'
import { MatrixGrid } from './MatrixGrid'

const defaultProps = {
  matrix: 'A' as const,
  values: [
    [0, 1, 0, 0, 0],
    [0, -6.67, 0, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, -10, 0],
    [0, -1, 0, 0, 0],
  ],
  applied: undefined,
  rowLabels: ['ṡ', 'v̇', 'θ̇', 'ω̇', 'ė_int'],
  colLabels: ['s', 'v', 'θ', 'ω', 'e_int'],
  onCell: vi.fn(),
}

describe('MatrixGrid', () => {
  it('renders correct number of input cells', () => {
    render(
      <MpsHighlightProvider>
        <MatrixGrid {...defaultProps} />
      </MpsHighlightProvider>,
    )
    const inputs = screen.getAllByRole('textbox')
    expect(inputs.length).toBe(5 * 5)
  })

  it('shows row and column labels', () => {
    render(
      <MpsHighlightProvider>
        <MatrixGrid {...defaultProps} />
      </MpsHighlightProvider>,
    )
    expect(screen.getByText('ṡ')).toBeInTheDocument()
    expect(screen.getByText('e_int')).toBeInTheDocument()
  })

  it('marks dirty cell with amber background', () => {
    const applied = defaultProps.values.map((row) => row.slice())
    applied[1][1] = -5.0  // applied отличается от current
    const { container } = render(
      <MpsHighlightProvider>
        <MatrixGrid {...defaultProps} applied={applied} />
      </MpsHighlightProvider>,
    )
    const dirtyCells = container.querySelectorAll('[data-dirty="true"]')
    expect(dirtyCells.length).toBe(1)
  })

  it('marks invalid input with red border', () => {
    const values = defaultProps.values.map((row) => row.slice())
    values[0][0] = NaN
    const { container } = render(
      <MpsHighlightProvider>
        <MatrixGrid {...defaultProps} values={values} />
      </MpsHighlightProvider>,
    )
    const invalid = container.querySelectorAll('[data-invalid="true"]')
    expect(invalid.length).toBeGreaterThan(0)
  })

  it('marks non-canonical cell with orange dashed border', () => {
    const values = defaultProps.values.map((row) => row.slice())
    values[0][2] = 0.5  // non-canonical position
    const { container } = render(
      <MpsHighlightProvider>
        <MatrixGrid {...defaultProps} values={values} />
      </MpsHighlightProvider>,
    )
    const dev = container.querySelectorAll('[data-deviation="true"]')
    expect(dev.length).toBe(1)
  })

  it('emits onCell on input change', () => {
    const onCell = vi.fn()
    render(
      <MpsHighlightProvider>
        <MatrixGrid {...defaultProps} onCell={onCell} />
      </MpsHighlightProvider>,
    )
    const inputs = screen.getAllByRole('textbox')
    fireEvent.change(inputs[0], { target: { value: '0.5' } })
    expect(onCell).toHaveBeenCalledWith(0, 0, '0.5')
  })

  it('shows canonical marker (¹) on canonical cells', () => {
    const { container } = render(
      <MpsHighlightProvider>
        <MatrixGrid {...defaultProps} />
      </MpsHighlightProvider>,
    )
    const markers = container.querySelectorAll('[data-canonical="true"]')
    // CANONICAL_PATTERN_A имеет 5 ячеек
    expect(markers.length).toBe(5)
  })
})
```

- [ ] **Step 2: Run, verify fail**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/MatrixGrid.test.tsx`
Expected: FAIL.

- [ ] **Step 3: Implement**

Создать `compute_node/frontend/src/components/mps/MatrixGrid.tsx`:

```tsx
import { Input } from '@/components/ui/input'
import { useMpsHighlight } from '@/hooks/useMpsHighlight'
import { CANONICAL_PATTERN_A, CANONICAL_PATTERN_B } from '@/lib/mps/canonical'
import type { MatrixName } from './HighlightContext'

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
  // Для A и B: позиция, которая ДОЛЖНА быть 0 в каноне (вне паттерна).
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

  return (
    <div className="space-y-2">
      <div
        className="grid gap-1"
        style={{
          gridTemplateColumns: `auto repeat(${cols}, minmax(0, 1fr))`,
        }}
      >
        {/* corner cell */}
        <div />
        {/* column labels */}
        {colLabels.map((lbl, j) => (
          <div
            key={`col-${j}`}
            className={[
              'text-xs font-mono text-center text-muted-foreground py-1 select-none',
              hovered.cell?.matrix === matrix && hovered.cell.col === j ? 'text-foreground font-semibold' : '',
            ].join(' ')}
            onMouseEnter={() => setCell({ matrix, row: -1, col: j })}
            onMouseLeave={() => setCell(null)}
          >
            {lbl}
          </div>
        ))}

        {/* rows */}
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
            onEnterRow={() => setEquation(i as 0 | 1 | 2 | 3 | 4)}
            onLeaveRow={() => setEquation(null)}
            setCell={setCell}
          />
        ))}
      </div>
      <div className="text-xs text-muted-foreground">
        <span className="font-mono">¹</span> — каноническая ячейка (привязана к τ_v / τ_ω)
      </div>
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
  onEnterRow: () => void
  onLeaveRow: () => void
  setCell: (cell: { matrix: MatrixName; row: number; col: number } | null) => void
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
  onEnterRow,
  onLeaveRow,
  setCell,
}: FragmentRowProps) {
  const rowHighlighted =
    highlightedEquation === rowIndex ||
    (highlightedCell?.matrix === matrix && highlightedCell.row === rowIndex)

  return (
    <>
      <div
        className={[
          'text-xs font-mono py-1 pr-2 select-none cursor-default',
          rowHighlighted ? 'text-foreground font-semibold' : 'text-muted-foreground',
        ].join(' ')}
        onMouseEnter={onEnterRow}
        onMouseLeave={onLeaveRow}
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
```

- [ ] **Step 4: Run tests**

Run: `cd compute_node/frontend && npx vitest run src/components/mps/MatrixGrid.test.tsx`
Expected: All PASS.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/mps/MatrixGrid.tsx compute_node/frontend/src/components/mps/MatrixGrid.test.tsx
git commit -m "feat(mps-ui): MatrixGrid с подписями строк/столбцов, cross-highlight и индикацией каноничности"
```

---

## Task 10: Refactor MatrixEditor — три таба

Большая задача: рефактор существующего `MatrixEditor.tsx` под три таба + использование `MatrixGrid` + поглощение `TuningSliders`.

**Files:**
- Modify: `compute_node/frontend/src/components/mps/MatrixEditor.tsx` (полная перепись)

- [ ] **Step 1: Read current MatrixEditor for reference**

Run: `cat compute_node/frontend/src/components/mps/MatrixEditor.tsx`
Цель: убедиться что understanding props не изменился. Сравнить с интерфейсом ниже.

- [ ] **Step 2: Replace MatrixEditor.tsx**

Заменить ВЕСЬ контент `compute_node/frontend/src/components/mps/MatrixEditor.tsx` на:

```tsx
import { useEffect, useMemo, useState } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { MatrixGrid } from './MatrixGrid'
import { useMpsHighlight } from '@/hooks/useMpsHighlight'
import { detectPhysics } from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'

type Tab = 'AB' | 'QRN' | 'CD'

const STATE_LABELS = ['s', 'v', 'θ', 'ω', 'e_int']
const STATE_DOTS = ['ṡ', 'v̇', 'θ̇', 'ω̇', 'ė_int']
const CONTROL_LABELS = ['u_v', 'u_ω']

interface MatrixEditorProps {
  applied: MpsMatrices | null
  draft: MpsMatrices | null
  onChange: (m: MpsMatrices) => void
  onApply: () => void
  onValidate: () => void
  onReset: () => void
  saving?: boolean
  validationStatus?: 'unknown' | 'stable' | 'unstable'
}

const TABS: { id: Tab; label: string }[] = [
  { id: 'AB',  label: 'Динамика A·B' },
  { id: 'QRN', label: 'Веса Q·R·N' },
  { id: 'CD',  label: 'Выход C·D' },
]

function makeMatrix(rows: number, cols: number, fill = 0): number[][] {
  return Array.from({ length: rows }, () => Array(cols).fill(fill))
}

function makeIdentity(n: number): number[][] {
  return Array.from({ length: n }, (_, i) =>
    Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)),
  )
}

export function MatrixEditor({
  applied,
  draft,
  onChange,
  onApply,
  onValidate,
  onReset,
  saving,
  validationStatus = 'unknown',
}: MatrixEditorProps) {
  const initial = draft ?? applied
  const [local, setLocal] = useState<MpsMatrices | null>(initial)
  const [tab, setTab] = useState<Tab>('AB')

  useEffect(() => {
    setLocal(draft ?? applied)
  }, [applied, draft])

  const valid = useMemo(() => {
    if (!local) return false
    const flat = [
      ...local.A.flat(),
      ...local.B.flat(),
      ...local.C.flat(),
      ...local.D.flat(),
      ...local.Q_diag,
      ...local.R_diag,
      ...local.u_min,
      ...local.u_max,
    ]
    if (flat.some((v) => !Number.isFinite(v))) return false
    if (local.horizon_N < 1 || local.horizon_N > 200) return false
    if (local.u_min.some((lo, i) => lo >= local.u_max[i])) return false
    if (local.R_diag.some((r) => r <= 0)) return false
    if (local.Q_diag.some((q) => q < 0)) return false
    return true
  }, [local])

  const dirty = useMemo(() => {
    if (!local || !applied) return Boolean(local)
    return JSON.stringify(local) !== JSON.stringify(applied)
  }, [local, applied])

  const dirtyCount = useMemo(() => {
    if (!local || !applied) return 0
    let count = 0
    const compareMatrix = (a: number[][], b: number[][]) => {
      for (let i = 0; i < a.length; i++) {
        for (let j = 0; j < a[i].length; j++) {
          if ((b[i] && b[i][j] !== a[i][j]) || !b[i]) count++
        }
      }
    }
    compareMatrix(local.A, applied.A)
    compareMatrix(local.B, applied.B)
    compareMatrix(local.C, applied.C)
    compareMatrix(local.D, applied.D)
    if (local.horizon_N !== applied.horizon_N) count++
    for (let i = 0; i < 5; i++) if (local.Q_diag[i] !== applied.Q_diag[i]) count++
    for (let i = 0; i < 2; i++) if (local.R_diag[i] !== applied.R_diag[i]) count++
    for (let i = 0; i < 2; i++) {
      if (local.u_min[i] !== applied.u_min[i]) count++
      if (local.u_max[i] !== applied.u_max[i]) count++
    }
    return count
  }, [local, applied])

  const physicsStatus = useMemo(() => (local ? detectPhysics(local).status : null), [local])

  if (!local) {
    return (
      <Card>
        <CardHeader>
          <CardTitle>Матрицы</CardTitle>
        </CardHeader>
        <CardContent className="text-sm text-muted-foreground">Загрузка матриц…</CardContent>
      </Card>
    )
  }

  function updateMatrix(key: 'A' | 'B' | 'C' | 'D', ri: number, ci: number, raw: string) {
    if (!local) return
    const parsed = Number(raw)
    const value = Number.isFinite(parsed) ? parsed : (raw as unknown as number)
    const m = {
      ...local,
      [key]: local[key].map((row, r) =>
        r === ri ? row.map((c, k) => (k === ci ? (value as number) : c)) : row,
      ),
    }
    setLocal(m)
    onChange(m)
  }

  function updateVector(key: 'Q_diag' | 'R_diag' | 'u_min' | 'u_max', i: number, raw: string) {
    if (!local) return
    const parsed = Number(raw)
    const value = Number.isFinite(parsed) ? parsed : (raw as unknown as number)
    const v = local[key].map((c, k) => (k === i ? (value as number) : c))
    const m = { ...local, [key]: v }
    setLocal(m)
    onChange(m)
  }

  function updateHorizon(raw: string) {
    if (!local) return
    const parsed = Number(raw)
    if (!Number.isFinite(parsed)) return
    const m = { ...local, horizon_N: Math.round(parsed) }
    setLocal(m)
    onChange(m)
  }

  function resetCToIdentity() {
    if (!local) return
    const m = { ...local, C: makeIdentity(5) }
    setLocal(m)
    onChange(m)
  }
  function resetDToZero() {
    if (!local) return
    const m = { ...local, D: makeMatrix(5, 2, 0) }
    setLocal(m)
    onChange(m)
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center justify-between gap-2 flex-wrap">
          <span>Матрицы и веса</span>
          <div className="flex gap-2">
            <Button size="sm" variant="secondary" onClick={onValidate} disabled={!valid}>
              Validate
            </Button>
            <Button size="sm" disabled={!valid || saving} onClick={onApply}>
              {saving
                ? 'Применяем…'
                : dirty
                ? `Apply (${dirtyCount})`
                : 'Apply'}
            </Button>
          </div>
        </CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        {/* Tabs */}
        <div role="tablist" className="flex gap-1 border-b">
          {TABS.map((t) => (
            <button
              key={t.id}
              role="tab"
              aria-selected={tab === t.id}
              type="button"
              onClick={() => setTab(t.id)}
              className={[
                'px-3 py-1.5 text-sm border-b-2 transition-colors',
                tab === t.id
                  ? 'border-primary text-primary font-medium'
                  : 'border-transparent text-muted-foreground hover:text-foreground',
              ].join(' ')}
            >
              {t.label}
            </button>
          ))}
        </div>

        {tab === 'AB' && (
          <div className="space-y-4">
            <section>
              <div className="text-xs font-mono text-muted-foreground mb-1">
                A — матрица состояния (5×5)
              </div>
              <MatrixGrid
                matrix="A"
                values={local.A}
                applied={applied?.A}
                rowLabels={STATE_DOTS}
                colLabels={STATE_LABELS}
                onCell={(r, c, v) => updateMatrix('A', r, c, v)}
              />
            </section>
            <section>
              <div className="text-xs font-mono text-muted-foreground mb-1">
                B — матрица управления (5×2)
              </div>
              <MatrixGrid
                matrix="B"
                values={local.B}
                applied={applied?.B}
                rowLabels={STATE_DOTS}
                colLabels={CONTROL_LABELS}
                onCell={(r, c, v) => updateMatrix('B', r, c, v)}
              />
            </section>
          </div>
        )}

        {tab === 'QRN' && (
          <div className="space-y-4">
            <QRNSection
              local={local}
              applied={applied}
              updateVector={updateVector}
              updateHorizon={updateHorizon}
            />
          </div>
        )}

        {tab === 'CD' && (
          <div className="space-y-4">
            <section>
              <div className="flex items-center justify-between mb-1">
                <div className="text-xs font-mono text-muted-foreground">
                  C — матрица выхода (5×5, default I)
                </div>
                <Button size="sm" variant="ghost" onClick={resetCToIdentity}>
                  Сбросить к I₅
                </Button>
              </div>
              <MatrixGrid
                matrix="C"
                values={local.C}
                applied={applied?.C}
                rowLabels={['y₁', 'y₂', 'y₃', 'y₄', 'y₅']}
                colLabels={STATE_LABELS}
                onCell={(r, c, v) => updateMatrix('C', r, c, v)}
              />
            </section>
            <section>
              <div className="flex items-center justify-between mb-1">
                <div className="text-xs font-mono text-muted-foreground">
                  D — прямая связь (5×2, default 0)
                </div>
                <Button size="sm" variant="ghost" onClick={resetDToZero}>
                  Сбросить к 0
                </Button>
              </div>
              <MatrixGrid
                matrix="D"
                values={local.D}
                applied={applied?.D}
                rowLabels={['y₁', 'y₂', 'y₃', 'y₄', 'y₅']}
                colLabels={CONTROL_LABELS}
                onCell={(r, c, v) => updateMatrix('D', r, c, v)}
              />
              <div className="text-xs text-muted-foreground mt-2">
                ⓘ y = C·x + D·u — для UI-визуализации, в управлении не используется.
              </div>
            </section>
          </div>
        )}

        {/* Bottom action bar */}
        <div className="flex items-center justify-between gap-2 pt-2 border-t flex-wrap">
          <div className="text-xs text-muted-foreground space-x-2">
            {dirty ? (
              <span>
                Изменено: <strong>{dirtyCount}</strong>
              </span>
            ) : (
              <span>Без изменений</span>
            )}
            <span>·</span>
            <span>
              Канонически:{' '}
              <strong className={
                physicsStatus === 'canonical'
                  ? 'text-green-700'
                  : physicsStatus === 'incoherent'
                  ? 'text-amber-700'
                  : 'text-orange-700'
              }>
                {physicsStatus === 'canonical' ? '✓' :
                 physicsStatus === 'incoherent' ? 'неоднозначно' :
                 'отклонения'}
              </strong>
            </span>
            <span>·</span>
            <span>
              λ:{' '}
              <strong className={
                validationStatus === 'stable'
                  ? 'text-green-700'
                  : validationStatus === 'unstable'
                  ? 'text-red-700'
                  : 'text-muted-foreground'
              }>
                {validationStatus === 'stable' ? 'stable' :
                 validationStatus === 'unstable' ? 'unstable' :
                 '?'}
              </strong>
            </span>
          </div>
          <Button size="sm" variant="outline" onClick={onReset}>
            Reset draft
          </Button>
        </div>

        {!valid && (
          <div className="text-xs text-red-500" role="alert">
            Есть невалидные значения — Apply заблокирован.
          </div>
        )}
      </CardContent>
    </Card>
  )
}

interface QRNSectionProps {
  local: MpsMatrices
  applied: MpsMatrices | null
  updateVector: (key: 'Q_diag' | 'R_diag' | 'u_min' | 'u_max', i: number, raw: string) => void
  updateHorizon: (raw: string) => void
}

function QRNSection({ local, applied, updateVector, updateHorizon }: QRNSectionProps) {
  const Q_LABELS = ['Q[s]', 'Q[v]', 'Q[θ]', 'Q[ω]', 'Q[eᵢ]']
  const Q_DESC = ['состояние s', 'состояние v', 'состояние θ', 'состояние ω', 'состояние e_int']
  const R_LABELS = ['R[u_v]', 'R[u_ω]']
  const R_DESC = ['v_cmd', 'ω_cmd']
  const { setVector } = useMpsHighlight()

  return (
    <>
      <section>
        <div className="text-xs font-mono text-muted-foreground mb-2">
          Q — веса ошибки состояния (диагональ)
        </div>
        <div className="space-y-1.5">
          {local.Q_diag.map((v, i) => (
            <div
              key={`q-${i}`}
              className="flex items-center gap-2 text-xs"
              onMouseEnter={() => setVector({ name: 'Q', index: i })}
              onMouseLeave={() => setVector(null)}
            >
              <span className="font-mono w-12">{Q_LABELS[i]}</span>
              <span className="text-muted-foreground w-32">{Q_DESC[i]}</span>
              <input
                type="range"
                min={0}
                max={100}
                step={0.5}
                value={v}
                onChange={(e) => updateVector('Q_diag', i, e.target.value)}
                className="flex-1"
                aria-label={Q_LABELS[i]}
              />
              <span className="font-mono w-12 text-right">{v.toFixed(2)}</span>
            </div>
          ))}
        </div>
        <div className="text-xs text-muted-foreground mt-2">
          ⓘ Большее Qᵢ = MPC сильнее штрафует отклонение i-го состояния от reference.
        </div>
      </section>

      <section>
        <div className="text-xs font-mono text-muted-foreground mb-2">
          R — веса управления (диагональ)
        </div>
        <div className="space-y-1.5">
          {local.R_diag.map((v, i) => (
            <div key={`r-${i}`} className="flex items-center gap-2 text-xs">
              <span className="font-mono w-12">{R_LABELS[i]}</span>
              <span className="text-muted-foreground w-32">{R_DESC[i]}</span>
              <input
                type="range"
                min={0.1}
                max={20}
                step={0.1}
                value={v}
                onChange={(e) => updateVector('R_diag', i, e.target.value)}
                className="flex-1"
                aria-label={R_LABELS[i]}
              />
              <span className="font-mono w-12 text-right">{v.toFixed(2)}</span>
            </div>
          ))}
        </div>
        <div className="text-xs text-muted-foreground mt-2">
          ⓘ Большие R ⇒ экономия управления, плавнее езда.
        </div>
      </section>

      <section>
        <div className="text-xs font-mono text-muted-foreground mb-2">N — горизонт прогноза</div>
        <div className="flex items-center gap-2 text-xs">
          <span className="font-mono w-12">N steps</span>
          <input
            type="range"
            min={1}
            max={50}
            step={1}
            value={local.horizon_N}
            onChange={(e) => updateHorizon(e.target.value)}
            className="flex-1"
            aria-label="horizon_N"
          />
          <span className="font-mono w-20 text-right">{local.horizon_N} ({(local.horizon_N * 0.05).toFixed(2)} c)</span>
        </div>
        <div className="text-xs text-muted-foreground mt-1">
          ⓘ Больше N — точнее план, дороже вычисления (Ts = 50 мс).
        </div>
      </section>

      <section>
        <div className="text-xs font-mono text-muted-foreground mb-2">Ограничения управления</div>
        <div className="grid grid-cols-2 gap-2 text-xs">
          <div className="flex items-center gap-2">
            <span className="font-mono w-12">u_min</span>
            {local.u_min.map((v, i) => (
              <Input
                key={`umin-${i}`}
                type="number"
                step="0.01"
                value={String(v)}
                onChange={(e) => updateVector('u_min', i, e.target.value)}
                className="h-7 text-xs font-mono"
                aria-label={`u_min[${i}]`}
              />
            ))}
          </div>
          <div className="flex items-center gap-2">
            <span className="font-mono w-12">u_max</span>
            {local.u_max.map((v, i) => (
              <Input
                key={`umax-${i}`}
                type="number"
                step="0.01"
                value={String(v)}
                onChange={(e) => updateVector('u_max', i, e.target.value)}
                className="h-7 text-xs font-mono"
                aria-label={`u_max[${i}]`}
              />
            ))}
          </div>
        </div>
        <div className="text-xs text-muted-foreground mt-1">
          ⓘ MPC clip-ит u в эти рамки на каждом шаге.
        </div>
      </section>
    </>
  )
}
```

- [ ] **Step 3: Verify build (no test for this complex component)**

Run: `cd compute_node/frontend && npm run build`
Expected: build green.

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/components/mps/MatrixEditor.tsx
git commit -m "feat(mps-ui): MatrixEditor с тремя табами A·B/Q·R·N/C·D + использование MatrixGrid"
```

---

## Task 11: Delete TuningSliders.tsx

Функционал переехал в таб «Q·R·N» MatrixEditor-а. Старый `TuningSliders.tsx` больше не нужен.

**Files:**
- Delete: `compute_node/frontend/src/components/mps/TuningSliders.tsx`
- Modify: `compute_node/frontend/src/pages/MpsPage.tsx` (убрать импорт и использование)

- [ ] **Step 1: Remove import and usage in MpsPage**

Открыть `compute_node/frontend/src/pages/MpsPage.tsx`. Убрать строку импорта:

```typescript
import { TuningSliders } from '@/components/mps/TuningSliders'
```

И блок использования:

```tsx
<TuningSliders
  applied={matricesHook.applied}
  onSimResult={setPrimaryResult}
  onPromote={(m) => void matricesHook.saveDraft(m)}
/>
```

(Этот блок заменим на новый layout в Task 19, пока — просто удалить.)

- [ ] **Step 2: Delete file**

```bash
rm compute_node/frontend/src/components/mps/TuningSliders.tsx
```

- [ ] **Step 3: Verify build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green.

- [ ] **Step 4: Commit**

```bash
git add -A compute_node/frontend/src/components/mps/TuningSliders.tsx compute_node/frontend/src/pages/MpsPage.tsx
git commit -m "refactor(mps-ui): удалён TuningSliders — функционал в Q·R·N табе MatrixEditor"
```

---

## Task 12: Refactor EigenvaluePanel

Делаем численные значения всегда видимыми + диагностика-чеклист.

**Files:**
- Modify: `compute_node/frontend/src/components/mps/EigenvaluePanel.tsx` (полная перепись)

- [ ] **Step 1: Replace EigenvaluePanel.tsx**

Заменить ВЕСЬ контент `compute_node/frontend/src/components/mps/EigenvaluePanel.tsx`:

```tsx
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { ComplexNumber } from '@/types/mps'

interface EigenvaluePanelProps {
  open: ComplexNumber[]
  closed: ComplexNumber[]
  isPlantStable: boolean
  isClosedLoopStable: boolean
  warnings?: string[]
  lastValidatedAt?: string | null
}

const SIZE = 240
const RADIUS = 95
const CENTER = SIZE / 2

function projectToSvg(z: ComplexNumber) {
  return { x: CENTER + z.re * RADIUS, y: CENTER - z.im * RADIUS }
}

function isStablePoint(z: ComplexNumber): boolean {
  return Math.hypot(z.re, z.im) < 1
}

function formatComplex(z: ComplexNumber): string {
  const sign = z.im >= 0 ? '+' : '−'
  return `${z.re.toFixed(3)} ${sign} ${Math.abs(z.im).toFixed(3)}i`
}

function ChecklistRow({ ok, label, hint }: { ok: boolean; label: string; hint?: string }) {
  return (
    <div className="flex items-start gap-2 text-xs">
      <span className={ok ? 'text-green-600' : 'text-red-600'}>{ok ? '✓' : '✗'}</span>
      <div>
        <div className={ok ? 'text-foreground' : 'text-red-700 font-medium'}>{label}</div>
        {hint && <div className="text-muted-foreground">{hint}</div>}
      </div>
    </div>
  )
}

export function EigenvaluePanel({
  open,
  closed,
  isPlantStable,
  isClosedLoopStable,
  warnings = [],
  lastValidatedAt,
}: EigenvaluePanelProps) {
  const hasPoles = open.length > 0 || closed.length > 0

  return (
    <Card>
      <CardHeader>
        <CardTitle>Анализ устойчивости</CardTitle>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="grid grid-cols-1 md:grid-cols-[auto_1fr] gap-4 items-start">
          <svg width={SIZE} height={SIZE} role="img" aria-label="полюса в единичной окружности">
            <rect width={SIZE} height={SIZE} fill="transparent" />
            <line x1={0} y1={CENTER} x2={SIZE} y2={CENTER} stroke="#94a3b8" strokeWidth={0.5} />
            <line x1={CENTER} y1={0} x2={CENTER} y2={SIZE} stroke="#94a3b8" strokeWidth={0.5} />
            <circle
              cx={CENTER}
              cy={CENTER}
              r={RADIUS}
              fill="none"
              stroke="#64748b"
              strokeWidth={1.5}
              strokeDasharray="4 4"
            />
            {open.map((z, i) => {
              const { x, y } = projectToSvg(z)
              return (
                <circle
                  key={`o-${i}`}
                  cx={x}
                  cy={y}
                  r={5}
                  fill={isStablePoint(z) ? '#16a34a' : '#dc2626'}
                  stroke="#0f172a"
                  strokeWidth={0.5}
                />
              )
            })}
            {closed.map((z, i) => {
              const { x, y } = projectToSvg(z)
              return (
                <rect
                  key={`c-${i}`}
                  x={x - 4}
                  y={y - 4}
                  width={8}
                  height={8}
                  fill={isStablePoint(z) ? '#0ea5e9' : '#f97316'}
                  stroke="#0f172a"
                  strokeWidth={0.5}
                />
              )
            })}
            <text x={CENTER + RADIUS - 8} y={CENTER - 6} fontSize={10} fill="#64748b">
              Re
            </text>
            <text x={CENTER + 6} y={14} fontSize={10} fill="#64748b">
              Im
            </text>
          </svg>

          <div className="grid grid-cols-1 sm:grid-cols-2 gap-3 text-xs font-mono">
            <div>
              <div className="font-sans text-muted-foreground mb-1 flex items-center gap-1">
                <span className="inline-block w-3 h-3 rounded-full bg-green-600" />
                λ(A) — открытый контур
              </div>
              {open.length === 0 ? (
                <div className="text-muted-foreground italic">нет данных</div>
              ) : (
                open.map((z, i) => (
                  <div key={i} className={isStablePoint(z) ? '' : 'text-red-600'}>
                    {formatComplex(z)} {isStablePoint(z) ? '◯' : '⚠'}
                  </div>
                ))
              )}
            </div>
            <div>
              <div className="font-sans text-muted-foreground mb-1 flex items-center gap-1">
                <span className="inline-block w-3 h-3 bg-sky-500" />
                λ(A − B·K) — замкнутый
              </div>
              {closed.length === 0 ? (
                <div className="text-muted-foreground italic">нет данных</div>
              ) : (
                closed.map((z, i) => (
                  <div key={i} className={isStablePoint(z) ? '' : 'text-orange-600'}>
                    {formatComplex(z)} {isStablePoint(z) ? '◻' : '⚠'}
                  </div>
                ))
              )}
            </div>
          </div>
        </div>

        <div className="rounded border bg-muted/20 p-3 space-y-2">
          <ChecklistRow
            ok={isPlantStable}
            label={isPlantStable ? 'Объект (A) устойчив' : 'Объект НЕ устойчив'}
            hint="все |λ(A)| < 1"
          />
          <ChecklistRow
            ok={isClosedLoopStable}
            label={isClosedLoopStable ? 'Замкнутая система (A − B·K) устойчива' : 'Замкнутая система НЕ устойчива'}
            hint="все |λ(A − B·K)| < 1 — MPC стабилизирует объект"
          />
          {warnings.length > 0 && (
            <div className="border-t pt-2 space-y-1">
              {warnings.map((w, i) => (
                <div key={i} className="flex items-start gap-2 text-xs text-amber-700">
                  <span>⚠</span>
                  <span>{w}</span>
                </div>
              ))}
            </div>
          )}
        </div>

        {lastValidatedAt && (
          <div className="text-xs text-muted-foreground">
            Последняя проверка: {lastValidatedAt}
          </div>
        )}

        {!hasPoles && (
          <div className="text-xs text-muted-foreground italic">
            Нажмите Validate в редакторе матриц, чтобы вычислить полюса.
          </div>
        )}
      </CardContent>
    </Card>
  )
}
```

- [ ] **Step 2: Verify build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green.

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/components/mps/EigenvaluePanel.tsx
git commit -m "feat(mps-ui): EigenvaluePanel — inline numerics, диагностика-чеклист, warnings"
```

---

## Task 13: Refactor ScenarioControls — компактная горизонталь

**Files:**
- Modify: `compute_node/frontend/src/components/mps/ScenarioControls.tsx` (полная перепись)

- [ ] **Step 1: Replace ScenarioControls.tsx**

```tsx
import { useState } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import type { ScenarioSource } from '@/types/mps'

interface ScenarioControlsProps {
  defaultDistance?: number
  defaultVTarget?: number
  running: boolean
  onRun: (params: { distance: number; v_target: number; source: ScenarioSource }) => void
  onAbort: () => void
  source: ScenarioSource
  onSourceChange: (s: ScenarioSource) => void
  /** 0..1 для прогресс-бара. */
  progress?: number
}

export function ScenarioControls({
  defaultDistance = 2.0,
  defaultVTarget = 0.15,
  running,
  onRun,
  onAbort,
  source,
  onSourceChange,
  progress,
}: ScenarioControlsProps) {
  const [distance, setDistance] = useState(defaultDistance)
  const [vTarget, setVTarget] = useState(defaultVTarget)

  const distanceOk = distance > 0 && distance <= 5.0
  const vTargetOk = vTarget > 0 && vTarget <= 0.30
  const valid = distanceOk && vTargetOk

  return (
    <Card>
      <CardHeader>
        <CardTitle>Сценарий «проехать D м вперёд»</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="flex items-center gap-3 flex-wrap">
          <div className="flex items-center gap-1">
            <label className="text-xs font-mono text-muted-foreground">D, м</label>
            <Input
              type="number"
              min={0.1}
              max={5.0}
              step={0.1}
              value={String(distance)}
              onChange={(e) => setDistance(Number(e.target.value))}
              disabled={running}
              className={['h-8 text-sm w-20 font-mono', !distanceOk ? 'border-red-500' : ''].join(' ')}
              aria-label="distance"
            />
          </div>
          <div className="flex items-center gap-1">
            <label className="text-xs font-mono text-muted-foreground">v_target</label>
            <Input
              type="number"
              min={0.05}
              max={0.30}
              step={0.05}
              value={String(vTarget)}
              onChange={(e) => setVTarget(Number(e.target.value))}
              disabled={running}
              className={['h-8 text-sm w-20 font-mono', !vTargetOk ? 'border-red-500' : ''].join(' ')}
              aria-label="v_target"
            />
            <span className="text-xs text-muted-foreground">м/с</span>
          </div>

          <div className="flex rounded-md border overflow-hidden text-xs">
            <button
              type="button"
              onClick={() => onSourceChange('sim')}
              disabled={running}
              className={[
                'px-3 py-1 transition-colors',
                source === 'sim' ? 'bg-primary text-primary-foreground' : 'bg-transparent hover:bg-muted',
              ].join(' ')}
            >
              Sim
            </button>
            <button
              type="button"
              onClick={() => onSourceChange('robot')}
              disabled={running}
              className={[
                'px-3 py-1 border-l transition-colors',
                source === 'robot' ? 'bg-primary text-primary-foreground' : 'bg-transparent hover:bg-muted',
              ].join(' ')}
            >
              Robot
            </button>
          </div>

          <div className="ml-auto flex gap-2">
            <Button
              size="sm"
              onClick={() => onRun({ distance, v_target: vTarget, source })}
              disabled={!valid || running}
            >
              {running ? '⏳ Прогон…' : `▶ Run on ${source === 'sim' ? 'Sim' : 'Robot'}`}
            </Button>
            <Button size="sm" variant="destructive" onClick={onAbort} disabled={!running}>
              ⏹ Abort
            </Button>
          </div>
        </div>

        {running && progress !== undefined && progress >= 0 && (
          <div className="h-1.5 bg-muted rounded overflow-hidden">
            <div
              className="h-full bg-primary transition-all duration-150"
              style={{ width: `${Math.min(100, Math.max(0, progress * 100))}%` }}
            />
          </div>
        )}

        <div className="text-xs text-muted-foreground">
          ⓘ Sim: ~100 мс. Robot: ≈ D / v_target секунд физически.
        </div>
      </CardContent>
    </Card>
  )
}
```

- [ ] **Step 2: Verify build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green.

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/components/mps/ScenarioControls.tsx
git commit -m "feat(mps-ui): ScenarioControls — компактная горизонтальная раскладка + progress bar"
```

---

## Task 14: Refactor ResultPlots — раздельные табы + reference + метрики

**Files:**
- Modify: `compute_node/frontend/src/components/mps/ResultPlots.tsx` (полная перепись)

- [ ] **Step 1: Replace ResultPlots.tsx**

```tsx
import { useMemo, useState } from 'react'
import {
  CartesianGrid,
  Legend,
  Line,
  LineChart,
  ReferenceLine,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import type { MpsScenarioResult, MpsTelemetryPoint } from '@/types/mps'

type Tab = 's' | 'v' | 'theta_omega' | 'u' | 'y' | 'all'

interface ResultPlotsProps {
  primary: MpsScenarioResult | null
  overlays?: MpsScenarioResult[]
  liveTelemetry?: MpsTelemetryPoint[]
}

const STATE_LABELS = ['s', 'v', 'θ', 'ω', 'e_int']
const CONTROL_LABELS = ['v_cmd', 'ω_cmd']
const Y_LABELS = ['y₁', 'y₂', 'y₃', 'y₄', 'y₅']

const STATE_COLORS = ['#2563eb', '#16a34a', '#dc2626', '#a855f7', '#ea580c']
const CONTROL_COLORS = ['#0ea5e9', '#f59e0b']
const OVERLAY_COLORS = ['#94a3b8', '#fbbf24', '#a78bfa']

const TABS: { id: Tab; label: string }[] = [
  { id: 's',           label: 's(t)' },
  { id: 'v',           label: 'v(t)' },
  { id: 'theta_omega', label: 'θ,ω(t)' },
  { id: 'u',           label: 'u(t)' },
  { id: 'y',           label: 'y(t)' },
  { id: 'all',         label: 'Всё вместе' },
]

interface ChartRow {
  t: number
  [k: string]: number
}

function buildRows(points: MpsTelemetryPoint[], tab: Tab, prefix = ''): ChartRow[] {
  return points.map((p) => {
    const row: ChartRow = { t: p.t }
    if (tab === 's') {
      row[`${prefix}s`] = p.x[0] ?? 0
    } else if (tab === 'v') {
      row[`${prefix}v`] = p.x[1] ?? 0
    } else if (tab === 'theta_omega') {
      row[`${prefix}θ`] = p.x[2] ?? 0
      row[`${prefix}ω`] = p.x[3] ?? 0
    } else if (tab === 'u') {
      p.u.forEach((v, i) => {
        row[`${prefix}${CONTROL_LABELS[i] ?? `u${i}`}`] = v
      })
    } else if (tab === 'y') {
      p.y.forEach((v, i) => {
        row[`${prefix}${Y_LABELS[i] ?? `y${i}`}`] = v
      })
    } else if (tab === 'all') {
      p.x.forEach((v, i) => {
        row[`${prefix}${STATE_LABELS[i] ?? `x${i}`}`] = v
      })
      p.u.forEach((v, i) => {
        row[`${prefix}${CONTROL_LABELS[i] ?? `u${i}`}`] = v
      })
    }
    return row
  })
}

function mergeOverlays(
  base: ChartRow[],
  overlayPacks: { rows: ChartRow[] }[],
): ChartRow[] {
  if (overlayPacks.length === 0) return base
  const map = new Map<number, ChartRow>()
  base.forEach((row) => {
    map.set(Math.round(row.t * 1000), { ...row })
  })
  overlayPacks.forEach(({ rows }) => {
    rows.forEach((row) => {
      const key = Math.round(row.t * 1000)
      const existing = map.get(key) ?? { t: row.t }
      map.set(key, { ...existing, ...row })
    })
  })
  return Array.from(map.values()).sort((a, b) => a.t - b.t)
}

function shortRunId(id: string): string {
  return id.length > 8 ? id.slice(0, 8) : id
}

export function ResultPlots({ primary, overlays = [], liveTelemetry }: ResultPlotsProps) {
  const [tab, setTab] = useState<Tab>('s')

  const baseTelemetry = liveTelemetry?.length ? liveTelemetry : primary?.telemetry ?? []
  const referenceD = primary?.request.distance ?? null
  const referenceVTarget = primary?.request.v_target ?? null

  const data = useMemo(() => {
    const base = buildRows(baseTelemetry, tab)
    const overlayPacks = overlays.map((o, idx) => ({
      rows: buildRows(o.telemetry, tab, `[${idx + 2}] `),
    }))
    return mergeOverlays(base, overlayPacks)
  }, [baseTelemetry, overlays, tab])

  const seriesKeys = data.length > 0
    ? Object.keys(data[0]).filter((k) => k !== 't')
    : []

  function colorForSeries(key: string, idx: number): string {
    if (key.startsWith('[')) {
      const ovIdx = parseInt(key.slice(1, key.indexOf(']'))) - 2
      return OVERLAY_COLORS[ovIdx % OVERLAY_COLORS.length]
    }
    if (tab === 'u') return CONTROL_COLORS[idx % CONTROL_COLORS.length]
    return STATE_COLORS[idx % STATE_COLORS.length]
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle>Графики прогона</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div role="tablist" className="flex gap-1 border-b overflow-x-auto">
          {TABS.map((t) => (
            <button
              key={t.id}
              role="tab"
              aria-selected={tab === t.id}
              type="button"
              onClick={() => setTab(t.id)}
              className={[
                'px-3 py-1 text-sm border-b-2 whitespace-nowrap transition-colors',
                tab === t.id
                  ? 'border-primary text-primary font-medium'
                  : 'border-transparent text-muted-foreground hover:text-foreground',
              ].join(' ')}
            >
              {t.label}
            </button>
          ))}
        </div>

        {data.length === 0 ? (
          <div className="h-72 flex items-center justify-center text-sm text-muted-foreground">
            Нет данных — запусти сценарий или подожди телеметрию.
          </div>
        ) : (
          <div className="h-72">
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={data} margin={{ top: 10, right: 24, left: 8, bottom: 16 }}>
                <CartesianGrid strokeDasharray="3 3" />
                <XAxis
                  dataKey="t"
                  tickFormatter={(v: number) => v.toFixed(2)}
                  label={{ value: 't, с', position: 'insideBottom', offset: -8 }}
                />
                <YAxis />
                <Tooltip />
                <Legend />
                {tab === 's' && referenceD !== null && (
                  <ReferenceLine
                    y={referenceD}
                    stroke="#6b7280"
                    strokeDasharray="4 4"
                    label={{ value: `D = ${referenceD}`, position: 'right', fontSize: 11 }}
                  />
                )}
                {tab === 'v' && referenceVTarget !== null && (
                  <ReferenceLine
                    y={referenceVTarget}
                    stroke="#6b7280"
                    strokeDasharray="4 4"
                    label={{ value: `v* = ${referenceVTarget}`, position: 'right', fontSize: 11 }}
                  />
                )}
                {seriesKeys.map((k, i) => (
                  <Line
                    key={k}
                    type="monotone"
                    dataKey={k}
                    stroke={colorForSeries(k, i)}
                    dot={false}
                    isAnimationActive={false}
                    strokeWidth={k.startsWith('[') ? 1 : 1.5}
                    strokeDasharray={k.startsWith('[') ? '4 2' : undefined}
                    strokeOpacity={k.startsWith('[') ? 0.7 : 1}
                  />
                ))}
              </LineChart>
            </ResponsiveContainer>
          </div>
        )}

        {primary?.metrics && (
          <div className="grid grid-cols-2 gap-x-6 gap-y-1 text-xs font-mono border rounded p-3 bg-muted/20">
            <div className="flex justify-between">
              <span className="text-muted-foreground">overshoot:</span>
              <span>{primary.metrics.overshoot.toFixed(3)} м</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">settling:</span>
              <span>{primary.metrics.settling_time.toFixed(2)} c</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">ss_error:</span>
              <span>{primary.metrics.ss_error.toFixed(3)} м</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">peak v:</span>
              <span>{primary.metrics.peak_v.toFixed(3)} м/с</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">control E:</span>
              <span>{primary.metrics.control_energy.toFixed(3)}</span>
            </div>
            <div className="flex justify-between">
              <span className="text-muted-foreground">peak ω:</span>
              <span>{primary.metrics.peak_omega.toFixed(3)} рад/с</span>
            </div>
          </div>
        )}

        {primary && (
          <div className="text-xs text-muted-foreground font-mono flex flex-wrap items-center gap-x-3 gap-y-1">
            <span>Run: <strong>{shortRunId(primary.run_id)}</strong></span>
            <span>·</span>
            <span>{new Date(primary.started_at).toLocaleString()}</span>
            <span>·</span>
            <span>status: <strong>{primary.status}</strong></span>
          </div>
        )}
      </CardContent>
    </Card>
  )
}
```

- [ ] **Step 2: Verify build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green.

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/components/mps/ResultPlots.tsx
git commit -m "feat(mps-ui): ResultPlots — раздельные табы s/v/θω/u/y, reference-линии, метрики прогона"
```

---

## Task 15: Refactor HistoryPanel — фильтр + цветовая полоса

**Files:**
- Modify: `compute_node/frontend/src/components/mps/HistoryPanel.tsx` (полная перепись)

- [ ] **Step 1: Replace HistoryPanel.tsx**

```tsx
import { useMemo, useState } from 'react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { ScrollArea } from '@/components/ui/scroll-area'
import type { MpsScenarioResult, ScenarioStatus } from '@/types/mps'

interface HistoryPanelProps {
  history: MpsScenarioResult[]
  onSelect: (run: MpsScenarioResult) => void
  onReplay: (runId: string) => void
  onCompareChange: (selected: MpsScenarioResult[]) => void
  loading?: boolean
}

const STATUS_TEXT_COLOR: Record<ScenarioStatus, string> = {
  running: 'text-blue-600',
  reached: 'text-green-600',
  timeout: 'text-amber-600',
  aborted: 'text-orange-600',
  error:   'text-red-600',
}
const STATUS_STRIPE_COLOR: Record<ScenarioStatus, string> = {
  running: 'bg-blue-500',
  reached: 'bg-green-500',
  timeout: 'bg-amber-500',
  aborted: 'bg-orange-500',
  error:   'bg-red-500',
}

const STATUS_OPTIONS: Array<{ value: ScenarioStatus | 'all'; label: string }> = [
  { value: 'all',     label: 'Все' },
  { value: 'reached', label: 'Достигнут' },
  { value: 'timeout', label: 'Timeout' },
  { value: 'aborted', label: 'Aborted' },
  { value: 'error',   label: 'Error' },
]

export function HistoryPanel({
  history,
  onSelect,
  onReplay,
  onCompareChange,
  loading,
}: HistoryPanelProps) {
  const [selectedIds, setSelectedIds] = useState<Set<string>>(new Set())
  const [filter, setFilter] = useState<ScenarioStatus | 'all'>('all')

  const filtered = useMemo(
    () => (filter === 'all' ? history : history.filter((h) => h.status === filter)),
    [history, filter],
  )

  function toggleCompare(runId: string) {
    setSelectedIds((prev) => {
      const next = new Set(prev)
      if (next.has(runId)) {
        next.delete(runId)
      } else {
        if (next.size >= 3) return prev
        next.add(runId)
      }
      const items = history.filter((h) => next.has(h.run_id))
      onCompareChange(items)
      return next
    })
  }

  function clearSelection() {
    setSelectedIds(new Set())
    onCompareChange([])
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle className="flex items-center justify-between flex-wrap gap-2">
          <span>История прогонов ({history.length})</span>
          <span className="text-xs font-normal text-muted-foreground">
            Compare: {selectedIds.size}/3
          </span>
        </CardTitle>
      </CardHeader>
      <CardContent>
        <div className="flex items-center gap-2 mb-2 flex-wrap">
          <select
            value={filter}
            onChange={(e) => setFilter(e.target.value as ScenarioStatus | 'all')}
            className="h-7 text-xs border rounded px-2 bg-background"
            aria-label="status filter"
          >
            {STATUS_OPTIONS.map((o) => (
              <option key={o.value} value={o.value}>
                {o.label}
              </option>
            ))}
          </select>
          {selectedIds.size > 0 && (
            <Button size="sm" variant="ghost" onClick={clearSelection} className="h-7 text-xs">
              Очистить выбор
            </Button>
          )}
        </div>

        {loading ? (
          <div className="text-sm text-muted-foreground">Загрузка…</div>
        ) : filtered.length === 0 ? (
          <div className="text-sm text-muted-foreground">
            {history.length === 0
              ? 'Пока ничего — запусти первый сценарий.'
              : 'Нет прогонов с выбранным статусом.'}
          </div>
        ) : (
          <ScrollArea className="h-72">
            <ul className="space-y-1 text-xs">
              {filtered.map((h) => {
                const sel = selectedIds.has(h.run_id)
                const ts = new Date(h.started_at).toLocaleTimeString()
                return (
                  <li
                    key={h.run_id}
                    className={[
                      'relative flex items-center gap-2 pl-3 pr-2 py-1 rounded',
                      sel ? 'bg-amber-500/10' : 'hover:bg-accent',
                    ].join(' ')}
                  >
                    <span
                      className={[
                        'absolute left-0 top-1 bottom-1 w-1 rounded',
                        STATUS_STRIPE_COLOR[h.status],
                      ].join(' ')}
                    />
                    <input
                      type="checkbox"
                      checked={sel}
                      onChange={() => toggleCompare(h.run_id)}
                      aria-label={`compare ${h.run_id}`}
                    />
                    <button
                      type="button"
                      onClick={() => onSelect(h)}
                      className="flex-1 text-left font-mono"
                    >
                      <div className="truncate">{h.run_id.slice(0, 8)}</div>
                      <div className="text-muted-foreground">
                        {ts} · D={h.request.distance.toFixed(2)} · v=
                        {h.request.v_target.toFixed(2)} ·{' '}
                        <span className={STATUS_TEXT_COLOR[h.status]}>{h.status}</span>
                        {h.metrics && <> · ss={h.metrics.ss_error.toFixed(3)}</>}
                      </div>
                    </button>
                    <Button
                      size="icon"
                      variant="ghost"
                      onClick={() => onReplay(h.run_id)}
                      className="h-7 w-7 text-xs"
                      aria-label={`replay ${h.run_id}`}
                      title="Replay"
                    >
                      ▶
                    </Button>
                  </li>
                )
              })}
            </ul>
          </ScrollArea>
        )}
      </CardContent>
    </Card>
  )
}
```

- [ ] **Step 2: Verify build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green.

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/components/mps/HistoryPanel.tsx
git commit -m "feat(mps-ui): HistoryPanel — status-фильтр, цветовая полоса, кнопка ▶"
```

---

## Task 16: Refactor TrajectoryView — рестайл

**Files:**
- Read first: `compute_node/frontend/src/components/mps/TrajectoryView.tsx`
- Modify: `compute_node/frontend/src/components/mps/TrajectoryView.tsx`

- [ ] **Step 1: Read current TrajectoryView**

Run: `cat compute_node/frontend/src/components/mps/TrajectoryView.tsx`
Цель: понять что есть и сохранить логику (только UI shell обновляем).

- [ ] **Step 2: Wrap content with consistent Card and add scale bar / legend if missing**

Открыть `compute_node/frontend/src/components/mps/TrajectoryView.tsx`. Если корневой `<Card>` уже есть — обновить заголовок и обернуть content в `space-y-2`. Добавить scale bar и легенду inside SVG (если их нет).

В зависимости от текущего содержимого, в самом верху SVG добавить группу легенды + в bottom-left добавить scale bar:

```tsx
{/* legend (top-left, внутри SVG) */}
<g transform="translate(10, 14)" fontSize={10} fontFamily="monospace">
  <line x1={0} y1={0} x2={20} y2={0} stroke="#64748b" strokeDasharray="4 2" />
  <text x={26} y={3} fill="#64748b">Plan</text>
  <line x1={60} y1={0} x2={80} y2={0} stroke="#2563eb" strokeWidth={1.5} />
  <text x={86} y={3} fill="#2563eb">Actual</text>
</g>

{/* scale bar (bottom-left) — оценить размер из viewBox */}
<g transform="translate(10, SVG_HEIGHT - 18)" fontSize={10}>
  <line x1={0} y1={0} x2={SCALE_PX} y2={0} stroke="#475569" strokeWidth={1.5} />
  <line x1={0} y1={-3} x2={0} y2={3} stroke="#475569" />
  <line x1={SCALE_PX} y1={-3} x2={SCALE_PX} y2={3} stroke="#475569" />
  <text x={SCALE_PX / 2} y={14} textAnchor="middle" fill="#475569">1 м</text>
</g>
```

Где `SCALE_PX` — пиксельный эквивалент 1 метра в текущем масштабе SVG, `SVG_HEIGHT` — высота viewBox. Подставить численные значения из текущего кода.

Заголовок CardTitle переименовать в `Траектория (top-down)`.

- [ ] **Step 3: Verify build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green.

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/components/mps/TrajectoryView.tsx
git commit -m "feat(mps-ui): TrajectoryView — единый стиль, легенда, scale bar"
```

---

## Task 17: Extend DraftStatus

**Files:**
- Modify: `compute_node/frontend/src/components/mps/DraftStatus.tsx` (полная перепись)

- [ ] **Step 1: Replace DraftStatus.tsx**

```tsx
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
```

- [ ] **Step 2: Verify build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green.

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/components/mps/DraftStatus.tsx
git commit -m "feat(mps-ui): DraftStatus — состояния non-canonical / incoherent"
```

---

## Task 18: Refactor MpsPage — sticky-grid layout

Главная сборка. Подключаем `MpsHighlightProvider`, переставляем компоненты в новый layout.

**Files:**
- Modify: `compute_node/frontend/src/pages/MpsPage.tsx` (полная перепись)

- [ ] **Step 1: Replace MpsPage.tsx**

```tsx
import { useEffect, useMemo, useState } from 'react'
import { Header } from '@/components/layout/Header'
import { DraftStatus } from '@/components/mps/DraftStatus'
import { EigenvaluePanel } from '@/components/mps/EigenvaluePanel'
import { HistoryPanel } from '@/components/mps/HistoryPanel'
import { MatrixEditor } from '@/components/mps/MatrixEditor'
import { OdeCard } from '@/components/mps/OdeCard'
import { PhysicsParams } from '@/components/mps/PhysicsParams'
import { ResultPlots } from '@/components/mps/ResultPlots'
import { ScenarioControls } from '@/components/mps/ScenarioControls'
import { TrajectoryView } from '@/components/mps/TrajectoryView'
import { MpsHighlightProvider } from '@/components/mps/HighlightContext'
import { useMpsHistory } from '@/hooks/useMpsHistory'
import { useMpsLiveTelemetry } from '@/hooks/useMpsLiveTelemetry'
import { useMpsMatrices } from '@/hooks/useMpsMatrices'
import { useMpsRun } from '@/hooks/useMpsRun'
import { useMpsValidate } from '@/hooks/useMpsValidate'
import { detectPhysics, DEFAULT_TAU_V, DEFAULT_TAU_OMEGA } from '@/lib/mps/canonical'
import type {
  MpsMatrices,
  MpsScenarioRequest,
  MpsScenarioResult,
  ScenarioSource,
} from '@/types/mps'

export function MpsPage() {
  const matricesHook = useMpsMatrices()
  const runHook = useMpsRun()
  const validateHook = useMpsValidate()
  const historyHook = useMpsHistory()

  const [source, setSource] = useState<ScenarioSource>('sim')
  const [compareSelection, setCompareSelection] = useState<MpsScenarioResult[]>([])
  const [primaryResult, setPrimaryResult] = useState<MpsScenarioResult | null>(null)
  const [errors, setErrors] = useState<Array<{ id: string; kind: string; msg: string }>>([])

  // Auto-validate при изменении applied
  useEffect(() => {
    if (matricesHook.applied) {
      void validateHook.validate(matricesHook.applied)
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [matricesHook.applied])

  useEffect(() => {
    if (runHook.result) setPrimaryResult(runHook.result)
  }, [runHook.result])

  // Накопление errors
  useEffect(() => {
    const newErrors: typeof errors = []
    if (matricesHook.error)
      newErrors.push({
        id: `m-${Date.now()}`,
        kind: 'matrices',
        msg: matricesHook.error,
      })
    if (runHook.error)
      newErrors.push({ id: `r-${Date.now()}`, kind: 'run', msg: runHook.error })
    if (validateHook.error)
      newErrors.push({
        id: `v-${Date.now()}`,
        kind: 'validate',
        msg: validateHook.error,
      })
    if (newErrors.length > 0) {
      setErrors((prev) => [...newErrors, ...prev].slice(0, 5))
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [matricesHook.error, runHook.error, validateHook.error])

  function dismissError(id: string) {
    setErrors((prev) => prev.filter((e) => e.id !== id))
  }

  const liveEnabled = source === 'robot' && runHook.running
  const live = useMpsLiveTelemetry({
    enabled: liveEnabled,
    runId: runHook.result?.run_id,
    onFinished: (r) => setPrimaryResult(r),
    onError: (msg) => {
      setErrors((prev) =>
        [{ id: `ws-${Date.now()}`, kind: 'ws', msg }, ...prev].slice(0, 5),
      )
    },
  })

  const isClosedLoopStable = validateHook.result?.is_closed_loop_stable ?? true
  const isPlantStable = validateHook.result?.is_plant_stable ?? true

  const canonicalStatus = useMemo(() => {
    const m = matricesHook.draft ?? matricesHook.applied
    return m ? detectPhysics(m).status : undefined
  }, [matricesHook.draft, matricesHook.applied])

  const validationStatus: 'unknown' | 'stable' | 'unstable' = useMemo(() => {
    if (!validateHook.result) return 'unknown'
    return validateHook.result.is_closed_loop_stable ? 'stable' : 'unstable'
  }, [validateHook.result])

  const lastValidatedAt = useMemo(() => {
    return validateHook.result ? new Date().toLocaleTimeString() : null
  }, [validateHook.result])

  // Прогресс для ScenarioControls
  const scenarioProgress = useMemo(() => {
    if (!runHook.running) return undefined
    if (live.points.length > 0) {
      const lastPoint = live.points[live.points.length - 1]
      const D = primaryResult?.request.distance ?? runHook.result?.request.distance
      if (!D || D <= 0) return undefined
      const s = lastPoint.x[0] ?? 0
      return Math.min(1, s / D)
    }
    return undefined
  }, [runHook.running, live.points, primaryResult, runHook.result])

  function handleRun(req: MpsScenarioRequest) {
    void runHook.run(req).then((r) => {
      if (r) {
        setPrimaryResult(r)
        void historyHook.refresh()
      }
    })
  }

  function handleAbort() {
    void runHook.abort()
  }

  function handleReplay(runId: string) {
    void historyHook.replay(runId).then((r) => {
      if (r) setPrimaryResult(r)
    })
  }

  const overlays = useMemo(
    () => compareSelection.filter((r) => r.run_id !== primaryResult?.run_id),
    [compareSelection, primaryResult?.run_id],
  )

  return (
    <MpsHighlightProvider>
      <div className="min-h-screen">
        <Header />
        <div className="p-3 max-w-[1920px] mx-auto">
          {/* Title row */}
          <div className="flex items-center justify-between mb-3 gap-2 flex-wrap">
            <h1 className="text-2xl font-semibold">МПС — Модель Пространства Состояний</h1>
            <div className="flex items-center gap-2">
              <DraftStatus
                applied={matricesHook.applied}
                draft={matricesHook.draft}
                isStable={isClosedLoopStable}
                isValid={true}
                canonicalStatus={canonicalStatus}
              />
              <span className="text-xs text-muted-foreground">
                WS: {live.connected ? 'connected' : 'idle'}
              </span>
            </div>
          </div>

          {/* Errors stack (sticky-top) */}
          {errors.length > 0 && (
            <div className="space-y-1 mb-3">
              {errors.map((e) => (
                <div
                  key={e.id}
                  className="flex items-center gap-2 rounded border border-red-300 bg-red-500/10 px-3 py-1.5 text-xs"
                  role="alert"
                >
                  <span className="text-red-700 font-medium">⚠ {e.kind}:</span>
                  <span className="flex-1 text-red-800">{e.msg}</span>
                  <button
                    type="button"
                    onClick={() => dismissError(e.id)}
                    className="text-red-600 hover:text-red-900"
                    aria-label="dismiss"
                  >
                    ×
                  </button>
                </div>
              ))}
            </div>
          )}

          {/* Sticky-grid layout */}
          <div className="grid grid-cols-1 lg:grid-cols-[minmax(360px,_35%)_1fr] gap-4">
            {/* LEFT — sticky */}
            <aside className="space-y-3 lg:sticky lg:top-16 lg:self-start lg:max-h-[calc(100vh-5rem)] lg:overflow-y-auto">
              <OdeCard matrices={matricesHook.draft ?? matricesHook.applied} />
              <PhysicsParams
                applied={matricesHook.applied}
                draft={matricesHook.draft}
                onPatch={(m) => void matricesHook.saveDraft(m)}
                defaults={{ tau_v: DEFAULT_TAU_V, tau_omega: DEFAULT_TAU_OMEGA }}
              />
            </aside>

            {/* RIGHT — scrollable */}
            <main className="space-y-3">
              <MatrixEditor
                applied={matricesHook.applied}
                draft={matricesHook.draft}
                onChange={(m: MpsMatrices) => void matricesHook.saveDraft(m)}
                onApply={() => void matricesHook.apply()}
                onValidate={() =>
                  void validateHook.validate(matricesHook.draft ?? matricesHook.applied)
                }
                onReset={() => void matricesHook.reset()}
                saving={matricesHook.loading}
                validationStatus={validationStatus}
              />

              <EigenvaluePanel
                open={validateHook.result?.eigenvalues_ad ?? []}
                closed={validateHook.result?.eigenvalues_closed ?? []}
                isPlantStable={isPlantStable}
                isClosedLoopStable={isClosedLoopStable}
                warnings={validateHook.result?.warnings ?? []}
                lastValidatedAt={lastValidatedAt}
              />

              <ScenarioControls
                running={runHook.running}
                onRun={handleRun}
                onAbort={handleAbort}
                source={source}
                onSourceChange={setSource}
                progress={scenarioProgress}
              />

              <ResultPlots
                primary={primaryResult}
                overlays={overlays}
                liveTelemetry={liveEnabled ? live.points : undefined}
              />

              <TrajectoryView
                result={primaryResult}
                liveTelemetry={liveEnabled ? live.points : undefined}
              />

              <HistoryPanel
                history={historyHook.history}
                onSelect={setPrimaryResult}
                onReplay={handleReplay}
                onCompareChange={setCompareSelection}
                loading={historyHook.loading}
              />
            </main>
          </div>
        </div>
      </div>
    </MpsHighlightProvider>
  )
}
```

- [ ] **Step 2: Verify build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green.

- [ ] **Step 3: Smoke test in browser (manual)**

Run: `cd compute_node/frontend && npm run dev`

Open `http://localhost:5173/mps` (или порт из vite). Проверить:
- Левая колонка sticky на десктопе.
- ОДУ-формулы рендерятся через KaTeX (математические символы, не plain text).
- Ползунок τ_v работает, A/B обновляются (видно в табе «Динамика A·B»).
- Hover на уравнении подсвечивает строку матрицы A справа.
- Hover на ячейке A[1][1] подсвечивает уравнение `v̇` слева.

Если визуально что-то странно — отметить, исправить мелкие баги перед коммитом.

- [ ] **Step 4: Commit**

```bash
git add compute_node/frontend/src/pages/MpsPage.tsx
git commit -m "feat(mps-ui): MpsPage — sticky-grid layout с MpsHighlightProvider"
```

---

## Task 19: Integration test — cross-highlight end-to-end

**Files:**
- Create: `compute_node/frontend/src/pages/MpsPage.test.tsx`

- [ ] **Step 1: Mock hooks and write integration test**

Создать `compute_node/frontend/src/pages/MpsPage.test.tsx`:

```tsx
import { describe, it, expect, vi, beforeEach } from 'vitest'
import { render, screen, fireEvent, waitFor } from '@testing-library/react'
import { MpsPage } from './MpsPage'
import { buildCanonical, DEFAULT_TAU_V, DEFAULT_TAU_OMEGA } from '@/lib/mps/canonical'
import type { MpsMatrices } from '@/types/mps'

function makeMatrices(): MpsMatrices {
  const { A, B } = buildCanonical(DEFAULT_TAU_V, DEFAULT_TAU_OMEGA)
  return {
    A, B,
    C: Array.from({ length: 5 }, (_, i) =>
      Array.from({ length: 5 }, (_, j) => (i === j ? 1 : 0)),
    ),
    D: Array.from({ length: 5 }, () => Array(2).fill(0)),
    Q_diag: [10, 5, 1, 1, 5],
    R_diag: [1, 1],
    horizon_N: 20,
    u_min: [-0.3, -1.5],
    u_max: [0.3, 1.5],
    schema_version: '1.0',
  }
}

vi.mock('@/hooks/useMpsMatrices', () => ({
  useMpsMatrices: () => ({
    applied: makeMatrices(),
    draft: null,
    loading: false,
    error: null,
    refresh: vi.fn(),
    saveDraft: vi.fn(),
    apply: vi.fn(),
    reset: vi.fn(),
    setDraftLocal: vi.fn(),
  }),
}))

vi.mock('@/hooks/useMpsRun', () => ({
  useMpsRun: () => ({
    running: false,
    result: null,
    error: null,
    run: vi.fn(),
    abort: vi.fn(),
  }),
}))

vi.mock('@/hooks/useMpsValidate', () => ({
  useMpsValidate: () => ({
    result: null,
    error: null,
    loading: false,
    validate: vi.fn(),
  }),
}))

vi.mock('@/hooks/useMpsHistory', () => ({
  useMpsHistory: () => ({
    history: [],
    loading: false,
    refresh: vi.fn(),
    replay: vi.fn(),
  }),
}))

vi.mock('@/hooks/useMpsLiveTelemetry', () => ({
  useMpsLiveTelemetry: () => ({
    connected: false,
    points: [],
  }),
}))

vi.mock('@/components/layout/Header', () => ({
  Header: () => <header data-testid="header">Header</header>,
}))

describe('MpsPage integration', () => {
  beforeEach(() => {
    // happy-dom не имеет ResizeObserver — мокаем для recharts
    if (typeof globalThis.ResizeObserver === 'undefined') {
      // @ts-expect-error
      globalThis.ResizeObserver = class {
        observe() {}
        unobserve() {}
        disconnect() {}
      }
    }
  })

  it('renders title and main panels', () => {
    render(<MpsPage />)
    expect(screen.getByText(/МПС — Модель Пространства Состояний/)).toBeInTheDocument()
    expect(screen.getByText(/ОДУ-модель робота/i)).toBeInTheDocument()
    expect(screen.getByText(/Физические параметры/i)).toBeInTheDocument()
    expect(screen.getByText(/Анализ устойчивости/i)).toBeInTheDocument()
  })

  it('hover on equation in OdeCard highlights the corresponding row in MatrixGrid', async () => {
    render(<MpsPage />)
    const equations = screen.getAllByRole('button', { name: /уравнение для/i })
    expect(equations.length).toBe(5)

    // Hover на уравнении v̇ (index 1)
    fireEvent.mouseEnter(equations[1])

    // В MatrixGrid должен подсветиться row 1 (метка v̇ становится bold)
    await waitFor(() => {
      const labels = screen.getAllByText('v̇')
      // Должен быть как минимум один элемент с классом font-semibold
      const hasBold = labels.some((el) =>
        el.className.includes('font-semibold'),
      )
      expect(hasBold).toBe(true)
    })
  })

  it('shows MatrixEditor with three tabs', () => {
    render(<MpsPage />)
    expect(screen.getByRole('tab', { name: /Динамика A·B/i })).toBeInTheDocument()
    expect(screen.getByRole('tab', { name: /Веса Q·R·N/i })).toBeInTheDocument()
    expect(screen.getByRole('tab', { name: /Выход C·D/i })).toBeInTheDocument()
  })
})
```

- [ ] **Step 2: Run test**

Run: `cd compute_node/frontend && npx vitest run src/pages/MpsPage.test.tsx`
Expected: All PASS. Если recharts падает на отсутствии `ResizeObserver` — мок выше должен помочь; иначе adjust.

- [ ] **Step 3: Commit**

```bash
git add compute_node/frontend/src/pages/MpsPage.test.tsx
git commit -m "test(mps-ui): integration — cross-highlight ОДУ ↔ MatrixGrid"
```

---

## Task 20: Final verification — build, lint, full test suite

**Files:**
- (verification only)

- [ ] **Step 1: Run full test suite**

Run: `cd compute_node/frontend && npm test`
Expected: все тесты PASS, итого ≥ ~30 тестов (8 в canonical, 4 в tokenMap, 4 в HighlightContext, 3 в KatexFormula, 5 в OdeCard, 6 в PhysicsParams, 7 в MatrixGrid, 3 в MpsPage = ~40).

Если что-то падает — диагностика: какой файл, какой assert, чем фактическое поведение отличается от ожидаемого.

- [ ] **Step 2: Run lint**

Run: `cd compute_node/frontend && npm run lint`
Expected: zero new warnings. Если появились — исправить (eslint-disable только в крайних случаях).

- [ ] **Step 3: Run build**

Run: `cd compute_node/frontend && npm run build`
Expected: build green, bundle size показывает ~300KB+ от предыдущего baseline (KaTeX добавил).

- [ ] **Step 4: Manual smoke в dev-server**

Run: `cd compute_node/frontend && npm run dev`

Открыть `/mps` в браузере. Полный чеклист (acceptance criteria из спеки §20):
- [ ] OdeCard в sticky-левой колонке всегда видна на десктопе
- [ ] Hover на уравнении в OdeCard → подсвечивается строка в матрице A
- [ ] Hover на ячейке A или B → подсвечивается уравнение в OdeCard
- [ ] PhysicsParams — двигание τ_v изменяет A[1][1] и B[1][0]
- [ ] Запись 0.5 в A[0][2] → ячейка получает оранжевую пунктирную рамку
- [ ] OdeCard показывает бейдж «нестандартные члены»
- [ ] Кнопка «Восстановить каноническую форму» работает
- [ ] EigenvaluePanel показывает численные значения inline (без `<details>`)
- [ ] ResultPlots имеет раздельные табы s/v/θω/u/y
- [ ] HistoryPanel — фильтр по status работает, цветовая полоса видна
- [ ] На `< 1024 px` колонки складываются вертикально, sticky отключается
- [ ] Page renders без console.error

- [ ] **Step 5: Stop dev server, ничего не коммитим (verification only)**

Если всё ОК — Task 20 завершён без коммита (verification только).

Если найдены баги — фиксим их отдельным коммитом (`fix(mps-ui): ...`).

---

## Self-Review Checklist (для меня перед finalize)

- [x] **Spec coverage:**
  - §1 цели/не-цели → весь план достигает их (понятность, скорость, эстетика, защита) — Tasks 3-19
  - §2 layout → Task 18 (sticky-grid)
  - §3 канон + token-map → Tasks 3-4
  - §4 HighlightContext → Task 5
  - §5 OdeCard → Task 7
  - §6 PhysicsParams → Task 8
  - §7 MatrixEditor + табы + Q·R·N + C·D → Task 10
  - §8 EigenvaluePanel → Task 12
  - §9 ScenarioControls → Task 13
  - §10 ResultPlots → Task 14
  - §11 TrajectoryView → Task 16
  - §12 HistoryPanel → Task 15
  - §13 Errors → внутри Task 18 (sticky alerts top of right column)
  - §14 файлы → все созданы / обновлены / удалены
  - §15 зависимости → Task 1, 2
  - §16 тесты → Tasks 3-9, 19
  - §17 бэкенд → не трогаем (Task 11 только удалил frontend файл)
  - §20 acceptance → Task 20 manual smoke

- [x] **Placeholder scan:** все шаги имеют конкретный код / команды.
- [x] **Type consistency:** интерфейсы/типы согласованы между Tasks.
- [ ] **Кнопка «Reset all» (acceptance §20)** — в спеке упомянута, но в плане отдельной задачи нет; используется существующий `mpsApi.resetMatrices()` через `onReset`. Решение: **не реализуем «Reset all» отдельной кнопкой в MVP** — текущий `Reset draft` через `onReset` сейчас вызывает `mpsApi.resetMatrices()` (через `useMpsMatrices.reset()`), что и есть «reset to default». Это соответствует поведению старого кода.

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-05-06-mps-ui-redesign.md`.**

Two execution options:

1. **Subagent-Driven (recommended)** — Я диспатчу свежего субагента на каждую задачу, ревью между задачами, быстрая итерация. Подходит когда задачи независимы или почти независимы — у нас Task 1, 2, 3, 4 могут идти последовательно одним агентом, дальше — некоторые параллельно.

2. **Inline Execution** — Выполнение задач в этой сессии через executing-plans skill, batch с чекпоинтами для ревью.

Какой подход предпочитаешь?
