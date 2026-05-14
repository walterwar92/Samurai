# MPS Continuous-Matrix Contract + ZOH Discretization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Заставить бэкенд МПС ZOH-дискретизировать непрерывные канонические матрицы, которые шлёт фронтенд, чтобы сценарий «проехать D метров вперёд» был устойчив в симуляторе и на роботе.

**Architecture:** Контракт `MpsMatrices.A/B` меняется на непрерывные `A_c/B_c`. Публичный `zoh_discretize(A,B,Ts)` вызывается в трёх точках потребления (`mps_runner`, `mps_node`); `/validate` репортит `λ(Ad)` после дискретизации с нюансированным предупреждением о маргинальной устойчивости. MATLAB генерирует обе модели — legacy `control:` (дискретная) и каноническую `mps:` (непрерывная). Единый период `Ts = 0.02 с` в `config.yaml mps.plant.Ts`.

**Tech Stack:** Python (numpy, scipy, FastAPI, pytest), MATLAB/Octave, React/TypeScript (vitest), YAML.

**Базовая спека:** `docs/superpowers/specs/2026-05-14-mps-continuous-discretization-design.md`

**Ветка:** `fix/mps-continuous-discretization` (уже создана от `fix/camera-flip`).

---

## ⚠ PLAN AMENDMENT 2026-05-14 — Option D (`e_int` → ∫heading) + `MPCController` fix

**Status:** Tasks 1-3 committed (`24210ef`, `ddd7741`, `65e799f`, `b855747`). During Task 3 two pre-existing bugs surfaced (spec §2.6):

1. **`MPCController.__init__` overwrites the computed gain** with the legacy `control.matrices.K_mpc` from config (proof: `K_first[0,:3]` after `__init__` = `[2.80934, 0, 0]` = config's `K_mpc`). Same footgun for `Pf` / `control.weights`.
2. **The canonical `ė_int = v_target − v` makes the model uncontrollable** (`s + e_int = const`, ctrb rank 4/5, DARE fails). User chose **Option D**: redefine `ė_int = −θ` (integral of *heading* error; `θ_ref ≡ 0` so no exogenous term) — `A_c[4][2] = −1` instead of `A_c[4][1] = −1`. Verified by simulation: rank 5/5, DARE OK, closed loop strictly stable (max|λ| ≈ 0.991), scenario `reached`. Option «∫position-error» (`A_c[4][0]=−1`) was also controllable but would have required an `x_ref[4]=−∫s_ref` feedforward in the runner — Option D works with the runner unchanged.

**Effect on the plan:**
- **NEW Task C1** below — runs BEFORE Task 4, fixes both bugs atomically.
- **Tasks 4, 5, 8, 11** — `A` fixture row 4 changes `[0,-1,0,0,0]` → `[0,0,-1,0,0]` (see "Task deltas" below).
- **Task 12** expands — also updates `canonical.ts` / `tokenMap.ts` / `OdeCard` equation.
- Task 3's committed `test_closed_loop_eigenvalues_returns_5_5` has a `< 1.5` band-aid — Task C1 replaces it with the honest `< 1.0`.

---

### Task C1: `e_int` → ∫heading-error + fix `MPCController` gain-override

**Files:**
- Modify: `pi_nodes/control/mpc_controller.py` (`__init__`, `_compute_terminal_penalty`)
- Modify: `config.yaml` (`mps.matrices.A` row 4)
- Modify: `compute_node/mps_runner.py` (`__main__` `A_DEFAULT` row 4)
- Test: `tests/test_mps_runner.py`, `tests/test_mpc_controller.py`

- [ ] **Step 1: Update tests to Option-D / fixed-`MPCController` expectations (RED)**

In `tests/test_mps_runner.py`, change `_A_CANONICAL` row 4 (e_int) from `[0.0, -1.0, 0.0, 0.0, 0.0]` to `[0.0, 0.0, -1.0, 0.0, 0.0]` (ė_int = −θ):

```python
_A_CANONICAL = [
    [0.0,  1.0,         0.0,  0.0,         0.0],
    [0.0, -1.0/_TAU_V,  0.0,  0.0,         0.0],
    [0.0,  0.0,         0.0,  1.0,         0.0],
    [0.0,  0.0,         0.0, -1.0/_TAU_W,  0.0],
    [0.0,  0.0,        -1.0,  0.0,         0.0],
]
```

Replace `test_closed_loop_eigenvalues_returns_5_5` (the `< 1.5` band-aid) with the honest version:

```python
def test_closed_loop_eigenvalues_returns_5_5():
    m = _default_matrices()
    eig_open, eig_closed = closed_loop_eigenvalues(m)
    assert len(eig_open) == 5
    assert len(eig_closed) == 5
    assert all(np.isfinite(z) for z in eig_closed)
    # Option D canonical model (ė_int = −θ) is fully controllable —
    # the MPC places ALL closed-loop poles strictly inside the unit circle.
    assert max(abs(z) for z in eig_closed) < 1.0
    # Open loop keeps 3 integrator poles on |λ|=1 (s, θ, e_int chain).
    assert sum(abs(abs(z) - 1.0) < 1e-6 for z in eig_open) == 3
```

In `tests/test_mpc_controller.py`, add two tests (match the file's existing import/style):

```python
def test_explicit_matrices_ignore_config_gain():
    """MPCController(Ad=, Bd=) with EXPLICIT matrices must compute its own
    K_first — NOT inherit control.matrices.K_mpc from config (that gain is
    for the legacy [px,py,θ,v,ω] model). Spec 2026-05-14 §2.6."""
    import numpy as np
    from pi_nodes.control.mpc_controller import MPCController
    Ad = np.diag([0.9, 0.8, 0.95, 0.85, 0.88])
    Bd = np.zeros((5, 2)); Bd[1, 0] = 0.1; Bd[3, 1] = 0.1
    Q = np.diag([10., 10., 5., 1., 1.]); R = np.diag([1., 1.])
    mpc = MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, N=10, Pf=Q,
                        u_min=[-0.3, -2.], u_max=[0.3, 2.], solver='clip')
    K_init = mpc.K_first.copy()
    mpc._build_qp_matrices()  # clean recompute — must match __init__
    np.testing.assert_allclose(
        K_init, mpc.K_first, atol=1e-12,
        err_msg="__init__ K_first differs from clean recompute — config "
                "K_mpc override leaked into the explicit-matrices path")


def test_explicit_matrices_compute_own_terminal_penalty():
    """With explicit Ad/Bd and no Pf, MPCController computes Pf via DARE on
    the supplied model — not control.matrices.Pf (legacy). Spec §2.6."""
    import numpy as np
    from scipy.linalg import solve_discrete_are
    from pi_nodes.control.mpc_controller import MPCController
    Ad = np.diag([0.9, 0.8, 0.95, 0.85, 0.88])
    Bd = np.zeros((5, 2)); Bd[1, 0] = 0.1; Bd[3, 1] = 0.1
    Q = np.diag([10., 10., 5., 1., 1.]); R = np.diag([1., 1.])
    mpc = MPCController(Ad=Ad, Bd=Bd, Q=Q, R=R, N=10,
                        u_min=[-0.3, -2.], u_max=[0.3, 2.], solver='clip')
    expected_pf = solve_discrete_are(Ad, Bd, Q, R)
    np.testing.assert_allclose(
        mpc.Pf, expected_pf, atol=1e-6,
        err_msg="Pf not computed from the supplied model — config Pf leaked in")
```

- [ ] **Step 2: Run — confirm RED**

Run: `python -m pytest tests/test_mps_runner.py::test_closed_loop_eigenvalues_returns_5_5 tests/test_mpc_controller.py::test_explicit_matrices_ignore_config_gain tests/test_mpc_controller.py::test_explicit_matrices_compute_own_terminal_penalty -v`
Expected: FAIL — `test_closed_loop_eigenvalues_returns_5_5` (config still has `A[4][1]=−1` → uncontrollable → closed loop not `< 1.0`); both `test_mpc_controller` tests (the K_mpc/Pf config override is still unconditional).

- [ ] **Step 3: Fix `MPCController.__init__` + config + mps_runner `__main__`**

In `pi_nodes/control/mpc_controller.py`, in `__init__`, capture the legacy-path flag at the very top of the method body (BEFORE `Ad`/`Bd` get resolved from config):

```python
        # True when built from config defaults (legacy [px,py,θ,v,ω] model).
        # Explicit Ad/Bd ⇒ a DIFFERENT model ⇒ config K_mpc/Pf must NOT leak in.
        _from_config = Ad is None or Bd is None
```

Gate the `Pf` config-load (the `if Pf is None:` block) on `_from_config`:

```python
        if Pf is None:
            Pf_cfg = cfg("control.matrices.Pf", None) if _from_config else None
            if Pf_cfg is not None:
                Pf = np.asarray(Pf_cfg, dtype=float)
            else:
                Pf = self._compute_terminal_penalty()
        self.Pf = np.asarray(Pf, dtype=float)
```

Gate the `K_mpc` override (the block at the end of `__init__`) on `_from_config`:

```python
        # ── Precomputed K_first from config — LEGACY PATH ONLY ──────
        # Explicit Ad/Bd ⇒ controller is for a different model than the
        # config's legacy [px,py,θ,v,ω]; the config gain must not be used.
        if _from_config:
            K_mpc_cfg = cfg("control.matrices.K_mpc", None)
            if K_mpc_cfg is not None:
                K_pre = np.asarray(K_mpc_cfg, dtype=float)
                if K_pre.shape == (self.r, self.n):
                    self.K_first = K_pre
```

Replace `_compute_terminal_penalty` to use the instance weights instead of re-reading config:

```python
    def _compute_terminal_penalty(self) -> np.ndarray:
        """If Pf not given, solve DARE for a guaranteed-stable terminal cost.

        Uses the instance weights `self.Q/self.R` (NOT config) — the
        controller may be built for a non-legacy model.
        """
        from scipy.linalg import solve_discrete_are

        return solve_discrete_are(self.Ad, self.Bd, self.Q, self.R)
```

In `config.yaml`, `mps.matrices.A` — change row 4 from `      - [0, -1, 0, 0, 0]` to `      - [0, 0, -1, 0, 0]` (ė_int = −θ). Update the adjacent comment line `# A_c: ... [4][1]=-1` → `[4][2]=-1`.

In `compute_node/mps_runner.py`, the `__main__` block's `A_DEFAULT` — change row 4 from `[0.0, -1.0, 0.0, 0.0, 0.0]` to `[0.0, 0.0, -1.0, 0.0, 0.0]`.

- [ ] **Step 4: Run — confirm GREEN**

Run: `python -m pytest tests/test_mps_runner.py tests/test_mpc_controller.py tests/test_state_space_extended.py tests/test_zoh_discretization.py tests/test_lqr_controller.py -q`
Expected: PASS — all green. (`test_lqr_controller.py` / `test_state_space_extended.py` confirm the legacy path is untouched.) If an existing `test_mpc_controller.py` test breaks, STOP and report — do not paper over.

- [ ] **Step 5: Commit**

```bash
git add pi_nodes/control/mpc_controller.py config.yaml compute_node/mps_runner.py tests/test_mps_runner.py tests/test_mpc_controller.py
git commit -m "fix(mps): e_int → ∫ошибки курса + MPCController не берёт legacy-гейн из config"
```

---

### Task deltas — apply when you reach each task

Option D: `ė_int = −θ`, i.e. `A_c[4][2] = −1` (e_int row = `[0, 0, -1, 0, 0]`).

- **Task 4** (`tests/test_mps_router.py`): in `matrices_payload`, `A` row 4 → `[0.0, 0.0, -1.0, 0.0, 0.0]`. Rest of Task 4 stands — the open-loop marginal-stability nuance is still needed (open loop keeps 3 integrators); `test_validate_canonical_plant_marginal_not_unstable`'s `is_closed_loop_stable is True` is now genuinely correct.
- **Task 5** (`tests/test_mps_node.py`): in `_good_matrices`, `A` row 4 → `[0.0, 0.0, -1.0, 0.0, 0.0]`. Rest stands.
- **Task 6** (docs): the canonical equation is `ė_int = −θ` (`A_c[4][2]=−1`), not `ė_int = v_target − v`. Use that wherever the model is described.
- **Task 8** (`build_canonical_mps.m`): the `A` matrix's e_int row is `[0, 0, -1, 0, 0]` (`A(5,3) = -1`), not `A(5,2) = -1`. Equation comment: `e_int_dot = -theta`. In `test_build_canonical_mps.m`: assert `A(5,3) == -1` (not `A(5,2)`), and the non-pattern mask sets `mask(5,3) = false`.
- **Task 11** (`test_export_to_yaml.m`): the `mps_export` fixture `A_c` row 4 → `0 0 -1 0 0`.
- **Task 12** — EXPANDED. In addition to the `OdeCard.tsx` Ts label (`50` → `20` мс):
  - `compute_node/frontend/src/lib/mps/canonical.ts` — `CANONICAL_PATTERN_A`: change the e_int entry `{ row: 4, col: 1, role: { kind: 'fixed', value: -1 } }` → `{ row: 4, col: 2, role: { kind: 'fixed', value: -1 } }`.
  - `compute_node/frontend/src/lib/mps/tokenMap.ts` — change `{ id: 'coef_eint_v', matrix: 'A', row: 4, col: 1, equationRow: 4, description: '∂ė_int/∂v = −1' }` → `{ id: 'coef_eint_theta', matrix: 'A', row: 4, col: 2, equationRow: 4, description: '∂ė_int/∂θ = −1' }`. `grep -rn coef_eint_v compute_node/frontend/src` and update any other references.
  - `compute_node/frontend/src/components/mps/OdeCard.tsx` — `EQUATIONS[4]`: both `symbolic` and `numericFormula` `'\\dot{e}_{int} = v_{target} - v'` → `'\\dot{e}_{int} = -\\theta'`.
  - Run `cd compute_node/frontend && npm run test -- mps`.

---

## File Structure

| Файл | Ответственность | Действие |
|---|---|---|
| `pi_nodes/control/state_space_model.py` | дискретный плант + публичный `zoh_discretize` | Modify |
| `compute_node/mps_runner.py` | идеальный sim + eigenvalue-анализ — дискретизирует вход | Modify |
| `compute_node/dashboard/routers/mps.py` | `/validate` — нюанс маргинальной устойчивости | Modify |
| `pi_nodes/nodes/mps_node.py` | Pi-orchestrator — дискретизирует в `_on_matrices_set` | Modify |
| `compute_node/dashboard/schemas/mps.py` | контракт `MpsMatrices` — docstring A/B | Modify |
| `config.yaml` | секция `mps:` — непрерывная каноническая + `mps.plant` | Modify |
| `docs/mps/api.md` | контракт A/B | Modify |
| `docs/superpowers/specs/2026-05-05-mps-state-space-design.md` | пометка о смене семантики A/B | Modify |
| `matlab/build_canonical_mps.m` | аналитическая непрерывная каноническая модель | Create |
| `matlab/samurai_params.m` | + под-структура `p.mps` | Modify |
| `matlab/main.m` | + ветка синтеза МПС | Modify |
| `matlab/export_to_yaml.m` | + запись блока `mps:` | Modify |
| `compute_node/frontend/src/components/mps/OdeCard.tsx` | подпись Ts (50→20 мс) | Modify |
| `tests/test_zoh_discretization.py` | тест `zoh_discretize` | Create |
| `tests/test_mps_runner.py` | фикстуры → непрерывные; тест propagation | Modify |
| `tests/test_mps_router.py` | фикстуры → непрерывные; тест warning | Modify |
| `tests/test_mps_node.py` | фикстуры → непрерывные; тест дискретизации | Modify |
| `matlab/test_build_canonical_mps.m` | тест структуры канонической модели | Create |
| `matlab/test_export_to_yaml.m` | + проверка блока `mps:` | Modify |

**Каноническая непрерывная модель** (используется ВЕЗДЕ — фикстуры, config, MATLAB; держать значения консистентными):

```
τ_v = 0.15,  τ_w = 0.10,  Ts = 0.02
A_c = [[0,        1,       0,   0,       0],
       [0,  -1/τ_v,       0,   0,       0],
       [0,        0,       0,   1,       0],
       [0,        0,       0, -1/τ_w,    0],
       [0,       -1,       0,   0,       0]]
B_c = [[0,       0],
       [1/τ_v,   0],
       [0,       0],
       [0,    1/τ_w],
       [0,       0]]
C = I₅,  D = 0₅ₓ₂
```

`1/0.15 = 6.666666666666667`, `1/0.10 = 10.0`.

---

## Task 1: Публичный `zoh_discretize` в `state_space_model.py`

**Files:**
- Modify: `pi_nodes/control/state_space_model.py:46-57` (функция `_zoh_discretize`), `:121` (вызов)
- Test: `tests/test_zoh_discretization.py`

- [ ] **Step 1: Написать падающий тест**

Создать `tests/test_zoh_discretization.py`:

```python
"""Tests for the public zoh_discretize helper in state_space_model."""
from __future__ import annotations

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

scipy = pytest.importorskip('scipy')

from pi_nodes.control.state_space_model import zoh_discretize  # noqa: E402


def test_zoh_scalar_known_values():
    """Continuous A=[[-10]], B=[[10]], Ts=0.02 →
    Ad = exp(-0.2) ≈ 0.818731, Bd = (1-exp(-0.2)) ≈ 0.181269."""
    Ad, Bd = zoh_discretize(np.array([[-10.0]]), np.array([[10.0]]), 0.02)
    np.testing.assert_allclose(Ad, [[0.8187307530779818]], atol=1e-9)
    np.testing.assert_allclose(Bd, [[0.1812692469220182]], atol=1e-9)


def test_zoh_eigenvalue_mapping():
    """ZOH maps continuous eigenvalues by λ_d = exp(λ_c · Ts)."""
    tau_v, tau_w, Ts = 0.15, 0.10, 0.02
    A_c = np.array([
        [0.0,  1.0,       0.0,  0.0,       0.0],
        [0.0, -1.0/tau_v, 0.0,  0.0,       0.0],
        [0.0,  0.0,       0.0,  1.0,       0.0],
        [0.0,  0.0,       0.0, -1.0/tau_w, 0.0],
        [0.0, -1.0,       0.0,  0.0,       0.0],
    ])
    B_c = np.array([
        [0.0,       0.0],
        [1.0/tau_v, 0.0],
        [0.0,       0.0],
        [0.0,       1.0/tau_w],
        [0.0,       0.0],
    ])
    Ad, _ = zoh_discretize(A_c, B_c, Ts)
    lam_c = np.sort_complex(np.linalg.eigvals(A_c))
    lam_d = np.sort_complex(np.linalg.eigvals(Ad))
    np.testing.assert_allclose(lam_d, np.exp(lam_c * Ts), atol=1e-9)


def test_zoh_shapes():
    Ad, Bd = zoh_discretize(np.zeros((5, 5)), np.ones((5, 2)), 0.05)
    assert Ad.shape == (5, 5)
    assert Bd.shape == (5, 2)
```

- [ ] **Step 2: Запустить — убедиться что падает**

Run: `pytest tests/test_zoh_discretization.py -v`
Expected: FAIL — `ImportError: cannot import name 'zoh_discretize'`

- [ ] **Step 3: Сделать `_zoh_discretize` публичным**

В `pi_nodes/control/state_space_model.py` переименовать `def _zoh_discretize(` → `def zoh_discretize(` (строка 46). Обновить единственный вызов на строке 121:

```python
        Ad, Bd = zoh_discretize(A, B, Ts)
```

- [ ] **Step 4: Запустить тесты — убедиться что проходят**

Run: `pytest tests/test_zoh_discretization.py -v`
Expected: PASS (3 passed)

Run: `pytest tests/test_state_space_extended.py tests/test_mpc_controller.py tests/test_lqr_controller.py -q`
Expected: PASS — переименование не сломало существующие тесты модели.

- [ ] **Step 5: Commit**

```bash
git add pi_nodes/control/state_space_model.py tests/test_zoh_discretization.py
git commit -m "feat(control): публичный zoh_discretize для переиспользования в МПС"
```

---

## Task 2: `config.yaml mps:` → непрерывная каноническая форма + `mps.plant`

**Files:**
- Modify: `config.yaml` (секция `mps:`, строки ~413-466)

- [ ] **Step 1: Заменить блок матриц и комментарий**

В `config.yaml` найти секцию `mps:`. Заменить комментарий-миф и `matrices.A/B` на непрерывную каноническую форму, добавить `plant`. Старый блок (от `  # Матрицы по умолчанию — копия из control.matrices` до конца `matrices:` с `D:`) заменить на:

```yaml
  # Непрерывная каноническая ОДУ-модель [s, v, θ, ω, e_int].
  # A/B — НЕПРЕРЫВНЫЕ (контракт docs/mps/api.md); бэкенд ZOH-дискретизирует
  # при mps.plant.Ts. Генерируется matlab/main.m (ветка МПС).
  plant:
    tau_v: 0.15        # с — постоянная времени linear-мотора
    tau_w: 0.10        # с — постоянная времени angular-мотора
    Ts:    0.02        # с — период дискретизации и tick-loop (50 Гц)

  matrices:
    # A_c: [0][1]=1, [1][1]=-1/τ_v, [2][3]=1, [3][3]=-1/τ_w, [4][1]=-1
    A:
      - [0, 1, 0, 0, 0]
      - [0, -6.66667, 0, 0, 0]
      - [0, 0, 0, 1, 0]
      - [0, 0, 0, -10, 0]
      - [0, -1, 0, 0, 0]
    # B_c: [1][0]=1/τ_v, [3][1]=1/τ_w
    B:
      - [0, 0]
      - [6.66667, 0]
      - [0, 0]
      - [0, 10]
      - [0, 0]
    # C по умолчанию — единичная (y == x). Меняется в UI для отображения y(t).
    C:
      - [1, 0, 0, 0, 0]
      - [0, 1, 0, 0, 0]
      - [0, 0, 1, 0, 0]
      - [0, 0, 0, 1, 0]
      - [0, 0, 0, 0, 1]
    # D по умолчанию — нулевая.
    D:
      - [0, 0]
      - [0, 0]
      - [0, 0]
      - [0, 0]
      - [0, 0]
```

Оставить `weights`, `horizon_N`, `limits`, `scenario`, `history_size`, `tick_dt` как есть. Убедиться что `tick_dt: 0.02` совпадает с `plant.Ts: 0.02`.

- [ ] **Step 2: Проверить что YAML валиден и значения читаются**

Run: `python -c "from config_loader import cfg; print('A11=', cfg('mps.matrices.A')[1][1], 'Ts=', cfg('mps.plant.Ts'), 'tick_dt=', cfg('mps.tick_dt'))"`
Expected: `A11= -6.66667 Ts= 0.02 tick_dt= 0.02`

- [ ] **Step 3: Commit**

```bash
git add config.yaml
git commit -m "fix(config): mps.matrices → непрерывная каноническая форма + mps.plant"
```

---

## Task 3: `mps_runner` ZOH-дискретизирует вход

**Files:**
- Modify: `compute_node/mps_runner.py` (`_build_controller`, `closed_loop_eigenvalues`, `run_scenario_idealized`, `short_step_response`, `__main__`)
- Test: `tests/test_mps_runner.py`

- [ ] **Step 1: Перевести фикстуру и тест propagation на непрерывный контракт (падающий тест)**

В `tests/test_mps_runner.py` заменить `_default_matrices` (строки 33-55) на:

```python
# Каноническая непрерывная модель: τ_v=0.15, τ_w=0.10.
_TAU_V = 0.15
_TAU_W = 0.10
_A_CANONICAL = [
    [0.0,  1.0,         0.0,  0.0,         0.0],
    [0.0, -1.0/_TAU_V,  0.0,  0.0,         0.0],
    [0.0,  0.0,         0.0,  1.0,         0.0],
    [0.0,  0.0,         0.0, -1.0/_TAU_W,  0.0],
    [0.0, -1.0,         0.0,  0.0,         0.0],
]
_B_CANONICAL = [
    [0.0,         0.0],
    [1.0/_TAU_V,  0.0],
    [0.0,         0.0],
    [0.0,         1.0/_TAU_W],
    [0.0,         0.0],
]


def _default_matrices(*, A=None, B=None, horizon_N=10) -> MpsMatrices:
    """Каноническая НЕПРЕРЫВНАЯ модель [s, v, θ, ω, e_int]."""
    A_default = A if A is not None else [row[:] for row in _A_CANONICAL]
    B_default = B if B is not None else [row[:] for row in _B_CANONICAL]
    C = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
    D = [[0.0, 0.0] for _ in range(5)]
    return MpsMatrices(
        A=A_default, B=B_default, C=C, D=D,
        Q_diag=[10, 10, 5, 1, 1], R_diag=[1, 1],
        horizon_N=horizon_N,
        u_min=[-0.30, -2.0], u_max=[0.30, 2.0],
    )
```

Заменить `test_zero_control_propagation_matches_manual` (строки 109-139) на тест, корректный для непрерывного контракта:

```python
def test_zero_control_propagation_matches_zoh():
    """Continuous diagonal A_c = diag(-0.5,...) discretized at Ts=dt gives
    x[k+1] = exp(-0.5·dt)·x[k]. With B=0 the explicit MPC law yields u=0,
    so the trajectory follows the ZOH-discretized free response exactly."""
    decay = -0.5
    A = [[decay if i == j else 0.0 for j in range(5)] for i in range(5)]
    B = [[0.0, 0.0] for _ in range(5)]
    C = [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)]
    D = [[0.0, 0.0] for _ in range(5)]
    m = MpsMatrices(
        A=A, B=B, C=C, D=D,
        Q_diag=[0, 0, 0, 0, 0], R_diag=[1, 1],
        horizon_N=5,
        u_min=[-0.001, -0.001], u_max=[0.001, 0.001],
    )
    req = MpsScenarioRequest(distance=4.0, v_target=0.10, source='sim')
    initial = np.array([0.5, 0.0, 0.0, 0.0, 0.0])
    dt = 1.0
    res = run_scenario_idealized(m, req, dt=dt, max_steps=10, initial_state=initial)
    s_seq = [p.x[0] for p in res.telemetry]
    ad = np.exp(decay * dt)  # ZOH eigenvalue at Ts=dt
    expected = [0.5 * ad ** k for k in range(len(s_seq))]
    np.testing.assert_allclose(s_seq, expected, atol=1e-6)
    assert res.status == 'timeout'  # s decays to 0, D=4 unreachable
```

- [ ] **Step 2: Запустить — убедиться что падает**

Run: `pytest tests/test_mps_runner.py -v`
Expected: FAIL — `test_zero_control_propagation_matches_zoh` падает (текущий `_build_controller` не дискретизирует: трактует `diag(-0.5)` как дискретную, `x[k+1] = -0.5·x[k]` ≠ `exp(-0.5)·x[k]`); `test_closed_loop_eigenvalues_returns_5_5` падает (непрерывная каноническая в недискретизированном MPC даёт `|λ_closed| > 1`).

- [ ] **Step 3: Добавить дискретизацию в `mps_runner.py`**

В `compute_node/mps_runner.py` после блока импортов (после строки 38, `from pi_nodes.control.state_space_model import StateSpaceModel`) добавить:

```python
from pi_nodes.control.state_space_model import StateSpaceModel, zoh_discretize

try:
    from config_loader import cfg
except ImportError:
    cfg = lambda key, default=None: default  # type: ignore


def _mps_ts() -> float:
    """Период дискретизации/цикла МПС — единый источник правды."""
    return float(cfg('mps.plant.Ts', 0.02))
```

(удалив старую строку `from pi_nodes.control.state_space_model import StateSpaceModel`).

Заменить `_build_controller` (строки 64-82):

```python
def _build_controller(m: MpsMatrices, ts: float) -> tuple[StateSpaceModel, MPCController]:
    """Build state-space model + MPC from the supplied matrices.

    `m.A`, `m.B` — НЕПРЕРЫВНЫЕ канонические матрицы (контракт
    docs/mps/api.md). Они ZOH-дискретизируются при `ts` перед передачей
    в StateSpaceModel / MPCController, которые работают с дискретными Ad/Bd.
    Mirrors what `mps_node` does on Pi for `source="robot"`.
    """
    A, B, C, D, Q_diag, R_diag, u_min, u_max = _matrices_to_arrays(m)
    Ad, Bd = zoh_discretize(A, B, ts)
    plant = StateSpaceModel(Ad=Ad, Bd=Bd, Cd=C, Dd=D, Ts=ts)
    mpc = MPCController(
        Ad=Ad,
        Bd=Bd,
        Q=np.diag(Q_diag),
        R=np.diag(R_diag),
        N=int(m.horizon_N),
        u_min=u_min,
        u_max=u_max,
        solver='clip',
    )
    return plant, mpc
```

В `run_scenario_idealized` изменить сигнатуру `dt` на `Optional[float] = None` и резолвить из конфига. Заменить строку `dt: float = 0.02,` (строка 147) на `dt: Optional[float] = None,` и в начале тела функции (после docstring, перед `started_at = ...`, строка 178) добавить:

```python
    if dt is None:
        dt = _mps_ts()
```

Заменить вызов `_build_controller(matrices)` (строка 204) на `_build_controller(matrices, dt)`.

Заменить `closed_loop_eigenvalues` (строки 312-327):

```python
def closed_loop_eigenvalues(matrices: MpsMatrices) -> tuple[list[complex], list[complex]]:
    """Return (λ(Ad), λ(Ad − Bd·K_first)) as Python complex lists.

    `matrices.A/B` — НЕПРЕРЫВНЫЕ; ZOH-дискретизируются при mps.plant.Ts
    перед анализом собственных значений.
    """
    A, B, _, _, Q_diag, R_diag, u_min, u_max = _matrices_to_arrays(matrices)
    ts = _mps_ts()
    Ad, Bd = zoh_discretize(A, B, ts)
    eig_open = list(np.linalg.eigvals(Ad))
    try:
        mpc = MPCController(
            Ad=Ad, Bd=Bd,
            Q=np.diag(Q_diag), R=np.diag(R_diag),
            N=int(matrices.horizon_N),
            u_min=u_min, u_max=u_max,
            solver='clip',
        )
        eig_closed = list(np.linalg.eigvals(Ad - Bd @ mpc.K_first))
    except Exception:
        eig_closed = [complex('nan')] * len(eig_open)
    return eig_open, eig_closed
```

В `short_step_response` (строки 296-309) изменить `dt: float = 0.02` → `dt: Optional[float] = None` и передавать дальше как есть (`run_scenario_idealized` сам резолвит `None`).

В `__main__`-блоке (строки 334-347) заменить `A_DEFAULT`/`B_DEFAULT` на непрерывную каноническую форму:

```python
    A_DEFAULT = [
        [0.0,  1.0,             0.0,  0.0,             0.0],
        [0.0, -1.0 / 0.15,      0.0,  0.0,             0.0],
        [0.0,  0.0,             0.0,  1.0,             0.0],
        [0.0,  0.0,             0.0, -1.0 / 0.10,      0.0],
        [0.0, -1.0,             0.0,  0.0,             0.0],
    ]
    B_DEFAULT = [
        [0.0,         0.0],
        [1.0 / 0.15,  0.0],
        [0.0,         0.0],
        [0.0,         1.0 / 0.10],
        [0.0,         0.0],
    ]
```

- [ ] **Step 4: Запустить тесты — убедиться что проходят**

Run: `pytest tests/test_mps_runner.py -v`
Expected: PASS — все тесты, включая `test_zero_control_propagation_matches_zoh` и `test_closed_loop_eigenvalues_returns_5_5` (замкнутый контур теперь `|λ| < 1`).

- [ ] **Step 5: Commit**

```bash
git add compute_node/mps_runner.py tests/test_mps_runner.py
git commit -m "fix(mps): mps_runner ZOH-дискретизирует непрерывные A/B перед MPC"
```

---

## Task 4: `/validate` — нюанс маргинальной устойчивости

**Files:**
- Modify: `compute_node/dashboard/routers/mps.py` (функция `validate`, строки 203-255)
- Test: `tests/test_mps_router.py`

- [ ] **Step 1: Перевести фикстуру и добавить падающий тест**

В `tests/test_mps_router.py` заменить фикстуру `matrices_payload` (строки 43-68) на непрерывную каноническую:

```python
@pytest.fixture
def matrices_payload():
    """Каноническая НЕПРЕРЫВНАЯ модель — проходит Pydantic-валидацию."""
    tau_v, tau_w = 0.15, 0.10
    return {
        'A': [
            [0.0,  1.0,        0.0,  0.0,        0.0],
            [0.0, -1.0/tau_v,  0.0,  0.0,        0.0],
            [0.0,  0.0,        0.0,  1.0,        0.0],
            [0.0,  0.0,        0.0, -1.0/tau_w,  0.0],
            [0.0, -1.0,        0.0,  0.0,        0.0],
        ],
        'B': [
            [0.0,        0.0],
            [1.0/tau_v,  0.0],
            [0.0,        0.0],
            [0.0,        1.0/tau_w],
            [0.0,        0.0],
        ],
        'C': [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)],
        'D': [[0.0, 0.0] for _ in range(5)],
        'Q_diag': [10, 10, 5, 1, 1],
        'R_diag': [1, 1],
        'horizon_N': 10,
        'u_min': [-0.30, -2.0],
        'u_max': [0.30, 2.0],
    }
```

Добавить новый тест в раздел `# ── /validate ──`:

```python
def test_validate_canonical_plant_marginal_not_unstable(client, matrices_payload):
    """Каноническая модель: 3 полюса Ad на |λ|=1 (интеграторы s,θ,e_int).
    is_plant_stable=False (строгая асимптотика), но предупреждение —
    про маргинальную устойчивость, НЕ про неустойчивость; замкнутый
    контур устойчив."""
    r = client.post('/api/v1/mps/validate', json=matrices_payload)
    assert r.status_code == 200
    body = r.json()
    assert body['is_plant_stable'] is False
    assert body['is_closed_loop_stable'] is True
    joined = ' '.join(body['warnings'])
    assert 'маргинально устойчива' in joined
    assert 'неустойчива' not in joined
```

- [ ] **Step 2: Запустить — убедиться что падает**

Run: `pytest tests/test_mps_router.py::test_validate_canonical_plant_marginal_not_unstable -v`
Expected: FAIL — текущий код добавляет warning `'Открытая система НЕ устойчива (|λ(A)| ≥ 1)'`, в котором есть подстрока `неустойчива`... фактически проверка `'неустойчива' not in joined` падает (старый текст содержит «НЕ устойчива»), и `'маргинально устойчива'` отсутствует.

- [ ] **Step 3: Реализовать нюансированную проверку**

В `compute_node/dashboard/routers/mps.py`, функция `validate`. Заменить строки 232-235:

```python
    is_plant_stable = bool(eig_open) and all(abs(z) < 1.0 - 1e-9 for z in eig_open)
    is_closed_stable = bool(eig_closed) and all(
        abs(z) < 1.0 - 1e-9 for z in eig_closed
    )
```

на:

```python
    _STAB_TOL = 1e-6
    is_plant_stable = bool(eig_open) and all(abs(z) < 1.0 - _STAB_TOL for z in eig_open)
    is_closed_stable = bool(eig_closed) and all(
        abs(z) < 1.0 - _STAB_TOL for z in eig_closed
    )
    plant_has_unstable = bool(eig_open) and any(
        abs(z) > 1.0 + _STAB_TOL for z in eig_open
    )
```

Заменить строки 243-246:

```python
    if not is_plant_stable:
        warnings.append('Открытая система НЕ устойчива (|λ(A)| ≥ 1)')
    if not is_closed_stable:
        warnings.append('Замкнутая система НЕ устойчива (|λ(A−B·K)| ≥ 1)')
```

на:

```python
    if plant_has_unstable:
        warnings.append('Открытая система неустойчива — есть |λ(Ad)| > 1')
    elif not is_plant_stable:
        warnings.append(
            'Открытая система маргинально устойчива: полюса-интеграторы '
            'на |λ|=1 (s, θ, e_int) — норма для канонической модели'
        )
    if not is_closed_stable:
        warnings.append('Замкнутая система НЕ устойчива (|λ(Ad−Bd·K)| ≥ 1)')
```

- [ ] **Step 4: Запустить тесты — убедиться что проходят**

Run: `pytest tests/test_mps_router.py -v`
Expected: PASS — все тесты, включая `test_validate_canonical_plant_marginal_not_unstable` и `test_validate_with_explicit_matrices` (`is_closed_loop_stable is True`).

- [ ] **Step 5: Commit**

```bash
git add compute_node/dashboard/routers/mps.py tests/test_mps_router.py
git commit -m "fix(mps): /validate различает маргинальную устойчивость и неустойчивость"
```

---

## Task 5: `mps_node._on_matrices_set` ZOH-дискретизирует

**Files:**
- Modify: `pi_nodes/nodes/mps_node.py` (импорт строка 47, `__init__` ~строка 102, `_on_matrices_set` строки 149-192)
- Test: `tests/test_mps_node.py`

- [ ] **Step 1: Перевести фикстуру и добавить падающий тест**

В `tests/test_mps_node.py` заменить `_good_matrices` (строки 58-82):

```python
def _good_matrices() -> dict:
    """Каноническая НЕПРЕРЫВНАЯ модель [s, v, θ, ω, e_int]."""
    tau_v, tau_w = 0.15, 0.10
    return {
        'A': [
            [0.0,  1.0,        0.0,  0.0,        0.0],
            [0.0, -1.0/tau_v,  0.0,  0.0,        0.0],
            [0.0,  0.0,        0.0,  1.0,        0.0],
            [0.0,  0.0,        0.0, -1.0/tau_w,  0.0],
            [0.0, -1.0,        0.0,  0.0,        0.0],
        ],
        'B': [
            [0.0,        0.0],
            [1.0/tau_v,  0.0],
            [0.0,        0.0],
            [0.0,        1.0/tau_w],
            [0.0,        0.0],
        ],
        'C': [[1.0 if i == j else 0.0 for j in range(5)] for i in range(5)],
        'D': [[0.0, 0.0] for _ in range(5)],
        'Q_diag': [10, 10, 5, 1, 1],
        'R_diag': [1, 1],
        'horizon_N': 10,
        'u_min': [-0.30, -2.0],
        'u_max': [0.30, 2.0],
        'schema_version': '1.0',
    }
```

Добавить тест в раздел `# ── matrices/set ──`:

```python
def test_matrices_set_discretizes_before_rebuild(mps_node):
    """A/B приходят непрерывными; mpc/plant должны получить ZOH-дискретные
    Ad/Bd, а не сырые непрерывные значения."""
    import numpy as np
    mps_node._on_matrices_set('mps/matrices/set', _good_matrices())
    # Непрерывная A_c[1][1] = -1/0.15 ≈ -6.667; дискретная Ad[1][1] =
    # exp(-6.667·0.02) ≈ 0.8752 — должна быть в (0, 1).
    assert 0.0 < mps_node._mpc.Ad[1][1] < 1.0
    assert 0.0 < mps_node._plant.Ad[1][1] < 1.0
    np.testing.assert_allclose(
        mps_node._mpc.Ad[1][1], np.exp(-1.0 / 0.15 * 0.02), atol=1e-9
    )
```

- [ ] **Step 2: Запустить — убедиться что падает**

Run: `pytest tests/test_mps_node.py::test_matrices_set_discretizes_before_rebuild -v`
Expected: FAIL — текущий `_on_matrices_set` передаёт сырые непрерывные A/B; `_mpc.Ad[1][1]` ≈ `-6.667`, не в `(0, 1)`. (Либо `rebuild` упадёт на проверке `R_diag>0` — нет, R валиден; либо просто ассерт диапазона падает.)

- [ ] **Step 3: Добавить дискретизацию в `mps_node.py`**

Строка 47 — заменить импорт:

```python
from pi_nodes.control.state_space_model import StateSpaceModel, zoh_discretize
```

В `__init__`, после строки `self._tick_dt = float(self._cfg('mps.tick_dt', 0.02))` (строка 102) добавить:

```python
        self._mps_ts = float(self._cfg('mps.plant.Ts', 0.02))
```

В `_on_matrices_set`, после блока парсинга payload (после строки 171, перед `with self._lock:`) добавить:

```python
        # A/B приходят НЕПРЕРЫВНЫМИ (контракт docs/mps/api.md) —
        # ZOH-дискретизируем перед передачей в дискретные plant/MPC.
        try:
            Ad, Bd = zoh_discretize(A, B, self._mps_ts)
        except Exception as exc:
            self._publish_error('precondition', f'matrices/set: ZOH failed: {exc}')
            return
```

Заменить тело `with self._lock:` (строки 173-182) — `reload`/`rebuild` теперь получают `Ad/Bd`:

```python
        with self._lock:
            try:
                self._plant.reload(Ad=Ad, Bd=Bd, Cd=C, Dd=D)
                self._mpc.rebuild(
                    Ad=Ad, Bd=Bd, Q_diag=Q, R_diag=R, N=N,
                    u_min=u_min, u_max=u_max,
                )
            except Exception as exc:
                self._publish_error('precondition', f'matrices apply failed: {exc}')
                return
```

Заменить лог-строку (строки 190-192):

```python
        self.log_info('mps: matrices applied (N=%d, Ts=%.3f, λ_open(Ad)=%s)',
                      N, self._mps_ts,
                      [f'{abs(z):.3f}' for z in np.linalg.eigvals(Ad)])
```

- [ ] **Step 4: Запустить тесты — убедиться что проходят**

Run: `pytest tests/test_mps_node.py -v`
Expected: PASS — все тесты, включая `test_matrices_set_discretizes_before_rebuild` и `test_matrices_set_acks_with_applied`.

- [ ] **Step 5: Commit**

```bash
git add pi_nodes/nodes/mps_node.py tests/test_mps_node.py
git commit -m "fix(mps): mps_node ZOH-дискретизирует A/B в _on_matrices_set"
```

---

## Task 6: Контракт — docstrings схемы + `docs/mps/api.md` + пометка в спеке

**Files:**
- Modify: `compute_node/dashboard/schemas/mps.py` (класс `MpsMatrices`, строки 44-68)
- Modify: `docs/mps/api.md` (§1, §2, §3.2)
- Modify: `docs/superpowers/specs/2026-05-05-mps-state-space-design.md` (§5.1 / §7.1)

- [ ] **Step 1: Обновить docstrings полей `A`/`B` в схеме**

В `compute_node/dashboard/schemas/mps.py` заменить docstring класса `MpsMatrices` (строки 45-52) — добавить абзац про непрерывность, и Field-описания `A`/`B` (строки 53-60):

```python
class MpsMatrices(BaseModel):
    """Матрицы пространства состояний + параметры регулятора.

    Порядок состояний:  x = [s, v, θ, ω, e_int]
    Порядок управлений: u = [v_cmd, ω_cmd]

    A, B — НЕПРЕРЫВНЫЕ матрицы A_c, B_c канонической ОДУ-модели. Бэкенд
    (mps_runner / mps_node) ZOH-дискретизирует их при mps.plant.Ts перед
    передачей в дискретный MPCController. См. docs/mps/api.md §2.

    C, D хранятся для документации курсовой; в `step()` контроллера не
    используются. В UI — только для y(t) визуализации (`StateSpaceModel.output`).
    """
    A: list[list[float]] = Field(
        ...,
        description='5×5 — непрерывная матрица состояния A_c (бэкенд ZOH-дискретизирует)'
    )
    B: list[list[float]] = Field(
        ...,
        description='5×2 — непрерывная матрица управления B_c (бэкенд ZOH-дискретизирует)'
    )
```

- [ ] **Step 2: Обновить `docs/mps/api.md`**

В `docs/mps/api.md`, §1 после строки `Размерности фиксированы: ...` (строка 28) добавить:

```markdown
**Контракт A/B (с 2026-05-14):** `MpsMatrices.A` и `B` — **непрерывные**
матрицы `A_c/B_c` канонической ОДУ-модели. Бэкенд ZOH-дискретизирует их
при `mps.plant.Ts` (= 0.02 с) перед передачей в дискретный MPC. Открытый
контур `λ(Ad)` имеет 3 полюса на `|λ|=1` (интеграторы `s, θ, e_int`) —
это норма.
```

В §2 в строке `- \`MpsMatrices\` — A/B/C/D ...` заменить на:

```markdown
- `MpsMatrices` — A/B (непрерывные A_c/B_c) + C/D + Q_diag/R_diag + horizon_N + u_min/u_max + schema_version.
  Валидация: shapes 5×5 / 5×2; `Q_diag ≥ 0`, `R_diag > 0`; `u_min < u_max` поэлементно.
```

В §3.2 заменить строку `Считает λ(Ad), λ(Ad − Bd·K_first), short step-response ...` на:

```markdown
ZOH-дискретизирует непрерывные A/B при `mps.plant.Ts`, затем считает
λ(Ad), λ(Ad − Bd·K_first), short step-response (2 сек) в идеальном
симуляторе. Никаких побочных эффектов.
```

- [ ] **Step 3: Пометка в главной спеке**

В `docs/superpowers/specs/2026-05-05-mps-state-space-design.md`, в §5.1 после блока `x = [s, v, θ, ω, e_int]ᵀ ∈ ℝ⁵ ...` добавить строку:

```markdown
> **Обновление 2026-05-14:** `MpsMatrices.A/B` несут НЕПРЕРЫВНЫЕ матрицы
> `A_c/B_c`; бэкенд ZOH-дискретизирует. См.
> `2026-05-14-mps-continuous-discretization-design.md`.
```

- [ ] **Step 4: Проверить что схема импортируется**

Run: `python -c "from compute_node.dashboard.schemas.mps import MpsMatrices; print(MpsMatrices.model_fields['A'].description)"`
Expected: `5×5 — непрерывная матрица состояния A_c (бэкенд ZOH-дискретизирует)`

- [ ] **Step 5: Commit**

```bash
git add compute_node/dashboard/schemas/mps.py docs/mps/api.md docs/superpowers/specs/2026-05-05-mps-state-space-design.md
git commit -m "docs(mps): контракт A/B — непрерывные матрицы, бэкенд ZOH-дискретизирует"
```

---

## Task 7: Sim-верификация + подбор Q/R/N при необходимости

**Files:** нет правок кода по умолчанию (только если метрики не проходят — `config.yaml mps.weights`)

- [ ] **Step 1: Полный прогон Python-тестов МПС**

Run: `pytest tests/test_zoh_discretization.py tests/test_mps_runner.py tests/test_mps_router.py tests/test_mps_node.py tests/test_mpc_controller.py tests/test_state_space_extended.py -q`
Expected: PASS — все тесты зелёные.

- [ ] **Step 2: Перезапустить дашборд**

Дашборд на `:5000` (запущен PID 2768) держит старый код в памяти. Остановить и перезапустить тем же способом, которым он был запущен (`./samurai.sh compute` или прямой uvicorn — если неизвестно, спросить пользователя; проверить процессы: `netstat -ano | grep :5000`).
После рестарта:

Run: `curl -s http://localhost:5000/api/v1/mps && echo`
Expected: `{"ok":true}`

- [ ] **Step 3: Проверить `/validate` на дефолтных (config) матрицах**

Run:
```bash
curl -s http://localhost:5000/api/v1/mps/matrices > /dev/null && \
curl -s -X POST http://localhost:5000/api/v1/mps/validate -H "Content-Type: application/json" -d 'null' | python -c "import sys,json; b=json.load(sys.stdin); print('plant_stable=',b['is_plant_stable'],'closed_stable=',b['is_closed_loop_stable']); print('warnings=',b['warnings'])"
```
Expected: `closed_stable= True`; `plant_stable= False` с warning «маргинально устойчива» (не «неустойчива»).

- [ ] **Step 4: Прогон сценария «D=2м вперёд» в sim**

Run:
```bash
curl -s -X POST http://localhost:5000/api/v1/mps/scenario/run -H "Content-Type: application/json" -d '{"distance":2.0,"v_target":0.15,"source":"sim"}' | python -c "import sys,json; r=json.load(sys.stdin)['result']; m=r.get('metrics') or {}; print('status=',r['status']); print('ss_error=',m.get('ss_error'),'overshoot=',m.get('overshoot'),'settling=',m.get('settling_time'),'peak_v=',m.get('peak_v'))"
```
Expected: `status= reached`; `ss_error < 0.05`; `overshoot < 0.2` (≤10%·D); `settling` — конечное число; `peak_v > 0`.

- [ ] **Step 5: Подбор Q/R/N — ТОЛЬКО если Step 4 не прошёл**

Если `status != 'reached'` или метрики хуже порогов: править `config.yaml mps.weights.Q_diag` / `R_diag` / `mps.horizon_N`. Ориентиры (порядок состояний `[s, v, θ, ω, e_int]`):
- слабый разгон / не доезжает → поднять `Q_diag[0]` (вес ошибки `s`) и/или `Q_diag[4]` (интеграл скорости);
- перерегулирование / рывки → поднять `R_diag` (штраф управления) или `Q_diag[1]` (вес `v`);
- близоруко (рывки на финише) → поднять `horizon_N` (10 → 15..20).
После каждой правки — перезапустить дашборд (Step 2) и повторить Step 4. Зафиксировать рабочие значения. Commit (если правил):

```bash
git add config.yaml
git commit -m "fix(mps): подбор Q/R/N — устойчивый forward-drive в sim"
```

Если Step 4 прошёл с дефолтами — этот шаг пропустить, коммита нет.

- [ ] **Step 6: Зафиксировать результат верификации**

Записать в описание задачи/PR: вывод Step 3 и Step 4 (status + метрики) — доказательство, что замкнутый контур устойчив и сценарий доезжает.

---

## Task 8: MATLAB `build_canonical_mps.m` (новый)

> **Перед началом:** проверить доступность MATLAB/Octave: `matlab -batch "ver" 2>/dev/null || octave --version 2>/dev/null || echo "NO MATLAB/OCTAVE"`. Если нет — Tasks 8-11 пишут код, верификация (`matlab -batch ...`) откладывается; отметить в PR как deferred (аналогично Task 13).

**Files:**
- Create: `matlab/build_canonical_mps.m`
- Test: `matlab/test_build_canonical_mps.m`

- [ ] **Step 1: Написать падающий тест**

Создать `matlab/test_build_canonical_mps.m`:

```matlab
function test_build_canonical_mps()
% Тест структуры канонической непрерывной модели [s, v, θ, ω, e_int].
% Run:  matlab -batch "addpath('matlab'); test_build_canonical_mps"
  here = fileparts(mfilename('fullpath'));
  addpath(here);

  tau_v = 0.15; tau_w = 0.10;
  [A, B, C, D] = build_canonical_mps(tau_v, tau_w);

  % -- shapes --
  assert(isequal(size(A), [5 5]), 'A must be 5x5');
  assert(isequal(size(B), [5 2]), 'B must be 5x2');
  assert(isequal(size(C), [5 5]), 'C must be 5x5');
  assert(isequal(size(D), [5 2]), 'D must be 5x2');

  % -- canonical pattern --
  assert(A(1,2) == 1,            'A[0][1] must be 1 (ṡ = v)');
  assert(abs(A(2,2) + 1/tau_v) < 1e-12, 'A[1][1] must be -1/tau_v');
  assert(A(3,4) == 1,            'A[2][3] must be 1 (θ̇ = ω)');
  assert(abs(A(4,4) + 1/tau_w) < 1e-12, 'A[3][3] must be -1/tau_w');
  assert(A(5,2) == -1,           'A[4][1] must be -1 (ė_int = -v)');
  assert(abs(B(2,1) - 1/tau_v) < 1e-12, 'B[1][0] must be 1/tau_v');
  assert(abs(B(4,2) - 1/tau_w) < 1e-12, 'B[3][1] must be 1/tau_w');
  assert(isequal(C, eye(5)), 'C must be I5');
  assert(isequal(D, zeros(5,2)), 'D must be 0');

  % -- all other A entries zero --
  mask = true(5,5);
  mask(1,2) = false; mask(2,2) = false; mask(3,4) = false;
  mask(4,4) = false; mask(5,2) = false;
  assert(all(A(mask) == 0), 'non-pattern A entries must be 0');

  % -- controllable --
  assert(rank(ctrb(A, B)) == 5, 'canonical (A,B) must be controllable');

  fprintf('PASS: build_canonical_mps — структура канонической модели верна\n');
end
```

- [ ] **Step 2: Запустить — убедиться что падает**

Run: `matlab -batch "addpath('matlab'); test_build_canonical_mps"`
Expected: FAIL — `Undefined function 'build_canonical_mps'`

- [ ] **Step 3: Реализовать `build_canonical_mps.m`**

Создать `matlab/build_canonical_mps.m`:

```matlab
function [A, B, C, D] = build_canonical_mps(tau_v, tau_w)
% BUILD_CANONICAL_MPS — непрерывная каноническая модель модуля МПС.
%
%   Вектор состояния (n=5):  x = [s, v, theta, omega, e_int]'
%   Вектор управления (r=2): u = [v_cmd, omega_cmd]'
%
%   Каноническая ОДУ-модель (после линеаризации, см. спеку МПС §4.2):
%       s_dot     = v
%       v_dot     = -(1/tau_v)·v     + (1/tau_v)·u_v
%       theta_dot = omega
%       omega_dot = -(1/tau_w)·omega + (1/tau_w)·u_omega
%       e_int_dot = v_target - v        ← v_target = reference, в A/B не входит
%
%   В ОТЛИЧИЕ от linearize_samurai.m здесь НЕТ якобианов: модель
%   аналитическая, строится напрямую из постоянных времени моторов.
%   Возвращает НЕПРЕРЫВНЫЕ A, B (контракт docs/mps/api.md — бэкенд
%   ZOH-дискретизирует их сам).

  A = [ 0,  1,        0,  0,        0;
        0, -1/tau_v,  0,  0,        0;
        0,  0,        0,  1,        0;
        0,  0,        0, -1/tau_w,  0;
        0, -1,        0,  0,        0 ];

  B = [ 0,        0;
        1/tau_v,  0;
        0,        0;
        0,        1/tau_w;
        0,        0 ];

  C = eye(5);
  D = zeros(5, 2);

  % ── Проверка управляемости (учебник §3.0) ────────────────────
  if rank(ctrb(A, B)) < 5
    error('build_canonical_mps:notControllable', ...
          'Каноническая (A,B) не управляема — проверьте tau_v, tau_w ≠ 0');
  end
end
```

- [ ] **Step 4: Запустить тест — убедиться что проходит**

Run: `matlab -batch "addpath('matlab'); test_build_canonical_mps"`
Expected: PASS — `PASS: build_canonical_mps — структура канонической модели верна`

- [ ] **Step 5: Commit**

```bash
git add matlab/build_canonical_mps.m matlab/test_build_canonical_mps.m
git commit -m "feat(matlab): build_canonical_mps — непрерывная каноническая модель МПС"
```

---

## Task 9: MATLAB `samurai_params.m` — под-структура `p.mps`

**Files:**
- Modify: `matlab/samurai_params.m` (перед `end`, после строки 89)

- [ ] **Step 1: Добавить блок `p.mps`**

В `matlab/samurai_params.m` перед закрывающим `end` (строка 90) добавить:

```matlab
  % ── МПС — параметры канонической модели [s, v, θ, ω, e_int] ────
  % Отдельная подсистема: модуль курсовой «проехать D метров вперёд».
  % НЕ путать с legacy-моделью [px, py, theta, v, omega] выше.
  % tau_v / tau_w — те же физические постоянные моторов (тот же робот).
  p.mps.tau_v = p.tau_v;          % с — постоянная linear-мотора
  p.mps.tau_w = p.tau_w;          % с — постоянная angular-мотора
  p.mps.Ts    = 0.02;             % с — период дискретизации МПС (50 Гц)

  % Веса критерия J для ПОРЯДКА [s, v, θ, ω, e_int] — НЕ совпадает
  % с порядком legacy-модели, хотя численно те же значения.
  p.mps.Q     = diag([10, 10, 5, 1, 1]);
  p.mps.R     = diag([1, 1]);
  p.mps.N     = 10;               % горизонт MPC
  p.mps.u_min = [-0.30; -2.0];    % [м/с; рад/с]
  p.mps.u_max = [+0.30; +2.0];
```

- [ ] **Step 2: Проверить что параметры загружаются**

Run: `matlab -batch "addpath('matlab'); p = samurai_params(); fprintf('mps.Ts=%g mps.N=%d tau_v=%g\n', p.mps.Ts, p.mps.N, p.mps.tau_v)"`
Expected: `mps.Ts=0.02 mps.N=10 tau_v=0.15`

- [ ] **Step 3: Commit**

```bash
git add matlab/samurai_params.m
git commit -m "feat(matlab): samurai_params — под-структура p.mps для канонической модели"
```

---

## Task 10: MATLAB `main.m` — ветка синтеза МПС

**Files:**
- Modify: `matlab/main.m` (между текущим шагом 10 и финальным баннером, после строки 122)

- [ ] **Step 1: Заменить шаг экспорта на ветку синтеза МПС + общий экспорт**

В `matlab/main.m` **заменить строки 118-122** целиком (текущий блок `%% 10. Экспорт в config.yaml`: комментарии + `fprintf` + `yaml_path = ...` + единственный вызов `export_to_yaml(...)`) на следующее. Строка 123 (пустая) и далее (`fprintf('\n══...` финальный баннер) — НЕ трогать.

```matlab
%% 10. Путь к config.yaml (экспорт — в шаге 12, после синтеза МПС)
yaml_path = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'config.yaml');

%% 11. МПС — каноническая модель пространства состояний [s,v,θ,ω,e_int]
% Отдельная подсистема (модуль курсовой). Аналитическая непрерывная
% модель из постоянных времени моторов → ZOH → MPC. A/B в config.yaml
% mps: пишутся НЕПРЕРЫВНЫМИ (контракт docs/mps/api.md).
fprintf('11. МПС — синтез канонической модели:\n');
[A_mps, B_mps, C_mps, D_mps] = build_canonical_mps(p.mps.tau_v, p.mps.tau_w);
fprintf('   Непрерывная A_mps (5×5):\n'); disp(A_mps);
[Ad_mps, Bd_mps] = discretize_samurai(A_mps, B_mps, p.mps.Ts);
[~, Pf_mps] = design_lqr(Ad_mps, Bd_mps, p.mps.Q, p.mps.R);
mpc_mps = design_mpc(Ad_mps, Bd_mps, p.mps.Q, p.mps.R, Pf_mps, p.mps.N);
% Переходный процесс «проехать 2 м вперёд»: x0 = ошибка позиции −2 м по s.
simulate_closed_loop(Ad_mps, Bd_mps, mpc_mps.K_first, ...
                     [-2; 0; 0; 0; 0], round(3 / p.mps.Ts), ...
                     p.mps.u_min, p.mps.u_max, ...
                     'МПС — переходный процесс (s: −2 м → 0)');
fprintf('\n');

%% 12. Экспорт обоих блоков (control: + mps:) в config.yaml
fprintf('12. Экспорт config.yaml (блоки control + mps)...\n');
mps_export = struct('A_c', A_mps, 'B_c', B_mps, 'C', C_mps, 'D', D_mps, ...
                    'tau_v', p.mps.tau_v, 'tau_w', p.mps.tau_w, ...
                    'Ts', p.mps.Ts, 'Q', p.mps.Q, 'R', p.mps.R, ...
                    'N', p.mps.N, 'u_min', p.mps.u_min, 'u_max', p.mps.u_max);
export_to_yaml(Ad, Bd, K_lqr, mpc.K_first, L_obs, P_inf, p, yaml_path, mps_export);
```

Итог: `export_to_yaml` вызывается ровно один раз (шаг 12, 9 аргументов) — пишет ОБА блока `control:` и `mps:`. Прежний 8-аргументный вызов удалён.

- [ ] **Step 2: Прогнать `main.m` целиком**

Run: `matlab -batch "cd matlab; main"`
Expected: выполняется без ошибок, в конце лог `export_to_yaml: блоки control + mps записаны`; в `config.yaml` появляются маркеры `# === mps: section auto-generated`.
(Этот шаг проходит только после Task 11 — порядок: реализовать Task 11, затем вернуться и прогнать Step 2 здесь. Если MATLAB недоступен — deferred.)

- [ ] **Step 3: Commit**

```bash
git add matlab/main.m
git commit -m "feat(matlab): main.m — ветка синтеза канонической модели МПС"
```

---

## Task 11: MATLAB `export_to_yaml.m` — запись блока `mps:`

**Files:**
- Modify: `matlab/export_to_yaml.m` (сигнатура + новая логика записи `mps:`)
- Test: `matlab/test_export_to_yaml.m`

- [ ] **Step 1: Расширить тест — проверка блока `mps:`**

В `matlab/test_export_to_yaml.m` после строки 25 (`K_lqr = ones(2, 5); ...`) добавить структуру `mps_export`:

```matlab
  mps_export = struct('A_c', [0 1 0 0 0; 0 -6.6667 0 0 0; 0 0 0 1 0; ...
                              0 0 0 -10 0; 0 -1 0 0 0], ...
                      'B_c', [0 0; 6.6667 0; 0 0; 0 10; 0 0], ...
                      'C', eye(5), 'D', zeros(5,2), ...
                      'tau_v', 0.15, 'tau_w', 0.10, 'Ts', 0.02, ...
                      'Q', diag([10 10 5 1 1]), 'R', diag([1 1]), ...
                      'N', 10, 'u_min', [-0.3; -2], 'u_max', [0.3; 2]);
```

Заменить три вызова `export_to_yaml(...)` (строки 37, 39, 41) на вариант с 9-м аргументом:

```matlab
  export_to_yaml(Ad, Bd, K_lqr, K_mpc, L, Pf, p, tmp, mps_export);
  after1 = fileread(tmp, 'Encoding', 'UTF-8');
  export_to_yaml(Ad, Bd, K_lqr, K_mpc, L, Pf, p, tmp, mps_export);
  after2 = fileread(tmp, 'Encoding', 'UTF-8');
  export_to_yaml(Ad, Bd, K_lqr, K_mpc, L, Pf, p, tmp, mps_export);
  after3 = fileread(tmp, 'Encoding', 'UTF-8');
```

После блока проверок `control:`-маркеров (после строки 57) добавить проверки `mps:`:

```matlab
  % -- exactly one of each mps: structural marker -------------------------
  n_mps_hdr = numel(strfind(after3, ...
    '# === mps: section auto-generated by matlab/main.m ==='));
  n_mps_end = numel(strfind(after3, ...
    '# === end of auto-generated mps: block ==='));
  n_mps_key = numel(regexp(after3, '(^|\n)mps:', 'start'));
  assert(n_mps_hdr == 1, sprintf('expected 1 mps header, got %d', n_mps_hdr));
  assert(n_mps_end == 1, sprintf('expected 1 mps end marker, got %d', n_mps_end));
  assert(n_mps_key == 1, sprintf('expected 1 mps: key, got %d', n_mps_key));
  % mps.matrices.A пишется НЕПРЕРЫВНОЙ — должна содержать -6.6667
  assert(~isempty(strfind(after3, '-6.6667')), ...
    'mps.matrices.A must contain continuous value -6.6667');
```

- [ ] **Step 2: Запустить — убедиться что падает**

Run: `matlab -batch "addpath('matlab'); test_export_to_yaml"`
Expected: FAIL — `export_to_yaml` принимает 8 аргументов, 9-й (`mps_export`) игнорируется → блок `mps:` не пишется → `n_mps_hdr == 0`.

- [ ] **Step 3: Расширить `export_to_yaml.m`**

В `matlab/export_to_yaml.m` изменить сигнатуру (строка 1):

```matlab
function export_to_yaml(Ad, Bd, K_lqr, K_mpc, L, Pf, p, yaml_path, mps)
```

В шапке-комментарии после строки описания `yaml_path` (строка 34) добавить:

```matlab
%     mps          — (опц.) struct канонической модели МПС. Если задан,
%                    функция дополнительно пишет/заменяет блок `mps:`.
%                    Поля: A_c, B_c, C, D (НЕПРЕРЫВНЫЕ), tau_v, tau_w, Ts,
%                    Q, R, N, u_min, u_max.
%
%   АСИММЕТРИЯ КОНТРАКТА:
%     control.matrices.A/B — ДИСКРЕТНЫЕ Ad/Bd (legacy-модель).
%     mps.matrices.A/B     — НЕПРЕРЫВНЫЕ A_c/B_c (бэкенд ZOH-дискретизирует).
```

После записи `control:`-блока (после строки 167 `fprintf('export_to_yaml: блок control ...')`) и перед `end` функции добавить вызов записи `mps:`:

```matlab

  % ── Блок mps: (если передан) ──────────────────────────────────
  if nargin >= 9 && ~isempty(mps)
    write_mps_block(yaml_path, mps);
    fprintf('export_to_yaml: блоки control + mps записаны в %s\n', yaml_path);
  end
end

% ── Запись/замена блока mps: ─────────────────────────────────────
function write_mps_block(yaml_path, mps)
% Идемпотентная замена блока mps: между маркерами (тот же приём, что
% для control:). mps.matrices.A/B пишутся НЕПРЕРЫВНЫМИ.
  M = {};
  M{end+1} = '# === mps: section auto-generated by matlab/main.m ===';
  M{end+1} = '# A/B — НЕПРЕРЫВНЫЕ (контракт docs/mps/api.md); бэкенд ZOH-дискретизирует.';
  M{end+1} = 'mps:';
  M{end+1} = '  enabled: true';
  M{end+1} = '';
  M{end+1} = '  plant:';
  M{end+1} = sprintf('    tau_v: %.4f', mps.tau_v);
  M{end+1} = sprintf('    tau_w: %.4f', mps.tau_w);
  M{end+1} = sprintf('    Ts:    %.4f', mps.Ts);
  M{end+1} = '';
  M{end+1} = '  matrices:';
  M{end+1} = sprintf('    A: %s', mat2yaml_2d(mps.A_c));
  M{end+1} = sprintf('    B: %s', mat2yaml_2d(mps.B_c));
  M{end+1} = sprintf('    C: %s', mat2yaml_2d(mps.C));
  M{end+1} = sprintf('    D: %s', mat2yaml_2d(mps.D));
  M{end+1} = '';
  M{end+1} = '  weights:';
  M{end+1} = sprintf('    Q_diag: %s', mat2yaml(diag(mps.Q)'));
  M{end+1} = sprintf('    R_diag: %s', mat2yaml(diag(mps.R)'));
  M{end+1} = '';
  M{end+1} = sprintf('  horizon_N: %d', mps.N);
  M{end+1} = '';
  M{end+1} = '  limits:';
  M{end+1} = sprintf('    u_min: %s', mat2yaml(mps.u_min'));
  M{end+1} = sprintf('    u_max: %s', mat2yaml(mps.u_max'));
  M{end+1} = '';
  M{end+1} = '  scenario:';
  M{end+1} = '    distance_max: 5.0';
  M{end+1} = '    v_target_max: 0.30';
  M{end+1} = '    omega_max_in_forward: 0.5';
  M{end+1} = '    default_distance: 2.0';
  M{end+1} = '    default_v_target: 0.15';
  M{end+1} = '';
  M{end+1} = '  history_size: 20';
  M{end+1} = sprintf('  tick_dt: %.4f', mps.Ts);
  M{end+1} = '# === end of auto-generated mps: block ===';

  fid = fopen(yaml_path, 'r', 'n', 'UTF-8');
  text = fread(fid, '*char')';
  fclose(fid);
  lines = strsplit(text, '\n', 'CollapseDelimiters', false);
  if ~isempty(lines) && isempty(lines{end})
    lines(end) = [];
  end

  % Поиск блока mps: — start: строка 'mps:' без отступа (с walk-back на
  % строку-маркер); end: следующий нулевой отступ или end-маркер.
  start_idx = 0; end_idx = 0;
  for i = 1:length(lines)
    l = lines{i};
    if start_idx == 0 && length(l) >= 4 && strcmp(l(1:4), 'mps:')
      start_idx = i;
      hdr = start_idx;
      while hdr > 1 && ~isempty(lines{hdr-1}) && lines{hdr-1}(1) == '#'
        hdr = hdr - 1;
        if ~isempty(strfind(lines{hdr}, 'mps: section auto-generated'))
          break;
        end
      end
      if ~isempty(strfind(lines{hdr}, 'mps: section auto-generated'))
        start_idx = hdr;
      end
      continue;
    end
    if start_idx > 0 && end_idx == 0
      if ~isempty(l) && l(1) ~= ' ' && l(1) ~= '#' && l(1) ~= 9
        end_idx = i - 1;
        break;
      end
      if ~isempty(strfind(l, 'end of auto-generated mps'))
        end_idx = i;
        break;
      end
    end
  end
  if start_idx > 0 && end_idx == 0
    end_idx = length(lines);
  end

  if start_idx > 0
    new_lines = [lines(1:start_idx-1), M, lines(end_idx+1:end)];
  else
    if ~isempty(lines) && ~isempty(lines{end})
      new_lines = [lines, {''}, M];
    else
      new_lines = [lines, M];
    end
  end

  fid = fopen(yaml_path, 'w', 'n', 'UTF-8');
  for i = 1:length(new_lines)
    fprintf(fid, '%s\n', new_lines{i});
  end
  fclose(fid);
end
```

(Локальные хелперы `mat2yaml` и `mat2yaml_2d` уже определены в файле — `write_mps_block` использует их как nested/local-функции того же файла.)

- [ ] **Step 4: Запустить тест — убедиться что проходит**

Run: `matlab -batch "addpath('matlab'); test_export_to_yaml"`
Expected: PASS — `PASS: export_to_yaml idempotent ...` + проверки `mps:`-маркеров и непрерывного `-6.6667`.

- [ ] **Step 5: Прогнать полный `main.m` и проверить `config.yaml`**

Run: `matlab -batch "cd matlab; main"` (это Step 2 из Task 10)
Expected: без ошибок; в конце — `export_to_yaml: блоки control + mps записаны`.

Примечание: `main.m` перезаписывает ОБА блока — `control:` (из `samurai_params.m`, как и раньше) и `mps:`. Изменение `control:` ожидаемо и должно быть минимальным (только переформатирование float'ов). Если `control:` изменился существенно — значит legacy-параметры в `samurai_params.m` разошлись с текущим `config.yaml`; это отдельный pre-existing вопрос, в рамках этой задачи `control:` не трогаем по смыслу.

Run: `python -c "from config_loader import cfg; print('mps A11=', cfg('mps.matrices.A')[1][1], 'control A11=', cfg('control.matrices.A')[0][0])"`
Expected: `mps A11= -6.66667 control A11= 1` (mps — непрерывная, control — дискретная; асимметрия соблюдена).

- [ ] **Step 6: Commit**

```bash
git add matlab/export_to_yaml.m matlab/test_export_to_yaml.m config.yaml
git commit -m "feat(matlab): export_to_yaml пишет блок mps: (непрерывные A/B)"
```

---

## Task 12: Frontend `OdeCard.tsx` — подпись Ts

**Files:**
- Modify: `compute_node/frontend/src/components/mps/OdeCard.tsx:128`
- Возможно Modify: `compute_node/frontend/src/components/mps/OdeCard.test.tsx`

- [ ] **Step 1: Найти все вхождения подписи Ts**

Run: `grep -rn "50 мс\|Дискретизация ZOH" compute_node/frontend/src/`
Expected: как минимум `OdeCard.tsx:128`. Если в `OdeCard.test.tsx` есть ассерт на «50 мс» — он попадёт в выдачу.

- [ ] **Step 2: Исправить подпись в `OdeCard.tsx`**

В `compute_node/frontend/src/components/mps/OdeCard.tsx` строка 128 — заменить:

```tsx
          <div className="text-muted-foreground">Дискретизация ZOH (Ts = 20 мс):</div>
```

(было «Ts = 50 мс»; реальный период МПС-цикла — `mps.plant.Ts = 0.02 с` = 20 мс).

- [ ] **Step 3: Обновить тест, если он ассертит «50 мс»**

Если Step 1 показал вхождение в `OdeCard.test.tsx` — заменить там `50 мс` → `20 мс`. Если вхождений в тесте нет — шаг пропустить.

- [ ] **Step 4: Прогнать vitest для OdeCard**

Run: `cd compute_node/frontend && npm run test -- OdeCard`
Expected: PASS — тесты `OdeCard` зелёные.

- [ ] **Step 5: Commit**

```bash
git add compute_node/frontend/src/components/mps/OdeCard.tsx
git commit -m "fix(mps-ui): OdeCard — Ts = 20 мс (реальный период МПС-цикла)"
```

(добавить `OdeCard.test.tsx` в `git add`, если он правился на Step 3.)

---

## Task 13: Robot-верификация

> **Зависит от железа:** нужен включённый робот Samurai, Pi с поднятым `mps_node`, MQTT-брокер. Если железо недоступно — задача deferred, отметить в `physical_tests_pending.md` и в PR. Sim-верификация (Task 7) при этом остаётся достаточным гейтом для мержа Python-части.

**Files:** нет правок кода

- [ ] **Step 1: Поднять робота и проверить связь**

Запустить робота (`./samurai.sh robot`), убедиться что `mps_node` в логах поднялся (`mps_node started — tick_dt=...`). Проверить MQTT-связь с дашбордом: в дашборде `:5000` индикатор робота должен быть online.

- [ ] **Step 2: Применить матрицы на робота**

Через дашборд (`/mps`, кнопка Apply) или curl:
```bash
curl -s -X POST http://localhost:5000/api/v1/mps/matrices/reset
```
Expected: `status: applied`; в логах `mps_node` — `mps: matrices applied (N=10, Ts=0.020, λ_open(Ad)=['1.000', '1.000', '1.000', '0.875', '0.819'])` (дискретизированные собственные значения, не сырые непрерывные).

- [ ] **Step 3: Прогон сценария на роботе**

Робот на ровной поверхности, пространство ≥ 2.5 м впереди свободно.
```bash
curl -s -X POST http://localhost:5000/api/v1/mps/scenario/run -H "Content-Type: application/json" -d '{"distance":2.0,"v_target":0.15,"source":"robot"}'
```
Затем опрашивать `GET /api/v1/mps/scenario/{run_id}` или смотреть Live в дашборде.
Expected: робот физически едет вперёд ~2 м плавно (без рывков/раскачки), финальный `status: reached`, `ss_error` разумный (< 0.1 м с учётом одометрии).

- [ ] **Step 4: Зафиксировать результат**

Записать в PR: вывод `mps/scenario/finished` (status + metrics) + краткое наблюдение поведения робота. Если поведение плохое — вернуться к Task 7 Step 5 (подбор Q/R/N), повторить.

---

## Финал: прогон всех тестов перед PR

- [ ] Полный прогон Python-тестов: `pytest tests/ -q` — все зелёные.
- [ ] MATLAB-тесты (если доступен MATLAB/Octave): `matlab -batch "addpath('matlab'); test_build_canonical_mps; test_export_to_yaml"` — PASS.
- [ ] Frontend: `cd compute_node/frontend && npm run test` — зелёные (или хотя бы `npm run test -- mps`).
- [ ] `git log --oneline fix/camera-flip..HEAD` — проверить, что история коммитов чистая и по теме.
```
