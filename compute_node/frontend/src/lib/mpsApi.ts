/**
 * MPS API client — typed wrapper над `/api/v1/mps/*`.
 *
 * Контракт: docs/mps/api.md, типы: src/types/mps.ts.
 * Для нового функционала (feat/mps) идём напрямую в /api/v1/, без legacy
 * `/api/` (нет deprecation-alias).
 */
import type {
  MpsHistoryResponse,
  MpsMatrices,
  MpsMatricesGetResponse,
  MpsMatricesSetResponse,
  MpsScenarioAbortResponse,
  MpsScenarioRequest,
  MpsScenarioResult,
  MpsScenarioRunResponse,
  MpsValidateResult,
} from '@/types/mps'

const BASE = '/api/v1/mps'

class MpsApiError extends Error {
  status: number
  body: string

  constructor(status: number, body: string, message: string) {
    super(message)
    this.status = status
    this.body = body
  }
}

async function _fetch<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${BASE}${path}`, {
    cache: 'no-store',
    headers: {
      'Content-Type': 'application/json',
      ...(init?.headers ?? {}),
    },
    ...init,
  })
  if (!res.ok) {
    const text = await res.text().catch(() => '')
    throw new MpsApiError(res.status, text, `${res.status} ${res.statusText}`)
  }
  return (await res.json()) as T
}

export const mpsApi = {
  // Matrices
  getMatrices: () => _fetch<MpsMatricesGetResponse>('/matrices'),
  setDraft: (m: MpsMatrices) =>
    _fetch<MpsMatricesSetResponse>('/matrices', {
      method: 'POST',
      body: JSON.stringify(m),
    }),
  applyMatrices: (m?: MpsMatrices) =>
    _fetch<MpsMatricesSetResponse>('/matrices/apply', {
      method: 'POST',
      body: m ? JSON.stringify(m) : 'null',
    }),
  resetMatrices: () =>
    _fetch<MpsMatricesSetResponse>('/matrices/reset', { method: 'POST' }),

  // Validate
  validate: (m?: MpsMatrices | null) =>
    _fetch<MpsValidateResult>('/validate', {
      method: 'POST',
      body: JSON.stringify(m ?? null),
    }),

  // Scenario
  runScenario: (req: MpsScenarioRequest) =>
    _fetch<MpsScenarioRunResponse>('/scenario/run', {
      method: 'POST',
      body: JSON.stringify(req),
    }),
  scenarioStatus: (runId: string) =>
    _fetch<MpsScenarioResult>(`/scenario/${encodeURIComponent(runId)}`),
  abort: () =>
    _fetch<MpsScenarioAbortResponse>('/scenario/abort', { method: 'POST' }),

  // History
  history: () => _fetch<MpsHistoryResponse>('/history'),
  replay: (runId: string) =>
    _fetch<MpsScenarioRunResponse>(
      `/history/${encodeURIComponent(runId)}/replay`,
      { method: 'POST' },
    ),

  // Config save
  saveConfig: () =>
    _fetch<{ ok: true; written: boolean; path: string }>('/config/save', {
      method: 'POST',
    }),
}

export { MpsApiError }
