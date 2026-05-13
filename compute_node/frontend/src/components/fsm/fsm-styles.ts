import type { FsmState } from '@/types/robot'

/**
 * Tailwind text-color classes для FSM-состояний.
 * TARGETING — единственный coral-accent в палитре FSM (текущий фокус).
 * Остальные — muted ink-tones.
 */
export const FSM_TEXT_CLASS: Record<FsmState, string> = {
  IDLE: 'text-fsm-idle',
  SEARCHING: 'text-fsm-searching',
  TARGETING: 'text-accent',
  APPROACHING: 'text-fsm-approaching',
  GRABBING: 'text-fsm-grabbing',
  CALLING: 'text-fsm-calling',
  RETURNING: 'text-fsm-returning',
}

export const FSM_BG_CLASS: Record<FsmState, string> = {
  IDLE: 'bg-fsm-idle',
  SEARCHING: 'bg-fsm-searching',
  TARGETING: 'bg-accent',
  APPROACHING: 'bg-fsm-approaching',
  GRABBING: 'bg-fsm-grabbing',
  CALLING: 'bg-fsm-calling',
  RETURNING: 'bg-fsm-returning',
}

/**
 * Hex для случаев, где Tailwind-классы не работают:
 * inline SVG `fill=` / canvas / three.js material color.
 */
export const FSM_DOT_HEX: Record<FsmState, string> = {
  IDLE: '#8A8278',
  SEARCHING: '#7DA1C9',
  TARGETING: '#CC785C',
  APPROACHING: '#C9A26B',
  GRABBING: '#B894C9',
  CALLING: '#7DA88A',
  RETURNING: '#9A9089',
}

/** Сокращённые имена для FsmTimeline и компактных бейджей. */
export const FSM_SHORT: Record<FsmState, string> = {
  IDLE: 'IDL',
  SEARCHING: 'SRCH',
  TARGETING: 'TGT',
  APPROACHING: 'APR',
  GRABBING: 'GRB',
  CALLING: 'CAL',
  RETURNING: 'RTN',
}

/** Канонический порядок состояний для timeline (IDLE → ... → RETURNING). */
export const FSM_ORDER: ReadonlyArray<FsmState> = [
  'IDLE',
  'SEARCHING',
  'TARGETING',
  'APPROACHING',
  'GRABBING',
  'CALLING',
  'RETURNING',
]
