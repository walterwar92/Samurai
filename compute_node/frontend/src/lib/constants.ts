import type { FsmState } from '@/types/robot'

export const COLOUR_RU: Record<string, string> = {
  red: 'красный',
  blue: 'синий',
  green: 'зелёный',
  yellow: 'жёлтый',
  orange: 'оранжевый',
  white: 'белый',
  black: 'чёрный',
  unknown: 'неизвестный',
}

export const COLOUR_CSS: Record<string, string> = {
  red: '#ef5350',
  blue: '#42a5f5',
  green: '#66bb6a',
  yellow: '#ffee58',
  orange: '#ffa726',
  white: '#fafafa',
  black: '#424242',
  unknown: '#9e9e9e',
}

export const ACTION_RU: Record<string, string> = {
  grab: 'захват',
}

export const ALL_STATES: FsmState[] = [
  'IDLE',
  'SEARCHING',
  'TARGETING',
  'APPROACHING',
  'GRABBING',
  'CALLING',
  'RETURNING',
]

export const FSM_COLORS: Record<FsmState, string> = {
  IDLE: '#37474f',
  SEARCHING: '#1b5e20',
  TARGETING: '#f57f17',
  APPROACHING: '#e65100',
  GRABBING: '#4a148c',
  CALLING: '#01579b',
  RETURNING: '#33691e',
}

export const QUICK_COMMANDS = [
  { label: 'Красный', command: 'найди красный мяч', color: '#ef5350' },
  { label: 'Синий', command: 'найди синий мяч', color: '#42a5f5' },
  { label: 'Зелёный', command: 'найди зелёный мяч', color: '#66bb6a' },
  { label: 'Жёлтый', command: 'найди жёлтый мяч', color: '#ffee58' },
  { label: 'Оранжевый', command: 'найди оранжевый мяч', color: '#ffa726' },
  { label: 'Стоп', command: 'стоп', color: undefined, variant: 'destructive' as const },
  { label: 'Домой', command: 'домой', color: undefined },
]
