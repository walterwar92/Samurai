/**
 * Стили цветов мячей для редизайна.
 * Десатурированные (~25% от чистых) для тёплой графитовой темы.
 * `COLOUR_CSS` в `src/lib/constants.ts` остаётся как есть (legacy),
 * новые компоненты используют эти карты.
 */
export const BALL_NAMES = ['red', 'blue', 'green', 'yellow', 'orange', 'white', 'black'] as const
export type BallColour = (typeof BALL_NAMES)[number]

export const BALL_HEX: Record<BallColour, string> = {
  red: '#B85C5C',
  blue: '#5C7DC9',
  green: '#6BA86B',
  yellow: '#C9B05C',
  orange: '#C9885C',
  white: '#D8D2C8',
  black: '#2A2826',
}

export const BALL_BG_CLASS: Record<BallColour, string> = {
  red: 'bg-ball-red',
  blue: 'bg-ball-blue',
  green: 'bg-ball-green',
  yellow: 'bg-ball-yellow',
  orange: 'bg-ball-orange',
  white: 'bg-ball-white',
  black: 'bg-ball-black',
}

export const BALL_TEXT_CLASS: Record<BallColour, string> = {
  red: 'text-ball-red',
  blue: 'text-ball-blue',
  green: 'text-ball-green',
  yellow: 'text-ball-yellow',
  orange: 'text-ball-orange',
  white: 'text-ball-white',
  black: 'text-ball-black',
}
