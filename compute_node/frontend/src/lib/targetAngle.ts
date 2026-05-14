// Чистая математика 3D-пикера цели МПС. Без зависимости от Three/WebGL,
// чтобы покрываться юнит-тестами в jsdom.
//
// Соглашение о координатах (как в Mps3DScene): мир (wx, wy) ↔ Three
// (wx, h, -wy). Робот yaw=0 смотрит по +X. Угол φ — относительный курс:
// φ=0 — прямо вперёд, φ>0 — влево (CCW), φ<0 — вправо (CW).

/** Высота маркера/кольца над полом сцены (Three Y), м. */
export const MARKER_HEIGHT = 0.03

/** Зона у центра: клики ближе этой доли радиуса к роботу игнорируются
 *  (там угол скачет от микродвижений мыши). */
export const CENTER_DEADZONE_FRACTION = 0.3

/**
 * Точка клика на полу сцены (Three-координаты x, z) → относительный курс φ.
 * Возвращает угол в радианах в диапазоне (−π, π].
 */
export function groundPointToAngle(threeX: number, threeZ: number): number {
  // мир: wx = threeX, wy = -threeZ. φ = atan2(wy, wx).
  return Math.atan2(-threeZ, threeX)
}

/**
 * Угол φ + радиус N → позиция маркера в Three-координатах [x, h, z].
 * Маркер всегда на окружности радиуса N (дистанция фиксирована).
 */
export function angleToMarkerPosition(
  angle: number,
  radius: number,
): [number, number, number] {
  return [radius * Math.cos(angle), MARKER_HEIGHT, -radius * Math.sin(angle)]
}

/**
 * φ → подпись для readout. «прямо» в дедзоне ~3°, иначе «+35°» / «-40°».
 * Знак: + влево (CCW), − вправо (CW).
 */
export function formatHeadingLabel(angle: number): string {
  const deg = (angle * 180) / Math.PI
  if (Math.abs(deg) < 3) return 'прямо'
  const rounded = Math.round(deg)
  return rounded > 0 ? `+${rounded}°` : `${rounded}°`
}

/**
 * Расстояние точки пола от центра (робота), м — для дедзоны центра.
 * threeX/threeZ — Three-координаты точки на полу.
 */
export function groundPointRadius(threeX: number, threeZ: number): number {
  return Math.hypot(threeX, threeZ)
}
