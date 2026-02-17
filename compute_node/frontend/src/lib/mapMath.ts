import type { MapInfo } from '@/types/robot'

export interface MapTransform {
  offsetX: number
  offsetY: number
  scale: number
  info: MapInfo
}

export function canvasToWorld(
  cx: number,
  cy: number,
  transform: MapTransform
): { wx: number; wy: number } {
  const { offsetX, offsetY, scale, info } = transform
  const mx = (cx - offsetX) / scale
  const my = (cy - offsetY) / scale
  const wx = mx * info.resolution + info.origin_x
  const wy = (info.height - my) * info.resolution + info.origin_y
  return { wx, wy }
}

export function worldToCanvas(
  wx: number,
  wy: number,
  transform: MapTransform
): { cx: number; cy: number } {
  const { offsetX, offsetY, scale, info } = transform
  const mx = (wx - info.origin_x) / info.resolution
  const my = info.height - (wy - info.origin_y) / info.resolution
  return {
    cx: mx * scale + offsetX,
    cy: my * scale + offsetY,
  }
}
