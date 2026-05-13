import type { SVGProps } from 'react'

/**
 * KatanaIcon — monoline 1.5px stroke, 24px viewBox.
 * Используется в Sidebar и брендинговых местах. Цвет — `currentColor`,
 * управляется через text-* классы (обычно `text-accent`).
 */
export function KatanaIcon({ className, ...props }: SVGProps<SVGSVGElement>) {
  return (
    <svg
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth={1.5}
      strokeLinecap="round"
      strokeLinejoin="round"
      className={className}
      {...props}
    >
      <path d="M3.5 20.5 L8 16" />
      <path d="M6.5 17.5 L8.5 19.5" />
      <path d="M9 15 L20.5 3.5" />
      <path d="M18.5 3.5 L20.5 3.5 L20.5 5.5" />
      <circle cx="7.5" cy="18.5" r=".4" fill="currentColor" />
    </svg>
  )
}
