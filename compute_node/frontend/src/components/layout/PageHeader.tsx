import type { ReactNode } from 'react'
import { Button } from '@/components/ui/button'
import { TerminalIcon } from '@/components/icons'

export interface PageHeaderProps {
  title: string
  simTime?: string
  right?: ReactNode
  onDebug?: () => void
}

/**
 * Контекстный заголовок страницы. Sticky сверху main-area (после Sidebar),
 * высота 48px, тонкая нижняя граница. Слева — заголовок, справа — sim time +
 * action-кнопки + опциональный Debug-триггер.
 */
export function PageHeader({ title, simTime, right, onDebug }: PageHeaderProps) {
  return (
    <header className="sticky top-0 z-20 h-12 border-b border-subtle bg-surface-1/95 backdrop-blur-sm">
      <div className="h-full px-6 flex items-center justify-between">
        <h1 className="text-display">{title}</h1>
        <div className="flex items-center gap-3">
          {simTime && (
            <span className="font-mono text-small text-foreground-muted tabular-nums">
              {simTime}
            </span>
          )}
          {right}
          {onDebug && (
            <Button variant="secondary" size="sm" onClick={onDebug}>
              <TerminalIcon className="h-3.5 w-3.5" />
              Debug
            </Button>
          )}
        </div>
      </div>
    </header>
  )
}
