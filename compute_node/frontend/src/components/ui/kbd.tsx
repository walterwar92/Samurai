import * as React from "react"
import { cn } from "@/lib/utils"

/**
 * Kbd — клавиатурная клавиша / shortcut indicator.
 * Моноспейс, тонкий бордер, surface-2 фон.
 */
const Kbd = React.forwardRef<HTMLElement, React.HTMLAttributes<HTMLElement>>(
  ({ className, children, ...props }, ref) => (
    <kbd
      ref={ref}
      className={cn(
        "inline-flex items-center justify-center min-w-[1.5rem] h-5 px-1.5 rounded",
        "border border-subtle bg-surface-2 font-mono text-micro text-foreground-muted",
        className,
      )}
      {...props}
    >
      {children}
    </kbd>
  )
)
Kbd.displayName = "Kbd"

export { Kbd }
