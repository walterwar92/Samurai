import * as React from "react"
import { cva, type VariantProps } from "class-variance-authority"

import { cn } from "@/lib/utils"

const badgeVariants = cva(
  cn(
    "inline-flex items-center gap-1.5 rounded-pill border px-2 py-0.5",
    "font-mono text-micro uppercase tracking-wider",
    "transition-colors duration-fast",
  ),
  {
    variants: {
      tone: {
        default: "bg-surface-2 text-foreground-muted border-subtle",
        accent: "bg-accent/10 text-accent border-accent/20",
        success: "bg-success/10 text-success border-success/25",
        warning: "bg-warning/10 text-warning border-warning/25",
        danger: "bg-danger/10 text-danger border-danger/25",
        info: "bg-info/10 text-info border-info/25",
      },
    },
    defaultVariants: {
      tone: "default",
    },
  }
)

// Legacy `variant` → `tone` map для обратной совместимости со старыми
// потребителями (Header.tsx: variant="destructive", и т.п.).
const LEGACY_VARIANT_MAP: Record<string, BadgeTone> = {
  default: "accent",
  secondary: "default",
  destructive: "danger",
  outline: "default",
}

type BadgeTone = NonNullable<VariantProps<typeof badgeVariants>["tone"]>

export interface BadgeProps
  extends React.HTMLAttributes<HTMLDivElement>,
    VariantProps<typeof badgeVariants> {
  /** Legacy shadcn API — переадресуется на `tone`. Новые компоненты используют `tone`. */
  variant?: "default" | "secondary" | "destructive" | "outline"
  /** Показывать ли точку слева от текста. */
  dot?: boolean
}

function Badge({ className, tone, variant, dot = false, children, ...props }: BadgeProps) {
  const resolved: BadgeTone = tone ?? (variant ? LEGACY_VARIANT_MAP[variant] : "default")
  return (
    <div className={cn(badgeVariants({ tone: resolved }), className)} {...props}>
      {dot && <span className="h-1.5 w-1.5 rounded-full bg-current shrink-0" />}
      {children}
    </div>
  )
}

export { Badge, badgeVariants }
