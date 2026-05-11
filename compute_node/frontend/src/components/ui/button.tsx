import * as React from "react"
import { Slot } from "@radix-ui/react-slot"
import { cva, type VariantProps } from "class-variance-authority"

import { cn } from "@/lib/utils"

const buttonVariants = cva(
  cn(
    "inline-flex items-center justify-center gap-1.5 whitespace-nowrap rounded-md font-medium",
    "transition-all duration-fast ease-standard",
    "focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-accent/40 focus-visible:ring-offset-2 focus-visible:ring-offset-background",
    "disabled:opacity-50 disabled:pointer-events-none",
    "[&_svg]:pointer-events-none [&_svg]:shrink-0",
  ),
  {
    variants: {
      variant: {
        default:
          "bg-accent text-accent-foreground hover:bg-accent-hover active:bg-accent-active active:scale-[.98]",
        secondary:
          "bg-surface-2 text-foreground hover:bg-surface-3 border border-subtle",
        ghost:
          "bg-transparent text-foreground-muted hover:bg-surface-2 hover:text-foreground",
        outline:
          "border border-strong bg-transparent hover:bg-surface-2",
        destructive:
          "bg-danger/90 text-foreground hover:bg-danger active:scale-[.98]",
        link:
          "text-accent underline-offset-4 hover:underline px-0",
      },
      size: {
        sm: "h-8 px-2.5 text-small",
        default: "h-9 px-3 text-body",
        lg: "h-11 px-4 text-body",
        icon: "h-9 w-9",
      },
    },
    defaultVariants: {
      variant: "default",
      size: "default",
    },
  }
)

export interface ButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement>,
    VariantProps<typeof buttonVariants> {
  asChild?: boolean
}

const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant, size, asChild = false, ...props }, ref) => {
    const Comp = asChild ? Slot : "button"
    return (
      <Comp
        className={cn(buttonVariants({ variant, size, className }))}
        ref={ref}
        {...props}
      />
    )
  }
)
Button.displayName = "Button"

export { Button, buttonVariants }
