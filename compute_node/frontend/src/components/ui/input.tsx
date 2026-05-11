import * as React from "react"
import { cn } from "@/lib/utils"

const Input = React.forwardRef<HTMLInputElement, React.ComponentProps<"input">>(
  ({ className, type = "text", ...props }, ref) => {
    return (
      <input
        type={type}
        ref={ref}
        className={cn(
          "h-9 w-full rounded-md border border-subtle bg-surface-2 px-3 py-1",
          "text-body text-foreground placeholder:text-foreground-faint",
          "file:border-0 file:bg-transparent file:text-small file:font-medium file:text-foreground",
          "focus-visible:outline-none focus-visible:border-accent focus-visible:ring-2 focus-visible:ring-accent/30",
          "disabled:cursor-not-allowed disabled:opacity-50",
          "transition-colors duration-fast",
          className,
        )}
        {...props}
      />
    )
  }
)
Input.displayName = "Input"

export { Input }
