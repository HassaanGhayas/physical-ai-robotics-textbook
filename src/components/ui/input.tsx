/**
 * Input Component (shadcn/ui)
 * Form input component with consistent styling
 */

import * as React from "react"
import { cn } from "@/lib/utils"

export interface InputProps
  extends React.InputHTMLAttributes<HTMLInputElement> {}

const Input = React.forwardRef<HTMLInputElement, InputProps>(
  ({ className, type, ...props }, ref) => {
    return (
      <input
        type={type}
        className={cn(
          "flex h-10 w-full rounded-md border-2 border-grayscale-300 bg-grayscale-0 px-3 py-2 text-sm text-grayscale-900 placeholder:text-grayscale-500 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-grayscale-900 focus-visible:ring-offset-2 disabled:cursor-not-allowed disabled:opacity-50 dark:border-grayscale-700 dark:bg-grayscale-950 dark:text-grayscale-50 dark:placeholder:text-grayscale-400 dark:focus-visible:ring-grayscale-300",
          className
        )}
        ref={ref}
        {...props}
      />
    )
  }
)
Input.displayName = "Input"

export { Input }
