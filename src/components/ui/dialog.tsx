/**
 * Dialog Component (shadcn/ui)
 * Modal dialog component with overlay
 */

import * as React from "react"
import { cn } from "@/lib/utils"

export interface DialogProps {
  open?: boolean
  onOpenChange?: (open: boolean) => void
  children?: React.ReactNode
}

export function Dialog({ open, onOpenChange, children }: DialogProps) {
  React.useEffect(() => {
    if (open) {
      document.body.style.overflow = 'hidden'
    } else {
      document.body.style.overflow = 'unset'
    }
    return () => {
      document.body.style.overflow = 'unset'
    }
  }, [open])

  if (!open) return null

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center">
      <DialogOverlay onClick={() => onOpenChange?.(false)} />
      <div className="relative z-50">{children}</div>
    </div>
  )
}

export interface DialogOverlayProps
  extends React.HTMLAttributes<HTMLDivElement> {}

export function DialogOverlay({ className, ...props }: DialogOverlayProps) {
  return (
    <div
      className={cn(
        "fixed inset-0 z-50 bg-grayscale-950/80 backdrop-blur-sm dark:bg-grayscale-950/80",
        className
      )}
      {...props}
    />
  )
}

export interface DialogContentProps
  extends React.HTMLAttributes<HTMLDivElement> {}

export function DialogContent({ className, children, ...props }: DialogContentProps) {
  return (
    <div
      role="dialog"
      aria-modal="true"
      className={cn(
        "fixed left-[50%] top-[50%] z-50 grid w-full max-w-lg translate-x-[-50%] translate-y-[-50%] gap-4 border-2 border-grayscale-300 bg-grayscale-0 p-6 shadow-lg rounded-lg dark:border-grayscale-700 dark:bg-grayscale-950",
        className
      )}
      {...props}
    >
      {children}
    </div>
  )
}

export interface DialogHeaderProps
  extends React.HTMLAttributes<HTMLDivElement> {}

export function DialogHeader({ className, ...props }: DialogHeaderProps) {
  return (
    <div
      className={cn(
        "flex flex-col space-y-1.5 text-center sm:text-left",
        className
      )}
      {...props}
    />
  )
}

export interface DialogFooterProps
  extends React.HTMLAttributes<HTMLDivElement> {}

export function DialogFooter({ className, ...props }: DialogFooterProps) {
  return (
    <div
      className={cn(
        "flex flex-col-reverse sm:flex-row sm:justify-end sm:space-x-2",
        className
      )}
      {...props}
    />
  )
}

export interface DialogTitleProps
  extends React.HTMLAttributes<HTMLHeadingElement> {}

export function DialogTitle({ className, ...props }: DialogTitleProps) {
  return (
    <h2
      className={cn(
        "text-lg font-semibold leading-none tracking-tight text-grayscale-900 dark:text-grayscale-50",
        className
      )}
      {...props}
    />
  )
}

export interface DialogDescriptionProps
  extends React.HTMLAttributes<HTMLParagraphElement> {}

export function DialogDescription({ className, ...props }: DialogDescriptionProps) {
  return (
    <p
      className={cn("text-sm text-grayscale-600 dark:text-grayscale-400", className)}
      {...props}
    />
  )
}
