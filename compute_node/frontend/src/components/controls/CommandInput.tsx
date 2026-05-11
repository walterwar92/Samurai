import { useState, type KeyboardEvent } from 'react'
import { useSend } from '@/stores/selectors'
import { Kbd } from '@/components/ui/kbd'
import { cn } from '@/lib/utils'

export interface CommandInputProps {
  placeholder?: string
  disabled?: boolean
}

/**
 * CommandInput — терминальный текстовый ввод команд (voice-like).
 * Префикс `>` слева (мигает при focus), input — JetBrains Mono, kbd ↵ справа.
 * Enter отправляет, очищает поле.
 */
export function CommandInput({
  placeholder = 'найди красный мяч',
  disabled,
}: CommandInputProps) {
  const [text, setText] = useState('')
  const [focused, setFocused] = useState(false)
  const send = useSend()

  const handleSend = () => {
    const trimmed = text.trim()
    if (!trimmed) return
    send(trimmed)
    setText('')
  }

  const handleKey = (e: KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') handleSend()
  }

  return (
    <div
      className={cn(
        'flex items-center gap-2 rounded-md border bg-surface-1 px-3 py-2',
        'transition-colors duration-fast',
        focused ? 'border-accent' : 'border-subtle',
        disabled && 'opacity-50 pointer-events-none',
      )}
    >
      <span
        className={cn(
          'font-mono text-body text-accent select-none',
          focused && 'animate-blink',
        )}
      >
        &gt;
      </span>
      <input
        value={text}
        onChange={(e) => setText(e.target.value)}
        onKeyDown={handleKey}
        onFocus={() => setFocused(true)}
        onBlur={() => setFocused(false)}
        placeholder={placeholder}
        disabled={disabled}
        className={cn(
          'flex-1 bg-transparent border-none outline-none',
          'font-mono text-body text-foreground placeholder:text-foreground-faint',
          'focus:ring-0',
        )}
      />
      <Kbd>↵</Kbd>
    </div>
  )
}
