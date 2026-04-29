import { useState, type KeyboardEvent } from 'react'
import { Input } from '@/components/ui/input'
import { Button } from '@/components/ui/button'
import { Send } from 'lucide-react'
import { useSend } from '@/stores/selectors'

export function CommandInput() {
  const [text, setText] = useState('')
  const send = useSend()

  const handleSend = () => {
    const trimmed = text.trim()
    if (!trimmed) return
    send(trimmed)
    setText('')
  }

  const handleKey = (e: KeyboardEvent) => {
    if (e.key === 'Enter') handleSend()
  }

  return (
    <div className="flex gap-2">
      <Input
        value={text}
        onChange={(e) => setText(e.target.value)}
        onKeyDown={handleKey}
        placeholder="Введите команду (напр. найди красный мяч)..."
        className="text-sm"
      />
      <Button onClick={handleSend} size="sm" className="shrink-0">
        <Send className="w-4 h-4" />
      </Button>
    </div>
  )
}
