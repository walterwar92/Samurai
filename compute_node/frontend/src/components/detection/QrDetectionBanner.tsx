import { QrCode } from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'

interface QrDetectionBannerProps {
  qr: { data: string; timestamp?: number } | null
}

export function QrDetectionBanner({ qr }: QrDetectionBannerProps) {
  if (!qr || !qr.data) return null

  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          QR-код
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3">
        <div className="flex items-center gap-2 text-xs">
          <QrCode size={14} className="text-blue-400 shrink-0" />
          <span className="font-mono truncate">{qr.data}</span>
        </div>
      </CardContent>
    </Card>
  )
}
