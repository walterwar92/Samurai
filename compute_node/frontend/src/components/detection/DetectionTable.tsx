import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import { COLOUR_CSS, COLOUR_RU } from '@/lib/constants'
import type { Detection } from '@/types/robot'

interface DetectionTableProps {
  detections: Detection[]
  closest: Detection | null
}

export function DetectionTable({ detections, closest }: DetectionTableProps) {
  return (
    <Card>
      <CardHeader className="py-2 px-3 flex-row items-center justify-between">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Детекции
        </CardTitle>
        <Badge variant="secondary" className="text-[10px]">
          {detections.length}
        </Badge>
      </CardHeader>
      <CardContent className="p-0">
        {closest && closest.colour && (
          <div className="flex items-center gap-2 px-3 py-1.5 border-b border-border text-xs bg-secondary/30">
            <div
              className="w-2.5 h-2.5 rounded-full shrink-0"
              style={{ backgroundColor: COLOUR_CSS[closest.colour] }}
            />
            <span>
              Ближайший: {COLOUR_RU[closest.colour] || closest.colour} —{' '}
              {closest.distance?.toFixed(2)} м
            </span>
          </div>
        )}
        <ScrollArea className="max-h-[200px]">
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead className="text-[10px]">Цвет</TableHead>
                <TableHead className="text-[10px]">Увер.</TableHead>
                <TableHead className="text-[10px]">Дист.</TableHead>
                <TableHead className="text-[10px]">x,y,w,h</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              {detections.length === 0 ? (
                <TableRow>
                  <TableCell colSpan={4} className="text-xs text-center text-muted-foreground py-4">
                    Нет детекций
                  </TableCell>
                </TableRow>
              ) : (
                detections.map((d, i) => (
                  <TableRow key={i}>
                    <TableCell className="text-xs py-1">
                      <span className="flex items-center gap-1.5">
                        <span
                          className="w-2 h-2 rounded-full inline-block shrink-0"
                          style={{ backgroundColor: COLOUR_CSS[d.colour] }}
                        />
                        {COLOUR_RU[d.colour] || d.colour}
                      </span>
                    </TableCell>
                    <TableCell className="text-xs py-1 font-mono">
                      {(d.conf * 100).toFixed(0)}%
                    </TableCell>
                    <TableCell className="text-xs py-1 font-mono">
                      {d.distance?.toFixed(2)}
                    </TableCell>
                    <TableCell className="text-xs py-1 font-mono">
                      {d.x},{d.y},{d.w},{d.h}
                    </TableCell>
                  </TableRow>
                ))
              )}
            </TableBody>
          </Table>
        </ScrollArea>
      </CardContent>
    </Card>
  )
}
