import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { ScrollArea } from '@/components/ui/scroll-area'
import { COLOUR_CSS, COLOUR_RU } from '@/lib/constants'
import type { BallInfo } from '@/types/robot'

interface BallsTableProps {
  balls: BallInfo[]
}

export function BallsTable({ balls }: BallsTableProps) {
  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Мячи на арене ({balls.length})
        </CardTitle>
      </CardHeader>
      <CardContent className="p-0">
        <ScrollArea className="max-h-[200px]">
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead className="text-[10px]">ID</TableHead>
                <TableHead className="text-[10px]">Цвет</TableHead>
                <TableHead className="text-[10px]">X</TableHead>
                <TableHead className="text-[10px]">Y</TableHead>
                <TableHead className="text-[10px]">Статус</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              {balls.length === 0 ? (
                <TableRow>
                  <TableCell colSpan={5} className="text-xs text-center text-muted-foreground py-4">
                    Нет мячей
                  </TableCell>
                </TableRow>
              ) : (
                balls.map((b) => (
                  <TableRow key={b.id}>
                    <TableCell className="text-xs py-1">{b.id}</TableCell>
                    <TableCell className="text-xs py-1">
                      <span className="flex items-center gap-1.5">
                        <span
                          className="w-2 h-2 rounded-full inline-block shrink-0"
                          style={{ backgroundColor: COLOUR_CSS[b.colour] }}
                        />
                        {COLOUR_RU[b.colour] || b.colour}
                      </span>
                    </TableCell>
                    <TableCell className="text-xs py-1 font-mono">{b.x?.toFixed(2)}</TableCell>
                    <TableCell className="text-xs py-1 font-mono">{b.y?.toFixed(2)}</TableCell>
                    <TableCell className="text-xs py-1">
                      <span className={b.grabbed ? 'text-samurai-red' : 'text-samurai-green'}>
                        {b.grabbed ? 'Захвачен' : 'Активен'}
                      </span>
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
