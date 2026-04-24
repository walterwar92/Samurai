import { Header } from '@/components/layout/Header'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Separator } from '@/components/ui/separator'
import { useRobot } from '@/providers/RobotProvider'

export function VperedPage() {
  const { robot } = useRobot()

  return (
    <div className="min-h-screen">
      <Header />

      <div className="max-w-[1100px] mx-auto p-4 space-y-3">
        {/* Заголовок */}
        <Card>
          <CardHeader className="py-3 px-4">
            <div className="flex items-center justify-between">
              <div>
                <CardTitle className="text-base">Робот Vpered</CardTitle>
                <div className="text-[11px] text-muted-foreground mt-1">
                  {robot.controller} · без WiFi — только USB / Serial
                </div>
              </div>
              <Badge variant="destructive" className="text-[10px]">OFFLINE</Badge>
            </div>
          </CardHeader>
        </Card>

        <div className="grid grid-cols-1 lg:grid-cols-2 gap-3">
          {/* Возможности */}
          <Card>
            <CardHeader className="py-2 px-3">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Возможности
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3">
              <div className="space-y-1 text-xs">
                <CapRow label="Двигатели" value="2× задних (PWM + 74HC595)" ok />
                <CapRow label="Клешня" value="3 серво (claw / arm / base)" ok />
                <CapRow label="Ультразвук" value="HC-SR04 (TRIG=D12 / ECHO=D13)" ok />
                <CapRow label="IMU" value="MPU-6050 (I2C 0x68)" ok />
                <CapRow label="Камера" value="—" />
                <CapRow label="WiFi / MQTT" value="—" />
                <CapRow label="Голос / YOLO / SLAM" value="—" />
              </div>
            </CardContent>
          </Card>

          {/* Распиновка */}
          <Card>
            <CardHeader className="py-2 px-3">
              <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
                Распиновка Arduino Uno
              </CardTitle>
            </CardHeader>
            <CardContent className="p-3">
              <div className="space-y-1 text-xs font-mono">
                <PinRow pin="D2"  role="SHCP (shift clock)" />
                <PinRow pin="D4"  role="STCP (storage clock)" />
                <PinRow pin="D5"  role="PWM1 — левый мотор" />
                <PinRow pin="D6"  role="PWM2 — правый мотор" />
                <PinRow pin="D7"  role="EN (motor driver)" />
                <PinRow pin="D8"  role="DATA (shift register)" />
                <PinRow pin="D9"  role="Серво BASE" />
                <PinRow pin="D10" role="Серво ARM" />
                <PinRow pin="D11" role="Серво CLAW" />
                <PinRow pin="D12" role="TRIG (ultrasonic)" />
                <PinRow pin="D13" role="ECHO (ultrasonic)" />
                <PinRow pin="A4/A5" role="I2C — MPU-6050" />
              </div>
            </CardContent>
          </Card>
        </div>

        {/* Прошивка */}
        <Card>
          <CardHeader className="py-2 px-3">
            <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
              Прошивка — «едет вперёд, стоп перед препятствием»
            </CardTitle>
          </CardHeader>
          <CardContent className="p-3 space-y-3">
            <div className="text-xs text-muted-foreground">
              Файл:{' '}
              <code className="text-zinc-300 bg-muted/40 px-1.5 py-0.5 rounded">
                firmware/vpered_uno/vpered_uno.ino
              </code>
            </div>

            <Separator />

            <div>
              <div className="text-[11px] uppercase text-muted-foreground mb-1.5">
                Исправление «левые колёса стартуют позже и медленнее»
              </div>
              <ul className="text-xs space-y-1.5 list-disc pl-4 text-zinc-300">
                <li>
                  <b>Kick-start</b> — короткий импульс PWM=255 одновременно на оба
                  мотора (~180 мс), чтобы оба борта преодолели статическое трение
                  вместе. Раньше старт был только для правого, левый оставался
                  стоять.
                </li>
                <li>
                  <b>MIN_PWM = 120</b> — нижний порог, ниже которого моторы не
                  крутятся стабильно. ПИД-коррекция не проваливается в мёртвую
                  зону.
                </li>
                <li>
                  <b>LEFT_TRIM = 1.12</b> — множитель PWM для левого мотора.
                  Компенсирует механическую разницу (трение, редуктор). Подбирается
                  опытно: уводит вправо → увеличить; уводит влево → уменьшить.
                </li>
                <li>
                  <b>Калибровка гироскопа</b> при старте (робот стоит неподвижно
                  ~0.6 сек) — убирает накопление ошибки от bias.
                </li>
              </ul>
            </div>

            <Separator />

            <div>
              <div className="text-[11px] uppercase text-muted-foreground mb-1.5">
                Как прошить
              </div>
              <ol className="text-xs space-y-1 list-decimal pl-4 text-zinc-300">
                <li>Arduino IDE → Board: Arduino Uno → выбрать COM-порт.</li>
                <li>Открыть <code>firmware/vpered_uno/vpered_uno.ino</code>.</li>
                <li>Upload (Ctrl+U).</li>
                <li>
                  Serial Monitor 9600 бод — видно <code>theta</code>, <code>L</code>,
                  <code>R</code>, <code>dist</code> для калибровки.
                </li>
              </ol>
            </div>
          </CardContent>
        </Card>

        {/* Roadmap */}
        <Card>
          <CardHeader className="py-2 px-3">
            <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
              План интеграции
            </CardTitle>
          </CardHeader>
          <CardContent className="p-3">
            <ol className="text-xs space-y-1.5 list-decimal pl-4 text-zinc-400">
              <li>✅ Прошивка: автономный режим «вперёд + стоп».</li>
              <li>⬜ Добавить WiFi-модуль (ESP8266 / ESP-01) для UART-bridge к MQTT.</li>
              <li>⬜ Протокол UART — простые команды <code>cmd_vel</code>, телеметрия дистанции и угла.</li>
              <li>⬜ MQTT-мост на ПК: <code>samurai/vpered/*</code> ↔ Serial.</li>
              <li>⬜ Управление клешнёй и серво с дашборда.</li>
            </ol>
          </CardContent>
        </Card>
      </div>
    </div>
  )
}

function CapRow({ label, value, ok }: { label: string; value: string; ok?: boolean }) {
  return (
    <div className="flex justify-between items-center">
      <span className="text-muted-foreground">{label}</span>
      <span className={ok ? 'text-zinc-200' : 'text-zinc-500'}>{value}</span>
    </div>
  )
}

function PinRow({ pin, role }: { pin: string; role: string }) {
  return (
    <div className="flex justify-between">
      <span className="text-primary">{pin}</span>
      <span className="text-zinc-400">{role}</span>
    </div>
  )
}
