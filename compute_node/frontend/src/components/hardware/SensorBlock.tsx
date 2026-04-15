import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Separator } from '@/components/ui/separator'
import type { HardwarePreset, ImuType, CameraType, RangeType } from '@/types/hardware'

const inputCls = 'w-full bg-zinc-900 border border-zinc-700 rounded px-2 py-1 text-xs font-mono text-zinc-200 focus:outline-none focus:border-amber-500'

function TypeButtons<T extends string>({ options, value, onChange }: {
  options: { id: T; label: string }[]
  value: T
  onChange: (v: T) => void
}) {
  return (
    <div className="flex gap-1 flex-1">
      {options.map(o => (
        <button
          key={o.id}
          onClick={() => onChange(o.id)}
          className={`text-[10px] px-2 py-0.5 rounded flex-1 ${
            value === o.id
              ? 'bg-primary text-primary-foreground'
              : 'bg-zinc-800 text-zinc-400 hover:bg-zinc-700'
          }`}
        >
          {o.label}
        </button>
      ))}
    </div>
  )
}

interface SensorBlockProps {
  imu: HardwarePreset['imu']
  camera: HardwarePreset['camera']
  range_sensor: HardwarePreset['range_sensor']
  i2c: HardwarePreset['i2c']
  onImuChange: (imu: HardwarePreset['imu']) => void
  onCameraChange: (camera: HardwarePreset['camera']) => void
  onRangeChange: (range: HardwarePreset['range_sensor']) => void
  onI2cChange: (i2c: HardwarePreset['i2c']) => void
}

export function SensorBlock({
  imu, camera, range_sensor, i2c,
  onImuChange, onCameraChange, onRangeChange, onI2cChange,
}: SensorBlockProps) {
  return (
    <Card>
      <CardHeader className="py-2 px-3">
        <CardTitle className="text-[11px] uppercase tracking-wider text-muted-foreground font-semibold">
          Сенсоры и периферия
        </CardTitle>
      </CardHeader>
      <CardContent className="p-3 space-y-2.5">

        {/* IMU */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1">
            IMU
          </span>
          <div className="space-y-1">
            <div className="flex items-center gap-2">
              <span className="text-[10px] text-zinc-500 w-14 shrink-0">Тип</span>
              <TypeButtons
                options={[
                  { id: 'mpu6050' as ImuType, label: 'MPU6050' },
                  { id: 'bno055' as ImuType, label: 'BNO055' },
                  { id: 'none' as ImuType, label: 'Нет' },
                ]}
                value={imu.type}
                onChange={type => onImuChange({ ...imu, type })}
              />
            </div>
            {imu.type !== 'none' && (
              <div className="flex items-center gap-2">
                <span className="text-[10px] text-zinc-500 w-14 shrink-0">I2C</span>
                <input
                  className={inputCls}
                  value={imu.address}
                  onChange={e => onImuChange({ ...imu, address: e.target.value })}
                  placeholder="0x68"
                />
              </div>
            )}
          </div>
        </div>

        <Separator />

        {/* Camera */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1">
            Камера
          </span>
          <div className="space-y-1">
            <div className="flex items-center gap-2">
              <span className="text-[10px] text-zinc-500 w-14 shrink-0">Тип</span>
              <TypeButtons
                options={[
                  { id: 'usb' as CameraType, label: 'USB' },
                  { id: 'csi' as CameraType, label: 'CSI' },
                  { id: 'ip' as CameraType, label: 'IP' },
                  { id: 'none' as CameraType, label: 'Нет' },
                ]}
                value={camera.type}
                onChange={type => onCameraChange({ ...camera, type })}
              />
            </div>
            {camera.type !== 'none' && (
              <>
                <div className="flex items-center gap-2">
                  <span className="text-[10px] text-zinc-500 w-14 shrink-0">Device</span>
                  <input
                    className={inputCls}
                    value={camera.device}
                    onChange={e => onCameraChange({ ...camera, device: e.target.value })}
                    placeholder="/dev/video0"
                  />
                </div>
                <div className="grid grid-cols-2 gap-1.5">
                  <div>
                    <span className="text-[9px] text-zinc-500">Width</span>
                    <input
                      className={inputCls}
                      type="number"
                      value={camera.width}
                      onChange={e => onCameraChange({ ...camera, width: parseInt(e.target.value) || 640 })}
                    />
                  </div>
                  <div>
                    <span className="text-[9px] text-zinc-500">Height</span>
                    <input
                      className={inputCls}
                      type="number"
                      value={camera.height}
                      onChange={e => onCameraChange({ ...camera, height: parseInt(e.target.value) || 480 })}
                    />
                  </div>
                </div>
              </>
            )}
          </div>
        </div>

        <Separator />

        {/* Range sensor */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1">
            Дальномер
          </span>
          <div className="space-y-1">
            <div className="flex items-center gap-2">
              <span className="text-[10px] text-zinc-500 w-14 shrink-0">Тип</span>
              <TypeButtons
                options={[
                  { id: 'hc_sr04' as RangeType, label: 'HC-SR04' },
                  { id: 'vl53l0x' as RangeType, label: 'VL53L0X' },
                  { id: 'none' as RangeType, label: 'Нет' },
                ]}
                value={range_sensor.type}
                onChange={type => onRangeChange({ ...range_sensor, type })}
              />
            </div>
            {range_sensor.type === 'hc_sr04' && (
              <div className="grid grid-cols-2 gap-1.5">
                <div>
                  <span className="text-[9px] text-zinc-500">Trigger GPIO</span>
                  <input
                    className={inputCls}
                    type="number"
                    value={range_sensor.trigger_pin}
                    onChange={e => onRangeChange({ ...range_sensor, trigger_pin: parseInt(e.target.value) || 0 })}
                  />
                </div>
                <div>
                  <span className="text-[9px] text-zinc-500">Echo GPIO</span>
                  <input
                    className={inputCls}
                    type="number"
                    value={range_sensor.echo_pin}
                    onChange={e => onRangeChange({ ...range_sensor, echo_pin: parseInt(e.target.value) || 0 })}
                  />
                </div>
              </div>
            )}
          </div>
        </div>

        <Separator />

        {/* I2C / PCA9685 */}
        <div>
          <span className="text-[10px] uppercase text-muted-foreground tracking-wider block mb-1">
            I2C / PCA9685
          </span>
          <div className="grid grid-cols-2 gap-1.5">
            <div>
              <span className="text-[9px] text-zinc-500">Адрес PCA</span>
              <input
                className={inputCls}
                value={i2c.pca9685_address}
                onChange={e => onI2cChange({ ...i2c, pca9685_address: e.target.value })}
                placeholder="0x5F"
              />
            </div>
            <div>
              <span className="text-[9px] text-zinc-500">Частота (Hz)</span>
              <input
                className={inputCls}
                type="number"
                value={i2c.pca9685_frequency}
                onChange={e => onI2cChange({ ...i2c, pca9685_frequency: parseInt(e.target.value) || 50 })}
              />
            </div>
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
