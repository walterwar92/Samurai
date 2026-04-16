export type Platform = 'raspberry_pi_pca9685' | 'arduino' | 'esp32' | 'custom'
export type MotorDriver = 'pca9685' | 'direct_gpio' | 'custom'
export type ServoDriver = 'pca9685' | 'direct_gpio' | 'custom'
export type ImuType = 'mpu6050' | 'bno055' | 'none'
export type CameraType = 'usb' | 'csi' | 'ip' | 'none'
export type RangeType = 'hc_sr04' | 'vl53l0x' | 'none'
export type LedType = 'ws2812b' | 'apa102' | 'none'

export interface MotorChannel {
  in1: number
  in2: number
  label: string
}

export interface ServoHeadConfig {
  channel: number
  home: number
  min: number
  max: number
}

export interface ServoArmConfig {
  channels: number[]
  home_angles: number[]
  min_angles: number[]
  max_angles: number[]
  labels: string[]
}

export interface BlockPosition {
  x: number
  y: number
}

export type LayoutMap = Record<string, BlockPosition>

export interface HardwarePreset {
  name: string
  platform: Platform
  description: string
  created: string
  modified: string

  motors: {
    driver: MotorDriver
    count: number
    channels: Record<string, MotorChannel>
  }

  servos: {
    driver: ServoDriver
    head: ServoHeadConfig
    arm: ServoArmConfig
  }

  imu: {
    type: ImuType
    address: string
  }

  camera: {
    type: CameraType
    device: string
    width: number
    height: number
  }

  range_sensor: {
    type: RangeType
    trigger_pin: number
    echo_pin: number
  }

  leds: {
    type: LedType
    gpio_pin: number
    count: number
    brightness: number
  }

  i2c: {
    pca9685_address: string
    pca9685_frequency: number
  }

  /** Optional block positions on the visual canvas (by block id) */
  layout?: LayoutMap
}

export interface PresetSummary {
  name: string
  platform: Platform
  description: string
  modified: string
}

/** Empty preset template for creating new configs */
export function emptyPreset(): HardwarePreset {
  return {
    name: '',
    platform: 'raspberry_pi_pca9685',
    description: '',
    created: '',
    modified: '',
    motors: {
      driver: 'pca9685',
      count: 2,
      channels: {
        M1: { in1: 11, in2: 10, label: 'Левый-задний' },
        M2: { in1: 8,  in2: 9,  label: 'Правый-задний' },
      },
    },
    servos: {
      driver: 'pca9685',
      head: { channel: 0, home: 90, min: 0, max: 180 },
      arm: {
        channels: [0, 0, 0, 0],
        home_angles: [0, 0, 0, 0],
        min_angles: [0, 0, 0, 0],
        max_angles: [180, 180, 180, 180],
        labels: ['Сустав 1', 'Сустав 2', 'Сустав 3', 'Сустав 4'],
      },
    },
    imu: { type: 'mpu6050', address: '0x68' },
    camera: { type: 'usb', device: '/dev/video0', width: 640, height: 480 },
    range_sensor: { type: 'hc_sr04', trigger_pin: 23, echo_pin: 24 },
    leds: { type: 'ws2812b', gpio_pin: 10, count: 12, brightness: 0.3 },
    i2c: { pca9685_address: '0x5F', pca9685_frequency: 50 },
    layout: {},
  }
}
