import { createContext, useContext, useEffect, useState, type ReactNode } from 'react'

export type RobotId = 'samurai' | 'samcan'

export interface RobotInfo {
  id: RobotId
  name: string
  controller: string
  online: boolean
  capabilities: {
    camera: boolean
    wifi: boolean
    yolo: boolean
    voice: boolean
    slam: boolean
    claw: boolean
    ultrasonic: boolean
    imu: boolean
  }
}

export const ROBOTS: Record<RobotId, RobotInfo> = {
  samurai: {
    id: 'samurai',
    name: 'Samurai',
    controller: 'Raspberry Pi 4',
    online: true,
    capabilities: {
      camera: true,
      wifi: true,
      yolo: true,
      voice: true,
      slam: true,
      claw: true,
      ultrasonic: true,
      imu: true,
    },
  },
  samcan: {
    id: 'samcan',
    name: 'Samcan',
    controller: 'Arduino Uno',
    online: false,
    capabilities: {
      camera: false,
      wifi: false,
      yolo: false,
      voice: false,
      slam: false,
      claw: true,
      ultrasonic: true,
      imu: true,
    },
  },
}

interface RobotContextValue {
  activeRobot: RobotId
  robot: RobotInfo
  setActiveRobot: (id: RobotId) => void
}

const RobotContext = createContext<RobotContextValue>({
  activeRobot: 'samurai',
  robot: ROBOTS.samurai,
  setActiveRobot: () => {},
})

const STORAGE_KEY = 'samurai.activeRobot'

export function RobotProvider({ children }: { children: ReactNode }) {
  const [activeRobot, setActiveRobotState] = useState<RobotId>(() => {
    const stored = localStorage.getItem(STORAGE_KEY)
    return stored === 'samcan' ? 'samcan' : 'samurai'
  })

  useEffect(() => {
    localStorage.setItem(STORAGE_KEY, activeRobot)
  }, [activeRobot])

  const setActiveRobot = (id: RobotId) => setActiveRobotState(id)

  return (
    <RobotContext.Provider value={{ activeRobot, robot: ROBOTS[activeRobot], setActiveRobot }}>
      {children}
    </RobotContext.Provider>
  )
}

export function useRobot() {
  return useContext(RobotContext)
}
