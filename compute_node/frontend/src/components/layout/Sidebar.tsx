import { useEffect, useState } from 'react'
import { Link, useLocation } from 'react-router-dom'
import { cn } from '@/lib/utils'
import {
  KatanaIcon,
  DashIcon,
  SlidersIcon,
  BoxIcon,
  CpuIcon,
  BotIcon,
  SunIcon,
  MoonIcon,
  PanelLeftClose,
  PanelLeftOpen,
} from '@/components/icons'
import { useTheme } from './useTheme'
import { useRobotStore } from '@/stores/robotStore'

const NAV = [
  { to: '/dashboard', label: 'Dashboard', Icon: DashIcon },
  { to: '/admin', label: 'Admin', Icon: SlidersIcon },
  { to: '/3d', label: '3D View', Icon: BoxIcon },
  { to: '/hardware', label: 'Hardware', Icon: CpuIcon },
  { to: '/samcan', label: 'Samcan', Icon: BotIcon },
  { to: '/mps', label: 'MPS', Icon: SlidersIcon },
] as const

const STORAGE_KEY = 'samurai.sidebarCollapsed'

/**
 * Sidebar — основная навигация. Складная (220 → 56px),
 * состояние в localStorage. ThemeToggle + индикатор соединения внизу.
 * Активный маршрут подсвечен 2px coral-полоской слева.
 */
export function Sidebar() {
  const location = useLocation()
  const [collapsed, setCollapsed] = useState(
    () => localStorage.getItem(STORAGE_KEY) === '1',
  )
  const { theme, toggle: toggleTheme } = useTheme()
  const connected = useRobotStore((s) => s.connected)

  useEffect(() => {
    localStorage.setItem(STORAGE_KEY, collapsed ? '1' : '0')
  }, [collapsed])

  return (
    <aside
      className={cn(
        'shrink-0 border-r border-subtle bg-surface-1',
        'transition-[width] duration-standard ease-standard',
        'flex flex-col h-screen sticky top-0 z-30',
        collapsed ? 'w-[56px]' : 'w-[220px]',
      )}
    >
      {/* Brand */}
      <Link
        to="/dashboard"
        className="flex items-center gap-2.5 h-12 px-3 border-b border-subtle hover:bg-surface-2/40 transition-colors"
      >
        <KatanaIcon className="h-5 w-5 text-accent shrink-0" />
        {!collapsed && <span className="font-semibold tracking-tight">Samurai</span>}
      </Link>

      {/* Connection status */}
      <div
        className={cn(
          'flex items-center gap-2 h-8 px-3 border-b border-subtle',
          collapsed && 'justify-center',
        )}
      >
        <span
          className={cn(
            'h-1.5 w-1.5 rounded-full',
            connected ? 'bg-success animate-pulse-soft' : 'bg-danger',
          )}
        />
        {!collapsed && (
          <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">
            {connected ? 'connected' : 'offline'}
          </span>
        )}
      </div>

      {/* Nav */}
      <nav className="flex-1 px-2 py-2 space-y-0.5">
        {NAV.map(({ to, label, Icon: I }) => {
          const isActive = location.pathname.startsWith(to)
          return (
            <Link
              key={to}
              to={to}
              title={collapsed ? label : undefined}
              className={cn(
                'relative w-full flex items-center gap-2.5 h-9 rounded-md transition-colors',
                collapsed ? 'justify-center px-0' : 'px-2.5',
                isActive
                  ? 'bg-surface-2 text-accent'
                  : 'text-foreground-muted hover:bg-surface-2/60 hover:text-foreground',
              )}
            >
              {isActive && (
                <span className="absolute left-0 top-1.5 bottom-1.5 w-[2px] rounded-full bg-accent" />
              )}
              <I className="h-[18px] w-[18px] shrink-0" />
              {!collapsed && <span className="text-body">{label}</span>}
            </Link>
          )
        })}
      </nav>

      {/* Footer */}
      <div className="border-t border-subtle px-2 py-2 space-y-1">
        <button
          onClick={toggleTheme}
          title={collapsed ? `Theme: ${theme}` : undefined}
          className={cn(
            'w-full flex items-center gap-2.5 h-9 rounded-md text-foreground-muted',
            'hover:bg-surface-2/60 hover:text-foreground transition-colors',
            collapsed ? 'justify-center px-0' : 'px-2.5',
          )}
        >
          {theme === 'dark' ? (
            <MoonIcon className="h-[18px] w-[18px]" />
          ) : (
            <SunIcon className="h-[18px] w-[18px]" />
          )}
          {!collapsed && <span className="text-body capitalize">{theme}</span>}
        </button>
        <button
          onClick={() => setCollapsed((c) => !c)}
          aria-label="Collapse sidebar"
          className={cn(
            'w-full flex items-center gap-2.5 h-9 rounded-md text-foreground-muted',
            'hover:bg-surface-2/60 hover:text-foreground transition-colors',
            collapsed ? 'justify-center px-0' : 'px-2.5',
          )}
        >
          {collapsed ? (
            <PanelLeftOpen className="h-[18px] w-[18px]" />
          ) : (
            <PanelLeftClose className="h-[18px] w-[18px]" />
          )}
          {!collapsed && <span className="text-body">Collapse</span>}
        </button>
      </div>
    </aside>
  )
}
