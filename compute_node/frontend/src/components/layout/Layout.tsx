import { Outlet } from 'react-router-dom'
import { Sidebar } from './Sidebar'

/**
 * Корневой layout: Sidebar слева + main-area (`<Outlet />`) справа.
 * Используется как родительский route в App.tsx, чтобы Sidebar монтировался
 * один раз и переключение страниц не передёргивало его state.
 */
export function Layout() {
  return (
    <div className="flex min-h-screen bg-background text-foreground">
      <Sidebar />
      <div className="flex-1 min-w-0 flex flex-col">
        <Outlet />
      </div>
    </div>
  )
}
