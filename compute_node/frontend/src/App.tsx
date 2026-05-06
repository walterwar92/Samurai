//Вар Волтер 228
import { useEffect, lazy, Suspense } from 'react'
import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom'
import { RobotProvider } from '@/providers/RobotProvider'
import { useRobotStore } from '@/stores/robotStore'
// DashboardPage is the landing page — eager so first paint isn't behind a chunk.
import { DashboardPage } from '@/pages/DashboardPage'

// Code-split the rest. Visualization3DPage in particular pulls in three.js +
// @react-three/fiber + @react-three/drei (~800 KB minified) which would
// otherwise be downloaded by every visitor regardless of whether they ever
// open /3d. The other pages are lazy-loaded for the same reason: they each
// pull a sizable component graph that the dashboard route never needs.
const AdminPage = lazy(() => import('@/pages/AdminPage').then(m => ({ default: m.AdminPage })))
const Visualization3DPage = lazy(() =>
  import('@/pages/Visualization3DPage').then(m => ({ default: m.Visualization3DPage })))
const HardwarePage = lazy(() => import('@/pages/HardwarePage').then(m => ({ default: m.HardwarePage })))
const SamcanPage = lazy(() => import('@/pages/SamcanPage').then(m => ({ default: m.SamcanPage })))
// /mps — учебный модуль курсовой Козлова (feat/mps). Lazy: страница тянет
// 5 mps/* компонентов и Recharts-графики, не нужно на главной.
const MpsPage = lazy(() => import('@/pages/MpsPage').then(m => ({ default: m.MpsPage })))

function PageLoadingFallback() {
  return (
    <div className="flex items-center justify-center min-h-[60vh] text-muted-foreground text-sm">
      Загрузка…
    </div>
  )
}

export default function App() {
  const connect = useRobotStore((s) => s.connect)
  const disconnect = useRobotStore((s) => s.disconnect)

  // Один SocketIO коннект на жизнь приложения. Идемпотентно для StrictMode.
  useEffect(() => {
    connect()
    return () => disconnect()
  }, [connect, disconnect])

  return (
    <RobotProvider>
      <BrowserRouter>
        <Suspense fallback={<PageLoadingFallback />}>
          <Routes>
            <Route path="/" element={<Navigate to="/dashboard" replace />} />
            <Route path="/dashboard" element={<DashboardPage />} />
            <Route path="/admin" element={<AdminPage />} />
            <Route path="/3d" element={<Visualization3DPage />} />
            <Route path="/hardware" element={<HardwarePage />} />
            <Route path="/samcan" element={<SamcanPage />} />
            <Route path="/mps" element={<MpsPage />} />
          </Routes>
        </Suspense>
      </BrowserRouter>
    </RobotProvider>
  )
}
