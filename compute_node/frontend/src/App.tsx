import { BrowserRouter, Routes, Route, Navigate } from 'react-router-dom'
import { SocketProvider } from '@/providers/SocketProvider'
import { DashboardPage } from '@/pages/DashboardPage'
import { AdminPage } from '@/pages/AdminPage'
import { Visualization3DPage } from '@/pages/Visualization3DPage'

export default function App() {
  return (
    <SocketProvider>
      <BrowserRouter>
        <Routes>
          <Route path="/" element={<Navigate to="/dashboard" replace />} />
          <Route path="/dashboard" element={<DashboardPage />} />
          <Route path="/admin" element={<AdminPage />} />
          <Route path="/3d" element={<Visualization3DPage />} />
        </Routes>
      </BrowserRouter>
    </SocketProvider>
  )
}
