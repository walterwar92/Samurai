import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import './index.css'
import App from './App'
import { initSentry } from '@/lib/sentry'

document.documentElement.classList.add('dark')

// Fire-and-forget — bails immediately if VITE_SENTRY_DSN is unset.
initSentry()

createRoot(document.getElementById('root')!).render(
  <StrictMode>
    <App />
  </StrictMode>,
)
