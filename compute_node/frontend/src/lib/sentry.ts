/**
 * Frontend Sentry wiring (#73) — opt-in.
 *
 * Default: NO-OP. The SDK is not bundled unless installed. To enable
 * error reporting in production:
 *
 *   1. cd compute_node/frontend && npm install @sentry/react
 *   2. Build with VITE_SENTRY_DSN=https://...@sentry.io/123 npm run build
 *      (or set the var on the dashboard host that serves index.html)
 *   3. Optionally: VITE_SENTRY_ENV=prod (default 'dev'),
 *                  VITE_SENTRY_TRACES_RATE=0.1 (default 0).
 *
 * This file uses dynamic import() so the SDK is tree-shaken out when
 * VITE_SENTRY_DSN is empty — zero bundle cost when disabled, lazy
 * download when enabled. Mirrors the backend sentry-sdk skeleton in
 * compute_node/dashboard/app.py :: _maybe_init_sentry.
 */

interface SentryInitOpts {
  dsn?: string
  env?: string
  tracesSampleRate?: number
}

// Minimal shape we use from @sentry/react. Defining it locally lets us avoid
// taking a hard dependency on the SDK's type declarations — TypeScript would
// otherwise refuse to compile when the SDK isn't installed (the default), and
// CI doesn't install it. Runtime behaviour stays identical: when the dynamic
// import succeeds, we get the real module that satisfies this interface.
interface SentryModule {
  init(opts: {
    dsn: string
    environment?: string
    tracesSampleRate?: number
    sendDefaultPii?: boolean
  }): void
}

export async function initSentry(opts: SentryInitOpts = {}): Promise<void> {
  const dsn = opts.dsn ?? import.meta.env.VITE_SENTRY_DSN
  if (!dsn) return

  let mod: SentryModule | undefined
  try {
    // Dynamic import — bundler can split this into a chunk that's only
    // fetched when VITE_SENTRY_DSN is non-empty at build time. Wrapped in
    // try/catch so a missing dep at runtime degrades to "no Sentry"
    // instead of a hard crash. The module name is built at runtime via a
    // string variable so neither tsc nor vite tries to resolve it at build
    // time when the package is absent.
    const sentryPkg = '@sentry/react'
    mod = (await import(/* @vite-ignore */ sentryPkg)) as SentryModule
  } catch {
    // SDK not installed; quietly do nothing.
    // eslint-disable-next-line no-console
    console.info('[sentry] @sentry/react not installed — error reporting disabled')
    return
  }

  const env = opts.env ?? import.meta.env.VITE_SENTRY_ENV ?? 'dev'
  const tracesSampleRate =
    opts.tracesSampleRate ??
    (Number(import.meta.env.VITE_SENTRY_TRACES_RATE) || 0)

  mod.init({
    dsn,
    environment: env,
    tracesSampleRate,
    // Don't accidentally exfiltrate PII (voice transcripts, robot IPs,
    // MQTT payloads in error breadcrumbs).
    sendDefaultPii: false,
  })
}
