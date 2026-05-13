import type { Config } from 'tailwindcss'

export default {
  content: ['./index.html', './src/**/*.{ts,tsx}'],
  darkMode: ['class', '.light'],
  theme: {
    extend: {
      colors: {
        background: 'hsl(var(--background))',
        foreground: 'hsl(var(--foreground))',
        'foreground-muted': 'hsl(var(--foreground-muted))',
        'foreground-faint': 'hsl(var(--foreground-faint))',
        surface: {
          1: 'hsl(var(--surface-1))',
          2: 'hsl(var(--surface-2))',
          3: 'hsl(var(--surface-3))',
        },
        border: {
          DEFAULT: 'hsl(var(--border-subtle))',
          subtle: 'hsl(var(--border-subtle))',
          strong: 'hsl(var(--border-strong))',
        },
        accent: {
          DEFAULT: 'hsl(var(--accent))',
          hover: 'hsl(var(--accent-hover))',
          active: 'hsl(var(--accent-active))',
          foreground: 'hsl(var(--accent-foreground))',
        },
        success: 'hsl(var(--success))',
        warning: 'hsl(var(--warning))',
        danger: 'hsl(var(--danger))',
        info: 'hsl(var(--info))',
        fsm: {
          idle: '#8A8278',
          searching: '#7DA1C9',
          approaching: '#C9A26B',
          grabbing: '#B894C9',
          calling: '#7DA88A',
          returning: '#9A9089',
        },
        ball: {
          red: '#B85C5C',
          blue: '#5C7DC9',
          green: '#6BA86B',
          yellow: '#C9B05C',
          orange: '#C9885C',
          white: '#D8D2C8',
          black: '#2A2826',
          unknown: '#9e9e9e', // legacy
        },
        // shadcn-aliases (для существующих компонентов, читают те же CSS-vars)
        card: {
          DEFAULT: 'hsl(var(--card))',
          foreground: 'hsl(var(--card-foreground))',
        },
        popover: {
          DEFAULT: 'hsl(var(--popover))',
          foreground: 'hsl(var(--popover-foreground))',
        },
        primary: {
          DEFAULT: 'hsl(var(--primary))',
          foreground: 'hsl(var(--primary-foreground))',
        },
        secondary: {
          DEFAULT: 'hsl(var(--secondary))',
          foreground: 'hsl(var(--secondary-foreground))',
        },
        muted: {
          DEFAULT: 'hsl(var(--muted))',
          foreground: 'hsl(var(--muted-foreground))',
        },
        destructive: {
          DEFAULT: 'hsl(var(--destructive))',
          foreground: 'hsl(var(--destructive-foreground))',
        },
        input: 'hsl(var(--input))',
        ring: 'hsl(var(--ring))',
        // Legacy `samurai` palette (старые компоненты до редизайна)
        samurai: {
          green: '#66bb6a',
          red: '#ef5350',
          orange: '#ffa726',
          yellow: '#ffee58',
          purple: '#ab47bc',
          accent: '#4fc3f7',
        },
      },
      fontFamily: {
        sans: ['Inter', 'Segoe UI', 'Ubuntu', 'sans-serif'],
        mono: ['"JetBrains Mono"', 'ui-monospace', 'Consolas', 'monospace'],
      },
      fontSize: {
        display: ['1.5rem', { lineHeight: '1.75rem', letterSpacing: '-0.01em', fontWeight: '600' }],
        h1: ['1.125rem', { lineHeight: '1.5rem', letterSpacing: '-0.005em', fontWeight: '600' }],
        h2: ['1rem', { lineHeight: '1.375rem', letterSpacing: '-0.003em', fontWeight: '500' }],
        body: ['0.875rem', { lineHeight: '1.25rem' }],
        small: ['0.75rem', { lineHeight: '1rem' }],
        micro: ['0.625rem', { lineHeight: '0.875rem', letterSpacing: '0.06em', fontWeight: '500' }],
      },
      borderRadius: {
        sm: '6px',
        DEFAULT: '8px',
        md: '8px',
        lg: '12px',
        xl: '16px',
        pill: '9999px',
      },
      transitionTimingFunction: {
        standard: 'cubic-bezier(0.4, 0, 0.2, 1)',
        spring: 'cubic-bezier(0.34, 1.56, 0.64, 1)',
        swift: 'cubic-bezier(0.4, 0, 1, 1)',
      },
      transitionDuration: {
        fast: '120ms',
        standard: '200ms',
        slow: '320ms',
      },
      animation: {
        'pulse-soft': 'pulse-soft 1.6s ease-in-out infinite',
        scanline: 'scanline 4s linear infinite',
        'fade-in-up': 'fade-in-up 200ms cubic-bezier(0.34, 1.56, 0.64, 1)',
        blink: 'blink 1s steps(2) infinite',
      },
    },
  },
  plugins: [],
} satisfies Config
