/* global React */
const { useState, useEffect, useRef, useMemo, useCallback } = React;

// ============ utility ============
const cn = (...xs) => xs.filter(Boolean).join(' ');

// ============ Icons (lucide-style 1.5px monoline) ============
const Icon = ({ children, className = 'h-4 w-4', ...p }) => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5"
       strokeLinecap="round" strokeLinejoin="round" className={className} {...p}>{children}</svg>
);
const KatanaIcon = (p) => (
  <Icon {...p}>
    <path d="M3.5 20.5 L8 16" />
    <path d="M6.5 17.5 L8.5 19.5" />
    <path d="M9 15 L20.5 3.5" />
    <path d="M18.5 3.5 L20.5 3.5 L20.5 5.5" />
    <circle cx="7.5" cy="18.5" r=".4" fill="currentColor" />
  </Icon>
);
const DashIcon = (p)=><Icon {...p}><rect x="3" y="3" width="7" height="9"/><rect x="14" y="3" width="7" height="5"/><rect x="14" y="12" width="7" height="9"/><rect x="3" y="16" width="7" height="5"/></Icon>;
const SlidersIcon = (p)=><Icon {...p}><line x1="4" y1="21" x2="4" y2="14"/><line x1="4" y1="10" x2="4" y2="3"/><line x1="12" y1="21" x2="12" y2="12"/><line x1="12" y1="8" x2="12" y2="3"/><line x1="20" y1="21" x2="20" y2="16"/><line x1="20" y1="12" x2="20" y2="3"/><line x1="1" y1="14" x2="7" y2="14"/><line x1="9" y1="8" x2="15" y2="8"/><line x1="17" y1="16" x2="23" y2="16"/></Icon>;
const BoxIcon = (p)=><Icon {...p}><path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"/><polyline points="3.27 6.96 12 12.01 20.73 6.96"/><line x1="12" y1="22.08" x2="12" y2="12"/></Icon>;
const CpuIcon = (p)=><Icon {...p}><rect x="4" y="4" width="16" height="16" rx="2"/><rect x="9" y="9" width="6" height="6"/><line x1="9" y1="2" x2="9" y2="4"/><line x1="15" y1="2" x2="15" y2="4"/><line x1="9" y1="20" x2="9" y2="22"/><line x1="15" y1="20" x2="15" y2="22"/><line x1="20" y1="9" x2="22" y2="9"/><line x1="20" y1="15" x2="22" y2="15"/><line x1="2" y1="9" x2="4" y2="9"/><line x1="2" y1="15" x2="4" y2="15"/></Icon>;
const BotIcon = (p)=><Icon {...p}><rect x="3" y="11" width="18" height="10" rx="2"/><circle cx="12" cy="5" r="2"/><path d="M12 7v4"/><line x1="8" y1="16" x2="8" y2="16"/><line x1="16" y1="16" x2="16" y2="16"/></Icon>;
const SunIcon = (p)=><Icon {...p}><circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M4.93 4.93l1.41 1.41M17.66 17.66l1.41 1.41M2 12h2M20 12h2M4.93 19.07l1.41-1.41M17.66 6.34l1.41-1.41"/></Icon>;
const MoonIcon = (p)=><Icon {...p}><path d="M21 12.79A9 9 0 1 1 11.21 3 7 7 0 0 0 21 12.79z"/></Icon>;
const PanelLeftIcon = (p)=><Icon {...p}><rect x="3" y="3" width="18" height="18" rx="2"/><line x1="9" y1="3" x2="9" y2="21"/></Icon>;
const PlayIcon = (p)=><Icon {...p}><polygon points="5 3 19 12 5 21 5 3"/></Icon>;
const PauseIcon = (p)=><Icon {...p}><rect x="6" y="4" width="4" height="16"/><rect x="14" y="4" width="4" height="16"/></Icon>;
const StopIcon = (p)=><Icon {...p}><rect x="5" y="5" width="14" height="14" rx="1"/></Icon>;
const RotateIcon = (p)=><Icon {...p}><polyline points="1 4 1 10 7 10"/><path d="M3.51 15a9 9 0 1 0 2.13-9.36L1 10"/></Icon>;
const AlertIcon = (p)=><Icon {...p}><path d="M10.29 3.86 1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><circle cx="12" cy="17" r=".4" fill="currentColor"/></Icon>;
const WifiIcon = (p)=><Icon {...p}><path d="M5 12.55a11 11 0 0 1 14.08 0"/><path d="M1.42 9a16 16 0 0 1 21.16 0"/><path d="M8.53 16.11a6 6 0 0 1 6.95 0"/><circle cx="12" cy="20" r=".4" fill="currentColor"/></Icon>;
const BatteryIcon = (p)=><Icon {...p}><rect x="2" y="7" width="16" height="10" rx="2"/><line x1="22" y1="11" x2="22" y2="13"/></Icon>;
const ThermoIcon = (p)=><Icon {...p}><path d="M14 14.76V3.5a2.5 2.5 0 0 0-5 0v11.26a4 4 0 1 0 5 0z"/></Icon>;
const TerminalIcon = (p)=><Icon {...p}><polyline points="4 17 10 11 4 5"/><line x1="12" y1="19" x2="20" y2="19"/></Icon>;
const GridIcon = (p)=><Icon {...p}><rect x="3" y="3" width="7" height="7"/><rect x="14" y="3" width="7" height="7"/><rect x="14" y="14" width="7" height="7"/><rect x="3" y="14" width="7" height="7"/></Icon>;
const LayersIcon = (p)=><Icon {...p}><polygon points="12 2 2 7 12 12 22 7 12 2"/><polyline points="2 17 12 22 22 17"/><polyline points="2 12 12 17 22 12"/></Icon>;
const TargetIcon = (p)=><Icon {...p}><circle cx="12" cy="12" r="10"/><circle cx="12" cy="12" r="6"/><circle cx="12" cy="12" r="2" fill="currentColor"/></Icon>;
const ZapIcon = (p)=><Icon {...p}><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></Icon>;
const PlusIcon = (p)=><Icon {...p}><line x1="12" y1="5" x2="12" y2="19"/><line x1="5" y1="12" x2="19" y2="12"/></Icon>;
const MinusIcon = (p)=><Icon {...p}><line x1="5" y1="12" x2="19" y2="12"/></Icon>;
const CheckIcon = (p)=><Icon {...p}><polyline points="20 6 9 17 4 12"/></Icon>;
const XIcon = (p)=><Icon {...p}><line x1="18" y1="6" x2="6" y2="18"/><line x1="6" y1="6" x2="18" y2="18"/></Icon>;
const RecordIcon = (p)=><Icon {...p}><circle cx="12" cy="12" r="6" fill="currentColor"/></Icon>;

// ============ shadcn-ish primitives ============
const Card = ({ className, children }) => (
  <div className={cn('rounded-lg border border-subtle bg-surface-1 transition-colors duration-standard', className)}>{children}</div>
);
const CardHeader = ({ className, children, right }) => (
  <div className={cn('flex items-center justify-between gap-2 px-4 py-3 border-b border-subtle', className)}>
    <div className="flex items-center gap-2 min-w-0">{children}</div>
    {right ? <div className="flex items-center gap-1.5">{right}</div> : null}
  </div>
);
const CardTitle = ({ children }) => (
  <h2 className="text-h2 text-foreground truncate">{children}</h2>
);
const CardSubtitle = ({ children }) => (
  <span className="font-mono text-micro uppercase tracking-wider text-foreground-faint">{children}</span>
);
const CardContent = ({ className, children }) => (
  <div className={cn('p-4', className)}>{children}</div>
);

const Button = ({ variant='default', size='default', className, children, ...p }) => {
  const v = {
    default:    'bg-accent text-accent-foreground hover:bg-accent-hover active:bg-accent-active active:scale-[.98]',
    secondary:  'bg-surface-2 text-foreground hover:bg-surface-3 border border-subtle',
    ghost:      'bg-transparent text-foreground-muted hover:bg-surface-2 hover:text-foreground',
    outline:    'border border-strong bg-transparent hover:bg-surface-2',
    destructive:'bg-danger/90 text-foreground hover:bg-danger active:scale-[.98]',
    link:       'text-accent underline-offset-4 hover:underline px-0',
  }[variant];
  const s = {
    sm:'h-8 px-2.5 text-small', default:'h-9 px-3 text-body', lg:'h-11 px-4 text-body', icon:'h-9 w-9',
  }[size];
  return (
    <button {...p}
      className={cn('inline-flex items-center justify-center gap-1.5 rounded-md font-medium',
        'transition-all duration-fast ease-standard',
        'focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-accent/40 focus-visible:ring-offset-2 focus-visible:ring-offset-background',
        'disabled:opacity-50 disabled:pointer-events-none',
        v, s, className)}>
      {children}
    </button>
  );
};

const Badge = ({ tone='default', dot=false, className, children }) => {
  const styles = {
    default: 'bg-surface-2 text-foreground-muted border-subtle',
    accent:  'bg-accent/10 text-accent border-accent/20',
    success: 'bg-success/10 text-success border-success/25',
    warning: 'bg-warning/10 text-warning border-warning/25',
    danger:  'bg-danger/10  text-danger  border-danger/25',
    info:    'bg-info/10    text-info    border-info/25',
  }[tone];
  return (
    <span className={cn('inline-flex items-center gap-1.5 rounded-pill border px-2 py-0.5',
      'font-mono text-micro uppercase tracking-wider', styles, className)}>
      {dot ? <span className="h-1.5 w-1.5 rounded-full bg-current"/> : null}
      {children}
    </span>
  );
};

const Kbd = ({ children, className }) => (
  <kbd className={cn('inline-flex items-center justify-center min-w-[1.5rem] h-5 px-1.5 rounded',
    'border border-subtle bg-surface-2 font-mono text-micro text-foreground-muted', className)}>{children}</kbd>
);

const Tooltip = ({ label, side='right', children }) => (
  <span className="group relative inline-flex">
    {children}
    <span className={cn('pointer-events-none absolute z-50 whitespace-nowrap rounded-md',
      'border border-subtle bg-surface-3 px-2 py-1 text-small text-foreground shadow-lg',
      'opacity-0 group-hover:opacity-100 transition-opacity duration-fast delay-200',
      side==='right' && 'left-full ml-2 top-1/2 -translate-y-1/2',
      side==='top'   && 'bottom-full mb-2 left-1/2 -translate-x-1/2',
    )}>{label}</span>
  </span>
);

// ============ FSM ============
const FSM_STATES = ['IDLE','SEARCHING','TARGETING','APPROACHING','GRABBING','CALLING','RETURNING'];
const FSM_SHORT = { IDLE:'IDL', SEARCHING:'SRCH', TARGETING:'TGT', APPROACHING:'APR', GRABBING:'GRB', CALLING:'CAL', RETURNING:'RTN' };
const FSM_HEX = {
  IDLE: '#8A8278', SEARCHING: '#7DA1C9', TARGETING: '#CC785C',
  APPROACHING: '#C9A26B', GRABBING: '#B894C9', CALLING: '#7DA88A', RETURNING: '#9A9089',
};

const FsmBadge = ({ state, target, compact }) => (
  <div className={cn('inline-flex items-center gap-2 rounded-pill border border-subtle bg-surface-2',
    compact ? 'px-2 py-1' : 'px-3 py-1.5')}>
    <span className={cn('h-2 w-2 rounded-full', state!=='IDLE' && 'animate-pulse-soft')}
          style={{ backgroundColor: FSM_HEX[state] }}/>
    <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">{state}</span>
    {target && (
      <span className="font-mono text-micro text-foreground-faint">→ {target}</span>
    )}
  </div>
);

const FsmTimeline = ({ current }) => {
  const idx = FSM_STATES.indexOf(current);
  return (
    <div className="flex items-end gap-1.5">
      {FSM_STATES.map((s, i) => {
        const isActive = i === idx;
        const isPast = i < idx;
        return (
          <div key={s} className="flex flex-col items-center gap-1.5 flex-1 min-w-0">
            <div className={cn('h-1 w-full rounded-full transition-colors duration-standard',
              isActive ? 'bg-accent' : isPast ? 'bg-foreground-muted/60' : 'bg-surface-3')}/>
            <span className={cn('font-mono text-micro tracking-wider',
              isActive ? 'text-accent' : 'text-foreground-faint')}>{FSM_SHORT[s]}</span>
          </div>
        );
      })}
    </div>
  );
};

// ============ Layout ============
const NAV_ITEMS = [
  { id:'dashboard', label:'Dashboard', Icon: DashIcon },
  { id:'admin',     label:'Admin',     Icon: SlidersIcon },
  { id:'3d',        label:'3D View',   Icon: BoxIcon },
  { id:'hardware',  label:'Hardware',  Icon: CpuIcon },
  { id:'samcan',    label:'Samcan',    Icon: BotIcon },
];

const Sidebar = ({ active, setActive, collapsed, setCollapsed, robot, setRobot, theme, setTheme, connected }) => (
  <aside className={cn('shrink-0 border-r border-subtle bg-surface-1 transition-[width] duration-standard ease-standard',
    'flex flex-col h-screen sticky top-0 z-30', collapsed ? 'w-[56px]' : 'w-[220px]')}>
    {/* Brand */}
    <div className={cn('flex items-center gap-2.5 h-12 px-3 border-b border-subtle')}>
      <KatanaIcon className="h-5 w-5 text-accent shrink-0"/>
      {!collapsed && <span className="font-semibold tracking-tight">Samurai</span>}
    </div>

    {/* Robot selector */}
    <div className="px-2 py-2 border-b border-subtle">
      {!collapsed ? (
        <button onClick={()=>setRobot(robot==='samurai'?'samcan':'samurai')}
          className="w-full flex items-center justify-between gap-2 rounded-md border border-subtle bg-surface-2 hover:bg-surface-3 px-2.5 py-1.5 transition-colors">
          <span className="flex items-center gap-2 min-w-0">
            <span className={cn('h-1.5 w-1.5 rounded-full', connected?'bg-success animate-pulse-soft':'bg-danger')}/>
            <span className="font-mono text-small truncate">{robot}</span>
          </span>
          <span className="font-mono text-micro text-foreground-faint">↕</span>
        </button>
      ) : (
        <Tooltip label={`Robot · ${robot}`} side="right">
          <span className={cn('mx-auto block h-1.5 w-1.5 rounded-full mt-1', connected?'bg-success animate-pulse-soft':'bg-danger')}/>
        </Tooltip>
      )}
    </div>

    {/* Nav */}
    <nav className="flex-1 px-2 py-2 space-y-0.5">
      {NAV_ITEMS.map(({id,label,Icon: I}) => {
        const isA = active===id;
        const btn = (
          <button key={id} onClick={()=>setActive(id)}
            className={cn('relative w-full flex items-center gap-2.5 h-9 rounded-md transition-colors',
              collapsed ? 'justify-center px-0' : 'px-2.5',
              isA ? 'bg-surface-2 text-accent' : 'text-foreground-muted hover:bg-surface-2/60 hover:text-foreground')}>
            {isA && <span className="absolute left-0 top-1.5 bottom-1.5 w-[2px] rounded-full bg-accent"/>}
            <I className="h-[18px] w-[18px] shrink-0"/>
            {!collapsed && <span className="text-body">{label}</span>}
          </button>
        );
        return collapsed ? <Tooltip key={id} label={label}>{btn}</Tooltip> : btn;
      })}
    </nav>

    {/* Footer */}
    <div className="border-t border-subtle px-2 py-2 space-y-1">
      <button onClick={()=>setTheme(theme==='dark'?'light':'dark')}
        className={cn('w-full flex items-center gap-2.5 h-9 rounded-md text-foreground-muted hover:bg-surface-2/60 hover:text-foreground transition-colors',
          collapsed?'justify-center px-0':'px-2.5')}>
        {theme==='dark' ? <MoonIcon className="h-[18px] w-[18px]"/> : <SunIcon className="h-[18px] w-[18px]"/>}
        {!collapsed && <span className="text-body capitalize">{theme}</span>}
      </button>
      <div className={cn('flex items-center gap-2.5 h-9 rounded-md text-foreground-muted',
          collapsed?'justify-center px-0':'px-2.5')}>
        <span className={cn('h-1.5 w-1.5 rounded-full', connected?'bg-success animate-pulse-soft':'bg-danger')}/>
        {!collapsed && <span className="font-mono text-micro uppercase tracking-wider">{connected?'connected':'offline'}</span>}
      </div>
      <button onClick={()=>setCollapsed(!collapsed)}
        className={cn('w-full flex items-center gap-2.5 h-9 rounded-md text-foreground-muted hover:bg-surface-2/60 hover:text-foreground transition-colors',
          collapsed?'justify-center px-0':'px-2.5')}>
        <PanelLeftIcon className={cn('h-[18px] w-[18px] transition-transform', collapsed && 'rotate-180')}/>
        {!collapsed && <span className="text-body">Collapse</span>}
      </button>
    </div>
  </aside>
);

const PageHeader = ({ title, simTime, right }) => (
  <header className="sticky top-0 z-20 h-12 border-b border-subtle bg-surface-1/95 backdrop-blur-sm">
    <div className="h-full px-6 flex items-center justify-between">
      <h1 className="text-display">{title}</h1>
      <div className="flex items-center gap-3">
        <span className="font-mono text-small text-foreground-muted tabular-nums">{simTime}</span>
        {right}
        <Button variant="secondary" size="sm"><TerminalIcon className="h-3.5 w-3.5"/>Debug</Button>
      </div>
    </div>
  </header>
);

// expose
Object.assign(window, {
  cn, FSM_STATES, FSM_SHORT, FSM_HEX,
  Card, CardHeader, CardTitle, CardSubtitle, CardContent,
  Button, Badge, Kbd, Tooltip, Icon,
  KatanaIcon, DashIcon, SlidersIcon, BoxIcon, CpuIcon, BotIcon, SunIcon, MoonIcon,
  PanelLeftIcon, PlayIcon, PauseIcon, StopIcon, RotateIcon, AlertIcon, WifiIcon,
  BatteryIcon, ThermoIcon, TerminalIcon, GridIcon, LayersIcon, TargetIcon, ZapIcon,
  PlusIcon, MinusIcon, CheckIcon, XIcon, RecordIcon,
  FsmBadge, FsmTimeline, Sidebar, PageHeader,
});
