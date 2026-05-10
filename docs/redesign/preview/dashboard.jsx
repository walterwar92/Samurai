/* global React */
const { useState: useStateD, useEffect: useEffectD, useRef: useRefD, useMemo: useMemoD } = React;

// =========== Camera with HUD + scanline ============
const CameraFeed = ({ fsm, target, connected, simTime }) => {
  const detections = [
    { id:1, x:0.18, y:0.42, w:0.12, h:0.18, color:'red',    conf:0.92, active:true  },
    { id:2, x:0.55, y:0.55, w:0.08, h:0.12, color:'blue',   conf:0.71, active:false },
    { id:3, x:0.74, y:0.30, w:0.07, h:0.10, color:'yellow', conf:0.64, active:false },
  ];
  return (
    <div className="relative aspect-video overflow-hidden rounded-lg border border-subtle bg-surface-2 scanline-bg">
      {/* placeholder camera image — diagonal stripes */}
      <div className="absolute inset-0" style={{
        background:
          'repeating-linear-gradient(135deg, hsl(var(--surface-2)) 0 18px, hsl(var(--surface-3)) 18px 19px), radial-gradient(60% 70% at 35% 55%, rgba(204,120,92,0.06) 0%, transparent 70%)'
      }}/>
      {/* mono caption */}
      <div className="absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 font-mono text-micro uppercase tracking-[0.2em] text-foreground-faint">
        camera feed
      </div>

      {/* scanline */}
      <div className="pointer-events-none absolute inset-0 overflow-hidden">
        <div className="absolute left-0 right-0 h-px bg-accent/40 animate-scanline"/>
      </div>

      {/* HUD top-left FSM */}
      <div className="absolute top-2 left-2">
        <FsmBadge state={fsm} target={target} compact/>
      </div>

      {/* HUD top-right time */}
      <div className="absolute top-2 right-2 font-mono text-micro text-foreground-muted bg-background/70 backdrop-blur-sm rounded px-2 py-1 border border-subtle">
        {simTime}
      </div>

      {/* HUD bottom-left crosshair */}
      <div className="absolute bottom-2 left-2 flex items-center gap-1.5 bg-background/70 backdrop-blur-sm rounded-full border border-subtle px-2 py-1">
        <TargetIcon className="h-3 w-3 text-foreground-muted"/>
        <span className="font-mono text-micro text-foreground-muted">CAM 0 · 1080p · 30fps</span>
      </div>

      {/* HUD bottom-right LIVE */}
      <div className="absolute bottom-2 right-2 flex items-center gap-1.5 bg-background/70 backdrop-blur-sm rounded-full border border-subtle px-2 py-1">
        <span className={cn('h-1.5 w-1.5 rounded-full', connected?'bg-success animate-pulse-soft':'bg-danger')}/>
        <span className="font-mono text-micro text-foreground-muted">{connected?'LIVE':'OFFLINE'}</span>
      </div>

      {/* Detection bboxes */}
      {detections.map(d => (
        <div key={d.id}
          className={cn('absolute border transition-all duration-fast animate-fade-in-up',
            d.active ? 'border-accent bg-accent/10' : 'border-foreground-muted/40 bg-foreground-muted/5')}
          style={{ left:`${d.x*100}%`, top:`${d.y*100}%`, width:`${d.w*100}%`, height:`${d.h*100}%` }}>
          <span className={cn('absolute -top-5 left-0 font-mono text-micro px-1.5 rounded',
            d.active?'text-accent bg-background/80 border border-accent/30':'text-foreground-muted bg-background/80 border border-subtle')}>
            {d.color} · {(d.conf*100).toFixed(0)}%
          </span>
          {d.active && (
            <>
              <div className="absolute -top-1 -left-1 w-2 h-2 border-t border-l border-accent"/>
              <div className="absolute -top-1 -right-1 w-2 h-2 border-t border-r border-accent"/>
              <div className="absolute -bottom-1 -left-1 w-2 h-2 border-b border-l border-accent"/>
              <div className="absolute -bottom-1 -right-1 w-2 h-2 border-b border-r border-accent"/>
            </>
          )}
        </div>
      ))}
    </div>
  );
};

// =========== Command input (terminal) ============
const CommandInput = ({ onSend }) => {
  const [v, setV] = useStateD('');
  const [focused, setFocused] = useStateD(false);
  return (
    <div className={cn('flex items-center gap-2 rounded-md border bg-surface-1 px-3 py-2 transition-colors',
      focused ? 'border-accent' : 'border-subtle')}>
      <span className={cn('font-mono text-body text-accent select-none', focused && 'animate-blink')}>&gt;</span>
      <input value={v} onChange={e=>setV(e.target.value)}
        onFocus={()=>setFocused(true)} onBlur={()=>setFocused(false)}
        onKeyDown={e=>{ if(e.key==='Enter'){ onSend?.(v); setV(''); } }}
        placeholder="найди красный мяч"
        className="flex-1 bg-transparent border-none outline-none font-mono text-body text-foreground placeholder:text-foreground-faint"/>
      <Kbd>↵</Kbd>
    </div>
  );
};

// =========== Quick commands ============
const QUICK = [
  { label:'Start', tone:'accent', Icon: PlayIcon },
  { label:'Pause', tone:'default', Icon: PauseIcon },
  { label:'Stop',  tone:'default', Icon: StopIcon },
  { label:'Reset', tone:'default', Icon: RotateIcon },
  { label:'Find red',    tone:'default' },
  { label:'Find blue',   tone:'default' },
  { label:'Return home', tone:'default' },
  { label:'Calibrate',   tone:'default' },
];
const QuickCommandButtons = () => (
  <div className="grid grid-cols-4 gap-1.5">
    {QUICK.map((q,i)=>(
      <Button key={i} size="sm" variant={i===0?'default':'secondary'} className="h-8 text-small font-medium">
        {q.Icon ? <q.Icon className="h-3.5 w-3.5"/> : null}{q.label}
      </Button>
    ))}
  </div>
);

// =========== Detection banner + balls table ============
const DetectionBanner = () => (
  <div className="rounded-md border border-accent/30 bg-accent/8 px-3 py-2 flex items-center gap-2.5">
    <TargetIcon className="h-4 w-4 text-accent"/>
    <span className="text-body text-foreground">Tracking <span className="text-accent font-medium">red ball</span></span>
    <span className="font-mono text-micro text-foreground-muted ml-auto">conf 0.92 · dist 1.42m · θ −12°</span>
  </div>
);

const BALLS = [
  { c:'red',    n:3, last:'2s' },
  { c:'blue',   n:2, last:'8s' },
  { c:'green',  n:1, last:'24s' },
  { c:'yellow', n:1, last:'31s' },
  { c:'orange', n:0, last:'—' },
];
const BALL_HEX = {
  red:'#B85C5C', blue:'#5C7DC9', green:'#6BA86B', yellow:'#C9B05C',
  orange:'#C9885C', white:'#D8D2C8', black:'#2A2826',
};
const BallsTable = () => (
  <table className="w-full text-body">
    <thead>
      <tr className="text-foreground-muted">
        <th className="text-left font-mono text-micro uppercase tracking-wider py-2 px-3">Color</th>
        <th className="text-right font-mono text-micro uppercase tracking-wider py-2 px-3">Count</th>
        <th className="text-right font-mono text-micro uppercase tracking-wider py-2 px-3">Last seen</th>
      </tr>
    </thead>
    <tbody>
      {BALLS.map(b=>(
        <tr key={b.c} className="border-t border-subtle hover:bg-surface-2/40 transition-colors">
          <td className="py-2 px-3">
            <span className="inline-flex items-center gap-2">
              <span className="h-2.5 w-2.5 rounded-full border border-strong" style={{background:BALL_HEX[b.c]}}/>
              <span className="capitalize">{b.c}</span>
            </span>
          </td>
          <td className="py-2 px-3 text-right font-mono tabular-nums">{b.n}</td>
          <td className="py-2 px-3 text-right font-mono tabular-nums text-foreground-muted">{b.last}</td>
        </tr>
      ))}
    </tbody>
  </table>
);

// =========== Map ============
const MapCanvas = () => (
  <div className="relative aspect-[4/3] rounded-md border border-subtle bg-surface-2 overflow-hidden">
    {/* base */}
    <div className="absolute inset-0" style={{
      background: 'radial-gradient(80% 80% at 50% 55%, hsl(var(--surface-3)) 0%, hsl(var(--surface-2)) 70%)'
    }}/>
    {/* grid */}
    <svg className="absolute inset-0 w-full h-full" viewBox="0 0 400 300" preserveAspectRatio="none">
      <defs>
        <pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse">
          <path d="M 20 0 L 0 0 0 20" fill="none" stroke="hsl(var(--surface-3))" strokeWidth="0.5"/>
        </pattern>
      </defs>
      <rect width="400" height="300" fill="url(#grid)" opacity="0.7"/>
      {/* obstacles */}
      <rect x="60" y="50" width="80" height="40" fill="hsl(var(--surface-3))" stroke="hsl(var(--border-strong))"/>
      <rect x="240" y="180" width="100" height="60" fill="hsl(var(--surface-3))" stroke="hsl(var(--border-strong))"/>
      {/* forbidden zone */}
      <rect x="280" y="40" width="80" height="60" fill="#C97B7B" fillOpacity="0.08"
            stroke="#C97B7B" strokeOpacity="0.6" strokeDasharray="4 3"/>
      {/* planned path */}
      <path d="M 200 200 Q 160 140 120 110 T 60 70" fill="none" stroke="#7DD3A8"
            strokeWidth="1.5" strokeDasharray="6 4"/>
      {/* travelled path */}
      <path d="M 50 270 L 90 250 L 130 230 L 170 215 L 200 200" fill="none"
            stroke="hsl(var(--accent))" strokeWidth="1.5" opacity="0.7"/>
      {/* scanpoints */}
      {[[80,200],[140,170],[210,150]].map(([x,y],i)=>(
        <circle key={i} cx={x} cy={y} r="2" fill="hsl(var(--foreground-muted))" fillOpacity="0.3"/>
      ))}
      {/* robot */}
      <g transform="translate(200 200) rotate(-30)">
        <circle r="8" fill="hsl(var(--accent))"/>
        <line x1="0" y1="0" x2="0" y2="-14" stroke="hsl(var(--accent))" strokeWidth="2" strokeLinecap="round"/>
        <circle r="14" fill="none" stroke="hsl(var(--accent))" strokeOpacity="0.3" strokeDasharray="2 3"/>
      </g>
    </svg>
    {/* legend */}
    <div className="absolute bottom-2 left-2 right-2 flex flex-wrap items-center gap-3 bg-background/70 backdrop-blur-sm border border-subtle rounded-md px-2 py-1">
      {[
        ['Robot', 'hsl(var(--accent))'],
        ['Path',  '#CC785C'],
        ['Plan',  '#7DD3A8'],
        ['Forbid','#C97B7B'],
      ].map(([k,c],i)=>(
        <span key={i} className="inline-flex items-center gap-1.5 font-mono text-micro text-foreground-muted">
          <span className="h-1.5 w-3 rounded-sm" style={{background:c}}/>{k}
        </span>
      ))}
      <span className="ml-auto font-mono text-micro text-foreground-faint tabular-nums">x 1.24  y −0.86  θ −30°</span>
    </div>
  </div>
);

const MapToolbar = () => {
  const [grid, setGrid] = useStateD(true);
  const [heat, setHeat] = useStateD(false);
  const [zones, setZones] = useStateD(true);
  return (
    <div className="flex flex-wrap items-center gap-1.5">
      <Button size="sm" variant={grid?'default':'secondary'} onClick={()=>setGrid(!grid)}>
        <GridIcon className="h-3.5 w-3.5"/>Grid
      </Button>
      <Button size="sm" variant={heat?'default':'secondary'} onClick={()=>setHeat(!heat)}>
        <LayersIcon className="h-3.5 w-3.5"/>Coverage
      </Button>
      <Button size="sm" variant={zones?'default':'secondary'} onClick={()=>setZones(!zones)}>
        <AlertIcon className="h-3.5 w-3.5"/>Zones
      </Button>
      <span className="ml-auto inline-flex items-center gap-1.5">
        <Button size="sm" variant="ghost"><PlusIcon className="h-3.5 w-3.5"/></Button>
        <span className="font-mono text-micro text-foreground-faint tabular-nums">100%</span>
        <Button size="sm" variant="ghost"><MinusIcon className="h-3.5 w-3.5"/></Button>
      </span>
    </div>
  );
};

// =========== Sensor charts (SVG line) ============
const seriesGen = (n, base, amp, seed) => {
  let r = seed;
  return Array.from({length:n}, (_,i) => {
    r = (r*9301+49297) % 233280;
    return base + Math.sin(i/4 + seed) * amp * 0.5 + (r/233280-0.5)*amp*0.6;
  });
};
const SparkLine = ({ data, color, label, value, unit }) => {
  const max = Math.max(...data), min = Math.min(...data);
  const W = 220, H = 56;
  const pts = data.map((v,i)=>[i/(data.length-1)*W, H - (v-min)/(max-min||1) * H * 0.9 - H*0.05]);
  const d = pts.map((p,i)=>`${i===0?'M':'L'}${p[0].toFixed(1)} ${p[1].toFixed(1)}`).join(' ');
  return (
    <div className="space-y-1">
      <div className="flex items-baseline justify-between">
        <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">{label}</span>
        <span className="font-mono text-small tabular-nums" style={{color}}>{value}<span className="text-foreground-faint ml-0.5">{unit}</span></span>
      </div>
      <svg viewBox={`0 0 ${W} ${H}`} className="w-full h-12">
        <line x1="0" y1={H*.5} x2={W} y2={H*.5} stroke="hsl(var(--surface-3))" strokeDasharray="2 3"/>
        <path d={d} fill="none" stroke={color} strokeWidth="1.5" strokeLinecap="round"/>
        <circle cx={pts[pts.length-1][0]} cy={pts[pts.length-1][1]} r="2" fill={color}/>
      </svg>
    </div>
  );
};
const SensorCharts = () => {
  const battery = useMemoD(()=>seriesGen(40, 78, 6, 13), []);
  const temp    = useMemoD(()=>seriesGen(40, 42, 4, 21), []);
  const speed   = useMemoD(()=>seriesGen(40, 0.8, 0.6, 7), []);
  return (
    <div className="grid grid-cols-3 gap-4">
      <SparkLine data={battery} color="hsl(var(--accent))"  label="Battery" value="76.4" unit="%"/>
      <SparkLine data={temp}    color="hsl(var(--warning))" label="Temp"    value="43.1" unit="°C"/>
      <SparkLine data={speed}   color="hsl(var(--info))"    label="Speed"   value="0.84" unit="m/s"/>
    </div>
  );
};

// =========== Joystick ============
const Joystick = () => {
  const [pos, setPos] = useStateD({ x:0, y:0, active:false });
  const ref = useRefD(null);
  const R = 88, KR = 22;
  const onDown = (e) => {
    ref.current.setPointerCapture?.(e.pointerId);
    update(e);
  };
  const onMove = (e) => { if(e.buttons) update(e); };
  const onUp = (e) => {
    ref.current.releasePointerCapture?.(e.pointerId);
    setPos({ x:0, y:0, active:false });
  };
  const update = (e) => {
    const rect = ref.current.getBoundingClientRect();
    let dx = e.clientX - rect.left - rect.width/2;
    let dy = e.clientY - rect.top  - rect.height/2;
    const m = Math.hypot(dx, dy);
    if (m > R-KR) { dx = dx*(R-KR)/m; dy = dy*(R-KR)/m; }
    setPos({ x:dx, y:dy, active:true });
  };
  const lin = (-pos.y / (R-KR)).toFixed(2);
  const ang = ( pos.x / (R-KR)).toFixed(2);

  return (
    <div className="flex flex-col items-center gap-3 select-none">
      <div ref={ref}
        onPointerDown={onDown} onPointerMove={onMove} onPointerUp={onUp} onPointerCancel={onUp}
        className="joystick-area relative h-44 w-44 rounded-full border border-subtle bg-surface-2 cursor-grab active:cursor-grabbing">
        <svg viewBox="0 0 176 176" className="absolute inset-0 pointer-events-none">
          <line x1="88" y1="20" x2="88" y2="156" stroke="hsl(var(--surface-3))" strokeWidth="1"/>
          <line x1="20" y1="88" x2="156" y2="88" stroke="hsl(var(--surface-3))" strokeWidth="1"/>
          <circle cx="88" cy="88" r="56" fill="none" stroke="hsl(var(--surface-3))" strokeWidth="1" strokeDasharray="3 4"/>
          <circle cx="88" cy="88" r="28" fill="none" stroke="hsl(var(--surface-3))" strokeWidth="1" strokeDasharray="2 3"/>
          {pos.active && (
            <line x1="88" y1="88" x2={88+pos.x} y2={88+pos.y}
              stroke="hsl(var(--accent))" strokeWidth="2" strokeLinecap="round"/>
          )}
        </svg>
        <div
          className={cn('absolute h-11 w-11 rounded-full bg-surface-3 border-2 border-accent',
            'transition-transform', pos.active ? 'duration-fast ease-standard' : 'duration-standard ease-spring')}
          style={{ left:`calc(50% - 22px + ${pos.x}px)`, top:`calc(50% - 22px + ${pos.y}px)` }}/>
      </div>
      <div className="font-mono text-small text-foreground-muted tabular-nums flex items-center gap-1">
        <span className={cn(lin!=='0.00' && 'text-accent')}>{lin}</span>
        <span className="text-foreground-faint">m/s</span>
        <span className="mx-2 text-foreground-faint">·</span>
        <span className={cn(ang!=='0.00' && 'text-accent')}>{ang}</span>
        <span className="text-foreground-faint">rad/s</span>
      </div>
    </div>
  );
};

// =========== Sensor panel ============
const SensorRow = ({ Icon: I, label, value, unit, tone='foreground' }) => (
  <div className="flex items-center justify-between py-1.5">
    <span className="inline-flex items-center gap-2 text-foreground-muted">
      <I className="h-3.5 w-3.5"/>
      <span className="text-body">{label}</span>
    </span>
    <span className="font-mono text-small tabular-nums">
      <span className={cn(`text-${tone}`)}>{value}</span>
      <span className="text-foreground-faint ml-1">{unit}</span>
    </span>
  </div>
);
const SensorPanel = () => (
  <div className="divide-y divide-subtle">
    <SensorRow Icon={BatteryIcon} label="Battery"    value="76.4" unit="%"   tone="foreground"/>
    <SensorRow Icon={ThermoIcon}  label="Temp"       value="43.1" unit="°C"  tone="warning"/>
    <SensorRow Icon={WifiIcon}    label="Signal"     value="−54"  unit="dBm" tone="foreground"/>
    <SensorRow Icon={ZapIcon}     label="Current"    value="2.81" unit="A"   tone="foreground"/>
    <SensorRow Icon={TargetIcon}  label="Lidar Hz"   value="20"   unit=""    tone="success"/>
  </div>
);

// =========== Toggles ============
const Toggle = ({ on, onChange, label }) => (
  <button onClick={()=>onChange?.(!on)}
    className={cn('flex items-center justify-between gap-2 w-full rounded-md border px-3 py-2 transition-colors',
      on ? 'border-accent/40 bg-accent/8' : 'border-subtle bg-surface-2 hover:bg-surface-3')}>
    <span className="text-body">{label}</span>
    <span className={cn('relative inline-block h-4 w-7 rounded-full transition-colors',
      on ? 'bg-accent' : 'bg-surface-3')}>
      <span className={cn('absolute top-0.5 h-3 w-3 rounded-full bg-foreground transition-transform',
        on ? 'translate-x-3.5' : 'translate-x-0.5')}/>
    </span>
  </button>
);
const ActuatorToggles = () => {
  const [s, setS] = useStateD({ claw:false, light:true, fan:false, lift:false });
  return (
    <div className="grid grid-cols-2 gap-1.5">
      <Toggle label="Claw"  on={s.claw}  onChange={v=>setS(x=>({...x,claw:v}))}/>
      <Toggle label="Light" on={s.light} onChange={v=>setS(x=>({...x,light:v}))}/>
      <Toggle label="Fan"   on={s.fan}   onChange={v=>setS(x=>({...x,fan:v}))}/>
      <Toggle label="Lift"  on={s.lift}  onChange={v=>setS(x=>({...x,lift:v}))}/>
    </div>
  );
};

// =========== Event log ============
const EVENTS = [
  ['12:34:51', 'fsm', 'TARGETING', 'red ball acquired conf=0.92'],
  ['12:34:48', 'det', 'red',       'new detection at (0.18, 0.42)'],
  ['12:34:42', 'fsm', 'SEARCHING', 'sweep 180° complete'],
  ['12:34:31', 'sys', 'info',      'lidar 20Hz nominal'],
  ['12:34:18', 'fsm', 'IDLE',      'awaiting command'],
  ['12:34:02', 'sys', 'ok',        'socket.io connected'],
  ['12:33:55', 'cmd', 'start',     'operator: start mission'],
];
const EventLog = () => (
  <ul className="font-mono text-small divide-y divide-subtle">
    {EVENTS.map((e,i)=>(
      <li key={i} className={cn('grid grid-cols-[auto_auto_auto_1fr] items-center gap-3 py-2 px-1', i===0 && 'animate-fade-in-up')}>
        <span className="text-foreground-faint tabular-nums">{e[0]}</span>
        <Badge tone={e[1]==='fsm'?'accent':e[1]==='det'?'info':e[1]==='cmd'?'warning':'default'} className="text-[10px]">{e[1]}</Badge>
        <span className="text-foreground uppercase tracking-wider text-micro">{e[2]}</span>
        <span className="text-foreground-muted truncate">{e[3]}</span>
      </li>
    ))}
  </ul>
);

// =========== Status banner (compact) ============
const StatusBanner = ({ robot, fsm, target }) => (
  <Card>
    <CardContent className="flex items-center justify-between gap-3">
      <div className="flex items-center gap-3 min-w-0">
        <div className="h-9 w-9 rounded-md bg-accent/10 border border-accent/20 grid place-items-center">
          <KatanaIcon className="h-4 w-4 text-accent"/>
        </div>
        <div className="min-w-0">
          <div className="text-body font-medium truncate">{robot}</div>
          <div className="font-mono text-micro text-foreground-faint">10.0.0.42 · sim mode</div>
        </div>
      </div>
      <FsmBadge state={fsm} target={target}/>
    </CardContent>
  </Card>
);

// =========== Detection toggles ============
const DetectionTogglePanel = () => {
  const colors = ['red','blue','green','yellow','orange','white','black'];
  const [on, setOn] = useStateD({ red:true, blue:true, green:false, yellow:true, orange:false, white:false, black:false });
  return (
    <div className="flex flex-wrap gap-1.5">
      {colors.map(c=>(
        <button key={c} onClick={()=>setOn(s=>({...s,[c]:!s[c]}))}
          className={cn('inline-flex items-center gap-2 rounded-pill border px-2.5 py-1 transition-all',
            on[c] ? 'border-strong bg-surface-2' : 'border-subtle bg-transparent text-foreground-faint hover:bg-surface-2/40')}>
          <span className="h-2 w-2 rounded-full border border-strong" style={{background: BALL_HEX[c]}}/>
          <span className="font-mono text-micro uppercase tracking-wider capitalize">{c}</span>
        </button>
      ))}
    </div>
  );
};

// =========== Path recorder ============
const PathRecorderPanel = () => {
  const [rec, setRec] = useStateD(false);
  return (
    <div className="flex items-center gap-2">
      <Button variant={rec?'destructive':'secondary'} size="sm" onClick={()=>setRec(!rec)}>
        <RecordIcon className={cn('h-3 w-3', rec && 'animate-pulse-soft')}/>
        {rec?'Stop':'Record'}
      </Button>
      <Button variant="secondary" size="sm">Save…</Button>
      <Button variant="ghost" size="sm">Clear</Button>
      <span className="ml-auto font-mono text-micro text-foreground-faint tabular-nums">
        {rec ? '00:14 · 28 pts' : 'idle'}
      </span>
    </div>
  );
};

// =========== Dashboard layout ============
const DashboardPage = ({ fsm, target, simTime, connected }) => (
  <div className="p-6 max-w-[1920px] mx-auto">
    <div className="grid grid-cols-12 gap-4">
      {/* Col 1 */}
      <div className="col-span-12 xl:col-span-4 space-y-4">
        <Card>
          <CardHeader right={<CardSubtitle>cam 0</CardSubtitle>}>
            <CardTitle>Camera</CardTitle>
          </CardHeader>
          <CardContent className="p-3">
            <CameraFeed fsm={fsm} target={target} connected={connected} simTime={simTime}/>
          </CardContent>
        </Card>

        <Card>
          <CardContent className="space-y-3">
            <CommandInput/>
            <QuickCommandButtons/>
            <DetectionBanner/>
          </CardContent>
        </Card>

        <Card>
          <CardHeader right={<CardSubtitle>{BALLS.reduce((a,b)=>a+b.n,0)} total</CardSubtitle>}>
            <CardTitle>Detected balls</CardTitle>
          </CardHeader>
          <CardContent className="p-0">
            <BallsTable/>
          </CardContent>
        </Card>
      </div>

      {/* Col 2 */}
      <div className="col-span-12 xl:col-span-5 space-y-4">
        <Card>
          <CardHeader right={<MapToolbar/>}>
            <CardTitle>Map</CardTitle>
          </CardHeader>
          <CardContent className="p-3">
            <MapCanvas/>
          </CardContent>
        </Card>

        <Card>
          <CardHeader right={<CardSubtitle>last 60s</CardSubtitle>}>
            <CardTitle>Telemetry</CardTitle>
          </CardHeader>
          <CardContent>
            <SensorCharts/>
          </CardContent>
        </Card>

        <Card>
          <CardHeader right={<CardSubtitle>{EVENTS.length} events</CardSubtitle>}>
            <CardTitle>Event log</CardTitle>
          </CardHeader>
          <CardContent className="p-2">
            <EventLog/>
          </CardContent>
        </Card>
      </div>

      {/* Col 3 */}
      <div className="col-span-12 xl:col-span-3 space-y-4">
        <StatusBanner robot="samurai" fsm={fsm} target={target}/>

        <Card>
          <CardHeader right={<Badge tone="accent" dot>{fsm}</Badge>}>
            <CardTitle>State machine</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <FsmTimeline current={fsm}/>
            <div className="font-mono text-micro text-foreground-faint">
              transitioned from <span className="text-foreground-muted">SEARCHING</span> 4.2s ago
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>Sensors</CardTitle></CardHeader>
          <CardContent><SensorPanel/></CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>Actuators</CardTitle></CardHeader>
          <CardContent><ActuatorToggles/></CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>Joystick</CardTitle></CardHeader>
          <CardContent><Joystick/></CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>Detect colors</CardTitle></CardHeader>
          <CardContent><DetectionTogglePanel/></CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>Path recorder</CardTitle></CardHeader>
          <CardContent><PathRecorderPanel/></CardContent>
        </Card>
      </div>
    </div>
  </div>
);

Object.assign(window, {
  CameraFeed, CommandInput, QuickCommandButtons, DetectionBanner, BallsTable,
  MapCanvas, MapToolbar, SensorCharts, Joystick, SensorPanel, ActuatorToggles,
  EventLog, StatusBanner, DetectionTogglePanel, PathRecorderPanel, DashboardPage,
  BALL_HEX,
});
