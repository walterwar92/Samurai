/* global React */
const { useState: useStateO, useMemo: useMemoO, useEffect: useEffectO, useRef: useRefO } = React;

// =========================================================
// ADMIN
// =========================================================
const ServoSlider = ({ label, value, min=0, max=180 }) => {
  const [v, setV] = useStateO(value);
  const pct = ((v-min)/(max-min))*100;
  return (
    <div className="space-y-1.5">
      <div className="flex items-center justify-between">
        <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted">{label}</span>
        <span className="font-mono text-small tabular-nums">{v}°</span>
      </div>
      <div className="relative h-1.5 rounded-full bg-surface-3">
        <div className="absolute left-0 top-0 h-full rounded-full bg-accent" style={{width:`${pct}%`}}/>
        <input type="range" min={min} max={max} value={v} onChange={e=>setV(+e.target.value)}
          className="absolute inset-0 w-full opacity-0 cursor-pointer"/>
        <div className="absolute top-1/2 -translate-y-1/2 -translate-x-1/2 h-3 w-3 rounded-full bg-accent border-2 border-background pointer-events-none"
          style={{left:`${pct}%`}}/>
      </div>
      <div className="flex items-center justify-between font-mono text-micro text-foreground-faint">
        <span>{min}°</span><span>{max}°</span>
      </div>
    </div>
  );
};

const SpeedBtns = () => {
  const [v, setV] = useStateO(2);
  return (
    <div className="flex items-center gap-1">
      {[1,2,3,4,5].map(n=>(
        <button key={n} onClick={()=>setV(n)}
          className={cn('h-8 w-8 rounded-md font-mono text-small transition-colors',
            v===n ? 'bg-accent text-accent-foreground' : 'bg-surface-2 text-foreground-muted hover:bg-surface-3')}>
          {n}
        </button>
      ))}
    </div>
  );
};

const FsmManualGrid = ({ current }) => (
  <div className="grid grid-cols-4 gap-1.5">
    {FSM_STATES.map(s => (
      <button key={s}
        className={cn('h-9 rounded-md border font-mono text-micro uppercase tracking-wider transition-colors',
          s===current ? 'border-accent bg-accent/10 text-accent' : 'border-subtle bg-surface-2 text-foreground-muted hover:bg-surface-3 hover:text-foreground')}>
        {FSM_SHORT[s]}
      </button>
    ))}
  </div>
);

const PathBtn = ({ children }) => (
  <Button variant="secondary" size="sm" className="font-mono text-micro uppercase tracking-wider">{children}</Button>
);

const AdminPage = ({ fsm, target, simTime }) => (
  <div className="p-6 max-w-[1920px] mx-auto space-y-4">
    <div className="grid grid-cols-12 gap-4">
      {/* Status row */}
      <div className="col-span-12">
        <Card>
          <CardContent className="flex flex-wrap items-center gap-4">
            <FsmBadge state={fsm} target={target}/>
            <div className="flex-1 min-w-[300px]">
              <FsmTimeline current={fsm}/>
            </div>
            <Button variant="destructive" size="lg" className="ml-auto">
              <AlertIcon className="h-4 w-4"/>Emergency stop
            </Button>
          </CardContent>
        </Card>
      </div>

      {/* FSM manual */}
      <div className="col-span-12 xl:col-span-4">
        <Card>
          <CardHeader><CardTitle>Manual FSM</CardTitle></CardHeader>
          <CardContent className="space-y-3">
            <FsmManualGrid current={fsm}/>
            <div className="font-mono text-micro text-foreground-faint pt-2 border-t border-subtle">
              forces a state transition. ignores guards.
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Path control */}
      <div className="col-span-12 xl:col-span-4">
        <Card>
          <CardHeader right={<SpeedBtns/>}>
            <CardTitle>Path control</CardTitle>
          </CardHeader>
          <CardContent className="space-y-3">
            <div className="grid grid-cols-3 gap-1.5">
              <div/><PathBtn>↑ fwd</PathBtn><div/>
              <PathBtn>← left</PathBtn>
              <Button variant="outline" size="sm" className="font-mono text-micro uppercase tracking-wider">stop</Button>
              <PathBtn>right →</PathBtn>
              <div/><PathBtn>↓ back</PathBtn><div/>
            </div>
            <div className="grid grid-cols-2 gap-1.5">
              <Button variant="secondary" size="sm">Rotate −90°</Button>
              <Button variant="secondary" size="sm">Rotate +90°</Button>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Calibration */}
      <div className="col-span-12 xl:col-span-4">
        <Card>
          <CardHeader><CardTitle>Calibration</CardTitle></CardHeader>
          <CardContent className="space-y-2">
            {['IMU bias','Wheel odometry','Camera intrinsics','Lidar offset'].map(l=>(
              <div key={l} className="flex items-center justify-between rounded-md bg-surface-2 px-3 py-2">
                <span className="text-body">{l}</span>
                <Button size="sm" variant="ghost">Run</Button>
              </div>
            ))}
          </CardContent>
        </Card>
      </div>

      {/* Servos */}
      <div className="col-span-12 xl:col-span-8">
        <Card>
          <CardHeader right={<CardSubtitle>4 channels</CardSubtitle>}>
            <CardTitle>Servo control</CardTitle>
          </CardHeader>
          <CardContent className="grid grid-cols-2 lg:grid-cols-4 gap-6">
            <ServoSlider label="base"     value={90}/>
            <ServoSlider label="shoulder" value={45}/>
            <ServoSlider label="elbow"    value={120}/>
            <ServoSlider label="gripper"  value={20} max={90}/>
          </CardContent>
        </Card>
      </div>

      {/* PID */}
      <div className="col-span-12 xl:col-span-4">
        <Card>
          <CardHeader><CardTitle>PID gains</CardTitle></CardHeader>
          <CardContent className="space-y-3">
            {[['Kp', '1.24'], ['Ki', '0.08'], ['Kd', '0.32']].map(([k,v])=>(
              <div key={k} className="flex items-center gap-3">
                <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted w-8">{k}</span>
                <input defaultValue={v}
                  className="flex-1 h-9 rounded-md border border-subtle bg-surface-2 px-3 font-mono text-body text-foreground focus:outline-none focus:border-accent"/>
              </div>
            ))}
            <Button variant="default" size="sm" className="w-full">Apply</Button>
          </CardContent>
        </Card>
      </div>
    </div>
  </div>
);

// =========================================================
// 3D VIEW
// =========================================================
const Visualization3DPage = ({ simTime }) => (
  <div className="p-6 max-w-[1920px] mx-auto">
    <div className="grid grid-cols-12 gap-4">
      <div className="col-span-12 xl:col-span-9">
        <Card>
          <CardHeader right={<><Button size="sm" variant="ghost">Reset cam</Button><Button size="sm" variant="secondary">Snapshot</Button></>}>
            <CardTitle>3D Scene</CardTitle>
          </CardHeader>
          <CardContent className="p-0">
            <div className="relative aspect-[16/9] bg-[#1F1E1D] overflow-hidden">
              {/* studio gradient */}
              <div className="absolute inset-0" style={{
                background:'radial-gradient(60% 80% at 50% 80%, #2a2724 0%, #1F1E1D 60%)'
              }}/>
              {/* floor grid (perspective) */}
              <svg viewBox="0 0 1600 900" className="absolute inset-0 w-full h-full" preserveAspectRatio="xMidYMid slice">
                <defs>
                  <radialGradient id="vignette" cx="50%" cy="60%" r="60%">
                    <stop offset="0%" stopColor="#2a2724" stopOpacity="0"/>
                    <stop offset="100%" stopColor="#1F1E1D" stopOpacity="1"/>
                  </radialGradient>
                </defs>
                {/* floor lines */}
                {Array.from({length:14}, (_,i)=>{
                  const t = i/13;
                  const y = 480 + t*420;
                  const w = 40 + t*1500;
                  return <line key={'h'+i} x1={800-w/2} x2={800+w/2} y1={y} y2={y}
                    stroke="#2D2B28" strokeWidth={1+t*0.6} opacity={0.5+t*0.5}/>;
                })}
                {Array.from({length:21}, (_,i)=>{
                  const x = i/20;
                  const x1 = 800 + (x-0.5)*80;
                  const x2 = 800 + (x-0.5)*1600;
                  return <line key={'v'+i} x1={x1} y1={480} x2={x2} y2={900}
                    stroke="#2D2B28" strokeWidth="1" opacity="0.6"/>;
                })}
                {/* axes */}
                <g transform="translate(640 700)">
                  <line x1="0" y1="0" x2="120" y2="20"  stroke="#C97B7B" strokeWidth="2"/>
                  <line x1="0" y1="0" x2="-90" y2="40"  stroke="#7DA88A" strokeWidth="2"/>
                  <line x1="0" y1="0" x2="0"  y2="-90" stroke="#7DA1C9" strokeWidth="2"/>
                  <text x="125" y="22" fontSize="11" fill="#C97B7B" fontFamily="JetBrains Mono">X</text>
                  <text x="-100" y="50" fontSize="11" fill="#7DA88A" fontFamily="JetBrains Mono">Y</text>
                  <text x="4" y="-95" fontSize="11" fill="#7DA1C9" fontFamily="JetBrains Mono">Z</text>
                </g>
                {/* robot box */}
                <g transform="translate(800 600)">
                  <polygon points="-60,-40 60,-40 80,-10 -80,-10" fill="#34322F" stroke="#3A3733"/>
                  <polygon points="-60,-40 -50,-70 50,-70 60,-40" fill="#3A3733" stroke="#3A3733"/>
                  <polygon points="60,-40 50,-70 50,-50 80,-10" fill="#2D2B28" stroke="#3A3733"/>
                  {/* coral accent panel */}
                  <rect x="-30" y="-65" width="60" height="20" fill="#CC785C" opacity="0.85"/>
                  <rect x="-30" y="-65" width="60" height="20" fill="none" stroke="#D5876C" strokeWidth="1"/>
                </g>
                {/* travelled path */}
                <path d="M 250 850 Q 460 760 640 720 T 800 600" fill="none" stroke="#CC785C" strokeWidth="2" opacity="0.85"/>
                {/* planned path */}
                <path d="M 800 600 Q 1000 540 1180 580 T 1400 660" fill="none" stroke="#7DD3A8" strokeWidth="2" strokeDasharray="6 4"/>
                {/* slam cloud */}
                {Array.from({length:80},(_,i)=>{
                  const a = (i*7919)%360, r = 200 + ((i*7)%180);
                  const x = 800 + Math.cos(a)*r;
                  const y = 650 + Math.sin(a)*r*0.4;
                  return <circle key={i} cx={x} cy={y} r="1.5" fill="#A8A39A" opacity="0.35"/>;
                })}
                <rect x="0" y="0" width="1600" height="900" fill="url(#vignette)"/>
              </svg>

              {/* corner overlays */}
              <div className="absolute top-3 left-3 rounded-md border border-subtle bg-surface-1/80 backdrop-blur-md px-3 py-2 font-mono text-small">
                <div className="flex items-center gap-2 mb-1">
                  <span className="h-1.5 w-1.5 rounded-full bg-success animate-pulse-soft"/>
                  <span className="uppercase text-micro tracking-wider text-foreground-muted">scene · live</span>
                </div>
                <div className="text-foreground-muted tabular-nums text-micro">
                  pos&nbsp; 1.24, −0.86, 0.00<br/>
                  rot&nbsp; 0°, 0°, −30°<br/>
                  fps&nbsp; 60<br/>
                  pts&nbsp; 1 248
                </div>
              </div>
              <div className="absolute top-3 right-3 flex flex-col gap-1.5">
                <Button size="sm" variant="secondary" className="bg-surface-1/80 backdrop-blur">EKF</Button>
                <Button size="sm" variant="secondary" className="bg-surface-1/80 backdrop-blur">SLAM</Button>
                <Button size="sm" variant="secondary" className="bg-surface-1/80 backdrop-blur">Path</Button>
              </div>
              <div className="absolute bottom-3 right-3 flex items-center gap-1.5">
                <Button size="sm" variant="ghost" className="bg-surface-1/80 backdrop-blur">Clear cloud</Button>
                <Button size="sm" variant="ghost" className="bg-surface-1/80 backdrop-blur">Reset position</Button>
              </div>
              <div className="absolute bottom-3 left-3 font-mono text-micro text-foreground-faint">
                drag · pan&nbsp;&nbsp; scroll · zoom&nbsp;&nbsp; shift+drag · orbit
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      <div className="col-span-12 xl:col-span-3 space-y-4">
        <Card>
          <CardHeader><CardTitle>Layers</CardTitle></CardHeader>
          <CardContent className="space-y-1.5">
            {[
              ['Robot model', true],
              ['Travelled path', true],
              ['Planned path', true],
              ['SLAM cloud', true],
              ['Coordinate frames', false],
              ['Bounding box', false],
            ].map(([l,v])=>(
              <Toggle key={l} label={l} on={v} onChange={()=>{}}/>
            ))}
          </CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>Camera</CardTitle></CardHeader>
          <CardContent className="grid grid-cols-2 gap-1.5">
            <Button size="sm" variant="secondary">Top</Button>
            <Button size="sm" variant="secondary">Side</Button>
            <Button size="sm" variant="secondary">Front</Button>
            <Button size="sm" variant="secondary">ISO</Button>
            <Button size="sm" variant="default" className="col-span-2">Follow robot</Button>
          </CardContent>
        </Card>
      </div>
    </div>
  </div>
);

// =========================================================
// HARDWARE
// =========================================================
const HwBlock = ({ title, kind, status='ok' }) => (
  <div className={cn('rounded-md border bg-surface-2 px-3 py-2.5 cursor-grab transition-colors',
    'border-subtle hover:border-strong')}>
    <div className="flex items-center justify-between">
      <span className="text-body">{title}</span>
      <Badge tone={status==='ok'?'success':status==='warn'?'warning':'danger'} dot>
        {status==='ok'?'ok':status}
      </Badge>
    </div>
    <div className="font-mono text-micro text-foreground-faint mt-0.5">{kind}</div>
  </div>
);

const HardwarePage = () => (
  <div className="p-6 max-w-[1920px] mx-auto">
    <div className="grid grid-cols-12 gap-4">
      {/* Platform selector */}
      <div className="col-span-12 xl:col-span-3">
        <Card>
          <CardHeader><CardTitle>Platform</CardTitle></CardHeader>
          <CardContent className="space-y-1.5">
            {[
              ['samurai-v1', true],
              ['samurai-v2', false],
              ['samcan',     false],
              ['custom',     false],
            ].map(([n,v])=>(
              <button key={n}
                className={cn('w-full flex items-center justify-between rounded-md border px-3 py-2 transition-colors',
                  v ? 'border-accent/40 bg-accent/8 text-foreground' : 'border-subtle bg-surface-2 text-foreground-muted hover:bg-surface-3 hover:text-foreground')}>
                <span className="font-mono text-small">{n}</span>
                {v && <CheckIcon className="h-3.5 w-3.5 text-accent"/>}
              </button>
            ))}
          </CardContent>
        </Card>
      </div>

      {/* Block diagram */}
      <div className="col-span-12 xl:col-span-6">
        <Card>
          <CardHeader right={
            <><Button size="sm" variant="ghost"><PlusIcon className="h-3.5 w-3.5"/>Add block</Button>
              <Badge tone="warning" dot>dirty</Badge></>
          }>
            <CardTitle>Block diagram</CardTitle>
          </CardHeader>
          <CardContent>
            <div className="relative rounded-md border border-dashed border-subtle bg-surface-2/30 p-4 min-h-[420px]">
              {/* connecting lines */}
              <svg className="absolute inset-0 w-full h-full pointer-events-none">
                <line x1="35%" y1="22%" x2="50%" y2="50%" stroke="hsl(var(--border-strong))" strokeWidth="1"/>
                <line x1="65%" y1="22%" x2="50%" y2="50%" stroke="hsl(var(--border-strong))" strokeWidth="1"/>
                <line x1="50%" y1="50%" x2="25%" y2="78%" stroke="hsl(var(--border-strong))" strokeWidth="1"/>
                <line x1="50%" y1="50%" x2="50%" y2="78%" stroke="hsl(var(--border-strong))" strokeWidth="1"/>
                <line x1="50%" y1="50%" x2="75%" y2="78%" stroke="hsl(var(--border-strong))" strokeWidth="1"/>
              </svg>
              <div className="grid grid-cols-3 gap-3 relative">
                <div/>
                <HwBlock title="MCU"     kind="esp32-s3"/>
                <div/>

                <HwBlock title="Camera"  kind="ov2640"/>
                <HwBlock title="Lidar"   kind="rplidar-a2"/>
                <HwBlock title="IMU"     kind="bno055" status="warn"/>

                <div className="col-span-3 mt-4"/>

                <HwBlock title="Wheel L" kind="dc 12V · 200rpm"/>
                <HwBlock title="Wheel R" kind="dc 12V · 200rpm"/>
                <HwBlock title="Gripper" kind="servo mg996"/>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Preset panel */}
      <div className="col-span-12 xl:col-span-3 space-y-4">
        <Card>
          <CardHeader right={<Badge tone="success" dot>saved</Badge>}>
            <CardTitle>Preset</CardTitle>
          </CardHeader>
          <CardContent className="space-y-3">
            <div>
              <label className="font-mono text-micro uppercase tracking-wider text-foreground-muted">Name</label>
              <input defaultValue="samurai-v1.production"
                className="mt-1 w-full h-9 rounded-md border border-subtle bg-surface-2 px-3 font-mono text-body focus:outline-none focus:border-accent"/>
            </div>
            <div>
              <label className="font-mono text-micro uppercase tracking-wider text-foreground-muted">Notes</label>
              <textarea rows={4} defaultValue="Stable build for outdoor field test. IMU recalibrated 2026-05-08."
                className="mt-1 w-full rounded-md border border-subtle bg-surface-2 px-3 py-2 font-mono text-small focus:outline-none focus:border-accent resize-none"/>
            </div>
            <div className="grid grid-cols-2 gap-1.5">
              <Button size="sm">Save</Button>
              <Button size="sm" variant="secondary">Duplicate</Button>
              <Button size="sm" variant="ghost">Export</Button>
              <Button size="sm" variant="ghost">Reset</Button>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>I/O map</CardTitle></CardHeader>
          <CardContent className="font-mono text-micro space-y-1">
            {[
              ['GPIO 4',  'cam_clk'],
              ['GPIO 18', 'lidar_tx'],
              ['GPIO 19', 'lidar_rx'],
              ['GPIO 21', 'i2c_sda'],
              ['GPIO 22', 'i2c_scl'],
              ['GPIO 25', 'pwm_l'],
              ['GPIO 26', 'pwm_r'],
            ].map(([a,b])=>(
              <div key={a} className="flex items-center justify-between">
                <span className="text-foreground-faint">{a}</span>
                <span className="text-foreground-muted">{b}</span>
              </div>
            ))}
          </CardContent>
        </Card>
      </div>
    </div>
  </div>
);

// =========================================================
// SAMCAN
// =========================================================
const HeadingCompass = ({ heading=42 }) => {
  const r = 70, cx=80, cy=80;
  return (
    <svg viewBox="0 0 160 160" className="w-full h-auto">
      <circle cx={cx} cy={cy} r={r} fill="hsl(var(--surface-2))" stroke="hsl(var(--border-subtle))"/>
      <circle cx={cx} cy={cy} r={r-1} fill="none" stroke="hsl(var(--surface-3))" strokeDasharray="2 4"/>
      {Array.from({length:36}, (_,i)=>{
        const a = (i*10) * Math.PI/180;
        const x1 = cx + Math.sin(a)*(r-2);
        const y1 = cy - Math.cos(a)*(r-2);
        const x2 = cx + Math.sin(a)*(r-(i%9===0?9:5));
        const y2 = cy - Math.cos(a)*(r-(i%9===0?9:5));
        return <line key={i} x1={x1} y1={y1} x2={x2} y2={y2}
          stroke={i%9===0?'hsl(var(--foreground-muted))':'hsl(var(--surface-3))'}
          strokeWidth={i%9===0?1.5:1}/>;
      })}
      {['N','E','S','W'].map((c,i)=>{
        const a = (i*90) * Math.PI/180;
        return <text key={c} x={cx + Math.sin(a)*(r-18)} y={cy - Math.cos(a)*(r-18)+4}
          textAnchor="middle" fontSize="10" fontFamily="JetBrains Mono"
          fill={c==='N'?'hsl(var(--accent))':'hsl(var(--foreground-faint))'}>{c}</text>;
      })}
      <g transform={`translate(${cx} ${cy}) rotate(${heading})`}>
        <polygon points="0,-50 6,8 0,4 -6,8" fill="hsl(var(--accent))"/>
        <circle r="3" fill="hsl(var(--surface-1))" stroke="hsl(var(--accent))" strokeWidth="1.5"/>
      </g>
    </svg>
  );
};

const DistanceRadar = () => (
  <svg viewBox="0 0 200 120" className="w-full h-auto">
    <defs>
      <linearGradient id="rd-g" x1="0" x2="0" y1="0" y2="1">
        <stop offset="0%" stopColor="#7DA1C9" stopOpacity="0.3"/>
        <stop offset="100%" stopColor="#7DA1C9" stopOpacity="0"/>
      </linearGradient>
    </defs>
    {/* arcs */}
    {[1,0.7,0.4].map((s,i)=>(
      <path key={i} d={`M ${100 - 80*s} 110 A ${80*s} ${80*s} 0 0 1 ${100 + 80*s} 110`}
        fill={i===0?'url(#rd-g)':'none'} stroke="hsl(var(--surface-3))" strokeWidth="1"/>
    ))}
    {/* sweep line */}
    <line x1="100" y1="110" x2="170" y2="40" stroke="hsl(var(--info))" strokeWidth="1.5" strokeLinecap="round"/>
    {/* obstacle */}
    <circle cx="135" cy="60" r="4" fill="hsl(var(--warning))"/>
    <circle cx="135" cy="60" r="9" fill="none" stroke="hsl(var(--warning))" strokeOpacity="0.4" className="animate-pulse-soft"/>
    {/* labels */}
    {[80,60,40].map((d,i)=>(
      <text key={i} x="100" y={110 - 80*[1,0.7,0.4][i] + 12} textAnchor="middle"
        fontSize="9" fontFamily="JetBrains Mono" fill="hsl(var(--foreground-faint))">{d}cm</text>
    ))}
    <circle cx="100" cy="110" r="3" fill="hsl(var(--accent))"/>
  </svg>
);

const MotorBars = () => {
  const motors = [
    ['L1', 0.65], ['L2', 0.55], ['R1', -0.35], ['R2', -0.40],
  ];
  return (
    <div className="space-y-2">
      {motors.map(([l,v]) => (
        <div key={l} className="flex items-center gap-2">
          <span className="font-mono text-micro uppercase tracking-wider text-foreground-muted w-6">{l}</span>
          <div className="flex-1 relative h-3 rounded-sm bg-surface-2 overflow-hidden">
            <div className="absolute top-0 bottom-0 w-px bg-foreground-faint left-1/2"/>
            <div className={cn('absolute top-0 bottom-0', v>=0?'bg-accent':'bg-info')}
              style={{
                left: v>=0 ? '50%' : `${50 + v*50}%`,
                width: `${Math.abs(v)*50}%`,
              }}/>
          </div>
          <span className="font-mono text-micro tabular-nums text-foreground-muted w-10 text-right">{v.toFixed(2)}</span>
        </div>
      ))}
    </div>
  );
};

const ArmVisualizer = () => (
  <svg viewBox="0 0 200 140" className="w-full h-auto">
    {/* base */}
    <rect x="80" y="120" width="40" height="12" rx="2" fill="hsl(var(--surface-3))" stroke="hsl(var(--border-strong))"/>
    {/* link 1 */}
    <line x1="100" y1="120" x2="60" y2="60" stroke="hsl(var(--foreground-muted))" strokeWidth="3" strokeLinecap="round"/>
    <circle cx="100" cy="120" r="4" fill="hsl(var(--accent))"/>
    {/* link 2 */}
    <line x1="60" y1="60" x2="135" y2="40" stroke="hsl(var(--foreground-muted))" strokeWidth="3" strokeLinecap="round"/>
    <circle cx="60" cy="60" r="4" fill="hsl(var(--accent))"/>
    {/* gripper */}
    <line x1="135" y1="40" x2="155" y2="32" stroke="hsl(var(--foreground-muted))" strokeWidth="2"/>
    <line x1="135" y1="40" x2="155" y2="52" stroke="hsl(var(--foreground-muted))" strokeWidth="2"/>
    <circle cx="135" cy="40" r="4" fill="hsl(var(--accent))"/>
    {/* labels */}
    <text x="65" y="75" fontSize="9" fontFamily="JetBrains Mono" fill="hsl(var(--foreground-faint))">42°</text>
    <text x="100" y="50" fontSize="9" fontFamily="JetBrains Mono" fill="hsl(var(--foreground-faint))">−18°</text>
  </svg>
);

const SamcanPage = () => (
  <div className="p-6 max-w-[1920px] mx-auto">
    <div className="grid grid-cols-12 gap-4">
      <div className="col-span-12 xl:col-span-4 space-y-4">
        <Card>
          <CardHeader right={<Badge tone="success" dot>connected</Badge>}>
            <CardTitle>Samcan</CardTitle>
          </CardHeader>
          <CardContent className="space-y-2 font-mono text-small">
            <div className="flex justify-between"><span className="text-foreground-muted">port</span><span>/dev/ttyUSB0</span></div>
            <div className="flex justify-between"><span className="text-foreground-muted">baud</span><span className="tabular-nums">115 200</span></div>
            <div className="flex justify-between"><span className="text-foreground-muted">firmware</span><span>2.4.1</span></div>
            <div className="flex justify-between"><span className="text-foreground-muted">uptime</span><span className="tabular-nums">00:42:18</span></div>
          </CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>Heading</CardTitle></CardHeader>
          <CardContent>
            <HeadingCompass heading={42}/>
            <div className="mt-2 flex items-center justify-between font-mono text-small">
              <span className="text-foreground-muted">42° NE</span>
              <span className="text-foreground-faint">±0.5°</span>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>Distance radar</CardTitle></CardHeader>
          <CardContent>
            <DistanceRadar/>
            <div className="mt-2 flex items-center justify-between font-mono text-small">
              <span className="text-foreground-muted">closest</span>
              <span className="text-warning tabular-nums">38 cm @ +20°</span>
            </div>
          </CardContent>
        </Card>
      </div>

      <div className="col-span-12 xl:col-span-5 space-y-4">
        <Card>
          <CardHeader><CardTitle>Arm</CardTitle></CardHeader>
          <CardContent className="grid grid-cols-2 gap-4">
            <ArmVisualizer/>
            <div className="space-y-3">
              <ServoSlider label="base"     value={42}/>
              <ServoSlider label="elbow"    value={68}/>
              <ServoSlider label="gripper"  value={20} max={90}/>
            </div>
          </CardContent>
        </Card>

        <Card>
          <CardHeader right={<Badge tone="accent" dot>active</Badge>}>
            <CardTitle>Motors</CardTitle>
          </CardHeader>
          <CardContent>
            <MotorBars/>
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle>Keyboard</CardTitle>
          </CardHeader>
          <CardContent className="space-y-2">
            <div className="grid grid-cols-3 gap-1.5 max-w-[200px] mx-auto">
              <div/><Kbd className="h-9 w-full">W</Kbd><div/>
              <Kbd className="h-9 w-full">A</Kbd><Kbd className="h-9 w-full">S</Kbd><Kbd className="h-9 w-full">D</Kbd>
            </div>
            <div className="font-mono text-micro text-foreground-faint text-center pt-2">
              hold <Kbd>shift</Kbd> for boost &nbsp;·&nbsp; <Kbd>space</Kbd> stop
            </div>
          </CardContent>
        </Card>
      </div>

      <div className="col-span-12 xl:col-span-3 space-y-4">
        <Card>
          <CardHeader right={<CardSubtitle>last 60s</CardSubtitle>}>
            <CardTitle>Telemetry</CardTitle>
          </CardHeader>
          <CardContent>
            <SensorCharts/>
          </CardContent>
        </Card>

        <Card>
          <CardHeader><CardTitle>Quick</CardTitle></CardHeader>
          <CardContent className="grid grid-cols-2 gap-1.5">
            <Button size="sm">Connect</Button>
            <Button size="sm" variant="secondary">Reset</Button>
            <Button size="sm" variant="secondary">Calibrate</Button>
            <Button size="sm" variant="destructive">Stop</Button>
          </CardContent>
        </Card>

        <Card>
          <CardHeader right={<CardSubtitle>{EVENTS.length} events</CardSubtitle>}>
            <CardTitle>Events</CardTitle>
          </CardHeader>
          <CardContent className="p-2">
            <EventLog/>
          </CardContent>
        </Card>
      </div>
    </div>
  </div>
);

Object.assign(window, {
  AdminPage, Visualization3DPage, HardwarePage, SamcanPage,
  ServoSlider, Toggle: window.Toggle,
});
