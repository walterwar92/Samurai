/* global React, ReactDOM */
const { useState: useStateA, useEffect: useEffectA } = React;

function App() {
  const [theme, setTheme]         = useStateA(localStorage.getItem('samurai.theme') || 'dark');
  const [collapsed, setCollapsed] = useStateA(localStorage.getItem('samurai.sidebarCollapsed') === '1');
  const [active, setActive]       = useStateA(localStorage.getItem('samurai.route') || 'dashboard');
  const [robot, setRobot]         = useStateA('samurai');
  const [fsm, setFsm]             = useStateA('TARGETING');
  const [target, setTarget]       = useStateA('red');
  const [simTime, setSimTime]     = useStateA('00:42:18');
  const connected = true;

  // theme
  useEffectA(() => {
    document.documentElement.classList.toggle('light', theme === 'light');
    localStorage.setItem('samurai.theme', theme);
  }, [theme]);
  useEffectA(() => { localStorage.setItem('samurai.sidebarCollapsed', collapsed ? '1' : '0'); }, [collapsed]);
  useEffectA(() => { localStorage.setItem('samurai.route', active); }, [active]);

  // ticking sim time
  useEffectA(() => {
    const start = Date.now();
    const base = 42*60 + 18;
    const id = setInterval(() => {
      const t = base + Math.floor((Date.now() - start)/1000);
      const hh = String(Math.floor(t/3600)).padStart(2,'0');
      const mm = String(Math.floor(t/60)%60).padStart(2,'0');
      const ss = String(t%60).padStart(2,'0');
      setSimTime(`${hh}:${mm}:${ss}`);
    }, 1000);
    return () => clearInterval(id);
  }, []);

  // FSM cycle (subtle motion in the prototype)
  useEffectA(() => {
    const id = setInterval(() => {
      setFsm(s => {
        const order = ['SEARCHING','TARGETING','APPROACHING','GRABBING','CALLING','RETURNING','IDLE'];
        const i = order.indexOf(s);
        return order[(i+1) % order.length];
      });
    }, 6000);
    return () => clearInterval(id);
  }, []);

  const titleFor = {
    dashboard:'Dashboard', admin:'Admin', '3d':'3D View', hardware:'Hardware', samcan:'Samcan',
  }[active];

  let page = null;
  if (active === 'dashboard') page = <DashboardPage fsm={fsm} target={target} simTime={simTime} connected={connected}/>;
  else if (active === 'admin') page = <AdminPage    fsm={fsm} target={target} simTime={simTime}/>;
  else if (active === '3d')    page = <Visualization3DPage simTime={simTime}/>;
  else if (active === 'hardware') page = <HardwarePage/>;
  else if (active === 'samcan') page = <SamcanPage/>;

  return (
    <div className="flex min-h-screen bg-background text-foreground" data-screen-label={`samurai · ${active}`}>
      <Sidebar
        active={active} setActive={setActive}
        collapsed={collapsed} setCollapsed={setCollapsed}
        robot={robot} setRobot={setRobot}
        theme={theme} setTheme={setTheme}
        connected={connected}
      />
      <div className="flex-1 min-w-0">
        <PageHeader title={titleFor} simTime={simTime}
          right={
            active === 'dashboard'
              ? <Button size="sm" variant="ghost"><RotateIcon className="h-3.5 w-3.5"/>Reset sim</Button>
              : null
          }
        />
        {page}
      </div>
    </div>
  );
}

ReactDOM.createRoot(document.getElementById('root')).render(<App/>);
