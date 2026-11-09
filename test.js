// test.js
import {
  CS, MAX_AGENTS, COST_HIGHWAY, COST_OPEN, COST_MUD, COST_BLOCK,
  clamp, createComlinkSimulation, runBenchmark
} from './flowFieldApi.js';

const createOffscreen = (w, h) => Object.assign(
  typeof OffscreenCanvas !== 'undefined' ? new OffscreenCanvas(w, h) : document.createElement('canvas'),
  { width: w, height: h }
);

(async () => {
//#region 1: BootstrapAndDOMInjection
  const { PIXI, GUI, UPlotClass, Comlink, canvas, minimapCard, minimapCanvas } = await (async () => {
    if (!window.crossOriginIsolated && window.isSecureContext && 'serviceWorker' in navigator) {
      const hasCoiScript = !!document.querySelector('script[src*="coi-serviceworker"]');
      if (!hasCoiScript) {
        await new Promise(resolve => {
          const coiScript = document.createElement('script');
          coiScript.src = new URL('./coi-serviceworker.min.js', import.meta.url).href;
          coiScript.onload = resolve;
          coiScript.onerror = resolve;
          document.head.appendChild(coiScript);
        });
      }
    }

    document.title = 'Hierarchical Flow Field Pathfinding (SAB + Atomics + Comlink)';

    const link = document.createElement('link');
    link.rel = 'stylesheet';
    link.crossOrigin = 'anonymous';
    link.href = 'https://cdn.jsdelivr.net/npm/uplot@1.6.31/dist/uPlot.min.css';
    document.head.appendChild(link);

    const style = document.createElement('style');
    style.textContent = `
*, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
body {
  background: #040711; height: 100dvh; overflow: hidden;
  font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace;
  user-select: none; -webkit-user-select: none;
}
canvas#canvas { display: block; width: 100%; height: 100%; cursor: crosshair; touch-action: none; }
.lil-gui {
  --background-color: rgba(4, 8, 18, 0.94);
  --text-color: #cbd5e1;
  --title-background-color: rgba(15, 23, 42, 0.95);
  --widget-color: rgba(30, 41, 59, 0.85);
  --hover-color: rgba(56, 189, 248, 0.25);
  --focus-color: rgba(56, 189, 248, 0.45);
  --number-color: #38bdf8;
  --string-color: #4ade80;
  border: 1px solid rgba(56, 189, 248, 0.35);
  border-radius: 8px;
  box-shadow: 0 12px 32px rgba(0, 0, 0, 0.85);
  backdrop-filter: blur(12px);
  font-family: ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, monospace !important;
}
.lil-gui .title { font-weight: 700; letter-spacing: 0.5px; color: #f8fafc; }
.lil-gui .lil-children { scrollbar-width: thin; scrollbar-color: rgba(56, 189, 248, 0.3) transparent; }
.lil-gui .lil-children::-webkit-scrollbar { width: 5px; }
.lil-gui .lil-children::-webkit-scrollbar-thumb { background: rgba(56, 189, 248, 0.3); border-radius: 3px; }
.uplot { font-family: inherit !important; user-select: none; }
.uplot .u-wrap { position: relative !important; }
.uplot canvas { display: block; }
#minimap-card {
  position: absolute; bottom: 12px; right: 12px; pointer-events: auto;
  background: rgba(4, 8, 18, 0.94); border: 1px solid rgba(56, 189, 248, 0.35);
  border-radius: 8px; padding: 6px; backdrop-filter: blur(12px);
  box-shadow: 0 12px 32px rgba(0,0,0,0.8); z-index: 10;
  display: flex; flex-direction: column; gap: 4px;
}
#minimap-canvas { display: block; border-radius: 4px; border: 1px solid rgba(56, 189, 248, 0.2); cursor: crosshair; }
`;
    document.head.appendChild(style);

    const minimapCard = document.createElement('div');
    minimapCard.id = 'minimap-card';

    const minimapCanvas = document.createElement('canvas');
    minimapCanvas.id = 'minimap-canvas';
    minimapCanvas.width = 160;
    minimapCanvas.height = 96;
    minimapCard.appendChild(minimapCanvas);

    const canvas = document.createElement('canvas');
    canvas.id = 'canvas';

    document.body.append(minimapCard, canvas);

    const [PIXI, guiMod, uplotMod, Comlink] = await Promise.all([
      import('https://cdn.jsdelivr.net/npm/pixi.js@8.22.0/dist/pixi.min.mjs'),
      import('https://cdn.jsdelivr.net/npm/lil-gui@0.21.0/dist/lil-gui.esm.min.js'),
      import('https://cdn.jsdelivr.net/npm/uplot@1.6.31/dist/uPlot.esm.js'),
      import('https://cdn.jsdelivr.net/npm/comlink@4.4.2/dist/esm/comlink.min.mjs')
    ]);

    const GUI = guiMod.default || guiMod;
    const uPlot = uplotMod.default || uplotMod;
    const UPlotClass = uPlot?.default || uPlot;

    return { PIXI, GUI, UPlotClass, Comlink, canvas, minimapCard, minimapCanvas };
  })();
  //#endregion

  //#region 2: ConfigurationStore
  const BRUSH_COSTS = {
    'Navigate (Set Goal)': -1,
    'Wall (Block)': COST_BLOCK,
    'Highway (Fast)': COST_HIGHWAY,
    'Swamp (Mud)': COST_MUD,
    'Erase (Open)': COST_OPEN
  };

  const CONFIG = (() => ({
    world: {
      seed: 1337666,
      chunksX: 10,
      chunksY: 6,
      preset: 'The Grand Citadel',
      wallDensity: 1.0,
      mudDensity: 1.0
    },
    brush: {
      mode: 'Navigate (Set Goal)',
      radius: 2.0
    },
    simulation: {
      paused: false,
      simSpeed: 3.0,
      agentCount: 10_000,
      spawnPattern: 'Left Flank',
      agentRadius: 0.25,
      maxSpeed: 25.0,
      steeringGain: 15.0,
      separationStrength: 60.0,
      pathBudgetMs: 40.0
    },
    autoGoal: {
      enabled: true,
      mode: 'On Arrival',
      arrivalPct: 0.95,
      delay: 0.5,
      interval: 1.0
    },
    audio: {
      enabled: true
    },
    telemetry: {
      ipcMode: 'Initializing...',
      workerTicks: 0,
      fps: 60,
      simMs: 0.0,
      activeChunks: 0,
      agentCount: 10000,
      movingCount: 0,
      arrivedCount: 0,
      flowCacheCount: 0,
      autoGoalTimer: 'Active',
      selectedId: 0,
      selectedState: 'Moving',
      selectedSpeed: 0.0,
      selectedGoalDist: 0.0,
      selectedCorridor: '--',
      lastBenchmark: 'Ready'
    }
  }))();
  //#endregion

  //#region 3: SoundSynthesizer
  const { audio } = (() => {
    class SoundEngine {
      constructor() { this.ctx = null; }
      init() {
        if (!this.ctx) {
          const AC = window.AudioContext || window.webkitAudioContext;
          if (AC) this.ctx = new AC();
        }
        if (this.ctx && this.ctx.state === 'suspended') this.ctx.resume();
      }
      playGoal() {
        if (!CONFIG.audio.enabled) return;
        this.init();
        if (!this.ctx) return;
        const now = this.ctx.currentTime;
        const osc = this.ctx.createOscillator(), g = this.ctx.createGain();
        osc.type = 'sine';
        osc.frequency.setValueAtTime(320, now);
        osc.frequency.exponentialRampToValueAtTime(880, now + 0.18);
        g.gain.setValueAtTime(0.12, now);
        g.gain.exponentialRampToValueAtTime(0.001, now + 0.28);
        osc.connect(g); g.connect(this.ctx.destination);
        osc.start(now); osc.stop(now + 0.28);
      }
      playArrival() {
        if (!CONFIG.audio.enabled) return;
        this.init();
        if (!this.ctx) return;
        const now = this.ctx.currentTime;
        const osc = this.ctx.createOscillator(), g = this.ctx.createGain();
        osc.type = 'triangle';
        osc.frequency.setValueAtTime(587.33, now);
        osc.frequency.exponentialRampToValueAtTime(1174.66, now + 0.22);
        g.gain.setValueAtTime(0.09, now);
        g.gain.exponentialRampToValueAtTime(0.001, now + 0.3);
        osc.connect(g); g.connect(this.ctx.destination);
        osc.start(now); osc.stop(now + 0.3);
      }
      playClick() {
        if (!CONFIG.audio.enabled) return;
        this.init();
        if (!this.ctx) return;
        const now = this.ctx.currentTime;
        const osc = this.ctx.createOscillator(), g = this.ctx.createGain();
        osc.type = 'square';
        osc.frequency.setValueAtTime(900, now);
        osc.frequency.exponentialRampToValueAtTime(300, now + 0.04);
        g.gain.setValueAtTime(0.05, now);
        g.gain.exponentialRampToValueAtTime(0.001, now + 0.04);
        osc.connect(g); g.connect(this.ctx.destination);
        osc.start(now); osc.stop(now + 0.04);
      }
    }
    return { audio: new SoundEngine() };
  })();
  //#endregion

  //#region 4: SparklineGUIExtension
  (() => {
    GUI.prototype.addSparkline = function(title, options = {}) {
      const height = options.height || 48, maxPoints = options.points || 40;
      const color = options.color || '#38bdf8', fill = options.fill || 'rgba(56, 189, 248, 0.15)', unit = options.unit || '';

      const row = document.createElement('div');
      row.className = 'lil-sparkline-row';
      row.style.cssText = 'padding:6px 8px;border-bottom:1px solid rgba(255,255,255,0.07);display:flex;flex-direction:column;gap:4px;background:rgba(15,23,42,0.45);box-sizing:border-box;width:100%;';

      const header = document.createElement('div');
      header.style.cssText = 'display:flex;justify-content:space-between;align-items:center;font-size:11px;color:#94a3b8;';

      const labelSpan = document.createElement('span');
      labelSpan.textContent = title;
      labelSpan.style.fontWeight = '600';

      const valueSpan = document.createElement('span');
      valueSpan.style.cssText = `color:${color};font-weight:bold;font-family:monospace;font-size:11px;`;
      valueSpan.textContent = `-- ${unit}`;

      header.append(labelSpan, valueSpan);
      row.appendChild(header);

      const plotContainer = document.createElement('div');
      plotContainer.style.cssText = `width:100%;height:${height}px;overflow:hidden;position:relative;border-radius:4px;border:1px solid rgba(56,189,248,0.15);background:rgba(4,8,18,0.7);`;
      row.appendChild(plotContainer);
      (this.$children || this.domElement).appendChild(row);

      const xData = new Float64Array(maxPoints), yData = new Float64Array(maxPoints).fill(0);
      for (let i = 0; i < maxPoints; i++) xData[i] = i;

      const uOpts = {
        width: 290, height,
        cursor: { show: false }, select: { show: false }, legend: { show: false },
        scales: {
          x: { time: false },
          y: {
            range: (u, min, max) => [
              options.min !== undefined ? options.min : Math.floor(Math.min(min, 0)),
              options.max !== undefined ? options.max : Math.ceil(Math.max(max * 1.15, 1))
            ]
          }
        },
        axes: [
          { show: false },
          {
            show: true, stroke: '#64748b', size: 32, font: '9px monospace', ticks: { show: false },
            grid: { stroke: 'rgba(56, 189, 248, 0.08)', width: 1 },
            values: (u, vals) => vals.map(v => v >= 10 ? v.toFixed(0) : v.toFixed(1))
          }
        ],
        series: [{}, { stroke: color, width: 1.5, fill }]
      };

      let plot = null;
      try {
        plot = new UPlotClass(uOpts, [xData, yData], plotContainer);
        if (typeof ResizeObserver !== 'undefined') {
          const ro = new ResizeObserver(entries => {
            for (const entry of entries) {
              const w = entry.contentRect.width;
              if (w > 50 && plot) plot.setSize({ width: w, height });
            }
          });
          ro.observe(plotContainer);
        }
      } catch (err) {
        console.warn('uPlot initialization fallback', err);
      }

      return {
        row, plot,
        push(val) {
          valueSpan.textContent = `${typeof val === 'number' ? (val >= 10 ? val.toFixed(1) : val.toFixed(2)) : val} ${unit}`;
          if (plot) {
            yData.copyWithin(0, 1);
            yData[maxPoints - 1] = val;
            plot.setData([xData, yData]);
          }
        }
      };
    };
  })();
  //#endregion

  //#region 5: ThemeAndColorLUTs
  const { THEME, HEATMAP_LUT, TINTS } = (() => {
    const THEME_RAW = {
      bg: '#040711', gA: '#090f1d', gB: '#0d1527',
      wall: '#1e293b', wallTop: '#334155',
      highway: '#0284c7', highwayLine: '#38bdf8',
      mud: '#3b2d13', mudSpeck: '#714e1a',
      chunkLine: 'rgba(56,189,248,0.18)',
      actFill: 'rgba(56,189,248,0.04)', actStr: 'rgba(56,189,248,0.35)',
      arrP: 'rgba(125,211,252,0.85)', arrG: 'rgba(244,114,182,0.9)',
      pIdle: 'rgba(250,204,21,0.3)', pPath: 'rgba(250,204,21,0.85)', pHot: '#fef08a',
      pGlow: 'rgba(253,224,71,0.25)', corridor: 'rgba(240,171,252,0.6)', aPath: '#f0abfc',
      goal: '#f472b6', moving: '#22d3ee', arrived: '#4ade80', noPath: '#f87171',
      idle: '#94a3b8', graphEdge: 'rgba(250,204,21,0.22)'
    };
    const THEME = Object.fromEntries(Object.entries(THEME_RAW).map(([k, css]) => {
      const c = new PIXI.Color(css); return [k, { num: c.toNumber(), alpha: c.alpha, css }];
    }));
    const HEATMAP_LUT = Array.from({ length: 64 }, (_, i) => {
      const c = new PIXI.Color({ h: 240 - (i * 20 / 7), s: 85, l: 52 });
      const [r, g, b] = c.toRgbArray();
      return { num: c.toNumber(), r: (r * 255) | 0, g: (g * 255) | 0, b: (b * 255) | 0 };
    });
    const TINTS = [0x94a3b8, 0x22d3ee, 0x4ade80, 0xf87171];
    return { THEME, HEATMAP_LUT, TINTS };
  })();
  //#endregion

  //#region 6: VisualPluginSystem
  const { VisualPlugin, VisualPluginManager } = (() => {
    class VisualPlugin {
      constructor(id, name, defaultEnabled = false, phase = 'beforeAgents') {
        this.id = id; this.name = name; this.phase = phase; this._enabled = defaultEnabled;
      }
      get enabled() { return this._enabled; }
      set enabled(val) {
        const b = !!val;
        if (this._enabled === b) return;
        this._enabled = b;
        b ? this.onEnable?.() : this.onDisable?.();
      }
      init(context) {}
      render(context) {}
      onEnable() {}
      onDisable() {}
      onWorldResized(context) {}
    }

    class VisualPluginManager {
      constructor(renderer) {
        this.renderer = renderer;
        this.plugins = new Map();
      }
      register(plugin) {
        this.plugins.set(plugin.id, plugin);
        plugin.init?.({ renderer: this.renderer, world: this.renderer.world, app: this.renderer.app });
        return plugin;
      }
      renderPhase(phase, ctx) {
        for (const plugin of this.plugins.values()) {
          if (plugin.enabled && plugin.phase === phase) plugin.render(ctx);
        }
      }
      onWorldResized(grid) {
        for (const plugin of this.plugins.values()) {
          plugin.onWorldResized?.({ renderer: this.renderer, grid });
        }
      }
      attachGUI(folder) {
        for (const plugin of this.plugins.values()) {
          folder.add(plugin, 'enabled').name(plugin.name);
        }
      }
    }
    return { VisualPlugin, VisualPluginManager };
  })();
  //#endregion

  //#region 7: ConcreteVisualPlugins
  const {
    FlowFieldsPlugin, DistanceHeatmapPlugin, PortalsPlugin, GraphEdgesPlugin,
    ChunkBordersPlugin, SelectedPathPlugin, VelocityTicksPlugin, MinimapPlugin
  } = (() => {
    class FlowFieldsPlugin extends VisualPlugin {
      constructor() {
        super('flowFields', 'Flow Vectors', false, 'beforeAgents');
        this.container = this.bgG = this.pG = this.gG = null;
      }
      init({ world }) {
        this.container = new PIXI.Container();
        this.container.zIndex = 40;
        this.bgG = new PIXI.Graphics(); this.pG = new PIXI.Graphics(); this.gG = new PIXI.Graphics();
        this.container.addChild(this.bgG, this.pG, this.gG);
        this.container.visible = false;
        world.addChild(this.container);
      }
      onDisable() {
        if (this.container) {
          this.bgG.clear(); this.pG.clear(); this.gG.clear();
          this.container.visible = false;
        }
      }
      render({ sim, renderer }) {
        const { bgG, pG, gG } = this, cp = renderer.cellPx;
        bgG.clear(); pG.clear(); gG.clear();
        bgG.visible = pG.visible = gG.visible = false;
        const half = 0.35, head = 0.22, cos = 0.89879, sin = 0.43837, stride = cp < 6 ? 4 : cp < 12 ? 2 : 1;
        let hasP = false, hasG = false, hasBox = false;
        const cMinX = renderer.camera.x - (renderer.cssWidth / 2) / cp - CS;
        const cMaxX = renderer.camera.x + (renderer.cssWidth / 2) / cp + CS;
        const cMinY = renderer.camera.y - (renderer.cssHeight / 2) / cp - CS;
        const cMaxY = renderer.camera.y + (renderer.cssHeight / 2) / cp + CS;
        for (let c = 0; c < sim.grid.chunkCount; c++) {
          if (sim.activeStamp[c] !== sim.frame || !sim.activeFields[c]) continue;
          const f = sim.activeFields[c], { originX: x, originY: y } = f;
          if (x + CS < cMinX || x > cMaxX || y + CS < cMinY || y > cMaxY) continue;
          bgG.rect(x, y, CS, CS).fill({ color: THEME.actFill.num, alpha: THEME.actFill.alpha })
             .rect(x + 0.05, y + 0.05, CS - 0.1, CS - 0.1).stroke({ color: THEME.actStr.num, width: 1 / cp, alpha: THEME.actStr.alpha });
          hasBox = true;
          const isGoal = f.kind === 'g', targetG = isGoal ? gG : pG;
          isGoal ? (hasG = true) : (hasP = true);
          for (let ly = 0; ly < CS; ly += stride) {
            for (let lx = 0; lx < CS; lx += stride) {
              const i = ly * CS + lx, fx = f.flowX[i], fy = f.flowY[i];
              if (!fx && !fy) continue;
              const cx = x + lx + 0.5, cy = y + ly + 0.5, tx = cx + fx * half, ty = cy + fy * half, bx = -fx, by = -fy;
              targetG.moveTo(cx - fx * half, cy - fy * half).lineTo(tx, ty);
              if (stride === 1) {
                targetG.moveTo(tx + (bx * cos - by * sin) * head, ty + (bx * sin + by * cos) * head)
                       .lineTo(tx).lineTo(tx + (bx * cos + by * sin) * head, ty + (-bx * sin + by * cos) * head);
              }
            }
          }
        }
        const aW = Math.max(1, cp * 0.08) / cp;
        const strokeArr = (g, th, has) => (g.visible = has) && g.stroke({ color: th.num, width: aW, alpha: th.alpha, cap: 'round', join: 'round' });
        strokeArr(pG, THEME.arrP, hasP);
        strokeArr(gG, THEME.arrG, hasG);
        bgG.visible = hasBox;
        this.container.visible = hasBox || hasP || hasG;
      }
    }

    class DistanceHeatmapPlugin extends VisualPlugin {
      constructor() {
        super('distanceHeatmap', 'Distance Heatmap', false, 'beforeAgents');
        this.heatCanvas = createOffscreen(16, 16);
        this.heatTexture = this.heatSprite = this.heatCtx = this.heatImg = null;
      }
      init({ world }) {
        this.heatSprite = new PIXI.Sprite(PIXI.Texture.WHITE);
        this.heatSprite.zIndex = 10;
        this.heatSprite.visible = false;
        world.addChild(this.heatSprite);
      }
      onWorldResized() {
        if (this.heatTexture) { this.heatTexture.destroy(true); this.heatTexture = null; }
      }
      onDisable() {
        if (this.heatSprite) this.heatSprite.visible = false;
      }
      render({ sim }) {
        const { grid } = sim, { width: W, height: H } = grid;
        if (this.heatCanvas.width !== W || this.heatCanvas.height !== H) {
          this.heatCanvas.width = W; this.heatCanvas.height = H;
          this.heatCtx = this.heatCanvas.getContext('2d');
          this.heatImg = this.heatCtx.createImageData(W, H);
          if (this.heatTexture) { this.heatTexture.destroy(true); this.heatTexture = null; }
        }
        const img = this.heatImg, data = img.data;
        data.fill(0);
        let drawn = false;
        for (let c = 0; c < grid.chunkCount; c++) {
          if (sim.activeStamp[c] !== sim.frame || !sim.activeFields[c]) continue;
          const f = sim.activeFields[c], ox = f.originX, oy = f.originY;
          for (let ly = 0; ly < CS; ly++) {
            const row = (oy + ly) * W;
            for (let lx = 0; lx < CS; lx++) {
              const d = f.integration[ly * CS + lx];
              if (d < Infinity && d > 0) {
                const idx = (row + ox + lx) * 4;
                const lut = HEATMAP_LUT[Math.min(63, d * 63 / 40 | 0)];
                data[idx] = lut.r; data[idx + 1] = lut.g; data[idx + 2] = lut.b; data[idx + 3] = 95;
                drawn = true;
              }
            }
          }
        }
        if (drawn) {
          this.heatCtx.putImageData(img, 0, 0);
          if (!this.heatTexture) {
            this.heatTexture = PIXI.Texture.from(this.heatCanvas);
            this.heatTexture.source.scaleMode = 'nearest';
            this.heatSprite.texture = this.heatTexture;
          } else {
            this.heatTexture.source.update();
          }
          this.heatSprite.width = W; this.heatSprite.height = H;
          this.heatSprite.visible = true;
        } else {
          this.heatSprite.visible = false;
        }
      }
    }

    class PortalsPlugin extends VisualPlugin {
      constructor() {
        super('portals', 'Portal Boundaries', false, 'beforeAgents');
        this.graphics = null;
      }
      init({ world }) {
        this.graphics = new PIXI.Graphics();
        this.graphics.zIndex = 50;
        this.graphics.visible = false;
        world.addChild(this.graphics);
      }
      onDisable() {
        if (this.graphics) { this.graphics.clear(); this.graphics.visible = false; }
      }
      render({ sim, renderer }) {
        const g = this.graphics, idle = [], path = [], hot = [], cp = renderer.cellPx;
        g.clear();
        for (let i = 0; i < sim.graph.portals.length; i++) {
          const { isH: h, start: s, border: b, end: e, id } = sim.graph.portals[i];
          (sim.portalTargets[id] > 0 ? hot : sim.portalOnPath[id] ? path : idle).push(h ? s : b, h ? b : s, h ? e + 1 : b, h ? b : e + 1);
        }
        const draw = (arr, c, w) => {
          if (!arr.length) return;
          for (let i = 0; i < arr.length; i += 4) g.moveTo(arr[i], arr[i + 1]).lineTo(arr[i + 2], arr[i + 3]);
          g.stroke({ color: c.num, width: w / cp, alpha: c.alpha, cap: 'square' });
        };
        draw(idle, THEME.pIdle, Math.max(2, cp * 0.22));
        draw(path, THEME.pPath, Math.max(2.5, cp * 0.3));
        draw(hot, THEME.pGlow, Math.max(6, cp * 0.95));
        draw(hot, THEME.pHot, Math.max(3, cp * 0.36));
        g.visible = !!(idle.length || path.length || hot.length);
      }
    }

    class GraphEdgesPlugin extends VisualPlugin {
      constructor() {
        super('graphEdges', 'Macro Graph Edges', false, 'beforeAgents');
        this.graphics = null;
      }
      init({ world }) {
        this.graphics = new PIXI.Graphics();
        this.graphics.zIndex = 30;
        this.graphics.visible = false;
        world.addChild(this.graphics);
      }
      onDisable() {
        if (this.graphics) { this.graphics.clear(); this.graphics.visible = false; }
      }
      render({ sim, renderer }) {
        const g = this.graphics, { portals, edgeOffsets, edgeTargets } = sim.graph;
        g.clear();
        let cnt = 0;
        for (let u = 0; u < portals.length; u++) {
          const p = portals[u];
          for (let e = edgeOffsets[u]; e < edgeOffsets[u + 1]; e++) {
            const v = edgeTargets[e];
            if (v > u && portals[v]) { g.moveTo(p.anchorX, p.anchorY).lineTo(portals[v].anchorX, portals[v].anchorY); cnt++; }
          }
        }
        if (cnt > 0) {
          g.visible = true;
          g.stroke({ color: THEME.graphEdge.num, width: 1.5 / renderer.cellPx, alpha: THEME.graphEdge.alpha });
        } else {
          g.visible = false;
        }
      }
    }

    class ChunkBordersPlugin extends VisualPlugin {
      constructor() {
        super('chunkBorders', 'Chunk Borders', false, 'beforeAgents');
        this.graphics = null;
      }
      init({ world }) {
        this.graphics = new PIXI.Graphics();
        this.graphics.zIndex = 20;
        this.graphics.visible = false;
        world.addChild(this.graphics);
      }
      onDisable() {
        if (this.graphics) { this.graphics.clear(); this.graphics.visible = false; }
      }
      render({ sim, renderer }) {
        const g = this.graphics, { cx, cy, width, height } = sim.grid, cp = renderer.cellPx;
        g.clear();
        for (let x = 1; x < cx; x++) g.moveTo(x * CS, 0).lineTo(x * CS, height);
        for (let y = 1; y < cy; y++) g.moveTo(0, y * CS).lineTo(width, y * CS);
        g.stroke({ color: THEME.chunkLine.num, width: 1.5 / cp, alpha: THEME.chunkLine.alpha });
        g.visible = true;
      }
    }

    class SelectedPathPlugin extends VisualPlugin {
      constructor() {
        super('selectedPath', 'Selected Corridor', false, 'beforeAgents');
        this.graphics = null;
      }
      init({ world }) {
        this.graphics = new PIXI.Graphics();
        this.graphics.zIndex = 60;
        this.graphics.visible = false;
        world.addChild(this.graphics);
      }
      onDisable() {
        if (this.graphics) { this.graphics.clear(); this.graphics.visible = false; }
      }
      render({ sim, renderer, primarySel }) {
        const g = this.graphics, A = sim.agents, cp = renderer.cellPx;
        const path = (primarySel >= 0 && primarySel < A.count) ? A.paths[primarySel] : null;
        if (!path || !path.chunks.length || path.graph !== sim.graph) {
          g.clear(); g.visible = false; return;
        }
        g.clear(); g.visible = true;
        const idx = clamp(A.pathIdx[primarySel], 0, path.chunks.length - 1), { grid, graph } = sim;
        for (let k = idx; k < path.chunks.length; k++) {
          const c = path.chunks[k];
          g.rect(grid.chunkOriginX(c) + 0.1, grid.chunkOriginY(c) + 0.1, CS - 0.2, CS - 0.2)
           .stroke({ color: THEME.corridor.num, width: 2 / cp, alpha: THEME.corridor.alpha });
        }
        g.moveTo(A.px[primarySel], A.py[primarySel]);
        const pts = [];
        for (let k = idx; k < path.portals.length; k++) {
          const p = graph.portals[path.portals[k]];
          if (p) pts.push(p);
        }
        for (let i = 0; i < pts.length; i++) g.lineTo(pts[i].anchorX, pts[i].anchorY);
        g.lineTo(path.goalX, path.goalY).stroke({ color: THEME.aPath.num, width: 2.5 / cp, cap: 'round', join: 'round' });
        for (let i = 0; i < pts.length; i++) g.circle(pts[i].anchorX, pts[i].anchorY, Math.max(3, cp * 0.35) / cp);
        if (pts.length) g.fill({ color: THEME.aPath.num });
      }
    }

    class VelocityTicksPlugin extends VisualPlugin {
      constructor() {
        super('velocityTicks', 'Heading Ticks', false, 'afterAgents');
        this.graphics = null;
      }
      init({ world }) {
        this.graphics = new PIXI.Graphics();
        this.graphics.zIndex = 80;
        this.graphics.visible = false;
        world.addChild(this.graphics);
      }
      onDisable() {
        if (this.graphics) { this.graphics.clear(); this.graphics.visible = false; }
      }
      render({ sim, renderer }) {
        const g = this.graphics, cp = renderer.cellPx;
        if (cp < 7) { g.clear(); g.visible = false; return; }
        g.clear();
        const A = sim.agents, n = A.count, rad = CONFIG.simulation.agentRadius;
        const cMinX = renderer.camera.x - (renderer.cssWidth / 2) / cp - 2;
        const cMaxX = renderer.camera.x + (renderer.cssWidth / 2) / cp + 2;
        const cMinY = renderer.camera.y - (renderer.cssHeight / 2) / cp - 2;
        const cMaxY = renderer.camera.y + (renderer.cssHeight / 2) / cp + 2;
        let tCnt = 0;
        for (let i = 0; i < n; i++) {
          if (A.state[i] !== 1) continue;
          const x = A.px[i], y = A.py[i];
          if (x < cMinX || x > cMaxX || y < cMinY || y > cMaxY) continue;
          const spd = Math.hypot(A.vx[i], A.vy[i]);
          if (spd > 0.3) {
            const s = rad * 1.8 / spd;
            g.moveTo(x, y).lineTo(x + A.vx[i] * s, y + A.vy[i] * s);
            if (++tCnt >= 3000) break;
          }
        }
        if (tCnt > 0) {
          g.visible = true;
          g.stroke({ color: 0x082f49, width: 1.5 / cp, cap: 'round', alpha: 0.95 });
        } else {
          g.visible = false;
        }
      }
    }

    class MinimapPlugin extends VisualPlugin {
      constructor(minimapInstance, cardElement) {
        super('minimap', 'Tactical Radar', true, 'post');
        this.minimap = minimapInstance;
        this.card = cardElement;
      }
      init() { if (this.card) this.card.style.display = this.enabled ? 'flex' : 'none'; }
      onEnable() { if (this.card) this.card.style.display = 'flex'; }
      onDisable() { if (this.card) this.card.style.display = 'none'; }
      onWorldResized() { if (this.minimap) this.minimap.bgVersion = -1; }
      render({ sim, renderer }) { if (this.minimap) this.minimap.render(sim, renderer); }
    }

    return {
      FlowFieldsPlugin, DistanceHeatmapPlugin, PortalsPlugin, GraphEdgesPlugin,
      ChunkBordersPlugin, SelectedPathPlugin, VelocityTicksPlugin, MinimapPlugin
    };
  })();
  //#endregion

  //#region 8: PixiWorldRenderer
  const Renderer = (() => {
    class Renderer {
      constructor(app) {
        this.app = app;
        this.cssWidth = this.cssHeight = this.screenX = this.screenY = this.lastActiveCount = 0;
        this.cellPx = this.baseCellPx = 1;
        this.mapVersion = -1;
        this.mapGrid = null;
        this.mapCanvas = createOffscreen(16, 16);
        this.camera = { x: 0, y: 0, zoom: 1 };
        this.followSelected = false;
        this.selectedAgents = new Set([0]);
        this.boxSelectRect = null;

        this.world = new PIXI.Container();
        this.world.sortableChildren = true;
        app.stage.addChild(this.world);

        this.mapSprite = new PIXI.Sprite(PIXI.Texture.WHITE);
        this.mapSprite.zIndex = 0;
        this.world.addChild(this.mapSprite);

        this.pluginManager = new VisualPluginManager(this);
        this.pluginManager.register(new FlowFieldsPlugin());
        this.pluginManager.register(new DistanceHeatmapPlugin());
        this.pluginManager.register(new PortalsPlugin());
        this.pluginManager.register(new GraphEdgesPlugin());
        this.pluginManager.register(new ChunkBordersPlugin());
        this.pluginManager.register(new SelectedPathPlugin());

        this.agentContainer = new PIXI.Container();
        this.agentContainer.sortableChildren = false;
        this.agentContainer.zIndex = 70;
        this.world.addChild(this.agentContainer);

        this.pluginManager.register(new VelocityTicksPlugin());

        const G = () => new PIXI.Graphics();
        this.selectionG = G(); this.selectionG.zIndex = 90;
        this.boxSelectG = G(); this.boxSelectG.zIndex = 100;
        this.goalG = G(); this.goalG.zIndex = 110;
        this.hoverG = G(); this.hoverG.zIndex = 120;
        this.world.addChild(this.selectionG, this.boxSelectG, this.goalG, this.hoverG);

        const c = createOffscreen(32, 32), ctx = c.getContext('2d');
        ctx.arc(16, 16, 13, 0, Math.PI * 2);
        ctx.fillStyle = '#ffffff'; ctx.fill();
        ctx.lineWidth = 2.5; ctx.strokeStyle = 'rgba(0,0,0,0.45)'; ctx.stroke();
        const tex = PIXI.Texture.from(c);
        this.agentSprites = Array.from({ length: MAX_AGENTS }, () => {
          const sp = new PIXI.Sprite(tex);
          sp.anchor.set(0.5); sp.visible = false; sp._tintNum = -1;
          return this.agentContainer.addChild(sp);
        });
        this.updateAgentBaseScale(0.25);
      }
      updateAgentBaseScale(rad) {
        const s = (rad * 2.2) / 32;
        for (let i = 0; i < MAX_AGENTS; i++) this.agentSprites[i].scale.set(s);
      }
      hide(...gs) { for (const g of gs) { g.clear?.(); g.visible = false; } }
      resetCamera(grid) { this.camera = { x: grid.width / 2, y: grid.height / 2, zoom: 1 }; }
      resize(w, h) { this.cssWidth = Math.max(1, w); this.cssHeight = Math.max(1, h); }
      panBy(dx, dy) { this.camera.x -= dx / this.cellPx; this.camera.y -= dy / this.cellPx; }
      screenToWorld(sx, sy) { return { x: (sx - this.screenX) / this.cellPx, y: (sy - this.screenY) / this.cellPx }; }
      zoomAt(sx, sy, f) {
        const b = this.screenToWorld(sx, sy), p = (this.baseCellPx * (this.camera.zoom = clamp(this.camera.zoom * f, 0.25, 14)));
        this.camera.x = b.x - (sx - this.cssWidth / 2) / p;
        this.camera.y = b.y - (sy - this.cssHeight / 2) / p;
      }
      onWorldResized(grid) {
        this.resetCamera(grid);
        this.mapVersion = -1;
        this.mapGrid = null;
        if (this.mapTexture) { this.mapTexture.destroy(true); this.mapTexture = null; }
        this.pluginManager.onWorldResized(grid);
      }
      render(sim, primarySel, hover, time) {
        const { grid } = sim;
        if (this.followSelected && primarySel >= 0 && primarySel < sim.agents.count) {
          this.camera.x += (sim.agents.px[primarySel] - this.camera.x) * 0.15;
          this.camera.y += (sim.agents.py[primarySel] - this.camera.y) * 0.15;
        }
        this.cellPx = (this.baseCellPx = Math.min(this.cssWidth / grid.width, this.cssHeight / grid.height) * 0.94) * this.camera.zoom;
        this.world.position.set(this.screenX = this.cssWidth / 2 - this.camera.x * this.cellPx, this.screenY = this.cssHeight / 2 - this.camera.y * this.cellPx);
        this.world.scale.set(this.cellPx);

        if (this.mapGrid !== grid || this.mapVersion !== grid.version) this.rebuildMapLayer(grid);
        this.mapSprite.width = grid.width; this.mapSprite.height = grid.height;

        const renderCtx = { sim, renderer: this, primarySel, hover, time, cellPx: this.cellPx };
        this.pluginManager.renderPhase('beforeAgents', renderCtx);

        this.agentContainer.visible = true;
        this.drawAgents(sim, primarySel);

        this.pluginManager.renderPhase('afterAgents', renderCtx);

        this.drawGoal(sim, time);
        this.drawHover(sim, hover);
        this.drawBoxSelect();

        this.app.renderer.render({ container: this.app.stage });
        this.pluginManager.renderPhase('post', renderCtx);
      }
      rebuildMapLayer(grid) {
        this.mapGrid = grid; this.mapVersion = grid.version;
        const layer = this.mapCanvas, { width: W, height: H, cx, cy, costs } = grid, TP = 8, CTP = CS * TP;
        layer.width = W * TP; layer.height = H * TP;
        const ctx = layer.getContext('2d');
        for (let y = 0; y < cy; y++) {
          for (let x = 0; x < cx; x++) {
            ctx.fillStyle = (x + y) % 2 ? THEME.gB.css : THEME.gA.css;
            ctx.fillRect(x * CTP, y * CTP, CTP, CTP);
          }
        }
        for (let y = 0; y < H; y++) {
          for (let x = 0; x < W; x++) {
            const c = costs[y * W + x], px = x * TP, py = y * TP;
            if (c >= COST_BLOCK) {
              ctx.fillStyle = THEME.wall.css; ctx.fillRect(px, py, TP, TP);
              if (!y || costs[(y - 1) * W + x] < COST_BLOCK) {
                ctx.fillStyle = THEME.wallTop.css; ctx.fillRect(px, py, TP, 2);
              }
            } else if (c === COST_HIGHWAY) {
              ctx.fillStyle = THEME.highway.css; ctx.fillRect(px, py, TP, TP);
              ctx.fillStyle = THEME.highwayLine.css; ctx.fillRect(px + 2, py + 2, TP - 4, TP - 4);
            } else if (c === COST_MUD) {
              ctx.fillStyle = THEME.mud.css; ctx.fillRect(px, py, TP, TP);
              if (!((x * 7 + y * 13) & 3)) {
                ctx.fillStyle = THEME.mudSpeck.css; ctx.fillRect(px + 2, py + 4, 2, 2);
              }
            }
          }
        }
        if (this.mapTexture) this.mapTexture.destroy(true);
        this.mapTexture = PIXI.Texture.from(layer);
        this.mapTexture.source.scaleMode = 'nearest';
        this.mapSprite.texture = this.mapTexture;
        this.mapSprite.width = grid.width;
        this.mapSprite.height = grid.height;
      }
      drawAgents(sim, sel) {
        const A = sim.agents, n = A.count, cp = this.cellPx;
        const cMinX = this.camera.x - (this.cssWidth / 2) / cp - 2;
        const cMaxX = this.camera.x + (this.cssWidth / 2) / cp + 2;
        const cMinY = this.camera.y - (this.cssHeight / 2) / cp - 2;
        const cMaxY = this.camera.y + (this.cssHeight / 2) / cp + 2;
        for (let i = 0; i < n; i++) {
          const sp = this.agentSprites[i], x = A.px[i], y = A.py[i];
          if (x < cMinX || x > cMaxX || y < cMinY || y > cMaxY) {
            if (sp.visible) sp.visible = false;
            continue;
          }
          const st = A.state[i];
          sp.visible = true; sp.x = x; sp.y = y;
          const targetCol = TINTS[st] || 0x94a3b8;
          if (sp._tintNum !== targetCol) sp.tint = sp._tintNum = targetCol;
        }
        for (let i = n; i < this.lastActiveCount; i++) this.agentSprites[i].visible = false;
        this.lastActiveCount = n;
        this.selectionG.clear();
        if (this.selectedAgents.size > 0) {
          this.selectionG.visible = true;
          let rings = 0;
          for (const idx of this.selectedAgents) {
            if (idx < n) {
              const isPrimary = (idx === sel);
              this.selectionG.circle(A.px[idx], A.py[idx], Math.max(5, cp * 0.75) / cp)
                .stroke({ color: isPrimary ? 0xffffff : 0xfacc15, width: (isPrimary ? 2.5 : 1.5) / cp });
              if (++rings > 250) break;
            }
          }
        } else {
          this.selectionG.visible = false;
        }
      }
      drawBoxSelect() {
        const g = this.boxSelectG, r = this.boxSelectRect;
        if (!r) { this.hide(g); return; }
        g.clear(); g.visible = true;
        const x = Math.min(r.x0, r.x1), y = Math.min(r.y0, r.y1), w = Math.abs(r.x1 - r.x0), h = Math.abs(r.y1 - r.y0);
        g.rect(x, y, w, h).fill({ color: 0x38bdf8, alpha: 0.15 }).stroke({ color: 0x38bdf8, width: 1.5 / this.cellPx });
      }
      drawGoal(sim, time) {
        const g = this.goalG, { goal, grid } = sim, cp = this.cellPx;
        if (!goal) return this.hide(g);
        g.clear(); g.visible = true;
        const gx = goal.x + 0.5, gy = goal.y + 0.5, arm = Math.max(6 / cp, 1.4);
        g.rect(grid.chunkOriginX(goal.chunk) + 0.05, grid.chunkOriginY(goal.chunk) + 0.05, CS - 0.1, CS - 0.1)
         .stroke({ color: THEME.goal.num, width: 1 / cp, alpha: 0.4 });
        for (let k = 0; k < 2; k++) {
          const t = (time * 0.9 + k * 0.5) % 1;
          g.circle(gx, gy, 0.8 + t * 3.2).stroke({ color: THEME.goal.num, width: 2 / cp, alpha: (1 - t) * 0.85 });
        }
        g.moveTo(gx - arm, gy).lineTo(gx - arm * 0.45, gy).moveTo(gx + arm * 0.45, gy).lineTo(gx + arm, gy)
         .moveTo(gx, gy - arm).lineTo(gx, gy - arm * 0.45).moveTo(gx, gy + arm * 0.45).lineTo(gx, gy + arm)
         .stroke({ color: THEME.goal.num, width: 2.2 / cp })
         .circle(gx, gy, Math.max(3 / cp, 0.32)).fill({ color: THEME.goal.num });
      }
      drawHover(sim, hover) {
        const g = this.hoverG, x = Math.floor(hover.x), y = Math.floor(hover.y);
        this.hide(g);
        if (!Number.isFinite(hover.x) || !sim.grid.inBounds(x, y)) return;
        const bCost = BRUSH_COSTS[CONFIG.brush.mode];
        if (bCost >= 0) {
          g.clear(); g.visible = true;
          const col = bCost === COST_BLOCK ? 0xf87171 : bCost === COST_HIGHWAY ? 0x38bdf8 : bCost === COST_MUD ? 0xf59e0b : 0x4ade80;
          g.circle(hover.x, hover.y, CONFIG.brush.radius)
           .fill({ color: col, alpha: 0.18 })
           .stroke({ color: col, width: 1.8 / this.cellPx, alpha: 0.85 });
          return;
        }
        if (!sim.grid.isWalkable(x, y)) return;
        g.clear(); g.visible = true;
        g.rect(x, y, 1, 1).stroke({ color: THEME.goal.num, width: 1.5 / this.cellPx, alpha: 0.65 });
      }
    }
    return Renderer;
  })();
  //#endregion

  //#region 9: TacticalMinimap
  const Minimap = (() => {
    class Minimap {
      constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.bgCanvas = createOffscreen(canvas.width, canvas.height);
        this.bgCtx = this.bgCanvas.getContext('2d');
        this.bgVersion = -1;
      }
      updateBackground(grid) {
        const { bgCtx, bgCanvas } = this, cw = bgCanvas.width, ch = bgCanvas.height;
        bgCtx.fillStyle = '#040711';
        bgCtx.fillRect(0, 0, cw, ch);
        const sx = cw / grid.width, sy = ch / grid.height;
        for (let y = 0; y < grid.height; y++) {
          for (let x = 0; x < grid.width; x++) {
            const c = grid.costs[y * grid.width + x];
            if (c >= COST_BLOCK) {
              bgCtx.fillStyle = '#1e293b'; bgCtx.fillRect(x * sx, y * sy, Math.max(1, sx), Math.max(1, sy));
            } else if (c === COST_HIGHWAY) {
              bgCtx.fillStyle = '#0284c7'; bgCtx.fillRect(x * sx, y * sy, Math.max(1, sx), Math.max(1, sy));
            } else if (c === COST_MUD) {
              bgCtx.fillStyle = '#451a03'; bgCtx.fillRect(x * sx, y * sy, Math.max(1, sx), Math.max(1, sy));
            }
          }
        }
        this.bgVersion = grid.version;
      }
      render(sim, renderer) {
        if (this.bgVersion !== sim.grid.version) this.updateBackground(sim.grid);
        const { ctx, canvas } = this, { grid, agents } = sim;
        const cw = canvas.width, ch = canvas.height;
        ctx.drawImage(this.bgCanvas, 0, 0);
        const sx = cw / grid.width, sy = ch / grid.height;
        ctx.fillStyle = '#38bdf8';
        const n = agents.count, step = n > 5000 ? 3 : n > 2000 ? 2 : 1;
        for (let i = 0; i < n; i += step) ctx.fillRect(agents.px[i] * sx, agents.py[i] * sy, 1.2, 1.2);
        if (sim.goal) {
          ctx.fillStyle = '#f472b6';
          ctx.beginPath();
          ctx.arc((sim.goal.x + 0.5) * sx, (sim.goal.y + 0.5) * sy, 3.5, 0, Math.PI * 2);
          ctx.fill();
        }
        const cp = renderer.cellPx;
        ctx.strokeStyle = '#facc15';
        ctx.lineWidth = 1;
        ctx.strokeRect(
          (renderer.camera.x - (renderer.cssWidth / 2) / cp) * sx,
          (renderer.camera.y - (renderer.cssHeight / 2) / cp) * sy,
          (renderer.cssWidth / cp) * sx,
          (renderer.cssHeight / cp) * sy
        );
      }
    }
    return Minimap;
  })();
  //#endregion

  //#region 10: TelemetryAndControlsGUI
  const setupGUIAndHelpers = (() => {
    return (sim, renderer) => {
      function selectAgent(id) {
        CONFIG.telemetry.selectedId = id;
        sim.setSelectedId(id);
      }

      function setAgentCount(count) {
        CONFIG.simulation.agentCount = count;
        CONFIG.telemetry.agentCount = count;
        sim.setAgentCount(count);
        if (CONFIG.telemetry.selectedId >= count) selectAgent(0);
        audio.playClick();
      }

      async function rebuildWorld() {
        CONFIG.world.chunksX = clamp(Math.round(CONFIG.world.chunksX), 3, 24);
        CONFIG.world.chunksY = clamp(Math.round(CONFIG.world.chunksY), 3, 18);
        await sim.regenerate(CONFIG.world.seed, CONFIG.world.chunksX, CONFIG.world.chunksY, { ...CONFIG.world });
        renderer.onWorldResized(sim.grid);
        if (CONFIG.telemetry.selectedId >= sim.agents.count) selectAgent(0);
        renderer.selectedAgents.clear();
        renderer.selectedAgents.add(CONFIG.telemetry.selectedId);
      }

      const syncAutoGoal = () => sim.updateParams({ autoGoal: { ...CONFIG.autoGoal } });

      const metricsGui = new GUI({ title: 'Metrics & Telemetry', width: 315 });
      metricsGui.domElement.style.cssText = 'position:fixed;left:12px;right:auto;top:12px;z-index:1000;max-height:calc(100vh - 24px);overflow-y:auto;';

      const fPerf = metricsGui.addFolder('Real-time Diagnostics (Comlink + SAB)');
      const sparkFps = fPerf.addSparkline('Render Framerate', { color: '#38bdf8', unit: 'FPS', min: 0, max: 75, height: 46 });
      const sparkSim = fPerf.addSparkline('Worker Step Time', { color: '#a78bfa', unit: 'ms', min: 0, max: 16, height: 46 });
      fPerf.add(CONFIG.telemetry, 'ipcMode').name('Memory / IPC').listen().disable();
      fPerf.add(CONFIG.telemetry, 'workerTicks').name('Atomic Heartbeat').listen().disable();
      fPerf.add(CONFIG.telemetry, 'lastBenchmark').name('Benchmark').listen().disable();
      fPerf.add({
        bench: () => {
          CONFIG.telemetry.lastBenchmark = 'Running...';
          setTimeout(() => {
            const table = runBenchmark({ agentCount: 5000, steps: 20 });
            console.table(table);
            const avg = table.reduce((s, r) => s + r['Surgical Edit (ms)'], 0) / table.length;
            CONFIG.telemetry.lastBenchmark = `Edit: ${avg.toFixed(2)}ms (See Console)`;
          }, 16);
        }
      }, 'bench').name('Run Engine Benchmark');

      const fSwarm = metricsGui.addFolder('Swarm Telemetry');
      fSwarm.add({ set1k: () => setAgentCount(1000) }, 'set1k').name('Preset: 1,000 Agents');
      fSwarm.add({ set5k: () => setAgentCount(5000) }, 'set5k').name('Preset: 5,000 Agents');
      fSwarm.add({ set10k: () => setAgentCount(10000) }, 'set10k').name('Preset: 10,000 Agents');
      fSwarm.add(CONFIG.telemetry, 'agentCount').name('Total Agents').listen().disable();
      fSwarm.add(CONFIG.telemetry, 'movingCount').name('Moving Agents').listen().disable();
      fSwarm.add(CONFIG.telemetry, 'arrivedCount').name('Arrived at Goal').listen().disable();
      fSwarm.add(CONFIG.telemetry, 'activeChunks').name('Active Chunks').listen().disable();
      fSwarm.add(CONFIG.telemetry, 'flowCacheCount').name('Flow Cache Fields').listen().disable();

      const fAutoTel = metricsGui.addFolder('Auto Goal Status');
      fAutoTel.add(CONFIG.telemetry, 'autoGoalTimer').name('Timer / Status').listen().disable();
      fAutoTel.add({ pick: () => sim.pickRandomGoal() }, 'pick').name('Pick Random Goal');

      const fInspect = metricsGui.addFolder('Agent Inspector');
      fInspect.add(CONFIG.telemetry, 'selectedId').name('Inspected Agent ID').listen().disable();
      fInspect.add(CONFIG.telemetry, 'selectedState').name('Agent State').listen().disable();
      fInspect.add(CONFIG.telemetry, 'selectedSpeed').name('Speed (u/s)').listen().disable();
      fInspect.add(CONFIG.telemetry, 'selectedGoalDist').name('Dist to Goal').listen().disable();
      fInspect.add(CONFIG.telemetry, 'selectedCorridor').name('Corridor').listen().disable();

      fInspect.add({
        selectNext: () => {
          if (!sim.agents.count) return;
          selectAgent((CONFIG.telemetry.selectedId + 1) % sim.agents.count);
          renderer.selectedAgents.clear(); renderer.selectedAgents.add(CONFIG.telemetry.selectedId);
        }
      }, 'selectNext').name('Inspect Next');
      fInspect.add({
        selectRandom: () => {
          if (!sim.agents.count) return;
          selectAgent(Math.floor(Math.random() * sim.agents.count));
          renderer.selectedAgents.clear(); renderer.selectedAgents.add(CONFIG.telemetry.selectedId);
        }
      }, 'selectRandom').name('Inspect Random');
      fInspect.add({
        selectAll: () => {
          renderer.selectedAgents.clear();
          for (let i = 0; i < Math.min(250, sim.agents.count); i++) renderer.selectedAgents.add(i);
        }
      }, 'selectAll').name('Select Sample (250)');
      fInspect.add({ clearSelect: () => renderer.selectedAgents.clear() }, 'clearSelect').name('Clear Selection');

      const fActions = metricsGui.addFolder('Tactical Operations');
      fActions.add(CONFIG.simulation, 'paused').name('Pause Simulation').onChange(v => sim.setPaused(v));
      fActions.add(renderer, 'followSelected').name('Follow Selected Cam').listen();
      fActions.add({ resetCam: () => renderer.resetCamera(sim.grid) }, 'resetCam').name('Reset Camera View');
      fActions.add(CONFIG.audio, 'enabled').name('Sound FX Enabled');

      const controlsGui = new GUI({ title: 'HFFP Simulator Controls', width: 310 });

      const fBrush = controlsGui.addFolder('Dynamic Terrain Brush');
      fBrush.add(CONFIG.brush, 'mode', Object.keys(BRUSH_COSTS)).name('Left-Click Tool');
      fBrush.add(CONFIG.brush, 'radius', 0.5, 8.0, 0.5).name('Brush Radius');

      const fW = controlsGui.addFolder('World & Architecture');
      fW.add(CONFIG.world, 'chunksX', 3, 24, 1).name('Chunks Width (X)').onChange(rebuildWorld);
      fW.add(CONFIG.world, 'chunksY', 3, 18, 1).name('Chunks Height (Y)').onChange(rebuildWorld);
      fW.add(CONFIG.world, 'preset', [
        'The Grand Citadel', 'The Great Maze', 'Twin Hourglass Bottlenecks', 'Highway vs Swamp', 'Open Steppes'
      ]).name('Scenario Preset').onChange(rebuildWorld);
      fW.add(CONFIG.world, 'wallDensity', 0.1, 2.5, 0.1).name('Wall Density').onChange(rebuildWorld);
      fW.add(CONFIG.world, 'mudDensity', 0.0, 3.0, 0.1).name('Mud Density').onChange(rebuildWorld);
      fW.add(CONFIG.world, 'seed', 1, 999999, 1).name('Random Seed').onChange(rebuildWorld);
      fW.add({ rebuildWorld }, 'rebuildWorld').name('Rebuild World');

      const fS = controlsGui.addFolder('Swarm & Physics');
      fS.add(CONFIG.simulation, 'simSpeed', 0.1, 10.0, 0.1).name('Speed Multiplier').onChange(simSpeed => sim.updateParams({ simSpeed }));
      fS.add(CONFIG.simulation, 'agentCount', 100, MAX_AGENTS, 100).name('Total Agents').listen().onChange(setAgentCount);
      fS.add(CONFIG.simulation, 'spawnPattern', ['Left Flank', 'Scatter Everywhere', 'Four Corners', 'Center Cluster']).name('Spawn Layout').onChange(spawnPattern => {
        sim.updateParams({ spawnPattern });
        sim.respawnAll();
      });
      fS.add(CONFIG.simulation, 'agentRadius', 0.15, 0.55, 0.05).name('Agent Radius').onChange(agentRadius => {
        sim.updateParams({ agentRadius });
        renderer.updateAgentBaseScale(agentRadius);
      });
      fS.add(CONFIG.simulation, 'maxSpeed', 1, 50, 0.5).name('Max Speed').onChange(maxSpeed => sim.updateParams({ maxSpeed }));
      fS.add(CONFIG.simulation, 'steeringGain', 2, 50, 0.5).name('Steering Gain').onChange(steeringGain => sim.updateParams({ steeringGain }));
      fS.add(CONFIG.simulation, 'separationStrength', 0, 60, 1).name('Repulsion Force').onChange(separationStrength => sim.updateParams({ separationStrength }));
      fS.add(CONFIG.simulation, 'pathBudgetMs', 0.5, 50, 0.5).name('Path Budget (ms)').onChange(pathBudgetMs => sim.updateParams({ pathBudgetMs }));

      const fG = controlsGui.addFolder('Auto Goal Routing');
      fG.add(CONFIG.autoGoal, 'enabled').name('Enable Auto Goals').onChange(syncAutoGoal);
      fG.add(CONFIG.autoGoal, 'mode', ['Either', 'On Arrival', 'Timer']).name('Trigger Mode').onChange(syncAutoGoal);
      fG.add(CONFIG.autoGoal, 'arrivalPct', 0.1, 1.0, 0.05).name('Arrival Target (Ratio)').onChange(syncAutoGoal);
      fG.add(CONFIG.autoGoal, 'delay', 0.2, 5.0, 0.1).name('Post-Arrival Pause (s)').onChange(syncAutoGoal);
      fG.add(CONFIG.autoGoal, 'interval', 3, 30, 1).name('Max Interval (s)').onChange(syncAutoGoal);

      const fPlugins = controlsGui.addFolder('Visual Plugins');
      renderer.pluginManager.attachGUI(fPlugins);

      return { sparkFps, sparkSim, selectAgent, findAgentNear: (wx, wy, r) => sim.findAgentNear(wx, wy, r) };
    };
  })();
  //#endregion

  //#region 11: PointerAndViewportInteraction
  const attachInteraction = (() => {
    return ({ canvas, minimapCanvas: mCanvas, sim, renderer, hover, findAgentNear, selectAgent }) => {
      let isDown = false, isBoxSelecting = false, isPainting = false, dragged = false, isPanning = false;
      let startX = 0, startY = 0, lastX = 0, lastY = 0, boxStart = { x: 0, y: 0 };

      const toScreen = e => {
        const r = canvas.getBoundingClientRect();
        return [e.clientX - r.left, e.clientY - r.top];
      };
      const toWorld = e => renderer.screenToWorld(...toScreen(e));
      const activePointers = new Map();
      let initDist = 0;

      canvas.onpointerdown = e => {
        audio.init();
        canvas.setPointerCapture?.(e.pointerId);
        activePointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
        const w = toWorld(e);

        if (e.shiftKey) {
          isBoxSelecting = true;
          boxStart = { x: w.x, y: w.y };
          renderer.boxSelectRect = { x0: w.x, y0: w.y, x1: w.x, y1: w.y };
          return;
        }

        if (e.button === 2 || e.button === 1) {
          isPanning = true;
          lastX = e.clientX; lastY = e.clientY;
          return;
        }

        const bCost = BRUSH_COSTS[CONFIG.brush.mode];
        if (bCost >= 0 && activePointers.size === 1) {
          isPainting = true;
          Object.assign(hover, w);
          sim.paintTerrain(w.x, w.y, CONFIG.brush.radius, bCost);
          return;
        }

        if (activePointers.size === 1) {
          isDown = true;
          dragged = false;
          startX = lastX = e.clientX;
          startY = lastY = e.clientY;
          Object.assign(hover, w);
        } else if (activePointers.size === 2) {
          const [p1, p2] = [...activePointers.values()];
          initDist = Math.hypot(p1.x - p2.x, p1.y - p2.y);
        }
      };

      canvas.onpointermove = e => {
        if (activePointers.has(e.pointerId)) activePointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
        const w = toWorld(e);
        Object.assign(hover, w);

        if (isPanning) {
          renderer.panBy(e.clientX - lastX, e.clientY - lastY);
          if (renderer.followSelected) renderer.followSelected = false;
          lastX = e.clientX; lastY = e.clientY;
          return;
        }

        if (isPainting) {
          const bCost = BRUSH_COSTS[CONFIG.brush.mode];
          if (bCost >= 0) sim.paintTerrain(w.x, w.y, CONFIG.brush.radius, bCost);
          return;
        }

        if (isBoxSelecting) {
          renderer.boxSelectRect = { x0: boxStart.x, y0: boxStart.y, x1: w.x, y1: w.y };
          return;
        }

        if (activePointers.size === 2) {
          const [p1, p2] = [...activePointers.values()];
          const d = Math.hypot(p1.x - p2.x, p1.y - p2.y);
          if (initDist > 0 && d > 0) {
            renderer.zoomAt((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, d / initDist);
            initDist = d;
          }
          dragged = true;
          return;
        }

        if (isDown) {
          if (!dragged && Math.hypot(e.clientX - startX, e.clientY - startY) >= 4) dragged = true;
          if (dragged) {
            renderer.panBy(e.clientX - lastX, e.clientY - lastY);
            if (renderer.followSelected) renderer.followSelected = false;
          }
          lastX = e.clientX; lastY = e.clientY;
        }
      };

      const onUp = e => {
        activePointers.delete(e.pointerId);
        const w = toWorld(e);
        if (e.button === 2 || e.button === 1) isPanning = false;
        if (isPainting) { isPainting = false; return; }

        if (isBoxSelecting) {
          isBoxSelecting = false;
          const r = renderer.boxSelectRect;
          renderer.boxSelectRect = null;
          if (r) {
            const minX = Math.min(r.x0, r.x1), maxX = Math.max(r.x0, r.x1);
            const minY = Math.min(r.y0, r.y1), maxY = Math.max(r.y0, r.y1);
            renderer.selectedAgents.clear();
            const A = sim.agents;
            for (let i = 0; i < A.count; i++) {
              if (A.px[i] >= minX && A.px[i] <= maxX && A.py[i] >= minY && A.py[i] <= maxY) {
                renderer.selectedAgents.add(i);
              }
            }
            if (renderer.selectedAgents.size > 0) {
              selectAgent(renderer.selectedAgents.values().next().value);
            }
          }
          return;
        }

        if (isDown && !dragged) {
          const clickedAgent = findAgentNear(hover.x, hover.y, 1.4);
          if (clickedAgent >= 0) {
            selectAgent(clickedAgent);
            renderer.selectedAgents.clear();
            renderer.selectedAgents.add(clickedAgent);
          } else {
            sim.setGoal(w.x, w.y, true);
          }
        }

        if (activePointers.size === 0) isDown = dragged = false;
        canvas.hasPointerCapture?.(e.pointerId) && canvas.releasePointerCapture(e.pointerId);
      };

      canvas.onpointerup = canvas.onpointercancel = onUp;
      canvas.onpointerleave = () => { hover.x = hover.y = NaN; };
      canvas.oncontextmenu = e => e.preventDefault();
      canvas.addEventListener('wheel', e => {
        e.preventDefault();
        renderer.zoomAt(...toScreen(e), e.deltaY < 0 ? 1.15 : 0.87);
      }, { passive: false });

      let minimapDragging = false;
      const handleMinimapPan = e => {
        const r = mCanvas.getBoundingClientRect();
        renderer.camera.x = clamp((e.clientX - r.left) / r.width, 0, 1) * sim.grid.width;
        renderer.camera.y = clamp((e.clientY - r.top) / r.height, 0, 1) * sim.grid.height;
        if (renderer.followSelected) renderer.followSelected = false;
      };
      mCanvas.onpointerdown = e => { minimapDragging = true; mCanvas.setPointerCapture?.(e.pointerId); handleMinimapPan(e); };
      mCanvas.onpointermove = e => { if (minimapDragging) handleMinimapPan(e); };
      mCanvas.onpointerup = mCanvas.onpointercancel = e => {
        minimapDragging = false;
        mCanvas.hasPointerCapture?.(e.pointerId) && mCanvas.releasePointerCapture(e.pointerId);
      };
    };
  })();
  //#endregion

  //#region 12: MainSimulationLoop
  await (async () => {
    const app = new PIXI.Application();
    await app.init({ canvas, resizeTo: window, autoDensity: true, antialias: false, backgroundColor: 0x040711 });
    app.ticker?.stop?.();

    const sim = await createComlinkSimulation(Comlink, {
      ...CONFIG.world,
      ...CONFIG.simulation,
      autoGoal: CONFIG.autoGoal,
      onGoal: () => audio.playGoal(),
      onArrival: () => audio.playArrival()
    });
    CONFIG.telemetry.ipcMode = sim.arena.isShared ? 'SAB + Atomics + Comlink' : 'Comlink + Clone Fallback';

    const renderer = new Renderer(app);
    renderer.resetCamera(sim.grid);

    const minimap = new Minimap(minimapCanvas);
    renderer.pluginManager.register(new MinimapPlugin(minimap, minimapCard));

    const hover = { x: NaN, y: NaN };

    const onResize = () => renderer.resize(innerWidth, innerHeight);
    window.onresize = onResize;
    onResize();

    const { sparkFps, sparkSim, selectAgent, findAgentNear } = setupGUIAndHelpers(sim, renderer);
    attachInteraction({ canvas, minimapCanvas, sim, renderer, hover, findAgentNear, selectAgent });

    let last = performance.now(), frames = 0, fpsTimer = 0;
    const start = last;
    const stateNames = ['Idle', 'Moving', 'Arrived', 'No Path'];

    const loop = now => {
      const dt = Math.min(0.1, (now - last) / 1000);
      last = now;
      frames++;
      fpsTimer += dt;

      sim.syncFromShared();

      if (fpsTimer >= 0.5) {
        const curFps = Math.round(frames / fpsTimer);
        CONFIG.telemetry.fps = curFps;
        CONFIG.telemetry.simMs = parseFloat(sim.lastSimDuration.toFixed(1));
        CONFIG.telemetry.workerTicks = sim.workerTicks;
        CONFIG.telemetry.agentCount = sim.agents.count;
        CONFIG.telemetry.movingCount = sim.agents.count - sim.arrivedAtGoal;
        CONFIG.telemetry.arrivedCount = sim.arrivedAtGoal;
        CONFIG.telemetry.flowCacheCount = sim.flowCacheSize;
        CONFIG.telemetry.activeChunks = sim.activeChunkCount;

        sparkFps.push(curFps);
        sparkSim.push(sim.lastSimDuration);

        if (CONFIG.autoGoal.enabled && sim.goal) {
          const pct = sim.agents.count > 0 ? (sim.arrivedAtGoal / sim.agents.count) : 0;
          CONFIG.telemetry.autoGoalTimer = `${sim.autoGoalTimerRem.toFixed(0)}s (${Math.round(pct * 100)}% arr)`;
        } else {
          CONFIG.telemetry.autoGoalTimer = CONFIG.autoGoal.enabled ? 'Active' : 'Disabled';
        }

        const sel = CONFIG.telemetry.selectedId;
        if (sel >= 0 && sel < sim.agents.count) {
          const st = sim.agents.state[sel];
          CONFIG.telemetry.selectedState = stateNames[st] || 'Unknown';
          const spd = Math.hypot(sim.agents.vx[sel], sim.agents.vy[sel]);
          CONFIG.telemetry.selectedSpeed = parseFloat(spd.toFixed(1));
          if (sim.goal) {
            const d = Math.hypot(sim.goal.x + 0.5 - sim.agents.px[sel], sim.goal.y + 0.5 - sim.agents.py[sel]);
            CONFIG.telemetry.selectedGoalDist = parseFloat(d.toFixed(1));
          } else {
            CONFIG.telemetry.selectedGoalDist = 0.0;
          }
          const p = sim.agents.paths[sel];
          CONFIG.telemetry.selectedCorridor = p ? `${p.chunks.length} chunks (${p.portals.length} portals)` : 'Direct';
        } else {
          CONFIG.telemetry.selectedState = '--';
          CONFIG.telemetry.selectedSpeed = 0.0;
          CONFIG.telemetry.selectedGoalDist = 0.0;
          CONFIG.telemetry.selectedCorridor = '--';
        }

        frames = 0;
        fpsTimer = 0;
      }

      renderer.render(sim, CONFIG.telemetry.selectedId, hover, (now - start) / 1000);
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  })();
  //#endregion
})();