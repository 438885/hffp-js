// flowFieldApi.js

//#region 1: ConstantsAndMathUtilities
/** @constant {number} Chunk width/height in cells (16x16) */
export const CS = 16;

/** @constant {number} Total cells per chunk (256) */
export const CC = CS * CS;

/** @constant {number} Square root of 2 */
export const SQ2 = Math.SQRT2;

/** @constant {number} Octile diagonal factor (SQRT2 - 1) */
export const OCT = SQ2 - 1;

/** @constant {number} Maximum supported agents in the simulation pool */
export const MAX_AGENTS = 20_000;

/** @constant {number} Maximum supported chunks (24 x 18) in shared memory */
export const MAX_CHUNKS = 24 * 18;

/** @constant {number} Maximum supported portals in shared memory */
export const MAX_PORTALS = 4096;

/** @constant {number} Fixed physics integration timestep (60 Hz) */
export const FIXED_DT = 1 / 60;

/** @constant {number} Traversal cost for high-speed road cells */
export const COST_HIGHWAY = 1;

/** @constant {number} Traversal cost for standard open terrain */
export const COST_OPEN = 2;

/** @constant {number} Traversal cost for swamp/mud terrain */
export const COST_MUD = 8;

/** @constant {number} Impassable obstacle cost threshold */
export const COST_BLOCK = 255;

/** @constant {number[]} 8-way neighbor X offsets */
export const DX = [1, -1, 0, 0, 1, -1, 1, -1];

/** @constant {number[]} 8-way neighbor Y offsets */
export const DY = [0, 0, 1, -1, 1, 1, -1, -1];

/** @constant {number[]} 8-way neighbor step lengths */
export const DL = [1, 1, 1, 1, SQ2, SQ2, SQ2, SQ2];

/**
 * Atomic control register indices inside `SharedArena.ctrlI32`.
 * @enum {number}
 */
export const CTRL = Object.freeze({
  SEQ: 0,
  PAUSED: 1,
  FRAME: 2,
  AGENT_COUNT: 3,
  ARRIVED_COUNT: 4,
  ACTIVE_CHUNKS: 5,
  FLOW_CACHE_COUNT: 6,
  SIM_DURATION_US: 7,
  GOAL_X: 8,
  GOAL_Y: 9,
  GOAL_CHUNK: 10,
  SELECTED_ID: 11,
  EVENTS: 12,
  GRID_VERSION: 13,
  GRAPH_VERSION: 14,
  AUTOGOAL_REM_MS: 15,
  SEL_CHUNKS_LEN: 16,
  SEL_PORTALS_LEN: 17,
  WORKER_TICKS: 18,
  CHUNKS_X: 19,
  CHUNKS_Y: 20
});

/**
 * Atomic event bitmask flags stored in `CTRL.EVENTS`.
 * @enum {number}
 */
export const EVENTS = Object.freeze({
  GOAL: 1 << 0,
  ARRIVAL: 1 << 1,
  GRAPH_REBUILT: 1 << 2
});

/**
 * Clamps a numeric value within an inclusive [min, max] range.
 * @param {number} v - Value to clamp.
 * @param {number} min - Lower bound.
 * @param {number} max - Upper bound.
 * @returns {number} Clamped value.
 */
export const clamp = (v, min, max) => Math.min(Math.max(v, min), max);

/**
 * Computes the octile distance heuristic between two 2D deltas.
 * @param {number} dx - Delta X.
 * @param {number} dy - Delta Y.
 * @returns {number} Octile distance.
 */
export const octile = (dx, dy) => {
  const ax = Math.abs(dx), ay = Math.abs(dy);
  return ax > ay ? ax + OCT * ay : ay + OCT * ax;
};

/**
 * Creates a seeded 32-bit Mulberry32 pseudo-random number generator.
 * @param {number} s - Initial 32-bit integer seed.
 * @returns {() => number} PRNG function returning floats in [0, 1).
 */
export const mulberry32 = s => () => {
  s = (s + 0x6d2b79f5) >>> 0;
  let t = Math.imul(s ^ (s >>> 15), s | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  return ((t ^ (t >>> 14)) >>> 0) / 4_294_967_296;
};
//#endregion

//#region 2: SharedMemoryArenaAndAtomics
/**
 * Contiguous `SharedArrayBuffer` arena and `Atomics` synchronization layer
 * backing agents, terrain costs, active chunk flow fields, and telemetry registers.
 */
export class SharedArena {
  /**
   * @param {SharedArrayBuffer|ArrayBuffer} [existingBuffer] - Optional pre-allocated buffer.
   */
  constructor(existingBuffer = null) {
    let offset = 0;
    const align8 = n => (n + 7) & ~7;
    const reserve = (BytesPerEl, len) => {
      const start = offset;
      offset = align8(offset + BytesPerEl * len);
      return [start, len];
    };

    const sCtrl = reserve(4, 64);
    const sPx = reserve(4, MAX_AGENTS), sPy = reserve(4, MAX_AGENTS);
    const sVx = reserve(4, MAX_AGENTS), sVy = reserve(4, MAX_AGENTS);
    const sSepX = reserve(4, MAX_AGENTS), sSepY = reserve(4, MAX_AGENTS);
    const sCorrX = reserve(4, MAX_AGENTS), sCorrY = reserve(4, MAX_AGENTS);
    const sRepath = reserve(4, MAX_AGENTS);
    const sPathIdx = reserve(4, MAX_AGENTS), sGoalCell = reserve(4, MAX_AGENTS);
    const sState = reserve(1, MAX_AGENTS), sQueued = reserve(1, MAX_AGENTS), sTouching = reserve(1, MAX_AGENTS);
    const sGridCosts = reserve(1, MAX_CHUNKS * CC);
    const sActStamp = reserve(4, MAX_CHUNKS), sActKind = reserve(1, MAX_CHUNKS);
    const sActInt = reserve(4, MAX_CHUNKS * CC);
    const sActFX = reserve(4, MAX_CHUNKS * CC), sActFY = reserve(4, MAX_CHUNKS * CC);
    const sPortTargets = reserve(4, MAX_PORTALS), sPortOnPath = reserve(1, MAX_PORTALS);
    const sSelChunks = reserve(4, 512), sSelPortals = reserve(4, 512), sSelGoal = reserve(4, 2);

    this.totalBytes = offset;
    if (existingBuffer) {
      this.buffer = existingBuffer;
    } else if (typeof SharedArrayBuffer !== 'undefined' && (typeof window === 'undefined' || window.crossOriginIsolated)) {
      this.buffer = new SharedArrayBuffer(this.totalBytes);
    } else {
      this.buffer = new ArrayBuffer(this.totalBytes);
    }
    this.isShared = typeof SharedArrayBuffer !== 'undefined' && (this.buffer instanceof SharedArrayBuffer);

    const B = this.buffer;
    this.ctrlI32 = new Int32Array(B, sCtrl[0], sCtrl[1]);
    this.px = new Float32Array(B, sPx[0], sPx[1]);
    this.py = new Float32Array(B, sPy[0], sPy[1]);
    this.vx = new Float32Array(B, sVx[0], sVx[1]);
    this.vy = new Float32Array(B, sVy[0], sVy[1]);
    this.sepX = new Float32Array(B, sSepX[0], sSepX[1]);
    this.sepY = new Float32Array(B, sSepY[0], sSepY[1]);
    this.corrX = new Float32Array(B, sCorrX[0], sCorrX[1]);
    this.corrY = new Float32Array(B, sCorrY[0], sCorrY[1]);
    this.repathCd = new Float32Array(B, sRepath[0], sRepath[1]);
    this.pathIdx = new Int32Array(B, sPathIdx[0], sPathIdx[1]);
    this.goalCell = new Int32Array(B, sGoalCell[0], sGoalCell[1]);
    this.state = new Uint8Array(B, sState[0], sState[1]);
    this.queued = new Uint8Array(B, sQueued[0], sQueued[1]);
    this.touching = new Uint8Array(B, sTouching[0], sTouching[1]);
    this.gridCosts = new Uint8Array(B, sGridCosts[0], sGridCosts[1]);
    this.activeStamp = new Int32Array(B, sActStamp[0], sActStamp[1]);
    this.activeKind = new Uint8Array(B, sActKind[0], sActKind[1]);
    this.activeIntegration = new Float32Array(B, sActInt[0], sActInt[1]);
    this.activeFlowX = new Float32Array(B, sActFX[0], sActFX[1]);
    this.activeFlowY = new Float32Array(B, sActFY[0], sActFY[1]);
    this.portalTargets = new Int32Array(B, sPortTargets[0], sPortTargets[1]);
    this.portalOnPath = new Uint8Array(B, sPortOnPath[0], sPortOnPath[1]);
    this.selPathChunks = new Int32Array(B, sSelChunks[0], sSelChunks[1]);
    this.selPathPortals = new Int32Array(B, sSelPortals[0], sSelPortals[1]);
    this.selPathGoal = new Float32Array(B, sSelGoal[0], sSelGoal[1]);
  }

  /**
   * Atomically loads a 32-bit integer register.
   * @param {number} idx - Register index from `CTRL`.
   * @returns {number} Register value.
   */
  load(idx) { return Atomics.load(this.ctrlI32, idx); }

  /**
   * Atomically stores a 32-bit integer register.
   * @param {number} idx - Register index from `CTRL`.
   * @param {number} val - Value to store.
   */
  store(idx, val) { Atomics.store(this.ctrlI32, idx, val | 0); }

  /**
   * Atomically adds `delta` to a register.
   * @param {number} idx - Register index from `CTRL`.
   * @param {number} delta - Integer delta.
   * @returns {number} Previous register value.
   */
  add(idx, delta) { return Atomics.add(this.ctrlI32, idx, delta | 0); }

  /**
   * Atomically sets bitmask flags in `CTRL.EVENTS`.
   * @param {number} mask - Bitmask from `EVENTS`.
   */
  raiseEvent(mask) { Atomics.or(this.ctrlI32, CTRL.EVENTS, mask | 0); }

  /**
   * Atomically reads and clears all pending event flags in `CTRL.EVENTS`.
   * @returns {number} Consumed bitmask.
   */
  consumeEvents() { return Atomics.exchange(this.ctrlI32, CTRL.EVENTS, 0); }

  /** Begins a SeqLock write section (increments `CTRL.SEQ` to odd). */
  beginWrite() { Atomics.add(this.ctrlI32, CTRL.SEQ, 1); }

  /** Ends a SeqLock write section (increments `CTRL.SEQ` to even and notifies waiters). */
  endWrite() {
    Atomics.add(this.ctrlI32, CTRL.SEQ, 1);
    if (this.isShared) Atomics.notify(this.ctrlI32, CTRL.SEQ, 1);
  }

  /**
   * Executes `fn` inside a lock-free SeqLock read loop to guarantee tear-free telemetry reads.
   * @template T
   * @param {() => T} fn - Reader callback.
   * @returns {T} Consistent snapshot result.
   */
  readConsistent(fn) {
    for (let tries = 0; tries < 8; tries++) {
      const s0 = Atomics.load(this.ctrlI32, CTRL.SEQ);
      if (s0 & 1) continue;
      const res = fn();
      const s1 = Atomics.load(this.ctrlI32, CTRL.SEQ);
      if (s0 === s1) return res;
    }
    return fn();
  }
}
//#endregion

//#region 3: PriorityQueuesAndContainers
/**
 * Pre-allocated FIFO integer queue for agent pathfinding requests.
 */
export class RequestQueue {
  constructor(cap) {
    this.q = new Int32Array(cap);
    this.head = 0;
    this.tail = 0;
  }
  get length() { return this.tail - this.head; }
  push(val) { this.q[this.tail++] = val; }
  pop() { return this.head < this.tail ? this.q[this.head++] : -1; }
  clear() { this.head = this.tail = 0; }
}

/**
 * Binary min-heap backed by TypedArrays for A* and Dijkstra searches.
 */
export class MinHeap {
  constructor(cap = 128) {
    this.it = new Int32Array(cap);
    this.k = new Float64Array(cap);
    this.sz = this.lastKey = 0;
  }
  get isEmpty() { return !this.sz; }
  clear() { this.sz = 0; }
  push(item, key) {
    let { it, k, sz } = this;
    if (sz === it.length) {
      const nSz = sz * 2;
      (this.it = new Int32Array(nSz)).set(it);
      (this.k = new Float64Array(nSz)).set(k);
      it = this.it; k = this.k;
    }
    let c = this.sz++, p;
    while (c > 0 && k[p = (c - 1) >> 1] > key) {
      it[c] = it[p]; k[c] = k[p]; c = p;
    }
    it[c] = item; k[c] = key;
  }
  pop() {
    let { it, k } = this, top = it[0], li = --this.sz, item = it[li], key = k[li], p = 0, ch;
    this.lastKey = k[0];
    while ((ch = p * 2 + 1) < li) {
      if (ch + 1 < li && k[ch + 1] < k[ch]) ch++;
      if (k[ch] >= key) break;
      it[p] = it[ch]; k[p] = k[ch]; p = ch;
    }
    it[p] = item; k[p] = key;
    return top;
  }
}
//#endregion

//#region 4: GridWorldDomain
/**
 * Represents the 2D uniform cost grid partitioned into CS x CS chunks.
 * Can be backed directly by a `SharedArena` slice.
 */
export class GridWorld {
  /**
   * @param {number} cx - Number of chunks horizontally.
   * @param {number} cy - Number of chunks vertically.
   * @param {SharedArena|null} [arena=null] - Optional shared memory arena.
   */
  constructor(cx, cy, arena = null) {
    this.cx = cx; this.cy = cy;
    this.width = cx * CS; this.height = cy * CS;
    this.chunkCount = cx * cy; this.version = 0;
    this.arena = arena;
    const len = this.width * this.height;
    this.costs = arena ? arena.gridCosts.subarray(0, len) : new Uint8Array(len);
    this.costs.fill(COST_OPEN);
    if (arena) {
      arena.store(CTRL.CHUNKS_X, cx);
      arena.store(CTRL.CHUNKS_Y, cy);
      arena.store(CTRL.GRID_VERSION, 0);
    }
  }
  index(x, y) { return y * this.width + x; }
  inBounds(x, y) { return x >= 0 && y >= 0 && x < this.width && y < this.height; }
  isWalkable(x, y) { return this.inBounds(x, y) && this.costs[this.index(x, y)] < COST_BLOCK; }
  chunkIdAt(x, y) { return (y / CS | 0) * this.cx + (x / CS | 0); }
  chunkOriginX(c) { return (c % this.cx) * CS; }
  chunkOriginY(c) { return (c / this.cx | 0) * CS; }
  localIndex(cell) { return ((cell / this.width | 0) % CS) * CS + (cell % this.width % CS); }
  touchAll() {
    this.version++;
    if (this.arena) this.arena.store(CTRL.GRID_VERSION, this.version);
  }
  findNearestWalkable(x, y, maxR = 24) {
    const { width: W, height: H } = this, sx = clamp(x | 0, 0, W - 1), sy = clamp(y | 0, 0, H - 1);
    if (this.isWalkable(sx, sy)) return sy * W + sx;
    let bestD = Infinity, bestI = -1;
    for (let r = 1, lim = Math.min(maxR, Math.max(W, H)); r <= lim; r++) {
      const r2 = r * r;
      for (let dy = -r; dy <= r; dy++) {
        const ny = sy + dy;
        if (ny < 0 || ny >= H) continue;
        for (let dx = -r; dx <= r; dx++) {
          if (dx * dx < r2 && dy * dy < r2) continue;
          const nx = sx + dx;
          if (nx >= 0 && nx < W) {
            const idx = ny * W + nx, d = dx * dx + dy * dy;
            if (this.costs[idx] < COST_BLOCK && d < bestD) { bestD = d; bestI = idx; }
          }
        }
      }
      if (bestI >= 0) return bestI;
    }
    return -1;
  }
}
//#endregion

//#region 5: PortalDetectionAndGeometry
/**
 * Scans all vertical and horizontal chunk boundaries to extract contiguous walkable portals.
 * @param {GridWorld} grid - The grid world to analyze.
 * @returns {Array<Object>} List of detected portal descriptors.
 */
export function detectPortals(grid) {
  const portals = [], { cx, cy, width } = grid;
  for (const isH of [false, true]) {
    const countX = cx - !isH, countY = cy - isH, bOffset = isH ? cy * (cx - 1) : 0;
    const W = (u, v) => grid.isWalkable(isH ? u : v, isH ? v : u);
    const at = (u, v) => isH ? v * width + u : u * width + v;
    for (let y = 0; y < countY; y++) {
      for (let x = 0; x < countX; x++) {
        const border = ((isH ? y : x) + 1) * CS, origin = (isH ? x : y) * CS;
        const cA = y * cx + x, cB = cA + (isH ? cx : 1), bIdx = bOffset + y * countX + x;
        let run = -1;
        for (let t = 0; t <= CS; t++) {
          if (t < CS && W(origin + t, border - 1) && W(origin + t, border)) {
            if (run < 0) run = t;
          } else if (run >= 0) {
            const len = t - run, start = origin + run, mid = (len - 1) >> 1, cMid = start + mid;
            const cellsA = new Int32Array(len), cellsB = new Int32Array(len);
            for (let k = 0; k < len; k++) {
              cellsA[k] = at(start + k, border - 1);
              cellsB[k] = at(start + k, border);
            }
            portals.push({
              id: portals.length, uid: (bIdx * CS + run) * (CS + 1) + len,
              isH, chunkA: cA, chunkB: cB, border, start, end: start + len - 1,
              cellsA, cellsB, centerA: cellsA[mid], centerB: cellsB[mid],
              anchorX: isH ? cMid + 0.5 : border, anchorY: isH ? border : cMid + 0.5
            });
            run = -1;
          }
        }
      }
    }
  }
  return portals;
}

export const portalSideCells = (p, c) => c === p.chunkA ? p.cellsA : p.cellsB;
export const portalCenterCell = (p, c) => c === p.chunkA ? p.centerA : p.centerB;
export const portalExitX = (p, c) => p.isH ? 0 : c === p.chunkA ? 1 : -1;
export const portalExitY = (p, c) => p.isH ? c === p.chunkA ? 1 : -1 : 0;
export const crossingCost = (grid, p) => ((grid.costs[p.centerA] + grid.costs[p.centerB]) * 0.5) / COST_OPEN;
export const pLocal = (grid, p, c) => grid.localIndex(portalCenterCell(p, c));
//#endregion

//#region 6: ChunkEikonalIntegrator
/**
 * Computes 8-connected Dijkstra integration fields across a single CS x CS chunk.
 */
export class ChunkIntegrator {
  heap = new MinHeap(CC * 2);
  costs = new Uint8Array(CC);
  dist = new Float64Array(CC);

  integrate(grid, chunkId, sources, out) {
    const { costs, dist, heap } = this, ox = grid.chunkOriginX(chunkId), oy = grid.chunkOriginY(chunkId);
    for (let ly = 0, r = oy * grid.width + ox; ly < CS; ly++, r += grid.width) {
      costs.set(grid.costs.subarray(r, r + CS), ly * CS);
    }
    dist.fill(Infinity);
    heap.clear();
    for (let i = 0; i < sources.length; i++) {
      const cell = sources[i];
      if (costs[cell] < COST_BLOCK && dist[cell]) heap.push(cell, dist[cell] = 0);
    }
    while (!heap.isEmpty) {
      const u = heap.pop(), du = heap.lastKey;
      if (du > dist[u]) continue;
      const ux = u % CS, uy = u / CS | 0, cu = costs[u];
      for (let d = 0; d < 8; d++) {
        const nx = ux + DX[d], ny = uy + DY[d];
        if (nx >>> 0 >= CS || ny >>> 0 >= CS) continue;
        const v = ny * CS + nx, cv = costs[v];
        if (cv >= COST_BLOCK || (d >= 4 && (costs[uy * CS + nx] >= COST_BLOCK || costs[ny * CS + ux] >= COST_BLOCK))) continue;
        const moveCost = DL[d] * ((cu + cv) * 0.25);
        const nd = du + moveCost;
        if (nd < dist[v]) heap.push(v, dist[v] = nd);
      }
    }
    out.set(dist);
  }
}
//#endregion

//#region 7: PortalGraphTopology
/**
 * High-level abstract graph connecting chunk boundary portals in CSR format.
 * Supports incremental O(1) reuse of unmodified chunk distance matrices.
 */
export class PortalGraph {
  constructor(grid, prevGraph = null, dirtyChunks = null) {
    this.grid = grid;
    const portals = this.portals = detectPortals(grid), P = portals.length;
    const lists = Array.from({ length: grid.chunkCount }, () => []);
    for (let i = 0; i < P; i++) {
      const p = portals[i];
      lists[p.chunkA].push(p.id); lists[p.chunkB].push(p.id);
    }
    const chunkPortals = this.chunkPortals = lists.map(l => new Int32Array(l));
    this.chunkDistCache = Array.from({ length: grid.chunkCount }, (_, c) =>
      prevGraph && dirtyChunks && !dirtyChunks.has(c) ? prevGraph.chunkDistCache[c] : new Map()
    );
    const adj = Array.from({ length: P }, () => []), it = new ChunkIntegrator(), field = new Float64Array(CC);
    for (let c = 0; c < grid.chunkCount; c++) {
      const ids = chunkPortals[c];
      if (ids.length < 2) continue;
      const cCache = this.chunkDistCache[c];
      for (let ia = 0; ia < ids.length; ia++) {
        const idA = ids[ia], locA = pLocal(grid, portals[idA], c);
        let row = cCache.get(locA);
        if (!row) {
          it.integrate(grid, c, [locA], field);
          row = new Float64Array(field);
          cCache.set(locA, row);
        }
        for (let ib = 0; ib < ids.length; ib++) {
          const idB = ids[ib];
          if (idA === idB) continue;
          const to = portals[idB], d = row[pLocal(grid, to, c)];
          if (d < Infinity) adj[idA].push({ to: idB, cost: d + crossingCost(grid, to), chunk: c });
        }
      }
    }
    const E = adj.reduce((acc, a) => acc + a.length, 0);
    this.edgeOffsets = new Int32Array(P + 1);
    this.edgeTargets = new Int32Array(E);
    this.edgeCosts = new Float64Array(E);
    this.edgeChunks = new Int32Array(E);
    for (let p = 0, e = 0; p < P; this.edgeOffsets[++p] = e) {
      for (let k = 0; k < adj[p].length; k++) {
        const edge = adj[p][k];
        this.edgeTargets[e] = edge.to; this.edgeCosts[e] = edge.cost; this.edgeChunks[e++] = edge.chunk;
      }
    }
  }

  /**
   * Exports a lightweight serializable snapshot of portals and CSR edges for Comlink transfer.
   * @returns {Object} Serializable graph topology.
   */
  toSnapshot() {
    return {
      portals: this.portals.map(p => ({
        id: p.id, uid: p.uid, isH: p.isH, start: p.start, border: p.border, end: p.end,
        anchorX: p.anchorX, anchorY: p.anchorY
      })),
      edgeOffsets: this.edgeOffsets.slice(),
      edgeTargets: this.edgeTargets.slice()
    };
  }
}
//#endregion

//#region 8: MacroPathfinder
/**
 * Hierarchical A* pathfinder operating over the `PortalGraph`.
 */
export class MacroPathfinder {
  constructor(grid, graph) {
    this.grid = grid; this.graph = graph;
    const N = graph.portals.length + 2;
    this.g = new Float64Array(N); this.parent = new Int32Array(N);
    this.parentChunk = new Int32Array(N); this.seen = new Uint32Array(N);
    this.closed = new Uint32Array(N); this.it = new ChunkIntegrator();
    this.startField = new Float64Array(CC); this.heap = new MinHeap(256); this.stamp = 0;
  }

  createGoal(cell) {
    const { grid, graph } = this, x = cell % grid.width, y = cell / grid.width | 0;
    const chunk = grid.chunkIdAt(x, y), field = new Float64Array(CC);
    this.it.integrate(grid, chunk, [grid.localIndex(cell)], field);
    const portalCost = new Float64Array(graph.portals.length).fill(Infinity);
    const cPortals = graph.chunkPortals[chunk];
    for (let i = 0; i < cPortals.length; i++) {
      const pid = cPortals[i];
      const d = field[pLocal(grid, graph.portals[pid], chunk)];
      if (d < Infinity) portalCost[pid] = d;
    }
    return { cell, x, y, chunk, field, portalCost, graph };
  }

  findPath(startCell, goal) {
    const { grid, graph, g, parent, parentChunk, seen, closed, heap } = this;
    const P = graph.portals.length, START = P, GOAL = P + 1;
    if (++this.stamp >= 0xffffffff) { seen.fill(0); closed.fill(0); this.stamp = 1; }
    const s = this.stamp;
    const sx = startCell % grid.width, sy = startCell / grid.width | 0, sc = grid.chunkIdAt(sx, sy);
    const gx = goal.x + 0.5, gy = goal.y + 0.5;
    this.it.integrate(grid, sc, [grid.localIndex(startCell)], this.startField);
    heap.clear(); g[START] = 0; seen[START] = s; parent[START] = -1; parentChunk[START] = sc;
    heap.push(START, octile(sx + 0.5 - gx, sy + 0.5 - gy));
    const relax = (v, cost, from, chunk) => {
      if (closed[v] === s || (seen[v] === s && cost >= g[v])) return;
      seen[v] = s; g[v] = cost; parent[v] = from; parentChunk[v] = chunk;
      const h = v < P ? Math.max(0, octile(graph.portals[v].anchorX - gx, graph.portals[v].anchorY - gy) - 0.5) : 0;
      heap.push(v, cost + h);
    };
    while (!heap.isEmpty) {
      const u = heap.pop();
      if (closed[u] === s) continue;
      closed[u] = s;
      if (u === GOAL) {
        const portals = [], chunks = [parentChunk[GOAL]];
        let n = parent[GOAL];
        while (n !== START) { portals.push(n); chunks.push(parentChunk[n]); n = parent[n]; }
        return {
          portals: new Int32Array(portals.reverse()), chunks: new Int32Array(chunks.reverse()),
          cost: g[GOAL], goalCell: goal.cell, goalX: gx, goalY: gy, graph
        };
      }
      const gu = g[u];
      if (u === START) {
        const cPorts = graph.chunkPortals[sc];
        for (let i = 0; i < cPorts.length; i++) {
          const pid = cPorts[i], p = graph.portals[pid], d = this.startField[pLocal(grid, p, sc)];
          if (d < Infinity) relax(pid, gu + d + crossingCost(grid, p), START, sc);
        }
        if (sc === goal.chunk) {
          const d = goal.field[grid.localIndex(startCell)];
          if (d < Infinity) relax(GOAL, gu + d, START, sc);
        }
      } else {
        for (let e = graph.edgeOffsets[u]; e < graph.edgeOffsets[u + 1]; e++) {
          relax(graph.edgeTargets[e], gu + graph.edgeCosts[e], u, graph.edgeChunks[e]);
        }
        const gc = goal.portalCost[u];
        if (gc < Infinity) relax(GOAL, gu + gc, u, goal.chunk);
      }
    }
    return null;
  }
}
//#endregion

//#region 9: FlowFieldAndBilinearSampler
export function computeFlowVectors(f, exX, exY) {
  const { integration: I, flowX: FX, flowY: FY } = f;
  const at = (x, y) => x >>> 0 >= CS || y >>> 0 >= CS ? Infinity : I[y * CS + x];
  for (let y = 0; y < CS; y++) {
    for (let x = 0; x < CS; x++) {
      const id = y * CS + x, cur = I[id];
      if (cur === Infinity) { FX[id] = FY[id] = 0; continue; }
      if (!cur) { FX[id] = exX; FY[id] = exY; continue; }
      const E = at(x + 1, y), W = at(x - 1, y), S = at(x, y + 1), N = at(x, y - 1);
      const gx = E < W ? (E < cur ? E - cur : 0) : W < cur ? cur - W : 0;
      const gy = S < N ? (S < cur ? S - cur : 0) : N < cur ? cur - N : 0;
      let dx = -gx, dy = -gy, len = Math.hypot(dx, dy), ok = len > 1e-6;
      if (ok) {
        dx /= len; dy /= len;
        if (Math.abs(dx) > 0.25 && Math.abs(dy) > 0.25 && at(x + (dx > 0 ? 1 : -1), y + (dy > 0 ? 1 : -1)) === Infinity) ok = false;
      }
      if (!ok) {
        let bS = 0, bx = 0, by = 0;
        for (let d = 0; d < 8; d++) {
          const nx = x + DX[d], ny = y + DY[d], v = at(nx, ny);
          if (v === Infinity || (d >= 4 && (at(nx, y) === Infinity || at(x, ny) === Infinity))) continue;
          const sl = (cur - v) / DL[d];
          if (sl > bS) { bS = sl; bx = DX[d]; by = DY[d]; }
        }
        len = Math.hypot(bx, by);
        dx = bx / len || 0; dy = by / len || 0;
      }
      FX[id] = dx; FY[id] = dy;
    }
  }
}

export class FlowFieldCache {
  constructor(grid, cap = 2048) {
    this.grid = grid; this.cap = cap; this.entries = new Map();
    this.it = new ChunkIntegrator();
  }
  key(c, kind, tid) { return (tid * 2 + (kind === 'g' ? 1 : 0)) * 65536 + c; }
  getPortalField(c, p) {
    const k = this.key(c, 'p', p.uid);
    const hit = this.entries.get(k);
    if (hit) return hit;
    return this.build(k, c, 'p', p.uid, portalSideCells(p, c).map(x => this.grid.localIndex(x)), portalExitX(p, c), portalExitY(p, c));
  }
  getGoalField(c, cell) {
    const k = this.key(c, 'g', cell);
    const hit = this.entries.get(k);
    if (hit) return hit;
    return this.build(k, c, 'g', cell, [this.grid.localIndex(cell)], 0, 0);
  }
  clear() { this.entries.clear(); }
  invalidateChunks(dirtyChunks) {
    for (const [k, f] of this.entries) {
      if (dirtyChunks.has(f.chunkId)) this.entries.delete(k);
    }
  }
  build(k, cid, kind, tid, src, exX, exY) {
    const { grid } = this, f = {
      chunkId: cid, kind, originX: grid.chunkOriginX(cid), originY: grid.chunkOriginY(cid),
      integration: new Float32Array(CC), flowX: new Float32Array(CC), flowY: new Float32Array(CC),
      usageFrame: -1, usageCount: 0
    };
    this.it.integrate(grid, cid, src, f.integration);
    computeFlowVectors(f, exX, exY);
    if (this.entries.size >= this.cap) {
      const firstKey = this.entries.keys().next().value;
      this.entries.delete(firstKey);
    }
    this.entries.set(k, f);
    return f;
  }
}

export function sampleFlowBilinear(f, px, py, out) {
  const lx = px - f.originX - 0.5, ly = py - f.originY - 0.5;
  const x0 = Math.floor(lx), y0 = Math.floor(ly);
  const tx = lx - x0, ty = ly - y0;
  const { integration: I, flowX: FX, flowY: FY } = f;
  const w00 = (1 - tx) * (1 - ty), w10 = tx * (1 - ty), w01 = (1 - tx) * ty, w11 = tx * ty;
  const cx0 = x0 < 0 ? 0 : (x0 > 15 ? 15 : x0);
  const cx1 = (x0 + 1) < 0 ? 0 : ((x0 + 1) > 15 ? 15 : (x0 + 1));
  const cy0 = y0 < 0 ? 0 : (y0 > 15 ? 15 : y0);
  const cy1 = (y0 + 1) < 0 ? 0 : ((y0 + 1) > 15 ? 15 : (y0 + 1));
  let sx = 0, sy = 0, ws = 0;
  if (w00 > 0) { const id = cy0 * 16 + cx0; if (I[id] < Infinity) { sx += FX[id] * w00; sy += FY[id] * w00; ws += w00; } }
  if (w10 > 0) { const id = cy0 * 16 + cx1; if (I[id] < Infinity) { sx += FX[id] * w10; sy += FY[id] * w10; ws += w10; } }
  if (w01 > 0) { const id = cy1 * 16 + cx0; if (I[id] < Infinity) { sx += FX[id] * w01; sy += FY[id] * w01; ws += w01; } }
  if (w11 > 0) { const id = cy1 * 16 + cx1; if (I[id] < Infinity) { sx += FX[id] * w11; sy += FY[id] * w11; ws += w11; } }
  if (ws < 1e-6) return false;
  const len = Math.hypot(sx, sy);
  if (len < 0.15 * ws) {
    const c = clamp(Math.floor(py - f.originY), 0, CS - 1) * CS + clamp(Math.floor(px - f.originX), 0, CS - 1);
    if (I[c] < Infinity && (FX[c] || FY[c])) { out.x = FX[c]; out.y = FY[c]; return true; }
    if (len < 1e-6) return false;
  }
  out.x = sx / len; out.y = sy / len;
  return true;
}
//#endregion

//#region 10: ProceduralMapGenerator
export function fillRect(grid, x, y, w, h, cost) {
  const x0 = Math.max(0, x), x1 = Math.min(grid.width, x + w);
  for (let yy = Math.max(0, y), y1 = Math.min(grid.height, y + h); yy < y1; yy++) {
    grid.costs.fill(cost, yy * grid.width + x0, yy * grid.width + x1);
  }
}

export function fillDisc(grid, cx, cy, r, cost) {
  const r2 = r * r, x0 = Math.max(0, Math.floor(cx - r)), x1 = Math.min(grid.width - 1, Math.ceil(cx + r));
  const y0 = Math.max(0, Math.floor(cy - r)), y1 = Math.min(grid.height - 1, Math.ceil(cy + r));
  for (let y = y0; y <= y1; y++) {
    for (let x = x0; x <= x1; x++) {
      if ((x + 0.5 - cx) ** 2 + (y + 0.5 - cy) ** 2 <= r2) {
        const id = y * grid.width + x;
        if (cost === COST_BLOCK || grid.costs[id] === COST_OPEN) grid.costs[id] = cost;
      }
    }
  }
}

export function removeDisconnectedPockets(grid) {
  const { width: W, costs } = grid, tot = costs.length;
  const label = new Int32Array(tot).fill(-1), queue = new Int32Array(tot);
  let best = -1, bestSz = 0, comp = 0, t = 0;
  const chk = ni => { if (label[ni] === -1 && costs[ni] < COST_BLOCK) { label[ni] = comp; queue[t++] = ni; } };
  for (let s = 0; s < tot; s++) {
    if (label[s] !== -1 || costs[s] >= COST_BLOCK) continue;
    let h = t = 0;
    queue[t++] = s; label[s] = comp;
    while (h < t) {
      const c = queue[h++], x = c % W;
      if (x > 0) chk(c - 1);
      if (x < W - 1) chk(c + 1);
      if (c >= W) chk(c - W);
      if (c + W < tot) chk(c + W);
    }
    if (t > bestSz) { bestSz = t; best = comp; }
    comp++;
  }
  for (let i = 0; i < tot; i++) if (label[i] !== -1 && label[i] !== best) costs[i] = COST_BLOCK;
}

export function generateMap(grid, seed, { preset = 'The Grand Citadel', wallDensity = 1, mudDensity = 1 } = {}) {
  const rng = mulberry32(seed), { width: W, height: H } = grid, area = W * H;
  grid.costs.fill(COST_OPEN);
  const clr = [];
  const N = (div, d) => Math.round((area / div) * d);
  const addMud = (cnt, r0, r1) => {
    for (let k = 0; k < cnt; k++) fillDisc(grid, rng() * W, rng() * H, r0 + rng() * r1, COST_MUD);
  };
  const addHighways = cnt => {
    for (let k = 0; k < cnt; k++) {
      if (rng() < 0.5) fillRect(grid, 0, Math.floor(rng() * H), W, 2, COST_HIGHWAY);
      else fillRect(grid, Math.floor(rng() * W), 0, 2, H, COST_HIGHWAY);
    }
  };
  if (preset === 'The Grand Citadel') {
    const midX = W / 2 | 0, midY = H / 2 | 0;
    fillRect(grid, midX - 2, 0, 4, H, COST_BLOCK);
    clr.push([midX - 3, midY - 6, 6, 12], [midX - 3, 4, 6, 8], [midX - 3, H - 12, 6, 8]);
    fillRect(grid, 0, midY - 2, W, 4, COST_BLOCK);
    clr.push([midX - 6, midY - 3, 12, 6], [12, midY - 3, 8, 6], [W - 20, midY - 3, 8, 6]);
    for (let p = N(400, wallDensity); p--;) {
      fillRect(grid, Math.floor(rng() * (W - 5)), Math.floor(rng() * (H - 5)), 3 + (rng() * 4 | 0), 3 + (rng() * 4 | 0), COST_BLOCK);
    }
    fillRect(grid, 2, 2, W - 4, 2, COST_HIGHWAY);
    fillRect(grid, 2, H - 4, W - 4, 2, COST_HIGHWAY);
    addMud(N(2000, mudDensity), 3, 5);
  } else if (preset === 'The Great Maze') {
    for (let x = 4; x < W - 4; x += 6) {
      fillRect(grid, x, 0, 2, H, COST_BLOCK);
      for (let y = 0; y < H - 2; y += 6) clr.push([x - 1, y + Math.floor(rng() * 4), 4, 3]);
    }
    for (let y = 4; y < H - 4; y += 6) {
      fillRect(grid, 0, y, W, 2, COST_BLOCK);
      for (let x = 0; x < W - 2; x += 6) clr.push([x + Math.floor(rng() * 4), y - 1, 3, 4]);
    }
    addMud(N(1500, mudDensity), 2, 3);
  } else if (preset === 'Twin Hourglass Bottlenecks') {
    fillRect(grid, 0, 0, W, H, COST_OPEN);
    const w3 = (W / 3) | 0;
    fillRect(grid, w3, 0, 3, H, COST_BLOCK);
    fillRect(grid, w3 * 2, 0, 3, H, COST_BLOCK);
    clr.push([w3 - 1, (H * 0.45) | 0, 5, 8], [w3 * 2 - 1, (H * 0.55) | 0, 5, 8]);
    addHighways(2);
    addMud(N(1800, mudDensity), 4, 6);
  } else if (preset === 'Highway vs Swamp') {
    fillRect(grid, (W * 0.25) | 0, 0, (W * 0.5) | 0, H, COST_MUD);
    fillRect(grid, 2, 2, W - 4, 3, COST_HIGHWAY);
    fillRect(grid, 2, H - 5, W - 4, 3, COST_HIGHWAY);
    for (let p = N(600, wallDensity); p--;) {
      fillRect(grid, Math.floor(rng() * (W - 4)), Math.floor(rng() * (H - 4)), 3, 3, COST_BLOCK);
    }
  } else {
    for (let p = N(450, wallDensity); p--;) fillDisc(grid, rng() * W, rng() * H, 2 + rng() * 3, COST_BLOCK);
    addMud(N(1400, mudDensity), 3, 6);
    addHighways(3);
  }
  for (const r of clr) fillRect(grid, ...r, COST_OPEN);
  removeDisconnectedPockets(grid);
  grid.touchAll();
}
//#endregion

//#region 11: SpatialHashAndAgentPool
export class SpatialHash {
  constructor(w, h, cs, cap) {
    this.cs = cs;
    this.cols = Math.max(1, Math.ceil(w / cs));
    this.rows = Math.max(1, Math.ceil(h / cs));
    const B = this.cols * this.rows;
    this.start = new Int32Array(B + 1);
    this.cur = new Int32Array(B);
    this.items = new Int32Array(cap);
    this.bOf = new Int32Array(cap);
  }
  bx(x) { return clamp(Math.floor(x / this.cs), 0, this.cols - 1); }
  by(y) { return clamp(Math.floor(y / this.cs), 0, this.rows - 1); }
  build(px, py, n) {
    const { start: s, cols, rows, cur, items, bOf } = this, B = cols * rows;
    s.fill(0);
    for (let i = 0; i < n; i++) {
      const b = this.by(py[i]) * cols + this.bx(px[i]);
      bOf[i] = b;
      s[b + 1]++;
    }
    for (let b = 0; b < B; b++) s[b + 1] += s[b];
    for (let b = 0; b < B; b++) cur[b] = s[b];
    for (let i = 0; i < n; i++) items[cur[bOf[i]]++] = i;
  }
}

/**
 * Structure-of-Arrays (SoA) memory pool backed directly by `SharedArena` views when provided.
 */
export class AgentPool {
  /**
   * @param {number} cap - Maximum agent capacity.
   * @param {SharedArena|null} [arena=null] - Shared memory arena backing the arrays.
   */
  constructor(cap, arena = null) {
    this.count = 0;
    this.arena = arena;
    const f32 = () => new Float32Array(cap);
    this.px = arena ? arena.px : f32();
    this.py = arena ? arena.py : f32();
    this.vx = arena ? arena.vx : f32();
    this.vy = arena ? arena.vy : f32();
    this.sepX = arena ? arena.sepX : f32();
    this.sepY = arena ? arena.sepY : f32();
    this.corrX = arena ? arena.corrX : f32();
    this.corrY = arena ? arena.corrY : f32();
    this.repathCd = arena ? arena.repathCd : f32();
    this.state = arena ? arena.state : new Uint8Array(cap);
    this.queued = arena ? arena.queued : new Uint8Array(cap);
    this.touching = arena ? arena.touching : new Uint8Array(cap);
    this.pathIdx = arena ? arena.pathIdx : new Int32Array(cap);
    this.goalCell = arena ? arena.goalCell : new Int32Array(cap);
    this.goalCell.fill(-1);
    this.paths = Array.from({ length: cap }, () => null);
    this.fields = Array.from({ length: cap }, () => null);
  }
  reset(i, x, y) {
    this.px[i] = x; this.py[i] = y;
    this.vx[i] = this.vy[i] = this.state[i] = this.pathIdx[i] = this.repathCd[i] = 0;
    this.goalCell[i] = -1; this.paths[i] = this.fields[i] = null;
  }
}
//#endregion

//#region 12: SimulationEngine
/**
 * Complete Hierarchical Flow Field Pathfinding & Crowd Physics Simulation Engine.
 * Synchronizes all kinematic, field, and telemetry state into `SharedArena` via `Atomics`.
 */
export class Simulation {
  constructor(opts = {}) {
    this.arena = opts.arena instanceof SharedArena ? opts.arena : new SharedArena(opts.buffer || null);
    this.agents = new AgentPool(MAX_AGENTS, this.arena);
    this.agents.count = clamp(Math.round(opts.agentCount ?? 10_000), 0, MAX_AGENTS);
    this.arena.store(CTRL.AGENT_COUNT, this.agents.count);
    this.agentRadius = opts.agentRadius ?? 0.25;
    this.spawnPattern = opts.spawnPattern ?? 'Left Flank';
    this.maxSpeed = opts.maxSpeed ?? 20.0;
    this.steeringGain = opts.steeringGain ?? 25.0;
    this.separationStrength = opts.separationStrength ?? 28.0;
    this.pathBudgetMs = opts.pathBudgetMs ?? 20.0;
    this.simSpeed = opts.simSpeed ?? 3.0;
    this.autoGoal = opts.autoGoal ?? null;
    this.onGoal = opts.onGoal ?? null;
    this.onArrival = opts.onArrival ?? null;
    this.goal = null;
    this.graphVersion = 0;
    this.frame = this.arrivedAtGoal = this.respawnCounter = this.accumulator = this.autoGoalTimer = this.arrivalTimer = 0;
    this.urgentQ = new RequestQueue(MAX_AGENTS * 2);
    this.bulkQ = new RequestQueue(MAX_AGENTS * 2);
    this.heading = { x: 0, y: 0 };
    this.lastSimDuration = 0;
    this.regenerate(opts.seed ?? 1337666, opts.chunksX ?? 10, opts.chunksY ?? 6, opts);
  }

  setAgentCount(cnt) {
    const prev = this.agents.count, next = this.agents.count = clamp(Math.round(cnt), 0, MAX_AGENTS);
    this.arena.store(CTRL.AGENT_COUNT, next);
    for (let i = prev; i < next; i++) { this.spawnAgent(i); this.goal && this.reqPath(i, false); }
  }

  regenerate(seed, cx, cy, mapOpts = {}) {
    this.seed = seed;
    this.goal = null;
    generateMap(this.grid = new GridWorld(cx, cy, this.arena), seed, mapOpts);
    this.rebuildGraph();
    this.respawnAll();
    this.setGoal(this.grid.width * 0.9 | 0, this.grid.height * 0.5 | 0, false);
  }

  rebuildGraph(dirtyChunks = null) {
    const { grid, arena } = this;
    const isIncremental = !!(dirtyChunks && this.graph && this.flowCache);
    this.graph = new PortalGraph(grid, isIncremental ? this.graph : null, dirtyChunks);
    const pLen = Math.min(this.graph.portals.length, MAX_PORTALS);
    if (isIncremental) {
      this.flowCache.invalidateChunks(dirtyChunks);
      for (const c of dirtyChunks) {
        this.activeFields[c] = null;
        this.activeStamp[c] = -1;
      }
    } else {
      this.flowCache = new FlowFieldCache(grid);
      this.hash = new SpatialHash(grid.width, grid.height, Math.max(1.2, this.agentRadius * 3.2), MAX_AGENTS);
      this.activeFields = Array(grid.chunkCount).fill(null);
      this.activeStamp = arena.activeStamp.subarray(0, grid.chunkCount);
      this.activeStamp.fill(-1);
    }
    this.pathfinder = new MacroPathfinder(grid, this.graph);
    this.portalTargets = arena.portalTargets.subarray(0, pLen);
    this.portalOnPath = arena.portalOnPath.subarray(0, pLen);
    this.graphVersion++;
    arena.store(CTRL.GRAPH_VERSION, this.graphVersion);
    arena.raiseEvent(EVENTS.GRAPH_REBUILT);
    if (this.goal) {
      if (isIncremental) {
        const cell = grid.isWalkable(this.goal.x, this.goal.y)
          ? grid.index(this.goal.x, this.goal.y)
          : grid.findNearestWalkable(this.goal.x, this.goal.y, 24);
        if (cell >= 0) {
          this.goal = this.pathfinder.createGoal(cell);
          arena.store(CTRL.GOAL_X, this.goal.x);
          arena.store(CTRL.GOAL_Y, this.goal.y);
          arena.store(CTRL.GOAL_CHUNK, this.goal.chunk);
        }
      } else {
        this.setGoal(this.goal.x, this.goal.y, false);
      }
    }
  }

  paintTerrain(wx, wy, r, cost) {
    const { grid, agents: A } = this, { width: W, height: H, costs } = grid;
    const r2 = r * r;
    const x0 = clamp(Math.floor(wx - r), 0, W - 1), x1 = clamp(Math.ceil(wx + r), 0, W - 1);
    const y0 = clamp(Math.floor(wy - r), 0, H - 1), y1 = clamp(Math.ceil(wy + r), 0, H - 1);
    const dirtyChunks = new Set();
    let changed = 0;

    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        if ((x + 0.5 - wx) ** 2 + (y + 0.5 - wy) ** 2 > r2) continue;
        const idx = y * W + x;
        if (costs[idx] === cost) continue;
        costs[idx] = cost;
        changed++;
        const c = grid.chunkIdAt(x, y);
        dirtyChunks.add(c);
        const lx = x % CS, ly = y % CS;
        if (lx === 0 && x > 0) dirtyChunks.add(c - 1);
        if (lx === CS - 1 && x < W - 1) dirtyChunks.add(c + 1);
        if (ly === 0 && y > 0) dirtyChunks.add(c - grid.cx);
        if (ly === CS - 1 && y < H - 1) dirtyChunks.add(c + grid.cx);
      }
    }

    if (!changed) return 0;
    grid.touchAll();
    this.rebuildGraph(dirtyChunks);

    for (let i = 0; i < A.count; i++) {
      const ac = grid.chunkIdAt(clamp(A.px[i] | 0, 0, W - 1), clamp(A.py[i] | 0, 0, H - 1));
      if (dirtyChunks.has(ac)) {
        A.fields[i] = null;
        this.reqPath(i, true);
        continue;
      }
      const p = A.paths[i];
      if (!p) {
        this.reqPath(i, false);
        continue;
      }
      let hit = false;
      for (let k = A.pathIdx[i]; k < p.chunks.length; k++) {
        if (dirtyChunks.has(p.chunks[k])) { hit = true; break; }
      }
      this.reqPath(i, hit);
    }
    return changed;
  }

  respawnAll() {
    this.rng = mulberry32((Math.imul(this.seed, 2654435761) + ++this.respawnCounter * 7919) >>> 0);
    this.clearQ();
    this.arrivedAtGoal = this.autoGoalTimer = this.arrivalTimer = 0;
    for (let i = 0; i < this.agents.count; i++) { this.spawnAgent(i); this.goal && this.reqPath(i, false); }
  }

  spawnAgent(i) {
    const { grid, rng, spawnPattern: sp } = this, { width: W, height: H } = grid;
    let x = 0, y = 0, ok = false;
    for (let a = 0; a < 80 && !ok; a++) {
      if (sp === 'Left Flank' || sp === 'Scatter Everywhere') {
        x = rng() * W * (sp === 'Left Flank' ? 0.2 : 1); y = rng() * H;
      } else if (sp === 'Four Corners') {
        const c = a % 4;
        x = ((c % 2) * 0.8 + 0.1 + (rng() - 0.5) * 0.14) * W;
        y = ((c >> 1) * 0.8 + 0.1 + (rng() - 0.5) * 0.14) * H;
      } else {
        x = (0.5 + (rng() - 0.5) * 0.32) * W; y = (0.5 + (rng() - 0.5) * 0.32) * H;
      }
      const cx = x | 0, cy = y | 0;
      if (grid.isWalkable(cx, cy)) { x = cx + 0.3 + rng() * 0.4; y = cy + 0.3 + rng() * 0.4; ok = true; }
    }
    if (!ok) {
      const cell = grid.findNearestWalkable(x, y, 16);
      if (cell >= 0) { x = (cell % W) + 0.5; y = (cell / W | 0) + 0.5; }
    }
    this.agents.reset(i, x, y);
  }

  setGoal(cx, cy, playAudio = true) {
    const { grid, arena } = this, x = clamp(cx | 0, 0, grid.width - 1), y = clamp(cy | 0, 0, grid.height - 1);
    const cell = grid.isWalkable(x, y) ? grid.index(x, y) : grid.findNearestWalkable(x, y, 24);
    if (cell < 0) return false;
    this.goal = this.pathfinder.createGoal(cell);
    arena.store(CTRL.GOAL_X, this.goal.x);
    arena.store(CTRL.GOAL_Y, this.goal.y);
    arena.store(CTRL.GOAL_CHUNK, this.goal.chunk);
    this.clearQ();
    this.autoGoalTimer = this.arrivalTimer = 0;
    if (playAudio) {
      arena.raiseEvent(EVENTS.GOAL);
      if (this.onGoal) this.onGoal(this.goal);
    }
    for (let i = 0; i < this.agents.count; i++) this.reqPath(i, false);
    return true;
  }

  pickRandomGoal() {
    const { grid, goal } = this, { width: W, height: H } = grid;
    const px = goal ? goal.x : W / 2, py = goal ? goal.y : H / 2, minDist = Math.min(W, H) * 0.38;
    for (let a = 0; a < 120; a++) {
      const rx = Math.random() * W | 0, ry = Math.random() * H | 0;
      if (grid.isWalkable(rx, ry) && (Math.hypot(rx - px, ry - py) >= minDist || a > 70)) return this.setGoal(rx, ry, true);
    }
    return false;
  }

  findAgentNear(wx, wy, maxRadius = 1.6) {
    const { agents: A, hash } = this, r2 = maxRadius * maxRadius;
    const bx = hash.bx(wx), by = hash.by(wy);
    let best = -1, bestD2 = r2;
    for (let yy = Math.max(0, by - 1); yy <= Math.min(hash.rows - 1, by + 1); yy++) {
      for (let xx = Math.max(0, bx - 1); xx <= Math.min(hash.cols - 1, bx + 1); xx++) {
        const b = yy * hash.cols + xx;
        for (let k = hash.start[b], end = hash.start[b + 1]; k < end; k++) {
          const i = hash.items[k];
          const dx = A.px[i] - wx, dy = A.py[i] - wy, d2 = dx * dx + dy * dy;
          if (d2 < bestD2) { bestD2 = d2; best = i; }
        }
      }
    }
    return best;
  }

  clearQ() {
    this.urgentQ.clear();
    this.bulkQ.clear();
    this.agents.queued.fill(0);
  }

  reqPath(i, urgent) {
    const A = this.agents;
    if (A.queued[i] || (urgent && A.repathCd[i] > 0)) return;
    urgent ? (A.repathCd[i] = 0.35, this.urgentQ.push(i)) : this.bulkQ.push(i);
    A.queued[i] = 1;
  }

  processRequests() {
    const t0 = performance.now();
    let n = 0;
    while (this.urgentQ.length > 0 && n < 256) {
      this.computePath(this.urgentQ.pop());
      n++;
    }
    while (this.bulkQ.length > 0 && (n < 16 || performance.now() - t0 < this.pathBudgetMs)) {
      this.computePath(this.bulkQ.pop());
      n++;
    }
    if (this.urgentQ.length === 0) this.urgentQ.clear();
    if (this.bulkQ.length === 0) this.bulkQ.clear();
  }

  computePath(i) {
    const A = this.agents;
    A.queued[i] = 0;
    if (i >= A.count) return;
    if (!this.goal) { A.state[i] = 0; A.paths[i] = A.fields[i] = null; A.goalCell[i] = -1; return; }
    const { grid } = this, cx = clamp(A.px[i] | 0, 0, grid.width - 1), cy = clamp(A.py[i] | 0, 0, grid.height - 1);
    const start = grid.isWalkable(cx, cy) ? grid.index(cx, cy) : grid.findNearestWalkable(cx, cy, 12);
    const path = start >= 0 ? this.pathfinder.findPath(start, this.goal) : null;
    A.fields[i] = null;
    if (!path) { A.state[i] = 3; A.paths[i] = null; A.goalCell[i] = this.goal.cell; return; }
    A.state[i] = (A.state[i] === 2 && A.goalCell[i] === this.goal.cell) ? 2 : 1;
    A.paths[i] = path; A.pathIdx[i] = 0; A.goalCell[i] = this.goal.cell;
  }

  update(dt) {
    const t0 = performance.now();
    this.arena.beginWrite();
    this.processRequests();
    this.accumulator += Math.min(dt * this.simSpeed, 0.2);
    for (let s = 0; this.accumulator >= FIXED_DT && s < 2; s++) {
      this.step(FIXED_DT);
      this.accumulator -= FIXED_DT;
    }
    if (this.accumulator >= FIXED_DT) this.accumulator = 0;
    this.lastSimDuration = performance.now() - t0;
    this.syncSharedTelemetry();
    this.arena.endWrite();
  }

  syncSharedTelemetry() {
    const { arena, grid, agents: A } = this;
    let actCnt = 0;
    for (let c = 0; c < grid.chunkCount; c++) {
      if (this.activeStamp[c] === this.frame && this.activeFields[c]) {
        actCnt++;
        const f = this.activeFields[c], off = c * CC;
        arena.activeKind[c] = f.kind === 'g' ? 2 : 1;
        arena.activeIntegration.set(f.integration, off);
        arena.activeFlowX.set(f.flowX, off);
        arena.activeFlowY.set(f.flowY, off);
      }
    }
    const sel = arena.load(CTRL.SELECTED_ID);
    const selPath = (sel >= 0 && sel < A.count) ? A.paths[sel] : null;
    if (selPath && selPath.graph === this.graph) {
      const cLen = Math.min(512, selPath.chunks.length), pLen = Math.min(512, selPath.portals.length);
      arena.selPathChunks.set(selPath.chunks.subarray(0, cLen));
      arena.selPathPortals.set(selPath.portals.subarray(0, pLen));
      arena.selPathGoal[0] = selPath.goalX;
      arena.selPathGoal[1] = selPath.goalY;
      arena.store(CTRL.SEL_CHUNKS_LEN, cLen);
      arena.store(CTRL.SEL_PORTALS_LEN, pLen);
    } else {
      arena.store(CTRL.SEL_CHUNKS_LEN, 0);
      arena.store(CTRL.SEL_PORTALS_LEN, 0);
    }

    arena.store(CTRL.FRAME, this.frame);
    arena.store(CTRL.AGENT_COUNT, A.count);
    arena.store(CTRL.ARRIVED_COUNT, this.arrivedAtGoal);
    arena.store(CTRL.ACTIVE_CHUNKS, actCnt);
    arena.store(CTRL.FLOW_CACHE_COUNT, this.flowCache.entries.size);
    arena.store(CTRL.SIM_DURATION_US, Math.round(this.lastSimDuration * 1000));
    const remMs = this.autoGoal ? Math.max(0, (this.autoGoal.interval - this.autoGoalTimer) * 1000) : 0;
    arena.store(CTRL.AUTOGOAL_REM_MS, Math.round(remMs));
  }

  step(h) {
    this.frame++;
    const { frame, agents: A, grid, hash } = this, n = A.count, { width: W, height: H } = grid;
    const rad = this.agentRadius, minDist = rad * 2, sepRad = rad * 3, sepR2 = sepRad * sepRad;
    const { px, py, vx, vy, state } = A;
    hash.build(px, py, n);
    const { cols, rows, start: cStart, items } = hash;
    const MAX_NEIGHBORS = 10;
    for (let i = 0; i < n; i++) {
      const xi = px[i], yi = py[i], gi = A.goalCell[i];
      let sx = 0, sy = 0, cx = 0, cy = 0, touch = 0, checked = 0;
      const by = hash.by(yi), bx = hash.bx(xi);
      neighborLoop:
      for (let yy = Math.max(0, by - 1), y1 = Math.min(rows - 1, by + 1); yy <= y1; yy++) {
        for (let xx = Math.max(0, bx - 1), x1 = Math.min(cols - 1, bx + 1); xx <= x1; xx++) {
          const b = yy * cols + xx;
          for (let k = cStart[b], end = cStart[b + 1]; k < end; k++) {
            const j = items[k];
            if (j === i) continue;
            let dx = xi - px[j], dy = yi - py[j], d2 = dx * dx + dy * dy;
            if (d2 >= sepR2) continue;
            let d = Math.sqrt(d2);
            if (d > 1e-5) { dx /= d; dy /= d; }
            else {
              const ang = ((Math.min(i, j) * 0.6180339887) % 1) * Math.PI * 2, s = i < j ? 1 : -1;
              dx = Math.cos(ang) * s; dy = Math.sin(ang) * s; d = 0;
            }
            const w = 1 - d / sepRad;
            sx += dx * w; sy += dy * w;
            if (d < minDist) {
              const push = (minDist - d) * 0.5;
              cx += dx * push; cy += dy * push;
              if (gi >= 0 && state[j] === 2 && A.goalCell[j] === gi) touch = 1;
            }
            if (++checked >= MAX_NEIGHBORS) break neighborLoop;
          }
        }
      }
      A.sepX[i] = sx; A.sepY[i] = sy; A.corrX[i] = cx; A.corrY[i] = cy; A.touching[i] = touch;
    }
    this.portalTargets.fill(0); this.portalOnPath.fill(0);
    let arrCnt = 0;
    const pCap = this.portalTargets.length;
    const crowdR = Math.sqrt((this.arrivedAtGoal * sepRad * sepRad * 0.866) / Math.PI) + 0.5;
    const contagR = Math.max(3, crowdR * 1.6), leaveR = crowdR * 2 + 4, vmax = this.maxSpeed * 1.25;
    for (let i = 0; i < n; i++) {
      if (A.repathCd[i] > 0) A.repathCd[i] -= h;
      let st = state[i], dvx = 0, dvy = 0;
      const tCell = clamp(px[i] | 0, 0, W - 1) + clamp(py[i] | 0, 0, H - 1) * W;
      const tCost = grid.costs[tCell];
      const terrainMult = tCost === COST_HIGHWAY ? 1.45 : tCost === COST_MUD ? 0.55 : 1.0;
      if (st === 1) {
        const field = this.resolveField(i), path = A.paths[i];
        if (field) {
          if (sampleFlowBilinear(field, px[i], py[i], this.heading)) {
            let sp = this.maxSpeed * terrainMult;
            if (field.kind === 'g' && path) {
              const d = Math.hypot(path.goalX - px[i], path.goalY - py[i]);
              if (d < 3) sp *= Math.max(0.35, d / 3);
            }
            dvx = this.heading.x * sp; dvy = this.heading.y * sp;
          }
          if (field.usageFrame !== frame) { field.usageFrame = frame; field.usageCount = 0; }
          const c = field.chunkId, af = this.activeFields[c];
          if (this.activeStamp[c] !== frame || !af || ++field.usageCount > af.usageCount) {
            this.activeStamp[c] = frame; this.activeFields[c] = field;
          }
        }
        if (path && path.graph === this.graph) {
          const idx = A.pathIdx[i];
          if (idx < path.portals.length && path.portals[idx] < pCap) this.portalTargets[path.portals[idx]]++;
          for (let k = idx; k < path.portals.length; k++) {
            const pid = path.portals[k];
            if (pid < pCap) this.portalOnPath[pid] = 1;
          }
        }
      }
      const gain = st === 1 ? this.steeringGain : this.steeringGain * 0.6;
      let nvx = vx[i] + ((dvx - vx[i]) * gain + A.sepX[i] * this.separationStrength) * h;
      let nvy = vy[i] + ((dvy - vy[i]) * gain + A.sepY[i] * this.separationStrength) * h;
      if (st === 2) { nvx *= 0.65; nvy *= 0.65; }
      const sp2 = nvx * nvx + nvy * nvy;
      if (sp2 > vmax * vmax) { const sc = vmax / Math.sqrt(sp2); nvx *= sc; nvy *= sc; }
      vx[i] = nvx; vy[i] = nvy;
      px[i] += nvx * h + A.corrX[i] * 0.5;
      py[i] += nvy * h + A.corrY[i] * 0.5;
      this.collide(i, W, H);
      const path = A.paths[i];
      if (path && (st === 1 || st === 2)) {
        const d = Math.hypot(path.goalX - px[i], path.goalY - py[i]);
        if (st === 1 && (d < 0.75 || (A.touching[i] && d < contagR))) {
          state[i] = st = 2;
        } else if (st === 2 && d > leaveR) {
          state[i] = st = 1; A.fields[i] = null;
        }
      }
      if (st === 2 && this.goal && A.goalCell[i] === this.goal.cell) arrCnt++;
    }
    if (this.arrivedAtGoal === 0 && arrCnt > 0) {
      this.arena.raiseEvent(EVENTS.ARRIVAL);
      if (this.onArrival) this.onArrival();
    }
    this.arrivedAtGoal = arrCnt;
    const ag = this.autoGoal;
    if (ag && ag.enabled && this.goal && n > 0) {
      this.autoGoalTimer += h;
      this.arrivalTimer = (this.arrivedAtGoal / n >= ag.arrivalPct) ? this.arrivalTimer + h : 0;
      const m = ag.mode;
      const hitArrival = (m === 'On Arrival' || m === 'Either') && this.arrivalTimer >= ag.delay;
      const hitTimer = (m === 'Timer' || m === 'Either') && this.autoGoalTimer >= ag.interval;
      if (hitArrival || hitTimer) this.pickRandomGoal();
    }
  }

  resolveField(i) {
    const { agents: A, grid } = this;
    const cx = clamp(A.px[i] | 0, 0, grid.width - 1), cy = clamp(A.py[i] | 0, 0, grid.height - 1);
    const chunk = grid.chunkIdAt(cx, cy), path = A.paths[i];
    if (!path || path.graph !== this.graph) {
      this.reqPath(i, true);
      const f = A.fields[i];
      return f && f.chunkId === chunk ? f : null;
    }
    let idx = A.pathIdx[i];
    if (path.chunks[idx] !== chunk) {
      let found = path.chunks.indexOf(chunk, idx + 1);
      if (found < 0) found = path.chunks.lastIndexOf(chunk, idx - 1);
      if (found < 0) { A.fields[i] = null; this.reqPath(i, true); return null; }
      A.pathIdx[i] = idx = found;
      A.fields[i] = null;
    }
    let field = A.fields[i];
    if (!field || field.chunkId !== chunk) {
      field = A.fields[i] = idx < path.portals.length
        ? this.flowCache.getPortalField(chunk, this.graph.portals[path.portals[idx]])
        : this.flowCache.getGoalField(chunk, path.goalCell);
    }
    if (!(field.integration[(cy - field.originY) * CS + (cx - field.originX)] < Infinity)) {
      this.reqPath(i, true); return null;
    }
    return field;
  }

  collide(i, W, H) {
    const { agents: A, grid } = this, costs = grid.costs, r = this.agentRadius;
    let x = clamp(A.px[i], r, W - r), y = clamp(A.py[i], r, H - r);
    if (x !== A.px[i]) A.vx[i] = 0;
    if (y !== A.py[i]) A.vy[i] = 0;
    const ccx = Math.min(W - 1, x | 0), ccy = Math.min(H - 1, y | 0);
    if (costs[ccy * W + ccx] >= COST_BLOCK) {
      const cell = grid.findNearestWalkable(ccx, ccy, 24);
      if (cell >= 0) { x = (cell % W) + 0.5; y = (cell / W | 0) + 0.5; }
      A.vx[i] = A.vy[i] = 0;
    }
    const x0 = Math.max(0, (x - r) | 0), x1 = Math.min(W - 1, (x + r) | 0);
    const y0 = Math.max(0, (y - r) | 0), y1 = Math.min(H - 1, (y + r) | 0);
    for (let cy = y0; cy <= y1; cy++) {
      for (let cx = x0; cx <= x1; cx++) {
        if (costs[cy * W + cx] < COST_BLOCK) continue;
        const qx = clamp(x, cx, cx + 1), qy = clamp(y, cy, cy + 1);
        const dx = x - qx, dy = y - qy, d2 = dx * dx + dy * dy;
        if (d2 >= r * r || d2 < 1e-12) continue;
        const d = Math.sqrt(d2), nx = dx / d, ny = dy / d;
        x += nx * (r - d); y += ny * (r - d);
        const vn = A.vx[i] * nx + A.vy[i] * ny;
        if (vn < 0) { A.vx[i] -= vn * nx; A.vy[i] -= vn * ny; }
      }
    }
    A.px[i] = x; A.py[i] = y;
  }
}
//#endregion

//#region 13: ComlinkWorkerHostAndSharedClient
/**
 * Comlink-exposed Worker Host that runs `Simulation` inside a Dedicated Web Worker
 * and synchronizes state through `SharedArena` and `Atomics`.
 */
export class WorkerSimulationHost {
  constructor() {
    this.sim = null;
    this.arena = null;
    this.timerId = null;
    this.lastTick = 0;
  }

  /**
   * Initializes the worker simulation engine over the shared memory buffer.
   * @param {SharedArrayBuffer|ArrayBuffer} buffer - Shared memory buffer.
   * @param {Object} opts - Initial simulation and world options.
   * @returns {Object} Initial portal graph snapshot.
   */
  init(buffer, opts) {
    this.arena = new SharedArena(buffer);
    this.sim = new Simulation({ ...opts, arena: this.arena });
    this.sim.syncSharedTelemetry();
    this.startLoop();
    return {
      isShared: this.arena.isShared,
      graph: this.sim.graph.toSnapshot(),
      fallbackBuffer: this.arena.isShared ? null : this.arena.buffer.slice(0)
    };
  }

  startLoop() {
    if (this.timerId) clearInterval(this.timerId);
    this.lastTick = performance.now();
    this.timerId = setInterval(() => {
      const now = performance.now();
      const dt = Math.min(0.1, (now - this.lastTick) / 1000);
      this.lastTick = now;
      if (!this.arena || !this.sim) return;
      this.arena.add(CTRL.WORKER_TICKS, 1);
      if (!this.arena.load(CTRL.PAUSED)) {
        this.sim.update(dt);
      }
    }, 1000 / 60);
  }

  setGoal(x, y, playAudio = true) {
    const ok = this.sim.setGoal(x, y, playAudio);
    this.sim.syncSharedTelemetry();
    return ok;
  }

  pickRandomGoal() {
    const ok = this.sim.pickRandomGoal();
    this.sim.syncSharedTelemetry();
    return ok;
  }

  paintTerrain(wx, wy, r, cost) {
    const changed = this.sim.paintTerrain(wx, wy, r, cost);
    if (changed > 0) this.sim.syncSharedTelemetry();
    return changed > 0 ? this.sim.graph.toSnapshot() : null;
  }

  regenerate(seed, cx, cy, mapOpts) {
    this.sim.regenerate(seed, cx, cy, mapOpts);
    this.sim.syncSharedTelemetry();
    return this.sim.graph.toSnapshot();
  }

  setAgentCount(cnt) {
    this.sim.setAgentCount(cnt);
    this.sim.syncSharedTelemetry();
  }

  respawnAll() {
    this.sim.respawnAll();
    this.sim.syncSharedTelemetry();
  }

  updateParams(params) {
    const { sim } = this;
    if (!sim) return;
    if (params.simSpeed !== undefined) sim.simSpeed = params.simSpeed;
    if (params.spawnPattern !== undefined) sim.spawnPattern = params.spawnPattern;
    if (params.agentRadius !== undefined) {
      sim.agentRadius = params.agentRadius;
      sim.hash = new SpatialHash(sim.grid.width, sim.grid.height, Math.max(1.2, sim.agentRadius * 3.2), MAX_AGENTS);
    }
    if (params.maxSpeed !== undefined) sim.maxSpeed = params.maxSpeed;
    if (params.steeringGain !== undefined) sim.steeringGain = params.steeringGain;
    if (params.separationStrength !== undefined) sim.separationStrength = params.separationStrength;
    if (params.pathBudgetMs !== undefined) sim.pathBudgetMs = params.pathBudgetMs;
    if (params.autoGoal !== undefined) Object.assign(sim.autoGoal, params.autoGoal);
  }

  getGraphSnapshot() {
    return this.sim.graph.toSnapshot();
  }

  pullFallbackBuffer() {
    return this.arena.buffer.slice(0);
  }
}

/**
 * Main-thread client facade that reads zero-copy `SharedArrayBuffer` state via `Atomics`
 * and dispatches commands to `WorkerSimulationHost` via `Comlink`.
 */
export class SharedSimulationClient {
  /**
   * @param {SharedArena} arena - Shared memory arena.
   * @param {Object} remote - Comlink-wrapped `WorkerSimulationHost` proxy.
   * @param {Object} initGraph - Initial serialized portal graph.
   * @param {Object} opts - Client callbacks and options.
   */
  constructor(arena, remote, initGraph, opts = {}) {
    this.arena = arena;
    this.remote = remote;
    this.onGoal = opts.onGoal ?? null;
    this.onArrival = opts.onArrival ?? null;
    this.onGraphRebuilt = opts.onGraphRebuilt ?? null;
    this.frame = 0;
    this.arrivedAtGoal = 0;
    this.lastSimDuration = 0;
    this.autoGoalTimerRem = 0;
    this.activeChunkCount = 0;
    this.flowCacheSize = 0;
    this.workerTicks = 0;
    this.goal = null;
    this.graph = initGraph;
    this._syncingGraph = false;

    const cx = opts.chunksX ?? 10, cy = opts.chunksY ?? 6;
    this.grid = new GridWorld(cx, cy, arena);
    this.activeStamp = arena.activeStamp;
    this.portalTargets = arena.portalTargets;
    this.portalOnPath = arena.portalOnPath;

    this.activeFields = Array.from({ length: MAX_CHUNKS }, (_, c) => ({
      chunkId: c,
      kind: 'p',
      originX: 0,
      originY: 0,
      integration: arena.activeIntegration.subarray(c * CC, (c + 1) * CC),
      flowX: arena.activeFlowX.subarray(c * CC, (c + 1) * CC),
      flowY: arena.activeFlowY.subarray(c * CC, (c + 1) * CC)
    }));

    this.selectedPathProxy = {
      chunks: new Int32Array(0),
      portals: new Int32Array(0),
      goalX: 0,
      goalY: 0,
      graph: this.graph
    };

    const pathsProxy = new Proxy([], {
      get: (_, prop) => {
        const idx = Number(prop);
        if (Number.isInteger(idx) && idx === this.arena.load(CTRL.SELECTED_ID)) {
          return this.selectedPathProxy.chunks.length > 0 ? this.selectedPathProxy : null;
        }
        return null;
      }
    });

    this.agents = {
      count: opts.agentCount ?? 10_000,
      px: arena.px,
      py: arena.py,
      vx: arena.vx,
      vy: arena.vy,
      state: arena.state,
      pathIdx: arena.pathIdx,
      goalCell: arena.goalCell,
      paths: pathsProxy
    };

    this.flowCache = {
      entries: {
        get size() { return arena.load(CTRL.FLOW_CACHE_COUNT); }
      }
    };

    this.refreshChunkOrigins();
    this.syncFromShared();
  }

  refreshChunkOrigins() {
    for (let c = 0; c < this.grid.chunkCount; c++) {
      this.activeFields[c].originX = this.grid.chunkOriginX(c);
      this.activeFields[c].originY = this.grid.chunkOriginY(c);
    }
  }

  /**
   * Reads consistent frame telemetry and events from `SharedArena` using `Atomics`.
   */
  syncFromShared() {
    const { arena } = this;
    if (!arena.isShared && !this._pullingFallback) {
      this._pullingFallback = true;
      this.remote.pullFallbackBuffer().then(buf => {
        new Uint8Array(arena.buffer).set(new Uint8Array(buf));
        this._pullingFallback = false;
      });
    }

    arena.readConsistent(() => {
      const cx = arena.load(CTRL.CHUNKS_X), cy = arena.load(CTRL.CHUNKS_Y);
      if (cx > 0 && cy > 0 && (cx !== this.grid.cx || cy !== this.grid.cy)) {
        this.grid.cx = cx; this.grid.cy = cy;
        this.grid.width = cx * CS; this.grid.height = cy * CS;
        this.grid.chunkCount = cx * cy;
        this.grid.costs = arena.gridCosts.subarray(0, this.grid.width * this.grid.height);
        this.refreshChunkOrigins();
      }
      this.grid.version = arena.load(CTRL.GRID_VERSION);
      this.frame = arena.load(CTRL.FRAME);
      this.agents.count = arena.load(CTRL.AGENT_COUNT);
      this.arrivedAtGoal = arena.load(CTRL.ARRIVED_COUNT);
      this.activeChunkCount = arena.load(CTRL.ACTIVE_CHUNKS);
      this.flowCacheSize = arena.load(CTRL.FLOW_CACHE_COUNT);
      this.lastSimDuration = arena.load(CTRL.SIM_DURATION_US) / 1000;
      this.autoGoalTimerRem = arena.load(CTRL.AUTOGOAL_REM_MS) / 1000;
      this.workerTicks = arena.load(CTRL.WORKER_TICKS);

      const gx = arena.load(CTRL.GOAL_X), gy = arena.load(CTRL.GOAL_Y), gc = arena.load(CTRL.GOAL_CHUNK);
      this.goal = gx >= 0 ? { x: gx, y: gy, chunk: gc } : null;

      for (let c = 0; c < this.grid.chunkCount; c++) {
        if (this.activeStamp[c] === this.frame) {
          this.activeFields[c].kind = arena.activeKind[c] === 2 ? 'g' : 'p';
        }
      }

      const cLen = arena.load(CTRL.SEL_CHUNKS_LEN), pLen = arena.load(CTRL.SEL_PORTALS_LEN);
      this.selectedPathProxy.chunks = arena.selPathChunks.subarray(0, cLen);
      this.selectedPathProxy.portals = arena.selPathPortals.subarray(0, pLen);
      this.selectedPathProxy.goalX = arena.selPathGoal[0];
      this.selectedPathProxy.goalY = arena.selPathGoal[1];
      this.selectedPathProxy.graph = this.graph;
    });

    const ev = arena.consumeEvents();
    if ((ev & EVENTS.GOAL) && this.onGoal) this.onGoal(this.goal);
    if ((ev & EVENTS.ARRIVAL) && this.onArrival) this.onArrival();
    if ((ev & EVENTS.GRAPH_REBUILT) && !this._syncingGraph) {
      this._syncingGraph = true;
      this.remote.getGraphSnapshot().then(g => {
        this.graph = g;
        this.selectedPathProxy.graph = g;
        this._syncingGraph = false;
        if (this.onGraphRebuilt) this.onGraphRebuilt(g);
      });
    }
  }

  setPaused(paused) { this.arena.store(CTRL.PAUSED, paused ? 1 : 0); }
  setSelectedId(id) { this.arena.store(CTRL.SELECTED_ID, id | 0); }
  setGoal(x, y, playAudio = true) { return this.remote.setGoal(x, y, playAudio); }
  pickRandomGoal() { return this.remote.pickRandomGoal(); }
  setAgentCount(cnt) {
    this.agents.count = clamp(Math.round(cnt), 0, MAX_AGENTS);
    this.arena.store(CTRL.AGENT_COUNT, this.agents.count);
    return this.remote.setAgentCount(cnt);
  }
  respawnAll() { return this.remote.respawnAll(); }
  updateParams(params) { return this.remote.updateParams(params); }

  async paintTerrain(wx, wy, r, cost) {
    const newGraph = await this.remote.paintTerrain(wx, wy, r, cost);
    if (newGraph) {
      this.graph = newGraph;
      this.selectedPathProxy.graph = newGraph;
    }
  }

  async regenerate(seed, cx, cy, mapOpts) {
    const newGraph = await this.remote.regenerate(seed, cx, cy, mapOpts);
    this.graph = newGraph;
    this.selectedPathProxy.graph = newGraph;
    this.syncFromShared();
  }

  findAgentNear(wx, wy, maxRadius = 1.6) {
    const A = this.agents, n = A.count;
    let best = -1, bestD2 = maxRadius * maxRadius;
    for (let i = 0; i < n; i++) {
      const dx = A.px[i] - wx, dy = A.py[i] - wy, d2 = dx * dx + dy * dy;
      if (d2 < bestD2) { bestD2 = d2; best = i; }
    }
    return best;
  }
}

/**
 * Spawns a Dedicated Module Web Worker, binds it with `Comlink`, initializes `SharedArena`
 * (`SharedArrayBuffer` + `Atomics`), and returns the connected `SharedSimulationClient`.
 * @param {Object} Comlink - Imported Comlink module.
 * @param {Object} opts - Initial world and simulation configuration.
 * @param {string} [comlinkEsmUrl] - CDN URL for Comlink inside the worker.
 * @returns {Promise<SharedSimulationClient>} Connected shared-memory simulation client.
 */
export async function createComlinkSimulation(
  Comlink,
  opts = {},
  comlinkEsmUrl = 'https://cdn.jsdelivr.net/npm/comlink@4.4.2/dist/esm/comlink.min.mjs'
) {
  const arena = new SharedArena();
  const apiUrl = new URL(import.meta.url).href;
  const workerSrc = `
    import * as Comlink from ${JSON.stringify(comlinkEsmUrl)};
    import { WorkerSimulationHost } from ${JSON.stringify(apiUrl)};
    Comlink.expose(new WorkerSimulationHost());
  `;
  const blob = new Blob([workerSrc], { type: 'text/javascript' });
  const worker = new Worker(URL.createObjectURL(blob), { type: 'module', name: 'HFFP-Comlink-Worker' });
  const remote = Comlink.wrap(worker);

  const serializableOpts = {
    seed: opts.seed,
    chunksX: opts.chunksX,
    chunksY: opts.chunksY,
    preset: opts.preset,
    wallDensity: opts.wallDensity,
    mudDensity: opts.mudDensity,
    agentCount: opts.agentCount,
    agentRadius: opts.agentRadius,
    spawnPattern: opts.spawnPattern,
    maxSpeed: opts.maxSpeed,
    steeringGain: opts.steeringGain,
    separationStrength: opts.separationStrength,
    pathBudgetMs: opts.pathBudgetMs,
    simSpeed: opts.simSpeed,
    autoGoal: opts.autoGoal ? { ...opts.autoGoal } : null
  };

  const initRes = await remote.init(arena.buffer, serializableOpts);
  if (!initRes.isShared && initRes.fallbackBuffer) {
    new Uint8Array(arena.buffer).set(new Uint8Array(initRes.fallbackBuffer));
  }
  return new SharedSimulationClient(arena, remote, initRes.graph, opts);
}

/**
 * Executes a deterministic headless performance benchmark across presets.
 * @param {Object} [opts] - Benchmark options.
 * @returns {Array<Object>} Benchmark results table.
 */
export function runBenchmark({ agentCount = 10_000, steps = 30 } = {}) {
  const presets = ['The Grand Citadel', 'The Great Maze', 'Twin Hourglass Bottlenecks', 'Highway vs Swamp'];
  const results = [];
  for (const preset of presets) {
    const tBuild0 = performance.now();
    const sim = new Simulation({ seed: 1337666, chunksX: 12, chunksY: 8, preset, agentCount });
    const buildMs = performance.now() - tBuild0;

    const tEdit0 = performance.now();
    sim.paintTerrain(sim.grid.width * 0.5, sim.grid.height * 0.5, 3.0, COST_BLOCK);
    const incrementalEditMs = performance.now() - tEdit0;

    const tStep0 = performance.now();
    for (let s = 0; s < steps; s++) sim.update(FIXED_DT);
    const avgStepMs = (performance.now() - tStep0) / steps;

    results.push({
      Preset: preset,
      SAB: sim.arena.isShared ? 'SharedArrayBuffer' : 'ArrayBuffer',
      Portals: sim.graph.portals.length,
      'Init Build (ms)': +buildMs.toFixed(2),
      'Surgical Edit (ms)': +incrementalEditMs.toFixed(2),
      'Avg Step 10k (ms)': +avgStepMs.toFixed(2),
      'Cached Fields': sim.flowCache.entries.size
    });
  }
  return results;
}

export default Simulation;
//#endregion