"use strict";
(() => {
  // ---- Model (mirrors Node / NodeLink in the Unity package) ----------------
  const WORLD_W = 16, WORLD_H = 10, LENGTH_SAMPLES = 20, CURVE_SEGMENTS = 20;
  let nodes = [];
  let nextId = 1;

  const vec = (x, y) => ({ x, y });
  const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

  function addNode(x, y) {
    const n = { id: nextId++, x, y, links: [] };
    nodes.push(n);
    return n;
  }
  function getLink(a, b) { return a.links.find(l => l.to === b) || null; }

  function connect(a, b, weight, bidirectional) {
    if (!a || !b || a === b) return null;
    let link = getLink(a, b);
    if (!link) {
      link = { to: b, weight, spline: false, t1: vec(0, 0), t2: vec(0, 0) };
      a.links.push(link);
    }
    link.weight = weight;
    if (bidirectional) connect(b, a, weight, false);
    return link;
  }
  function removeLink(a, link, bidirectional) {
    const b = link.to;
    a.links = a.links.filter(l => l !== link);
    if (bidirectional) b.links = b.links.filter(l => l.to !== a);
  }
  function removeNode(n) {
    nodes = nodes.filter(x => x !== n);
    for (const o of nodes) o.links = o.links.filter(l => l.to !== n);
  }

  function evalLink(owner, l, t) {
    const a = owner, b = l.to;
    if (!l.spline) return vec(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
    const c1 = vec(a.x + l.t1.x, a.y + l.t1.y), c2 = vec(b.x + l.t2.x, b.y + l.t2.y);
    const u = 1 - t, w0 = u * u * u, w1 = 3 * u * u * t, w2 = 3 * u * t * t, w3 = t * t * t;
    return vec(w0 * a.x + w1 * c1.x + w2 * c2.x + w3 * b.x, w0 * a.y + w1 * c1.y + w2 * c2.y + w3 * b.y);
  }
  function linkDistance(owner, l) {
    if (!l.spline) return dist(owner, l.to);
    let len = 0, prev = owner;
    for (let i = 1; i <= LENGTH_SAMPLES; i++) {
      const p = evalLink(owner, l, i / LENGTH_SAMPLES);
      len += dist(prev, p);
      prev = p;
    }
    return len;
  }
  const linkCost = (owner, l) => linkDistance(owner, l) * l.weight;

  function defaultTangents(a, b) {
    const dx = b.x - a.x, dy = b.y - a.y;
    const px = -dy * 0.35, py = dx * 0.35;
    return [vec(dx / 3 + px, dy / 3 + py), vec(-dx / 3 + px, -dy / 3 + py)];
  }

  // ---- A* (mirrors Pathfinder.cs) -------------------------------------------
  function getMinWeight() {
    let min = Infinity;
    for (const n of nodes) for (const l of n.links) if (l.weight < min) min = l.weight;
    return min === Infinity ? 1 : min;
  }

  function findPath(start, end, useHeuristic) {
    const res = { found: false, path: [], cost: 0, expanded: [] };
    if (!start || !end) return res;
    if (start === end) { res.found = true; res.path = [start]; return res; }

    const hScale = useHeuristic ? Math.min(1, getMinWeight()) : 0;
    const h = n => dist(n, end) * hScale;
    const g = new Map([[start, 0]]);
    const from = new Map();
    const closed = new Set();
    const heap = [];

    const push = (node, f) => {
      heap.push({ node, f });
      let i = heap.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (heap[p].f <= heap[i].f) break;
        [heap[p], heap[i]] = [heap[i], heap[p]];
        i = p;
      }
    };
    const pop = () => {
      const top = heap[0].node, last = heap.pop();
      if (heap.length) {
        heap[0] = last;
        let i = 0;
        for (; ;) {
          const l = 2 * i + 1, r = l + 1;
          let s = i;
          if (l < heap.length && heap[l].f < heap[s].f) s = l;
          if (r < heap.length && heap[r].f < heap[s].f) s = r;
          if (s === i) break;
          [heap[s], heap[i]] = [heap[i], heap[s]];
          i = s;
        }
      }
      return top;
    };

    push(start, h(start));
    while (heap.length) {
      const cur = pop();
      if (closed.has(cur)) continue;
      closed.add(cur);
      res.expanded.push(cur);

      if (cur === end) {
        res.found = true;
        res.cost = g.get(end);
        for (let n = end; n !== start; n = from.get(n)) res.path.push(n);
        res.path.push(start);
        res.path.reverse();
        return res;
      }
      const cg = g.get(cur);
      for (const l of cur.links) {
        const next = l.to;
        if (closed.has(next)) continue;
        const tent = cg + linkCost(cur, l);
        if (g.has(next) && tent >= g.get(next)) continue;
        g.set(next, tent);
        from.set(next, cur);
        push(next, tent + h(next));
      }
    }
    return res;
  }

  function closestNode(p) {
    let best = null, bestD = Infinity;
    for (const n of nodes) { const d = dist(n, p); if (d < bestD) { bestD = d; best = n; } }
    return best;
  }

  // ---- State / UI ------------------------------------------------------------
  const $ = id => document.getElementById(id);
  const canvas = $("canvas"), ctx = canvas.getContext("2d");
  const ui = {
    weight: $("weight"), bidir: $("bidir"), radius: $("radius"),
    showLabels: $("showLabels"), showExplored: $("showExplored"), walker: $("walker"), dijkstra: $("dijkstra"),
    lWeight: $("lWeight"), lSpline: $("lSpline"),
  };
  const state = {
    tool: "path", start: null, end: null, result: null,
    selected: null,            // { node, link }
    drag: null,                // { type: "node"|"h1"|"h2"|"link", ... }
    hover: null,
    k: 1,
  };

  const HINTS = {
    path: "Click a start point, then an end point. Each snaps to the closest node, like TryFindPath(Vector2, Vector2).",
    select: "Drag nodes to move them. Click a link to edit its weight or turn it into a spline.",
    add: "Click empty space to add a node.",
    link: "Drag from one node to another. Linking an existing pair updates its weight.",
    delete: "Click a node or a link to remove it.",
  };

  function setTool(t) {
    state.tool = t;
    state.drag = null;
    document.querySelectorAll("#tools button").forEach(b => b.classList.toggle("active", b.dataset.tool === t));
    $("hint").textContent = HINTS[t];
    canvas.style.cursor = t === "select" ? "default" : t === "delete" ? "not-allowed" : "crosshair";
    draw();
  }

  function recompute() {
    if (state.start && !nodes.includes(state.start)) state.start = null;
    if (state.end && !nodes.includes(state.end)) state.end = null;
    state.result = state.start && state.end ? findPath(state.start, state.end, !ui.dijkstra.checked) : null;
    const r = state.result;
    $("sStatus").textContent = !r ? (state.start ? "Pick an end" : "Pick a start and end") : r.found ? "Path found" : "No path";
    $("sCost").textContent = r && r.found ? r.cost.toFixed(2) : "-";
    $("sNodes").textContent = r && r.found ? r.path.length : "-";
    $("sExpanded").textContent = r ? r.expanded.length : "-";
    restartWalker();
    draw();
  }

  function syncLinkPanel() {
    const s = state.selected;
    $("linkPanel").hidden = !!s;
    $("linkEdit").hidden = !s;
    if (!s) return;
    ui.lWeight.value = s.link.weight;
    $("lwOut").textContent = s.link.weight.toFixed(1);
    ui.lSpline.checked = s.link.spline;
    $("lInfo").textContent = `Node ${s.node.id} to ${s.link.to.id}, length ${linkDistance(s.node, s.link).toFixed(2)}, cost ${linkCost(s.node, s.link).toFixed(2)}`;
  }

  const twin = (n, l) => (ui.bidir.checked ? getLink(l.to, n) : null);

  // ---- Drawing ---------------------------------------------------------------
  const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  let colors = {};
  function readColors() {
    for (const n of ["node", "link", "spline", "path", "explored", "label", "grid", "text"]) colors[n] = css("--" + n);
  }

  function resize() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(rect.width * dpr);
    canvas.height = Math.round(rect.height * dpr);
    state.k = rect.width / WORLD_W;
    ctx.setTransform(dpr * state.k, 0, 0, dpr * state.k, 0, 0);
    draw();
  }
  const px = n => n / state.k; // screen pixels -> world units

  function strokeLink(owner, l) {
    ctx.beginPath();
    const segs = l.spline ? CURVE_SEGMENTS : 1;
    ctx.moveTo(owner.x, owner.y);
    for (let i = 1; i <= segs; i++) { const p = evalLink(owner, l, i / segs); ctx.lineTo(p.x, p.y); }
    ctx.stroke();
  }

  function arrow(owner, l) {
    const end = l.to, before = evalLink(owner, l, 0.9);
    let dx = end.x - before.x, dy = end.y - before.y;
    const len = Math.hypot(dx, dy);
    if (!len) return;
    dx /= len; dy /= len;
    const tip = evalLink(owner, l, 0.85), s = px(9);
    ctx.beginPath();
    ctx.moveTo(tip.x - dx * s + -dy * s * 0.5, tip.y - dy * s + dx * s * 0.5);
    ctx.lineTo(tip.x, tip.y);
    ctx.lineTo(tip.x - dx * s - -dy * s * 0.5, tip.y - dy * s - dx * s * 0.5);
    ctx.stroke();
  }

  function draw() {
    ctx.clearRect(0, 0, WORLD_W, WORLD_H);

    // grid
    ctx.strokeStyle = colors.grid; ctx.lineWidth = px(1);
    ctx.beginPath();
    for (let x = 1; x < WORLD_W; x++) { ctx.moveTo(x, 0); ctx.lineTo(x, WORLD_H); }
    for (let y = 1; y < WORLD_H; y++) { ctx.moveTo(0, y); ctx.lineTo(WORLD_W, y); }
    ctx.stroke();

    // links
    ctx.lineCap = "round"; ctx.lineJoin = "round";
    for (const n of nodes) {
      for (const l of n.links) {
        const sel = state.selected && state.selected.link === l;
        const touch = state.hover === n || state.hover === l.to;
        ctx.strokeStyle = l.spline ? colors.spline : colors.link;
        ctx.globalAlpha = sel || touch ? 1 : 0.55;
        ctx.lineWidth = px(sel ? 3.5 : 2);
        strokeLink(n, l);
        ctx.lineWidth = px(1.5);
        arrow(n, l);
      }
    }
    ctx.globalAlpha = 1;

    // path
    const r = state.result;
    if (r && r.found) {
      ctx.strokeStyle = colors.path; ctx.lineWidth = px(5);
      for (let i = 0; i + 1 < r.path.length; i++) strokeLink(r.path[i], getLink(r.path[i], r.path[i + 1]));
    }

    // labels
    if (ui.showLabels.checked) {
      ctx.font = `${px(11)}px system-ui, sans-serif`;
      ctx.textAlign = "center"; ctx.textBaseline = "bottom";
      ctx.fillStyle = colors.label;
      for (const n of nodes) for (const l of n.links) {
        // draw one label per pair when two-way with equal weight to limit clutter
        const back = getLink(l.to, n);
        if (back && n.id > l.to.id && back.weight === l.weight && back.spline === l.spline) continue;
        const m = evalLink(n, l, 0.5);
        ctx.fillText(`${linkDistance(n, l).toFixed(1)} x ${l.weight.toFixed(1)}`, m.x, m.y - px(4));
      }
    }

    // spline handles
    const s = state.selected;
    if (s && s.link.spline) {
      const a = s.node, b = s.link.to;
      const h1 = vec(a.x + s.link.t1.x, a.y + s.link.t1.y), h2 = vec(b.x + s.link.t2.x, b.y + s.link.t2.y);
      ctx.strokeStyle = colors.label; ctx.globalAlpha = 0.5; ctx.lineWidth = px(1);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(h1.x, h1.y); ctx.moveTo(b.x, b.y); ctx.lineTo(h2.x, h2.y); ctx.stroke();
      ctx.globalAlpha = 1; ctx.fillStyle = colors.text;
      for (const h of [h1, h2]) { ctx.beginPath(); ctx.arc(h.x, h.y, px(6), 0, Math.PI * 2); ctx.fill(); }
    }

    // link rubber band
    if (state.drag && state.drag.type === "linking") {
      ctx.strokeStyle = colors.link; ctx.lineWidth = px(2); ctx.setLineDash([px(6), px(5)]);
      ctx.beginPath(); ctx.moveTo(state.drag.from.x, state.drag.from.y); ctx.lineTo(state.drag.at.x, state.drag.at.y); ctx.stroke();
      ctx.setLineDash([]);
    }

    // nodes
    const expanded = ui.showExplored.checked && r ? new Set(r.expanded) : null;
    const radius = 0.17;
    for (const n of nodes) {
      ctx.fillStyle = colors.node;
      ctx.beginPath(); ctx.arc(n.x, n.y, radius, 0, Math.PI * 2); ctx.fill();
      if (expanded && expanded.has(n)) {
        ctx.strokeStyle = colors.explored; ctx.lineWidth = px(3);
        ctx.beginPath(); ctx.arc(n.x, n.y, radius + px(4), 0, Math.PI * 2); ctx.stroke();
      }
      if (n === state.hover || (state.selected && state.selected.node === n)) {
        ctx.strokeStyle = colors.text; ctx.lineWidth = px(2);
        ctx.beginPath(); ctx.arc(n.x, n.y, radius + px(1), 0, Math.PI * 2); ctx.stroke();
      }
    }

    // start / end markers
    const mark = (n, text) => {
      if (!n) return;
      ctx.fillStyle = colors.path;
      ctx.beginPath(); ctx.arc(n.x, n.y, radius + px(3), 0, Math.PI * 2); ctx.fill();
      ctx.fillStyle = "#fff"; ctx.font = `bold ${px(11)}px system-ui, sans-serif`;
      ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillText(text, n.x, n.y + px(0.5));
    };
    mark(state.start, "S"); mark(state.end, "E");

    // walker
    if (walker.pos) {
      ctx.fillStyle = colors.text; ctx.strokeStyle = colors.path; ctx.lineWidth = px(3);
      ctx.beginPath(); ctx.arc(walker.pos.x, walker.pos.y, px(8), 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    }
  }

  // ---- Walker animation ------------------------------------------------------
  const walker = { pos: null, seg: 0, t: 0, raf: 0, last: 0, speed: 4 };
  function restartWalker() {
    cancelAnimationFrame(walker.raf);
    walker.raf = 0; walker.pos = null; walker.seg = 0; walker.t = 0;
    const r = state.result;
    if (!ui.walker.checked || !r || !r.found || r.path.length < 2) return;
    walker.pos = vec(r.path[0].x, r.path[0].y);
    walker.last = performance.now();
    walker.raf = requestAnimationFrame(stepWalker);
  }
  function stepWalker(now) {
    const r = state.result;
    if (!r || !r.found) return;
    const dt = Math.min(0.05, (now - walker.last) / 1000);
    walker.last = now;
    let move = walker.speed * dt;
    while (move > 0) {
      const a = r.path[walker.seg], l = getLink(a, r.path[walker.seg + 1]);
      if (!l) break;
      const len = Math.max(linkDistance(a, l), 1e-6);
      const remaining = (1 - walker.t) * len;
      if (move < remaining) { walker.t += move / len; move = 0; }
      else {
        move -= remaining; walker.seg++; walker.t = 0;
        if (walker.seg >= r.path.length - 1) { walker.seg = 0; move = 0; walker.t = 0; }
      }
    }
    const a = r.path[walker.seg], l = getLink(a, r.path[walker.seg + 1]);
    if (l) walker.pos = evalLink(a, l, walker.t);
    draw();
    walker.raf = requestAnimationFrame(stepWalker);
  }

  // ---- Hit testing -----------------------------------------------------------
  function worldPoint(e) {
    const rect = canvas.getBoundingClientRect();
    return vec((e.clientX - rect.left) / state.k, (e.clientY - rect.top) / state.k);
  }
  function pickNode(p) {
    let best = null, bestD = px(14);
    for (const n of nodes) { const d = dist(n, p); if (d < bestD) { bestD = d; best = n; } }
    return best;
  }
  function pickLink(p) {
    let best = null, bestD = px(9);
    for (const n of nodes) for (const l of n.links) {
      let prev = vec(n.x, n.y);
      const segs = l.spline ? 24 : 1;
      for (let i = 1; i <= segs; i++) {
        const q = evalLink(n, l, i / segs);
        const d = pointSegDist(p, prev, q);
        if (d < bestD) { bestD = d; best = { node: n, link: l }; }
        prev = q;
      }
    }
    return best;
  }
  function pointSegDist(p, a, b) {
    const dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
    const t = len2 ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2)) : 0;
    return Math.hypot(p.x - (a.x + dx * t), p.y - (a.y + dy * t));
  }
  function pickHandle(p) {
    const s = state.selected;
    if (!s || !s.link.spline) return null;
    const h1 = vec(s.node.x + s.link.t1.x, s.node.y + s.link.t1.y);
    const h2 = vec(s.link.to.x + s.link.t2.x, s.link.to.y + s.link.t2.y);
    if (dist(p, h1) < px(12)) return "h1";
    if (dist(p, h2) < px(12)) return "h2";
    return null;
  }
  const clampP = p => vec(Math.max(0.2, Math.min(WORLD_W - 0.2, p.x)), Math.max(0.2, Math.min(WORLD_H - 0.2, p.y)));

  // ---- Pointer input ---------------------------------------------------------
  canvas.addEventListener("pointerdown", e => {
    canvas.setPointerCapture(e.pointerId);
    const p = worldPoint(e), node = pickNode(p);

    switch (state.tool) {
      case "path":
        if (!nodes.length) return;
        if (!state.start || state.end) { state.start = closestNode(p); state.end = null; }
        else state.end = closestNode(p);
        recompute();
        break;
      case "add":
        if (!node) { const q = clampP(p); addNode(q.x, q.y); recompute(); }
        break;
      case "link":
        if (node) state.drag = { type: "linking", from: node, at: p };
        break;
      case "delete": {
        if (node) { removeNode(node); state.selected = null; syncLinkPanel(); recompute(); break; }
        const hit = pickLink(p);
        if (hit) {
          removeLink(hit.node, hit.link, ui.bidir.checked);
          state.selected = null; syncLinkPanel(); recompute();
        }
        break;
      }
      case "select": {
        const handle = pickHandle(p);
        if (handle) { state.drag = { type: handle }; break; }
        if (node) { state.drag = { type: "node", node }; state.selected = null; syncLinkPanel(); draw(); break; }
        state.selected = pickLink(p);
        syncLinkPanel(); draw();
        break;
      }
    }
  });

  canvas.addEventListener("pointermove", e => {
    const p = worldPoint(e), d = state.drag;
    if (!d) {
      const h = pickNode(p);
      if (h !== state.hover) { state.hover = h; draw(); }
      return;
    }
    if (d.type === "node") {
      const q = clampP(p);
      d.node.x = q.x; d.node.y = q.y;
      recomputeKeepWalker();
    } else if (d.type === "linking") {
      d.at = p; state.hover = pickNode(p); draw();
    } else if (d.type === "h1" || d.type === "h2") {
      const s = state.selected, base = d.type === "h1" ? s.node : s.link.to;
      const off = vec(p.x - base.x, p.y - base.y);
      if (d.type === "h1") s.link.t1 = off; else s.link.t2 = off;
      const t = twin(s.node, s.link);
      if (t) { t.t1 = s.link.t2; t.t2 = s.link.t1; t.spline = true; }
      syncLinkPanel(); recomputeKeepWalker();
    }
  });

  function recomputeKeepWalker() {
    const keep = { seg: walker.seg, t: walker.t };
    recompute();
    if (walker.raf && state.result && keep.seg < state.result.path.length - 1) { walker.seg = keep.seg; walker.t = keep.t; }
  }

  canvas.addEventListener("pointerup", e => {
    const d = state.drag;
    state.drag = null;
    if (d && d.type === "linking") {
      const target = pickNode(worldPoint(e));
      if (target && target !== d.from) {
        connect(d.from, target, parseFloat(ui.weight.value), ui.bidir.checked);
        recompute();
      } else draw();
    }
  });
  canvas.addEventListener("pointercancel", () => { state.drag = null; draw(); });
  canvas.addEventListener("pointerleave", () => { if (!state.drag && state.hover) { state.hover = null; draw(); } });

  // ---- Controls --------------------------------------------------------------
  document.querySelectorAll("#tools button").forEach(b => b.addEventListener("click", () => setTool(b.dataset.tool)));

  ui.weight.addEventListener("input", () => { $("wOut").textContent = parseFloat(ui.weight.value).toFixed(1); });
  ui.radius.addEventListener("input", () => { $("rOut").textContent = parseFloat(ui.radius.value).toFixed(1); });

  $("autoLink").addEventListener("click", () => {
    const r2 = parseFloat(ui.radius.value) ** 2, w = parseFloat(ui.weight.value);
    for (let i = 0; i < nodes.length; i++) for (let j = i + 1; j < nodes.length; j++) {
      const dx = nodes[i].x - nodes[j].x, dy = nodes[i].y - nodes[j].y;
      if (dx * dx + dy * dy <= r2) connect(nodes[i], nodes[j], w, true);
    }
    recompute();
  });
  $("clearLinks").addEventListener("click", () => {
    for (const n of nodes) n.links = [];
    state.selected = null; syncLinkPanel(); recompute();
  });

  ui.lWeight.addEventListener("input", () => {
    const s = state.selected; if (!s) return;
    const w = parseFloat(ui.lWeight.value);
    s.link.weight = w;
    const t = twin(s.node, s.link); if (t) t.weight = w;
    syncLinkPanel(); recompute();
  });
  ui.lSpline.addEventListener("change", () => {
    const s = state.selected; if (!s) return;
    const on = ui.lSpline.checked, t = twin(s.node, s.link);
    s.link.spline = on;
    if (on && !s.link.t1.x && !s.link.t1.y && !s.link.t2.x && !s.link.t2.y) {
      [s.link.t1, s.link.t2] = defaultTangents(s.node, s.link.to);
    }
    if (t) { t.spline = on; t.t1 = s.link.t2; t.t2 = s.link.t1; }
    syncLinkPanel(); recompute();
  });

  for (const id of ["showLabels", "showExplored"]) ui[id].addEventListener("change", draw);
  ui.walker.addEventListener("change", restartWalker);
  ui.dijkstra.addEventListener("change", recompute);

  // ---- Presets ---------------------------------------------------------------
  function grid(cols, rows, skip) {
    nodes = []; nextId = 1;
    const sx = 2.4, sy = 2.4, ox = (WORLD_W - (cols - 1) * sx) / 2, oy = (WORLD_H - (rows - 1) * sy) / 2;
    const map = [];
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) {
      if (skip && skip(c, r)) continue;
      map.push(addNode(ox + c * sx, oy + r * sy));
    }
    for (let i = 0; i < map.length; i++) for (let j = i + 1; j < map.length; j++) {
      if (dist(map[i], map[j]) <= 2.5) connect(map[i], map[j], 1, true);
    }
    return map;
  }
  // Hand-placed layout. Edit the coordinates (world units, 16 x 10) to change it.
  // Names are only used to describe the links below.
  const NETWORK_NODES = {
    E: [3.7, 1.0], T: [10.8, 1.2], S: [8.1, 4.0], L: [1.5, 5.5],
    R: [15.3, 5.6], M: [8.2, 7.1], BL: [4.9, 9.1], BR: [11.1, 9.0],
  };
  const NETWORK_LINKS = [
    ["E", "S"], ["E", "L"], ["T", "S"], ["T", "R"], ["S", "L"], ["S", "R"], ["S", "M"],
    ["S", "BL"], ["S", "BR"], ["L", "BL"], ["M", "BL"], ["M", "BR"], ["BL", "BR"], ["BR", "R"],
  ];
  function network() {
    nodes = []; nextId = 1;
    const byName = {};
    for (const [name, [x, y]] of Object.entries(NETWORK_NODES)) byName[name] = addNode(x, y);
    for (const [a, b] of NETWORK_LINKS) connect(byName[a], byName[b], 1, true);
    return byName;
  }

  function loadPreset(name) {
    state.selected = null; state.start = state.end = null; syncLinkPanel();
    if (name === "network") {
      const n = network();
      state.start = n.BR; state.end = n.E;
    }
    else if (name === "empty") nodes = [];
    else if (name === "grid") grid(6, 4);
    else if (name === "wall") grid(6, 4, (c, r) => c === 3 && r !== 3);
    else if (name === "weighted") {
      grid(6, 4);
      // a costly "river" down the middle that the path would rather avoid or cross cheaply
      for (const n of nodes) for (const l of n.links) {
        const mid = (n.x + l.to.x) / 2;
        if (Math.abs(mid - WORLD_W / 2) < 1) l.weight = 4;
      }
    }
    recompute();
  }
  document.querySelectorAll("[data-preset]").forEach(b => b.addEventListener("click", () => loadPreset(b.dataset.preset)));

  // ---- Init ------------------------------------------------------------------
  readColors();
  window.addEventListener("resize", resize);
  if (window.matchMedia) {
    window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => { readColors(); draw(); });
  }
  setTool("path");
  resize();
  loadPreset("network"); // sets start and end and computes the first route
})();
