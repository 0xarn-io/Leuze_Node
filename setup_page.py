"""Setup & visualization page: one self-contained HTML document.

Follows the RTSP_Node pattern: ``render()`` returns static HTML with a few
``__PLACEHOLDER__`` substitutions; everything live comes from fetch
polling of ``/live`` (~8 Hz), ``/measurements`` and ``/config`` -- no
WebSockets, no build step, no external assets.
"""

from __future__ import annotations

_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>__NODE_NAME__ · __GATE_ID__</title>
<style>
  :root { --bg:#10151c; --panel:#1a2230; --line:#2c3a50; --text:#dde6f2;
          --dim:#8aa0bd; --ok:#3fd07f; --warn:#ffb347; --err:#ff5d5d;
          --accent:#4da3ff; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--text);
         font:14px/1.45 system-ui, sans-serif; }
  header { display:flex; align-items:center; gap:14px; padding:10px 16px;
           background:var(--panel); border-bottom:1px solid var(--line);
           flex-wrap:wrap; }
  header h1 { font-size:16px; margin:0; }
  .badge { padding:2px 10px; border-radius:10px; font-weight:600;
           text-transform:uppercase; font-size:11px; background:#2c3a50; }
  .badge.idle { background:#2c3a50; }
  .badge.armed { background:#6b5d1f; color:#ffe28a; }
  .badge.measuring { background:#1f5d36; color:#9af2c0; }
  .badge.capturing_baseline { background:#1f4b6b; color:#a8dcff; }
  .readout { font-size:18px; font-weight:700; font-variant-numeric:tabular-nums; }
  .readout small { color:var(--dim); font-size:11px; font-weight:400; display:block; }
  main { display:grid; grid-template-columns: minmax(420px, 1fr) 360px;
         gap:12px; padding:12px 16px; }
  .panel { background:var(--panel); border:1px solid var(--line);
           border-radius:8px; padding:12px; }
  canvas { width:100%; background:#0b0f15; border-radius:6px; display:block; }
  button { background:var(--accent); border:0; color:#08243f; font-weight:700;
           padding:8px 14px; border-radius:6px; cursor:pointer; margin:2px; }
  button.secondary { background:#2c3a50; color:var(--text); }
  button.danger { background:var(--err); color:#3d0606; }
  button:disabled { opacity:.45; cursor:default; }
  table { width:100%; border-collapse:collapse; font-size:13px; }
  th, td { text-align:left; padding:4px 6px; border-bottom:1px solid var(--line);
           font-variant-numeric:tabular-nums; }
  th { color:var(--dim); font-weight:600; }
  .laser { border:1px solid var(--line); border-radius:6px; padding:8px;
           margin-bottom:8px; }
  .dot { display:inline-block; width:9px; height:9px; border-radius:50%;
         margin-right:6px; background:var(--err); }
  .dot.on { background:var(--ok); }
  .cfg label { display:block; color:var(--dim); font-size:11px; margin-top:6px; }
  .cfg input[type=text], .cfg input[type=number] {
      width:100%; background:#0b0f15; color:var(--text);
      border:1px solid var(--line); border-radius:4px; padding:5px 7px; }
  .cfg .row { display:grid; grid-template-columns:repeat(auto-fit, minmax(90px,1fr));
              gap:6px; }
  .muted { color:var(--dim); }
  #error { color:var(--err); min-height:18px; font-size:12px; }
  #saveMsg { font-size:12px; min-height:16px; }
  @media (max-width: 900px) { main { grid-template-columns:1fr; } }
</style>
</head>
<body>
<header>
  <h1>__NODE_NAME__ <span class="muted">· __GATE_ID__ · v__VERSION__</span></h1>
  <span id="state" class="badge idle">idle</span>
  <span id="mockBadge" class="badge" style="display:none">mock</span>
  <span class="readout"><span id="w">–</span><small>width m</small></span>
  <span class="readout"><span id="h">–</span><small>height m</small></span>
  <span class="readout"><span id="l">–</span><small>last length m</small></span>
  <span class="readout"><span id="np">0</span><small>points</small></span>
  <span id="error"></span>
</header>
<main>
  <div>
    <div class="panel">
      <canvas id="view" height="520"></canvas>
    </div>
    <div class="panel" style="margin-top:12px">
      <h3 style="margin:0 0 8px">Recent measurements</h3>
      <table id="hist"><thead><tr>
        <th>time</th><th>outcome</th><th>W max</th><th>H max</th><th>L</th>
        <th>ref</th><th>src</th></tr></thead><tbody></tbody></table>
    </div>
  </div>
  <div>
    <div class="panel">
      <button id="btnTrigger">Trigger</button>
      <button id="btnAbort" class="danger">Abort</button>
      <button id="btnBaseline" class="secondary">Capture baseline</button>
      <button id="btnSpawn" class="secondary" style="display:none">Spawn box</button>
      <div id="lasers" style="margin-top:10px"></div>
    </div>
    <div class="panel cfg" style="margin-top:12px">
      <h3 style="margin:0 0 4px">Configuration</h3>
      <div id="cfgForm"></div>
      <button id="btnSave" style="margin-top:10px">Save &amp; apply</button>
      <div id="saveMsg"></div>
    </div>
  </div>
</main>
<script>
"use strict";
const MOCK = __MOCK__;
const COLORS = ["#4da3ff", "#3fd07f", "#ffb347", "#d98cff", "#ff5d5d"];
let cfg = null, live = null, lastLength = null;

const $ = id => document.getElementById(id);
const fmt = (v, d=3) => v == null ? "–" : Number(v).toFixed(d);

async function jget(url) { const r = await fetch(url); return r.json(); }
async function jpost(url, body) {
  const r = await fetch(url, {method:"POST",
    headers:{"Content-Type":"application/json"},
    body: JSON.stringify(body || {})});
  let data = null; try { data = await r.json(); } catch (e) {}
  return {ok: r.ok, status: r.status, data};
}

// ---------------------------------------------------------------- canvas
function draw() {
  const canvas = $("view"), ctx = canvas.getContext("2d");
  const cssW = canvas.clientWidth;
  if (canvas.width !== cssW) canvas.width = cssW;
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  if (!cfg) return;
  const roi = (live && live.roi) || cfg.gate.roi;
  const poses = cfg.lasers.map(l => l.pose);
  let x0 = Math.min(roi.x_min_m, ...poses.map(p => p.x_m)) - 0.3;
  let x1 = Math.max(roi.x_max_m, ...poses.map(p => p.x_m)) + 0.3;
  let z0 = Math.min(-0.15, ...poses.map(p => p.z_m)) - 0.1;
  let z1 = Math.max(roi.z_max_m, ...poses.map(p => p.z_m)) + 0.3;
  const s = Math.min(W / (x1 - x0), H / (z1 - z0));
  const px = x => (x - x0) * s + (W - (x1 - x0) * s) / 2;
  const pz = z => H - ((z - z0) * s + (H - (z1 - z0) * s) / 2);

  // grid every 0.5 m
  ctx.strokeStyle = "#16202e"; ctx.lineWidth = 1; ctx.beginPath();
  for (let gx = Math.ceil(x0 * 2) / 2; gx <= x1; gx += 0.5)
    { ctx.moveTo(px(gx), 0); ctx.lineTo(px(gx), H); }
  for (let gz = Math.ceil(z0 * 2) / 2; gz <= z1; gz += 0.5)
    { ctx.moveTo(0, pz(gz)); ctx.lineTo(W, pz(gz)); }
  ctx.stroke();

  // belt
  const beltZ = cfg.gate.belt_z_m;
  ctx.strokeStyle = "#5d6b80"; ctx.lineWidth = 2; ctx.beginPath();
  ctx.moveTo(0, pz(beltZ)); ctx.lineTo(W, pz(beltZ)); ctx.stroke();

  // ROI
  ctx.strokeStyle = "#3a5f8a"; ctx.setLineDash([5, 4]);
  ctx.strokeRect(px(roi.x_min_m), pz(roi.z_max_m),
                 (roi.x_max_m - roi.x_min_m) * s,
                 (roi.z_max_m - roi.z_min_m) * s);
  ctx.setLineDash([]);

  // lasers: dot + heading line
  cfg.lasers.forEach((l, i) => {
    const c = COLORS[i % COLORS.length];
    const x = px(l.pose.x_m), z = pz(l.pose.z_m);
    const th = l.pose.rotation_deg * Math.PI / 180;
    ctx.fillStyle = c; ctx.beginPath(); ctx.arc(x, z, 6, 0, 7); ctx.fill();
    ctx.strokeStyle = c; ctx.lineWidth = 2; ctx.beginPath();
    ctx.moveTo(x, z);
    ctx.lineTo(x + Math.cos(th) * 22, z - Math.sin(th) * 22); ctx.stroke();
    ctx.fillStyle = "#8aa0bd"; ctx.font = "11px sans-serif";
    ctx.fillText(l.id, x + 8, z - 8);
  });

  // foreground points + W/H bracket
  if (live && live.points && live.points.length) {
    let minX = 1e9, maxX = -1e9, maxZ = -1e9;
    for (const p of live.points) {
      const [li, x, z] = p;
      minX = Math.min(minX, x); maxX = Math.max(maxX, x);
      maxZ = Math.max(maxZ, z);
      ctx.fillStyle = COLORS[li % COLORS.length];
      ctx.fillRect(px(x) - 1.5, pz(z) - 1.5, 3, 3);
    }
    if (live.present) {
      ctx.strokeStyle = "#ffffff66"; ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
      ctx.strokeRect(px(minX), pz(maxZ), (maxX - minX) * s,
                     (maxZ - beltZ) * s);
      ctx.setLineDash([]);
    }
  }
}

// ---------------------------------------------------------------- polling
async function pollLive() {
  try {
    live = await jget("/live");
    $("state").textContent = live.state;
    $("state").className = "badge " + live.state;
    $("w").textContent = fmt(live.width_m);
    $("h").textContent = fmt(live.height_m);
    $("np").textContent = live.n_points;
    $("error").textContent = live.last_error || "";
    draw();
    renderLasers(live.lasers || []);
  } catch (e) { /* node restarting */ }
  setTimeout(pollLive, 125);
}

function renderLasers(lasers) {
  const host = $("lasers");
  lasers.forEach((l, i) => {
    let el = document.getElementById("laser-" + l.id);
    if (!el) {
      el = document.createElement("div");
      el.className = "laser"; el.id = "laser-" + l.id;
      host.appendChild(el);
    }
    const st = l.status || {};
    const flags = [st.error && "ERROR", st.alarm && "alarm",
                   st.screen_contaminated && "dirty screen"]
                  .filter(Boolean).join(" · ");
    el.innerHTML =
      `<span class="dot ${l.is_receiving ? "on" : ""}"></span>` +
      `<b style="color:${COLORS[i % COLORS.length]}">${l.name}</b> ` +
      `<span class="muted">:${l.port}</span> · ${fmt(l.scan_rate_hz, 1)} Hz` +
      ` · fg ${l.n_foreground}` +
      ` · baseline ${l.baseline && l.baseline.ok ? "✓" : "—"}` +
      (flags ? ` · <span style="color:var(--warn)">${flags}</span>` : "");
  });
}

async function pollSlow() {
  try {
    const hist = await jget("/measurements?limit=12");
    const tbody = $("hist").querySelector("tbody");
    tbody.innerHTML = hist.map(m => {
      if (m.outcome === "completed" && m.length_m != null) lastLength = m.length_m;
      return `<tr><td>${(m.ended_at || "").slice(11, 19)}</td>` +
        `<td>${m.outcome}</td>` +
        `<td>${m.width_m ? fmt(m.width_m.max) : "–"}</td>` +
        `<td>${m.height_m ? fmt(m.height_m.max) : "–"}</td>` +
        `<td>${fmt(m.length_m)}</td>` +
        `<td>${m.ref || ""}</td><td>${m.trigger_source || ""}</td></tr>`;
    }).join("");
    $("l").textContent = fmt(lastLength);
  } catch (e) {}
  setTimeout(pollSlow, 2000);
}

// ---------------------------------------------------------------- config UI
const GATE_FIELDS = [
  ["conveyor_speed_mps", "conveyor speed m/s"], ["belt_z_m", "belt z m"],
  ["foreground_margin_mm", "fg margin mm"],
  ["min_foreground_points", "min points"],
  ["on_debounce_samples", "on debounce"], ["off_debounce_samples", "off debounce"],
  ["arm_timeout_s", "arm timeout s"], ["max_event_s", "max event s"],
  ["outlier_trim_points", "outlier trim"],
];
const ROI_FIELDS = ["x_min_m", "x_max_m", "z_min_m", "z_max_m"];
const POSE_FIELDS = ["x_m", "z_m", "rotation_deg"];

function num(id) { return parseFloat($(id).value); }

function buildForm() {
  let html = '<div class="row">';
  for (const [key, label] of GATE_FIELDS)
    html += `<span><label>${label}</label>` +
            `<input type="number" step="any" id="g_${key}" value="${cfg.gate[key]}"></span>`;
  html += "</div><label>ROI (x min / x max / z min / z max)</label><div class='row'>";
  for (const key of ROI_FIELDS)
    html += `<input type="number" step="any" id="r_${key}" value="${cfg.gate.roi[key]}">`;
  html += "</div>";
  cfg.lasers.forEach((l, i) => {
    html += `<label style="color:${COLORS[i % COLORS.length]}">laser ${l.id} ` +
            `(port / x / z / rot° / sector min / max / mirror)</label><div class="row">` +
      `<input type="number" id="L${i}_port" value="${l.port}">`;
    for (const key of POSE_FIELDS)
      html += `<input type="number" step="any" id="L${i}_${key}" value="${l.pose[key]}">`;
    html += `<input type="number" step="any" id="L${i}_smin" value="${l.sector.min_deg}">` +
            `<input type="number" step="any" id="L${i}_smax" value="${l.sector.max_deg}">` +
            `<span style="align-self:center"><input type="checkbox" id="L${i}_mirror" ` +
            `${l.pose.mirror ? "checked" : ""}> mirror</span></div>`;
  });
  html += `<label>webhook URL (empty = disabled)</label>` +
          `<input type="text" id="wh_url" value="${cfg.webhook.url}">`;
  $("cfgForm").innerHTML = html;
}

async function saveConfig() {
  const update = {gate: {roi: {}}, webhook: {url: $("wh_url").value.trim()}, lasers: []};
  for (const [key] of GATE_FIELDS) update.gate[key] = num("g_" + key);
  for (const key of ROI_FIELDS) update.gate.roi[key] = num("r_" + key);
  cfg.lasers.forEach((l, i) => {
    update.lasers.push({id: l.id, port: parseInt($(`L${i}_port`).value),
      pose: {x_m: num(`L${i}_x_m`), z_m: num(`L${i}_z_m`),
             rotation_deg: num(`L${i}_rotation_deg`),
             mirror: $(`L${i}_mirror`).checked},
      sector: {min_deg: num(`L${i}_smin`), max_deg: num(`L${i}_smax`)}});
  });
  const res = await jpost("/config", update);
  const msg = $("saveMsg");
  if (res.ok) {
    msg.textContent = "saved ✓"; msg.style.color = "var(--ok)";
    cfg = await jget("/config"); buildForm();
  } else {
    msg.textContent = (res.data && res.data.detail) || ("error " + res.status);
    msg.style.color = "var(--err)";
  }
}

// ---------------------------------------------------------------- buttons
$("btnTrigger").onclick = async () => {
  const r = await jpost("/trigger", {source: "setup-page"});
  if (!r.ok) flashError((r.data && r.data.detail) || "trigger rejected");
};
$("btnAbort").onclick = () => jpost("/abort");
$("btnBaseline").onclick = async () => {
  const r = await jpost("/baseline/capture");
  if (!r.ok) flashError((r.data && r.data.detail) || "baseline rejected");
};
$("btnSpawn").onclick = () => jpost("/mock/spawn", {});
function flashError(text) {
  $("error").textContent = text;
  setTimeout(() => { $("error").textContent = ""; }, 3000);
}

// ---------------------------------------------------------------- boot
(async function boot() {
  if (MOCK) { $("mockBadge").style.display = ""; $("btnSpawn").style.display = ""; }
  cfg = await jget("/config");
  buildForm();
  $("btnSave").onclick = saveConfig;
  pollLive(); pollSlow();
})();
</script>
</body>
</html>
"""


def render(node_name: str, gate_id: str, version: str, mock: bool) -> str:
    return (_PAGE
            .replace("__NODE_NAME__", node_name)
            .replace("__GATE_ID__", gate_id)
            .replace("__VERSION__", version)
            .replace("__MOCK__", "true" if mock else "false"))
