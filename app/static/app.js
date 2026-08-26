"use strict";

// Arayuz durumu. terrain sunucudan bir kez geliyor, waypointler kullanicidan,
// legs son planlama sonucundan.
let terrain = null;
let waypoints = [];        // {x, y, agl, z}
let legs = [];             // [{found, cost, path}]
let path = [];             // animasyon icin butun bacaklarin birlesimi
let frameSeconds = 0.5;
let playing = false;
let playStart = 0;         // performance.now(), oynatma basladigi an
let startIndex = 0;        // o andaki kare indeksi
let lastIndex = -1;        // ayni kareyi iki kez cizmemek icin
let draggingScene = false; // 3B kamera suruklenirken ucagi guncelleme

const DEFAULT_AGL = 300;   // tiklanan noktanin varsayilan yerden yuksekligi

// Koyu yuzey uzerinde dogrulanmis palet: OKLCH bandi, kroma tabani, CVD
// ayrimi ve kontrast olculdu. Rota ve waypoint kategorik iki yuva.
// Basarisiz bacak ucuncu bir seri DEGIL, veri yoklugu - notr ve kesikli,
// anlamini yazi tasiyor.
const COLOR = {
  route: "#ef4d5e",
  waypoint: "#9a6bff",
  missing: "#5b6478",
  plane: "#e6e9ef",
  ink: "#e6e9ef",
  muted: "#8b93a7",
  grid: "#2a3040",
  surface: "#12151b"
};

// Sabit iz sirasi: restyle indeksleri buna dayaniyor, bacak sayisi
// degistikce iz eklemiyoruz - bacaklar null ayraciyla tek ize giriyor.
const MAP_ROUTE = 1, MAP_FAILED = 2, MAP_PLANE = 3;
const VIEW_WP = 1, VIEW_ROUTE = 2, VIEW_FAILED = 3, VIEW_PLANE = 4;

const $ = (id) => document.getElementById(id);

// --- arazi yardimcilari -----------------------------------------------
// Python tarafindaki Terrain.elevation_at ile ayni cift dogrusal
// enterpolasyon; arayuzun AGL gostermesi ve varsayilan irtifa secmesi icin.
function elevationAt(x, y) {
  const fx = Math.min(Math.max(x, 0), terrain.extent_x) / terrain.spacing_x;
  const fy = Math.min(Math.max(y, 0), terrain.extent_y) / terrain.spacing_y;
  const col = Math.min(Math.floor(fx), terrain.cols - 2);
  const row = Math.min(Math.floor(fy), terrain.rows - 2);
  const tx = fx - col, ty = fy - row;
  const h = terrain.heights;
  const south = h[row][col] + (h[row][col + 1] - h[row][col]) * tx;
  const north = h[row + 1][col] + (h[row + 1][col + 1] - h[row + 1][col]) * tx;
  return south + (north - south) * ty;
}

const axisX = () => terrain.heights[0].map((_, c) => c * terrain.spacing_x);
const axisY = () => terrain.heights.map((_, r) => r * terrain.spacing_y);

// --- rota izleri -------------------------------------------------------
// Basarili bacaklar tek ize, aralarina null konarak giriyor. Basarisiz
// bacaklar iki waypointi birlestiren kesikli duz cizgi olarak ayri izde.
function routeCoords(dimension) {
  const out = [];
  legs.forEach((leg) => {
    if (!leg.found) return;
    leg.path.forEach((p) => out.push(p[dimension]));
    out.push(null);
  });
  return out;
}

function failedCoords(dimension) {
  const out = [];
  legs.forEach((leg, index) => {
    if (leg.found) return;
    const from = waypoints[index], to = waypoints[index + 1];
    if (!from || !to) return;
    const pick = (w) => (dimension === 0 ? w.x : dimension === 1 ? w.y : w.z);
    out.push(pick(from), pick(to), null);
  });
  return out;
}

// --- cizim -------------------------------------------------------------
function waypointAnnotations() {
  return waypoints.map((w, index) => ({
    x: w.x, y: w.y, text: String(index + 1),
    showarrow: false, captureevents: true,
    font: { size: 11, color: "#fff" },
    bgcolor: COLOR.waypoint, bordercolor: COLOR.surface, borderwidth: 1.5,
    borderpad: 3, opacity: 0.95
  }));
}

function drawMap2d() {
  const contour = {
    type: "contour", x: axisX(), y: axisY(), z: terrain.heights,
    colorscale: "Earth", reversescale: true, ncontours: 22,
    colorbar: { title: { text: "kot (m)", font: { color: COLOR.muted } },
                thickness: 10, len: 0.9, outlinewidth: 0,
                tickfont: { color: COLOR.muted, size: 10 } },
    hovertemplate: "x %{x:.0f}<br>y %{y:.0f}<br>kot %{z:.0f} m<extra></extra>"
  };
  const route = {
    type: "scatter", mode: "lines", x: [], y: [],
    line: { color: COLOR.route, width: 3 }, hoverinfo: "skip"
  };
  const failed = {
    type: "scatter", mode: "lines", x: [], y: [],
    line: { color: COLOR.missing, width: 2, dash: "dot" }, hoverinfo: "skip"
  };

  Plotly.newPlot("map2d", [contour, route, failed,
                            aircraftTrace2d(0)], {
    margin: { l: 54, r: 8, t: 6, b: 42 },
    paper_bgcolor: "rgba(0,0,0,0)", plot_bgcolor: "rgba(0,0,0,0)",
    font: { color: COLOR.muted, size: 11 },
    xaxis: { title: "x (dogu, m)", constrain: "domain",
             gridcolor: COLOR.grid, zeroline: false },
    yaxis: { title: "y (kuzey, m)", scaleanchor: "x",
             gridcolor: COLOR.grid, zeroline: false },
    annotations: waypointAnnotations(),
    showlegend: false
  }, {
    displayModeBar: false, responsive: true,
    // Waypointler surukleneble olsun diye; baslik/eksen duzenlemeye kapali.
    edits: { annotationPosition: true }
  });

  $("map2d").on("plotly_click", (event) => {
    const point = event.points[0];
    if (!point || point.data.type !== "contour") return;
    addWaypoint(point.x, point.y);
  });

  // Surukleme bitince Plotly annotations[i].x / .y anahtarlariyla haber
  // veriyor; ayni olay yakinlastirma icin de tetikleniyor, o yuzden
  // yalnizca bu anahtarlara bakiliyor.
  $("map2d").on("plotly_relayout", (event) => {
    let moved = false;
    Object.keys(event).forEach((key) => {
      const match = key.match(/^annotations\[(\d+)\]\.(x|y)$/);
      if (!match) return;
      const waypoint = waypoints[Number(match[1])];
      if (!waypoint) return;
      waypoint[match[2]] = event[key];
      moved = true;
    });
    if (!moved) return;
    waypoints.forEach((w) => { w.z = elevationAt(w.x, w.y) + w.agl; });
    renderList();
    refreshWaypointViews();
  });
}

function drawView3d() {
  const surface = {
    type: "surface", x: axisX(), y: axisY(), z: terrain.heights,
    colorscale: "Earth", reversescale: true, showscale: false,
    hovertemplate: "x %{x:.0f}<br>y %{y:.0f}<br>kot %{z:.0f} m<extra></extra>"
  };
  const marks = {
    type: "scatter3d", mode: "markers+text", x: [], y: [], z: [],
    marker: { size: 5, color: COLOR.waypoint },
    textfont: { color: COLOR.ink, size: 11 },
    textposition: "top center", hoverinfo: "skip"
  };
  const route = {
    type: "scatter3d", mode: "lines", x: [], y: [], z: [],
    line: { color: COLOR.route, width: 5 }, hoverinfo: "skip"
  };
  const failed = {
    type: "scatter3d", mode: "lines", x: [], y: [], z: [],
    line: { color: COLOR.missing, width: 3, dash: "dot" }, hoverinfo: "skip"
  };

  Plotly.newPlot("view3d", [surface, marks, route, failed, aircraftTrace(0)], {
    margin: { l: 0, r: 0, t: 0, b: 0 },
    paper_bgcolor: "rgba(0,0,0,0)",
    font: { color: COLOR.muted, size: 11 },
    scene: {
      bgcolor: "rgba(0,0,0,0)",
      xaxis: { title: "x (dogu, m)", gridcolor: COLOR.grid,
               backgroundcolor: "rgba(0,0,0,0)" },
      yaxis: { title: "y (kuzey, m)", gridcolor: COLOR.grid,
               backgroundcolor: "rgba(0,0,0,0)" },
      zaxis: { title: "irtifa (m)", range: [terrain.low, terrain.ceiling],
               gridcolor: COLOR.grid, backgroundcolor: "rgba(0,0,0,0)" },
      // Dusey abarti: 20 km'lik haritada 2 km'lik irtifa yassi kaliyor.
      aspectmode: "manual",
      aspectratio: { x: 1, y: terrain.extent_y / terrain.extent_x, z: 0.42 },
      camera: { eye: { x: 1.5, y: -1.5, z: 0.9 } }
    },
    showlegend: false
  }, { responsive: true });

  // Kamera surukleniyorken ucagi guncellemeyi biraktigimizi bilmemiz lazim;
  // plotly_relayouting surukleme boyunca, plotly_relayout birakinca geliyor.
  const view = $("view3d");
  view.on("plotly_relayouting", () => { draggingScene = true; });
  view.on("plotly_relayout", () => {
    draggingScene = false;
    showPlane(Number($("scrub").value));      // birakilinca yerine otursun
  });
}

// --- ucak figuru -------------------------------------------------------
// Govde yerel koordinatta tanimli: a ileri, b saga, c yukari. Her karede
// rotanin gidis yonunden bir ortonormal cerceve kurulup buraya uygulaniyor.
//
//        nose
//         /\
//   sol  /  \  sag       kanatlar tek bir delta; arkada dikey dumen
//   ----+----+----
//        \  /
//        tail
const PLANE_BODY = [
  // govde: burun, dortgen kesit halkasi, kuyruk konisi (0-5)
  [1.00, 0.000, 0.000],    // 0  burun
  [0.30, 0.000, 0.080],    // 1  halka ust
  [0.30, -0.080, 0.000],   // 2  halka sag
  [0.30, 0.000, -0.060],   // 3  halka alt
  [0.30, 0.080, 0.000],    // 4  halka sol
  [-0.95, 0.000, 0.030],   // 5  kuyruk ucu
  // sag kanat, ok acili (6-9)
  [0.30, -0.060, 0.000],   // 6  kok on kenar
  [-0.02, -0.620, 0.000],  // 7  uc on kenar
  [-0.20, -0.620, 0.000],  // 8  uc firar kenari
  [-0.16, -0.060, 0.000],  // 9  kok firar kenari
  // sol kanat (10-13)
  [0.30, 0.060, 0.000],    // 10
  [-0.02, 0.620, 0.000],   // 11
  [-0.20, 0.620, 0.000],   // 12
  [-0.16, 0.060, 0.000],   // 13
  // sag yatay dengeleyici (14-17)
  [-0.68, -0.040, 0.030],  // 14
  [-0.82, -0.260, 0.030],  // 15
  [-0.92, -0.260, 0.030],  // 16
  [-0.90, -0.040, 0.030],  // 17
  // sol yatay dengeleyici (18-21)
  [-0.68, 0.040, 0.030],   // 18
  [-0.82, 0.260, 0.030],   // 19
  [-0.92, 0.260, 0.030],   // 20
  [-0.90, 0.040, 0.030],   // 21
  // dikey dumen (22-25)
  [-0.66, 0.000, 0.050],   // 22
  [-0.84, 0.000, 0.340],   // 23
  [-0.93, 0.000, 0.340],   // 24
  [-0.93, 0.000, 0.050]    // 25
];
const PLANE_FACES = {
  i: [0, 0, 0, 0,  5, 5, 5, 5,  6, 6,  10, 10,  14, 14,  18, 18,  22, 22],
  j: [1, 2, 3, 4,  2, 3, 4, 1,  7, 8,  12, 13,  15, 16,  20, 21,  23, 24],
  k: [2, 3, 4, 1,  1, 2, 3, 4,  8, 9,  11, 12,  16, 17,  19, 20,  24, 25]
};
// 2B haritadaki dolu siluet: burundan sag kanat ve dengeleyici uzerinden
// kuyruga, oradan sol taraftan geri.
const PLANE_OUTLINE = [0, 6, 7, 8, 9, 14, 15, 16, 5, 20, 19, 18, 13, 12, 11, 10];
const PLANE_SCALE = 55;                  // govde boyu = harita genisligi / bu

// 3B sahnede irtifa ekseni abartili ciziliyor; govdenin dikey parcalarini
// ayni oranda kucultmezsek dumen absurt uzun gorunuyor.
function verticalExaggeration() {
  const trueRatio = (terrain.ceiling - terrain.low) / terrain.extent_x;
  return 0.42 / trueRatio;
}

// Gidis yonunden sag-el uclusu: f ileri, r saga, u yukari.
function planeFrame(index) {
  const p = path[Math.min(index, path.length - 1)];
  const next = path[Math.min(index + 1, path.length - 1)];
  let fx = next[0] - p[0], fy = next[1] - p[1], fz = next[2] - p[2];
  const norm = Math.hypot(fx, fy, fz);
  if (norm < 1e-6) {                     // son karede ileri yon yok
    fx = Math.cos(p[3]); fy = Math.sin(p[3]); fz = 0;
  } else {
    fx /= norm; fy /= norm; fz /= norm;
  }
  // r = f x z_ekseni, normallenmis; f dikeye yakinsa yedek eksen
  let rx = fy, ry = -fx, rz = 0;
  const rNorm = Math.hypot(rx, ry, rz);
  if (rNorm < 1e-6) { rx = 0; ry = 1; rz = 0; } else { rx /= rNorm; ry /= rNorm; }
  const ux = ry * fz - rz * fy;
  const uy = rz * fx - rx * fz;
  const uz = rx * fy - ry * fx;
  return { p, f: [fx, fy, fz], r: [rx, ry, rz], u: [ux, uy, uz] };
}

function planeVertices(index) {
  if (!path.length) return { x: [], y: [], z: [] };
  const { p, f, r, u } = planeFrame(index);
  const size = terrain.extent_x / PLANE_SCALE;
  const squash = verticalExaggeration();
  const x = [], y = [], z = [];
  PLANE_BODY.forEach(([a, b, c]) => {
    x.push(p[0] + size * (a * f[0] + b * r[0] + c * u[0]));
    y.push(p[1] + size * (a * f[1] + b * r[1] + c * u[1]));
    z.push(p[2] + size * (a * f[2] + b * r[2] + c * u[2]) / squash);
  });
  return { x, y, z };
}

function aircraftTrace(index) {
  const v = planeVertices(index);
  return {
    type: "mesh3d", x: v.x, y: v.y, z: v.z,
    i: PLANE_FACES.i, j: PLANE_FACES.j, k: PLANE_FACES.k,
    color: COLOR.plane, flatshading: true, hoverinfo: "skip",
    lighting: { ambient: 0.62, diffuse: 0.85, specular: 0.12 }
  };
}

function aircraftTrace2d(index) {
  const v = planeVertices(index);
  const pick = (list) => PLANE_OUTLINE.map((n) => list[n]).concat(
    v.x.length ? [list[PLANE_OUTLINE[0]]] : []);
  return {
    type: "scatter", mode: "lines", fill: "toself",
    x: v.x.length ? pick(v.x) : [], y: v.x.length ? pick(v.y) : [],
    fillcolor: COLOR.plane, line: { color: COLOR.surface, width: 1 },
    hoverinfo: "skip"
  };
}

// restyle dizileri IZ BASINA deger sayiyor: {x: [1,2]} iki ize birer sayi
// atar. Dizi degerli bir ozelligi guncellemek icin bir kat daha sarmak
// gerekiyor - {x: [[1,2]]} tek ize [1,2] dizisini verir.
function showPlane(index) {
  const v = planeVertices(index);
  // Kamera surukleniyorsa 3B sahneye dokunmuyoruz: her restyle sahneyi
  // yeniden cizip suruklemeyi kesiyor. 2B harita ve saat calismaya devam
  // ediyor, birakinca ucak yerine oturuyor.
  if (!draggingScene) {
    Plotly.restyle("view3d", { x: [v.x], y: [v.y], z: [v.z] }, [VIEW_PLANE]);
  }
  const outline2d = aircraftTrace2d(index);
  Plotly.restyle("map2d", { x: [outline2d.x], y: [outline2d.y] },
                 [MAP_PLANE]);
}

function refreshWaypointViews() {
  Plotly.relayout("map2d", { annotations: waypointAnnotations() });
  Plotly.restyle("view3d", {
    x: [waypoints.map((w) => w.x)], y: [waypoints.map((w) => w.y)],
    z: [waypoints.map((w) => w.z)],
    text: [waypoints.map((_, i) => String(i + 1))]
  }, [VIEW_WP]);
}

function refreshRouteViews() {
  Plotly.restyle("map2d", { x: [routeCoords(0)], y: [routeCoords(1)] },
                 [MAP_ROUTE]);
  Plotly.restyle("map2d", { x: [failedCoords(0)], y: [failedCoords(1)] },
                 [MAP_FAILED]);
  Plotly.restyle("view3d", { x: [routeCoords(0)], y: [routeCoords(1)],
                             z: [routeCoords(2)] }, [VIEW_ROUTE]);
  Plotly.restyle("view3d", { x: [failedCoords(0)], y: [failedCoords(1)],
                             z: [failedCoords(2)] }, [VIEW_FAILED]);
  showPlane(0);
}

// --- waypoint listesi --------------------------------------------------
function addWaypoint(x, y) {
  waypoints.push({ x, y, agl: DEFAULT_AGL, z: elevationAt(x, y) + DEFAULT_AGL });
  legs = [];                 // eski rota artik gecersiz
  path = [];
  renderList();
  refreshWaypointViews();
  refreshRouteViews();
}

// Bir waypoint hangi bacaklarin ucu? Basarisiz bacaklari listede
// isaretlemek icin.
function inFailedLeg(index) {
  return legs.some((leg, legIndex) =>
    !leg.found && (legIndex === index || legIndex + 1 === index));
}

function renderList() {
  const body = $("wplist");
  body.innerHTML = "";
  waypoints.forEach((w, index) => {
    const ground = elevationAt(w.x, w.y);
    const row = document.createElement("tr");
    if (inFailedLeg(index)) row.className = "leg-bad";
    row.innerHTML =
      `<td>${index + 1}</td><td>${w.x.toFixed(0)}</td>` +
      `<td>${w.y.toFixed(0)}</td>` +
      `<td><input type="number" step="10" value="${w.agl.toFixed(0)}"></td>` +
      `<td>${w.z.toFixed(0)}<span class="ghost"> / zemin ` +
      `${ground.toFixed(0)}</span></td>` +
      `<td><button title="sil">&times;</button></td>`;
    row.querySelector("input").addEventListener("change", (event) => {
      w.agl = Number(event.target.value);
      w.z = elevationAt(w.x, w.y) + w.agl;
      renderList();
      refreshWaypointViews();
    });
    row.querySelector("button").addEventListener("click", () => {
      waypoints.splice(index, 1);
      legs = [];
      path = [];
      renderList();
      refreshWaypointViews();
      refreshRouteViews();
    });
    body.appendChild(row);
  });
  $("empty").style.display = waypoints.length ? "none" : "block";
  $("plan").disabled = waypoints.length < 2;
}

// --- durum kutulari ----------------------------------------------------
const DASH = "\u2013";

function setTile(id, value, unit) {
  $(id).innerHTML = unit ? `${value}<span class="u">${unit}</span>` : value;
}

function clearStats() {
  setTile("statLength", DASH, "km");
  setTile("statTime", DASH, "");
  setTile("statAgl", DASH, "m");
  setTile("statLegs", DASH, "");
  $("tileAgl").className = "tile";
}

// AGL kutusu durum rengi tasiyor; renk tek basina anlam tasimasin diye
// esik degeri de yazi olarak durum satirinda veriliyor.
function aglState(minAgl, clearance) {
  if (minAgl < clearance) return "bad";
  if (minAgl < clearance * 1.5) return "warn";
  return "good";
}

function fillStats(data) {
  const clearance = Number($("clearance").value);
  setTile("statLength", (data.cost / 1000).toFixed(2), "km");
  const minutes = Math.floor(data.duration / 60);
  const seconds = Math.round(data.duration % 60);
  setTile("statTime", `${minutes}<span class="u">dk</span> ` +
                      `${String(seconds).padStart(2, "0")}`, "sn");
  setTile("statAgl", data.agl.min.toFixed(0), "m");
  setTile("statLegs", String(legs.length), "");
  $("tileAgl").className = "tile " + aglState(data.agl.min, clearance);
}

// --- planlama ----------------------------------------------------------
async function requestPlan() {
  const status = $("status");
  status.className = "";
  status.textContent = "planlaniyor...";
  $("plan").disabled = true;
  stopPlayback();

  const body = {
    waypoints: waypoints.map((w) => [w.x, w.y, w.z]),
    speed: Number($("speed").value),
    bank_deg: Number($("bank").value),
    climb_deg: Number($("climb").value),
    clearance: Number($("clearance").value),
    iterations: Number($("iterations").value),
    seed: Number($("seed").value)
  };

  let data;
  try {
    const response = await fetch("/api/plan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body)
    });
    data = await response.json();
  } catch (error) {
    status.className = "bad";
    status.textContent = "sunucuya ulasilamadi: " + error;
    $("plan").disabled = false;
    return;
  }

  $("plan").disabled = false;
  if (data.rho) $("rho").textContent = `donus yaricapi ${data.rho.toFixed(0)} m`;

  legs = data.legs || [];
  path = data.path || [];
  renderList();
  refreshRouteViews();

  if (!data.ok) {
    clearStats();
    status.className = "bad";
    status.textContent = data.message || "rota bulunamadi";
    $("scrub").disabled = true;
    $("play").disabled = true;
    return;
  }

  frameSeconds = data.frame_seconds;
  fillStats(data);
  const clearance = Number($("clearance").value);
  const state = aglState(data.agl.min, clearance);
  const verdict = state === "good" ? "emniyet payinin rahat ustunde"
                : state === "warn" ? "emniyet payina yakin"
                : "emniyet payinin ALTINDA";
  status.className = state === "bad" ? "bad" : "";
  status.textContent =
    `ortalama AGL ${data.agl.mean.toFixed(0)} m, en dusuk ` +
    `${data.agl.min.toFixed(0)} m - ${clearance} m ${verdict}.`;

  $("scrub").max = String(path.length - 1);
  $("scrub").value = "0";
  $("scrub").disabled = false;
  $("play").disabled = false;
  updateClock(0);
}

// --- animasyon ---------------------------------------------------------
function showFrame(index) {
  showPlane(index);
  updateClock(index);
}

function updateClock(index) {
  if (!path.length) { $("clock").textContent = "--"; return; }
  const p = path[index];
  const elapsed = index * frameSeconds;
  const agl = p[2] - elevationAt(p[0], p[1]);
  $("clock").textContent =
    `${Math.floor(elapsed / 60)}:${String(Math.round(elapsed % 60))
      .padStart(2, "0")} | ${p[2].toFixed(0)} m | AGL ${agl.toFixed(0)} m`;
}

function stopPlayback() {
  playing = false;
  $("play").textContent = "Oynat";
}

// setInterval yerine requestAnimationFrame: kare indeksi gecen GERCEK
// sureden hesaplaniyor, boylece bir cizim gecikirse animasyon geri
// kalmiyor, atlayarak yetisiyor.
function tick(now) {
  if (!playing) return;
  const rate = Number($("rate").value);
  const elapsed = (now - playStart) / 1000 * rate;
  let index = startIndex + Math.round(elapsed / frameSeconds);
  if (index >= path.length) {                 // basa sar
    index = 0;
    playStart = now;
    startIndex = 0;
  }
  if (index !== lastIndex) {
    lastIndex = index;
    $("scrub").value = String(index);
    showFrame(index);
  }
  requestAnimationFrame(tick);
}

function togglePlayback() {
  if (playing) { stopPlayback(); return; }
  if (!path.length) return;
  playing = true;
  $("play").textContent = "Duraklat";
  playStart = performance.now();
  startIndex = Number($("scrub").value);
  lastIndex = -1;
  requestAnimationFrame(tick);
}

// --- baslangic ---------------------------------------------------------
async function start() {
  terrain = await (await fetch("/api/terrain")).json();
  $("extent").textContent =
    `${(terrain.extent_x / 1000).toFixed(1)} x ` +
    `${(terrain.extent_y / 1000).toFixed(1)} km, kot ` +
    `${terrain.low.toFixed(0)}-${terrain.high.toFixed(0)} m`;
  drawMap2d();
  drawView3d();
  renderList();
  clearStats();

  $("plan").addEventListener("click", requestPlan);
  $("play").addEventListener("click", togglePlayback);
  $("scrub").addEventListener("input", (event) => {
    const index = Number(event.target.value);
    playStart = performance.now();     // oynatiyorsa buradan devam etsin
    startIndex = index;
    lastIndex = index;
    showFrame(index);
  });
  $("clear").addEventListener("click", () => {
    stopPlayback();
    waypoints = [];
    legs = [];
    path = [];
    renderList();
    refreshWaypointViews();
    refreshRouteViews();
    $("scrub").disabled = true;
    $("play").disabled = true;
    clearStats();
    $("rho").textContent = "";
    $("status").className = "";
    $("status").textContent = "Rota henuz planlanmadi.";
  });
}

start();
