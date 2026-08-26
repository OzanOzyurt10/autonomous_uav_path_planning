"use strict";

// Arayuz durumu. terrain sunucudan bir kez geliyor, waypointler kullanicidan,
// path son planlama sonucundan.
let terrain = null;
let waypoints = [];        // {x, y, agl, z}
let path = [];             // [x, y, z, yaw]
let frameSeconds = 0.5;
let playing = false;
let timer = null;

const DEFAULT_AGL = 300;   // tiklanan noktanin varsayilan yerden yuksekligi

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

// --- cizim -------------------------------------------------------------
function drawMap2d() {
  const contour = {
    type: "contour", x: axisX(), y: axisY(), z: terrain.heights,
    colorscale: "Earth", reversescale: true, ncontours: 20,
    contours: { showlines: true },
    colorbar: { title: "kot (m)", thickness: 10, len: 0.9 },
    hovertemplate: "x %{x:.0f}<br>y %{y:.0f}<br>kot %{z:.0f} m<extra></extra>"
  };
  Plotly.newPlot("map2d", [contour, waypointTrace2d(), routeTrace2d()], {
    margin: { l: 46, r: 10, t: 6, b: 38 },
    xaxis: { title: "x (dogu, m)", constrain: "domain" },
    yaxis: { title: "y (kuzey, m)", scaleanchor: "x" },
    showlegend: false
  }, { displayModeBar: false, responsive: true });

  $("map2d").on("plotly_click", (event) => {
    const point = event.points[0];
    // Konturun uzerine tiklanabiliyor; waypoint izlerine tiklama yok sayilir
    if (point.data.type !== "contour") return;
    addWaypoint(point.x, point.y);
  });
}

function waypointTrace2d() {
  return {
    type: "scatter", mode: "markers+text",
    x: waypoints.map((w) => w.x), y: waypoints.map((w) => w.y),
    text: waypoints.map((_, i) => String(i + 1)),
    textposition: "top center", textfont: { color: "#111", size: 11 },
    marker: { size: 11, color: "#1d4ed8", line: { color: "#fff", width: 1.5 } },
    hoverinfo: "skip"
  };
}

function routeTrace2d() {
  return {
    type: "scatter", mode: "lines",
    x: path.map((p) => p[0]), y: path.map((p) => p[1]),
    line: { color: "#b91c3c", width: 3 }, hoverinfo: "skip"
  };
}

function drawView3d() {
  const surface = {
    type: "surface", x: axisX(), y: axisY(), z: terrain.heights,
    colorscale: "Earth", reversescale: true, showscale: false,
    hovertemplate: "x %{x:.0f}<br>y %{y:.0f}<br>kot %{z:.0f} m<extra></extra>"
  };
  const span = terrain.extent_x;
  Plotly.newPlot("view3d", [surface, waypointTrace3d(), routeTrace3d(),
                            aircraftTrace(0)], {
    margin: { l: 0, r: 0, t: 0, b: 0 },
    scene: {
      xaxis: { title: "x (dogu, m)" },
      yaxis: { title: "y (kuzey, m)" },
      zaxis: { title: "irtifa (m)", range: [terrain.low, terrain.ceiling] },
      // Dusey abarti: 10 km'lik haritada 2 km'lik irtifa yassi kaliyor.
      aspectmode: "manual",
      aspectratio: { x: 1, y: terrain.extent_y / span, z: 0.45 },
      camera: { eye: { x: 1.5, y: -1.5, z: 0.9 } }
    },
    showlegend: false
  }, { responsive: true });
}

function waypointTrace3d() {
  return {
    type: "scatter3d", mode: "markers+text",
    x: waypoints.map((w) => w.x), y: waypoints.map((w) => w.y),
    z: waypoints.map((w) => w.z),
    text: waypoints.map((_, i) => String(i + 1)),
    marker: { size: 5, color: "#1d4ed8" }, hoverinfo: "skip"
  };
}

function routeTrace3d() {
  return {
    type: "scatter3d", mode: "lines",
    x: path.map((p) => p[0]), y: path.map((p) => p[1]),
    z: path.map((p) => p[2]),
    line: { color: "#b91c3c", width: 5 }, hoverinfo: "skip"
  };
}

// Ucak koni olarak ciziliyor: yonu u/v/w vektoru veriyor, boylece bas acisi
// ve tirmanma acisi gozle gorunuyor.
function aircraftTrace(index) {
  if (!path.length) {
    return { type: "cone", x: [], y: [], z: [], u: [], v: [], w: [] };
  }
  const p = path[Math.min(index, path.length - 1)];
  const next = path[Math.min(index + 1, path.length - 1)];
  let dx = next[0] - p[0], dy = next[1] - p[1], dz = next[2] - p[2];
  if (Math.hypot(dx, dy, dz) < 1e-6) {          // son karede ileri yon yok
    dx = Math.cos(p[3]); dy = Math.sin(p[3]); dz = 0;
  }
  return {
    type: "cone", x: [p[0]], y: [p[1]], z: [p[2]],
    u: [dx], v: [dy], w: [dz],
    sizemode: "absolute", sizeref: terrain.extent_x / 22, anchor: "center",
    colorscale: [[0, "#111827"], [1, "#111827"]], showscale: false,
    hoverinfo: "skip"
  };
}

function refreshWaypointViews() {
  Plotly.restyle("map2d", {
    x: [waypoints.map((w) => w.x)], y: [waypoints.map((w) => w.y)],
    text: [waypoints.map((_, i) => String(i + 1))]
  }, [1]);
  Plotly.restyle("view3d", {
    x: [waypoints.map((w) => w.x)], y: [waypoints.map((w) => w.y)],
    z: [waypoints.map((w) => w.z)],
    text: [waypoints.map((_, i) => String(i + 1))]
  }, [1]);
}

function refreshRouteViews() {
  Plotly.restyle("map2d", { x: [path.map((p) => p[0])],
                            y: [path.map((p) => p[1])] }, [2]);
  Plotly.restyle("view3d", { x: [path.map((p) => p[0])],
                             y: [path.map((p) => p[1])],
                             z: [path.map((p) => p[2])] }, [2]);
  Plotly.restyle("view3d", aircraftTrace(0), [3]);
}

// --- waypoint listesi --------------------------------------------------
function addWaypoint(x, y) {
  const ground = elevationAt(x, y);
  waypoints.push({ x, y, agl: DEFAULT_AGL, z: ground + DEFAULT_AGL });
  renderList();
  refreshWaypointViews();
}

function renderList() {
  const body = $("wplist");
  body.innerHTML = "";
  waypoints.forEach((w, index) => {
    const ground = elevationAt(w.x, w.y);
    const row = document.createElement("tr");
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
      renderList();
      refreshWaypointViews();
    });
    body.appendChild(row);
  });
  $("empty").style.display = waypoints.length ? "none" : "block";
  $("plan").disabled = waypoints.length < 2;
}

// --- planlama ----------------------------------------------------------
async function requestPlan() {
  const status = $("status");
  status.className = "ghost";
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

  if (!data.ok) {
    status.className = "bad";
    status.textContent = data.message || "rota bulunamadi";
    return;
  }

  path = data.path;
  frameSeconds = data.frame_seconds;
  refreshRouteViews();

  const minutes = Math.floor(data.duration / 60);
  const seconds = Math.round(data.duration % 60);
  status.className = "";
  status.innerHTML =
    `uzunluk <b>${(data.cost / 1000).toFixed(2)} km</b> &nbsp; ` +
    `ucus suresi <b>${minutes} dk ${seconds} sn</b> &nbsp; ` +
    `en dusuk AGL <b>${data.agl.min.toFixed(0)} m</b> &nbsp; ` +
    `ortalama AGL <b>${data.agl.mean.toFixed(0)} m</b> &nbsp; ` +
    `${data.legs.length} bacak`;

  $("scrub").max = String(path.length - 1);
  $("scrub").value = "0";
  $("scrub").disabled = false;
  $("play").disabled = false;
  updateClock(0);
}

// --- animasyon ---------------------------------------------------------
function showFrame(index) {
  Plotly.restyle("view3d", aircraftTrace(index), [3]);
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
  if (timer) { clearInterval(timer); timer = null; }
  $("play").textContent = "Oynat";
}

function togglePlayback() {
  if (playing) { stopPlayback(); return; }
  playing = true;
  $("play").textContent = "Duraklat";
  // Gercek zamanli: bir kare frameSeconds saniyelik ucus demek.
  timer = setInterval(() => {
    let index = Number($("scrub").value) + 1;
    if (index >= path.length) { index = 0; }
    $("scrub").value = String(index);
    showFrame(index);
  }, frameSeconds * 1000);
}

// --- baslangic ---------------------------------------------------------
async function start() {
  terrain = await (await fetch("/api/terrain")).json();
  drawMap2d();
  drawView3d();
  renderList();

  $("plan").addEventListener("click", requestPlan);
  $("play").addEventListener("click", togglePlayback);
  $("scrub").addEventListener("input", (event) => {
    stopPlayback();
    showFrame(Number(event.target.value));
  });
  $("clear").addEventListener("click", () => {
    stopPlayback();
    waypoints = [];
    path = [];
    renderList();
    refreshWaypointViews();
    refreshRouteViews();
    $("scrub").disabled = true;
    $("play").disabled = true;
    $("status").className = "ghost";
    $("status").textContent = "Rota henuz planlanmadi.";
  });
}

start();
