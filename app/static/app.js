"use strict";

// Arayuz durumu. Waypointler ENLEM/BOYLAM tutuluyor: harita kaydirilip
// yakinlastirildikca yerel metre koordinatlari degisir, cografi konum
// degismez. Metreye cevirme plan aninda, sunucunun sectigi cerceveyle
// yapiliyor.
let waypoints = [];        // {lat, lon, alt}
let legs = [];             // [{found, cost, path}] - yol YEREL metre
let path = [];             // butun bacaklarin birlesimi, yerel metre
let frame = null;          // {lat, lon, midLat, lonScale}
let terrain = null;        // izgara | null
let extent = null;         // {x, y, low, ceiling}
let hasTerrain = false;
let frameSeconds = 0.5;

let zones = [];            // {lat, lon, radius} yasak bolgeler
let mode = "2d";           // "2d" | "3d"
let placing = "waypoint";  // haritaya tiklayinca ne konacak

let view = { lon: 20, lat: 25, zoom: 2 };   // acilista tum dunya
let basemap = "sat";
let show3d = false;
let mapReady = false;

let playing = false;
let playStart = 0;
let startIndex = 0;
let lastIndex = -1;
let draggingScene = false;

const DEFAULT_ALT = 300;   // yeni waypointin varsayilan irtifasi

// Koyu yuzey uzerinde dogrulanmis palet: OKLCH bandi, kroma tabani, CVD
// ayrimi ve kontrast olculdu. Basarisiz bacak ucuncu bir seri DEGIL,
// veri yoklugu - notr ve kesikli, anlamini yazi tasiyor.
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

// Sabit iz sirasi: restyle indeksleri buna dayaniyor.
const MAP_ZONE = 0, MAP_ROUTE = 1, MAP_FAILED = 2, MAP_PLANE = 3;
const VIEW_WP = 1, VIEW_ROUTE = 2, VIEW_FAILED = 3, VIEW_PLANE = 4;

const $ = (id) => document.getElementById(id);

// --- geo: src/geo.py aynasi -------------------------------------------
// Olcek CERCEVEDEN okunuyor, noktanin kendi enleminden degil; aksi halde
// ileri-geri donusum ayni yere donmez.
const METRES_PER_DEGREE = 111320;

function makeFrame(lat, lon, midLat) {
  return { lat: lat, lon: lon, midLat: midLat,
           lonScale: METRES_PER_DEGREE * Math.cos(midLat * Math.PI / 180) };
}

function toGeo(f, x, y) {
  return [f.lat + y / METRES_PER_DEGREE, f.lon + x / f.lonScale];
}

// --- Web Mercator: piksel ile enlem/boylam arasi ----------------------
// MapLibre'de dunya genisligi 512 * 2^zoom CSS piksel. Bu matematigi
// kendimiz yapiyoruz ki Plotly'nin ic nesnelerine bagimli olmayalim.
function project(lon, lat, zoom) {
  const scale = 512 * Math.pow(2, zoom);
  const s = Math.sin(lat * Math.PI / 180);
  return [(lon + 180) / 360 * scale,
          (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * scale];
}

function unproject(x, y, zoom) {
  const scale = 512 * Math.pow(2, zoom);
  const n = Math.PI * (1 - 2 * y / scale);
  return [x / scale * 360 - 180, Math.atan(Math.sinh(n)) * 180 / Math.PI];
}

function pixelToLonLat(clientX, clientY) {
  const rect = $("map").getBoundingClientRect();
  const centre = project(view.lon, view.lat, view.zoom);
  return unproject(centre[0] + (clientX - rect.left - rect.width / 2),
                   centre[1] + (clientY - rect.top - rect.height / 2),
                   view.zoom);
}

// Etiketleri yerlestirmek icin ters yon: enlem/boylam -> kutu ici piksel.
function lonLatToPixel(lon, lat) {
  const rect = $("map").getBoundingClientRect();
  const centre = project(view.lon, view.lat, view.zoom);
  const point = project(lon, lat, view.zoom);
  return [point[0] - centre[0] + rect.width / 2,
          point[1] - centre[1] + rect.height / 2];
}

// --- taban haritalar ---------------------------------------------------
// glyphs YOK ve olmamali: yazi cizen katman font sunucusu ister. Waypoint
// numaralarini haritanin ustune HTML olarak koyuyoruz, hem font sunucusuna
// bagimli olmuyoruz hem de suruklenebiliyorlar.
const LAYERS = {
  sat: {
    ad: "Uydu",
    tiles: "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2020_3857/" +
           "default/g/{z}/{y}/{x}.jpg",
    ornek: "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2020_3857/" +
           "default/g/6/22/34.jpg",
    credit: "Sentinel-2 cloudless 2020 by EOX IT Services GmbH " +
            "(Modified Copernicus Sentinel data 2020)"
  },
  street: {
    ad: "Harita",
    tiles: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    ornek: "https://tile.openstreetmap.org/6/34/22.png",
    credit: "© OpenStreetMap katkicilari"
  },
  topo: {
    ad: "Topo",
    tiles: "https://tile.opentopomap.org/{z}/{x}/{y}.png",
    ornek: "https://tile.opentopomap.org/6/34/22.png",
    credit: "© OpenTopoMap (CC-BY-SA), © OpenStreetMap katkicilari"
  },
  offline: {
    ad: "Cevrimdisi",
    tiles: null,
    credit: "Cevrimdisi vektor kita sinirlari - Natural Earth"
  }
};

// Karo sunucusu erisilebilir mi? Tek gorselle yokluyoruz: erisilemezse
// MapLibre hata firlatiyor ve Plotly haritayi HIC cizmiyor, ekran bos
// kaliyor. Once bakip sonra cizmek o sessiz bosluktan iyi.
function probeLayer(kind) {
  const source = LAYERS[kind];
  if (!source.tiles) return Promise.resolve(true);
  return new Promise((resolve) => {
    const image = new Image();
    const timer = setTimeout(() => resolve(false), 8000);
    image.onload = () => { clearTimeout(timer); resolve(true); };
    image.onerror = () => { clearTimeout(timer); resolve(false); };
    image.src = source.ornek;
  });
}

async function styleFor(kind) {
  if (kind !== "offline") {
    return {
      version: 8,
      sources: { taban: { type: "raster", tileSize: 256,
                          tiles: [LAYERS[kind].tiles] } },
      layers: [
        { id: "arka", type: "background",
          paint: { "background-color": "#0f1622" } },
        { id: "taban", type: "raster", source: "taban" }
      ]
    };
  }
  let data = { type: "FeatureCollection", features: [] };
  try {
    data = await (await fetch("world.geojson")).json();
  } catch (error) {
    /* dosya yoksa bos deniz kalir; harita yine acilir */
  }
  return {
    version: 8,
    sources: { kara: { type: "geojson", data: data } },
    layers: [
      { id: "deniz", type: "background",
        paint: { "background-color": "#0f1622" } },
      { id: "kara", type: "fill", source: "kara",
        paint: { "fill-color": "#1e2633", "fill-outline-color": "#38415a" } }
    ]
  };
}

// --- yasak bolgeler ----------------------------------------------------
// Daire enlem/boylamda ELIPS: bir boylam derecesi enlemle daraliyor.
// Iki yaricap ayri hesaplaniyor, yoksa kuzeyde bolge yassilasir.
function zoneRings() {
  const lon = [], lat = [];
  zones.forEach((zone) => {
    const dLat = zone.radius / METRES_PER_DEGREE;
    const dLon = zone.radius /
                 (METRES_PER_DEGREE * Math.cos(zone.lat * Math.PI / 180));
    for (let k = 0; k <= 48; k += 1) {
      const t = k / 48 * 2 * Math.PI;
      lat.push(zone.lat + dLat * Math.cos(t));
      lon.push(zone.lon + dLon * Math.sin(t));
    }
    lon.push(null);
    lat.push(null);
  });
  return { lon: lon, lat: lat };
}

function renderZones() {
  const body = $("zonelist");
  body.innerHTML = "";
  zones.forEach((zone, index) => {
    const row = document.createElement("tr");
    row.innerHTML =
      "<td>" + (index + 1) + "</td>" +
      "<td>" + zone.lat.toFixed(5) + "</td>" +
      "<td>" + zone.lon.toFixed(5) + "</td>" +
      '<td><input type="number" step="100" min="50" value="' +
      zone.radius.toFixed(0) + '"></td>' +
      '<td><button title="sil">&times;</button></td>';
    row.querySelector("input").addEventListener("change", (event) => {
      zone.radius = Math.max(50, Number(event.target.value));
      invalidatePlan();
      renderZones();
    });
    row.querySelector("button").addEventListener("click", () => {
      zones.splice(index, 1);
      invalidatePlan();
      renderZones();
    });
    body.appendChild(row);
  });
  $("noZone").style.display = zones.length ? "none" : "block";
  if (mapReady) {
    const rings = zoneRings();
    Plotly.restyle("map", { lon: [rings.lon], lat: [rings.lat] }, [MAP_ZONE]);
  }
}

// --- harita cizimi -----------------------------------------------------
function emptyMapTrace(style) {
  return Object.assign({ type: "scattermap", lon: [], lat: [],
                         hoverinfo: "skip" }, style);
}

async function drawMap() {
  const style = await styleFor(basemap);
  $("credit").textContent = LAYERS[basemap].credit;

  const rings = zoneRings();
  const zoneTrace = emptyMapTrace({
    mode: "lines", fill: "toself", fillcolor: "rgba(239, 77, 94, 0.20)",
    line: { color: COLOR.route, width: 1.5 },
    lon: rings.lon, lat: rings.lat });
  const route = emptyMapTrace({
    mode: "lines", line: { color: COLOR.route, width: 3 } });
  const failed = emptyMapTrace({
    mode: "lines", line: { color: COLOR.missing, width: 2 } });
  const plane = emptyMapTrace({
    mode: "lines", fill: "toself", fillcolor: COLOR.plane,
    line: { color: COLOR.surface, width: 1 } });

  await Plotly.newPlot("map", [zoneTrace, route, failed, plane], {
    map: { style: style, center: { lon: view.lon, lat: view.lat },
           zoom: view.zoom },
    margin: { l: 0, r: 0, t: 0, b: 0 },
    paper_bgcolor: "rgba(0,0,0,0)", showlegend: false
  }, { responsive: true, displaylogo: false, scrollZoom: true,
       modeBarButtonsToRemove: ["select2d", "lasso2d", "toImage"] });

  mapReady = true;
  const readView = (event) => {
    if (event["map.center"]) {
      view.lon = event["map.center"].lon;
      view.lat = event["map.center"].lat;
    }
    if (event["map.zoom"] !== undefined) view.zoom = event["map.zoom"];
    updateLabels();
  };
  $("map").on("plotly_relayout", readView);
  $("map").on("plotly_relayouting", readView);

  refreshMapRoute();
  updateLabels();
}

async function setBasemap(kind) {
  const ok = await probeLayer(kind);
  if (!ok) {
    setStatus(LAYERS[kind].ad + " karolari yuklenemedi; cevrimdisi " +
              "haritaya dusuldu. Internet baglantini kontrol et.", "bad");
    kind = "offline";
  }
  basemap = kind;
  ["sat", "street", "topo", "offline"].forEach((name) => {
    const id = "layer" + name.charAt(0).toUpperCase() + name.slice(1);
    $(id).classList.toggle("on", name === basemap);
  });
  await drawMap();
}

// --- waypoint etiketleri (haritanin ustunde HTML) ----------------------
function updateLabels() {
  const box = $("labels");
  while (box.children.length > waypoints.length) box.lastChild.remove();
  while (box.children.length < waypoints.length) {
    const label = document.createElement("div");
    label.className = "wplabel";
    attachDrag(label);
    box.appendChild(label);
  }
  const rect = $("map").getBoundingClientRect();
  waypoints.forEach((w, index) => {
    const label = box.children[index];
    const point = lonLatToPixel(w.lon, w.lat);
    label.textContent = String(index + 1);
    label.dataset.index = String(index);
    label.classList.toggle("bad", inFailedLeg(index));
    // Gorunur alanin disina cikanlari gizle; yoksa kutunun kenarinda
    // yigilip yanlis yer gosteriyorlar.
    const outside = point[0] < -40 || point[1] < -40 ||
                    point[0] > rect.width + 40 || point[1] > rect.height + 40;
    label.style.display = outside ? "none" : "block";
    label.style.left = point[0] + "px";
    label.style.top = point[1] + "px";
  });
}

// Etiketi surukleyerek waypoint tasima. Haritanin kendi suruklemesini
// kesmek icin mousedown'da durduruluyor.
function attachDrag(label) {
  let dragging = false;
  label.addEventListener("mousedown", (event) => {
    event.stopPropagation();
    event.preventDefault();
    dragging = true;
    label.classList.add("dragging");
  });
  window.addEventListener("mousemove", (event) => {
    if (!dragging) return;
    const index = Number(label.dataset.index);
    const point = pixelToLonLat(event.clientX, event.clientY);
    waypoints[index].lon = point[0];
    waypoints[index].lat = point[1];
    updateLabels();
  });
  window.addEventListener("mouseup", () => {
    if (!dragging) return;
    dragging = false;
    label.classList.remove("dragging");
    invalidatePlan();
    renderList();
  });
}

// --- rota izleri -------------------------------------------------------
function refreshMapRoute() {
  if (!mapReady) return;
  const route = { lon: [], lat: [] };
  const failed = { lon: [], lat: [] };

  legs.forEach((leg, index) => {
    if (leg.found && frame) {
      leg.path.forEach((p) => {
        const g = toGeo(frame, p[0], p[1]);
        route.lat.push(g[0]);
        route.lon.push(g[1]);
      });
      route.lat.push(null);
      route.lon.push(null);
      return;
    }
    const from = waypoints[index], to = waypoints[index + 1];
    if (!from || !to) return;
    failed.lat.push(from.lat, to.lat, null);
    failed.lon.push(from.lon, to.lon, null);
  });

  const rings = zoneRings();
  Plotly.restyle("map", { lon: [rings.lon], lat: [rings.lat] }, [MAP_ZONE]);
  Plotly.restyle("map", { lon: [route.lon], lat: [route.lat] }, [MAP_ROUTE]);
  Plotly.restyle("map", { lon: [failed.lon], lat: [failed.lat] },
                 [MAP_FAILED]);
  showPlane(0);
}

// --- arazi yardimcilari -----------------------------------------------
// Python tarafindaki Terrain.elevation_at ile ayni cift dogrusal
// enterpolasyon. Arazi yoksa zemin bilinmiyor: sifir donuyor ve irtifa
// MSL olarak okunuyor.
function elevationAt(x, y) {
  if (!terrain) return 0;
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

// --- 3B gorunum --------------------------------------------------------
function groundTrace() {
  if (terrain) {
    return {
      type: "surface",
      x: terrain.heights[0].map((_, c) => c * terrain.spacing_x),
      y: terrain.heights.map((_, r) => r * terrain.spacing_y),
      z: terrain.heights,
      colorscale: "Earth", reversescale: true, showscale: false,
      hovertemplate: "x %{x:.0f}<br>y %{y:.0f}<br>kot %{z:.0f} m<extra></extra>"
    };
  }
  // Arazi yok: duz zemin. Yuzeyi hic cizmemek 3B'yi yonsuz birakiyor.
  return {
    type: "surface", x: [0, extent.x], y: [0, extent.y],
    z: [[extent.low, extent.low], [extent.low, extent.low]],
    colorscale: [[0, "#20262f"], [1, "#20262f"]], showscale: false,
    hoverinfo: "skip", opacity: 0.85
  };
}

function drawView3d() {
  if (!extent) return;
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

  Plotly.newPlot("view3d",
                 [groundTrace(), marks, route, failed, aircraftTrace(0)], {
    margin: { l: 0, r: 0, t: 0, b: 0 },
    paper_bgcolor: "rgba(0,0,0,0)",
    font: { color: COLOR.muted, size: 11 },
    scene: {
      bgcolor: "rgba(0,0,0,0)",
      xaxis: { title: "x (dogu, m)", gridcolor: COLOR.grid,
               backgroundcolor: "rgba(0,0,0,0)" },
      yaxis: { title: "y (kuzey, m)", gridcolor: COLOR.grid,
               backgroundcolor: "rgba(0,0,0,0)" },
      zaxis: { title: "irtifa (m)", range: zRange(),
               gridcolor: COLOR.grid, backgroundcolor: "rgba(0,0,0,0)" },
      // Dusey abarti: 20 km'lik haritada 2 km'lik irtifa yassi kaliyor.
      aspectmode: "manual",
      aspectratio: { x: 1, y: extent.y / extent.x, z: 0.42 },
      camera: { eye: { x: 1.5, y: -1.5, z: 0.9 } }
    },
    showlegend: false
  }, { responsive: true });

  const panel = $("view3d");
  panel.on("plotly_relayouting", () => { draggingScene = true; });
  panel.on("plotly_relayout", () => {
    draggingScene = false;
    showPlane(Number($("scrub").value));
  });
  refresh3dTraces();
}

function refresh3dTraces() {
  if (!show3d || !extent) return;
  const wpLocal = (plannedWaypoints || []);
  Plotly.restyle("view3d", {
    x: [wpLocal.map((w) => w[0])], y: [wpLocal.map((w) => w[1])],
    z: [wpLocal.map((w) => w[2])],
    text: [wpLocal.map((_, i) => String(i + 1))]
  }, [VIEW_WP]);

  const route = [[], [], []];
  legs.forEach((leg) => {
    if (!leg.found) return;
    leg.path.forEach((p) => {
      route[0].push(p[0]); route[1].push(p[1]); route[2].push(p[2]);
    });
    route[0].push(null); route[1].push(null); route[2].push(null);
  });
  Plotly.restyle("view3d", { x: [route[0]], y: [route[1]], z: [route[2]] },
                 [VIEW_ROUTE]);
}

let plannedWaypoints = null;   // plan anindaki yerel koordinatlar

// --- ucak figuru -------------------------------------------------------
// Govde yerel koordinatta tanimli: a ileri, b saga, c yukari. Her karede
// rotanin gidis yonunden bir ortonormal cerceve kurulup buraya uygulaniyor.
const PLANE_BODY = [
  [1.00, 0.000, 0.000],    // 0  burun
  [0.30, 0.000, 0.080],    // 1  halka ust
  [0.30, -0.080, 0.000],   // 2  halka sag
  [0.30, 0.000, -0.060],   // 3  halka alt
  [0.30, 0.080, 0.000],    // 4  halka sol
  [-0.95, 0.000, 0.030],   // 5  kuyruk ucu
  [0.30, -0.060, 0.000],   // 6  sag kanat kok on
  [-0.02, -0.620, 0.000],  // 7
  [-0.20, -0.620, 0.000],  // 8
  [-0.16, -0.060, 0.000],  // 9
  [0.30, 0.060, 0.000],    // 10 sol kanat
  [-0.02, 0.620, 0.000],   // 11
  [-0.20, 0.620, 0.000],   // 12
  [-0.16, 0.060, 0.000],   // 13
  [-0.68, -0.040, 0.030],  // 14 sag dengeleyici
  [-0.82, -0.260, 0.030],  // 15
  [-0.92, -0.260, 0.030],  // 16
  [-0.90, -0.040, 0.030],  // 17
  [-0.68, 0.040, 0.030],   // 18 sol dengeleyici
  [-0.82, 0.260, 0.030],   // 19
  [-0.92, 0.260, 0.030],   // 20
  [-0.90, 0.040, 0.030],   // 21
  [-0.66, 0.000, 0.050],   // 22 dikey dumen
  [-0.84, 0.000, 0.340],   // 23
  [-0.93, 0.000, 0.340],   // 24
  [-0.93, 0.000, 0.050]    // 25
];
const PLANE_FACES = {
  i: [0, 0, 0, 0,  5, 5, 5, 5,  6, 6,  10, 10,  14, 14,  18, 18,  22, 22],
  j: [1, 2, 3, 4,  2, 3, 4, 1,  7, 8,  12, 13,  15, 16,  20, 21,  23, 24],
  k: [2, 3, 4, 1,  1, 2, 3, 4,  8, 9,  11, 12,  16, 17,  19, 20,  24, 25]
};
const PLANE_OUTLINE = [0, 6, 7, 8, 9, 14, 15, 16, 5, 20, 19, 18, 13, 12, 11, 10];
const PLANE_SCALE = 55;

// 2B'de seyir irtifasi harita tavaninin uzerinde olabiliyor; eksen
// araligi rotayi kapsamazsa ucak sahnenin disinda kalir.
function zRange() {
  let low = extent.low, high = extent.ceiling;
  path.forEach((p) => {
    if (p[2] < low) low = p[2];
    if (p[2] > high) high = p[2];
  });
  const pad = Math.max(150, (high - low) * 0.08);
  return [low, high + pad];
}

function verticalExaggeration() {
  const span = zRange();
  return 0.42 / ((span[1] - span[0]) / extent.x);
}

function planeFrame(index) {
  const p = path[Math.min(index, path.length - 1)];
  const next = path[Math.min(index + 1, path.length - 1)];
  let fx = next[0] - p[0], fy = next[1] - p[1], fz = next[2] - p[2];
  const norm = Math.hypot(fx, fy, fz);
  if (norm < 1e-6) {
    fx = Math.cos(p[3]); fy = Math.sin(p[3]); fz = 0;
  } else {
    fx /= norm; fy /= norm; fz /= norm;
  }
  let rx = fy, ry = -fx, rz = 0;
  const rNorm = Math.hypot(rx, ry, rz);
  if (rNorm < 1e-6) { rx = 0; ry = 1; rz = 0; } else { rx /= rNorm; ry /= rNorm; }
  return { p: p, f: [fx, fy, fz], r: [rx, ry, rz],
           u: [ry * fz - rz * fy, rz * fx - rx * fz, rx * fy - ry * fx] };
}

function planeVertices(index) {
  if (!path.length || !extent) return { x: [], y: [], z: [] };
  const basis = planeFrame(index);
  const size = extent.x / PLANE_SCALE;
  const squash = verticalExaggeration();
  const x = [], y = [], z = [];
  PLANE_BODY.forEach((v) => {
    x.push(basis.p[0] + size * (v[0] * basis.f[0] + v[1] * basis.r[0] +
                                v[2] * basis.u[0]));
    y.push(basis.p[1] + size * (v[0] * basis.f[1] + v[1] * basis.r[1] +
                                v[2] * basis.u[1]));
    z.push(basis.p[2] + size * (v[0] * basis.f[2] + v[1] * basis.r[2] +
                                v[2] * basis.u[2]) / squash);
  });
  return { x: x, y: y, z: z };
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

// restyle dizileri IZ BASINA deger sayiyor: dizi degerli bir ozelligi
// guncellemek icin bir kat daha sarmak gerekiyor - {x: [[1,2]]}.
function showPlane(index) {
  const v = planeVertices(index);
  if (mapReady) {
    const lon = [], lat = [];
    if (v.x.length && frame) {
      PLANE_OUTLINE.concat([PLANE_OUTLINE[0]]).forEach((n) => {
        const g = toGeo(frame, v.x[n], v.y[n]);
        lat.push(g[0]);
        lon.push(g[1]);
      });
    }
    Plotly.restyle("map", { lon: [lon], lat: [lat] }, [MAP_PLANE]);
  }
  if (show3d && !draggingScene && extent) {
    Plotly.restyle("view3d", { x: [v.x], y: [v.y], z: [v.z] }, [VIEW_PLANE]);
  }
}

// --- mod ------------------------------------------------------------
function setPlacing(kind) {
  placing = kind;
  $("map").classList.toggle("placing", kind === "zone");
  $("addZone").classList.toggle("on", kind === "zone");
  $("addZone").textContent = kind === "zone" ? "Haritaya tikla"
                                             : "Bolge ekle";
}

function setMode(next) {
  mode = next;
  $("mode2d").classList.toggle("on", next === "2d");
  $("mode3d").classList.toggle("on", next === "3d");
  $("cruiseField").style.display = next === "2d" ? "flex" : "none";
  applyAltitudeLabels();
  renderList();
  invalidatePlan();
}

// --- waypoint listesi --------------------------------------------------
function addWaypoint(lat, lon) {
  waypoints.push({ lat: lat, lon: lon, alt: DEFAULT_ALT });
  invalidatePlan();
  renderList();
  updateLabels();
}

function invalidatePlan() {
  legs = [];
  path = [];
  plannedWaypoints = null;
  stopPlayback();
  $("scrub").disabled = true;
  $("play").disabled = true;
  clearStats();
  refreshMapRoute();
}

function inFailedLeg(index) {
  return legs.some((leg, legIndex) =>
    !leg.found && (legIndex === index || legIndex + 1 === index));
}

function renderList() {
  const body = $("wplist");
  body.innerHTML = "";
  waypoints.forEach((w, index) => {
    const row = document.createElement("tr");
    if (inFailedLeg(index)) row.className = "leg-bad";
    row.innerHTML =
      "<td>" + (index + 1) + "</td>" +
      "<td>" + w.lat.toFixed(5) + "</td>" +
      "<td>" + w.lon.toFixed(5) + "</td>" +
      (mode === "2d"
        ? '<td class="ghost">' + Number($("cruise").value).toFixed(0) + "</td>"
        : '<td><input type="number" step="50" value="' + w.alt.toFixed(0) +
          '"></td>') +
      '<td><button title="sil">&times;</button></td>';
    const altInput = row.querySelector('input[type="number"]');
    if (altInput) {
      altInput.addEventListener("change", (event) => {
        w.alt = Number(event.target.value);
        invalidatePlan();
      });
    }
    row.querySelector("button").addEventListener("click", () => {
      waypoints.splice(index, 1);
      invalidatePlan();
      renderList();
      updateLabels();
    });
    body.appendChild(row);
  });
  $("empty").style.display = waypoints.length ? "none" : "block";
  $("plan").disabled = waypoints.length < 2;
}

// --- durum kutulari ----------------------------------------------------
const DASH = "–";

function setTile(id, value, unit) {
  $(id).innerHTML = unit ? value + '<span class="u">' + unit + "</span>"
                         : value;
}

function clearStats() {
  setTile("statLength", DASH, "km");
  setTile("statTime", DASH, "");
  setTile("statAgl", DASH, "m");
  setTile("statLegs", DASH, "");
  $("tileAgl").className = "tile";
}

function setStatus(text, kind) {
  $("status").className = kind === "bad" ? "bad" : "";
  $("status").textContent = text;
}

function aglState(minAgl, clearance) {
  if (minAgl < clearance) return "bad";
  if (minAgl < clearance * 1.5) return "warn";
  return "good";
}

// --- planlama ----------------------------------------------------------
async function requestPlan() {
  setStatus("planlaniyor...");
  $("plan").disabled = true;
  stopPlayback();

  const body = {
    waypoints: waypoints.map((w) => [w.lat, w.lon, w.alt]),
    speed: Number($("speed").value),
    bank_deg: Number($("bank").value),
    climb_deg: Number($("climb").value),
    clearance: Number($("clearance").value),
    iterations: Number($("iterations").value),
    seed: Number($("seed").value),
    mode: mode,
    cruise: Number($("cruise").value),
    zones: zones.map((z) => ({ lat: z.lat, lon: z.lon, radius_m: z.radius })),
    want_terrain: true
  };

  let data;
  try {
    const response = await fetch("/api/plan", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body)
    });
    data = await response.json();
  } catch (error) {
    setStatus("sunucuya ulasilamadi: " + error, "bad");
    $("plan").disabled = false;
    return;
  }

  $("plan").disabled = false;
  if (data.rho) {
    $("rho").textContent = "donus yaricapi " + data.rho.toFixed(0) + " m";
  }

  if (data.frame) {
    frame = makeFrame(data.frame.lat, data.frame.lon, data.frame.mid_lat);
    extent = { x: data.extent_x, y: data.extent_y,
               low: data.low, ceiling: data.ceiling };
    terrain = data.terrain || null;
    hasTerrain = Boolean(data.has_terrain);
    plannedWaypoints = data.waypoints || null;
    applyAltitudeLabels();
  }

  legs = data.legs || [];
  path = data.path || [];
  renderList();
  updateLabels();
  refreshMapRoute();
  if (show3d) drawView3d();

  if (!data.ok) {
    clearStats();
    setStatus(data.message || "rota bulunamadi", "bad");
    $("scrub").disabled = true;
    $("play").disabled = true;
    return;
  }

  frameSeconds = data.frame_seconds;
  fillStats(data);
  $("scrub").max = String(path.length - 1);
  $("scrub").value = "0";
  $("scrub").disabled = false;
  $("play").disabled = false;
  updateClock(0);
}

function fillStats(data) {
  const clearance = Number($("clearance").value);
  setTile("statLength", (data.cost / 1000).toFixed(2), "km");
  const minutes = Math.floor(data.duration / 60);
  const seconds = Math.round(data.duration % 60);
  setTile("statTime", minutes + '<span class="u">dk</span> ' +
                      String(seconds).padStart(2, "0"), "sn");
  setTile("statLegs", String(legs.length), "");

  if (!data.agl) {
    setTile("statAgl", DASH, "");
    $("tileAgl").className = "tile";
    setStatus("Bu bolgede arazi verisi yok" +
              (data.missing && data.missing.length
                ? " (eksik karo: " + data.missing.join(", ") + ")" : "") +
              ". Irtifa MSL olarak alindi ve yerden yukseklik " +
              "DOGRULANMADI - 2B planlama gecerli, engelden kacinma degil.",
              "bad");
    return;
  }
  setTile("statAgl", data.agl.min.toFixed(0), "m");
  const state = aglState(data.agl.min, clearance);
  $("tileAgl").className = "tile " + state;

  // Negatif AGL "paya yakin" degil, rotanin arazinin ICINDEN gectigi
  // anlamina geliyor. 2B planlamada bu beklenen bir sonuc: yatay rota
  // araziyi gormuyor. Ayni renkle gecistirilmemeli.
  if (data.agl.min < 0) {
    setStatus("ROTA ARAZIYE GIRIYOR: en dusuk AGL " +
              data.agl.min.toFixed(0) + " m. 2B planlama araziden " +
              "kacinmaz - ya seyir irtifasini en az " +
              (Number($("cruise").value) - data.agl.min + clearance)
                .toFixed(0) + " m yap, ya 3B moda gec.", "bad");
    return;
  }
  const verdict = state === "good" ? "emniyet payinin rahat ustunde"
                : state === "warn" ? "emniyet payina yakin"
                : "emniyet payinin ALTINDA";
  setStatus("ortalama AGL " + data.agl.mean.toFixed(0) + " m, en dusuk " +
            data.agl.min.toFixed(0) + " m - " + clearance + " m " + verdict +
            ".", state === "bad" ? "bad" : "");
}

// Etiketin dogru olmasi emniyet meselesi. 2B'de irtifa tek seyir degeri
// ve MSL; 3B'de waypoint basina ve arazi zemininden (AGL).
function applyAltitudeLabels() {
  if (mode === "2d") {
    $("altHead").textContent = "seyir MSL";
    $("altUnit").textContent = "yatay rota, tek seyir irtifasi";
  } else {
    $("altHead").textContent = "AGL m";
    $("altUnit").textContent = "waypoint basina, yerden yukseklik";
  }
  $("aglKey").textContent = hasTerrain ? "en dusuk AGL" : "AGL yok";
}

// --- animasyon ---------------------------------------------------------
function updateClock(index) {
  if (!path.length) { $("clock").textContent = "--"; return; }
  const p = path[index];
  const elapsed = index * frameSeconds;
  const agl = p[2] - elevationAt(p[0], p[1]);
  $("clock").textContent =
    Math.floor(elapsed / 60) + ":" +
    String(Math.round(elapsed % 60)).padStart(2, "0") + " | " +
    p[2].toFixed(0) + " m" + (hasTerrain ? " | AGL " + agl.toFixed(0) + " m"
                                         : " MSL");
}

function showFrame(index) {
  showPlane(index);
  updateClock(index);
}

function stopPlayback() {
  playing = false;
  $("play").textContent = "Oynat";
}

// setInterval yerine requestAnimationFrame: kare indeksi gecen GERCEK
// sureden hesaplaniyor, boylece bir cizim gecikirse animasyon atlayarak
// yetisiyor.
function tick(now) {
  if (!playing) return;
  const rate = Number($("rate").value);
  const elapsed = (now - playStart) / 1000 * rate;
  let index = startIndex + Math.round(elapsed / frameSeconds);
  if (index >= path.length) {
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

// --- 3B panel ----------------------------------------------------------
function toggle3d() {
  show3d = !show3d;
  $("panel3d").hidden = !show3d;
  $("below").classList.toggle("split", show3d);
  $("toggle3d").textContent = show3d ? "3B gizle" : "3B goster";
  $("toggle3d").classList.toggle("on", show3d);
  if (!show3d) return;
  if (!extent) {
    $("hint3d").textContent =
      "Once rotayi planla; 3B gorunum planlanan alani gosteriyor.";
    return;
  }
  $("hint3d").textContent = hasTerrain ? ""
    : "Bu bolgede arazi verisi yok; zemin duz cizildi ve irtifa MSL.";
  drawView3d();
}

// --- baslangic ---------------------------------------------------------
async function start() {
  renderList();
  renderZones();
  clearStats();
  setPlacing("waypoint");
  setMode("2d");
  await setBasemap("sat");

  // Haritanin bosluguna tiklama: Plotly map alt grafiginde iz uzerinde
  // olmayan tiklamalar plotly_click uretmiyor, o yuzden DOM olayini
  // kendi Mercator matematigimizle cozuyoruz.
  //
  // Surukleme ayirt edilmek zorunda: tarayici haritayi kaydirdiktan sonra
  // da click uretiyor, yani her pan bir waypoint birakirdi.
  let pressAt = null;
  $("map").addEventListener("mousedown", (event) => {
    pressAt = [event.clientX, event.clientY];
  });
  $("map").addEventListener("click", (event) => {
    if (!pressAt) return;
    const moved = Math.hypot(event.clientX - pressAt[0],
                             event.clientY - pressAt[1]);
    pressAt = null;
    if (moved > 4) return;                 // kaydirma, tiklama degil
    const point = pixelToLonLat(event.clientX, event.clientY);
    if (placing === "zone") {
      zones.push({ lat: point[1], lon: point[0], radius: 1500 });
      setPlacing("waypoint");
      invalidatePlan();
      renderZones();
      return;
    }
    addWaypoint(point[1], point[0]);
  });

  window.addEventListener("resize", updateLabels);

  $("layerSat").addEventListener("click", () => setBasemap("sat"));
  $("layerStreet").addEventListener("click", () => setBasemap("street"));
  $("layerTopo").addEventListener("click", () => setBasemap("topo"));
  $("layerOffline").addEventListener("click", () => setBasemap("offline"));
  $("toggle3d").addEventListener("click", toggle3d);
  $("mode2d").addEventListener("click", () => setMode("2d"));
  $("mode3d").addEventListener("click", () => setMode("3d"));
  $("addZone").addEventListener("click", () => {
    setPlacing(placing === "zone" ? "waypoint" : "zone");
  });
  $("cruise").addEventListener("change", () => {
    invalidatePlan();
    renderList();
  });

  $("plan").addEventListener("click", requestPlan);
  $("play").addEventListener("click", togglePlayback);
  $("scrub").addEventListener("input", (event) => {
    const index = Number(event.target.value);
    playStart = performance.now();
    startIndex = index;
    lastIndex = index;
    showFrame(index);
  });
  $("clear").addEventListener("click", () => {
    waypoints = [];
    zones = [];
    setPlacing("waypoint");
    renderZones();
    invalidatePlan();
    renderList();
    updateLabels();
    setStatus("Haritaya tiklayarak en az iki waypoint koy.");
  });
}

start();
