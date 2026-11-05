"use strict";
// Panel Follow Line: gráficas en vivo (uPlot), sliders de parámetros, mapa,
// tabla de vueltas y dispersión parámetro → métrica. Habla con tools/panel/panel.py por WebSocket.

const $ = (id) => document.getElementById(id);
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const MAX_PUNTOS = 3000;

// --- Gráficas -----------------------------------------------------------------
// [clave, etiqueta, color, ancho, discontinua]
const GRAFICAS = [
  { titulo: "Error lateral (px) · + = línea a la derecha · e = filas cercanas, e_ctl = el que usa el PID", series: [
    ["e", "e", "#2f6fde", 1.4], ["e_ctl", "e_ctl", "#d97706", 1.4], ["L", "L mirada (m)", "#16a34a", 1, true]] },
  { titulo: "Giro (rad/s) · w mandada y sus partes; wz = giro real (odometría)", series: [
    ["w", "w", "#111827", 1.8], ["p", "P", "#2f6fde", 1], ["i", "I", "#d97706", 1],
    ["d", "D", "#16a34a", 1], ["ff", "FF", "#9333ea", 1], ["wz", "wz real", "#dc2626", 1.2, true]] },
  { titulo: "Velocidad (m/s)", series: [
    ["v", "v", "#2f6fde", 1.6], ["v_ref", "v_ref", "#d97706", 1, true], ["vx", "vx real", "#dc2626", 1, true]] },
  { titulo: "Curvatura (1/m) · cámara frente a la recorrida (wz/vx)", series: [
    ["kappa", "κ", "#2f6fde", 1.4], ["kappa_lejos", "κ_lejos", "#d97706", 1.2],
    ["k_odom", "κ real", "#111827", 1.2, true]] },
  { titulo: "Percepción y tiempo · dt y t_perc en ms", series: [
    ["validos", "puntos válidos", "#2f6fde", 1.2], ["dt_ms", "dt", "#d97706", 1],
    ["t_perc", "t_perc", "#16a34a", 1]] },
];
const CLAVES = [...new Set(GRAFICAS.flatMap((g) => g.series.map((s) => s[0])))];

const buf = { t: [] };
CLAVES.forEach((k) => (buf[k] = []));
let t0 = null;
let eventos = []; // {t, texto, color}: cambios de parámetros, marcas, vueltas
let pausado = false;
let sucio = false;
let graficas = [];

function plugEventos() {
  return {
    hooks: {
      draw: (u) => {
        const ctx = u.ctx;
        const [xmin, xmax] = [u.scales.x.min, u.scales.x.max];
        ctx.save();
        ctx.font = `${11 * devicePixelRatio}px system-ui`;
        for (const ev of eventos) {
          if (ev.t < xmin || ev.t > xmax) continue;
          const x = u.valToPos(ev.t, "x", true);
          ctx.strokeStyle = ev.color;
          ctx.setLineDash([4, 3]);
          ctx.beginPath();
          ctx.moveTo(x, u.bbox.top);
          ctx.lineTo(x, u.bbox.top + u.bbox.height);
          ctx.stroke();
          ctx.fillStyle = ev.color;
          ctx.fillText(ev.texto, x + 3, u.bbox.top + 12 * devicePixelRatio);
        }
        ctx.restore();
      },
    },
  };
}

function crearGraficas() {
  const cont = $("graficas");
  cont.innerHTML = "";
  const ancho = cont.clientWidth || 900;
  const sync = uPlot.sync("panel");
  const ejeColor = css("--suave");
  const rejilla = css("--borde");
  graficas = GRAFICAS.map((g) => {
    const opts = {
      title: g.titulo, width: ancho, height: 190,
      plugins: [plugEventos()],
      cursor: { sync: { key: sync.key }, drag: { x: true, y: false } },
      scales: { x: { time: false } },
      axes: [
        { stroke: ejeColor, grid: { stroke: rejilla, width: 1 }, label: "t (s)", labelSize: 14 },
        { stroke: ejeColor, grid: { stroke: rejilla, width: 1 }, size: 55 },
      ],
      series: [{ label: "t", value: (u, v) => (v == null ? "–" : v.toFixed(2)) }].concat(
        g.series.map(([, etiqueta, color, w, disc]) => ({
          label: etiqueta, stroke: color, width: w, dash: disc ? [5, 4] : undefined,
          spanGaps: false, value: (u, v) => (v == null ? "–" : (+v).toPrecision(3)),
        }))),
    };
    const datos = [[], ...g.series.map(() => [])];
    return { u: new uPlot(opts, datos, cont), g };
  });
}

function redibujarGraficas() {
  if (!graficas.length || !buf.t.length) return;
  const ventana = +$("ventana").value;
  const tmax = buf.t[buf.t.length - 1];
  let i0 = buf.t.length - 1;
  while (i0 > 0 && buf.t[i0 - 1] >= tmax - ventana) i0--;
  const xs = buf.t.slice(i0);
  for (const { u, g } of graficas) {
    u.setData([xs, ...g.series.map(([k]) => buf[k].slice(i0))]);
  }
}

window.addEventListener("resize", () => {
  const ancho = $("graficas").clientWidth;
  graficas.forEach(({ u }) => u.setSize({ width: ancho, height: 190 }));
  dibujarMapa();
  dibujarDispersion();
});

// --- Estado de la sesión ------------------------------------------------------
let esquema = [];
let valores = {};
let vueltas = [];
let tray = []; // vuelta actual: [x, y, |e|]
let trayAnt = []; // vueltas anteriores
let vueltaActual = 1;
const llegadas = [];
const errores5s = [];

function reiniciarDatos() {
  t0 = null;
  for (const k in buf) buf[k].length = 0;
  eventos = [];
  tray = [];
  trayAnt = [];
  vueltas = [];
  vueltaActual = 1;
}

function anadirDato(d) {
  if (t0 === null) t0 = d.t;
  const t = d.t - t0;
  buf.t.push(t);
  for (const k of CLAVES) {
    let v = d[k];
    if (k === "dt_ms") v = d.dt * 1000;
    else if (k === "k_odom") v = d.vx > 1 && d.wz != null ? d.wz / d.vx : null;
    buf[k].push(v === undefined ? null : v);
  }
  if (buf.t.length > MAX_PUNTOS) {
    const sobra = buf.t.length - MAX_PUNTOS;
    for (const k in buf) buf[k].splice(0, sobra);
  }
  if (d.marca) eventos.push({ t, texto: d.marca, color: "#d97706" });
  if (d.vuelta !== vueltaActual) {
    if (tray.length) trayAnt.push(tray);
    tray = [];
    eventos.push({ t, texto: `vuelta ${d.vuelta}`, color: css("--ok") });
    vueltaActual = d.vuelta;
  }
  if (d.x != null) tray.push([d.x, d.y, d.e == null ? 999 : Math.abs(d.e)]);
  const ahora = performance.now();
  llegadas.push(ahora);
  while (llegadas.length && llegadas[0] < ahora - 1000) llegadas.shift();
  if (d.e != null) errores5s.push([ahora, Math.abs(d.e)]);
  while (errores5s.length && errores5s[0][0] < ahora - 5000) errores5s.shift();
  $("k-vuelta").textContent = d.vuelta;
  $("k-tvuelta").textContent = d.t_vuelta == null ? "–" : d.t_vuelta.toFixed(1) + " s";
  sucio = true;
}

// --- Sliders --------------------------------------------------------------------
let pendientes = {};
let temporizador = null;
const decimales = (paso) => Math.max(0, -Math.floor(Math.log10(paso) + 1e-9));

function crearSliders() {
  const cont = $("sliders");
  cont.innerHTML = "";
  let grupo = null;
  for (const p of esquema) {
    if (p.grupo !== grupo) {
      grupo = p.grupo;
      const h = document.createElement("h3");
      h.textContent = grupo;
      cont.appendChild(h);
    }
    const div = document.createElement("div");
    div.className = "param";
    div.id = "p-" + p.nombre;
    div.title = `${p.desc}\nrango ${p.min} … ${p.max}, por defecto ${p.defecto}`;
    div.innerHTML = `<label><b>${p.nombre}</b><span class="desc">${p.desc}</span></label>
      <input type="range" min="${p.min}" max="${p.max}" step="${p.paso}">
      <input type="number" min="${p.min}" max="${p.max}" step="${p.paso}">
      <button title="valor por defecto (${p.defecto})">↺</button>`;
    const [rango, num] = div.querySelectorAll("input");
    const cambiar = (v) => {
      rango.value = v;
      num.value = (+v).toFixed(decimales(p.paso));
      pendientes[p.nombre] = +v;
      div.classList.add("pendiente");
      clearTimeout(temporizador);
      temporizador = setTimeout(enviarPendientes, 120);
    };
    rango.addEventListener("input", () => cambiar(rango.value));
    num.addEventListener("change", () => cambiar(num.value));
    div.querySelector("button").addEventListener("click", () => cambiar(p.defecto));
    cont.appendChild(div);
  }
  pintarValores();
}

function enviarPendientes() {
  if (Object.keys(pendientes).length) enviar({ tipo: "set", valores: pendientes });
  pendientes = {};
}

function pintarValores() {
  for (const p of esquema) {
    const div = $("p-" + p.nombre);
    if (!div || pendientes[p.nombre] !== undefined) continue;
    const [rango, num] = div.querySelectorAll("input");
    const v = valores[p.nombre];
    if (document.activeElement !== rango) rango.value = v;
    if (document.activeElement !== num) num.value = (+v).toFixed(decimales(p.paso));
    div.classList.remove("pendiente");
    div.classList.toggle("cambiado", Math.abs(v - p.defecto) > 1e-12);
  }
}

let tAviso = null;
function aviso(texto) {
  const a = $("aviso");
  a.textContent = texto;
  a.classList.add("visible");
  clearTimeout(tAviso);
  tAviso = setTimeout(() => a.classList.remove("visible"), 4000);
}

function nuevosValores(v, pendiente) {
  const cambios = Object.keys(v).filter((k) => valores[k] !== v[k]);
  if (cambios.length && buf.t.length && !pendiente) {
    const texto = cambios.map((k) => `${k}=${+v[k].toPrecision(3)}`).join(" ");
    eventos.push({ t: buf.t[buf.t.length - 1], texto, color: css("--acento") });
  }
  valores = v;
  pintarValores();
  sucio = true;
}

// --- Mapa -------------------------------------------------------------------------
function colorError(e) {
  if (e >= 999) return "#000";
  const a = Math.min(e / 200, 1);
  return `hsl(${120 * (1 - a)}, 80%, 45%)`;
}

function prepararCanvas(cv) {
  const r = cv.getBoundingClientRect();
  cv.width = r.width * devicePixelRatio;
  cv.height = r.height * devicePixelRatio;
  const ctx = cv.getContext("2d");
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  ctx.clearRect(0, 0, r.width, r.height);
  return [ctx, r.width, r.height];
}

function dibujarMapa() {
  const [ctx, W, Hh] = prepararCanvas($("mapa"));
  const todos = trayAnt.flat().concat(tray);
  if (!todos.length) {
    ctx.fillStyle = css("--suave");
    ctx.fillText("sin odometría todavía", 10, 20);
    return;
  }
  let [x0, x1, y0, y1] = [Infinity, -Infinity, Infinity, -Infinity];
  for (const [x, y] of todos) { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); }
  const esc = Math.min((W - 20) / Math.max(x1 - x0, 1), (Hh - 20) / Math.max(y1 - y0, 1));
  const px = (x) => 10 + (x - x0) * esc + (W - 20 - (x1 - x0) * esc) / 2;
  const py = (y) => Hh - 10 - (y - y0) * esc - (Hh - 20 - (y1 - y0) * esc) / 2;
  ctx.lineWidth = 1;
  ctx.strokeStyle = css("--borde");
  for (const t of trayAnt) {
    ctx.beginPath();
    t.forEach(([x, y], i) => (i ? ctx.lineTo(px(x), py(y)) : ctx.moveTo(px(x), py(y))));
    ctx.stroke();
  }
  ctx.lineWidth = 2.5;
  for (let i = 1; i < tray.length; i++) {
    ctx.strokeStyle = colorError(tray[i][2]);
    ctx.beginPath();
    ctx.moveTo(px(tray[i - 1][0]), py(tray[i - 1][1]));
    ctx.lineTo(px(tray[i][0]), py(tray[i][1]));
    ctx.stroke();
  }
  const ini = (trayAnt[0] || tray)[0];
  ctx.fillStyle = css("--texto");
  ctx.fillRect(px(ini[0]) - 3, py(ini[1]) - 3, 6, 6);
  const fin = tray[tray.length - 1];
  if (fin) {
    ctx.beginPath();
    ctx.arc(px(fin[0]), py(fin[1]), 5, 0, 2 * Math.PI);
    ctx.fill();
  }
}

// --- Vueltas: tabla y dispersión ------------------------------------------------
function pintarOptimizador(m) {
  const caja = $("optimizador");
  caja.hidden = false;
  const mejor = m.mejor ? `${m.mejor.valor.toFixed(2)} (ensayo ${m.mejor.ensayo}, ${m.mejor.tiempo_s} s)` : "–";
  caja.innerHTML = `<h2>Optimizador <small>${m.activo ? "en marcha" : "parado"}</small></h2>
    <div class="opt"><span>estudio</span><b>${m.estudio}</b><span>ensayo</span><b>${m.ensayo}</b>
    <span>fase</span><b>${m.fase}</b><span>mejor J</span><b>${mejor}</b>
    <span>último</span><b>${m.ultimo || "–"}</b></div>`;
}

const METRICAS = ["tiempo_s", "tiempo_reloj_s", "e_abs_medio", "e_rms", "e_max", "perdidos",
  "cambios_w_s", "w_abs_medio", "v_media", "distancia_m"];

function actualizarSelects() {
  const x = $("disp-x"), y = $("disp-y");
  const vx = x.value, vy = y.value;
  const opcionesX = esquema.map((p) => p.nombre).concat(METRICAS);
  x.innerHTML = opcionesX.map((o) => `<option>${o}</option>`).join("");
  y.innerHTML = METRICAS.map((o) => `<option>${o}</option>`).join("");
  x.value = opcionesX.includes(vx) ? vx : (variaEnVueltas()[0] || "kp");
  y.value = METRICAS.includes(vy) ? vy : "tiempo_s";
}

function variaEnVueltas() {
  return esquema.map((p) => p.nombre).filter((k) => new Set(vueltas.map((v) => v[k])).size > 1);
}

function pintarTabla() {
  const varian = new Set(variaEnVueltas());
  const params = esquema.map((p) => p.nombre);
  const cols = ["vuelta", "motivo", ...METRICAS.filter((m) => m !== "distancia_m"), ...params];
  $("tabla").querySelector("thead").innerHTML =
    "<tr>" + cols.map((c) => `<th>${c}</th>`).join("") + "</tr>";
  const completas = vueltas.filter((v) => v.completada !== 0);
  const limpias = completas.filter((v) => !v.params_cambiados);
  const mejor = Math.min(...(limpias.length ? limpias : completas).map((v) => v.tiempo_s));
  $("tabla").querySelector("tbody").innerHTML = vueltas.slice().reverse().map((v) => {
    const celdas = cols.map((c) => {
      let txt = v[c];
      if (c === "vuelta" && v.params_cambiados) txt += "*";
      if (c === "motivo") txt = v.completada === 0 ? "✗ " + v.motivo : "✓";
      const cls = varian.has(c) ? ' class="dif"' : "";
      return `<td${cls}>${typeof txt === "number" ? +txt.toPrecision(4) : txt}</td>`;
    });
    return `<tr data-v="${v.vuelta}"${v.tiempo_s === mejor ? ' class="mejor"' : ""}>${celdas.join("")}</tr>`;
  }).join("");
  const ult = completas[completas.length - 1];
  $("k-ultima").textContent = ult ? ult.tiempo_s.toFixed(2) + " s" : "–";
  $("k-mejor").textContent = completas.length ? mejor.toFixed(2) + " s" : "–";
}

$("tabla").addEventListener("click", (ev) => {
  const tr = ev.target.closest("tr[data-v]");
  if (!tr) return;
  const v = vueltas.find((x) => x.vuelta === +tr.dataset.v);
  if (v && confirm(`¿Cargar los parámetros de la vuelta ${v.vuelta} (${v.tiempo_s} s)?`)) {
    enviar({ tipo: "set", valores: Object.fromEntries(esquema.map((p) => [p.nombre, v[p.nombre]])) });
  }
});

function dibujarDispersion() {
  const [ctx, W, Hh] = prepararCanvas($("dispersion"));
  const kx = $("disp-x").value, ky = $("disp-y").value;
  const pts = vueltas.map((v) => [v[kx], v[ky], v]).filter(([a, b]) => a != null && b != null);
  ctx.fillStyle = css("--suave");
  ctx.font = "12px system-ui";
  if (!pts.length) {
    ctx.fillText("Aún no hay vueltas completas. Cambia un parámetro entre vueltas y aparecerá aquí.", 10, 20);
    return;
  }
  const ext = (i) => {
    const vs = pts.map((p) => p[i]);
    let a = Math.min(...vs), b = Math.max(...vs);
    if (a === b) { a -= Math.abs(a) * 0.1 || 1; b += Math.abs(b) * 0.1 || 1; }
    const m = (b - a) * 0.08;
    return [a - m, b + m];
  };
  const [xa, xb] = ext(0), [ya, yb] = ext(1);
  const L = 55, B = 30;
  const px = (x) => L + (x - xa) / (xb - xa) * (W - L - 10);
  const py = (y) => Hh - B - (y - ya) / (yb - ya) * (Hh - B - 10);
  ctx.strokeStyle = css("--borde");
  ctx.beginPath();
  ctx.moveTo(L, 10); ctx.lineTo(L, Hh - B); ctx.lineTo(W - 10, Hh - B);
  ctx.stroke();
  for (let k = 0; k <= 4; k++) {
    const x = xa + (xb - xa) * k / 4, y = ya + (yb - ya) * k / 4;
    ctx.fillText(+x.toPrecision(3), px(x) - 12, Hh - B + 16);
    ctx.fillText(+y.toPrecision(3), 4, py(y) + 4);
  }
  ctx.fillText(`${ky} frente a ${kx}`, L + 6, 22);
  for (const [x, y, v] of pts) {
    ctx.beginPath();
    if (v.completada === 0) {  // intento fallido: aspa roja
      ctx.strokeStyle = css("--mal");
      ctx.moveTo(px(x) - 5, py(y) - 5); ctx.lineTo(px(x) + 5, py(y) + 5);
      ctx.moveTo(px(x) + 5, py(y) - 5); ctx.lineTo(px(x) - 5, py(y) + 5);
      ctx.stroke();
      ctx.fillStyle = css("--suave");
      ctx.fillText(v.vuelta, px(x) + 7, py(y) - 5);
      continue;
    }
    ctx.arc(px(x), py(y), 5, 0, 2 * Math.PI);
    ctx.strokeStyle = ctx.fillStyle = css("--acento");
    if (v.params_cambiados) ctx.stroke(); else ctx.fill();
    ctx.fillStyle = css("--suave");
    ctx.fillText(v.vuelta, px(x) + 7, py(y) - 5);
  }
}
$("disp-x").addEventListener("change", dibujarDispersion);
$("disp-y").addEventListener("change", dibujarDispersion);

// --- WebSocket ----------------------------------------------------------------------
let ws;
function enviar(m) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(m)); }

function conectar() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.tipo === "dato") anadirDato(m);
    else if (m.tipo === "params") nuevosValores(m.valores, m.pendiente);
    else if (m.tipo === "aviso") aviso(m.texto);
    else if (m.tipo === "reinicio") {
      if (buf.t.length) eventos.push({ t: buf.t[buf.t.length - 1], texto: "reinicio: " + m.motivo, color: css("--mal") });
      if (tray.length) trayAnt.push(tray);
      tray = [];
      if (m.motivo !== "pedido") aviso(`Reinicio automático (${m.motivo}): el coche vuelve a la salida`);
    } else if (m.tipo === "opt_estado") pintarOptimizador(m);
    else if (m.tipo === "vuelta") {
      vueltas.push(m.fila);
      actualizarSelects();
      pintarTabla();
      dibujarDispersion();
    } else if (m.tipo === "hola") {
      reiniciarDatos();
      esquema = m.params;
      valores = m.valores;
      if (esquema.length) crearSliders();
      (m.historial || []).forEach(anadirDato);
      vueltas = m.vueltas || [];
      actualizarSelects();
      pintarTabla();
      dibujarDispersion();
      sucio = true;
    } else if (m.tipo === "estado") {
      const e = $("estado");
      e.className = "estado " + (m.conectado ? "on" : "off");
      e.textContent = m.conectado ? "controlador conectado" : "esperando al controlador (play en la web)";
      $("fichero").textContent = m.fichero ? `→ logs/${m.fichero}.csv` : "";
      if (m.desactualizado) {
        e.className = "estado off";
        e.textContent = "panel desactualizado: reinícialo (tools/panel.sh) y recarga";
      }
      $("preset-lista").innerHTML = m.presets.map((p) => `<option>${p}</option>`).join("");
    }
  };
  ws.onclose = () => {
    const e = $("estado");
    e.className = "estado off";
    e.textContent = "panel.py no responde";
    setTimeout(conectar, 1000);
  };
}

// --- Botones --------------------------------------------------------------------------
$("b-pausa").addEventListener("click", (ev) => {
  pausado = !pausado;
  ev.target.classList.toggle("activo", pausado);
  ev.target.textContent = pausado ? "Reanudar gráficas" : "Pausar gráficas";
});
function marca() {
  const texto = prompt("Texto de la marca (se guarda en el CSV):", "marca") || "marca";
  enviar({ tipo: "marca", texto });
}
$("b-marca").addEventListener("click", marca);
document.addEventListener("keydown", (ev) => {
  if ((ev.key === "m" || ev.key === "M") && !["INPUT", "SELECT"].includes(document.activeElement.tagName)) marca();
});
$("b-reset-pid").addEventListener("click", () => enviar({ tipo: "reset_pid" }));
$("b-salida").addEventListener("click", () => enviar({ tipo: "reiniciar" }));
$("b-nueva").addEventListener("click", () => {
  if (confirm("¿Cerrar los CSV actuales y empezar una prueba nueva? (reinicia el cronómetro)")) enviar({ tipo: "nueva_prueba" });
});
$("b-guardar").addEventListener("click", () => enviar({ tipo: "preset_guardar", nombre: $("preset-nombre").value }));
$("b-cargar").addEventListener("click", () => enviar({ tipo: "preset_cargar", nombre: $("preset-lista").value }));
$("b-defecto").addEventListener("click", () => {
  if (confirm("¿Volver todos los parámetros a su valor por defecto?"))
    enviar({ tipo: "set", valores: Object.fromEntries(esquema.map((p) => [p.nombre, p.defecto])) });
});

// --- Bucle de dibujo --------------------------------------------------------------------
crearGraficas();
conectar();
setInterval(() => {
  $("k-hz").textContent = llegadas.length;
  $("k-e").textContent = errores5s.length
    ? (errores5s.reduce((s, x) => s + x[1], 0) / errores5s.length).toFixed(0) + " px" : "–";
  if (sucio && !pausado) {
    redibujarGraficas();
    sucio = false;
  }
}, 100);
setInterval(() => { if (!pausado) dibujarMapa(); }, 250);
