const state = {
  connected: false,
  status: "disconnected",
  port: "",
  baud: 115200,
  mode: "--",
  main_state: "--",
  challenge: "--",
  challenge_phase: "--",
  challenge_action: "--",
  challenge_event: "--",
  lap_index: 0,
  lap_total: 0,
  checkpoint_count: 0,
  stop: 0,
  tick: 0,
  irq_per_s: 0,
  distance_m: 0,
  target_distance_m: 0,
  target_left_speed: 0,
  target_right_speed: 0,
  measured_left_speed: 0,
  measured_right_speed: 0,
  applied_left_duty: 0,
  applied_right_duty: 0,
  count_left: 0,
  count_right: 0,
  twist_v: 0,
  twist_w: 0,
  line: "-------",
  line_bits: 0,
  line_detected: 0,
  line_position: 0,
  imu_ready: 0,
  imu_stable: 0,
  yaw: 0,
  hold_heading: 0,
  heading_error: 0,
  yaw_dmp: 0,
  yaw_rel: 0,
  gz: 0,
  gx: 0,
  gy: 0,
  ax: 0,
  ay: 0,
  az: 0,
  pitch: 0,
  roll: 0,
  imu_up_ms: 0,
  raw_line: "",
};

const hist = {
  t: [],
  left: [],
  right: [],
  leftTarget: [],
  rightTarget: [],
  yaw: [],
  gz: [],
};

const maxPoints = 500;
const $ = (id) => document.getElementById(id);

function num(value, fallback = 0) {
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : fallback;
}

function fmt(value, digits = 2) {
  return num(value).toFixed(digits);
}

function pushHistory() {
  const now = Date.now() / 1000;
  hist.t.push(now);
  hist.left.push(num(state.measured_left_speed));
  hist.right.push(num(state.measured_right_speed));
  hist.leftTarget.push(num(state.target_left_speed));
  hist.rightTarget.push(num(state.target_right_speed));
  hist.yaw.push(num(state.yaw));
  hist.gz.push(num(state.gz));
  for (const key of Object.keys(hist)) {
    if (hist[key].length > maxPoints) hist[key].shift();
  }
}

function setText(id, text) {
  const el = $(id);
  if (el) el.textContent = text;
}

async function api(path, body) {
  const res = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  const data = await res.json();
  if (!data.ok) throw new Error(data.error || "request failed");
  return data;
}

async function sendCommand(command) {
  if (!command) return;
  appendLog(`> ${command}`);
  try {
    await api("/api/command", { command });
  } catch (err) {
    appendLog(`! ${err.message}`);
  }
}

async function refreshPorts() {
  const res = await fetch("/api/ports");
  const data = await res.json();
  const select = $("portSelect");
  const current = select.value || "/dev/ttyACM1";
  select.innerHTML = "";
  const ports = data.ports && data.ports.length ? data.ports : [current];
  for (const port of ports) {
    const opt = document.createElement("option");
    opt.value = port;
    opt.textContent = port;
    select.appendChild(opt);
  }
  select.value = ports.includes(current) ? current : ports[0];
}

function appendLog(line) {
  const box = $("logBox");
  box.textContent += `${line}\n`;
  const lines = box.textContent.split("\n");
  if (lines.length > 900) box.textContent = lines.slice(-900).join("\n");
  box.scrollTop = box.scrollHeight;
}

function updateState(next) {
  Object.assign(state, next || {});
  pushHistory();
  render();
}

function render() {
  const route = state.lap_total > 0
    ? `${state.challenge} lap=${state.lap_index}/${state.lap_total} cp=${state.checkpoint_count}`
    : state.challenge;
  setText("routeTitle", `${route || "--"}  ${state.challenge_phase || "--"}`);
  setText("modeValue", state.mode || "--");
  setText("modeSub", `${route || "--"} state=${state.main_state || "--"} stop=${state.stop || 0}`);
  setText("distanceValue", `${fmt(state.distance_m)} m`);
  setText("distanceSub", state.target_distance_m > 0 ? `target ${fmt(state.target_distance_m)} m` : "target --");
  setText("headingValue", `${fmt(state.heading_error, 1)} deg`);
  setText("headingSub", `hold ${fmt(state.hold_heading, 1)} deg`);
  setText("imuValue", `yaw ${fmt(state.yaw, 1)} deg`);
  setText("imuSub", `${state.imu_ready ? "ready" : "not ready"}, ${state.imu_stable ? "stable" : "warming"}, gz=${fmt(state.gz, 2)}`);
  setText("speedValue", `${fmt(state.measured_left_speed, 3)} / ${fmt(state.measured_right_speed, 3)}`);
  setText("speedSub", `target ${fmt(state.target_left_speed, 3)} / ${fmt(state.target_right_speed, 3)} m/s`);
  setText("dutyValue", `${fmt(state.applied_left_duty, 3)} / ${fmt(state.applied_right_duty, 3)}`);
  setText("dutySub", `count ${state.count_left || 0} / ${state.count_right || 0}`);
  setText("lineValue", state.line || "-------");
  setText("lineSub", `bits=0x${Number(state.line_bits || 0).toString(16).toUpperCase().padStart(2, "0")} det=${state.line_detected || 0} pos=${state.line_position || 0}`);
  setText("tickValue", `${state.tick || 0}`);
  setText("tickSub", `irq/s=${state.irq_per_s || 0} up=${state.imu_up_ms || 0}ms`);
  const link = $("linkStatus");
  link.textContent = `${state.status || "disconnected"} ${state.port || ""}`;
  link.classList.toggle("online", !!state.connected);
  const badge = $("stableBadge");
  badge.textContent = state.imu_stable ? "stable" : "warming";
  badge.classList.toggle("ok", !!state.imu_stable);
  setText("detailText", [
    `mode=${state.mode} tick=${state.tick} irq/s=${state.irq_per_s}`,
    `challenge=${state.challenge} phase=${state.challenge_phase} action=${state.challenge_action} lap=${state.lap_index}/${state.lap_total} cp=${state.checkpoint_count} event=${state.challenge_event}`,
    `distance=${fmt(state.distance_m, 3)}/${fmt(state.target_distance_m, 3)} hold=${fmt(state.hold_heading, 1)} err=${fmt(state.heading_error, 1)}`,
    `speed target=(${fmt(state.target_left_speed, 3)},${fmt(state.target_right_speed, 3)}) meas=(${fmt(state.measured_left_speed, 3)},${fmt(state.measured_right_speed, 3)}) duty=(${fmt(state.applied_left_duty, 3)},${fmt(state.applied_right_duty, 3)})`,
    `imu ready=${state.imu_ready} stable=${state.imu_stable} yaw=${fmt(state.yaw, 2)} gz=${fmt(state.gz, 2)} gx=${fmt(state.gx, 2)} gy=${fmt(state.gy, 2)} pitch=${fmt(state.pitch, 1)} roll=${fmt(state.roll, 1)}`,
    `line=${state.line} bits=0x${Number(state.line_bits || 0).toString(16).toUpperCase().padStart(2, "0")} det=${state.line_detected} pos=${state.line_position}`,
    `last: ${state.raw_line || ""}`,
  ].join("\n"));
  renderLineSensors();
  drawCompass();
  drawChart($("speedChart"), [
    { name: "L meas", data: hist.left, color: "#38bdf8" },
    { name: "R meas", data: hist.right, color: "#f97316" },
    { name: "L target", data: hist.leftTarget, color: "#22c55e" },
    { name: "R target", data: hist.rightTarget, color: "#eab308" },
  ]);
  drawChart($("imuChart"), [
    { name: "yaw", data: hist.yaw, color: "#a78bfa" },
    { name: "gz", data: hist.gz, color: "#f43f5e" },
  ]);
}

function renderLineSensors() {
  const wrap = $("lineSensors");
  const text = state.line || "-------";
  wrap.innerHTML = "";
  for (let i = 0; i < 7; i += 1) {
    const div = document.createElement("div");
    div.className = `sensor ${text[i] === "1" ? "active" : ""}`;
    div.textContent = String(i);
    wrap.appendChild(div);
  }
}

function drawCompass() {
  const canvas = $("compassCanvas");
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const r = Math.min(w, h) * 0.38;
  ctx.clearRect(0, 0, w, h);
  ctx.strokeStyle = "#2a3342";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.stroke();
  for (let i = 0; i < 12; i += 1) {
    const a = (i / 12) * Math.PI * 2;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(a) * (r - 8), cy + Math.sin(a) * (r - 8));
    ctx.lineTo(cx + Math.cos(a) * r, cy + Math.sin(a) * r);
    ctx.stroke();
  }
  const angle = ((state.yaw || 0) - 90) * Math.PI / 180;
  ctx.strokeStyle = "#22c55e";
  ctx.lineWidth = 5;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + Math.cos(angle) * (r - 14), cy + Math.sin(angle) * (r - 14));
  ctx.stroke();
  ctx.fillStyle = "#e6edf6";
  ctx.font = "600 24px system-ui";
  ctx.textAlign = "center";
  ctx.fillText(`${fmt(state.yaw, 1)} deg`, cx, cy + r + 30);
}

function drawChart(canvas, series) {
  const ctx = canvas.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(320, Math.floor(rect.width * dpr));
  const height = Math.max(160, Math.floor((canvas.getAttribute("height") || 210) * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  ctx.clearRect(0, 0, width, height);
  const pad = 32 * dpr;
  const values = series.flatMap((s) => s.data).filter(Number.isFinite);
  const min = values.length ? Math.min(...values, -0.05) : -1;
  const max = values.length ? Math.max(...values, 0.05) : 1;
  const span = Math.max(0.1, max - min);
  ctx.strokeStyle = "#2a3342";
  ctx.lineWidth = 1 * dpr;
  for (let i = 0; i < 4; i += 1) {
    const y = pad + (i / 3) * (height - pad * 1.6);
    ctx.beginPath();
    ctx.moveTo(pad, y);
    ctx.lineTo(width - pad / 2, y);
    ctx.stroke();
  }
  for (const item of series) {
    const data = item.data;
    if (data.length < 2) continue;
    ctx.strokeStyle = item.color;
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    data.forEach((v, i) => {
      v = num(v);
      const x = pad + (i / (maxPoints - 1)) * (width - pad * 1.5);
      const y = pad + (1 - ((v - min) / span)) * (height - pad * 1.6);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }
  ctx.font = `${11 * dpr}px system-ui`;
  let x = pad;
  for (const item of series) {
    ctx.fillStyle = item.color;
    ctx.fillText(item.name, x, height - 8 * dpr);
    x += (item.name.length * 8 + 20) * dpr;
  }
}

function setupEvents() {
  const es = new EventSource("/events");
  es.onmessage = (event) => {
    const payload = JSON.parse(event.data);
    if (payload.kind === "line" || payload.kind === "tx") appendLog(payload.line);
    if (payload.state) updateState(payload.state);
  };
}

function wireUi() {
  document.querySelectorAll(".nav button").forEach((button) => {
    button.addEventListener("click", () => {
      document.querySelectorAll(".nav button").forEach((b) => b.classList.remove("active"));
      document.querySelectorAll(".view").forEach((v) => v.classList.remove("active"));
      button.classList.add("active");
      $(button.dataset.view).classList.add("active");
    });
  });
  document.querySelectorAll("[data-command]").forEach((button) => {
    button.addEventListener("click", () => sendCommand(button.dataset.command));
  });
  $("refreshPorts").addEventListener("click", refreshPorts);
  $("connectBtn").addEventListener("click", async () => {
    try {
      await api("/api/connect", {
        port: $("portSelect").value,
        baud: Number($("baudSelect").value),
        auto_imu: $("autoImu").checked,
      });
    } catch (err) {
      appendLog(`! ${err.message}`);
    }
  });
  $("disconnectBtn").addEventListener("click", () => api("/api/disconnect").catch((err) => appendLog(`! ${err.message}`)));
  $("runStraight").addEventListener("click", () => sendCommand(`straight,${Number($("straightDistance").value).toFixed(2)},${Number($("straightSpeed").value).toFixed(2)}`));
  $("runAB").addEventListener("click", () => {
    $("straightDistance").value = "1.00";
    $("runStraight").click();
  });
  $("runLine").addEventListener("click", () => {
    const speed = Number($("lineSpeed").value).toFixed(2);
    const distance = Number($("lineDistance").value);
    sendCommand(distance > 0 ? `line,${speed},${distance.toFixed(2)}` : `line,${speed}`);
  });
  $("runLine120").addEventListener("click", () => {
    $("lineDistance").value = "1.20";
    $("runLine").click();
  });
  $("runQ2").addEventListener("click", () => {
    sendCommand("q2");
    sendCommand("run");
  });
  $("setWheelRatio").addEventListener("click", () => {
    const base = num($("wheelBase").value, 0);
    const left = base * num($("leftRatio").value, 100) / 100;
    const right = base * num($("rightRatio").value, 100) / 100;
    sendCommand(`spd,${left.toFixed(3)},${right.toFixed(3)}`);
  });
  $("setTwist").addEventListener("click", () => sendCommand(`twist,${num($("twistV").value).toFixed(3)},${num($("twistW").value).toFixed(3)}`));
  $("sendRaw").addEventListener("click", () => {
    const input = $("rawCommand");
    sendCommand(input.value.trim());
    input.value = "";
  });
  $("rawCommand").addEventListener("keydown", (event) => {
    if (event.key === "Enter") $("sendRaw").click();
  });
  $("clearLog").addEventListener("click", () => { $("logBox").textContent = ""; });
  window.addEventListener("resize", render);
}

async function boot() {
  wireUi();
  await refreshPorts();
  setupEvents();
  renderLineSensors();
  render();
}

boot();
