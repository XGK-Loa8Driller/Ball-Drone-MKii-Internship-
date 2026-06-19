const clamp = (value, min, max) => Math.min(max, Math.max(min, value));
const lerp = (a, b, t) => a + (b - a) * t;
const qs = (selector) => document.querySelector(selector);
const qsa = (selector) => [...document.querySelectorAll(selector)];

const state = {
  connected: false,
  armed: false,
  mode: "ANGLE",
  voltage: 12.4,
  altitude: 0,
  yawDeg: 0,
  attitude: { roll: 0, pitch: 0 },
  rc: { throttle: 0, yaw: 0, pitch: 0, roll: 0 },
  trim: { roll: 0, pitch: 0 },
  pid: { p: 105, i: 28, d: 62 },
  location: {
    lat: null,
    lon: null,
    accuracy: null,
    mapUrl: "#",
    source: "Saved",
  },
  outputs: {
    motor: 0,
    front: 1500,
    right: 1500,
    rear: 1500,
    left: 1500,
    vaneNorm: { front: 0, right: 0, rear: 0, left: 0 },
  },
};

let threeModule    = null;
let serialPort     = null;
let serialWriter   = null;
let map            = null;
let marker         = null;
let droneMarker    = null;
let lastFrameTime  = performance.now();
let flightStartTime    = null;
let flightTimerInterval = null;
let toastTimer     = 0;
let gpsToastShown = false;

const el = {
  connectionPill:  qs("#connectionPill"),
  flightTimer:     qs("#flightTimer"),
  connectButton:   qs("#connectButton"),
  armButton:       qs("#armButton"),
  levelButton:     qs("#levelButton"),
  killButton:      qs("#killButton"),
  trimButton:      qs("#trimButton"),
  locateButton:    qs("#locateButton"),
  linkState:       qs("#linkState"),
  modeLabel:       qs("#modeLabel"),
  armedLabel:      qs("#armedLabel"),
  thrustLabel:     qs("#thrustLabel"),
  packetLabel:     qs("#packetLabel"),
  voltageValue:    qs("#voltageValue"),
  altitudeValue:   qs("#altitudeValue"),
  rollValue:       qs("#rollValue"),
  pitchValue:      qs("#pitchValue"),
  throttleReadout: qs("#throttleReadout"),
  yawReadout:      qs("#yawReadout"),
  pitchReadout:    qs("#pitchReadout"),
  rollReadout:     qs("#rollReadout"),
  gpsStatus:       qs("#gpsStatus"),
  latValue:        qs("#latValue"),
  lonValue:        qs("#lonValue"),
  accuracyValue:   qs("#accuracyValue"),
  mapLink:         qs("#mapLink"),
  leftThumb:       qs("#leftThumb"),
  rightThumb:      qs("#rightThumb"),
  toast:           qs("#toast"),
  pfdCanvas:       qs("#pfdCanvas"),
  modelMount:      qs("#modelMount"),
};

// ─── Toast ────────────────────────────────────────────────────────────────────

function showToast(message) {
  el.toast.textContent = message;
  el.toast.classList.add("is-visible");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.toast.classList.remove("is-visible"), 2200);
}

// ─── Connection ───────────────────────────────────────────────────────────────

let _controlInterval = null;

function setConnected(connected, label = connected ? "WIFI" : "SIM") {
  state.connected = connected;
  el.connectionPill.textContent = label;
  el.connectionPill.classList.toggle("is-live", connected);
  el.connectButton.textContent = connected ? "Disconnect" : "Connect";
  el.linkState.textContent = connected ? "WiFi live" : "Bench sim";

  if (connected) {
    if (!_controlInterval) {
      _controlInterval = setInterval(sendControlFrame, 50);
    }
  } else {
    clearInterval(_controlInterval);
    _controlInterval = null;
    setArmed(false);
  }
}

// ─── Arming ───────────────────────────────────────────────────────────────────

function setArmed(armed) {
  if (armed && state.rc.throttle > 0.05) {
    showToast("Lower throttle before arming.");
    return;
  }

  state.armed = armed;
  el.armButton.textContent = armed ? "Disarm" : "Arm";
  el.armButton.classList.toggle("is-armed", armed);
  el.armedLabel.textContent = armed ? "ARMED" : "SAFE";

  if (armed) {
    startFlightTimer();
  } else {
    stopFlightTimer();
  }

  if (!armed) {
    state.outputs.motor = 0;
  }
}

// ─── Outputs ──────────────────────────────────────────────────────────────────

function computeOutputs() {
  const roll   = state.rc.roll  + state.trim.roll;
  const pitch  = state.rc.pitch + state.trim.pitch;
  const yaw    = state.rc.yaw;
  const yawMix = 0.72;

  const vaneNorm = {
    front: clamp(roll  - yaw * yawMix, -1, 1),
    rear:  clamp(roll  + yaw * yawMix, -1, 1),
    left:  clamp(pitch - yaw * yawMix, -1, 1),
    right: clamp(pitch + yaw * yawMix, -1, 1),
  };

  state.outputs.vaneNorm = vaneNorm;
  state.outputs.motor = state.armed ? state.rc.throttle : 0;
  state.outputs.front = Math.round(1500 + vaneNorm.front * 360);
  state.outputs.right = Math.round(1500 + vaneNorm.right * 360);
  state.outputs.rear  = Math.round(1500 + vaneNorm.rear  * 360);
  state.outputs.left  = Math.round(1500 + vaneNorm.left  * 360);
}

function channelUs(name) {
  const { throttle, yaw, pitch, roll } = state.rc;
  if (name === "thr") return Math.round(1000 + throttle * 1000);
  if (name === "yaw") return Math.round(1500 + yaw   * 500);
  if (name === "pit") return Math.round(1500 + pitch  * 500);
  return                      Math.round(1500 + roll   * 500);
}

// ─── Location UI ──────────────────────────────────────────────────────────────

function formatCoordinate(value) {
  return Number.isFinite(value) ? value.toFixed(6) : "--";
}

function updateLocationUI() {
  el.gpsStatus.textContent  = state.location.source.toUpperCase();
  el.latValue.textContent   = state.location.lat === null ? "Tap GPS" : formatCoordinate(state.location.lat);
  el.lonValue.textContent   = state.location.lon === null ? "Saved map" : formatCoordinate(state.location.lon);
  el.accuracyValue.textContent = state.location.accuracy === null
    ? "--"
    : `${Math.round(state.location.accuracy)} m`;
  el.mapLink.href = state.location.mapUrl;
}

// ─── Map (Leaflet) ────────────────────────────────────────────────────────────

function initMap() {
  // Default to Chennai until GPS fires
  const startLat = state.location.lat ?? 13.0827;
  const startLon = state.location.lon ?? 80.2707;

  map = L.map("map", { zoomControl: true, attributionControl: false })
         .setView([startLat, startLon], 16);

  // Satellite-style tiles from Esri — no API key, works offline after first load
  L.tileLayer(
    "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    { maxZoom: 19 }
  ).addTo(map);

  // Phone / pilot marker — amber dot
  const phoneIcon = L.divIcon({
    className: "",
    html: '<div style="width:12px;height:12px;background:#f4bc4f;border-radius:50%;border:2px solid #fff;box-shadow:0 0 8px #f4bc4f"></div>',
    iconSize:   [12, 12],
    iconAnchor: [6,  6],
  });

  // Drone marker — cyan dot
  const droneIcon = L.divIcon({
    className: "",
    html: '<div style="width:12px;height:12px;background:#58c7ef;border-radius:50%;border:2px solid #fff;box-shadow:0 0 8px #58c7ef"></div>',
    iconSize:   [12, 12],
    iconAnchor: [6,  6],
  });

  marker      = L.marker([startLat, startLon], { icon: phoneIcon }).addTo(map);
  droneMarker = L.marker([startLat, startLon], { icon: droneIcon }).addTo(map);
}

// Move phone (amber) marker
function updatePhoneMarker(lat, lon) {
  if (!map || !marker) return;
  marker.setLatLng([lat, lon]);
  map.setView([lat, lon], 18);
}

// Move drone (cyan) marker — called from mergeTelemetry when NEO-M8N data arrives
function updateDroneMarker(lat, lon) {
  if (!map || !droneMarker) return;
  droneMarker.setLatLng([lat, lon]);
}

// ─── Text + Bars ──────────────────────────────────────────────────────────────

function updateTextAndBars() {
  computeOutputs();

  const batteryPercent = clamp(
    ((state.voltage - 10.5) / (12.6 - 10.5)) * 100,
    0, 100
  );

  el.voltageValue.textContent = `${state.voltage.toFixed(1)}V 🔋 ${Math.round(batteryPercent)}%`;
  el.altitudeValue.textContent = `${state.altitude.toFixed(1)} m`;

  if (state.voltage < 11.0) {
    el.voltageValue.style.color = "#ff4d4d";
    if (!window.lowBatteryShown) {
      showToast("⚠ LOW BATTERY");
      window.lowBatteryShown = true;
    }
  } else {
    el.voltageValue.style.color = "";
    window.lowBatteryShown = false;
  }

  el.rollValue.textContent     = `${Math.round(state.attitude.roll)} deg`;
  el.pitchValue.textContent    = `${Math.round(state.attitude.pitch)} deg`;
  el.throttleReadout.textContent = `${Math.round(state.rc.throttle * 100)}%`;
  el.yawReadout.textContent    = `${Math.round(state.rc.yaw   * 100)}%`;
  el.pitchReadout.textContent  = `${Math.round(state.rc.pitch * 100)}%`;
  el.rollReadout.textContent   = `${Math.round(state.rc.roll  * 100)}%`;
  el.modeLabel.textContent     = state.mode;
  el.thrustLabel.textContent   = `${Math.round(state.outputs.motor * 100)}%`;

  const rows = {
    motor: state.outputs.motor,
    front: (state.outputs.front - 1000) / 1000,
    right: (state.outputs.right - 1000) / 1000,
    rear:  (state.outputs.rear  - 1000) / 1000,
    left:  (state.outputs.left  - 1000) / 1000,
  };

  qsa(".bar-row").forEach((row) => {
    const key   = row.dataset.output;
    const value = rows[key];
    row.querySelector("i").style.width = `${clamp(value, 0, 1) * 100}%`;
    row.querySelector("b").textContent = key === "motor"
      ? `${Math.round(state.outputs.motor * 100)}%`
      : state.outputs[key];
  });

  qsa(".rc-row").forEach((row) => {
    const us = channelUs(row.dataset.channel);
    row.querySelector("i").style.setProperty("--pos", String((us - 1000) / 10));
    row.querySelector("b").textContent = us;
  });
}

// ─── Canvas helpers ───────────────────────────────────────────────────────────

function resizeCanvas(canvas) {
  const rect   = canvas.getBoundingClientRect();
  const dpr    = Math.min(window.devicePixelRatio || 1, 2);
  const width  = Math.max(1, Math.round(rect.width  * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width  = width;
    canvas.height = height;
  }
  return { width, height, dpr };
}

function varColor(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

// ─── PFD ─────────────────────────────────────────────────────────────────────

function drawPfd() {
  const canvas = el.pfdCanvas;
  const ctx    = canvas.getContext("2d");
  const { width, height, dpr } = resizeCanvas(canvas);
  const cx = width  / 2;
  const cy = height / 2;
  const roll        = (state.attitude.roll  * Math.PI) / 180;
  const pitchOffset = state.attitude.pitch * dpr * 1.7;

  ctx.clearRect(0, 0, width, height);
  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(-roll);
  ctx.translate(0, pitchOffset);

  ctx.fillStyle = "#2f7aa0";
  ctx.fillRect(-width, -height * 2, width * 2, height * 2);
  ctx.fillStyle = "#7b5a34";
  ctx.fillRect(-width, 0, width * 2, height * 2);
  ctx.strokeStyle = "#f2f7f1";
  ctx.lineWidth = 2 * dpr;
  ctx.beginPath();
  ctx.moveTo(-width, 0);
  ctx.lineTo( width, 0);
  ctx.stroke();

  ctx.strokeStyle = "rgba(242,247,241,0.55)";
  ctx.lineWidth = 1 * dpr;
  for (let deg = -30; deg <= 30; deg += 10) {
    if (deg === 0) continue;
    const y = -deg * dpr * 1.7;
    ctx.beginPath();
    ctx.moveTo(-28 * dpr, y);
    ctx.lineTo( 28 * dpr, y);
    ctx.stroke();
  }
  ctx.restore();

  ctx.strokeStyle = varColor("--amber");
  ctx.lineWidth   = 3 * dpr;
  ctx.beginPath();
  ctx.moveTo(cx - 34 * dpr, cy);
  ctx.lineTo(cx -  9 * dpr, cy);
  ctx.moveTo(cx +  9 * dpr, cy);
  ctx.lineTo(cx + 34 * dpr, cy);
  ctx.moveTo(cx, cy - 6 * dpr);
  ctx.lineTo(cx, cy + 8 * dpr);
  ctx.stroke();

  ctx.fillStyle = "rgba(8,16,13,0.72)";
  ctx.fillRect(0, height - 24 * dpr, width, 24 * dpr);
  ctx.fillStyle  = "#f2f7f1";
  ctx.font       = `${11 * dpr}px ui-sans-serif, system-ui`;
  ctx.textAlign  = "center";
  ctx.fillText(
    `YAW ${Math.round(((state.yawDeg % 360) + 360) % 360)} deg`,
    cx, height - 8 * dpr
  );
}

// ─── Joysticks ────────────────────────────────────────────────────────────────

function initJoysticks() {
  qsa(".joystick").forEach((stick) => {
    const type  = stick.dataset.stick;
    const thumb = type === "left" ? el.leftThumb : el.rightThumb;

    const setThumb = (x, y) => {
      thumb.style.left = `${50 + x * 32}%`;
      thumb.style.top  = `${50 + y * 32}%`;
    };

    const updateFromPointer = (event) => {
      const rect   = stick.getBoundingClientRect();
      const cx     = rect.left + rect.width  / 2;
      const cy     = rect.top  + rect.height / 2;
      const radius = rect.width * 0.42;
      let x = (event.clientX - cx) / radius;
      let y = (event.clientY - cy) / radius;
      const length = Math.hypot(x, y);
      if (length > 1) { x /= length; y /= length; }

      if (type === "left") {
        state.rc.yaw      = clamp(x, -1, 1);
        state.rc.throttle = clamp((1 - y) / 2, 0, 1);
      } else {
        state.rc.roll  = clamp(x,  -1, 1);
        state.rc.pitch = clamp(-y, -1, 1);
      }
      setThumb(x, y);
      updateTextAndBars();
    };

    stick.addEventListener("pointerdown", (event) => {
      stick.setPointerCapture(event.pointerId);
      updateFromPointer(event);
    });

    stick.addEventListener("pointermove", (event) => {
      if (stick.hasPointerCapture(event.pointerId)) updateFromPointer(event);
    });

    const release = (event) => {
      if (stick.hasPointerCapture(event.pointerId)) stick.releasePointerCapture(event.pointerId);
      if (type === "left") {
        state.rc.yaw = 0;
        setThumb(0, 1 - state.rc.throttle * 2);
      } else {
        state.rc.roll  = 0;
        state.rc.pitch = 0;
        setThumb(0, 0);
      }
      updateTextAndBars();
    };

    stick.addEventListener("pointerup",     release);
    stick.addEventListener("pointercancel", release);
  });
}

// ─── Serial ───────────────────────────────────────────────────────────────────

async function connectSerial() {
  if (!("serial" in navigator)) {
    showToast("USB serial needs a Web Serial capable browser or native Android wrapper.");
    return;
  }
  try {
    serialPort   = await navigator.serial.requestPort();
    await serialPort.open({ baudRate: 115200 });
    serialWriter = serialPort.writable.getWriter();
    setConnected(true, "USB");
    showToast("USB serial connected.");
    readSerialLoop();
  } catch {
    showToast("Serial connection was not opened.");
  }
}

async function readSerialLoop() {
  if (!serialPort?.readable) return;
  const decoder = new TextDecoder();
  const reader  = serialPort.readable.getReader();
  let buffer    = "";
  try {
    while (state.connected) {
      const { value, done } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split("\n");
      buffer = lines.pop() || "";
      lines.forEach((line) => {
        try   { mergeTelemetry(JSON.parse(line)); }
        catch { /* non-JSON line, ignore */ }
      });
    }
  } finally {
    reader.releaseLock();
  }
}

// ─── Telemetry merge (handles NEO-M8N GPS data from ESP32) ───────────────────

function mergeTelemetry(packet) {
  if (Number.isFinite(packet.voltage))  state.voltage        = packet.voltage;
  if (Number.isFinite(packet.altitude)) state.altitude       = packet.altitude;
  if (Number.isFinite(packet.roll))     state.attitude.roll  = packet.roll;
  if (Number.isFinite(packet.pitch))    state.attitude.pitch = packet.pitch;
  if (Number.isFinite(packet.yaw))      state.yawDeg         = packet.yaw;

  // Drone GPS from NEO-M8N — updates the cyan marker on the map
  if (packet.location &&
      Number.isFinite(packet.location.lat) &&
      Number.isFinite(packet.location.lon)) {
    state.location.lat      = packet.location.lat;
    state.location.lon      = packet.location.lon;
    state.location.accuracy = Number.isFinite(packet.location.accuracy)
      ? packet.location.accuracy : null;
    state.location.source   = "Drone";
    state.location.mapUrl   = `https://www.google.com/maps?q=${state.location.lat},${state.location.lon}`;
    updateLocationUI();
    updateDroneMarker(state.location.lat, state.location.lon);
  }
}

// ─── Control frame sender ─────────────────────────────────────────────────────

let _imgInFlight = false;

async function sendControlFrame() {
  if (_imgInFlight) return;
  _imgInFlight = true;

  const thr = channelUs("thr");
  const yaw = channelUs("yaw");
  const pit = channelUs("pit");
  const rol = channelUs("rol");

  // USB serial path
  if (serialWriter) {
    const frame = {
      type:    "BDMK2_RC",
      armed:   state.armed,
      mode:    state.mode,
      rc:      { throttle: thr, yaw, pitch: pit, roll: rol },
      outputs: state.outputs,
      pid:     state.pid,
      timestamp: Date.now(),
    };
    try {
      await serialWriter.write(new TextEncoder().encode(`${JSON.stringify(frame)}\n`));
    } catch {
      setConnected(false);
      serialWriter = null;
    }
  }

  // WiFi path — Image() bypasses CORS/mixed-content on http:// scheme
  const img    = new Image();
  img.onerror  = () => { _imgInFlight = false; };
  img.onload   = () => { _imgInFlight = false; };
  img.src = `http://192.168.4.1/control?thr=${thr}&yaw=${yaw}&pit=${pit}&rol=${rol}&armed=${state.armed}`;
}

// ─── Drone simulation (bench mode) ───────────────────────────────────────────

function simulateDrone(dt) {
  const targetRoll  = (state.rc.roll  + state.trim.roll)  * 22;
  const targetPitch = (state.rc.pitch + state.trim.pitch) * 22;
  const response    = state.mode === "ANGLE" ? 0.08 : 0.035;
  state.attitude.roll  = lerp(state.attitude.roll,  targetRoll,  response);
  state.attitude.pitch = lerp(state.attitude.pitch, targetPitch, response);
  state.yawDeg        += state.rc.yaw * dt * 90;

  if (state.armed) {
    const lift     = (state.rc.throttle - 0.44) * dt * 2.2;
    state.altitude = Math.max(0, state.altitude + lift);
    state.voltage  = Math.max(10.4, state.voltage - dt * (0.0016 + state.rc.throttle * 0.0028));
  } else {
    state.altitude = Math.max(0, state.altitude - dt * 0.4);
  }
}

// ─── 3-D model ────────────────────────────────────────────────────────────────

function initModel() {
  if (threeModule) initThreeModel(threeModule);
  else             initFallbackModel();
}

function initThreeModel(THREE) {
  const scene    = new THREE.Scene();
  const camera   = new THREE.PerspectiveCamera(40, 1, 0.1, 100);
  camera.position.set(0, 2.6, 5.3);
  camera.lookAt(0, 0, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setClearColor(0x000000, 0);
  el.modelMount.appendChild(renderer.domElement);

  const drone        = new THREE.Group();
  scene.add(drone);

  const cageMaterial = new THREE.MeshStandardMaterial({ color: 0x66d99b, metalness: 0.25, roughness: 0.45 });
  const vaneMaterial = new THREE.MeshStandardMaterial({ color: 0x58c7ef, roughness: 0.55 });
  const propMaterial = new THREE.MeshStandardMaterial({ color: 0xf4bc4f, roughness: 0.38 });
  const darkMaterial = new THREE.MeshStandardMaterial({ color: 0x0f1713, roughness: 0.7  });

  const torusGeometry = new THREE.TorusGeometry(1.28, 0.035, 12, 80);
  const ringA = new THREE.Mesh(torusGeometry, cageMaterial);
  const ringB = new THREE.Mesh(torusGeometry, cageMaterial);
  const ringC = new THREE.Mesh(torusGeometry, cageMaterial);
  ringB.rotation.x = Math.PI / 2;
  ringC.rotation.y = Math.PI / 2;
  drone.add(ringA, ringB, ringC);

  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, 0.22, 36), darkMaterial);
  hub.rotation.x = Math.PI / 2;
  drone.add(hub);

  const prop       = new THREE.Group();
  const bladeGeo   = new THREE.BoxGeometry(1.55, 0.08, 0.025);
  const bladeA     = new THREE.Mesh(bladeGeo, propMaterial);
  const bladeB     = new THREE.Mesh(bladeGeo, propMaterial);
  bladeB.rotation.z = Math.PI / 2;
  prop.add(bladeA, bladeB);
  prop.position.z = 0.02;
  drone.add(prop);

  const vanes        = {};
  const vaneGeometry = new THREE.BoxGeometry(0.15, 0.52, 0.035);
  const vanePositions = {
    front: [ 0,     0.95, -0.46],
    rear:  [ 0,    -0.95, -0.46],
    left:  [-0.95,  0,    -0.46],
    right: [ 0.95,  0,    -0.46],
  };
  Object.entries(vanePositions).forEach(([key, position]) => {
    const vane = new THREE.Mesh(vaneGeometry, vaneMaterial);
    vane.position.set(...position);
    if (key === "left" || key === "right") vane.rotation.z = Math.PI / 2;
    drone.add(vane);
    vanes[key] = vane;
  });

  const lightA = new THREE.DirectionalLight(0xffffff, 2.1);
  lightA.position.set(3, 5, 4);
  const lightB = new THREE.AmbientLight(0x9fd5ba, 1.5);
  scene.add(lightA, lightB);

  const resize = () => {
    const rect   = el.modelMount.getBoundingClientRect();
    const width  = Math.max(1, Math.floor(rect.width));
    const height = Math.max(1, Math.floor(rect.height));
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  };
  new ResizeObserver(resize).observe(el.modelMount);
  resize();

  const render = () => {
    drone.rotation.z = (-state.attitude.roll  * Math.PI) / 180;
    drone.rotation.x = ( state.attitude.pitch * Math.PI) / 180;
    drone.rotation.y = ( state.yawDeg         * Math.PI) / 180;
    prop.rotation.z += 0.14 + state.outputs.motor * 0.64;
    vanes.front.rotation.y =  state.outputs.vaneNorm.front * 0.75;
    vanes.rear.rotation.y  =  state.outputs.vaneNorm.rear  * 0.75;
    vanes.left.rotation.x  = -state.outputs.vaneNorm.left  * 0.75;
    vanes.right.rotation.x = -state.outputs.vaneNorm.right * 0.75;
    renderer.render(scene, camera);
    requestAnimationFrame(render);
  };
  render();
}

function initFallbackModel() {
  const canvas = document.createElement("canvas");
  el.modelMount.appendChild(canvas);
  const ctx = canvas.getContext("2d");

  const render = () => {
    const { width, height } = resizeCanvas(canvas);
    const cx = width  / 2;
    const cy = height / 2;
    const r  = Math.min(width, height) * 0.33;
    ctx.clearRect(0, 0, width, height);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate((-state.attitude.roll * Math.PI) / 180);
    ctx.strokeStyle = "#54df92";
    ctx.lineWidth   = Math.max(2, width * 0.012);
    ctx.beginPath();
    ctx.arc(0, 0, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.strokeStyle = "#f4bc4f";
    ctx.rotate(performance.now() * 0.006 * (0.5 + state.outputs.motor));
    ctx.beginPath();
    ctx.moveTo(-r * 0.65, 0);
    ctx.lineTo( r * 0.65, 0);
    ctx.moveTo(0, -r * 0.65);
    ctx.lineTo(0,  r * 0.65);
    ctx.stroke();
    ctx.restore();
    requestAnimationFrame(render);
  };
  render();
}

// ─── Phone GPS ────────────────────────────────────────────────────────────────

function requestPhoneLocation() {
  if (!navigator.geolocation) {
    showToast("Phone GPS is not available.");
    return;
  }

  state.location.source = "Asking";
  updateLocationUI();

  navigator.geolocation.watchPosition(
  (position) => {
    state.location.lat      = position.coords.latitude;
    state.location.lon      = position.coords.longitude;
    state.location.accuracy = position.coords.accuracy;
    state.location.source   = "Phone";
    state.location.mapUrl   = `https://www.google.com/maps?q=${state.location.lat},${state.location.lon}`;

    updateLocationUI();
    updatePhoneMarker(state.location.lat, state.location.lon);

    // showToast("Phone GPS location added.");
  },
    () => {
      state.location.source = "Saved";
      updateLocationUI();
      showToast("Location permission was not granted.");
    },
    { enableHighAccuracy: true, maximumAge: 5000, timeout: 12000 }
  );
}

// ─── Controls ─────────────────────────────────────────────────────────────────

function initControls() {
  el.connectButton.addEventListener("click", () => {
    if (state.connected) {
      setConnected(false);
      showToast("Disconnected from drone.");
    } else {
      fetch("http://192.168.4.1/")
        .then(() => {
          setConnected(true, "WIFI");
          showToast("Connected to Ball Drone.");
        })
        .catch(() => {
          // fetch will fail due to CORS but network path exists — connect anyway
          setConnected(true, "WIFI");
          showToast("Connected to Ball Drone.");
        });
    }
  });

  el.locateButton.addEventListener("click", requestPhoneLocation);
  el.armButton.addEventListener("click",    () => setArmed(!state.armed));

  el.killButton.addEventListener("click", () => {
    state.rc.throttle     = 0;
    state.rc.yaw          = 0;
    setArmed(false);
    el.leftThumb.style.left = "50%";
    el.leftThumb.style.top  = "82%";
    showToast("Motor output killed.");
  });

  el.levelButton.addEventListener("click", () => {
    state.trim.roll       = 0;
    state.trim.pitch      = 0;
    state.attitude.roll   = 0;
    state.attitude.pitch  = 0;
    showToast("Level reference reset.");
  });

  el.trimButton.addEventListener("click", () => {
    state.trim.roll  = -state.rc.roll  * 0.15;
    state.trim.pitch = -state.rc.pitch * 0.15;
    showToast("Current stick trim stored.");
  });

  qsa(".mode-button").forEach((button) => {
    button.addEventListener("click", () => {
      qsa(".mode-button").forEach((item) => item.classList.remove("is-active"));
      button.classList.add("is-active");
      state.mode = button.dataset.mode;
      updateTextAndBars();
    });
  });

  qsa(".slider-row input").forEach((slider) => {
    slider.addEventListener("input", () => {
      const value = Number(slider.value);
      slider.nextElementSibling.textContent = value;
      if (slider.id === "pidP") state.pid.p = value;
      if (slider.id === "pidI") state.pid.i = value;
      if (slider.id === "pidD") state.pid.d = value;
    });
  });
}

// ─── Flight timer ─────────────────────────────────────────────────────────────

function startFlightTimer() {
  flightStartTime = Date.now();
  clearInterval(flightTimerInterval);
  flightTimerInterval = setInterval(() => {
    const elapsed  = Math.floor((Date.now() - flightStartTime) / 1000);
    const minutes  = String(Math.floor(elapsed / 60)).padStart(2, "0");
    const seconds  = String(elapsed % 60).padStart(2, "0");
    el.flightTimer.textContent = `${minutes}:${seconds}`;
  }, 1000);
}

function stopFlightTimer() {
  clearInterval(flightTimerInterval);
  el.flightTimer.textContent = "00:00";
}

// ─── Main loop ────────────────────────────────────────────────────────────────

function loop(now) {
  const dt = Math.min(0.05, (now - lastFrameTime) / 1000);
  lastFrameTime = now;
  simulateDrone(dt);
  updateTextAndBars();
  drawPfd();
  requestAnimationFrame(loop);
}

// ─── Boot ─────────────────────────────────────────────────────────────────────

async function boot() {
  try {
    threeModule = await import("https://unpkg.com/three@0.160.1/build/three.module.js");
  } catch {
    threeModule = null;
  }

  initControls();
  initJoysticks();
  initModel();
  initMap();          // Leaflet — always works, no callback needed
  updateLocationUI();
  updateTextAndBars();
  requestAnimationFrame(loop);
}

boot();