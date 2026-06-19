/**
 * Ball Drone MK II — GPS Tracking Module
 * Tuned for: NEO-6M (GY-NEO6MV2), GPS constellation only, max 5 Hz
 *
 * Extend your existing app.js — do NOT replace it.
 * Hook in with 3 lines:
 *
 *   // DOMContentLoaded:
 *   window.BDGps.init();
 *
 *   // Inside mergeTelemetry(data):
 *   window.BDGps.updateDroneGPS(data);
 *
 *   // Inside phone GPS watchPosition callback:
 *   window.BDGps.updatePilotPosition(lat, lon);
 */

// ═══════════════════════════════════════════════════════════════════════════════
// 1. CONSTANTS — tuned for NEO-6M behaviour
// ═══════════════════════════════════════════════════════════════════════════════
const GPS_CONSTANTS = {
  EARTH_RADIUS_M:    6371000,

  // NEO-6M at 5 Hz sends a packet every 200 ms.
  // Allow 3 missed packets (600 ms) before marking fix stale,
  // but wait 6 s total before declaring LOST (monocopter spins
  // can briefly lose signal without an actual fix loss).
  FIX_STALE_MS:      600,
  FIX_LOST_MS:       6000,

  PATH_MAX_POINTS:   500,

  // NEO-6M HDOP thresholds
  HDOP_GOOD:         2.0,   // excellent outdoors
  HDOP_USABLE:       5.0,   // acceptable
  HDOP_BAD:         10.0,   // marginal — warn user

  // Minimum satellites for a reliable fix on NEO-6M (GPS-only)
  SAT_MIN_RELIABLE:  6,

  TILE_CACHE_NAME:  'balldrone-tiles-v1',
};

// ═══════════════════════════════════════════════════════════════════════════════
// 2. STATE
// ═══════════════════════════════════════════════════════════════════════════════
const gpsState = {
  drone: {
    lat: null, lon: null,
    alt: 0, satellites: 0, hdop: 99.9,
    fix: false, speed: 0, course: 0,
    lastUpdateMs: 0,
    module: 'NEO-6M',
  },
  pilot: { lat: null, lon: null },
  home:  { lat: null, lon: null, set: false },

  trackEnabled:  false,
  flightPath:    [],          // [{lat,lon}, …] capped at 500 points
  fixLost:       false,
  fixStale:      false,       // between stale and lost
};

// ═══════════════════════════════════════════════════════════════════════════════
// 3. MAP MARKERS
// ═══════════════════════════════════════════════════════════════════════════════
let droneMarker    = null;
let pilotMarker    = null;
let homeMarker     = null;
let pilotDroneLine = null;
let flightPolyline = null;

function makeSvgIcon(color, shape = 'drone') {
  const shapes = {
    drone: `<polygon points="12,2 22,22 12,17 2,22"
              fill="${color}" stroke="#0a0e1a" stroke-width="1.5"/>`,
    pilot: `<circle cx="12" cy="12" r="9"
              fill="${color}" stroke="#0a0e1a" stroke-width="1.5"/>
            <text x="12" y="16" text-anchor="middle"
              font-size="9" font-weight="bold" fill="#0a0e1a">P</text>`,
    home:  `<polygon points="12,2 20,10 17,10 17,22 7,22 7,10 4,10"
              fill="${color}" stroke="#0a0e1a" stroke-width="1.5"/>`,
  };
  const svg = `<svg xmlns="http://www.w3.org/2000/svg"
                  viewBox="0 0 24 24" width="32" height="32"
                  style="filter:drop-shadow(0 2px 4px rgba(0,0,0,.6))">
                  ${shapes[shape]}
               </svg>`;
  return L.divIcon({
    html: svg, className: 'bd-map-icon',
    iconSize: [32,32], iconAnchor: [16,16], popupAnchor: [0,-16],
  });
}

function initMapMarkers() {
  if (!window.map) {
    console.error('[GPS] window.map not found — init Leaflet map first');
    return;
  }

  // Drone — cyan, turns red on fix loss
  droneMarker = L.marker([13.0827, 80.2707], {
    icon: makeSvgIcon('#00e5ff', 'drone'),
    zIndexOffset: 1000,
  }).addTo(window.map)
    .bindPopup('<b style="font-family:monospace">Drone</b><br><small>Awaiting NEO-6M fix…</small>');

  // Pilot — amber
  pilotMarker = L.marker([13.0827, 80.2707], {
    icon: makeSvgIcon('#ffab00', 'pilot'),
    zIndexOffset: 900,
  }).addTo(window.map)
    .bindPopup('<b style="font-family:monospace">Pilot</b>');

  // Pilot-to-Drone dashed line
  pilotDroneLine = L.polyline([], {
    color: '#ffab00', weight: 1.5, opacity: 0.65, dashArray: '6 4',
  }).addTo(window.map);

  // Flight path trail
  flightPolyline = L.polyline([], {
    color: '#00e5ff', weight: 2, opacity: 0.75,
  }).addTo(window.map);

  console.log('[GPS] Markers initialised (NEO-6M mode, 5 Hz)');
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. COORDINATE MATH
// ═══════════════════════════════════════════════════════════════════════════════

/** Haversine distance in metres */
function distanceBetweenPoints(lat1, lon1, lat2, lon2) {
  const R  = GPS_CONSTANTS.EARTH_RADIUS_M;
  const φ1 = lat1 * Math.PI / 180;
  const φ2 = lat2 * Math.PI / 180;
  const Δφ = (lat2 - lat1) * Math.PI / 180;
  const Δλ = (lon2 - lon1) * Math.PI / 180;
  const a  = Math.sin(Δφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(Δλ/2)**2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
}

/** Initial bearing in degrees (0 = North, clockwise) */
function bearingBetweenPoints(lat1, lon1, lat2, lon2) {
  const φ1 = lat1 * Math.PI / 180;
  const φ2 = lat2 * Math.PI / 180;
  const Δλ = (lon2 - lon1) * Math.PI / 180;
  const y  = Math.sin(Δλ) * Math.cos(φ2);
  const x  = Math.cos(φ1)*Math.sin(φ2) - Math.sin(φ1)*Math.cos(φ2)*Math.cos(Δλ);
  return ((Math.atan2(y, x) * 180 / Math.PI) + 360) % 360;
}

function formatBearing(deg) {
  const dirs = ['N','NE','E','SE','S','SW','W','NW'];
  return `${Math.round(deg)}° ${dirs[Math.round(deg/45)%8]}`;
}

function formatDistance(m) {
  return m >= 1000 ? `${(m/1000).toFixed(2)} km` : `${Math.round(m)} m`;
}

/** NEO-6M HDOP quality label */
function hdopQuality(hdop) {
  if (hdop <= GPS_CONSTANTS.HDOP_GOOD)   return { label: 'Excellent', cls: 'gps-value--good' };
  if (hdop <= GPS_CONSTANTS.HDOP_USABLE) return { label: 'Good',      cls: 'gps-value--ok'  };
  if (hdop <= GPS_CONSTANTS.HDOP_BAD)    return { label: 'Marginal',  cls: 'gps-value--bad' };
  return                                          { label: 'Poor',     cls: 'gps-value--lost'};
}

// ═══════════════════════════════════════════════════════════════════════════════
// 5. TELEMETRY INGESTION
// Call window.BDGps.updateDroneGPS(data) from your mergeTelemetry(data)
// ═══════════════════════════════════════════════════════════════════════════════
function updateDroneGPS(data) {
  if (!data?.location) return;
  const loc = data.location;
  if (typeof loc.lat !== 'number' || typeof loc.lon !== 'number') return;
  if (Math.abs(loc.lat) > 90 || Math.abs(loc.lon) > 180) return;

  const hadFix = gpsState.drone.fix;

  gpsState.drone.lat          = loc.lat;
  gpsState.drone.lon          = loc.lon;
  gpsState.drone.alt          = data.altitude           ?? gpsState.drone.alt;
  gpsState.drone.satellites   = loc.satellites          ?? gpsState.drone.satellites;
  gpsState.drone.hdop         = typeof loc.hdop === 'number' ? loc.hdop : gpsState.drone.hdop;
  gpsState.drone.fix          = !!loc.fix;
  gpsState.drone.speed        = loc.speed               ?? gpsState.drone.speed;
  gpsState.drone.course       = loc.course              ?? gpsState.drone.course;
  gpsState.drone.lastUpdateMs = Date.now();

  // NEO-6M: also require ≥ 4 satellites for a usable position
  const enoughSats = gpsState.drone.satellites >= 4;
  const effectiveFix = gpsState.drone.fix && enoughSats;

  if (!hadFix && effectiveFix) {
    gpsState.fixLost  = false;
    gpsState.fixStale = false;
    showGpsToast(`GPS Fix acquired — ${gpsState.drone.satellites} sats ✓`, 'success');
    setDroneMarkerColor('#00e5ff');
  }

  if (effectiveFix) {
    updateDroneMarker();
    updateFlightPath();
    updatePilotDroneLine();
    if (gpsState.trackEnabled) panMapToDrone();
  }

  updateGpsPanel();
}

// ═══════════════════════════════════════════════════════════════════════════════
// 6. MARKER & PATH UPDATES
// ═══════════════════════════════════════════════════════════════════════════════
function updateDroneMarker() {
  if (!droneMarker) return;
  const { lat, lon, fix } = gpsState.drone;
  droneMarker.setLatLng([lat, lon]);
  // Stale = amber, lost = red, good = cyan
  const color = gpsState.fixLost ? '#f44336'
              : gpsState.fixStale ? '#ffab00'
              : '#00e5ff';
  droneMarker.setIcon(makeSvgIcon(color, 'drone'));
}

function setDroneMarkerColor(color) {
  if (droneMarker) droneMarker.setIcon(makeSvgIcon(color, 'drone'));
}

function updateFlightPath() {
  const { lat, lon } = gpsState.drone;
  gpsState.flightPath.push({ lat, lon });
  if (gpsState.flightPath.length > GPS_CONSTANTS.PATH_MAX_POINTS)
    gpsState.flightPath.shift();
  flightPolyline?.setLatLngs(gpsState.flightPath.map(p => [p.lat, p.lon]));
}

function updatePilotDroneLine() {
  const { pilot, drone } = gpsState;
  if (pilot.lat == null || drone.lat == null) return;
  pilotDroneLine?.setLatLngs([
    [pilot.lat, pilot.lon],
    [drone.lat, drone.lon],
  ]);
}

/** Call from your phone geolocation watchPosition handler */
function updatePilotPosition(lat, lon) {
  gpsState.pilot = { lat, lon };
  pilotMarker?.setLatLng([lat, lon]);
  updatePilotDroneLine();
  updateGpsPanel();
}

function panMapToDrone() {
  const { lat, lon } = gpsState.drone;
  window.map?.panTo([lat, lon], { animate: true, duration: 0.5 });
}

// ═══════════════════════════════════════════════════════════════════════════════
// 7. HOME POSITION
// ═══════════════════════════════════════════════════════════════════════════════
function setHomePosition() {
  const { lat, lon, fix, satellites, hdop } = gpsState.drone;

  if (!fix || lat == null) {
    showGpsToast('No GPS fix — cannot set home', 'warning');
    return;
  }

  // Extra check for NEO-6M: warn if signal quality is poor
  if (satellites < GPS_CONSTANTS.SAT_MIN_RELIABLE) {
    showGpsToast(`Only ${satellites} sats — home set with low confidence`, 'warning');
  } else if (hdop > GPS_CONSTANTS.HDOP_USABLE) {
    showGpsToast(`HDOP ${hdop.toFixed(1)} — home set with marginal accuracy`, 'warning');
  } else {
    showGpsToast(`Home set ✓  (${satellites} sats, HDOP ${hdop.toFixed(1)})`, 'success');
  }

  gpsState.home = { lat, lon, set: true };

  if (!homeMarker) {
    homeMarker = L.marker([lat, lon], {
      icon: makeSvgIcon('#00c853', 'home'),
      zIndexOffset: 800,
    }).addTo(window.map)
      .bindPopup(`<b style="font-family:monospace">Home</b><br>
        <small>${lat.toFixed(6)}, ${lon.toFixed(6)}</small>`);
  } else {
    homeMarker.setLatLng([lat, lon]);
    homeMarker.getPopup()?.setContent(
      `<b style="font-family:monospace">Home</b><br>
       <small>${lat.toFixed(6)}, ${lon.toFixed(6)}</small>`
    );
  }

  // Highlight SET HOME button briefly
  const btn = document.getElementById('btn-set-home');
  if (btn) {
    btn.classList.add('btn--active');
    setTimeout(() => btn.classList.remove('btn--active'), 2000);
  }

  updateGpsPanel();
}

// ═══════════════════════════════════════════════════════════════════════════════
// 8. FIX WATCHDOG
// Two-stage: STALE (amber) then LOST (red)
// NEO-6M at 5 Hz: expected update every 200 ms
// ═══════════════════════════════════════════════════════════════════════════════
function startFixWatchdog() {
  setInterval(() => {
    if (!gpsState.drone.lastUpdateMs) return;
    const age = Date.now() - gpsState.drone.lastUpdateMs;

    // Stage 1 — STALE: no update for >600 ms (3 missed 5Hz packets)
    if (age > GPS_CONSTANTS.FIX_STALE_MS && !gpsState.fixStale && !gpsState.fixLost) {
      gpsState.fixStale = true;
      setDroneMarkerColor('#ffab00');
      updateGpsPanel();
    }

    // Stage 2 — LOST: no update for >6 s
    if (age > GPS_CONSTANTS.FIX_LOST_MS && !gpsState.fixLost) {
      gpsState.fixLost       = true;
      gpsState.fixStale      = false;
      gpsState.drone.fix     = false;
      setDroneMarkerColor('#f44336');
      showGpsToast('⚠ GPS Fix LOST — position frozen', 'danger');
      updateGpsPanel();
    }

    // Recovery
    if (age <= GPS_CONSTANTS.FIX_STALE_MS) {
      gpsState.fixStale = false;
      gpsState.fixLost  = false;
    }
  }, 500);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 9. GPS PANEL UPDATER
// ═══════════════════════════════════════════════════════════════════════════════
function updateGpsPanel() {
  const { drone, pilot, home, fixLost, fixStale } = gpsState;

  const setEl = (id, val) => { const e = document.getElementById(id); if (e) e.textContent = val; };
  const setClass = (id, cls) => {
    const e = document.getElementById(id);
    if (!e) return;
    e.className = 'gps-value ' + cls;
  };

  // Coordinates
  setEl('gps-lat', drone.lat != null ? drone.lat.toFixed(7) : '--');
  setEl('gps-lon', drone.lon != null ? drone.lon.toFixed(7) : '--');
  setEl('gps-alt', drone.alt != null ? `${drone.alt.toFixed(1)} m` : '--');

  // Satellites — colour-code by count
  setEl('gps-sats', drone.satellites);
  setClass('gps-sats',
    drone.satellites >= GPS_CONSTANTS.SAT_MIN_RELIABLE ? 'gps-value--good'
    : drone.satellites >= 4 ? 'gps-value--ok'
    : 'gps-value--bad'
  );

  // HDOP
  const hq = hdopQuality(drone.hdop);
  setEl('gps-hdop', `${drone.hdop.toFixed(2)} (${hq.label})`);
  setClass('gps-hdop', hq.cls);

  // Fix status badge
  const fixEl = document.getElementById('gps-fix');
  if (fixEl) {
    if (fixLost) {
      fixEl.textContent = 'LOST';
      fixEl.className   = 'gps-fix-badge gps-value--lost';
    } else if (fixStale) {
      fixEl.textContent = 'STALE';
      fixEl.className   = 'gps-fix-badge gps-value--bad';
    } else if (drone.fix) {
      fixEl.textContent = 'FIX ✓';
      fixEl.className   = 'gps-fix-badge gps-value--good';
    } else {
      fixEl.textContent = 'NO FIX';
      fixEl.className   = 'gps-fix-badge gps-value--bad';
    }
  }

  // Motion
  setEl('gps-speed',   `${drone.speed.toFixed(1)} km/h`);
  setEl('gps-heading', formatBearing(drone.course));

  // GPS update age
  const ageMs = drone.lastUpdateMs ? Date.now() - drone.lastUpdateMs : null;
  setEl('gps-age', ageMs != null ? `${(ageMs/1000).toFixed(1)} s` : '--');
  // Colour the age: >600ms = stale (amber), >6s = lost (red)
  setClass('gps-age',
    ageMs == null ? '' :
    ageMs > GPS_CONSTANTS.FIX_LOST_MS  ? 'gps-value--lost' :
    ageMs > GPS_CONSTANTS.FIX_STALE_MS ? 'gps-value--bad'  : 'gps-value--good'
  );

  // Distances — freeze if fix lost
  if (!fixLost && drone.lat != null && pilot.lat != null) {
    const d = distanceBetweenPoints(pilot.lat, pilot.lon, drone.lat, drone.lon);
    const b = bearingBetweenPoints(pilot.lat, pilot.lon, drone.lat, drone.lon);
    setEl('gps-dist-pilot', formatDistance(d));
    setEl('gps-bear-pilot', formatBearing(b));
  } else {
    setEl('gps-dist-pilot', '--');
    setEl('gps-bear-pilot', '--');
  }

  if (!fixLost && drone.lat != null && home.set) {
    const d = distanceBetweenPoints(drone.lat, drone.lon, home.lat, home.lon);
    const b = bearingBetweenPoints(drone.lat, drone.lon, home.lat, home.lon);
    setEl('gps-dist-home', formatDistance(d));
    setEl('gps-bear-home', formatBearing(b));
  } else {
    setEl('gps-dist-home', home.set ? '--' : 'Not set');
    setEl('gps-bear-home', '--');
  }

  // Module tag
  setEl('gps-module', drone.module || 'NEO-6M');
}

// ═══════════════════════════════════════════════════════════════════════════════
// 10. TRACK TOGGLE
// ═══════════════════════════════════════════════════════════════════════════════
function toggleTrack() {
  gpsState.trackEnabled = !gpsState.trackEnabled;
  const btn = document.getElementById('btn-track');
  if (btn) {
    btn.textContent = gpsState.trackEnabled ? 'TRACK ON' : 'TRACK OFF';
    btn.classList.toggle('btn--active', gpsState.trackEnabled);
  }
  if (gpsState.trackEnabled && gpsState.drone.fix) panMapToDrone();
}

// ═══════════════════════════════════════════════════════════════════════════════
// 11. OFFLINE TILE CACHE
// ═══════════════════════════════════════════════════════════════════════════════
function initOfflineTiles() {
  if ('serviceWorker' in navigator) {
    navigator.serviceWorker.register('./sw-tiles.js')
      .then(reg  => console.log('[SW] Tile cache active:', reg.scope))
      .catch(err => { console.warn('[SW] Registration failed:', err); addGridFallbackLayer(); });
  } else {
    addGridFallbackLayer();
  }
}

function addGridFallbackLayer() {
  const GridLayer = L.GridLayer.extend({
    createTile(coords) {
      const tile = document.createElement('canvas');
      const size = this.getTileSize();
      tile.width = size.x; tile.height = size.y;
      const ctx  = tile.getContext('2d');
      ctx.fillStyle = '#111827';
      ctx.fillRect(0, 0, size.x, size.y);
      ctx.strokeStyle = '#1e2d40';
      ctx.lineWidth   = 0.5;
      const step = size.x / 4;
      for (let i = 0; i <= 4; i++) {
        ctx.beginPath(); ctx.moveTo(i*step, 0);     ctx.lineTo(i*step, size.y); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, i*step);     ctx.lineTo(size.x, i*step); ctx.stroke();
      }
      ctx.fillStyle = '#2a3f5a';
      ctx.font      = '9px monospace';
      ctx.fillText(`${coords.z}/${coords.x}/${coords.y}`, 4, 12);
      return tile;
    }
  });
  new GridLayer({ opacity: 1, zIndex: 1 }).addTo(window.map);
  showGpsToast('Offline mode — grid map active', 'info');
}

// ═══════════════════════════════════════════════════════════════════════════════
// 12. TOAST
// ═══════════════════════════════════════════════════════════════════════════════
function showGpsToast(msg, type = 'info') {
  if (typeof window.showToast === 'function') { window.showToast(msg, type); return; }
  let c = document.getElementById('gps-toast-container');
  if (!c) {
    c = document.createElement('div');
    c.id = 'gps-toast-container';
    c.style.cssText = 'position:fixed;top:16px;right:16px;z-index:9999;display:flex;flex-direction:column;gap:8px;pointer-events:none;';
    document.body.appendChild(c);
  }
  const cols = { success:'#00c853', warning:'#ffab00', danger:'#f44336', info:'#00e5ff' };
  const col  = cols[type] ?? cols.info;
  const t    = document.createElement('div');
  t.textContent = msg;
  t.style.cssText = `
    background:${col}18;border-left:3px solid ${col};color:#dde6f0;
    padding:9px 14px;border-radius:3px;font-size:12px;font-family:monospace;
    box-shadow:0 2px 12px rgba(0,0,0,.45);backdrop-filter:blur(6px);
    animation:gpsToastIn .2s ease;
  `;
  c.appendChild(t);
  setTimeout(() => t.remove(), 4500);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 13. INIT
// ═══════════════════════════════════════════════════════════════════════════════
function initGPSModule() {
  initMapMarkers();
  initOfflineTiles();
  startFixWatchdog();

  document.getElementById('btn-track')?.addEventListener('click', toggleTrack);
  document.getElementById('btn-set-home')?.addEventListener('click', setHomePosition);

  console.log('[GPS] Module ready — NEO-6M @ 5 Hz, GPS-only constellation');
}

// ─── Public API ───────────────────────────────────────────────────────────────
window.BDGps = {
  init:                   initGPSModule,
  updateDroneGPS,
  updatePilotPosition,
  setHomePosition,
  toggleTrack,
  distanceBetweenPoints,
  bearingBetweenPoints,
  getState: ()            => gpsState,
};
