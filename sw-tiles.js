/**
 * sw-tiles.js — Ball Drone MK II Service Worker
 * Caches OSM map tiles for offline operation (ESP32-only WiFi).
 * Place at web root alongside index.html.
 */

const CACHE_NAME   = 'balldrone-tiles-v1';
const TILE_ORIGINS = [
  'https://tile.openstreetmap.org',
  'https://a.tile.openstreetmap.org',
  'https://b.tile.openstreetmap.org',
  'https://c.tile.openstreetmap.org',
];
const APP_SHELL = ['./', './index.html', './app.js', './gps_module.js', './styles.css'];

self.addEventListener('install',  e => { e.waitUntil(caches.open(CACHE_NAME).then(c => c.addAll(APP_SHELL))); self.skipWaiting(); });
self.addEventListener('activate', e => { e.waitUntil(caches.keys().then(keys => Promise.all(keys.filter(k => k !== CACHE_NAME).map(k => caches.delete(k))))); self.clients.claim(); });

self.addEventListener('fetch', event => {
  const isTile = TILE_ORIGINS.some(o => event.request.url.startsWith(o));
  if (isTile) {
    event.respondWith(
      caches.match(event.request).then(cached => {
        if (cached) return cached;
        return fetch(event.request).then(res => {
          if (res && res.status === 200) {
            const clone = res.clone();
            caches.open(CACHE_NAME).then(c => c.put(event.request, clone));
          }
          return res;
        }).catch(() => new Response(TRANSPARENT_PNG, { headers: { 'Content-Type': 'image/png' } }));
      })
    );
    return;
  }
  event.respondWith(
    fetch(event.request)
      .then(res => { const clone = res.clone(); caches.open(CACHE_NAME).then(c => c.put(event.request, clone)); return res; })
      .catch(() => caches.match(event.request))
  );
});

const TRANSPARENT_PNG = Uint8Array.from(atob(
  'iVBORw0KGgoAAAANSUhEUgAAAQAAAAEAAQMAAABmvDolAAAAA1BMVEUAAACnej3aAAAAAXRSTlMAQObYZg' +
  'AAAAJcEhZcwAACxMAAAsTAQCanBgAAAAGSURBVGje2yAAAAAASUVORK5CYII='
), c => c.charCodeAt(0)).buffer;
