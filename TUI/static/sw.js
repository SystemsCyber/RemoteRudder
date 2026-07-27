// static/sw.js
const CACHE_VER = "telemetry-v1";
const PRECACHE = [
  "/",  // cache the shell HTML
  "/static/vendor/plotly-2.35.2.min.js",
  "/static/vendor/leaflet-1.9.4/leaflet.js",
  "/static/vendor/leaflet-1.9.4/leaflet.css",
  "/static/vendor/leaflet-1.9.4/images/marker-icon.png",
  "/static/vendor/leaflet-1.9.4/images/marker-icon-2x.png",
  "/static/vendor/leaflet-1.9.4/images/marker-shadow.png"
  // add your own /static/css/... or /static/js/... files if you split code later
];

self.addEventListener("install", (e) => {
  e.waitUntil(
    caches.open(CACHE_VER).then((cache) => cache.addAll(PRECACHE))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener("activate", (e) => {
  e.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(keys.map((k) => (k === CACHE_VER ? null : caches.delete(k))))
    ).then(() => self.clients.claim())
  );
});

// Strategy:
// - HTML: network-first (fall back to cache when offline)
// - Static assets (js/css/img): cache-first
// - OSM tiles: cache-first (optional; only tiles you've visited will be available offline)
self.addEventListener("fetch", (e) => {
  const req = e.request;
  const url = new URL(req.url);

  // Only handle GET
  if (req.method !== "GET") return;

  // HTML pages
  if (req.destination === "document" || req.headers.get("accept")?.includes("text/html")) {
    e.respondWith(
      fetch(req).then((res) => {
        const resClone = res.clone();
        caches.open(CACHE_VER).then((c) => c.put(req, resClone));
        return res;
      }).catch(() => caches.match(req).then((m) => m || caches.match("/")))
    );
    return;
  }

  // Static assets: cache-first
  if (["script","style","image","font"].includes(req.destination) ||
      url.pathname.startsWith("/static/")) {
    e.respondWith(
      caches.match(req).then((cached) => cached || fetch(req).then((res) => {
        const resClone = res.clone();
        caches.open(CACHE_VER).then((c) => c.put(req, resClone));
        return res;
      }))
    );
    return;
  }

  // Optional: cache OSM tiles you touch during online sessions
  if (url.hostname.includes("tile.openstreetmap.org")) {
    e.respondWith(
      caches.match(req).then((cached) => cached || fetch(req).then((res) => {
        const resClone = res.clone();
        caches.open(CACHE_VER).then((c) => c.put(req, resClone));
        return res;
      }).catch(() => cached))
    );
    return;
  }
});
