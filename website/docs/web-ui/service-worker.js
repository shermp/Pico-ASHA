const CACHE_PREFIX = "pico-asha-control-";
const CACHE_NAME = `${CACHE_PREFIX}v1`;
const INDEX_URL = new URL("./index.html", self.location).href;

const APP_SHELL = [
  "./",
  "./index.html",
  "./app.js",
  "./styles.css",
  "./manifest.webmanifest",
  "./icons/icon.svg",
  "./icons/maskable-icon.svg",
  "./vendor/lit-core.min.js",
  "./vendor/material-symbols.css",
  "./vendor/material-symbols-outlined.woff2",
  "./components/adapter-controls.js",
  "./components/adapter-log.js",
  "./components/app-header.js",
  "./components/app-shell.js",
  "./components/component-styles.js",
  "./components/dialog-element.js",
  "./components/hci-capture-controls.js",
  "./components/icon.js",
  "./components/pairing-dialog.js",
  "./components/remote-card.js",
  "./components/remote-grid.js",
  "./components/settings-dialog.js",
  "./components/toast-message.js",
  "./protocol/cobs.js",
  "./protocol/codec.js",
  "./protocol/command-responses.js",
  "./protocol/constants.js",
  "./protocol/errors.js",
  "./protocol/state.js",
  "./serial/hci-capture.js",
  "./serial/serial-controller.js",
];

self.addEventListener("install", (event) => {
  event.waitUntil((async () => {
    const cache = await caches.open(CACHE_NAME);
    await cache.addAll(APP_SHELL);
    await self.skipWaiting();
  })());
});

self.addEventListener("activate", (event) => {
  event.waitUntil((async () => {
    const cacheNames = await caches.keys();
    await Promise.all(cacheNames
      .filter((name) => name.startsWith(CACHE_PREFIX) && name !== CACHE_NAME)
      .map((name) => caches.delete(name)));
    await self.clients.claim();
  })());
});

self.addEventListener("fetch", (event) => {
  const { request } = event;
  const url = new URL(request.url);

  if (request.method !== "GET" || url.origin !== self.location.origin) {
    return;
  }

  if (request.mode === "navigate") {
    event.respondWith((async () => {
      try {
        const response = await fetch(request);
        if (response.ok) {
          const cache = await caches.open(CACHE_NAME);
          await cache.put(request, response.clone());
        }
        return response;
      } catch (error) {
        const cached = await caches.match(INDEX_URL);
        if (cached) {
          return cached;
        }
        throw error;
      }
    })());
    return;
  }

  event.respondWith((async () => {
    try {
      const response = await fetch(request);
      if (response.ok) {
        const cache = await caches.open(CACHE_NAME);
        await cache.put(request, response.clone());
      }
      return response;
    } catch (error) {
      const cached = await caches.match(request);
      if (cached) {
        return cached;
      }
      throw error;
    }
  })());
});
