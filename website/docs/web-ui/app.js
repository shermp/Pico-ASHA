import "./components/app-shell.js";

if ("serviceWorker" in navigator) {
  window.addEventListener("load", () => {
    const workerUrl = new URL("./service-worker.js", import.meta.url);
    void navigator.serviceWorker.register(workerUrl, {
      scope: "./",
      updateViaCache: "none",
    }).catch((error) => {
      console.warn("Pico-ASHA offline support could not be enabled.", error);
    });
  });
}
