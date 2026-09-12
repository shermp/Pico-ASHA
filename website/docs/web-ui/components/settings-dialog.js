import { css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { DialogElement, dialogStyles } from "./dialog-element.js";
import { icon } from "./icon.js";

export class SettingsDialog extends DialogElement {
  static properties = {
    ready: { type: Boolean }, intro: { type: Object }, usbInfo: { type: Object }, remotes: { type: Array },
    hci: { type: Object }, busy: { type: Boolean },
    uacVersion: { state: true }, minimumDb: { state: true }, maximumDb: { state: true },
  };

  static styles = [componentStyles, dialogStyles, css`
    dialog {
      width: min(43rem, calc(100% - 1.5rem));
      max-height: calc(100dvh - 2rem);
      overflow: hidden;
    }

    .section-title,
    .button-row,
    .field-grid,
    .unpair-row,
    .range-heading {
      display: flex;
      align-items: center;
    }

    h2,
    h3,
    p {
      margin: 0;
    }

    h3 {
      font-size: 0.9rem;
    }

    .content {
      max-height: calc(100dvh - 5.5rem);
      padding: 1rem;
      overflow: auto;
    }

    section + section {
      margin-top: 1rem;
      padding-top: 1rem;
      border-top: 1px solid var(--app-border);
    }

    .section-title {
      gap: 0.5rem;
      margin-bottom: 0.7rem;
      color: var(--app-muted);
    }

    .button-row {
      flex-wrap: wrap;
      gap: 0.55rem;
    }

    button {
      min-height: 2.55rem;
      margin: 0;
      padding: 0.55rem 0.8rem;
      border: 1px solid var(--app-border);
      border-radius: 0.72rem;
      background: var(--app-panel-soft);
      color: var(--app-text);
      cursor: pointer;
    }

    button:hover:not(:disabled) {
      border-color: var(--app-accent);
      color: var(--app-accent-strong);
    }

    button:disabled {
      cursor: not-allowed;
      opacity: 0.42;
    }

    button.danger {
      color: var(--app-danger);
    }

    button.primary {
      border-color: var(--app-accent);
      background: color-mix(in srgb, var(--app-accent) 22%, var(--app-panel));
    }

    .control-button {
      display: inline-grid;
      place-items: center;
      width: 2.55rem;
      padding: 0;
    }

    .field-grid {
      align-items: end;
      gap: 0.65rem;
    }

    label {
      flex: 1 1 8rem;
      color: var(--app-muted);
      font-size: 0.78rem;
    }

    select {
      width: 100%;
      height: 2.55rem;
      margin-top: 0.25rem;
      padding: 0.45rem 0.6rem;
      border: 1px solid var(--app-border);
      border-radius: 0.6rem;
      background: var(--app-bg);
      color: var(--app-text);
    }

    .range-field {
      flex: 2 1 18rem;
      min-width: 14rem;
      margin: 0;
      padding: 0;
      border: 0;
    }

    .range-heading {
      width: 100%;
      justify-content: space-between;
      gap: 1rem;
      color: var(--app-muted);
      font-size: 0.78rem;
    }

    .range-values {
      color: var(--app-text);
      font-variant-numeric: tabular-nums;
      white-space: nowrap;
    }

    .range-control {
      position: relative;
      height: 2.55rem;
      margin-top: 0.25rem;
    }

    .range-track {
      position: absolute;
      top: 50%;
      right: 0.55rem;
      left: 0.55rem;
      height: 0.35rem;
      transform: translateY(-50%);
      border-radius: 999px;
      background: linear-gradient(to right, var(--app-border) 0 var(--range-min), var(--app-accent) var(--range-min) var(--range-max), var(--app-border) var(--range-max) 100%);
    }

    input[type="range"] {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 2.55rem;
      margin: 0;
      padding: 0;
      border: 0;
      background: transparent;
      pointer-events: none;
      appearance: none;
    }

    input[type="range"]::-webkit-slider-runnable-track {
      height: 0.35rem;
      background: transparent;
    }

    input[type="range"]::-webkit-slider-thumb {
      width: 1.1rem;
      height: 1.1rem;
      margin-top: -0.375rem;
      border: 2px solid var(--app-panel);
      border-radius: 50%;
      background: var(--app-accent-strong);
      box-shadow: 0 0 0 1px var(--app-accent);
      pointer-events: auto;
      appearance: none;
      cursor: grab;
    }

    input[type="range"]::-moz-range-track {
      height: 0.35rem;
      background: transparent;
    }

    input[type="range"]::-moz-range-thumb {
      width: 1.1rem;
      height: 1.1rem;
      border: 2px solid var(--app-panel);
      border-radius: 50%;
      background: var(--app-accent-strong);
      box-shadow: 0 0 0 1px var(--app-accent);
      pointer-events: auto;
      cursor: grab;
    }

    input[type="range"]:disabled::-webkit-slider-thumb {
      cursor: not-allowed;
      opacity: 0.42;
    }

    input[type="range"]:disabled::-moz-range-thumb {
      cursor: not-allowed;
      opacity: 0.42;
    }

    .unpair-row {
      justify-content: space-between;
      gap: 0.75rem;
      padding: 0.5rem 0;
      font-size: 0.84rem;
    }

    .unpair-row span {
      min-width: 0;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
    }

    .unpair-row button {
      flex: 0 0 auto;
    }

    .hint {
      margin-top: 0.55rem;
      color: var(--app-muted);
      font-size: 0.76rem;
    }

    @media (max-width: 35rem) {
      .field-grid {
        align-items: stretch;
        flex-direction: column;
      }

      .field-grid label {
        width: 100%;
        flex: 0 0 auto;
      }

      .range-field {
        width: 100%;
        min-width: 0;
        flex: 0 0 auto;
      }

      .field-grid button {
        align-self: end;
      }
    }
  `];

  constructor() {
    super();
    this.ready = false; this.intro = null; this.usbInfo = null; this.remotes = [];
    this.hci = { phase: "idle", bytes: 0, downloadAvailable: false }; this.busy = false;
    this.uacVersion = 2; this.minimumDb = -60; this.maximumDb = 0;
  }

  willUpdate(changed) {
    if (changed.has("usbInfo") && this.usbInfo) {
      // A device report is authoritative; replace any local draft after the adapter publishes new settings.
      this.uacVersion = this.usbInfo.uacVersion;
      this.minimumDb = this.usbInfo.minimumDb;
      this.maximumDb = this.usbInfo.maximumDb;
    }
  }

  emit(name, detail = undefined) { this.dispatchEvent(new CustomEvent(name, { detail, bubbles: true, composed: true })); }

  submitUSB(event) {
    event.preventDefault();
    this.emit("usb-update", { uacVersion: Number(this.uacVersion), minimumDb: Number(this.minimumDb), maximumDb: Number(this.maximumDb) });
  }

  updateMinimum(event) {
    // Clamp only the moved thumb, preserving at least one step between the two slider values.
    const minimumDb = Math.min(Number(event.target.value), Number(this.maximumDb) - 1);
    event.target.value = String(minimumDb);
    this.minimumDb = minimumDb;
  }

  updateMaximum(event) {
    // Do not move the lower thumb while enforcing the same ordered, one-step range.
    const maximumDb = Math.max(Number(event.target.value), Number(this.minimumDb) + 1);
    event.target.value = String(maximumDb);
    this.maximumDb = maximumDb;
  }

  render() {
    const audioEnabled = this.intro?.audioStreamingEnabled ?? false;
    const connectionsAllowed = this.intro?.connectionsAllowed ?? false;
    const hciActive = ["starting", "capturing", "stopping"].includes(this.hci.phase);
    const minimumPercent = ((Number(this.minimumDb) + 127) / 127) * 100;
    const maximumPercent = ((Number(this.maximumDb) + 127) / 127) * 100;
    // Saving restarts the adapter, so only enable it when this draft differs from its last device report.
    const usbChanged = Boolean(this.usbInfo) && (
      Number(this.uacVersion) !== Number(this.usbInfo.uacVersion)
      || Number(this.minimumDb) !== Number(this.usbInfo.minimumDb)
      || Number(this.maximumDb) !== Number(this.usbInfo.maximumDb)
    );
    return html`
      <dialog aria-labelledby="settings-title" @cancel=${() => this.close()}>
        <header><h2 id="settings-title">Adapter settings</h2><button class="icon-button" type="button" aria-label="Close settings" title="Close" @click=${this.close}>${icon("close")}</button></header>
        <div class="content">
          <section>
            <div class="section-title">${icon("graphic_eq")}<h3>Adapter controls</h3></div>
            <div class="button-row">
              <button class="control-button" type="button" aria-label=${audioEnabled ? "Stop audio" : "Start audio"} title=${audioEnabled ? "Stop audio" : "Start audio"} ?disabled=${!this.ready || this.busy} @click=${() => this.emit("audio-change", { enabled: !audioEnabled })}>${icon(audioEnabled ? "stop" : "play_arrow")}</button>
              <button class="control-button" type="button" aria-label=${connectionsAllowed ? "Disable connections" : "Enable connections"} title=${connectionsAllowed ? "Disable connections" : "Enable connections"} ?disabled=${!this.ready || this.busy} @click=${() => this.emit("connection-change", { enabled: !connectionsAllowed })}>${icon(connectionsAllowed ? "link_off" : "cable")}</button>
              <button class="control-button danger" type="button" aria-label="Restart adapter" title="Restart adapter" ?disabled=${!this.ready || this.busy} @click=${() => this.emit("adapter-restart")}>${icon("restart_alt")}</button>
            </div>
          </section>
          <section>
            <div class="section-title">${icon("usb")}<h3>USB audio</h3></div>
            <form @submit=${this.submitUSB}>
              <div class="field-grid">
                <label>USB Audio Class<select .value=${String(this.uacVersion)} @change=${(event) => { this.uacVersion = Number(event.target.value); }} ?disabled=${!this.ready || this.busy}><option value="1">UAC1</option><option value="2">UAC2</option></select></label>
                <fieldset class="range-field">
                  <legend class="range-heading"><span>Volume range</span><span class="range-values"><output>${this.minimumDb}</output> – <output>${this.maximumDb}</output></span></legend>
                  <div class="range-control" style=${`--range-min: ${minimumPercent}%; --range-max: ${maximumPercent}%`}>
                    <span class="range-track" aria-hidden="true"></span>
                    <input type="range" min="-127" max="0" step="1" .value=${String(this.minimumDb)} aria-label="Minimum volume" .ariaValueText=${String(this.minimumDb)} @input=${this.updateMinimum} ?disabled=${!this.ready || this.busy}>
                    <input type="range" min="-127" max="0" step="1" .value=${String(this.maximumDb)} aria-label="Maximum volume" .ariaValueText=${String(this.maximumDb)} @input=${this.updateMaximum} ?disabled=${!this.ready || this.busy}>
                  </div>
                </fieldset>
                <button class="primary control-button" type="submit" aria-label="Save USB settings" title="Save USB settings" ?disabled=${!this.ready || this.busy || !usbChanged}>${icon("save")}</button>
              </div>
              <p class="hint">Saving changed USB settings restarts the adapter.</p>
            </form>
          </section>
          <section>
            <div class="section-title">${icon("terminal")}<h3>HCI capture</h3></div>
            <div class="button-row">
              <button class="control-button" type="button" aria-label="Start HCI capture" title="Start HCI capture" ?disabled=${!this.ready || this.busy || hciActive} @click=${() => this.emit("hci-start")}>${icon("play_arrow")}</button>
              <button class="control-button" type="button" aria-label="Stop and download HCI capture" title="Stop and download HCI capture" ?disabled=${!this.ready || this.busy || !hciActive || this.hci.phase === "stopping"} @click=${() => this.emit("hci-stop")}>${icon("stop")}</button>
              <button class="control-button" type="button" aria-label="Download HCI capture again" title="Download HCI capture again" ?disabled=${!this.hci.downloadAvailable} @click=${() => this.emit("hci-download")}>${icon("download")}</button>
            </div>
            <p class="hint">${hciActive ? `${(this.hci.bytes / 1048576).toFixed(2)} MiB buffered. Refreshing or closing loses an unfinished capture.` : this.hci.downloadAvailable ? "Completed capture is ready to download." : "Capture is stored in this page up to 64 MiB."}</p>
          </section>
          <section>
            <div class="section-title">${icon("delete")}<h3>Paired devices</h3></div>
            ${this.remotes.length ? this.remotes.map((remote) => html`<div class="unpair-row"><span>${remote.name || remote.address} · ${remote.side}</span><button class="control-button danger" type="button" ?disabled=${!this.ready || this.busy || !remote.paired} @click=${() => this.emit("remote-unpair", { remote })} aria-label=${`Unpair ${remote.name || remote.address}`} title=${`Unpair ${remote.name || remote.address}`}>${icon("delete")}</button></div>`) : html`<p class="hint">No connected hearing aids.</p>`}
          </section>
        </div>
      </dialog>
    `;
  }
}

customElements.define("settings-dialog", SettingsDialog);
