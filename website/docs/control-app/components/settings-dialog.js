import { css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { DialogElement, dialogStyles } from "./dialog-element.js";
import { icon } from "./icon.js";

export class SettingsDialog extends DialogElement {
  static properties = {
    ready: { type: Boolean }, usbInfo: { type: Object }, remotes: { type: Array }, busy: { type: Boolean },
    uacVersion: { state: true }, minimumDb: { state: true }, maximumDb: { state: true },
  };

  static styles = [componentStyles, dialogStyles, css`
    dialog {
      width: min(34.4rem, calc(100% - 1.2rem));
      max-height: calc(100dvh - 1.6rem);
      overflow: hidden;
    }

    .section-title,
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
      font-size: 0.72rem;
    }

    .content {
      max-height: calc(100dvh - 4.4rem);
      padding: 0.8rem;
      overflow: auto;
    }

    section + section {
      margin-top: 0.8rem;
      padding-top: 0.8rem;
      border-top: 1px solid var(--app-border);
    }

    .section-title {
      gap: 0.4rem;
      margin-bottom: 0.56rem;
      color: var(--app-muted);
    }

    button {
      min-height: 2.04rem;
      margin: 0;
      padding: 0.44rem 0.64rem;
      border: 1px solid var(--app-border);
      border-radius: var(--app-control-radius, 0.576rem);
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

    .field-grid {
      align-items: end;
      gap: 0.52rem;
    }

    label {
      flex: 1 1 6.4rem;
      color: var(--app-muted);
      font-size: 0.624rem;
    }

    select {
      width: 100%;
      height: 2.04rem;
      margin-top: 0.2rem;
      padding: 0.36rem 0.48rem;
      border: 1px solid var(--app-border);
      border-radius: var(--app-control-radius, 0.48rem);
      background: var(--app-bg);
      color: var(--app-text);
    }

    .range-field {
      flex: 2 1 14.4rem;
      min-width: 11.2rem;
      margin: 0;
      padding: 0;
      border: 0;
    }

    .range-heading {
      width: 100%;
      justify-content: space-between;
      gap: 0.8rem;
      color: var(--app-muted);
      font-size: 0.624rem;
    }

    .range-values {
      color: var(--app-text);
      font-variant-numeric: tabular-nums;
      white-space: nowrap;
    }

    .range-control {
      position: relative;
      height: 2.04rem;
      margin-top: 0.2rem;
    }

    .range-track {
      position: absolute;
      top: 50%;
      right: 0.44rem;
      left: 0.44rem;
      height: 0.28rem;
      transform: translateY(-50%);
      border-radius: 999px;
      background: linear-gradient(to right, var(--app-border) 0 var(--range-min), var(--app-accent) var(--range-min) var(--range-max), var(--app-border) var(--range-max) 100%);
    }

    input[type="range"] {
      position: absolute;
      inset: 0;
      width: 100%;
      height: 2.04rem;
      margin: 0;
      padding: 0;
      border: 0;
      background: transparent;
      pointer-events: none;
      appearance: none;
    }

    input[type="range"]::-webkit-slider-runnable-track {
      height: 0.28rem;
      background: transparent;
    }

    input[type="range"]::-webkit-slider-thumb {
      width: 0.88rem;
      height: 0.88rem;
      margin-top: -0.3rem;
      border: 2px solid var(--app-panel);
      border-radius: 50%;
      background: var(--app-accent-strong);
      box-shadow: 0 0 0 1px var(--app-accent);
      pointer-events: auto;
      appearance: none;
      cursor: grab;
    }

    input[type="range"]::-moz-range-track {
      height: 0.28rem;
      background: transparent;
    }

    input[type="range"]::-moz-range-thumb {
      width: 0.88rem;
      height: 0.88rem;
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
      gap: 0.6rem;
      padding: 0.4rem 0;
      font-size: 0.672rem;
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
      margin-top: 0.44rem;
      color: var(--app-muted);
      font-size: 0.608rem;
    }

    @media (max-width: 28rem) {
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
    this.ready = false; this.usbInfo = null; this.remotes = []; this.busy = false;
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
                <button class="primary" type="submit" ?disabled=${!this.ready || this.busy || !usbChanged}>Save USB settings</button>
              </div>
              <p class="hint">Saving changed USB settings restarts the adapter.</p>
            </form>
          </section>
          <section>
            <div class="section-title">${icon("bluetooth_connected")}<h3>Paired devices</h3></div>
            ${this.remotes.length ? this.remotes.map((remote) => html`<div class="unpair-row"><span>${remote.name || remote.address} · ${remote.side}</span><button class="danger" type="button" ?disabled=${!this.ready || this.busy || !remote.paired} @click=${() => this.emit("remote-unpair", { remote })}>Unpair</button></div>`) : html`<p class="hint">No connected hearing aids.</p>`}
          </section>
        </div>
      </dialog>
    `;
  }
}

customElements.define("settings-dialog", SettingsDialog);
