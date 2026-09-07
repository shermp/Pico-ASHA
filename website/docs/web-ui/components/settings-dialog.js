import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

export class SettingsDialog extends LitElement {
  static properties = {
    ready: { type: Boolean }, intro: { type: Object }, usbInfo: { type: Object }, remotes: { type: Array },
    candidateCount: { type: Number }, hci: { type: Object }, busy: { type: Boolean },
    uacVersion: { state: true }, minimumDb: { state: true }, maximumDb: { state: true },
  };

  static styles = [componentStyles, css`
    dialog { width: min(43rem, calc(100% - 1.5rem)); max-height: calc(100dvh - 2rem); margin: auto; padding: 0; overflow: hidden; border: 1px solid var(--app-border); border-radius: var(--app-radius); background: var(--app-panel); color: var(--app-text); box-shadow: 0 2rem 6rem rgba(0, 0, 0, 0.45); }
    dialog::backdrop { background: rgba(2, 8, 15, 0.72); backdrop-filter: blur(4px); }
    header, footer, .section-title, .button-row, .field-grid, .unpair-row { display: flex; align-items: center; }
    header { justify-content: space-between; padding: 0.9rem 1rem; border-bottom: 1px solid var(--app-border); }
    h2, h3, p { margin: 0; }
    h2 { font-size: 1.05rem; }
    h3 { font-size: 0.9rem; }
    .content { max-height: calc(100dvh - 10rem); padding: 1rem; overflow: auto; }
    section + section { margin-top: 1rem; padding-top: 1rem; border-top: 1px solid var(--app-border); }
    .section-title { gap: 0.5rem; margin-bottom: 0.7rem; color: var(--app-muted); }
    .button-row { flex-wrap: wrap; gap: 0.55rem; }
    button { min-height: 2.55rem; margin: 0; padding: 0.55rem 0.8rem; border: 1px solid var(--app-border); border-radius: 0.72rem; background: var(--app-panel-soft); color: var(--app-text); cursor: pointer; }
    button:hover:not(:disabled) { border-color: var(--app-accent); color: var(--app-accent-strong); }
    button:disabled { cursor: not-allowed; opacity: 0.42; }
    button.danger { color: var(--app-danger); }
    button.primary { border-color: var(--app-accent); background: color-mix(in srgb, var(--app-accent) 22%, var(--app-panel)); }
    .control-button { display: inline-flex; align-items: center; gap: 0.45rem; }
    .field-grid { align-items: end; gap: 0.65rem; }
    label { flex: 1 1 8rem; color: var(--app-muted); font-size: 0.78rem; }
    input, select { width: 100%; height: 2.55rem; margin-top: 0.25rem; padding: 0.45rem 0.6rem; border: 1px solid var(--app-border); border-radius: 0.6rem; background: var(--app-bg); color: var(--app-text); }
    .unpair-row { justify-content: space-between; gap: 0.75rem; padding: 0.5rem 0; font-size: 0.84rem; }
    .unpair-row span { min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    .unpair-row button { flex: 0 0 auto; }
    .hint { margin-top: 0.55rem; color: var(--app-muted); font-size: 0.76rem; }
    footer { justify-content: flex-end; padding: 0.75rem 1rem; border-top: 1px solid var(--app-border); }
    @media (max-width: 35rem) {
      .field-grid { align-items: stretch; flex-direction: column; }
      .field-grid label { width: 100%; flex: 0 0 auto; }
      .field-grid button { justify-content: center; width: 100%; }
    }
  `];

  constructor() {
    super();
    this.ready = false; this.intro = null; this.usbInfo = null; this.remotes = []; this.candidateCount = 0;
    this.hci = { phase: "idle", bytes: 0, downloadAvailable: false }; this.busy = false;
    this.uacVersion = 2; this.minimumDb = -60; this.maximumDb = 0;
  }

  willUpdate(changed) {
    if (changed.has("usbInfo") && this.usbInfo) {
      this.uacVersion = this.usbInfo.uacVersion;
      this.minimumDb = this.usbInfo.minimumDb;
      this.maximumDb = this.usbInfo.maximumDb;
    }
  }

  show() {
    const dialog = this.renderRoot.querySelector("dialog");
    if (!dialog.open) { dialog.showModal(); }
  }

  close() { this.renderRoot.querySelector("dialog")?.close(); }
  emit(name, detail = undefined) { this.dispatchEvent(new CustomEvent(name, { detail, bubbles: true, composed: true })); }

  submitUSB(event) {
    event.preventDefault();
    this.emit("usb-update", { uacVersion: Number(this.uacVersion), minimumDb: Number(this.minimumDb), maximumDb: Number(this.maximumDb) });
  }

  render() {
    const audioEnabled = this.intro?.audioStreamingEnabled ?? false;
    const connectionsAllowed = this.intro?.connectionsAllowed ?? false;
    const hciActive = ["starting", "capturing", "stopping"].includes(this.hci.phase);
    return html`
      <dialog aria-labelledby="settings-title" @cancel=${() => this.close()}>
        <header><h2 id="settings-title">Adapter settings</h2><button class="icon-button" type="button" aria-label="Close settings" title="Close" @click=${this.close}>${icon("close")}</button></header>
        <div class="content">
          <section>
            <div class="section-title">${icon("graphic_eq")}<h3>Adapter controls</h3></div>
            <div class="button-row">
              <button class="control-button" type="button" ?disabled=${!this.ready || this.busy} @click=${() => this.emit("audio-change", { enabled: !audioEnabled })}>${icon(audioEnabled ? "stop" : "play_arrow")}${audioEnabled ? "Stop audio" : "Start audio"}</button>
              <button class="control-button" type="button" ?disabled=${!this.ready || this.busy} @click=${() => this.emit("connection-change", { enabled: !connectionsAllowed })}>${icon(connectionsAllowed ? "link_off" : "cable")}${connectionsAllowed ? "Disable connections" : "Enable connections"}</button>
              <button class="control-button" type="button" ?disabled=${!this.ready || this.busy} @click=${() => this.emit("pairing-open")}>${icon("pairing")}Pair device${this.candidateCount ? ` (${this.candidateCount})` : ""}</button>
              <button class="control-button danger" type="button" ?disabled=${!this.ready || this.busy} @click=${() => this.emit("adapter-restart")}>${icon("restart_alt")}Restart</button>
            </div>
          </section>
          <section>
            <div class="section-title">${icon("usb")}<h3>USB audio</h3></div>
            <form @submit=${this.submitUSB}>
              <div class="field-grid">
                <label>USB Audio Class<select .value=${String(this.uacVersion)} @change=${(event) => { this.uacVersion = Number(event.target.value); }} ?disabled=${!this.ready || this.busy}><option value="1">UAC1</option><option value="2">UAC2</option></select></label>
                <label>Minimum volume (dB)<input type="number" min="-127" max="0" step="1" .value=${String(this.minimumDb)} @input=${(event) => { this.minimumDb = event.target.value; }} ?disabled=${!this.ready || this.busy}></label>
                <label>Maximum volume (dB)<input type="number" min="-127" max="0" step="1" .value=${String(this.maximumDb)} @input=${(event) => { this.maximumDb = event.target.value; }} ?disabled=${!this.ready || this.busy}></label>
                <button class="primary control-button" type="submit" ?disabled=${!this.ready || this.busy}>${icon("save")}Save</button>
              </div>
              <p class="hint">Saving changed USB settings restarts the adapter.</p>
            </form>
          </section>
          <section>
            <div class="section-title">${icon("terminal")}<h3>HCI capture</h3></div>
            <div class="button-row">
              <button class="control-button" type="button" ?disabled=${!this.ready || this.busy || hciActive} @click=${() => this.emit("hci-start")}>${icon("play_arrow")}Start capture</button>
              <button class="control-button" type="button" ?disabled=${!this.ready || this.busy || !hciActive || this.hci.phase === "stopping"} @click=${() => this.emit("hci-stop")}>${icon("stop")}Stop and download</button>
              <button class="control-button" type="button" ?disabled=${!this.hci.downloadAvailable} @click=${() => this.emit("hci-download")}>${icon("download")}Download again</button>
            </div>
            <p class="hint">${hciActive ? `${(this.hci.bytes / 1048576).toFixed(2)} MiB buffered. Refreshing or closing loses an unfinished capture.` : this.hci.downloadAvailable ? "Completed capture is ready to download." : "Capture is stored in this page up to 64 MiB."}</p>
          </section>
          <section>
            <div class="section-title">${icon("delete")}<h3>Paired devices</h3></div>
            ${this.remotes.length ? this.remotes.map((remote) => html`<div class="unpair-row"><span>${remote.name || remote.address} · ${remote.side}</span><button class="danger" type="button" ?disabled=${!this.ready || this.busy || !remote.paired} @click=${() => this.emit("remote-unpair", { remote })} aria-label=${`Unpair ${remote.name || remote.address}`}>Unpair</button></div>`) : html`<p class="hint">No connected hearing aids.</p>`}
          </section>
        </div>
        <footer><button type="button" @click=${this.close}>Done</button></footer>
      </dialog>
    `;
  }
}

customElements.define("settings-dialog", SettingsDialog);
