import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

export class PairingDialog extends LitElement {
  static properties = { candidates: { type: Array }, busy: { type: Boolean } };
  static styles = [componentStyles, css`
    dialog { width: min(34rem, calc(100% - 1.5rem)); margin: auto; padding: 0; border: 1px solid var(--app-border); border-radius: var(--app-radius); background: var(--app-panel); color: var(--app-text); box-shadow: 0 2rem 6rem rgba(0, 0, 0, 0.45); }
    dialog::backdrop { background: rgba(2, 8, 15, 0.72); backdrop-filter: blur(4px); }
    header, .candidate { display: flex; align-items: center; }
    header { justify-content: space-between; padding: 0.9rem 1rem; border-bottom: 1px solid var(--app-border); }
    h2, p { margin: 0; } h2 { font-size: 1.05rem; }
    .list { display: grid; gap: 0.55rem; max-height: 60dvh; padding: 1rem; overflow: auto; }
    .candidate { width: 100%; justify-content: space-between; gap: 1rem; margin: 0; padding: 0.75rem; border: 1px solid var(--app-border); border-radius: 0.75rem; background: var(--app-panel-soft); color: var(--app-text); text-align: left; cursor: pointer; }
    .candidate:hover:not(:disabled) { border-color: var(--app-accent); } .candidate:disabled { opacity: 0.45; }
    .identity { display: grid; min-width: 0; gap: 0.15rem; }
    strong, small { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
    small, .empty { color: var(--app-muted); }
    .signal { display: flex; flex: 0 0 auto; align-items: center; gap: 0.35rem; color: var(--app-muted); }
    .empty { padding: 2.5rem 1rem; text-align: center; }
  `];

  constructor() { super(); this.candidates = []; this.busy = false; }
  get open() { return Boolean(this.renderRoot.querySelector("dialog")?.open); }
  show() { const dialog = this.renderRoot.querySelector("dialog"); if (!dialog.open) { dialog.showModal(); } }
  close({ dismissed = true } = {}) {
    this.renderRoot.querySelector("dialog")?.close();
    this.dispatchEvent(new CustomEvent("pairing-close", { detail: { dismissed }, bubbles: true, composed: true }));
  }
  select(candidate) { this.dispatchEvent(new CustomEvent("pairing-select", { detail: { candidate }, bubbles: true, composed: true })); }

  render() {
    return html`
      <dialog aria-labelledby="pair-title" @cancel=${this.close}>
        <header><h2 id="pair-title">Nearby hearing aids</h2><button class="icon-button" type="button" aria-label="Close pairing" title="Close" @click=${this.close}>${icon("close")}</button></header>
        ${this.candidates.length ? html`<div class="list">${this.candidates.map((candidate) => html`<button class="candidate" type="button" ?disabled=${this.busy} @click=${() => this.select(candidate)} aria-label=${`Pair ${candidate.name || candidate.address}`}><span class="identity"><strong>${candidate.name || "Unnamed hearing aid"}</strong><small>${candidate.address}</small></span><span class="signal">${icon("pairing")}${candidate.rssi} dBm</span></button>`)}</div>` : html`<p class="empty">No hearing-aid advertisements received yet.</p>`}
      </dialog>
    `;
  }
}

customElements.define("pairing-dialog", PairingDialog);
