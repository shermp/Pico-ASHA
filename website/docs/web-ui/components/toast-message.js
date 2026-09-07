import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

export class ToastMessage extends LitElement {
  static properties = { message: { type: String }, kind: { type: String }, visible: { type: Boolean, reflect: true } };
  static styles = [componentStyles, css`
    :host { position: fixed; z-index: 30; top: 1rem; left: 50%; display: none; width: min(34rem, calc(100% - 2rem)); transform: translateX(-50%); }
    :host([visible]) { display: block; }
    .toast { display: flex; align-items: flex-start; gap: 0.6rem; padding: 0.8rem 0.9rem; }
    .toast.error { border-color: color-mix(in srgb, var(--app-danger) 60%, var(--app-border)); }
    .toast.warning { border-color: color-mix(in srgb, var(--app-warning) 60%, var(--app-border)); }
    p { flex: 1; margin: 0; font-size: 0.84rem; } button { flex: 0 0 auto; }
  `];

  constructor() { super(); this.message = ""; this.kind = "info"; this.visible = false; }
  render() { return html`<div class="toast panel ${this.kind}" role=${this.kind === "error" ? "alert" : "status"}>${icon(this.kind === "error" ? "warning" : "info")}<p>${this.message}</p><button class="icon-button" type="button" aria-label="Dismiss message" title="Dismiss" @click=${() => { this.visible = false; }}>${icon("close")}</button></div>`; }
}

customElements.define("toast-message", ToastMessage);
