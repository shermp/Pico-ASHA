import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

export class ToastMessage extends LitElement {
  static properties = { message: { type: String }, kind: { type: String }, visible: { type: Boolean, reflect: true } };
  static styles = [componentStyles, css`
    :host {
      position: fixed;
      z-index: 30;
      bottom: max(0.8rem, env(safe-area-inset-bottom));
      left: 50%;
      display: none;
      width: min(27.2rem, calc(100% - 1.6rem));
      transform: translateX(-50%);
    }

    :host([visible]) {
      display: block;
    }

    .toast {
      display: flex;
      align-items: center;
      gap: 0.48rem;
      padding: 0.64rem 0.72rem;
    }

    .toast.error {
      border-color: color-mix(in srgb, var(--app-danger) 60%, var(--app-border));
      background: color-mix(in srgb, var(--app-danger) 16%, var(--app-panel));
    }

    .toast.warning {
      border-color: color-mix(in srgb, var(--app-warning) 60%, var(--app-border));
      background: color-mix(in srgb, var(--app-warning) 16%, var(--app-panel));
    }

    .toast.error > .material-symbols-outlined {
      color: var(--app-danger);
    }

    .toast.warning > .material-symbols-outlined {
      color: var(--app-warning);
    }

    p {
      flex: 1;
      margin: 0;
      font-size: 0.672rem;
    }

    button {
      flex: 0 0 auto;
    }
  `];

  constructor() { super(); this.message = ""; this.kind = "info"; this.visible = false; }
  render() { return html`<div class="toast panel ${this.kind}" role=${this.kind === "error" ? "alert" : "status"}>${icon(this.kind === "error" ? "error" : this.kind === "warning" ? "warning" : "info")}<p>${this.message}</p><button class="icon-button" type="button" aria-label="Dismiss message" title="Dismiss" @click=${() => { this.visible = false; }}>${icon("close")}</button></div>`; }
}

customElements.define("toast-message", ToastMessage);
