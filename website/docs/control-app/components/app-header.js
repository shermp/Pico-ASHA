import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

export class AppHeader extends LitElement {
  static properties = {
    connection: { type: Object },
    uacVersion: { type: Number },
    candidateCount: { type: Number },
    busy: { type: Boolean },
  };

  static styles = [
    componentStyles,
    css`
      header {
        display: flex;
        min-height: 3.4rem;
        align-items: center;
        justify-content: space-between;
        gap: 0.8rem;
        padding: 0.576rem 0.8rem;
      }

      .brand,
      .actions {
        display: flex;
        align-items: center;
        gap: 0.6rem;
      }

      .pairing-button {
        position: relative;
      }

      .notification-badge {
        position: absolute;
        top: -0.28rem;
        right: -0.28rem;
        display: grid;
        min-width: 0.92rem;
        height: 0.92rem;
        padding: 0 0.176rem;
        place-items: center;
        border: 2px solid var(--app-panel);
        border-radius: 999px;
        background: var(--app-warning);
        color: #1c1306;
        font-size: 0.48rem;
        font-weight: 700;
        line-height: 1;
      }

      .mark {
        display: grid;
        width: 1.92rem;
        height: 1.92rem;
        place-items: center;
        border-radius: var(--app-control-radius, 0.64rem);
        background: var(--app-brand-background, var(--app-accent));
        color: var(--app-brand-foreground, #fff);
      }

      h1,
      p {
        margin: 0;
      }

      h1 {
        font-size: 0.76rem;
        letter-spacing: 0.02em;
      }

      p {
        margin-top: 0.12rem;
        color: var(--app-muted);
        font-size: 0.6rem;
      }

      .status {
        display: flex;
        align-items: center;
        gap: 0.36rem;
        color: var(--app-muted);
        font-size: 0.6rem;
      }

      .dot {
        width: 0.44rem;
        height: 0.44rem;
        border-radius: 50%;
        background: var(--app-muted);
      }

      .dot.ready {
        background: var(--app-success);
        box-shadow: 0 0 0.56rem color-mix(in srgb, var(--app-success) 65%, transparent);
      }

      @media (max-width: 28.8rem) {
        .status span:last-child {
          display: none;
        }
      }
    `,
  ];

  constructor() {
    super();
    this.connection = { phase: "idle", label: "Adapter disconnected" };
    this.uacVersion = null;
    this.candidateCount = 0;
    this.busy = false;
  }

  emit(name) {
    this.dispatchEvent(new CustomEvent(name, { bubbles: true, composed: true }));
  }

  render() {
    const ready = this.connection.phase === "ready";
    const canDisconnect = ready || this.connection.phase === "reconnecting";
    const busy = ["connecting", "disconnecting"].includes(this.connection.phase);
    const uacLabel = ready && Number.isFinite(this.uacVersion) ? ` · UAC${this.uacVersion}` : "";
    const pairingLabel = this.candidateCount ? `Pair device (${this.candidateCount} nearby)` : "Pair device";
    // "pairing" is not a valid bundled ligature; use the included Bluetooth glyph instead of fallback text.
    return html`
      <header class="panel">
        <div class="brand">
          <span class="mark">${icon("hearing")}</span>
          <div>
            <h1>Pico-ASHA</h1>
            <p>Hearing aid bridge</p>
          </div>
        </div>
        <div class="actions">
          <div class="status" role="status" aria-live="polite">
            <span class="dot ${ready ? "ready" : ""}"></span>
            <span>${this.connection.label}${uacLabel}</span>
          </div>
          <button
            class="icon-button"
            type="button"
            aria-label=${canDisconnect ? "Disconnect adapter" : "Connect adapter"}
            title=${canDisconnect ? "Disconnect adapter" : "Connect adapter"}
            ?disabled=${busy}
            @click=${() => this.emit(canDisconnect ? "adapter-disconnect" : "adapter-connect")}
          >
            ${icon(canDisconnect ? "link_off" : "cable")}
          </button>
          <button
            class="icon-button pairing-button"
            type="button"
            aria-label=${pairingLabel}
            title=${pairingLabel}
            ?disabled=${!ready || this.busy}
            @click=${() => this.emit("pairing-open")}
          >
            ${icon("bluetooth_searching")}
            ${this.candidateCount ? html`<span class="notification-badge" aria-hidden="true">${this.candidateCount > 99 ? "99+" : this.candidateCount}</span>` : ""}
          </button>
          <button class="icon-button" type="button" aria-label="Open settings" title="Settings" @click=${() => this.emit("settings-open")}>
            ${icon("settings")}
          </button>
        </div>
      </header>
    `;
  }
}

customElements.define("app-header", AppHeader);
