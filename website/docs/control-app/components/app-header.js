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
        min-height: 4.25rem;
        align-items: center;
        justify-content: space-between;
        gap: 1rem;
        padding: 0.72rem 1rem;
      }

      .brand,
      .actions {
        display: flex;
        align-items: center;
        gap: 0.75rem;
      }

      .pairing-button {
        position: relative;
      }

      .notification-badge {
        position: absolute;
        top: -0.35rem;
        right: -0.35rem;
        display: grid;
        min-width: 1.15rem;
        height: 1.15rem;
        padding: 0 0.22rem;
        place-items: center;
        border: 2px solid var(--app-panel);
        border-radius: 999px;
        background: var(--app-warning);
        color: #1c1306;
        font-size: 0.64rem;
        font-weight: 700;
        line-height: 1;
      }

      .mark {
        display: grid;
        width: 2.55rem;
        height: 2.55rem;
        place-items: center;
        border-radius: 0.8rem;
        background: linear-gradient(145deg, var(--app-accent), #4887e8);
        color: #031318;
      }

      h1,
      p {
        margin: 0;
      }

      h1 {
        font-size: 1rem;
        letter-spacing: 0.02em;
      }

      p {
        margin-top: 0.15rem;
        color: var(--app-muted);
        font-size: 0.78rem;
      }

      .status {
        display: flex;
        align-items: center;
        gap: 0.45rem;
        color: var(--app-muted);
        font-size: 0.82rem;
      }

      .dot {
        width: 0.55rem;
        height: 0.55rem;
        border-radius: 50%;
        background: var(--app-muted);
      }

      .dot.ready {
        background: var(--app-success);
        box-shadow: 0 0 0.7rem color-mix(in srgb, var(--app-success) 65%, transparent);
      }

      @media (max-width: 36rem) {
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
