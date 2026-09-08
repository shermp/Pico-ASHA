import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

export class AppHeader extends LitElement {
  static properties = {
    connection: { type: Object },
    uacVersion: { type: Number },
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
  }

  emit(name) {
    this.dispatchEvent(new CustomEvent(name, { bubbles: true, composed: true }));
  }

  render() {
    const ready = this.connection.phase === "ready";
    const canDisconnect = ready || this.connection.phase === "reconnecting";
    const busy = ["connecting", "disconnecting"].includes(this.connection.phase);
    const uacLabel = ready && Number.isFinite(this.uacVersion) ? ` · UAC${this.uacVersion}` : "";
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
          <button class="icon-button" type="button" aria-label="Open settings" title="Settings" @click=${() => this.emit("settings-open")}>
            ${icon("settings")}
          </button>
        </div>
      </header>
    `;
  }
}

customElements.define("app-header", AppHeader);
