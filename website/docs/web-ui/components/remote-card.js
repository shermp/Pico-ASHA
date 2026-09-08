import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

const VOLUME_DB_STEP = 0.375;

export function volumeToDb(volume) {
  return volume * VOLUME_DB_STEP;
}

export class RemoteCard extends LitElement {
  static properties = {
    remote: { type: Object },
    side: { type: String },
  };

  static styles = [
    componentStyles,
    css`
      article {
        min-height: 20rem;
        overflow: hidden;
      }

      .edge {
        height: 0.25rem;
        background: linear-gradient(90deg, var(--app-accent), #4e83e8);
      }

      .body {
        padding: 1.15rem;
      }

      .top,
      .metric,
      .empty {
        display: flex;
        align-items: center;
      }

      .top {
        justify-content: space-between;
        gap: 1rem;
      }

      .identity {
        display: flex;
        min-width: 0;
        align-items: center;
        gap: 0.8rem;
      }

      .aid {
        display: grid;
        width: 3.1rem;
        height: 3.1rem;
        flex: 0 0 auto;
        place-items: center;
        border: 1px solid var(--app-border);
        border-radius: 50%;
        background: var(--app-panel-soft);
        color: var(--app-accent);
      }

      h2,
      p {
        margin: 0;
      }

      h2 {
        overflow: hidden;
        font-size: 1.08rem;
        text-overflow: ellipsis;
        white-space: nowrap;
      }

      p,
      small {
        color: var(--app-muted);
      }

      .side {
        margin-top: 0.15rem;
        font-size: 0.78rem;
        letter-spacing: 0.08em;
        text-transform: uppercase;
      }

      .connected {
        color: var(--app-success);
      }

      .metrics {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        gap: 0.55rem;
        margin-top: 1.15rem;
      }

      .metric {
        min-width: 0;
        flex-direction: column;
        gap: 0.3rem;
        padding: 0.72rem 0.4rem;
        border: 1px solid var(--app-border);
        border-radius: 0.72rem;
        background: var(--app-panel-soft);
        text-align: center;
      }

      .metric strong {
        max-width: 100%;
        overflow: hidden;
        font-size: 0.85rem;
        text-overflow: ellipsis;
      }

      details {
        margin-top: 1rem;
        border-top: 1px solid var(--app-border);
        padding-top: 0.85rem;
      }

      summary {
        color: var(--app-muted);
        cursor: pointer;
        font-size: 0.85rem;
      }

      dl {
        display: grid;
        grid-template-columns: auto 1fr;
        gap: 0.35rem 0.8rem;
        margin: 0.8rem 0 0;
        font-size: 0.8rem;
      }

      dt {
        color: var(--app-muted);
      }

      dd {
        margin: 0;
        overflow-wrap: anywhere;
        text-align: right;
      }

      .empty {
        min-height: 17.5rem;
        flex-direction: column;
        justify-content: center;
        gap: 0.6rem;
        text-align: center;
      }

      .empty .material-symbols-outlined {
        color: var(--app-muted);
        font-size: 2.5rem;
      }
    `,
  ];

  constructor() {
    super();
    this.remote = null;
    this.side = "Left";
  }

  render() {
    if (!this.remote) {
      return html`
        <article class="panel">
          <div class="edge"></div>
          <div class="body empty">
            ${icon("hearing_disabled")}
            <strong>No ${this.side.toLowerCase()} hearing aid</strong>
            <small>Connect an adapter or pair a device.</small>
          </div>
        </article>
      `;
    }

    const remote = this.remote;
    const volume = remote.muted ? "Muted" : `${volumeToDb(remote.volume)} dB`;
    return html`
      <article class="panel">
        <div class="edge"></div>
        <div class="body">
          <div class="top">
            <div class="identity">
              <span class="aid">${icon("hearing")}</span>
              <div>
                <h2>${remote.name || "Unnamed hearing aid"}</h2>
                <p class="side">${this.side} channel</p>
              </div>
            </div>
            <span class="connected" title="Connected">${icon("bluetooth_connected")}</span>
          </div>
          <div class="metrics">
            <div class="metric">${icon(remote.streaming ? "graphic_eq" : "stop")}<strong>${remote.streaming ? "Streaming" : "Idle"}</strong></div>
            <div class="metric">${icon(remote.muted ? "volume_off" : "volume_up")}<strong>${volume}</strong></div>
            <div class="metric">${icon("battery_full")}<strong>${remote.battery ?? "—"}${remote.battery == null ? "" : "/10"}</strong></div>
          </div>
          <details>
            <summary>Technical details</summary>
            <dl>
              <dt>Make / model</dt><dd>${[remote.manufacturer, remote.model].filter(Boolean).join(" · ") || "—"}</dd>
              <dt>Bluetooth address</dt><dd>${remote.address || "—"}</dd>
              <dt>Firmware / software</dt><dd>${[remote.firmware, remote.software].filter(Boolean).join(" · ") || "—"}</dd>
              <dt>ASHA mode</dt><dd>${remote.mode || "—"}</dd>
              <dt>Audio</dt><dd>${remote.audioFormat || "—"}</dd>
              <dt>PSM / L2CAP CID</dt><dd>${remote.psm ?? "—"} / ${remote.l2capCid ?? "—"}</dd>
              <dt>Connection / HCI</dt><dd>${remote.connectionId ?? "—"} / ${remote.hciHandle ?? "—"}</dd>
              <dt>Bonded</dt><dd>${remote.paired ? "Yes" : "No"}</dd>
            </dl>
          </details>
        </div>
      </article>
    `;
  }
}

customElements.define("remote-card", RemoteCard);
