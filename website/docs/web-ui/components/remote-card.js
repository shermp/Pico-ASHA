import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

// The firmware's signed volume value uses 3/8 dB increments.
const VOLUME_DB_STEP = 0.375;

export function volumeToDb(volume) {
  return volume * VOLUME_DB_STEP;
}

export function batteryIcon(level) {
  if (!Number.isFinite(level)) {
    return "battery_unknown";
  }
  const clampedLevel = Math.min(Math.max(Math.round(level), 0), 10);
  if (clampedLevel === 10) {
    return "battery_full";
  }
  return `battery_${Math.ceil((clampedLevel * 6) / 9)}_bar`;
}

export function batteryColor(level) {
  if (!Number.isFinite(level)) {
    return "unknown";
  }
  const clampedLevel = Math.min(Math.max(Math.round(level), 0), 10);
  if (clampedLevel <= 2) {
    return "low";
  }
  if (clampedLevel <= 5) {
    return "medium";
  }
  return "high";
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
        border: 1px solid transparent;
        border-radius: 50%;
        color: #ffffff;
        font-size: 1.3rem;
        font-weight: 700;
      }

      .aid.left {
        background: #1e70d1;
        border-color: #1e70d1;
      }

      .aid.right {
        background: #c83737;
        border-color: #c83737;
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

      .battery.low {
        color: var(--app-battery-low);
      }

      .battery {
        display: inline-grid;
        height: 1.25rem;
        place-items: center;
      }

      .battery.medium {
        color: #ba7a00;
      }

      .battery.high {
        color: var(--app-success);
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
    const sideInitial = this.side === "Right" ? "R" : "L";
    const sideClass = sideInitial === "L" ? "left" : "right";
    const volume = remote.muted ? "Muted" : `${volumeToDb(remote.volume)} dB`;
    const batteryGlyph = batteryIcon(remote.battery);
    const batteryTone = batteryColor(remote.battery);
    return html`
      <article class="panel">
        <div class="edge"></div>
        <div class="body">
          <div class="top">
            <div class="identity">
              <span class="aid ${sideClass}" role="img" aria-label="${this.side} hearing aid">${sideInitial}</span>
              <div>
                <h2>${remote.name || "Unnamed hearing aid"}</h2>
              </div>
            </div>
            <span class="connected" title="Connected">${icon("bluetooth_connected")}</span>
          </div>
          <div class="metrics">
            <div class="metric">${icon(remote.streaming ? "graphic_eq" : "stop")}<strong>${remote.streaming ? "Streaming" : "Idle"}</strong></div>
            <div class="metric">${icon(remote.muted ? "volume_off" : "volume_up")}<strong>${volume}</strong></div>
            <div class="metric"><span class="battery ${batteryTone}">${icon(batteryGlyph)}</span><strong>${remote.battery ?? "—"}${remote.battery == null ? "" : "/10"}</strong></div>
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
