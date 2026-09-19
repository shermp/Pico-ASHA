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
        height: 0.2rem;
        background: var(--app-card-edge, var(--app-accent));
      }

      .body {
        padding: 0.92rem;
      }

      .top,
      .metric,
      .empty {
        display: flex;
        align-items: center;
      }

      .top {
        justify-content: space-between;
        gap: 0.8rem;
      }

      .identity {
        display: flex;
        min-width: 0;
        align-items: center;
        gap: 0.64rem;
      }

      .aid {
        display: grid;
        width: 2rem;
        height: 2rem;
        flex: 0 0 auto;
        place-items: center;
        border: 1px solid transparent;
        border-radius: 50%;
        color: #ffffff;
        font-size: 0.88rem;
        font-weight: 700;
      }

      .aid.left {
        background: var(--app-left-aid, #4051b5);
        border-color: var(--app-left-aid, #4051b5);
      }

      .aid.right {
        background: var(--app-right-aid, #d52a2a);
        border-color: var(--app-right-aid, #d52a2a);
      }

      h2,
      p {
        margin: 0;
      }

      h2 {
        overflow: hidden;
        font-size: 0.76rem;
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
        gap: 0.44rem;
        margin-top: 0.92rem;
      }

      .metric {
        min-width: 0;
        flex-direction: column;
        gap: 0.24rem;
        padding: 0.576rem 0.32rem;
        border: 1px solid var(--app-border);
        border-radius: var(--app-control-radius, 0.576rem);
        background: var(--app-panel-soft);
        text-align: center;
      }

      .metric strong {
        max-width: 100%;
        overflow: hidden;
        font-size: 0.6rem;
        text-overflow: ellipsis;
      }

      .battery.low {
        color: var(--app-battery-low);
      }

      .battery {
        display: inline-grid;
        height: 0.96rem;
        place-items: center;
      }

      .battery.medium {
        color: var(--app-warning);
      }

      .battery.high {
        color: var(--app-success);
      }

      details {
        margin-top: 0.8rem;
        border-top: 1px solid var(--app-border);
        padding-top: 0.68rem;
      }

      summary {
        color: var(--app-muted);
        cursor: pointer;
        font-size: 0.6rem;
      }

      dl {
        display: grid;
        grid-template-columns: auto 1fr;
        gap: 0.28rem 0.64rem;
        margin: 0.64rem 0 0;
        font-size: 0.6rem;
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
        gap: 0.48rem;
        text-align: center;
      }

      .empty .material-symbols-outlined {
        color: var(--app-muted);
        font-size: 1.6rem;
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
