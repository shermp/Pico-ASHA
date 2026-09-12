import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

export class HciCaptureControls extends LitElement {
  static properties = {
    ready: { type: Boolean },
    hci: { type: Object },
    busy: { type: Boolean },
  };

  static styles = [componentStyles, css`
    section {
      padding: 1rem;
      text-align: center;
    }

    .title,
    .button-row {
      display: flex;
      align-items: center;
    }

    .title {
      justify-content: center;
      gap: 0.5rem;
      margin-bottom: 0.8rem;
      color: var(--app-muted);
    }

    h2,
    p {
      margin: 0;
    }

    h2 {
      color: var(--app-text);
      font-size: 0.9rem;
    }

    .button-row {
      justify-content: center;
      flex-wrap: wrap;
      gap: 0.55rem;
    }

    button {
      min-height: 2.55rem;
      margin: 0;
      padding: 0.55rem 0.8rem;
      border: 1px solid var(--app-border);
      border-radius: 0.72rem;
      background: var(--app-panel-soft);
      color: var(--app-text);
      cursor: pointer;
    }

    button:hover:not(:disabled) {
      border-color: var(--app-accent);
      color: var(--app-accent-strong);
    }

    button:disabled {
      cursor: not-allowed;
      opacity: 0.42;
    }

    button.primary {
      border-color: var(--app-accent);
      background: color-mix(in srgb, var(--app-accent) 22%, var(--app-panel));
    }

    p {
      margin-top: 0.65rem;
      color: var(--app-muted);
      font-size: 0.76rem;
    }
  `];

  constructor() {
    super();
    this.ready = false;
    this.hci = { phase: "idle", bytes: 0, downloadAvailable: false };
    this.busy = false;
  }

  emit(name) {
    this.dispatchEvent(new CustomEvent(name, { bubbles: true, composed: true }));
  }

  render() {
    const hciActive = ["starting", "capturing", "stopping"].includes(this.hci.phase);
    const hint = hciActive
      ? `${(this.hci.bytes / 1048576).toFixed(2)} MiB buffered. Refreshing or closing loses an unfinished capture.`
      : this.hci.downloadAvailable
        ? "Completed capture is ready to download."
        : "Capture is stored in this page up to 64 MiB.";
    return html`
      <section class="panel" aria-labelledby="hci-capture-title">
        <div class="title">${icon("terminal")}<h2 id="hci-capture-title">HCI capture</h2></div>
        <div class="button-row">
          <button class="primary" type="button" ?disabled=${!this.ready || this.busy || hciActive} @click=${() => this.emit("hci-start")}>Start capture</button>
          <button type="button" ?disabled=${!this.ready || this.busy || !hciActive || this.hci.phase === "stopping"} @click=${() => this.emit("hci-stop")}>Stop and download</button>
          <button type="button" ?disabled=${!this.hci.downloadAvailable} @click=${() => this.emit("hci-download")}>Download capture</button>
        </div>
        <p>${hint}</p>
      </section>
    `;
  }
}

customElements.define("hci-capture-controls", HciCaptureControls);
