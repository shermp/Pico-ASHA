import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

export class AdapterControls extends LitElement {
  static properties = {
    ready: { type: Boolean },
    intro: { type: Object },
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

    button.danger {
      color: var(--app-danger);
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
    this.intro = null;
    this.busy = false;
  }

  emit(name, detail = undefined) {
    this.dispatchEvent(new CustomEvent(name, { detail, bubbles: true, composed: true }));
  }

  render() {
    const audioEnabled = this.intro?.audioStreamingEnabled ?? false;
    const connectionsAllowed = this.intro?.connectionsAllowed ?? false;
    return html`
      <section class="panel" aria-labelledby="adapter-controls-title">
        <div class="title">${icon("graphic_eq")}<h2 id="adapter-controls-title">Adapter controls</h2></div>
        <div class="button-row">
          <button type="button" ?disabled=${!this.ready || this.busy} @click=${() => this.emit("audio-change", { enabled: !audioEnabled })}>${audioEnabled ? "Stop audio" : "Start audio"}</button>
          <button type="button" ?disabled=${!this.ready || this.busy} @click=${() => this.emit("connection-change", { enabled: !connectionsAllowed })}>${connectionsAllowed ? "Disable connections" : "Enable connections"}</button>
          <button class="danger" type="button" ?disabled=${!this.ready || this.busy} @click=${() => this.emit("adapter-restart")}>Restart adapter</button>
        </div>
        ${this.ready ? "" : html`<p>Connect the adapter to use these controls.</p>`}
      </section>
    `;
  }
}

customElements.define("adapter-controls", AdapterControls);
