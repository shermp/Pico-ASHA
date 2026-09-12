import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";

export class AdapterLog extends LitElement {
  static properties = { entries: { type: Array }, timing: { type: Object } };
  static styles = [componentStyles, css`
    :host {
      display: block;
    }

    .log {
      width: 100%;
      overflow: hidden;
    }

    header {
      display: flex;
      min-height: 3.3rem;
      align-items: center;
      justify-content: space-between;
      gap: 1rem;
      padding: 0.4rem 0.55rem 0.4rem 0.9rem;
    }

    .title,
    .actions,
    .timing {
      display: flex;
      align-items: center;
      gap: 0.45rem;
    }

    h2 {
      margin: 0;
      font-size: 0.86rem;
    }

    .count {
      color: var(--app-muted);
      font-size: 0.72rem;
    }

    .timing {
      margin-left: auto;
      color: var(--app-muted);
      font-size: 0.72rem;
    }

    pre {
      height: min(52dvh, 32rem);
      margin: 0;
      padding: 0.85rem 1rem;
      overflow: auto;
      border-top: 1px solid var(--app-border);
      background: color-mix(in srgb, var(--app-bg) 86%, transparent);
      color: #cfe7ff;
      font: 0.75rem/1.5 ui-monospace, SFMono-Regular, Consolas, monospace;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
    }

    @media (max-width: 42rem) {
      .timing {
        display: none;
      }
    }
  `];

  constructor() { super(); this.entries = []; this.timing = null; }
  updated(changed) {
    if (changed.has("entries")) {
      // Keep the newest diagnostic entry visible.
      const pre = this.renderRoot.querySelector("pre");
      if (pre) { pre.scrollTop = pre.scrollHeight; }
    }
  }
  emit(name) { this.dispatchEvent(new CustomEvent(name, { bubbles: true, composed: true })); }

  render() {
    return html`
      <section class="log panel" aria-label="Adapter log"><header>
        <div class="title">${icon("terminal")}<h2>Adapter log</h2><span class="count">${this.entries.length}</span></div>
        ${this.timing ? html`<span class="timing" title="Encoder timing after 1,000 samples">Encoder µs: ${this.timing.minimum.toFixed(0)} / ${this.timing.average.toFixed(1)} / ${this.timing.maximum.toFixed(0)}</span>` : ""}
        <div class="actions">
          <button class="icon-button" type="button" aria-label="Copy adapter log" title="Copy log" ?disabled=${!this.entries.length} @click=${() => this.emit("log-copy")}>${icon("content_copy")}</button>
          <button class="icon-button" type="button" aria-label="Download adapter log" title="Download log" ?disabled=${!this.entries.length} @click=${() => this.emit("log-download")}>${icon("download")}</button>
          <button class="icon-button" type="button" aria-label="Clear adapter log" title="Clear log" ?disabled=${!this.entries.length} @click=${() => this.emit("log-clear")}>${icon("delete")}</button>
        </div>
      </header><pre role="log" aria-live="polite">${this.entries.join("\n")}</pre></section>
    `;
  }
}

customElements.define("adapter-log", AdapterLog);
