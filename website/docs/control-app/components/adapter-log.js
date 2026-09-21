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
      min-height: 2.64rem;
      align-items: center;
      justify-content: space-between;
      gap: 0.8rem;
      padding: 0.32rem 0.44rem 0.32rem 0.72rem;
    }

    .title,
    .actions,
    .timing {
      display: flex;
      align-items: center;
      gap: 0.36rem;
    }

    h2 {
      margin: 0;
      font-size: 0.688rem;
    }

    .count {
      color: var(--app-muted);
      font-size: 0.576rem;
    }

    .timing {
      margin-left: auto;
      color: var(--app-muted);
      font-size: 0.576rem;
    }

    pre {
      height: clamp(7.5rem, calc(100dvh - var(--app-log-viewport-offset, 18rem)), 20rem);
      margin: 0;
      padding: 0.68rem 0.8rem;
      overflow: auto;
      border-top: 1px solid var(--app-border);
      background: color-mix(in srgb, var(--app-bg) 86%, transparent);
      color: var(--app-code-text, #cfe7ff);
      font: 0.6rem/1.5 var(--md-code-font-family, ui-monospace, SFMono-Regular, Consolas, monospace);
      white-space: pre-wrap;
      overflow-wrap: anywhere;
    }

    @media (max-width: 33.6rem) {
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
        ${this.timing ? html`<span class="timing" title="Encoder timing after 1,000 samples">Encoder µs (min/avg/max): ${this.timing.minimum.toFixed(0)} / ${this.timing.average.toFixed(1)} / ${this.timing.maximum.toFixed(0)}</span>` : ""}
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
