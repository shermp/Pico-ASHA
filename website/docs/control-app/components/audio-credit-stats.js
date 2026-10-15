import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";

export class AudioCreditStats extends LitElement {
  static properties = { remotes: { type: Array } };
  static styles = [componentStyles, css`
    section { display: flex; flex-wrap: wrap; align-items: baseline; gap: 0.3rem 1.2rem; padding: 0.48rem 0.72rem; font-size: 0.64rem; }
    h2, p { margin: 0; font-size: inherit; }
    h2, span { color: var(--app-muted); }
    h2, strong { font-weight: 500; }
    p { white-space: nowrap; font-variant-numeric: tabular-nums; }
  `];

  constructor() { super(); this.remotes = []; }

  render() {
    return html`
      <section class="panel" aria-labelledby="credit-stats-title">
        <h2 id="credit-stats-title">Credits</h2>
        ${this.remotes.map((remote) => html`
          <p title=${`Connection ${remote.connectionId}${remote.creditStats ? ` · ${remote.creditStats.sampleCount.toLocaleString()} audio ticks` : ""}`}>
            <strong>${remote.side}:</strong>
            ${remote.creditStats ? html`
              ${remote.creditStats.average.toFixed(2)}
              <span>· base ${remote.creditStats.baseline.toFixed(2)} · Δ ${remote.creditStats.difference > 0 ? "+" : ""}${remote.creditStats.difference.toFixed(2)}</span>
            ` : html`<span>${remote.streaming ? "Waiting for baseline" : "Not streaming"}</span>`}
          </p>
        `)}
        ${!this.remotes.length ? html`<span>No hearing aids connected</span>` : ""}
      </section>
    `;
  }
}

customElements.define("audio-credit-stats", AudioCreditStats);
