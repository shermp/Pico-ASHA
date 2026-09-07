import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import "./remote-card.js";

export class RemoteGrid extends LitElement {
  static properties = { remotes: { type: Array } };
  static styles = [componentStyles, css`
    section { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 1rem; }
    @media (max-width: 46rem) { section { grid-template-columns: 1fr; } }
  `];

  constructor() {
    super();
    this.remotes = [];
  }

  remoteFor(side) {
    return this.remotes.find((remote) => remote.side === side) ?? null;
  }

  render() {
    return html`
      <section aria-label="Hearing aids">
        <remote-card side="Left" .remote=${this.remoteFor("Left")}></remote-card>
        <remote-card side="Right" .remote=${this.remoteFor("Right")}></remote-card>
      </section>
    `;
  }
}

customElements.define("remote-grid", RemoteGrid);
