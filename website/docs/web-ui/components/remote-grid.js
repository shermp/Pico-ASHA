import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { componentStyles } from "./component-styles.js";
import { icon } from "./icon.js";
import "./remote-card.js";

export class RemoteGrid extends LitElement {
  static properties = { remotes: { type: Array }, adapterConnected: { type: Boolean } };
  static styles = [componentStyles, css`
    section {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 1rem;
    }

    section.empty {
      grid-template-columns: 1fr;
    }

    .empty-state {
      display: grid;
      min-height: 10rem;
      place-items: center;
      align-content: center;
      gap: 0.6rem;
      padding: 1.5rem;
      text-align: center;
    }

    .empty-state .material-symbols-outlined {
      color: var(--app-muted);
      font-size: 2.5rem;
    }

    .empty-state p {
      margin: 0;
      color: var(--app-muted);
    }

    @media (max-width: 46rem) {
      section {
        grid-template-columns: 1fr;
      }
    }
  `];

  constructor() {
    super();
    this.remotes = [];
    this.adapterConnected = false;
  }

  remoteFor(side) {
    return this.remotes.find((remote) => remote.side === side) ?? null;
  }

  render() {
    const empty = !this.remotes.length;
    const emptyTitle = this.adapterConnected ? "No hearing aids connected" : "Adapter disconnected";
    const emptyHint = this.adapterConnected
      ? "Pair a hearing aid to get started."
      : "Use the cable button to connect your Pico-ASHA adapter.";
    return html`
      <section class=${empty ? "empty" : ""} aria-label="Hearing aids">
        ${empty ? html`
          <article class="empty-state panel">
            ${icon("hearing_disabled")}
            <strong>${emptyTitle}</strong>
            <p>${emptyHint}</p>
          </article>
        ` : html`
          <remote-card side="Left" .remote=${this.remoteFor("Left")}></remote-card>
          <remote-card side="Right" .remote=${this.remoteFor("Right")}></remote-card>
        `}
      </section>
    `;
  }
}

customElements.define("remote-grid", RemoteGrid);
