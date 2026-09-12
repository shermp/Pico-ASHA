import { LitElement, css } from "../vendor/lit-core.min.js";

export const dialogStyles = css`
  dialog {
    margin: auto;
    padding: 0;
    border: 1px solid var(--app-border);
    border-radius: var(--app-radius);
    background: var(--app-panel);
    color: var(--app-text);
    box-shadow: 0 2rem 6rem rgba(0, 0, 0, 0.45);
  }

  dialog::backdrop {
    background: rgba(2, 8, 15, 0.72);
    backdrop-filter: blur(4px);
  }

  dialog > header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 0.9rem 1rem;
    border-bottom: 1px solid var(--app-border);
  }

  dialog > header h2 {
    margin: 0;
    font-size: 1.05rem;
  }
`;

export class DialogElement extends LitElement {
  get dialog() {
    return this.renderRoot.querySelector("dialog");
  }

  get open() {
    return Boolean(this.dialog?.open);
  }

  show() {
    if (!this.dialog?.open) {
      this.dialog?.showModal();
    }
  }

  close() {
    this.dialog?.close();
  }
}
