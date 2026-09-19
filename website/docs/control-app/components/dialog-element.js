import { LitElement, css } from "../vendor/lit-core.min.js";

export const dialogStyles = css`
  dialog {
    margin: auto;
    padding: 0;
    border: 1px solid var(--app-border);
    border-radius: var(--app-radius);
    background: var(--app-panel);
    color: var(--app-text);
    box-shadow: var(--app-dialog-shadow, 0 1.6rem 4.8rem rgba(0, 0, 0, 0.45));
  }

  dialog::backdrop {
    background: color-mix(in srgb, var(--app-bg) 86%, transparent);
  }

  dialog > header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 0.72rem 0.8rem;
    border-bottom: 1px solid var(--app-border);
  }

  dialog > header h2 {
    margin: 0;
    font-size: 0.84rem;
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
