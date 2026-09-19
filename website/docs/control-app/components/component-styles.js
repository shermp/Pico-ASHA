import { css } from "../vendor/lit-core.min.js";

export const componentStyles = css`
  :host {
    box-sizing: border-box;
    color: var(--app-text);
    font-size: 0.75rem;
    line-height: 1.5;
  }

  *,
  *::before,
  *::after {
    box-sizing: inherit;
  }

  .panel {
    border: 1px solid var(--app-border);
    border-radius: var(--app-radius);
    background: var(--app-panel);
    box-shadow: var(--app-shadow, 0 0.8rem 2.2rem rgba(0, 0, 0, 0.16));
  }

  .icon-button {
    display: inline-grid;
    width: 1.92rem;
    height: 1.92rem;
    margin: 0;
    padding: 0;
    place-items: center;
    border: 1px solid var(--app-border);
    border-radius: var(--app-control-radius, 0.576rem);
    background: var(--app-panel-soft);
    color: var(--app-text);
    cursor: pointer;
  }

  .icon-button:hover:not(:disabled) {
    border-color: var(--app-accent);
    color: var(--app-accent-strong);
  }

  .icon-button:disabled {
    cursor: not-allowed;
    opacity: 0.42;
  }

  .material-symbols-outlined {
    display: inline-block;
    overflow: hidden;
    width: 1em;
    height: 1em;
    font-family: "Material Symbols Outlined";
    /* Match Zensical's .md-icon svg dimensions. */
    font-size: 0.96rem;
    font-style: normal;
    font-weight: 400;
    font-feature-settings: "liga";
    font-variation-settings: "FILL" 0, "wght" 400, "GRAD" 0, "opsz" 24;
    line-height: 1;
    text-rendering: optimizeLegibility;
    white-space: nowrap;
  }

  button,
  input,
  select {
    font: inherit;
  }

  button,
  select {
    font-size: 0.7rem;
  }

  button:focus-visible,
  input:focus-visible,
  select:focus-visible,
  summary:focus-visible {
    outline: 2px solid var(--app-accent);
    outline-offset: 2px;
  }
`;
