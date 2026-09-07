import { css } from "../vendor/lit-core.min.js";

export const componentStyles = css`
  :host {
    box-sizing: border-box;
    color: var(--app-text);
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
    box-shadow: 0 1rem 2.75rem rgba(0, 0, 0, 0.16);
    backdrop-filter: blur(18px);
  }

  .muted {
    color: var(--app-muted);
  }

  .icon-button {
    display: inline-grid;
    width: 2.55rem;
    height: 2.55rem;
    margin: 0;
    padding: 0;
    place-items: center;
    border: 1px solid var(--app-border);
    border-radius: 0.72rem;
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
    font-family: "Material Symbols Outlined";
    font-size: 1.25rem;
    font-style: normal;
    font-weight: 400;
    font-feature-settings: "liga";
    font-variation-settings: "FILL" 0, "wght" 400, "GRAD" 0, "opsz" 24;
    line-height: 1;
    text-rendering: optimizeLegibility;
    white-space: nowrap;
  }

  button:focus-visible,
  input:focus-visible,
  select:focus-visible,
  summary:focus-visible {
    outline: 3px solid color-mix(in srgb, var(--app-accent) 75%, white);
    outline-offset: 3px;
  }
`;
