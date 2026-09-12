import { LitElement, css, html } from "../vendor/lit-core.min.js";
import { Command, EVENT_NAMES, EventType, StatusType } from "../protocol/constants.js";
import { validateUSBSettings } from "../protocol/codec.js";
import { describeEventError, describeSerialError, webSerialSupportMessage } from "../protocol/errors.js";
import { AdapterState } from "../protocol/state.js";
import { HciCapture } from "../serial/hci-capture.js";
import { SerialController } from "../serial/serial-controller.js";
import { componentStyles } from "./component-styles.js";
import "./adapter-controls.js";
import "./adapter-log.js";
import "./app-header.js";
import "./hci-capture-controls.js";
import "./pairing-dialog.js";
import "./remote-grid.js";
import "./settings-dialog.js";
import "./toast-message.js";

export class PicoAshaApp extends LitElement {
  static properties = {
    connection: { type: Object }, adapter: { type: Object }, logEntries: { type: Array },
    hci: { type: Object }, busy: { type: Boolean }, supportMessage: { type: String }, toastState: { type: Object },
    activeTab: { state: true },
  };

  static styles = [componentStyles, css`
    :host {
      display: block;
      min-height: 100vh;
    }

    main {
      width: min(72rem, calc(100% - 2rem));
      margin: 0 auto;
      padding: 1rem 0;
    }

    .tabs {
      display: flex;
      gap: 0.25rem;
      margin-top: 1rem;
      padding: 0.3rem;
      border: 1px solid var(--app-border);
      border-radius: 0.85rem;
      background: color-mix(in srgb, var(--app-panel) 88%, transparent);
      box-shadow: 0 0.65rem 1.75rem rgba(0, 0, 0, 0.1);
      backdrop-filter: blur(18px);
    }

    [role="tab"] {
      display: inline-flex;
      flex: 1 1 0;
      min-height: 2.7rem;
      align-items: center;
      justify-content: center;
      margin: 0;
      padding: 0.55rem 1rem;
      border: 1px solid transparent;
      border-radius: 0.62rem;
      background: transparent;
      color: var(--app-muted);
      cursor: pointer;
      font-weight: 600;
    }

    [role="tab"]:hover {
      color: var(--app-text);
    }

    [role="tab"][aria-selected="true"] {
      border-color: color-mix(in srgb, var(--app-accent) 42%, var(--app-border));
      background: color-mix(in srgb, var(--app-accent) 14%, var(--app-panel-soft));
      color: var(--app-accent-strong);
    }

    [role="tabpanel"] {
      margin-top: 1rem;
    }

    [role="tabpanel"][hidden] {
      display: none;
    }

    adapter-controls,
    hci-capture-controls,
    adapter-log,
    remote-grid {
      display: block;
    }

    remote-grid,
    adapter-log {
      margin-top: 1rem;
    }

    .notice {
      display: flex;
      align-items: flex-start;
      gap: 0.55rem;
      margin: 1rem 0 0;
      padding: 0.75rem 0.9rem;
      border: 1px solid color-mix(in srgb, var(--app-warning) 45%, var(--app-border));
      border-radius: 0.8rem;
      background: color-mix(in srgb, var(--app-warning) 9%, var(--app-panel));
      color: var(--app-muted);
      font-size: 0.8rem;
    }

  `];

  constructor() {
    super();
    this.connection = Object.freeze({ phase: "idle", label: "Adapter disconnected" });
    this.store = new AdapterState();
    this.adapter = this.store.snapshot;
    this.logEntries = [];
    this.capture = new HciCapture();
    this.hci = Object.freeze({ phase: "idle", bytes: 0, downloadAvailable: false });
    this.busy = false;
    this.supportMessage = webSerialSupportMessage();
    this.toastState = Object.freeze({ message: "", kind: "info", visible: false });
    this.activeTab = "adapter";
    this.toastTimer = null;
    this.controller = new SerialController({
      onPacket: (packet) => this.handlePacket(packet),
      onStatus: (status) => this.handleStatus(status),
      onError: (error) => this.handleError(error),
      onDiagnostic: (error) => this.addLog(describeSerialError(error), "warning"),
    });
    this.beforeUnload = (event) => {
      if (this.capture.active) {
        event.preventDefault();
        event.returnValue = "";
      }
    };
    globalThis.addEventListener?.("beforeunload", this.beforeUnload);
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    globalThis.removeEventListener?.("beforeunload", this.beforeUnload);
    this.controller.dispose();
  }

  firstUpdated() {
    if (this.supportMessage) {
      this.showToast(this.supportMessage, "warning", 9000);
    }
  }

  addLog(message, level = "info", connectionId = 0) {
    const now = new Date();
    const time = `${now.toLocaleTimeString([], { hour12: false })}.${String(now.getMilliseconds()).padStart(3, "0")}`;
    const marker = level === "error" ? "ERROR" : level === "warning" ? "WARN" : "INFO";
    const connection = connectionId ? ` [Connection ${connectionId}]` : "";
    this.logEntries = [...this.logEntries.slice(-1999), `[${time}] ${marker}${connection}  ${message}`];
  }

  showToast(message, kind = "info", duration = 5000) {
    clearTimeout(this.toastTimer);
    this.toastState = Object.freeze({ message, kind, visible: true });
    if (duration) {
      this.toastTimer = setTimeout(() => {
        this.toastState = Object.freeze({ ...this.toastState, visible: false });
      }, duration);
    }
  }

  handleStatus(status) {
    this.connection = status;
    if (status.phase === "reconnecting") {
      this.adapter = this.store.resetSession();
    }
    if (status.phase === "ready") {
      if (this.hci.phase === "starting") {
        this.hci = Object.freeze({ phase: "capturing", bytes: this.capture.bytes, downloadAvailable: false });
        this.addLog("HCI capture started");
      } else if (this.hci.phase === "stopping") {
        this.finishCapture();
      }
    }
  }

  handleError(error, connectionId = 0) {
    const message = describeSerialError(error);
    this.addLog(message, "error", connectionId);
    this.showToast(message, "error", 8000);
  }

  handlePacket(packet) {
    const connectionId = packet.header?.connectionId || packet.connectionId || 0;
    if (["intro", "usb-info", "remote-info", "advert", "event"].includes(packet.kind)) {
      // Advertisements update the pairing badge only; opening the dialog is always an explicit user action.
      this.adapter = this.store.apply(packet);
    }

    if (packet.kind === "event") {
      if (packet.eventType === EventType.AudioVolume) {
        return;
      }
      if (packet.eventType === 0 && packet.text) {
        this.addLog(packet.text, "info", connectionId);
      } else if (packet.statusType !== StatusType.Success && packet.status !== 0) {
        this.addLog(describeEventError(packet), "error", connectionId);
      } else {
        this.addLog(EVENT_NAMES[packet.eventType] ?? `Event ${packet.eventType}`, "info", connectionId);
      }
    } else if (packet.kind === "intro") {
      this.addLog(`Pico-ASHA firmware ${packet.version}; ${packet.numberConnected} hearing aid(s) connected`, "info", connectionId);
    } else if (packet.kind === "usb-info") {
      this.addLog(`USB Audio Class ${packet.uacVersion}; ${packet.minimumDb} to ${packet.maximumDb} dB`, "info", connectionId);
    } else if (packet.kind === "hci") {
      if (!this.capture.append(packet.data)) {
        this.addLog("HCI capture reached the 64 MiB limit and was stopped", "warning", connectionId);
        this.showToast("HCI capture reached 64 MiB. Stopping and preparing the download.", "warning", 9000);
        this.hci = Object.freeze({ phase: "stopping", bytes: this.capture.bytes, downloadAvailable: true });
        void this.sendRestartingCommand(Command.HCIDump, { enabled: false });
      } else {
        this.hci = Object.freeze({ ...this.hci, bytes: this.capture.bytes });
      }
    }
  }

  async connect() {
    if (this.supportMessage) {
      this.showToast(this.supportMessage, "warning", 9000);
      return;
    }
    try {
      await this.controller.connect();
    } catch (error) {
      this.connection = Object.freeze({ phase: "idle", label: "Adapter disconnected" });
      this.handleError(error);
    }
  }

  async disconnect() {
    this.connection = Object.freeze({ phase: "disconnecting", label: "Disconnecting…" });
    await this.controller.disconnect({ manual: true });
    this.adapter = this.store.resetSession();
    this.addLog("Adapter disconnected by user");
  }

  async sendCommand(command, data = {}, options = {}, successMessage = "Setting updated") {
    const connectionId = options.connectionId || data.connectionId || 0;
    this.busy = true;
    try {
      await this.controller.sendCommand(command, data, options);
      if (options.expectRestart) {
        await new Promise((resolve) => setTimeout(resolve, 500));
        if (this.controller.ready) {
          this.controller.restartExpected = false;
          this.controller.restartObserved = false;
          this.addLog("Setting was already active; no adapter restart was needed", "info", connectionId);
        }
      }
      if (successMessage) {
        this.showToast(successMessage);
        this.addLog(successMessage, "info", connectionId);
      }
      return true;
    } catch (error) {
      if (options.expectRestart && this.controller.restartObserved) {
        this.addLog("Adapter restart detected; waiting to reconnect", "info", connectionId);
        return true;
      }
      this.handleError(error, connectionId);
      return false;
    } finally {
      this.busy = false;
    }
  }

  sendRestartingCommand(command, data = {}, successMessage = "Adapter is restarting…") {
    return this.sendCommand(command, data, { expectRestart: true }, successMessage);
  }

  async changeAudio(enabled) {
    if (await this.sendCommand(Command.AudioStreaming, { enabled }, {}, enabled ? "Audio streaming enabled" : "Audio streaming disabled")) {
      this.adapter = this.store.updateIntro({ audioStreamingEnabled: enabled });
    }
  }

  async changeConnections(enabled) {
    if (await this.sendCommand(Command.AllowConnect, { enabled }, {}, enabled ? "New connections enabled" : "New connections disabled")) {
      this.adapter = this.store.updateIntro({ connectionsAllowed: enabled });
    }
  }

  async restartAdapter() {
    this.renderRoot.querySelector("settings-dialog")?.close();
    await this.sendRestartingCommand(Command.Restart);
  }

  async unpair(remote) {
    const label = remote.name || remote.address;
    if (!globalThis.confirm?.(`Unpair ${label}? The hearing aid will need to be paired again before use.`)) {
      return;
    }
    await this.sendCommand(Command.DeletePair, { connectionId: remote.connectionId }, { connectionId: remote.connectionId }, `${label} unpaired`);
  }

  async updateUSB(settings) {
    try {
      const validated = validateUSBSettings(settings);
      const current = this.adapter.usbInfo;
      if (current && current.uacVersion === validated.uacVersion && current.minimumRaw === validated.minimumRaw && current.maximumRaw === validated.maximumRaw) {
        this.showToast("USB settings are unchanged");
        return;
      }
      this.renderRoot.querySelector("settings-dialog")?.close();
      await this.sendRestartingCommand(Command.USBSettings, validated, "USB settings saved; adapter is restarting…");
    } catch (error) {
      this.handleError(error);
    }
  }

  openPairing() {
    this.renderRoot.querySelector("settings-dialog")?.close();
    this.renderRoot.querySelector("pairing-dialog")?.show();
  }

  handlePairingClose(event) {
    const dismissed = event?.detail?.dismissed ?? true;
    if (dismissed) {
      // A manual close ends the current scan so stale nearby-device notifications are not retained.
      this.adapter = this.store.clearAdverts();
    }
  }

  async pairCandidate(candidate) {
    if (await this.sendCommand(Command.PairBond, candidate, {}, `Pairing with ${candidate.name || candidate.address}…`)) {
      // Keep any other candidate in the store so its badge can notify the user after this dialog closes.
      this.adapter = this.store.removeAdvert(candidate.address);
      this.renderRoot.querySelector("pairing-dialog")?.close({ dismissed: false });
    }
  }

  async startCapture() {
    this.capture.start();
    this.hci = Object.freeze({ phase: "starting", bytes: this.capture.bytes, downloadAvailable: false });
    this.renderRoot.querySelector("settings-dialog")?.close();
    const accepted = await this.sendRestartingCommand(Command.HCIDump, { enabled: true }, "Enabling HCI capture; adapter is restarting…");
    if (!accepted && !this.controller.restartExpected) {
      this.capture.reset();
      this.hci = Object.freeze({ phase: "idle", bytes: 0, downloadAvailable: false });
    } else if (!this.controller.restartExpected && this.hci.phase === "starting") {
      this.hci = Object.freeze({ phase: "capturing", bytes: this.capture.bytes, downloadAvailable: false });
      this.addLog("HCI capture started without an adapter restart");
    }
  }

  async stopCapture() {
    this.hci = Object.freeze({ ...this.hci, phase: "stopping" });
    this.renderRoot.querySelector("settings-dialog")?.close();
    const accepted = await this.sendRestartingCommand(Command.HCIDump, { enabled: false }, "Stopping HCI capture; adapter is restarting…");
    if (!accepted && !this.controller.restartExpected) {
      this.hci = Object.freeze({ ...this.hci, phase: "capturing" });
    } else if (!this.controller.restartExpected && this.hci.phase === "stopping") {
      this.finishCapture();
    }
  }

  finishCapture() {
    this.capture.stop();
    const downloaded = this.capture.download();
    this.hci = Object.freeze({ phase: "ready", bytes: this.capture.bytes, downloadAvailable: true });
    this.addLog(`HCI capture completed (${(this.capture.bytes / 1048576).toFixed(2)} MiB)`);
    this.showToast(downloaded ? "HCI capture download started" : "HCI capture is ready; use Download again if the browser blocked it", downloaded ? "info" : "warning", 8000);
  }

  downloadCapture() {
    if (!this.capture.download()) {
      this.showToast("The browser blocked the download. Allow downloads for this site and try again.", "warning", 8000);
    }
  }

  logText() { return `${this.logEntries.join("\n")}\n`; }

  async copyLog() {
    try {
      await navigator.clipboard.writeText(this.logText());
      this.showToast("Adapter log copied");
    } catch (error) {
      this.handleError(new Error(`Could not copy the log: ${error.message}`));
    }
  }

  downloadLog() {
    const blob = new Blob([this.logText()], { type: "text/plain" });
    const anchor = document.createElement("a");
    const url = URL.createObjectURL(blob);
    anchor.href = url;
    anchor.download = `pico-asha-log-${new Date().toISOString().replace(/[:.]/g, "-")}.txt`;
    anchor.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  selectTab(tab, { focus = false } = {}) {
    if (!["adapter", "diagnostics"].includes(tab)) {
      return;
    }
    this.activeTab = tab;
    if (focus) {
      void this.updateComplete.then(() => this.renderRoot.querySelector(`#${tab}-tab`)?.focus());
    }
  }

  handleTabKeydown(event) {
    const tabs = ["adapter", "diagnostics"];
    const current = tabs.indexOf(this.activeTab);
    let next = current;
    if (event.key === "ArrowRight") next = (current + 1) % tabs.length;
    else if (event.key === "ArrowLeft") next = (current - 1 + tabs.length) % tabs.length;
    else if (event.key === "Home") next = 0;
    else if (event.key === "End") next = tabs.length - 1;
    else return;
    event.preventDefault();
    this.selectTab(tabs[next], { focus: true });
  }

  render() {
    return html`
      <main
        @adapter-connect=${this.connect} @adapter-disconnect=${this.disconnect}
        @settings-open=${() => this.renderRoot.querySelector("settings-dialog")?.show()}
        @audio-change=${(event) => this.changeAudio(event.detail.enabled)}
        @connection-change=${(event) => this.changeConnections(event.detail.enabled)}
        @pairing-open=${this.openPairing} @adapter-restart=${this.restartAdapter}
        @remote-unpair=${(event) => this.unpair(event.detail.remote)}
        @usb-update=${(event) => this.updateUSB(event.detail)}
        @hci-start=${this.startCapture} @hci-stop=${this.stopCapture} @hci-download=${this.downloadCapture}
        @pairing-select=${(event) => this.pairCandidate(event.detail.candidate)}
        @pairing-close=${this.handlePairingClose}
        @log-copy=${this.copyLog} @log-download=${this.downloadLog} @log-clear=${() => { this.logEntries = []; }}
      >
        <app-header
          .connection=${this.connection}
          .uacVersion=${this.adapter.usbInfo?.uacVersion ?? this.adapter.intro?.uacVersion ?? null}
          .candidateCount=${this.adapter.adverts.length}
          .busy=${this.busy}
        ></app-header>
        ${this.supportMessage ? html`<p class="notice">${this.supportMessage}</p>` : ""}
        <div class="tabs" role="tablist" aria-label="Main content" @keydown=${this.handleTabKeydown}>
          <button
            id="adapter-tab"
            type="button"
            role="tab"
            aria-controls="adapter-panel"
            aria-selected=${this.activeTab === "adapter"}
            tabindex=${this.activeTab === "adapter" ? "0" : "-1"}
            @click=${() => this.selectTab("adapter")}
          >Adapter</button>
          <button
            id="diagnostics-tab"
            type="button"
            role="tab"
            aria-controls="diagnostics-panel"
            aria-selected=${this.activeTab === "diagnostics"}
            tabindex=${this.activeTab === "diagnostics" ? "0" : "-1"}
            @click=${() => this.selectTab("diagnostics")}
          >Diagnostics</button>
        </div>
        <section
          id="adapter-panel"
          role="tabpanel"
          aria-labelledby="adapter-tab"
          tabindex="0"
          ?hidden=${this.activeTab !== "adapter"}
        >
          <adapter-controls .ready=${this.connection.phase === "ready"} .intro=${this.adapter.intro} .busy=${this.busy}></adapter-controls>
          <remote-grid .remotes=${this.adapter.remotes} .adapterConnected=${this.connection.phase === "ready"}></remote-grid>
        </section>
        <section
          id="diagnostics-panel"
          role="tabpanel"
          aria-labelledby="diagnostics-tab"
          tabindex="0"
          ?hidden=${this.activeTab !== "diagnostics"}
        >
          <hci-capture-controls .ready=${this.connection.phase === "ready"} .hci=${this.hci} .busy=${this.busy}></hci-capture-controls>
          <adapter-log .entries=${this.logEntries} .timing=${this.adapter.timing}></adapter-log>
        </section>
        <settings-dialog .ready=${this.connection.phase === "ready"} .usbInfo=${this.adapter.usbInfo} .remotes=${this.adapter.remotes} .busy=${this.busy}></settings-dialog>
        <pairing-dialog .candidates=${this.adapter.adverts} .busy=${this.busy}></pairing-dialog>
        <toast-message .message=${this.toastState.message} .kind=${this.toastState.kind} .visible=${this.toastState.visible}></toast-message>
      </main>
    `;
  }
}

customElements.define("pico-asha-app", PicoAshaApp);
