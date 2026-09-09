import { CobsFrameDecoder, framePacket } from "../protocol/cobs.js";
import { decodePacket, encodeCommandPacket } from "../protocol/codec.js";
import { Command, INTRO_TIMEOUT_MS, RECONNECT_DELAY_MS, SERIAL_OPTIONS, USB_FILTER } from "../protocol/constants.js";
import { CommandResponseTracker } from "../protocol/command-responses.js";

function portMatches(port) {
  const info = port?.getInfo?.() ?? {};
  return info.usbVendorId === USB_FILTER.usbVendorId && info.usbProductId === USB_FILTER.usbProductId;
}

function portIsConnected(port) {
  return port?.connected !== false;
}

export class SerialOperationError extends Error {
  constructor(operation, cause) {
    const labels = {
      open: "Opening the serial port",
      dtr: "Asserting DTR",
    };
    const originalMessage = cause?.message || String(cause || "Unknown serial error");
    super(`${labels[operation] || operation} failed: ${originalMessage}`, { cause });
    this.name = cause?.name || "SerialOperationError";
    this.operation = operation;
    this.originalMessage = originalMessage;
  }
}

export class SerialController {
  constructor({
    serial = globalThis.navigator?.serial,
    onPacket = () => {},
    onStatus = () => {},
    onError = () => {},
    onDiagnostic = () => {},
    setTimer = (callback, delay) => globalThis.setTimeout(callback, delay),
    clearTimer = (timer) => globalThis.clearTimeout(timer),
  } = {}) {
    this.serial = serial;
    this.onPacket = onPacket;
    this.onStatus = onStatus;
    this.onError = onError;
    this.onDiagnostic = onDiagnostic;
    this.setTimer = setTimer;
    this.clearTimer = clearTimer;
    this.port = null;
    this.reader = null;
    this.readTask = null;
    // WritableStream permits one locked writer at a time, so commands are serialized through this promise tail.
    this.writeTail = Promise.resolve();
    this.manualDisconnect = true;
    this.closing = false;
    this.ready = false;
    this.restartExpected = false;
    // A restart is confirmed only after the active port disconnects.
    this.restartObserved = false;
    this.introTimer = null;
    this.reconnectTimer = null;
    this.tracker = new CommandResponseTracker({ setTimer, clearTimer });
    this.decoder = new CobsFrameDecoder({ onError: (error) => this.onError(error) });
    this.handleSerialDisconnect = (event) => {
      if (event.target === this.port || event.port === this.port) {
        void this.handleUnexpectedDisconnect();
      }
    };
    this.serial?.addEventListener?.("disconnect", this.handleSerialDisconnect);
  }

  setStatus(phase, label) {
    this.onStatus(Object.freeze({ phase, label }));
  }

  async connect({ requestPort = true } = {}) {
    if (!this.serial) {
      throw new Error("Web Serial is unavailable");
    }
    this.manualDisconnect = false;
    this.clearReconnect();
    const reconnecting = !requestPort;
    this.setStatus(reconnecting ? "reconnecting" : "connecting", reconnecting ? "Waiting for adapter…" : "Selecting adapter…");

    const authorized = (await this.serial.getPorts()).find((port) => portMatches(port) && portIsConnected(port));
    let port = authorized;
    if (!port && requestPort) {
      port = await this.requestMatchingPort();
    }
    if (!port) {
      throw new Error("No previously authorized Pico-ASHA adapter is available");
    }
    if (!portMatches(port)) {
      throw new Error("The selected serial device is not a Pico-ASHA adapter");
    }
    try {
      await this.open(port, { reconnecting });
    } catch (error) {
      const retryCachedPort = Boolean(authorized) && requestPort && error.operation === "open" && error.name === "NetworkError";
      if (!retryCachedPort) {
        throw error;
      }
      this.onDiagnostic(error);
      this.setStatus("connecting", "Cached adapter unavailable; select it again…");
      port = await this.requestMatchingPort();
      await this.open(port);
    }
  }

  async requestMatchingPort() {
    const port = await this.serial.requestPort({ filters: [USB_FILTER] });
    if (!portMatches(port)) {
      throw new Error("The selected serial device is not a Pico-ASHA adapter");
    }
    return port;
  }

  async open(port, { reconnecting = false } = {}) {
    this.port = port;
    this.closing = false;
    this.ready = false;
    this.decoder.reset();
    const restarting = reconnecting || this.restartExpected;
    this.setStatus(restarting ? "reconnecting" : "connecting", this.restartExpected ? "Adapter restarting…" : "Opening adapter…");
    try {
      await port.open(SERIAL_OPTIONS);
    } catch (error) {
      const operationError = new SerialOperationError("open", error);
      await this.closePort();
      throw operationError;
    }
    try {
      await port.setSignals?.({ dataTerminalReady: true });
    } catch (error) {
      const operationError = new SerialOperationError("dtr", error);
      await this.closePort();
      throw operationError;
    }
    this.readTask = this.readLoop(port);
    this.armIntroTimeout();
    // The intro packet is both the device handshake and the signal that enables adapter controls.
    void this.sendCommand(Command.IntroPacket).catch((error) => {
      if (!this.restartExpected) {
        this.onError(error);
      }
    });
  }

  armIntroTimeout() {
    this.clearTimer(this.introTimer);
    this.introTimer = this.setTimer(() => {
      if (!this.ready) {
        this.onError(new Error("The adapter did not send its introduction within 1 second"));
        void this.handleUnexpectedDisconnect();
      }
    }, INTRO_TIMEOUT_MS);
  }

  markReady(version = "") {
    this.clearTimer(this.introTimer);
    this.introTimer = null;
    this.ready = true;
    this.restartExpected = false;
    this.restartObserved = false;
    this.setStatus("ready", version ? `Firmware ${version}` : "Adapter connected");
  }

  async readLoop(port) {
    try {
      while (port === this.port && port.readable) {
        this.reader = port.readable.getReader();
        try {
          while (true) {
            const { value, done } = await this.reader.read();
            if (done) {
              break;
            }
            for (const frame of this.decoder.push(value)) {
              try {
                const packet = decodePacket(frame);
                if (packet.kind === "command") {
                  this.tracker.accept(packet);
                }
                if (packet.kind === "intro") {
                  this.markReady(packet.version);
                }
                this.onPacket(packet);
              } catch (error) {
                this.onError(error);
              }
            }
          }
        } finally {
          this.reader.releaseLock();
          this.reader = null;
        }
        break;
      }
    } catch (error) {
      if (!this.closing) {
        this.onError(error);
      }
    } finally {
      if (!this.closing && port === this.port) {
        void this.handleUnexpectedDisconnect();
      }
    }
  }

  async sendCommand(command, data = {}, options = {}) {
    if (!this.port?.writable) {
      throw new Error("Adapter is not connected");
    }
    if (options.expectRestart) {
      this.restartExpected = true;
      this.restartObserved = false;
    }
    const response = this.tracker.expect(command);
    const packet = encodeCommandPacket(command, data, {
      connectionId: options.connectionId ?? 0,
      timestampMs: Math.round(globalThis.performance?.now?.() ?? Date.now()),
    });
    try {
      await this.enqueueWrite(framePacket(packet));
    } catch (error) {
      this.tracker.cancel(command, error);
      await response.catch(() => {});
      if (options.expectRestart && !this.restartObserved) {
        this.restartExpected = false;
      }
      throw error;
    }
    try {
      return await response;
    } catch (error) {
      if (options.expectRestart && !this.restartObserved) {
        this.restartExpected = false;
      }
      throw error;
    }
  }

  enqueueWrite(bytes) {
    // Continue after a failed earlier write so one transport error does not permanently block later commands.
    const operation = this.writeTail.catch(() => {}).then(async () => {
      if (!this.port?.writable) {
        throw new Error("Adapter write stream is unavailable");
      }
      const writer = this.port.writable.getWriter();
      try {
        await writer.write(bytes);
      } finally {
        writer.releaseLock();
      }
    });
    this.writeTail = operation;
    return operation;
  }

  async disconnect({ manual = true } = {}) {
    this.manualDisconnect = manual;
    this.clearReconnect();
    if (manual) {
      this.restartExpected = false;
      this.restartObserved = false;
    }
    await this.closePort();
    this.setStatus("idle", manual ? "Adapter disconnected" : "Adapter unavailable");
  }

  async closePort() {
    if (!this.port || this.closing) {
      return;
    }
    const port = this.port;
    this.closing = true;
    this.ready = false;
    this.clearTimer(this.introTimer);
    this.introTimer = null;
    this.tracker.cancelAll();
    try {
      await this.reader?.cancel();
    } catch {}
    try {
      await port.setSignals?.({ dataTerminalReady: false });
    } catch {}
    try {
      await port.close();
    } catch {}
    if (this.port === port) {
      this.port = null;
    }
    this.closing = false;
  }

  async handleUnexpectedDisconnect() {
    if (this.closing) {
      return;
    }
    const shouldReconnect = !this.manualDisconnect;
    if (this.restartExpected) {
      // USB and HCI settings intentionally restart the adapter; seeing the disconnect confirms that transition.
      this.restartObserved = true;
    }
    await this.closePort();
    if (shouldReconnect) {
      this.setStatus("reconnecting", "Waiting for adapter…");
      this.scheduleReconnect();
    }
  }

  scheduleReconnect() {
    if (this.manualDisconnect || this.reconnectTimer) {
      return;
    }
    this.reconnectTimer = this.setTimer(async () => {
      this.reconnectTimer = null;
      try {
        // Reuse a previously authorized port without reopening the browser's device picker.
        await this.connect({ requestPort: false });
      } catch {
        if (!this.manualDisconnect) {
          this.setStatus("reconnecting", "Waiting for adapter…");
        }
        this.scheduleReconnect();
      }
    }, RECONNECT_DELAY_MS);
  }

  clearReconnect() {
    this.clearTimer(this.reconnectTimer);
    this.reconnectTimer = null;
  }

  dispose() {
    this.manualDisconnect = true;
    this.clearReconnect();
    this.serial?.removeEventListener?.("disconnect", this.handleSerialDisconnect);
  }
}
