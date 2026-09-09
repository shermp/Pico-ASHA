import { COMMAND_NAMES, COMMAND_TIMEOUT_MS } from "./constants.js";

export class CommandRejectedError extends Error {}
export class CommandTimeoutError extends Error {}

export class CommandResponseTracker {
  constructor({
    timeoutMs = COMMAND_TIMEOUT_MS,
    setTimer = (callback, delay) => globalThis.setTimeout(callback, delay),
    clearTimer = (timer) => globalThis.clearTimeout(timer),
  } = {}) {
    this.timeoutMs = timeoutMs;
    this.setTimer = setTimer;
    this.clearTimer = clearTimer;
    this.pending = new Map();
  }

  expect(command) {
    // Responses identify only the command, not a request sequence number. Keep one in flight per command.
    if (this.pending.has(command)) {
      return Promise.reject(new Error(`${COMMAND_NAMES[command] ?? command} is already pending`));
    }
    return new Promise((resolve, reject) => {
      const timer = this.setTimer(() => {
        this.pending.delete(command);
        reject(new CommandTimeoutError(`${COMMAND_NAMES[command] ?? command} timed out`));
      }, this.timeoutMs);
      this.pending.set(command, { resolve, reject, timer });
    });
  }

  accept(packet) {
    const pending = this.pending.get(packet.command);
    if (!pending) {
      // A delayed response after a timeout or disconnect is no longer actionable.
      return false;
    }
    this.clearTimer(pending.timer);
    this.pending.delete(packet.command);
    if (packet.accepted) {
      pending.resolve(packet);
    } else {
      pending.reject(new CommandRejectedError(`${COMMAND_NAMES[packet.command] ?? packet.command} was rejected by the adapter`));
    }
    return true;
  }

  cancel(command, reason = new Error("Command cancelled")) {
    const pending = this.pending.get(command);
    if (!pending) {
      return;
    }
    this.clearTimer(pending.timer);
    this.pending.delete(command);
    pending.reject(reason);
  }

  cancelAll(reason = new Error("Adapter disconnected")) {
    for (const command of [...this.pending.keys()]) {
      this.cancel(command, reason);
    }
  }
}
