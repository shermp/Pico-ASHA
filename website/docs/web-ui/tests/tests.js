import { CobsFrameDecoder, cobsDecode, cobsEncode, framePacket } from "../protocol/cobs.js";
import { CommandResponseTracker, CommandRejectedError, CommandTimeoutError } from "../protocol/command-responses.js";
import { decodePacket, encodeCommandPacket, parseROP, validateUSBSettings } from "../protocol/codec.js";
import { Command, EventType, IntroFlag, PACKET_SIZE, PacketType, SERIAL_OPTIONS, StatusType, USB_FILTER } from "../protocol/constants.js";
import { describeSerialError, describeStatus, webSerialSupportMessage } from "../protocol/errors.js";
import { AdapterState } from "../protocol/state.js";
import { BTSNOOP_FILE_HEADER, HciCapture } from "../serial/hci-capture.js";
import { SerialController } from "../serial/serial-controller.js";
import "../components/adapter-log.js";
import "../components/app-header.js";
import "../components/pairing-dialog.js";
import { batteryColor, batteryIcon, volumeToDb } from "../components/remote-card.js";
import "../components/settings-dialog.js";
import { PicoAshaApp } from "../components/app-shell.js";

const tests = [];
const test = (name, body) => tests.push({ name, body });
const assert = (condition, message = "Assertion failed") => { if (!condition) { throw new Error(message); } };
const equal = (actual, expected, message = "Values differ") => {
  const a = JSON.stringify(actual);
  const e = JSON.stringify(expected);
  if (a !== e) { throw new Error(`${message}: expected ${e}, received ${a}`); }
};
const wait = () => new Promise((resolve) => setTimeout(resolve, 0));

function packet(type, size, connectionId = 0) {
  const bytes = new Uint8Array(size);
  const view = new DataView(bytes.buffer);
  view.setUint8(0, type);
  view.setUint8(1, size);
  view.setUint16(2, connectionId, true);
  view.setUint32(4, 123456, true);
  return bytes;
}

function writeText(bytes, offset, text, length) {
  bytes.set(new TextEncoder().encode(text).subarray(0, length - 1), offset);
}

function makeIntro() {
  const bytes = packet(PacketType.Intro, PACKET_SIZE.Intro);
  const view = new DataView(bytes.buffer);
  bytes.set([1, 8, 2], 8);
  view.setInt8(11, -1);
  view.setUint16(12, IntroFlag.ConnectionsAllowed | IntroFlag.AudioStreamingEnabled | IntroFlag.UAC2, true);
  return bytes;
}

function makeRemote({ connectionId = 7, side = 0, address = [1, 2, 3, 4, 5, 6] } = {}) {
  const bytes = packet(PacketType.RemoteInfo, PACKET_SIZE.RemoteInfo, connectionId);
  const view = new DataView(bytes.buffer);
  view.setUint16(8, connectionId, true);
  view.setUint16(10, 0x1234, true);
  bytes.set(address, 12);
  bytes[18] = 1;
  bytes[19] = 1;
  view.setUint16(20, 0x25, true);
  view.setUint16(22, 0x42, true);
  writeText(bytes, 24, "Demo Aid", 32);
  writeText(bytes, 56, "Demo Make", 32);
  writeText(bytes, 88, "Model X", 32);
  writeText(bytes, 120, "1.0", 32);
  writeText(bytes, 152, "2.0", 32);
  bytes[184] = side;
  bytes[185] = 1;
  bytes[186] = 1;
  view.setInt8(187, -22);
  bytes[188] = 8;
  return bytes;
}

function makeEvent(type, connectionId = 7) {
  const bytes = packet(PacketType.Event, PACKET_SIZE.Event, connectionId);
  bytes[8] = type;
  bytes[9] = StatusType.Success;
  return bytes;
}

test("COBS round-trips empty, zero-rich, and 254-byte values", () => {
  for (const value of [new Uint8Array(), Uint8Array.of(0, 1, 0, 2, 0), Uint8Array.from({ length: 254 }, (_, i) => (i + 1) & 0xff)]) {
    equal([...cobsDecode(cobsEncode(value))], [...value]);
  }
});

test("COBS stream decoder handles fragmented frames", () => {
  const framed = framePacket(Uint8Array.of(1, 0, 2, 3));
  const decoder = new CobsFrameDecoder();
  equal(decoder.push(framed.subarray(0, 3)), []);
  equal([...decoder.push(framed.subarray(3))[0]], [1, 0, 2, 3]);
});

test("COBS stream decoder handles leading delimiters and adjacent frames", () => {
  const joined = new Uint8Array([...framePacket(Uint8Array.of(1)), ...framePacket(Uint8Array.of(2, 0, 3))]);
  const frames = new CobsFrameDecoder().push(joined);
  equal(frames.map((value) => [...value]), [[1], [2, 0, 3]]);
});

test("Malformed COBS data is rejected without losing the next frame", () => {
  const errors = [];
  const decoder = new CobsFrameDecoder({ onError: (error) => errors.push(error) });
  const frames = decoder.push(Uint8Array.from([0, 5, 1, 0, ...framePacket(Uint8Array.of(9))]));
  assert(errors.length === 1);
  equal([...frames[0]], [9]);
});

test("Oversized COBS frames are discarded until a delimiter", () => {
  const errors = [];
  const decoder = new CobsFrameDecoder({ maxPacketBytes: 4, onError: (error) => errors.push(error) });
  decoder.push(Uint8Array.of(1, 1, 1, 1, 1, 1, 0));
  const frames = decoder.push(framePacket(Uint8Array.of(4)));
  assert(errors.length === 1);
  equal([...frames[0]], [4]);
});

test("Intro packet decodes signed count, flags, version, and UAC", () => {
  const result = decodePacket(makeIntro());
  equal([result.version, result.numberConnected, result.connectionsAllowed, result.audioStreamingEnabled, result.uacVersion], ["1.8.2", -1, true, true, 2]);
});

test("RemoteInfo decodes every displayed field and signed volume", () => {
  const result = decodePacket(makeRemote());
  equal([result.connectionId, result.hciHandle, result.address, result.paired, result.psm, result.l2capCid], [7, 0x1234, "01:02:03:04:05:06", true, 0x25, 0x42]);
  equal([result.name, result.manufacturer, result.model, result.firmware, result.software], ["Demo Aid", "Demo Make", "Model X", "1.0", "2.0"]);
  equal([result.side, result.mode, result.streaming, result.volume, result.battery], ["Left", "Binaural", true, -22, 8]);
});

test("Event packet exposes strings, signed data, ROP, and timing views", () => {
  const bytes = makeEvent(EventType.G722EncodeTimings);
  const view = new DataView(bytes.buffer);
  for (let i = 0; i < 10; i += 1) { view.setInt16(12 + i * 2, i - 5, true); }
  const result = decodePacket(bytes);
  equal(result.encodeTimings, [-5, -4, -3, -2, -1, 0, 1, 2, 3, 4]);
  assert(result.data.length === 32 && result.rop.length === 17);
});

test("HCI packet preserves its variable binary payload", () => {
  const bytes = packet(PacketType.HCI, 13);
  bytes.set([1, 0, 2, 0, 3], 8);
  equal([...decodePacket(bytes).data], [1, 0, 2, 0, 3]);
});

test("Command response decodes acceptance and rejection", () => {
  const bytes = packet(PacketType.Command, PACKET_SIZE.Command);
  bytes[8] = Command.Restart;
  bytes[9] = 1;
  const result = decodePacket(bytes);
  assert(result.command === Command.Restart && !result.accepted);
});

test("Advert decodes address, signed RSSI, hearing-aid flag, and fixed name", () => {
  const bytes = packet(PacketType.Advert, PACKET_SIZE.Advert);
  const view = new DataView(bytes.buffer);
  bytes.set([10, 11, 12, 13, 14, 15], 8);
  bytes[14] = 1;
  view.setInt8(15, -71);
  bytes[16] = 1;
  writeText(bytes, 20, "Nearby Aid", 28);
  const result = decodePacket(bytes);
  equal([result.address, result.addressType, result.rssi, result.isHearingAid, result.name], ["0a:0b:0c:0d:0e:0f", 1, -71, true, "Nearby Aid"]);
});

test("USBInfo decodes UAC and signed fixed-point volume", () => {
  const bytes = packet(PacketType.USBInfo, PACKET_SIZE.USBInfo);
  const view = new DataView(bytes.buffer);
  view.setUint16(8, 2, true);
  view.setInt16(10, -5760, true);
  view.setInt16(12, -96, true);
  const result = decodePacket(bytes);
  equal([result.uacVersion, result.minimumDb, result.maximumDb], [2, -60, -1]);
});

test("Every fixed packet type enforces its exact length", () => {
  for (const [type, size] of [[0, 16], [1, 192], [2, 44], [4, 20], [5, 48], [6, 16]]) {
    const bytes = packet(type, size - 1);
    bytes[1] = size - 1;
    let rejected = false;
    try { decodePacket(bytes); } catch { rejected = true; }
    assert(rejected, `type ${type} accepted the wrong length`);
  }
});

test("Header length mismatch and unknown packet type are rejected", () => {
  const bytes = makeIntro();
  bytes[1] = 15;
  assertThrows(() => decodePacket(bytes));
  const unknown = packet(99, 8);
  assertThrows(() => decodePacket(unknown));
});

function assertThrows(body) {
  let threw = false;
  try { body(); } catch { threw = true; }
  assert(threw, "Expected function to throw");
}

test("ROP capabilities decode side, mode, IDs, delay, and codecs", () => {
  const rop = new Uint8Array(17);
  const view = new DataView(rop.buffer);
  rop[0] = 1; rop[1] = 3; view.setUint16(2, 0x4567, true); rop.set([1, 2, 3, 4, 5, 6], 4); rop[10] = 1; view.setUint16(11, 44, true); view.setUint16(15, 6, true);
  const result = parseROP(rop);
  equal([result.side, result.mode, result.manufacturerId, result.leCocSupported, result.renderDelayMs, result.supportsG72216, result.supportsG72224], ["Right", "Binaural", 0x4567, true, 44, true, true]);
});

test("Status descriptions cover Pico-ASHA, BTstack, ATT, L2CAP, and SM", () => {
  equal([
    describeStatus(StatusType.PicoASHA, 2), describeStatus(StatusType.BTstack, 8), describeStatus(StatusType.ATT, 5),
    describeStatus(StatusType.L2CAP, 4), describeStatus(StatusType.SecurityManager, 9),
  ], ["Maximum connected devices reached", "Connection timeout", "Insufficient authentication", "No resources available", "Repeated attempts"]);
});

test("All eight commands produce exact 20-byte layouts", () => {
  const cases = [
    [Command.HCIDump, { enabled: true }], [Command.DeletePair, { connectionId: 0x3344 }], [Command.Restart, {}],
    [Command.AllowConnect, { enabled: true }], [Command.AudioStreaming, { enabled: true }], [Command.IntroPacket, {}],
    [Command.PairBond, { address: "01:02:03:04:05:06", addressType: 1 }],
    [Command.USBSettings, { uacVersion: 2, minimumDb: -60, maximumDb: -1 }],
  ];
  for (const [command, data] of cases) {
    const bytes = encodeCommandPacket(command, data, { connectionId: data.connectionId ?? 0x22, timestampMs: 99 });
    assert(bytes.length === 20 && bytes[0] === PacketType.Command && bytes[1] === 20 && bytes[8] === command);
  }
});

test("Boolean, delete, pair, and USB command union offsets are exact", () => {
  assert(encodeCommandPacket(Command.HCIDump, { enabled: true })[10] === 1);
  const deletion = encodeCommandPacket(Command.DeletePair, { connectionId: 0x1234 });
  assert(new DataView(deletion.buffer).getUint16(10, true) === 0x1234);
  equal([...encodeCommandPacket(Command.PairBond, { address: "01:02:03:04:05:06", addressType: 1 }).subarray(10, 17)], [1, 2, 3, 4, 5, 6, 1]);
  const usb = encodeCommandPacket(Command.USBSettings, { uacVersion: 2, minimumDb: -60, maximumDb: -1 });
  const view = new DataView(usb.buffer);
  equal([view.getUint16(10, true), view.getInt16(12, true), view.getInt16(14, true)], [2, -5760, -96]);
});

test("USB settings validate bounds and ordering", () => {
  equal(validateUSBSettings({ uacVersion: 1, minimumDb: -127, maximumDb: 0 }).minimumRaw, -12192);
  for (const value of [{ uacVersion: 3, minimumDb: -60, maximumDb: 0 }, { uacVersion: 2, minimumDb: -128, maximumDb: 0 }, { uacVersion: 2, minimumDb: 0, maximumDb: 0 }]) {
    assertThrows(() => validateUSBSettings(value));
  }
});

test("Remote snapshots are left/right ordered and immutable", () => {
  const store = new AdapterState();
  store.apply(decodePacket(makeRemote({ connectionId: 8, side: 1, address: [8, 8, 8, 8, 8, 8] })));
  const snapshot = store.apply(decodePacket(makeRemote({ connectionId: 7, side: 0 })));
  equal(snapshot.remotes.map((remote) => remote.side), ["Left", "Right"]);
  assert(Object.isFrozen(snapshot) && Object.isFrozen(snapshot.remotes) && Object.isFrozen(snapshot.remotes[0]));
});

test("Remote info uses the selected firmware codec before properties are read", () => {
  const store = new AdapterState();
  const snapshot = store.apply(decodePacket(makeRemote()));
  equal(snapshot.remotes[0].audioFormat, "G.722 @ 16 kHz");
});

test("Remote identity cache survives temporary disconnect", () => {
  const store = new AdapterState();
  store.apply(decodePacket(makeRemote()));
  const disconnected = makeEvent(EventType.RemoteDisconnected);
  store.apply(decodePacket(disconnected));
  const connected = makeEvent(EventType.RemoteConnected);
  connected.set([1, 2, 3, 4, 5, 6], 12);
  new DataView(connected.buffer).setUint16(18, 0x7777, true);
  const snapshot = store.apply(decodePacket(connected));
  equal([snapshot.remotes[0].name, snapshot.remotes[0].manufacturer, snapshot.remotes[0].hciHandle], ["Demo Aid", "Demo Make", 0x7777]);
});

test("Closing pairing clears stale nearby-device candidates", () => {
  const app = new PicoAshaApp();
  app.adapter = app.store.apply({ kind: "advert", isHearingAid: true, address: "01:02:03:04:05:06", rssi: -50 });
  app.handlePairingClose();
  equal(app.adapter.adverts, []);
  app.controller.dispose();
  globalThis.removeEventListener("beforeunload", app.beforeUnload);
});

test("Pairing advertisements notify the main-page button without auto-opening", async () => {
  const app = new PicoAshaApp(); document.querySelector("#fixtures").append(app); await app.updateComplete;
  app.connection = { phase: "ready", label: "Adapter ready" }; await app.updateComplete;
  app.sendCommand = async () => true;
  const first = { kind: "advert", isHearingAid: true, name: "Left Aid", address: "01:02:03:04:05:06", addressType: 1, rssi: -45 };
  app.handlePacket(first); await app.updateComplete; await Promise.resolve();
  const dialog = app.renderRoot.querySelector("pairing-dialog");
  const header = app.renderRoot.querySelector("app-header"); await header.updateComplete;
  const firstButton = header.renderRoot.querySelector('[aria-label="Pair device (1 nearby)"]');
  assert(!dialog.open && firstButton?.querySelector(".notification-badge")?.textContent === "1");
  firstButton.click(); await app.updateComplete; await dialog.updateComplete;
  assert(dialog.open);
  await app.pairCandidate(first); await app.updateComplete;
  assert(!dialog.open && app.adapter.adverts.length === 0);
  const second = { kind: "advert", isHearingAid: true, name: "Right Aid", address: "11:12:13:14:15:16", addressType: 1, rssi: -48 };
  app.handlePacket(second); await app.updateComplete; await Promise.resolve();
  await header.updateComplete;
  const secondButton = header.renderRoot.querySelector('[aria-label="Pair device (1 nearby)"]');
  assert(!dialog.open && secondButton?.querySelector(".notification-badge")?.textContent === "1" && app.adapter.adverts[0].address === second.address);
  secondButton.click(); await app.updateComplete; await dialog.updateComplete;
  assert(dialog.open);
  dialog.close(); app.remove();
});

test("Remote event updates battery, volume, streaming, PSM, and L2CAP", () => {
  const store = new AdapterState();
  store.apply(decodePacket(makeRemote()));
  for (const [type, write] of [
    [EventType.MFIBatteryRead, (bytes) => { bytes[12] = 6; }],
    [EventType.AudioVolume, (bytes) => { new DataView(bytes.buffer).setInt8(12, -128); }],
    [EventType.ASPStop, () => {}],
    [EventType.PSMRead, (bytes) => { bytes[12] = 0x31; }],
    [EventType.L2CAPConnected, (bytes) => { new DataView(bytes.buffer).setUint16(12, 0x88, true); }],
  ]) {
    const bytes = makeEvent(type); write(bytes); store.apply(decodePacket(bytes));
  }
  const remote = store.snapshot.remotes[0];
  equal([remote.battery, remote.volume, remote.muted, remote.streaming, remote.psm, remote.l2capCid], [6, -128, true, false, 0x31, 0x88]);
});

test("Remote active format follows the selected firmware codec", () => {
  const store = new AdapterState();
  store.apply(decodePacket(makeRemote()));
  const bytes = makeEvent(EventType.ROPRead);
  const view = new DataView(bytes.buffer);
  view.setUint16(27, 0x0006, true);
  const snapshot = store.apply(decodePacket(bytes));
  equal([snapshot.remotes[0].supportsG72216, snapshot.remotes[0].supportsG72224, snapshot.remotes[0].audioFormat], [true, true, "G.722 @ 16 kHz"]);
});

test("Intro and USB packets drive immutable adapter settings", () => {
  const store = new AdapterState();
  store.apply(decodePacket(makeIntro()));
  const usb = packet(PacketType.USBInfo, 16); const view = new DataView(usb.buffer); view.setUint16(8, 1, true); view.setInt16(10, -4800, true); view.setInt16(12, 0, true);
  const snapshot = store.apply(decodePacket(usb));
  equal([snapshot.intro.connectionsAllowed, snapshot.intro.audioStreamingEnabled, snapshot.usbInfo.uacVersion], [true, true, 1]);
});

test("Runtime control changes persist when later packets refresh the snapshot", () => {
  const store = new AdapterState();
  store.apply(decodePacket(makeIntro()));
  store.updateIntro({ audioStreamingEnabled: false, connectionsAllowed: false });
  store.apply(decodePacket(makeEvent(EventType.ShortLog)));
  equal([store.snapshot.intro.audioStreamingEnabled, store.snapshot.intro.connectionsAllowed], [false, false]);
});

test("Encoder timing publishes min/average/max after 1,000 samples", () => {
  const store = new AdapterState();
  for (let batch = 0; batch < 100; batch += 1) {
    const bytes = makeEvent(EventType.G722EncodeTimings); const view = new DataView(bytes.buffer);
    for (let i = 0; i < 10; i += 1) { view.setInt16(12 + i * 2, batch * 10 + i, true); }
    store.apply(decodePacket(bytes));
  }
  equal(store.snapshot.timing, { count: 1000, minimum: 0, average: 499.5, maximum: 999 });
});

test("Command tracker resolves acceptance and rejects adapter errors", async () => {
  const tracker = new CommandResponseTracker();
  const accepted = tracker.expect(Command.Restart);
  tracker.accept({ command: Command.Restart, accepted: true });
  await accepted;
  const rejected = tracker.expect(Command.HCIDump);
  tracker.accept({ command: Command.HCIDump, accepted: false });
  let error; try { await rejected; } catch (caught) { error = caught; }
  assert(error instanceof CommandRejectedError);
});

test("Command tracker reports timeout", async () => {
  let callback;
  const tracker = new CommandResponseTracker({ setTimer: (body) => { callback = body; return 1; }, clearTimer: () => {} });
  const pending = tracker.expect(Command.Restart);
  callback();
  let error; try { await pending; } catch (caught) { error = caught; }
  assert(error instanceof CommandTimeoutError);
});

class MockPort {
  constructor({ connected, openError = null, signalError = null } = {}) {
    this.openCalls = [];
    this.signalCalls = [];
    this.writes = [];
    this.closed = false;
    this.openError = openError;
    this.signalError = signalError;
    if (connected !== undefined) {
      this.connected = connected;
    }
    this.makeStreams();
  }
  makeStreams() {
    this.readable = new ReadableStream({ start: (controller) => { this.readableController = controller; } });
    this.writable = new WritableStream({ write: async (chunk) => { this.writes.push(Uint8Array.from(chunk)); } });
  }
  getInfo() { return { ...USB_FILTER }; }
  async open(options) {
    if (this.closed) {
      this.closed = false;
      this.makeStreams();
    }
    this.openCalls.push(options);
    if (this.openError) {
      throw this.openError;
    }
  }
  async setSignals(signals) {
    this.signalCalls.push(signals);
    if (signals.dataTerminalReady && this.signalError) {
      throw this.signalError;
    }
  }
  async close() { this.closed = true; }
  send(bytes) { this.readableController.enqueue(bytes); }
}

function noTimers() { return { setTimer: () => 1, clearTimer: () => {} }; }

test("Web Serial requests the exact VID/PID filter and open options", async () => {
  const port = new MockPort();
  let requestOptions;
  const serial = { getPorts: async () => [], requestPort: async (options) => { requestOptions = options; return port; }, addEventListener() {}, removeEventListener() {} };
  const controller = new SerialController({ serial, ...noTimers() });
  await controller.connect();
  equal(requestOptions, { filters: [USB_FILTER] });
  equal(port.openCalls[0], SERIAL_OPTIONS);
  equal(port.signalCalls[0], { dataTerminalReady: true });
  await controller.disconnect();
});

test("Authorized matching ports are reused without a picker", async () => {
  const port = new MockPort();
  let requested = false;
  const serial = { getPorts: async () => [port], requestPort: async () => { requested = true; return port; }, addEventListener() {}, removeEventListener() {} };
  const controller = new SerialController({ serial, ...noTimers() });
  await controller.connect();
  assert(!requested);
  await controller.disconnect();
});

test("Explicitly disconnected authorized ports are skipped for the filtered picker", async () => {
  const cached = new MockPort({ connected: false });
  const selected = new MockPort({ connected: true });
  let requestOptions;
  const serial = {
    getPorts: async () => [cached],
    requestPort: async (options) => { requestOptions = options; return selected; },
    addEventListener() {},
    removeEventListener() {},
  };
  const controller = new SerialController({ serial, ...noTimers() });
  await controller.connect();
  assert(cached.openCalls.length === 0 && selected.openCalls.length === 1);
  equal(requestOptions, { filters: [USB_FILTER] });
  await controller.disconnect();
});

test("A stale cached port retries once through the filtered picker", async () => {
  const cached = new MockPort({ connected: true, openError: new DOMException("cached handle is stale", "NetworkError") });
  const selected = new MockPort({ connected: true });
  const statuses = [];
  const diagnostics = [];
  let requestCount = 0;
  const serial = {
    getPorts: async () => [cached],
    requestPort: async () => { requestCount += 1; return selected; },
    addEventListener() {},
    removeEventListener() {},
  };
  const controller = new SerialController({
    serial,
    onStatus: (status) => statuses.push(status),
    onDiagnostic: (error) => diagnostics.push(error),
    ...noTimers(),
  });
  await controller.connect();
  assert(requestCount === 1 && cached.closed && controller.port === selected);
  assert(statuses.some((status) => status.label.includes("Cached adapter unavailable")));
  assert(diagnostics[0]?.operation === "open" && diagnostics[0]?.originalMessage === "cached handle is stale");
  await controller.disconnect();
});

test("Picker-selected open failures preserve the operation and browser detail", async () => {
  const port = new MockPort({ openError: new DOMException("driver refused the port", "NetworkError") });
  const serial = {
    getPorts: async () => [],
    requestPort: async () => port,
    addEventListener() {},
    removeEventListener() {},
  };
  const controller = new SerialController({ serial, ...noTimers() });
  let error;
  try {
    await controller.connect();
  } catch (caught) {
    error = caught;
  }
  const message = describeSerialError(error);
  assert(error?.operation === "open" && error?.name === "NetworkError");
  assert(error?.originalMessage === "driver refused the port" && controller.port === null);
  assert(message.includes("could not be opened") && message.includes("driver refused the port"));
});

test("DTR failures are distinct and close the opened port", async () => {
  const port = new MockPort({ signalError: new DOMException("control transfer failed", "NetworkError") });
  const serial = {
    getPorts: async () => [],
    requestPort: async () => port,
    addEventListener() {},
    removeEventListener() {},
  };
  const controller = new SerialController({ serial, ...noTimers() });
  let error;
  try {
    await controller.connect();
  } catch (caught) {
    error = caught;
  }
  const message = describeSerialError(error);
  assert(error?.operation === "dtr" && error?.name === "NetworkError");
  assert(error?.originalMessage === "control transfer failed" && port.closed && controller.port === null);
  assert(message.includes("assert DTR") && message.includes("control transfer failed"));
});

test("Serial writes are queued in request order", async () => {
  const port = new MockPort();
  const controller = new SerialController({ serial: { addEventListener() {}, removeEventListener() {} }, ...noTimers() });
  controller.port = port;
  await Promise.all([controller.enqueueWrite(Uint8Array.of(1)), controller.enqueueWrite(Uint8Array.of(2)), controller.enqueueWrite(Uint8Array.of(3))]);
  equal(port.writes.map((bytes) => bytes[0]), [1, 2, 3]);
});

test("Intro reception marks the serial session ready", async () => {
  const port = new MockPort();
  const statuses = [];
  const serial = { getPorts: async () => [port], addEventListener() {}, removeEventListener() {} };
  const controller = new SerialController({ serial, onStatus: (status) => statuses.push(status), ...noTimers() });
  await controller.connect();
  port.send(framePacket(makeIntro()));
  await wait(); await wait();
  assert(controller.ready && statuses.at(-1).phase === "ready");
  await controller.disconnect();
});

test("Manual disconnect lowers DTR, closes the port, and suppresses reconnect", async () => {
  const port = new MockPort();
  const serial = { getPorts: async () => [port], addEventListener() {}, removeEventListener() {} };
  const controller = new SerialController({ serial, ...noTimers() });
  await controller.connect();
  await controller.disconnect({ manual: true });
  equal(port.signalCalls.at(-1), { dataTerminalReady: false });
  assert(port.closed && controller.manualDisconnect && controller.reconnectTimer === null);
});

test("Unexpected disconnect schedules authorized-port reconnect", async () => {
  const port = new MockPort();
  const timers = [];
  const serial = { getPorts: async () => [port], addEventListener() {}, removeEventListener() {} };
  const controller = new SerialController({ serial, setTimer: (body) => { timers.push(body); return timers.length; }, clearTimer: () => {} });
  await controller.connect();
  await controller.handleUnexpectedDisconnect();
  assert(!controller.manualDisconnect && controller.reconnectTimer !== null);
  await timers.at(-1)();
  assert(port.openCalls.length === 2);
  await controller.disconnect();
});

test("Failed automatic reconnect remains in a cancellable reconnecting state", async () => {
  const timers = [];
  const statuses = [];
  const serial = { getPorts: async () => [], addEventListener() {}, removeEventListener() {} };
  const controller = new SerialController({
    serial,
    onStatus: (status) => statuses.push(status),
    setTimer: (body) => { timers.push(body); return timers.length; },
    clearTimer: () => {},
  });
  controller.manualDisconnect = false;
  controller.scheduleReconnect();
  await timers[0]();
  assert(statuses.at(-1).phase === "reconnecting");
  await controller.disconnect();
});

test("Restart command failures clear intent unless a disconnect was observed", async () => {
  const port = new MockPort();
  const serial = { addEventListener() {}, removeEventListener() {} };
  const controller = new SerialController({ serial, ...noTimers() });
  controller.port = port;
  const rejected = controller.sendCommand(Command.Restart, {}, { expectRestart: true });
  controller.tracker.accept({ command: Command.Restart, accepted: false });
  let error;
  try { await rejected; } catch (caught) { error = caught; }
  assert(error instanceof CommandRejectedError && !controller.restartExpected && !controller.restartObserved);

  controller.port = port;
  const restarting = controller.sendCommand(Command.Restart, {}, { expectRestart: true });
  await controller.handleUnexpectedDisconnect();
  try { await restarting; } catch {}
  assert(controller.restartExpected && controller.restartObserved);
  controller.dispose();
});

test("Capability messaging covers insecure pages and unavailable/policy-disabled API", () => {
  assert(webSerialSupportMessage({ serial: {}, secure: false, hostname: "example.com" }).includes("HTTPS"));
  const unavailable = webSerialSupportMessage({ serial: undefined, secure: true, hostname: "example.com" });
  assert(unavailable.includes("Firefox 151+") && unavailable.includes("enterprise policy"));
  assert(webSerialSupportMessage({ serial: {}, secure: true, hostname: "example.com" }) === "");
});

test("Permission cancellation and policy denial have actionable Firefox guidance", () => {
  const cancelled = describeSerialError({ name: "NotFoundError" });
  const denied = describeSerialError({ name: "NotAllowedError" });
  assert(cancelled.includes("Firefox 151+") && cancelled.includes("permission add-on gate"));
  assert(denied.includes("managed Firefox") && denied.includes("policy"));
});

test("Serial controller arms the exact one-second introduction timeout", async () => {
  const port = new MockPort();
  const delays = [];
  const serial = { getPorts: async () => [port], addEventListener() {}, removeEventListener() {} };
  const controller = new SerialController({ serial, setTimer: (_body, delay) => { delays.push(delay); return delays.length; }, clearTimer: () => {} });
  await controller.connect();
  assert(delays.includes(1000));
  await controller.disconnect();
});

test("btsnoop header is version 1 with H4 data-link type 1002", () => {
  equal([...BTSNOOP_FILE_HEADER.subarray(0, 8)], [...new TextEncoder().encode("btsnoop\0")]);
  const view = new DataView(BTSNOOP_FILE_HEADER.buffer);
  equal([view.getUint32(8), view.getUint32(12)], [1, 1002]);
});

test("HCI capture finalizes valid header and packet data", async () => {
  const capture = new HciCapture({ now: () => new Date("2026-01-02T03:04:05Z") });
  capture.start(); capture.append(Uint8Array.of(1, 2, 3)); const blob = capture.stop();
  equal([...new Uint8Array(await blob.arrayBuffer())], [...BTSNOOP_FILE_HEADER, 1, 2, 3]);
  assert(capture.filename === "pico-asha-2026-01-02T03-04-05-000Z.log");
});

test("HCI capture stops and preserves a downloadable blob at its size cap", () => {
  const capture = new HciCapture({ limit: BTSNOOP_FILE_HEADER.length + 2 });
  capture.start(); assert(capture.append(Uint8Array.of(1, 2))); assert(!capture.append(Uint8Array.of(3)));
  assert(!capture.active && capture.limitReached && capture.blob instanceof Blob);
});

test("HCI download fallback reports blocked document creation", () => {
  const capture = new HciCapture(); capture.start(); capture.stop();
  assert(!capture.download(null, null));
});

test("App reports failed restart commands unless the controller observed a restart", async () => {
  const app = new PicoAshaApp();
  const originalController = app.controller;
  const errors = [];
  app.handleError = (error) => errors.push(error);
  app.controller = {
    restartExpected: false,
    restartObserved: false,
    sendCommand: async () => {
      app.controller.restartExpected = true;
      throw new Error("Command rejected");
    },
  };
  assert(!await app.sendRestartingCommand(Command.Restart));
  assert(errors.length === 1);

  app.controller.restartObserved = true;
  assert(await app.sendRestartingCommand(Command.Restart));
  assert(errors.length === 1);
  originalController.dispose();
  globalThis.removeEventListener("beforeunload", app.beforeUnload);
});

test("App header emits connection events and shows firmware/UAC together", async () => {
  const element = document.createElement("app-header"); document.querySelector("#fixtures").append(element); await element.updateComplete;
  let received = false; element.addEventListener("adapter-connect", () => { received = true; });
  const button = element.renderRoot.querySelector('[aria-label="Connect adapter"]'); button.click();
  assert(received && button.title === "Connect adapter");
  element.connection = { phase: "ready", label: "Firmware 1.8.2" };
  element.uacVersion = 2;
  await element.updateComplete;
  assert(element.renderRoot.textContent.includes("Firmware 1.8.2 · UAC2"));
  let pairingOpened = false; element.addEventListener("pairing-open", () => { pairingOpened = true; });
  element.candidateCount = 2; await element.updateComplete;
  const pairingButton = element.renderRoot.querySelector('[aria-label="Pair device (2 nearby)"]');
  assert(pairingButton?.title === "Pair device (2 nearby)" && pairingButton.querySelector(".notification-badge")?.textContent === "2" && pairingButton.querySelector(".material-symbols-outlined")?.textContent === "bluetooth_connected");
  pairingButton.click(); assert(pairingOpened);
  element.remove();
});

test("Remote card reacts to immutable state and presents battery/volume/streaming", async () => {
  const element = document.createElement("remote-card"); document.querySelector("#fixtures").append(element); element.side = "Left"; await element.updateComplete;
  assert(element.renderRoot.textContent.includes("No left hearing aid"));
  element.remote = Object.freeze({ side: "Left", name: "Test Aid", streaming: true, muted: false, volume: -12, battery: 9, paired: true }); await element.updateComplete;
  const text = element.renderRoot.textContent;
  const leftBadge = element.renderRoot.querySelector(".aid");
  const battery = element.renderRoot.querySelector(".battery");
  assert(text.includes("Test Aid") && text.includes("Streaming") && text.includes("-4.5 dB") && text.includes("9/10"));
  assert(leftBadge.textContent.trim() === "L" && leftBadge.classList.contains("left") && !text.includes("Left channel"));
  assert(battery.textContent === "battery_6_bar" && battery.classList.contains("high"));
  element.remote = Object.freeze({ ...element.remote, battery: 4 }); await element.updateComplete;
  assert(element.renderRoot.querySelector(".battery").textContent === "battery_3_bar" && element.renderRoot.querySelector(".battery").classList.contains("medium"));
  element.remote = Object.freeze({ ...element.remote, battery: 1 }); await element.updateComplete;
  assert(element.renderRoot.querySelector(".battery").textContent === "battery_1_bar" && element.renderRoot.querySelector(".battery").classList.contains("low"));
  element.side = "Right"; await element.updateComplete;
  const rightBadge = element.renderRoot.querySelector(".aid");
  assert(rightBadge.textContent.trim() === "R" && rightBadge.classList.contains("right") && rightBadge.getAttribute("aria-label") === "Right hearing aid");
  element.remove();
});

test("Battery indicators map protocol readings to discrete Material glyphs", () => {
  equal(
    [batteryIcon(-1), batteryIcon(0), batteryIcon(1), batteryIcon(2), batteryIcon(5), batteryIcon(9), batteryIcon(10), batteryIcon(12), batteryIcon(null)],
    ["battery_0_bar", "battery_0_bar", "battery_1_bar", "battery_2_bar", "battery_4_bar", "battery_6_bar", "battery_full", "battery_full", "battery_unknown"]
  );
});

test("Battery indicator colours follow the requested level thresholds", () => {
  equal(
    [batteryColor(-1), batteryColor(0), batteryColor(1), batteryColor(2), batteryColor(4), batteryColor(5), batteryColor(10), batteryColor(12), batteryColor(null)],
    ["low", "low", "low", "low", "medium", "medium", "high", "high", "unknown"]
  );
});

test("Remote volume levels convert from protocol units to dB", () => {
  equal([-128, -127, 0].map(volumeToDb), [-48, -47.625, 0]);
});

test("Settings form keeps visible labels and emits a composed USB event", async () => {
  const element = document.createElement("settings-dialog"); document.querySelector("#fixtures").append(element); element.ready = true; await element.updateComplete;
  const labels = [...element.renderRoot.querySelectorAll("label, legend")];
  assert(labels[0].textContent.includes("USB Audio Class") && labels[1].querySelector("span").textContent === "Volume range");
  let detail; element.addEventListener("usb-update", (event) => { detail = event.detail; });
  element.renderRoot.querySelector("form").dispatchEvent(new SubmitEvent("submit", { bubbles: true, cancelable: true }));
  equal(detail, { uacVersion: 2, minimumDb: -60, maximumDb: 0 }); element.remove();
});

test("USB volume uses a dual slider that preserves ordered bounds and submitted values", async () => {
  const element = document.createElement("settings-dialog"); document.querySelector("#fixtures").append(element); element.ready = true; await element.updateComplete;
  const sliders = [...element.renderRoot.querySelectorAll('input[type="range"]')];
  equal(sliders.map((slider) => slider.getAttribute("aria-label")), ["Minimum volume", "Maximum volume"]);
  equal(sliders.map((slider) => [Number(slider.min), Number(slider.max)]), [[-127, 0], [-127, 0]]);
  sliders[0].value = "-20"; sliders[0].dispatchEvent(new InputEvent("input", { bubbles: true })); await element.updateComplete;
  sliders[1].value = "-10"; sliders[1].dispatchEvent(new InputEvent("input", { bubbles: true })); await element.updateComplete;
  equal([element.minimumDb, element.maximumDb], [-20, -10]);
  equal([...element.renderRoot.querySelectorAll("output")].map((output) => output.textContent), ["-20", "-10"]);
  assert(!element.renderRoot.querySelector(".range-field").textContent.includes("dB"));
  sliders[0].value = "0"; sliders[0].dispatchEvent(new InputEvent("input", { bubbles: true })); await element.updateComplete;
  equal([element.minimumDb, element.maximumDb], [-11, -10]);
  sliders[1].value = "-127"; sliders[1].dispatchEvent(new InputEvent("input", { bubbles: true })); await element.updateComplete;
  equal([element.minimumDb, element.maximumDb], [-11, -10]);
  let detail; element.addEventListener("usb-update", (event) => { detail = event.detail; });
  element.renderRoot.querySelector("form").dispatchEvent(new SubmitEvent("submit", { bubbles: true, cancelable: true }));
  equal(detail, { uacVersion: 2, minimumDb: -11, maximumDb: -10 }); element.remove();
});

test("USB save is enabled only while device settings have been changed", async () => {
  const element = document.createElement("settings-dialog"); document.querySelector("#fixtures").append(element);
  element.ready = true;
  element.usbInfo = { uacVersion: 2, minimumDb: -60, maximumDb: 0 };
  await element.updateComplete;
  const save = element.renderRoot.querySelector('[aria-label="Save USB settings"]');
  const uac = element.renderRoot.querySelector("select");
  const minimum = element.renderRoot.querySelector('[aria-label="Minimum volume"]');
  assert(save.disabled);
  uac.value = "1"; uac.dispatchEvent(new Event("change", { bubbles: true })); await element.updateComplete;
  assert(!save.disabled);
  uac.value = "2"; uac.dispatchEvent(new Event("change", { bubbles: true })); await element.updateComplete;
  assert(save.disabled);
  minimum.value = "-61"; minimum.dispatchEvent(new InputEvent("input", { bubbles: true })); await element.updateComplete;
  assert(!save.disabled);
  minimum.value = "-60"; minimum.dispatchEvent(new InputEvent("input", { bubbles: true })); await element.updateComplete;
  assert(save.disabled);
  element.usbInfo = null; await element.updateComplete;
  assert(save.disabled);
  element.remove();
});

test("Settings dialog exposes disabled states until the adapter is ready", async () => {
  const element = document.createElement("settings-dialog"); document.querySelector("#fixtures").append(element); await element.updateComplete;
  assert([...element.renderRoot.querySelectorAll("button")].some((button) => button.disabled)); element.ready = true; await element.updateComplete;
  assert(!element.renderRoot.querySelector("button.control-button").disabled); element.remove();
});

test("Every settings dialog button is icon-only with a native tooltip and accessible name", async () => {
  const element = document.createElement("settings-dialog"); document.querySelector("#fixtures").append(element);
  element.ready = true; element.intro = { audioStreamingEnabled: false, connectionsAllowed: false };
  element.remotes = [{ name: "Test Aid", address: "01:02:03:04:05:06", side: "Left", paired: true }];
  await element.updateComplete;
  const buttons = [...element.renderRoot.querySelectorAll("button")];
  assert(buttons.length === 9);
  for (const button of buttons) {
    assert(button.title && button.getAttribute("aria-label"));
    assert(button.children.length === 1 && button.firstElementChild.classList.contains("material-symbols-outlined"));
  }
  const unpair = element.renderRoot.querySelector('[aria-label="Unpair Test Aid"]');
  assert(unpair.classList.contains("danger") && unpair.querySelector(".material-symbols-outlined").textContent === "delete");
  assert(!element.renderRoot.querySelector("footer"));
  element.remove();
});

test("Pairing dialog emits candidate selection from keyboard-operable buttons", async () => {
  const element = document.createElement("pairing-dialog"); document.querySelector("#fixtures").append(element);
  const candidate = { name: "Nearby", address: "01:02:03:04:05:06", addressType: 1, rssi: -50 }; element.candidates = [candidate]; await element.updateComplete;
  let selected; element.addEventListener("pairing-select", (event) => { selected = event.detail.candidate; });
  const button = element.renderRoot.querySelector("button.candidate"); assert(button.getAttribute("aria-label") === "Pair Nearby" && button.querySelector(".signal .material-symbols-outlined")?.textContent === "bluetooth_connected"); button.click(); equal(selected, candidate); element.remove();
});

test("Adapter log expands, exposes accessible actions, and auto-scrolls", async () => {
  const element = document.createElement("adapter-log"); document.querySelector("#fixtures").append(element); element.entries = ["one", "two"]; element.expanded = true; await element.updateComplete;
  assert(element.renderRoot.querySelector('[role="log"]').textContent.includes("two"));
  equal([...element.renderRoot.querySelectorAll("button")].map((button) => button.getAttribute("aria-label")), ["Copy adapter log", "Download adapter log", "Clear adapter log", "Collapse adapter log"]); element.remove();
});

test("The subsetted Material Symbols font is locally available", async () => {
  await document.fonts.load('24px "Material Symbols Outlined"');
  assert(document.fonts.check('24px "Material Symbols Outlined"'));
});

async function run() {
  const results = document.querySelector("#results");
  let passed = 0;
  const failures = [];
  for (const { name, body } of tests) {
    const item = document.createElement("li");
    try {
      await body();
      item.className = "pass";
      item.textContent = name;
      passed += 1;
    } catch (error) {
      item.className = "fail";
      item.innerHTML = `${name}<br><code>${String(error?.stack || error)}</code>`;
      failures.push({ name, error: String(error?.stack || error) });
    }
    results.append(item);
  }
  const summary = document.querySelector("#summary");
  summary.textContent = `${passed}/${tests.length} passed`;
  summary.dataset.state = failures.length ? "failed" : "passed";
  document.title = failures.length ? `FAILED: ${passed}/${tests.length}` : `PASS: ${passed}/${tests.length}`;
  globalThis.__picoAshaTestResults = { passed, total: tests.length, failures };
}

void run();
