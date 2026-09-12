import {
  Command,
  CommandStatus,
  IntroFlag,
  Mode,
  PACKET_SIZE,
  PacketType,
  Side,
} from "./constants.js";

const decoder = new TextDecoder();

export class PacketError extends Error {}

function requireLength(bytes, length, name) {
  if (bytes.length !== length) {
    throw new PacketError(`${name} packet must be ${length} bytes; received ${bytes.length}`);
  }
}

function readString(bytes) {
  const nul = bytes.indexOf(0);
  return decoder.decode(nul < 0 ? bytes : bytes.subarray(0, nul));
}

export function formatAddress(bytes) {
  return [...bytes].map((byte) => byte.toString(16).padStart(2, "0")).join(":");
}

export function parseAddress(value) {
  const parts = String(value).split(":");
  if (parts.length !== 6 || parts.some((part) => !/^[0-9a-f]{2}$/i.test(part))) {
    throw new PacketError(`Invalid Bluetooth address: ${value}`);
  }
  return Uint8Array.from(parts.map((part) => Number.parseInt(part, 16)));
}

export function decodeHeader(bytes) {
  if (bytes.length < PACKET_SIZE.Header) {
    throw new PacketError(`Packet header must be ${PACKET_SIZE.Header} bytes`);
  }
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const header = {
    type: view.getUint8(0),
    length: view.getUint8(1),
    connectionId: view.getUint16(2, true),
    timestampMs: view.getUint32(4, true),
  };
  if (header.length !== bytes.length) {
    throw new PacketError(`Header length ${header.length} does not match received length ${bytes.length}`);
  }
  return header;
}

function decodeIntro(bytes, header) {
  requireLength(bytes, PACKET_SIZE.Intro, "Intro");
  const view = new DataView(bytes.buffer, bytes.byteOffset + 8, 8);
  const flags = view.getUint16(4, true);
  return {
    header,
    kind: "intro",
    version: `${view.getUint8(0)}.${view.getUint8(1)}.${view.getUint8(2)}`,
    versionParts: [view.getUint8(0), view.getUint8(1), view.getUint8(2)],
    numberConnected: view.getInt8(3),
    flags,
    connectionsAllowed: Boolean(flags & IntroFlag.ConnectionsAllowed),
    audioStreamingEnabled: Boolean(flags & IntroFlag.AudioStreamingEnabled),
    uacVersion: flags & IntroFlag.UAC2 ? 2 : 1,
    reserved: view.getUint16(6, true),
  };
}

function decodeRemoteInfo(bytes, header) {
  requireLength(bytes, PACKET_SIZE.RemoteInfo, "RemoteInfo");
  const payload = bytes.subarray(8);
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  return {
    header,
    kind: "remote-info",
    connectionId: view.getUint16(0, true),
    hciHandle: view.getUint16(2, true),
    address: formatAddress(payload.subarray(4, 10)),
    connected: Boolean(view.getUint8(10)),
    paired: Boolean(view.getUint8(11)),
    psm: view.getUint16(12, true),
    l2capCid: view.getUint16(14, true),
    name: readString(payload.subarray(16, 48)),
    manufacturer: readString(payload.subarray(48, 80)),
    model: readString(payload.subarray(80, 112)),
    firmware: readString(payload.subarray(112, 144)),
    software: readString(payload.subarray(144, 176)),
    sideValue: view.getUint8(176),
    side: ["Left", "Right", "Unknown"][view.getUint8(176)] ?? "Unknown",
    modeValue: view.getUint8(177),
    mode: ["Mono", "Binaural", "Unknown"][view.getUint8(177)] ?? "Unknown",
    streaming: Boolean(view.getUint8(178)),
    volume: view.getInt8(179),
    muted: view.getInt8(179) === -128,
    battery: view.getUint8(180),
  };
}

function decodeEvent(bytes, header) {
  requireLength(bytes, PACKET_SIZE.Event, "Event");
  const payload = bytes.subarray(8);
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const data = payload.subarray(4, 36);
  const dataView = new DataView(data.buffer, data.byteOffset, data.byteLength);
  // Event data is a firmware union; expose each supported interpretation and let state select by event type.
  return {
    header,
    kind: "event",
    eventType: view.getUint8(0),
    statusType: view.getUint8(1),
    status: view.getUint8(2),
    reason: view.getUint8(3),
    data: Uint8Array.from(data),
    connection: { address: formatAddress(data.subarray(0, 6)), hciHandle: dataView.getUint16(6, true) },
    psm: dataView.getUint8(0),
    aspNotification: dataView.getInt8(0),
    volume: dataView.getInt8(0),
    cid: dataView.getUint16(0, true),
    credits: dataView.getUint16(0, true),
    rop: Uint8Array.from(data.subarray(0, 17)),
    text: readString(data),
    battery: dataView.getUint8(0),
    encodeTimings: Array.from({ length: 10 }, (_, index) => dataView.getInt16(index * 2, true)),
  };
}

function decodeCommand(bytes, header) {
  requireLength(bytes, PACKET_SIZE.Command, "Command");
  const payload = bytes.subarray(8);
  return {
    header,
    kind: "command",
    command: payload[0],
    status: payload[1],
    accepted: payload[1] === CommandStatus.Ok,
    data: Uint8Array.from(payload.subarray(2)),
  };
}

function decodeAdvert(bytes, header) {
  requireLength(bytes, PACKET_SIZE.Advert, "Advert");
  const payload = bytes.subarray(8);
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  return {
    header,
    kind: "advert",
    address: formatAddress(payload.subarray(0, 6)),
    addressType: view.getUint8(6),
    rssi: view.getInt8(7),
    isHearingAid: Boolean(view.getUint8(8)),
    name: readString(payload.subarray(12, 40)),
  };
}

function decodeUSBInfo(bytes, header) {
  requireLength(bytes, PACKET_SIZE.USBInfo, "USBInfo");
  const view = new DataView(bytes.buffer, bytes.byteOffset + 8, 8);
  return {
    header,
    kind: "usb-info",
    uacVersion: view.getUint16(0, true),
    minimumRaw: view.getInt16(2, true),
    maximumRaw: view.getInt16(4, true),
    minimumDb: view.getInt16(2, true) / 96,
    maximumDb: view.getInt16(4, true) / 96,
    reserved: view.getUint16(6, true),
  };
}

export function decodePacket(input) {
  const bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
  const header = decodeHeader(bytes);
  switch (header.type) {
    case PacketType.Intro: return decodeIntro(bytes, header);
    case PacketType.RemoteInfo: return decodeRemoteInfo(bytes, header);
    case PacketType.Event: return decodeEvent(bytes, header);
    case PacketType.HCI:
      if (bytes.length < PACKET_SIZE.Header) {
        throw new PacketError("HCI packet is shorter than its header");
      }
      return { header, kind: "hci", data: Uint8Array.from(bytes.subarray(8)) };
    case PacketType.Command: return decodeCommand(bytes, header);
    case PacketType.Advert: return decodeAdvert(bytes, header);
    case PacketType.USBInfo: return decodeUSBInfo(bytes, header);
    default: throw new PacketError(`Unknown packet type ${header.type}`);
  }
}

function writeHeader(view, type, length, connectionId, timestampMs) {
  view.setUint8(0, type);
  view.setUint8(1, length);
  view.setUint16(2, connectionId, true);
  view.setUint32(4, timestampMs >>> 0, true);
}

export function validateUSBSettings({ uacVersion, minimumDb, maximumDb }) {
  const min = Number(minimumDb);
  const max = Number(maximumDb);
  if (![1, 2].includes(Number(uacVersion))) {
    throw new PacketError("USB Audio Class must be 1 or 2");
  }
  if (!Number.isFinite(min) || !Number.isFinite(max) || min < -127 || max > 0 || min >= max) {
    throw new PacketError("Volume range must be within -127 to 0 dB, with minimum below maximum");
  }
  const minimumRaw = Math.round(min * 96);
  const maximumRaw = Math.round(max * 96);
  if (minimumRaw < -12192 || maximumRaw > 0) {
    throw new PacketError("Volume values exceed the adapter's fixed-point range");
  }
  return { uacVersion: Number(uacVersion), minimumDb: min, maximumDb: max, minimumRaw, maximumRaw };
}

export function encodeCommandPacket(command, data = {}, options = {}) {
  if (!Object.values(Command).includes(command)) {
    throw new PacketError(`Unknown command ${command}`);
  }
  const bytes = new Uint8Array(PACKET_SIZE.Command);
  // Commands use one fixed-size union packet; the zero-filled bytes are unused by the selected command.
  const view = new DataView(bytes.buffer);
  writeHeader(view, PacketType.Command, PACKET_SIZE.Command, options.connectionId ?? 0, options.timestampMs ?? 0);
  view.setUint8(8, command);
  view.setUint8(9, options.status ?? CommandStatus.Ok);

  switch (command) {
    case Command.HCIDump:
      view.setUint8(10, data.enabled ? 1 : 0);
      break;
    case Command.DeletePair:
      view.setUint16(10, data.connectionId ?? options.connectionId ?? 0, true);
      break;
    case Command.AllowConnect:
      view.setUint8(10, data.enabled ? 1 : 0);
      break;
    case Command.AudioStreaming:
      view.setUint8(10, data.enabled ? 1 : 0);
      break;
    case Command.PairBond: {
      const address = data.address instanceof Uint8Array ? data.address : parseAddress(data.address);
      bytes.set(address, 10);
      view.setUint8(16, data.addressType ?? 0);
      break;
    }
    case Command.USBSettings: {
      const settings = validateUSBSettings(data);
      view.setUint16(10, settings.uacVersion, true);
      view.setInt16(12, settings.minimumRaw, true);
      view.setInt16(14, settings.maximumRaw, true);
      break;
    }
    default:
      break;
  }
  return bytes;
}

export function parseROP(rop) {
  if (!(rop instanceof Uint8Array) || rop.length !== 17) {
    throw new PacketError("ASHA read-only properties must be exactly 17 bytes");
  }
  const view = new DataView(rop.buffer, rop.byteOffset, rop.byteLength);
  const capability = rop[1];
  const codecs = view.getUint16(15, true);
  return {
    version: rop[0],
    sideValue: capability & 1 ? Side.Right : Side.Left,
    side: capability & 1 ? "Right" : "Left",
    modeValue: capability & 2 ? Mode.Binaural : Mode.Mono,
    mode: capability & 2 ? "Binaural" : "Mono",
    manufacturerId: view.getUint16(2, true),
    uniqueId: formatAddress(rop.subarray(4, 10)),
    leCocSupported: Boolean(rop[10] & 1),
    renderDelayMs: view.getUint16(11, true),
    supportsG72216: Boolean(codecs & 0x0002),
    supportsG72224: Boolean(codecs & 0x0004),
  };
}
