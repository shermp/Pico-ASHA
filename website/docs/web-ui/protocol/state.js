import { EventType, PacketType, StatusType } from "./constants.js";
import { parseROP } from "./codec.js";

const cacheFields = ["name", "manufacturer", "model", "firmware", "software", "side", "sideValue", "mode", "modeValue", "audioFormat"];

function remoteSkeleton(connectionId, address = "00:00:00:00:00:00") {
  return {
    connectionId,
    hciHandle: 0,
    address,
    connected: true,
    paired: false,
    psm: 0,
    l2capCid: 0,
    name: "",
    manufacturer: "",
    model: "",
    firmware: "",
    software: "",
    side: "Unknown",
    sideValue: 2,
    mode: "Unknown",
    modeValue: 2,
    streaming: false,
    volume: -128,
    muted: true,
    battery: 0,
    audioFormat: "Unknown",
  };
}

function eventSucceeded(event) {
  return event.statusType === StatusType.Success || event.status === 0;
}

export class AdapterState {
  constructor() {
    this.remoteMap = new Map();
    this.cache = new Map();
    this.advertMap = new Map();
    this.timingSamples = [];
    this.snapshot = this.makeSnapshot();
  }

  makeSnapshot(overrides = {}) {
    return Object.freeze({
      intro: this.snapshot?.intro ?? null,
      usbInfo: this.snapshot?.usbInfo ?? null,
      remotes: Object.freeze([...this.remoteMap.values()].map((remote) => Object.freeze({ ...remote })).sort((a, b) => a.sideValue - b.sideValue || a.connectionId - b.connectionId)),
      adverts: Object.freeze([...this.advertMap.values()].map((advert) => Object.freeze({ ...advert })).sort((a, b) => b.rssi - a.rssi)),
      timing: this.snapshot?.timing ?? null,
      ...overrides,
    });
  }

  resetSession() {
    for (const remote of this.remoteMap.values()) {
      this.cacheRemote(remote);
    }
    this.remoteMap.clear();
    this.advertMap.clear();
    this.timingSamples = [];
    this.snapshot = this.makeSnapshot({ intro: null, usbInfo: null, timing: null });
    return this.snapshot;
  }

  updateIntro(changes) {
    if (!this.snapshot.intro) {
      return this.snapshot;
    }
    this.snapshot = this.makeSnapshot({ intro: Object.freeze({ ...this.snapshot.intro, ...changes }) });
    return this.snapshot;
  }

  clearAdverts() {
    this.advertMap.clear();
    this.snapshot = this.makeSnapshot();
    return this.snapshot;
  }

  cacheRemote(remote) {
    if (!remote.address || remote.address === "00:00:00:00:00:00") {
      return;
    }
    const cached = {};
    for (const field of cacheFields) {
      cached[field] = remote[field];
    }
    this.cache.set(remote.address, cached);
  }

  updateRemote(connectionId, changes) {
    const current = this.remoteMap.get(connectionId) ?? remoteSkeleton(connectionId, changes.address);
    const cached = this.cache.get(changes.address ?? current.address) ?? {};
    const remote = { ...current, ...cached, ...changes };
    this.remoteMap.set(connectionId, remote);
    this.cacheRemote(remote);
  }

  apply(packet) {
    if (packet.kind === "intro") {
      this.snapshot = this.makeSnapshot({ intro: Object.freeze({ ...packet }) });
      return this.snapshot;
    }
    if (packet.kind === "usb-info") {
      this.snapshot = this.makeSnapshot({ usbInfo: Object.freeze({ ...packet }) });
      return this.snapshot;
    }
    if (packet.kind === "remote-info") {
      if (packet.connected) {
        this.updateRemote(packet.connectionId, packet);
      } else {
        const old = this.remoteMap.get(packet.connectionId);
        if (old) {
          this.cacheRemote({ ...old, ...packet });
        }
        this.remoteMap.delete(packet.connectionId);
      }
      this.snapshot = this.makeSnapshot();
      return this.snapshot;
    }
    if (packet.kind === "advert") {
      if (packet.isHearingAid) {
        this.advertMap.set(packet.address, packet);
      }
      this.snapshot = this.makeSnapshot();
      return this.snapshot;
    }
    if (packet.kind === "event") {
      this.applyEvent(packet);
      this.snapshot = this.makeSnapshot();
    }
    return this.snapshot;
  }

  applyEvent(event) {
    const connectionId = event.header.connectionId;
    if (event.eventType === EventType.RemoteConnected && eventSucceeded(event)) {
      this.updateRemote(connectionId, { ...event.connection, connectionId, connected: true });
      return;
    }
    if (event.eventType === EventType.RemoteDisconnected) {
      const remote = this.remoteMap.get(connectionId);
      if (remote) {
        this.cacheRemote(remote);
      }
      this.remoteMap.delete(connectionId);
      return;
    }
    if (event.eventType === EventType.G722EncodeTimings) {
      this.timingSamples.push(...event.encodeTimings);
      if (this.timingSamples.length >= 1000) {
        const batch = this.timingSamples.splice(0, 1000);
        const sum = batch.reduce((total, value) => total + value, 0);
        this.snapshot = { ...this.snapshot, timing: Object.freeze({ count: 1000, minimum: Math.min(...batch), average: sum / batch.length, maximum: Math.max(...batch) }) };
      }
      return;
    }
    if (!eventSucceeded(event) || !this.remoteMap.has(connectionId)) {
      return;
    }

    const updates = {};
    switch (event.eventType) {
      case EventType.DeletePair: updates.paired = false; break;
      case EventType.PairAndBond: updates.paired = true; break;
      case EventType.ROPRead: {
        const rop = parseROP(event.rop);
        Object.assign(updates, rop, { audioFormat: rop.supportsG72224 ? "G.722 @ 24 kHz" : "G.722 @ 16 kHz" });
        break;
      }
      case EventType.PSMRead: updates.psm = event.psm; break;
      case EventType.DeviceNameRead: updates.name = event.text; break;
      case EventType.ManufacturerRead: updates.manufacturer = event.text; break;
      case EventType.ModelRead: updates.model = event.text; break;
      case EventType.FirmwareRead: updates.firmware = event.text; break;
      case EventType.SoftwareRead: updates.software = event.text; break;
      case EventType.L2CAPConnected: updates.l2capCid = event.cid; break;
      case EventType.L2CAPDisconnected: updates.l2capCid = 0; updates.streaming = false; break;
      case EventType.ASPStart:
      case EventType.StreamReady: updates.streaming = true; break;
      case EventType.ASPStop:
      case EventType.StreamPause: updates.streaming = false; break;
      case EventType.AudioVolume: updates.volume = event.volume; updates.muted = event.volume === -128; break;
      case EventType.MFIBatteryRead: updates.battery = event.battery; break;
      default: return;
    }
    this.updateRemote(connectionId, updates);
  }
}
