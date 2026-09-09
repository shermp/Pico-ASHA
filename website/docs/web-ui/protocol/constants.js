// Keep these values aligned with include/asha_comms.cpp in the firmware protocol.
export const USB_FILTER = Object.freeze({ usbVendorId: 0xcafe, usbProductId: 16401 });

export const PacketType = Object.freeze({
  Intro: 0,
  RemoteInfo: 1,
  Event: 2,
  HCI: 3,
  Command: 4,
  Advert: 5,
  USBInfo: 6,
});

export const PACKET_SIZE = Object.freeze({
  Header: 8,
  Intro: 16,
  RemoteInfo: 192,
  Event: 44,
  Command: 20,
  Advert: 48,
  USBInfo: 16,
});

export const IntroFlag = Object.freeze({
  ConnectionsAllowed: 1 << 0,
  AudioStreamingEnabled: 1 << 1,
  UAC2: 1 << 2,
});

export const Side = Object.freeze({ Left: 0, Right: 1, Unset: 2 });
export const Mode = Object.freeze({ Mono: 0, Binaural: 1, Unset: 2 });

export const StatusType = Object.freeze({
  Success: 0,
  PicoASHA: 1,
  BTstack: 2,
  ATT: 3,
  L2CAP: 4,
  SecurityManager: 5,
});

export const EventType = Object.freeze({
  ShortLog: 0,
  PicoASHAInit: 1,
  DeletePair: 2,
  RemoteConnected: 3,
  RemoteDisconnected: 4,
  DiscoverServices: 5,
  PairAndBond: 6,
  DataLengthExtension: 7,
  DiscoverASHACharacteristics: 8,
  ROPRead: 9,
  PSMRead: 10,
  DiscoverGAPCharacteristics: 11,
  DeviceNameRead: 12,
  DiscoverDISCharacteristics: 13,
  ManufacturerRead: 14,
  ModelRead: 15,
  FirmwareRead: 16,
  SoftwareRead: 17,
  L2CAPConnected: 18,
  L2CAPDisconnected: 19,
  ASPNotificationsEnabled: 20,
  ACPStart: 21,
  ACPStop: 22,
  ACPStatus: 23,
  ASPStart: 24,
  ASPStop: 25,
  ASPError: 26,
  StreamReady: 27,
  StreamPause: 28,
  AudioVolume: 29,
  DiscoverMFICharacteristics: 30,
  MFIBatteryRead: 31,
  MFIBatteryNotificationsEnabled: 32,
  G722EncodeTimings: 33,
});

export const EVENT_NAMES = Object.freeze(Object.fromEntries(
  Object.entries(EventType).map(([name, value]) => [value, name]),
));

export const Command = Object.freeze({
  HCIDump: 0,
  DeletePair: 1,
  Restart: 2,
  AllowConnect: 3,
  AudioStreaming: 4,
  IntroPacket: 5,
  PairBond: 6,
  USBSettings: 7,
});

export const COMMAND_NAMES = Object.freeze(Object.fromEntries(
  Object.entries(Command).map(([name, value]) => [value, name]),
));

export const CommandStatus = Object.freeze({ Ok: 0, Error: 1 });

export const SERIAL_OPTIONS = Object.freeze({
  baudRate: 115200,
  dataBits: 8,
  stopBits: 1,
  parity: "none",
  bufferSize: 4096,
  flowControl: "none",
});

export const MAX_PACKET_BYTES = 255;
export const COMMAND_TIMEOUT_MS = 2500;
export const INTRO_TIMEOUT_MS = 1000;
export const RECONNECT_DELAY_MS = 1000;
