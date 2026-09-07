import { EVENT_NAMES, StatusType } from "./constants.js";

const picoAsha = ["Success", "Invalid runtime settings", "Maximum connected devices reached", "ASHA service not found"];
const att = {
  0x00: "Success", 0x01: "Invalid handle", 0x02: "Read not permitted", 0x03: "Write not permitted",
  0x04: "Invalid PDU", 0x05: "Insufficient authentication", 0x06: "Request not supported",
  0x07: "Invalid offset", 0x08: "Insufficient authorization", 0x09: "Prepare queue full",
  0x0a: "Attribute not found", 0x0b: "Attribute not long", 0x0c: "Insufficient encryption key size",
  0x0d: "Invalid attribute value length", 0x0e: "Unlikely error", 0x0f: "Insufficient encryption",
  0x10: "Unsupported group type", 0x11: "Insufficient resources", 0x13: "Value not allowed",
  0x1f: "HCI disconnect received", 0x70: "Bonding information missing", 0x7e: "Data mismatch",
  0x7f: "Timeout", 0xfc: "Write request rejected", 0xfd: "Descriptor improperly configured",
  0xfe: "Procedure already in progress", 0xff: "Out of range",
};
const l2cap = {
  0x00: "Success", 0x02: "SPSM not supported", 0x04: "No resources available",
  0x05: "Insufficient authentication", 0x06: "Insufficient authorization", 0x07: "Encryption key too short",
  0x08: "Insufficient encryption", 0x09: "Invalid source CID", 0x0a: "Source CID already allocated",
  0x0b: "Unacceptable parameters",
};
const security = {
  0x00: "Reserved", 0x01: "Passkey entry failed", 0x02: "Out-of-band data unavailable",
  0x03: "Authentication requirements", 0x04: "Confirm value failed", 0x05: "Pairing not supported",
  0x06: "Encryption key size", 0x07: "Command not supported", 0x08: "Unspecified reason",
  0x09: "Repeated attempts", 0x0a: "Invalid parameters", 0x0b: "DHKey check failed",
  0x0c: "Numeric comparison failed", 0x0d: "BR/EDR pairing in progress",
  0x0e: "Cross-transport key derivation not allowed", 0x0f: "Key rejected",
};
const btstack = {
  0x00: "Success", 0x01: "Unknown HCI command", 0x02: "Unknown connection identifier", 0x03: "Hardware failure",
  0x04: "Page timeout", 0x05: "Authentication failure", 0x06: "PIN or key missing", 0x07: "Memory capacity exceeded",
  0x08: "Connection timeout", 0x09: "Connection limit exceeded", 0x0a: "Synchronous connection limit exceeded",
  0x0b: "ACL connection already exists", 0x0c: "Command disallowed", 0x0d: "Connection rejected: limited resources",
  0x0e: "Connection rejected: security", 0x0f: "Connection rejected: unacceptable address",
  0x10: "Connection accept timeout exceeded", 0x11: "Unsupported feature or parameter", 0x12: "Invalid HCI command parameters",
  0x13: "Remote user ended connection", 0x14: "Remote device ended connection: low resources",
  0x15: "Remote device ended connection: powered off", 0x16: "Connection ended by local host", 0x17: "Repeated attempts",
  0x18: "Pairing not allowed", 0x19: "Unknown LMP PDU", 0x1a: "Unsupported remote feature",
  0x1b: "SCO offset rejected", 0x1c: "SCO interval rejected", 0x1d: "SCO air mode rejected",
  0x1e: "Invalid LMP or link-layer parameters", 0x1f: "Unspecified error", 0x20: "Unsupported LMP or link-layer parameter",
  0x21: "Role change not allowed", 0x22: "LMP or link-layer response timeout", 0x23: "LMP transaction collision",
  0x24: "LMP PDU not allowed", 0x25: "Encryption mode not acceptable", 0x26: "Link key cannot be changed",
  0x27: "Requested QoS not supported", 0x28: "Instant passed", 0x29: "Pairing with unit key not supported",
  0x2a: "Different transaction collision", 0x2b: "Reserved error", 0x2c: "Unacceptable QoS parameter",
  0x2d: "QoS rejected", 0x2e: "Channel classification not supported", 0x2f: "Insufficient security",
  0x30: "Parameter outside mandatory range", 0x32: "Role switch pending", 0x34: "Reserved slot violation",
  0x35: "Role switch failed", 0x36: "Extended inquiry response too large", 0x37: "Secure Simple Pairing not supported by host",
  0x38: "Host busy pairing", 0x39: "Connection rejected: no suitable channel", 0x3a: "Controller busy",
  0x3b: "Unacceptable connection parameters", 0x3c: "Directed advertising timeout", 0x3d: "Connection ended: MIC failure",
  0x3e: "Connection could not be established", 0x3f: "MAC connection failed", 0x40: "Coarse clock adjustment rejected",
  0x50: "Connection to Bluetooth daemon failed", 0x51: "System Bluetooth activation failed", 0x52: "Bluetooth power-on failed",
  0x53: "Bluetooth activation failed", 0x54: "BTstack not activated", 0x55: "BTstack busy",
  0x56: "Memory allocation failed", 0x57: "ACL buffers full", 0x60: "L2CAP command not understood",
  0x61: "L2CAP signalling MTU exceeded", 0x62: "Invalid CID in L2CAP request", 0x63: "L2CAP connection successful",
  0x64: "L2CAP connection pending", 0x65: "L2CAP PSM refused", 0x66: "L2CAP security refused",
  0x67: "L2CAP resources refused", 0x68: "L2CAP ERTM not supported", 0x69: "L2CAP timeout",
  0x6a: "L2CAP baseband disconnected", 0x6b: "L2CAP service already registered", 0x6c: "L2CAP data exceeds remote MTU",
  0x6d: "L2CAP service does not exist", 0x6e: "Local L2CAP CID does not exist", 0x6f: "Unknown L2CAP connection error",
  0x70: "RFCOMM multiplexer stopped", 0x71: "RFCOMM channel already registered", 0x72: "RFCOMM has no outgoing credits",
  0x73: "RFCOMM aggregate flow off", 0x74: "RFCOMM data exceeds MTU", 0x7f: "Remote rejected HFP audio connection",
  0x80: "SDP handle already registered", 0x81: "SDP query incomplete", 0x82: "SDP service not found",
  0x83: "Invalid SDP handle", 0x84: "SDP query busy", 0x90: "ATT indication in progress",
  0x91: "ATT indication timeout", 0x92: "ATT indication disconnected", 0x93: "GATT client not connected",
  0x94: "GATT client busy", 0x95: "GATT client in wrong state", 0x96: "GATT address context already exists",
  0x97: "GATT value too long", 0x98: "GATT notification not supported", 0x99: "GATT indication not supported",
  0xa0: "BNEP service already registered", 0xa1: "BNEP channel not connected", 0xa2: "BNEP data exceeds MTU",
  0xb0: "Unknown OBEX error", 0xb1: "OBEX connection failed", 0xb2: "OBEX disconnected",
  0xb3: "OBEX object not found", 0xb4: "OBEX request not acceptable", 0xb5: "OBEX request aborted",
  0xd0: "Mesh application-key index invalid",
};

export function describeStatus(statusType, status) {
  if (statusType === StatusType.Success || status === 0) {
    return "Success";
  }
  const tables = {
    [StatusType.PicoASHA]: picoAsha,
    [StatusType.BTstack]: btstack,
    [StatusType.ATT]: att,
    [StatusType.L2CAP]: l2cap,
    [StatusType.SecurityManager]: security,
  };
  return tables[statusType]?.[status] ?? `Unknown status 0x${status.toString(16).padStart(2, "0")}`;
}

export function describeEventError(event) {
  const name = EVENT_NAMES[event.eventType] ?? `Event ${event.eventType}`;
  const status = describeStatus(event.statusType, event.status);
  return event.reason ? `${name}: ${status} (reason 0x${event.reason.toString(16).padStart(2, "0")})` : `${name}: ${status}`;
}

export function describeSerialError(error) {
  const originalDetail = error?.originalMessage || error?.cause?.message || "";
  const detail = originalDetail ? ` Browser detail: ${originalDetail}` : "";
  if (error?.operation === "open") {
    return `The adapter was selected but its serial port could not be opened.${detail} Reconnect the adapter, then select it again.`;
  }
  if (error?.operation === "dtr") {
    return `The serial port opened, but the browser could not assert DTR.${detail} Reconnect the adapter and try again.`;
  }
  if (error?.name === "NotFoundError") {
    return "No adapter was selected. In Firefox 151+, complete the first-use Web Serial permission add-on gate, then select Pico-ASHA.";
  }
  if (error?.name === "SecurityError" || error?.name === "NotAllowedError") {
    return "Serial access was denied. Check the site permission; on managed Firefox, ask your administrator to allow Web Serial policy access.";
  }
  if (error?.name === "NetworkError") {
    return "The adapter could not be opened. Close other applications using its serial port and try again.";
  }
  return error?.message || "Unable to connect to the Pico-ASHA adapter.";
}

export function webSerialSupportMessage(options = {}) {
  const serial = Object.hasOwn(options, "serial") ? options.serial : globalThis.navigator?.serial;
  const secure = Object.hasOwn(options, "secure") ? options.secure : globalThis.isSecureContext;
  const hostname = Object.hasOwn(options, "hostname") ? options.hostname : globalThis.location?.hostname;
  const local = hostname === "localhost" || hostname === "127.0.0.1" || hostname === "[::1]";
  if (!secure && !local) {
    return "Web Serial requires HTTPS or localhost.";
  }
  if (!serial) {
    return "Web Serial is unavailable. Use Firefox 151+ desktop, current Chrome, or current Edge. On managed Firefox, Web Serial may be disabled by enterprise policy.";
  }
  return "";
}
