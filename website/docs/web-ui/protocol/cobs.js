import { MAX_PACKET_BYTES } from "./constants.js";

export class CobsError extends Error {}

export function cobsEncode(input) {
  const source = input instanceof Uint8Array ? input : new Uint8Array(input);
  const output = new Uint8Array(source.length + Math.ceil(source.length / 254) + 1);
  let codeIndex = 0;
  let outputIndex = 1;
  let code = 1;

  for (const byte of source) {
    if (byte === 0) {
      output[codeIndex] = code;
      codeIndex = outputIndex;
      outputIndex += 1;
      code = 1;
    } else {
      output[outputIndex] = byte;
      outputIndex += 1;
      code += 1;
      if (code === 0xff) {
        output[codeIndex] = code;
        codeIndex = outputIndex;
        outputIndex += 1;
        code = 1;
      }
    }
  }

  output[codeIndex] = code;
  return output.slice(0, outputIndex);
}

export function cobsDecode(input) {
  const source = input instanceof Uint8Array ? input : new Uint8Array(input);
  if (source.length === 0) {
    throw new CobsError("COBS frame is empty");
  }

  const output = [];
  let index = 0;
  while (index < source.length) {
    const code = source[index];
    if (code === 0) {
      throw new CobsError("COBS frame contains an unexpected delimiter");
    }
    index += 1;
    const end = index + code - 1;
    if (end > source.length) {
      throw new CobsError("COBS code extends past the frame boundary");
    }
    while (index < end) {
      output.push(source[index]);
      index += 1;
    }
    if (code !== 0xff && index < source.length) {
      output.push(0);
    }
  }
  return Uint8Array.from(output);
}

export function framePacket(packet) {
  const encoded = cobsEncode(packet);
  const frame = new Uint8Array(encoded.length + 2);
  frame[0] = 0;
  frame.set(encoded, 1);
  frame[frame.length - 1] = 0;
  return frame;
}

export class CobsFrameDecoder {
  constructor({ maxPacketBytes = MAX_PACKET_BYTES, onError = () => {} } = {}) {
    this.maxPacketBytes = maxPacketBytes;
    this.maxEncodedBytes = maxPacketBytes + Math.ceil(maxPacketBytes / 254) + 1;
    this.onError = onError;
    this.buffer = [];
    this.discarding = false;
  }

  reset() {
    this.buffer = [];
    this.discarding = false;
  }

  push(chunk) {
    const frames = [];
    for (const byte of chunk) {
      if (byte === 0) {
        if (this.discarding) {
          this.discarding = false;
          this.buffer = [];
          continue;
        }
        if (this.buffer.length === 0) {
          continue;
        }
        try {
          const decoded = cobsDecode(Uint8Array.from(this.buffer));
          if (decoded.length > this.maxPacketBytes) {
            throw new CobsError(`Decoded packet exceeds ${this.maxPacketBytes} bytes`);
          }
          frames.push(decoded);
        } catch (error) {
          this.onError(error);
        }
        this.buffer = [];
        continue;
      }

      if (this.discarding) {
        continue;
      }
      this.buffer.push(byte);
      if (this.buffer.length > this.maxEncodedBytes) {
        this.onError(new CobsError(`Encoded frame exceeds ${this.maxEncodedBytes} bytes`));
        this.buffer = [];
        this.discarding = true;
      }
    }
    return frames;
  }
}
