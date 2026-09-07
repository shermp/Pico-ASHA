export const BTSNOOP_FILE_HEADER = Uint8Array.of(
  0x62, 0x74, 0x73, 0x6e, 0x6f, 0x6f, 0x70, 0x00,
  0x00, 0x00, 0x00, 0x01,
  0x00, 0x00, 0x03, 0xea,
);

export const HCI_CAPTURE_LIMIT = 64 * 1024 * 1024;

export class HciCapture {
  constructor({ limit = HCI_CAPTURE_LIMIT, now = () => new Date() } = {}) {
    this.limit = limit;
    this.now = now;
    this.reset();
  }

  reset() {
    this.active = false;
    this.chunks = [];
    this.bytes = 0;
    this.blob = null;
    this.filename = "";
    this.limitReached = false;
  }

  start() {
    this.reset();
    this.active = true;
    this.chunks = [BTSNOOP_FILE_HEADER];
    this.bytes = BTSNOOP_FILE_HEADER.byteLength;
  }

  append(data) {
    if (!this.active) {
      return true;
    }
    const chunk = data instanceof Uint8Array ? Uint8Array.from(data) : new Uint8Array(data);
    if (this.bytes + chunk.byteLength > this.limit) {
      this.limitReached = true;
      this.stop();
      return false;
    }
    this.chunks.push(chunk);
    this.bytes += chunk.byteLength;
    return true;
  }

  stop() {
    if (!this.active && this.blob) {
      return this.blob;
    }
    this.active = false;
    this.blob = new Blob(this.chunks, { type: "application/vnd.bluetooth.btsnoop" });
    const stamp = this.now().toISOString().replace(/[:.]/g, "-");
    this.filename = `pico-asha-${stamp}.log`;
    return this.blob;
  }

  download(documentRef = globalThis.document, urlRef = globalThis.URL) {
    if (!this.blob || !documentRef || !urlRef) {
      return false;
    }
    const url = urlRef.createObjectURL(this.blob);
    try {
      const anchor = documentRef.createElement("a");
      anchor.href = url;
      anchor.download = this.filename;
      anchor.hidden = true;
      documentRef.body.append(anchor);
      anchor.click();
      anchor.remove();
      return true;
    } catch {
      return false;
    } finally {
      setTimeout(() => urlRef.revokeObjectURL(url), 1000);
    }
  }
}
