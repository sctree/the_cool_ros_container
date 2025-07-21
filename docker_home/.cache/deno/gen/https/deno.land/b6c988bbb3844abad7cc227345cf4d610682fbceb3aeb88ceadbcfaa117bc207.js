// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
import { copy } from "../bytes/copy.ts";
const DEFAULT_BUF_SIZE = 4096;
class AbstractBufBase {
  buf;
  usedBufferBytes = 0;
  err = null;
  constructor(buf){
    this.buf = buf;
  }
  /** Size returns the size of the underlying buffer in bytes. */ size() {
    return this.buf.byteLength;
  }
  /** Returns how many bytes are unused in the buffer. */ available() {
    return this.buf.byteLength - this.usedBufferBytes;
  }
  /** buffered returns the number of bytes that have been written into the
   * current buffer.
   */ buffered() {
    return this.usedBufferBytes;
  }
}
/** BufWriter implements buffering for an deno.Writer object.
 * If an error occurs writing to a Writer, no more data will be
 * accepted and all subsequent writes, and flush(), will return the error.
 * After all data has been written, the client should call the
 * flush() method to guarantee all data has been forwarded to
 * the underlying deno.Writer.
 */ export class BufWriter extends AbstractBufBase {
  #writer;
  /** return new BufWriter unless writer is BufWriter */ static create(writer, size = DEFAULT_BUF_SIZE) {
    return writer instanceof BufWriter ? writer : new BufWriter(writer, size);
  }
  constructor(writer, size = DEFAULT_BUF_SIZE){
    super(new Uint8Array(size <= 0 ? DEFAULT_BUF_SIZE : size));
    this.#writer = writer;
  }
  /** Discards any unflushed buffered data, clears any error, and
   * resets buffer to write its output to w.
   */ reset(w) {
    this.err = null;
    this.usedBufferBytes = 0;
    this.#writer = w;
  }
  /** Flush writes any buffered data to the underlying io.Writer. */ async flush() {
    if (this.err !== null) throw this.err;
    if (this.usedBufferBytes === 0) return;
    try {
      const p = this.buf.subarray(0, this.usedBufferBytes);
      let nwritten = 0;
      while(nwritten < p.length){
        nwritten += await this.#writer.write(p.subarray(nwritten));
      }
    } catch (e) {
      if (e instanceof Error) {
        this.err = e;
      }
      throw e;
    }
    this.buf = new Uint8Array(this.buf.length);
    this.usedBufferBytes = 0;
  }
  /** Writes the contents of `data` into the buffer.  If the contents won't fully
   * fit into the buffer, those bytes that can are copied into the buffer, the
   * buffer is the flushed to the writer and the remaining bytes are copied into
   * the now empty buffer.
   *
   * @return the number of bytes written to the buffer.
   */ async write(data) {
    if (this.err !== null) throw this.err;
    if (data.length === 0) return 0;
    let totalBytesWritten = 0;
    let numBytesWritten = 0;
    while(data.byteLength > this.available()){
      if (this.buffered() === 0) {
        // Large write, empty buffer.
        // Write directly from data to avoid copy.
        try {
          numBytesWritten = await this.#writer.write(data);
        } catch (e) {
          if (e instanceof Error) {
            this.err = e;
          }
          throw e;
        }
      } else {
        numBytesWritten = copy(data, this.buf, this.usedBufferBytes);
        this.usedBufferBytes += numBytesWritten;
        await this.flush();
      }
      totalBytesWritten += numBytesWritten;
      data = data.subarray(numBytesWritten);
    }
    numBytesWritten = copy(data, this.buf, this.usedBufferBytes);
    this.usedBufferBytes += numBytesWritten;
    totalBytesWritten += numBytesWritten;
    return totalBytesWritten;
  }
}
/** BufWriterSync implements buffering for a deno.WriterSync object.
 * If an error occurs writing to a WriterSync, no more data will be
 * accepted and all subsequent writes, and flush(), will return the error.
 * After all data has been written, the client should call the
 * flush() method to guarantee all data has been forwarded to
 * the underlying deno.WriterSync.
 */ export class BufWriterSync extends AbstractBufBase {
  #writer;
  /** return new BufWriterSync unless writer is BufWriterSync */ static create(writer, size = DEFAULT_BUF_SIZE) {
    return writer instanceof BufWriterSync ? writer : new BufWriterSync(writer, size);
  }
  constructor(writer, size = DEFAULT_BUF_SIZE){
    super(new Uint8Array(size <= 0 ? DEFAULT_BUF_SIZE : size));
    this.#writer = writer;
  }
  /** Discards any unflushed buffered data, clears any error, and
   * resets buffer to write its output to w.
   */ reset(w) {
    this.err = null;
    this.usedBufferBytes = 0;
    this.#writer = w;
  }
  /** Flush writes any buffered data to the underlying io.WriterSync. */ flush() {
    if (this.err !== null) throw this.err;
    if (this.usedBufferBytes === 0) return;
    try {
      const p = this.buf.subarray(0, this.usedBufferBytes);
      let nwritten = 0;
      while(nwritten < p.length){
        nwritten += this.#writer.writeSync(p.subarray(nwritten));
      }
    } catch (e) {
      if (e instanceof Error) {
        this.err = e;
      }
      throw e;
    }
    this.buf = new Uint8Array(this.buf.length);
    this.usedBufferBytes = 0;
  }
  /** Writes the contents of `data` into the buffer.  If the contents won't fully
   * fit into the buffer, those bytes that can are copied into the buffer, the
   * buffer is the flushed to the writer and the remaining bytes are copied into
   * the now empty buffer.
   *
   * @return the number of bytes written to the buffer.
   */ writeSync(data) {
    if (this.err !== null) throw this.err;
    if (data.length === 0) return 0;
    let totalBytesWritten = 0;
    let numBytesWritten = 0;
    while(data.byteLength > this.available()){
      if (this.buffered() === 0) {
        // Large write, empty buffer.
        // Write directly from data to avoid copy.
        try {
          numBytesWritten = this.#writer.writeSync(data);
        } catch (e) {
          if (e instanceof Error) {
            this.err = e;
          }
          throw e;
        }
      } else {
        numBytesWritten = copy(data, this.buf, this.usedBufferBytes);
        this.usedBufferBytes += numBytesWritten;
        this.flush();
      }
      totalBytesWritten += numBytesWritten;
      data = data.subarray(numBytesWritten);
    }
    numBytesWritten = copy(data, this.buf, this.usedBufferBytes);
    this.usedBufferBytes += numBytesWritten;
    totalBytesWritten += numBytesWritten;
    return totalBytesWritten;
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE2OC4wL2lvL2J1Zl93cml0ZXIudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyMiB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cblxuaW1wb3J0IHsgY29weSB9IGZyb20gXCIuLi9ieXRlcy9jb3B5LnRzXCI7XG5pbXBvcnQgdHlwZSB7IFdyaXRlciwgV3JpdGVyU3luYyB9IGZyb20gXCIuL3R5cGVzLmQudHNcIjtcblxuY29uc3QgREVGQVVMVF9CVUZfU0laRSA9IDQwOTY7XG5cbmFic3RyYWN0IGNsYXNzIEFic3RyYWN0QnVmQmFzZSB7XG4gIGJ1ZjogVWludDhBcnJheTtcbiAgdXNlZEJ1ZmZlckJ5dGVzID0gMDtcbiAgZXJyOiBFcnJvciB8IG51bGwgPSBudWxsO1xuXG4gIGNvbnN0cnVjdG9yKGJ1ZjogVWludDhBcnJheSkge1xuICAgIHRoaXMuYnVmID0gYnVmO1xuICB9XG5cbiAgLyoqIFNpemUgcmV0dXJucyB0aGUgc2l6ZSBvZiB0aGUgdW5kZXJseWluZyBidWZmZXIgaW4gYnl0ZXMuICovXG4gIHNpemUoKTogbnVtYmVyIHtcbiAgICByZXR1cm4gdGhpcy5idWYuYnl0ZUxlbmd0aDtcbiAgfVxuXG4gIC8qKiBSZXR1cm5zIGhvdyBtYW55IGJ5dGVzIGFyZSB1bnVzZWQgaW4gdGhlIGJ1ZmZlci4gKi9cbiAgYXZhaWxhYmxlKCk6IG51bWJlciB7XG4gICAgcmV0dXJuIHRoaXMuYnVmLmJ5dGVMZW5ndGggLSB0aGlzLnVzZWRCdWZmZXJCeXRlcztcbiAgfVxuXG4gIC8qKiBidWZmZXJlZCByZXR1cm5zIHRoZSBudW1iZXIgb2YgYnl0ZXMgdGhhdCBoYXZlIGJlZW4gd3JpdHRlbiBpbnRvIHRoZVxuICAgKiBjdXJyZW50IGJ1ZmZlci5cbiAgICovXG4gIGJ1ZmZlcmVkKCk6IG51bWJlciB7XG4gICAgcmV0dXJuIHRoaXMudXNlZEJ1ZmZlckJ5dGVzO1xuICB9XG59XG5cbi8qKiBCdWZXcml0ZXIgaW1wbGVtZW50cyBidWZmZXJpbmcgZm9yIGFuIGRlbm8uV3JpdGVyIG9iamVjdC5cbiAqIElmIGFuIGVycm9yIG9jY3VycyB3cml0aW5nIHRvIGEgV3JpdGVyLCBubyBtb3JlIGRhdGEgd2lsbCBiZVxuICogYWNjZXB0ZWQgYW5kIGFsbCBzdWJzZXF1ZW50IHdyaXRlcywgYW5kIGZsdXNoKCksIHdpbGwgcmV0dXJuIHRoZSBlcnJvci5cbiAqIEFmdGVyIGFsbCBkYXRhIGhhcyBiZWVuIHdyaXR0ZW4sIHRoZSBjbGllbnQgc2hvdWxkIGNhbGwgdGhlXG4gKiBmbHVzaCgpIG1ldGhvZCB0byBndWFyYW50ZWUgYWxsIGRhdGEgaGFzIGJlZW4gZm9yd2FyZGVkIHRvXG4gKiB0aGUgdW5kZXJseWluZyBkZW5vLldyaXRlci5cbiAqL1xuZXhwb3J0IGNsYXNzIEJ1ZldyaXRlciBleHRlbmRzIEFic3RyYWN0QnVmQmFzZSBpbXBsZW1lbnRzIFdyaXRlciB7XG4gICN3cml0ZXI6IFdyaXRlcjtcblxuICAvKiogcmV0dXJuIG5ldyBCdWZXcml0ZXIgdW5sZXNzIHdyaXRlciBpcyBCdWZXcml0ZXIgKi9cbiAgc3RhdGljIGNyZWF0ZSh3cml0ZXI6IFdyaXRlciwgc2l6ZTogbnVtYmVyID0gREVGQVVMVF9CVUZfU0laRSk6IEJ1ZldyaXRlciB7XG4gICAgcmV0dXJuIHdyaXRlciBpbnN0YW5jZW9mIEJ1ZldyaXRlciA/IHdyaXRlciA6IG5ldyBCdWZXcml0ZXIod3JpdGVyLCBzaXplKTtcbiAgfVxuXG4gIGNvbnN0cnVjdG9yKHdyaXRlcjogV3JpdGVyLCBzaXplOiBudW1iZXIgPSBERUZBVUxUX0JVRl9TSVpFKSB7XG4gICAgc3VwZXIobmV3IFVpbnQ4QXJyYXkoc2l6ZSA8PSAwID8gREVGQVVMVF9CVUZfU0laRSA6IHNpemUpKTtcbiAgICB0aGlzLiN3cml0ZXIgPSB3cml0ZXI7XG4gIH1cblxuICAvKiogRGlzY2FyZHMgYW55IHVuZmx1c2hlZCBidWZmZXJlZCBkYXRhLCBjbGVhcnMgYW55IGVycm9yLCBhbmRcbiAgICogcmVzZXRzIGJ1ZmZlciB0byB3cml0ZSBpdHMgb3V0cHV0IHRvIHcuXG4gICAqL1xuICByZXNldCh3OiBXcml0ZXIpIHtcbiAgICB0aGlzLmVyciA9IG51bGw7XG4gICAgdGhpcy51c2VkQnVmZmVyQnl0ZXMgPSAwO1xuICAgIHRoaXMuI3dyaXRlciA9IHc7XG4gIH1cblxuICAvKiogRmx1c2ggd3JpdGVzIGFueSBidWZmZXJlZCBkYXRhIHRvIHRoZSB1bmRlcmx5aW5nIGlvLldyaXRlci4gKi9cbiAgYXN5bmMgZmx1c2goKSB7XG4gICAgaWYgKHRoaXMuZXJyICE9PSBudWxsKSB0aHJvdyB0aGlzLmVycjtcbiAgICBpZiAodGhpcy51c2VkQnVmZmVyQnl0ZXMgPT09IDApIHJldHVybjtcblxuICAgIHRyeSB7XG4gICAgICBjb25zdCBwID0gdGhpcy5idWYuc3ViYXJyYXkoMCwgdGhpcy51c2VkQnVmZmVyQnl0ZXMpO1xuICAgICAgbGV0IG53cml0dGVuID0gMDtcbiAgICAgIHdoaWxlIChud3JpdHRlbiA8IHAubGVuZ3RoKSB7XG4gICAgICAgIG53cml0dGVuICs9IGF3YWl0IHRoaXMuI3dyaXRlci53cml0ZShwLnN1YmFycmF5KG53cml0dGVuKSk7XG4gICAgICB9XG4gICAgfSBjYXRjaCAoZSkge1xuICAgICAgaWYgKGUgaW5zdGFuY2VvZiBFcnJvcikge1xuICAgICAgICB0aGlzLmVyciA9IGU7XG4gICAgICB9XG4gICAgICB0aHJvdyBlO1xuICAgIH1cblxuICAgIHRoaXMuYnVmID0gbmV3IFVpbnQ4QXJyYXkodGhpcy5idWYubGVuZ3RoKTtcbiAgICB0aGlzLnVzZWRCdWZmZXJCeXRlcyA9IDA7XG4gIH1cblxuICAvKiogV3JpdGVzIHRoZSBjb250ZW50cyBvZiBgZGF0YWAgaW50byB0aGUgYnVmZmVyLiAgSWYgdGhlIGNvbnRlbnRzIHdvbid0IGZ1bGx5XG4gICAqIGZpdCBpbnRvIHRoZSBidWZmZXIsIHRob3NlIGJ5dGVzIHRoYXQgY2FuIGFyZSBjb3BpZWQgaW50byB0aGUgYnVmZmVyLCB0aGVcbiAgICogYnVmZmVyIGlzIHRoZSBmbHVzaGVkIHRvIHRoZSB3cml0ZXIgYW5kIHRoZSByZW1haW5pbmcgYnl0ZXMgYXJlIGNvcGllZCBpbnRvXG4gICAqIHRoZSBub3cgZW1wdHkgYnVmZmVyLlxuICAgKlxuICAgKiBAcmV0dXJuIHRoZSBudW1iZXIgb2YgYnl0ZXMgd3JpdHRlbiB0byB0aGUgYnVmZmVyLlxuICAgKi9cbiAgYXN5bmMgd3JpdGUoZGF0YTogVWludDhBcnJheSk6IFByb21pc2U8bnVtYmVyPiB7XG4gICAgaWYgKHRoaXMuZXJyICE9PSBudWxsKSB0aHJvdyB0aGlzLmVycjtcbiAgICBpZiAoZGF0YS5sZW5ndGggPT09IDApIHJldHVybiAwO1xuXG4gICAgbGV0IHRvdGFsQnl0ZXNXcml0dGVuID0gMDtcbiAgICBsZXQgbnVtQnl0ZXNXcml0dGVuID0gMDtcbiAgICB3aGlsZSAoZGF0YS5ieXRlTGVuZ3RoID4gdGhpcy5hdmFpbGFibGUoKSkge1xuICAgICAgaWYgKHRoaXMuYnVmZmVyZWQoKSA9PT0gMCkge1xuICAgICAgICAvLyBMYXJnZSB3cml0ZSwgZW1wdHkgYnVmZmVyLlxuICAgICAgICAvLyBXcml0ZSBkaXJlY3RseSBmcm9tIGRhdGEgdG8gYXZvaWQgY29weS5cbiAgICAgICAgdHJ5IHtcbiAgICAgICAgICBudW1CeXRlc1dyaXR0ZW4gPSBhd2FpdCB0aGlzLiN3cml0ZXIud3JpdGUoZGF0YSk7XG4gICAgICAgIH0gY2F0Y2ggKGUpIHtcbiAgICAgICAgICBpZiAoZSBpbnN0YW5jZW9mIEVycm9yKSB7XG4gICAgICAgICAgICB0aGlzLmVyciA9IGU7XG4gICAgICAgICAgfVxuICAgICAgICAgIHRocm93IGU7XG4gICAgICAgIH1cbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIG51bUJ5dGVzV3JpdHRlbiA9IGNvcHkoZGF0YSwgdGhpcy5idWYsIHRoaXMudXNlZEJ1ZmZlckJ5dGVzKTtcbiAgICAgICAgdGhpcy51c2VkQnVmZmVyQnl0ZXMgKz0gbnVtQnl0ZXNXcml0dGVuO1xuICAgICAgICBhd2FpdCB0aGlzLmZsdXNoKCk7XG4gICAgICB9XG4gICAgICB0b3RhbEJ5dGVzV3JpdHRlbiArPSBudW1CeXRlc1dyaXR0ZW47XG4gICAgICBkYXRhID0gZGF0YS5zdWJhcnJheShudW1CeXRlc1dyaXR0ZW4pO1xuICAgIH1cblxuICAgIG51bUJ5dGVzV3JpdHRlbiA9IGNvcHkoZGF0YSwgdGhpcy5idWYsIHRoaXMudXNlZEJ1ZmZlckJ5dGVzKTtcbiAgICB0aGlzLnVzZWRCdWZmZXJCeXRlcyArPSBudW1CeXRlc1dyaXR0ZW47XG4gICAgdG90YWxCeXRlc1dyaXR0ZW4gKz0gbnVtQnl0ZXNXcml0dGVuO1xuICAgIHJldHVybiB0b3RhbEJ5dGVzV3JpdHRlbjtcbiAgfVxufVxuXG4vKiogQnVmV3JpdGVyU3luYyBpbXBsZW1lbnRzIGJ1ZmZlcmluZyBmb3IgYSBkZW5vLldyaXRlclN5bmMgb2JqZWN0LlxuICogSWYgYW4gZXJyb3Igb2NjdXJzIHdyaXRpbmcgdG8gYSBXcml0ZXJTeW5jLCBubyBtb3JlIGRhdGEgd2lsbCBiZVxuICogYWNjZXB0ZWQgYW5kIGFsbCBzdWJzZXF1ZW50IHdyaXRlcywgYW5kIGZsdXNoKCksIHdpbGwgcmV0dXJuIHRoZSBlcnJvci5cbiAqIEFmdGVyIGFsbCBkYXRhIGhhcyBiZWVuIHdyaXR0ZW4sIHRoZSBjbGllbnQgc2hvdWxkIGNhbGwgdGhlXG4gKiBmbHVzaCgpIG1ldGhvZCB0byBndWFyYW50ZWUgYWxsIGRhdGEgaGFzIGJlZW4gZm9yd2FyZGVkIHRvXG4gKiB0aGUgdW5kZXJseWluZyBkZW5vLldyaXRlclN5bmMuXG4gKi9cbmV4cG9ydCBjbGFzcyBCdWZXcml0ZXJTeW5jIGV4dGVuZHMgQWJzdHJhY3RCdWZCYXNlIGltcGxlbWVudHMgV3JpdGVyU3luYyB7XG4gICN3cml0ZXI6IFdyaXRlclN5bmM7XG5cbiAgLyoqIHJldHVybiBuZXcgQnVmV3JpdGVyU3luYyB1bmxlc3Mgd3JpdGVyIGlzIEJ1ZldyaXRlclN5bmMgKi9cbiAgc3RhdGljIGNyZWF0ZShcbiAgICB3cml0ZXI6IFdyaXRlclN5bmMsXG4gICAgc2l6ZTogbnVtYmVyID0gREVGQVVMVF9CVUZfU0laRSxcbiAgKTogQnVmV3JpdGVyU3luYyB7XG4gICAgcmV0dXJuIHdyaXRlciBpbnN0YW5jZW9mIEJ1ZldyaXRlclN5bmNcbiAgICAgID8gd3JpdGVyXG4gICAgICA6IG5ldyBCdWZXcml0ZXJTeW5jKHdyaXRlciwgc2l6ZSk7XG4gIH1cblxuICBjb25zdHJ1Y3Rvcih3cml0ZXI6IFdyaXRlclN5bmMsIHNpemU6IG51bWJlciA9IERFRkFVTFRfQlVGX1NJWkUpIHtcbiAgICBzdXBlcihuZXcgVWludDhBcnJheShzaXplIDw9IDAgPyBERUZBVUxUX0JVRl9TSVpFIDogc2l6ZSkpO1xuICAgIHRoaXMuI3dyaXRlciA9IHdyaXRlcjtcbiAgfVxuXG4gIC8qKiBEaXNjYXJkcyBhbnkgdW5mbHVzaGVkIGJ1ZmZlcmVkIGRhdGEsIGNsZWFycyBhbnkgZXJyb3IsIGFuZFxuICAgKiByZXNldHMgYnVmZmVyIHRvIHdyaXRlIGl0cyBvdXRwdXQgdG8gdy5cbiAgICovXG4gIHJlc2V0KHc6IFdyaXRlclN5bmMpIHtcbiAgICB0aGlzLmVyciA9IG51bGw7XG4gICAgdGhpcy51c2VkQnVmZmVyQnl0ZXMgPSAwO1xuICAgIHRoaXMuI3dyaXRlciA9IHc7XG4gIH1cblxuICAvKiogRmx1c2ggd3JpdGVzIGFueSBidWZmZXJlZCBkYXRhIHRvIHRoZSB1bmRlcmx5aW5nIGlvLldyaXRlclN5bmMuICovXG4gIGZsdXNoKCkge1xuICAgIGlmICh0aGlzLmVyciAhPT0gbnVsbCkgdGhyb3cgdGhpcy5lcnI7XG4gICAgaWYgKHRoaXMudXNlZEJ1ZmZlckJ5dGVzID09PSAwKSByZXR1cm47XG5cbiAgICB0cnkge1xuICAgICAgY29uc3QgcCA9IHRoaXMuYnVmLnN1YmFycmF5KDAsIHRoaXMudXNlZEJ1ZmZlckJ5dGVzKTtcbiAgICAgIGxldCBud3JpdHRlbiA9IDA7XG4gICAgICB3aGlsZSAobndyaXR0ZW4gPCBwLmxlbmd0aCkge1xuICAgICAgICBud3JpdHRlbiArPSB0aGlzLiN3cml0ZXIud3JpdGVTeW5jKHAuc3ViYXJyYXkobndyaXR0ZW4pKTtcbiAgICAgIH1cbiAgICB9IGNhdGNoIChlKSB7XG4gICAgICBpZiAoZSBpbnN0YW5jZW9mIEVycm9yKSB7XG4gICAgICAgIHRoaXMuZXJyID0gZTtcbiAgICAgIH1cbiAgICAgIHRocm93IGU7XG4gICAgfVxuXG4gICAgdGhpcy5idWYgPSBuZXcgVWludDhBcnJheSh0aGlzLmJ1Zi5sZW5ndGgpO1xuICAgIHRoaXMudXNlZEJ1ZmZlckJ5dGVzID0gMDtcbiAgfVxuXG4gIC8qKiBXcml0ZXMgdGhlIGNvbnRlbnRzIG9mIGBkYXRhYCBpbnRvIHRoZSBidWZmZXIuICBJZiB0aGUgY29udGVudHMgd29uJ3QgZnVsbHlcbiAgICogZml0IGludG8gdGhlIGJ1ZmZlciwgdGhvc2UgYnl0ZXMgdGhhdCBjYW4gYXJlIGNvcGllZCBpbnRvIHRoZSBidWZmZXIsIHRoZVxuICAgKiBidWZmZXIgaXMgdGhlIGZsdXNoZWQgdG8gdGhlIHdyaXRlciBhbmQgdGhlIHJlbWFpbmluZyBieXRlcyBhcmUgY29waWVkIGludG9cbiAgICogdGhlIG5vdyBlbXB0eSBidWZmZXIuXG4gICAqXG4gICAqIEByZXR1cm4gdGhlIG51bWJlciBvZiBieXRlcyB3cml0dGVuIHRvIHRoZSBidWZmZXIuXG4gICAqL1xuICB3cml0ZVN5bmMoZGF0YTogVWludDhBcnJheSk6IG51bWJlciB7XG4gICAgaWYgKHRoaXMuZXJyICE9PSBudWxsKSB0aHJvdyB0aGlzLmVycjtcbiAgICBpZiAoZGF0YS5sZW5ndGggPT09IDApIHJldHVybiAwO1xuXG4gICAgbGV0IHRvdGFsQnl0ZXNXcml0dGVuID0gMDtcbiAgICBsZXQgbnVtQnl0ZXNXcml0dGVuID0gMDtcbiAgICB3aGlsZSAoZGF0YS5ieXRlTGVuZ3RoID4gdGhpcy5hdmFpbGFibGUoKSkge1xuICAgICAgaWYgKHRoaXMuYnVmZmVyZWQoKSA9PT0gMCkge1xuICAgICAgICAvLyBMYXJnZSB3cml0ZSwgZW1wdHkgYnVmZmVyLlxuICAgICAgICAvLyBXcml0ZSBkaXJlY3RseSBmcm9tIGRhdGEgdG8gYXZvaWQgY29weS5cbiAgICAgICAgdHJ5IHtcbiAgICAgICAgICBudW1CeXRlc1dyaXR0ZW4gPSB0aGlzLiN3cml0ZXIud3JpdGVTeW5jKGRhdGEpO1xuICAgICAgICB9IGNhdGNoIChlKSB7XG4gICAgICAgICAgaWYgKGUgaW5zdGFuY2VvZiBFcnJvcikge1xuICAgICAgICAgICAgdGhpcy5lcnIgPSBlO1xuICAgICAgICAgIH1cbiAgICAgICAgICB0aHJvdyBlO1xuICAgICAgICB9XG4gICAgICB9IGVsc2Uge1xuICAgICAgICBudW1CeXRlc1dyaXR0ZW4gPSBjb3B5KGRhdGEsIHRoaXMuYnVmLCB0aGlzLnVzZWRCdWZmZXJCeXRlcyk7XG4gICAgICAgIHRoaXMudXNlZEJ1ZmZlckJ5dGVzICs9IG51bUJ5dGVzV3JpdHRlbjtcbiAgICAgICAgdGhpcy5mbHVzaCgpO1xuICAgICAgfVxuICAgICAgdG90YWxCeXRlc1dyaXR0ZW4gKz0gbnVtQnl0ZXNXcml0dGVuO1xuICAgICAgZGF0YSA9IGRhdGEuc3ViYXJyYXkobnVtQnl0ZXNXcml0dGVuKTtcbiAgICB9XG5cbiAgICBudW1CeXRlc1dyaXR0ZW4gPSBjb3B5KGRhdGEsIHRoaXMuYnVmLCB0aGlzLnVzZWRCdWZmZXJCeXRlcyk7XG4gICAgdGhpcy51c2VkQnVmZmVyQnl0ZXMgKz0gbnVtQnl0ZXNXcml0dGVuO1xuICAgIHRvdGFsQnl0ZXNXcml0dGVuICs9IG51bUJ5dGVzV3JpdHRlbjtcbiAgICByZXR1cm4gdG90YWxCeXRlc1dyaXR0ZW47XG4gIH1cbn1cbiJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQSwwRUFBMEU7QUFFMUUsU0FBUyxJQUFJLFFBQVEsbUJBQW1CO0FBR3hDLE1BQU0sbUJBQW1CO0FBRXpCLE1B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
// denoCacheMetadata=11139326649385159917,10566233025547351660