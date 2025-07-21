// Copyright 2018-2023 the Deno authors. All rights reserved. MIT license.
// This module is browser compatible.
import { assert } from "../_util/asserts.ts";
import { copy } from "../bytes/copy.ts";
const DEFAULT_BUF_SIZE = 4096;
const MIN_BUF_SIZE = 16;
const MAX_CONSECUTIVE_EMPTY_READS = 100;
const CR = "\r".charCodeAt(0);
const LF = "\n".charCodeAt(0);
export class BufferFullError extends Error {
  partial;
  name;
  constructor(partial){
    super("Buffer full"), this.partial = partial, this.name = "BufferFullError";
  }
}
export class PartialReadError extends Error {
  name = "PartialReadError";
  partial;
  constructor(){
    super("Encountered UnexpectedEof, data only partially read");
  }
}
export class BufReader {
  #buf;
  #rd;
  #r = 0;
  #w = 0;
  #eof = false;
  // private lastByte: number;
  // private lastCharSize: number;
  /** return new BufReader unless r is BufReader */ static create(r, size = DEFAULT_BUF_SIZE) {
    return r instanceof BufReader ? r : new BufReader(r, size);
  }
  constructor(rd, size = DEFAULT_BUF_SIZE){
    if (size < MIN_BUF_SIZE) {
      size = MIN_BUF_SIZE;
    }
    this.#reset(new Uint8Array(size), rd);
  }
  /** Returns the size of the underlying buffer in bytes. */ size() {
    return this.#buf.byteLength;
  }
  buffered() {
    return this.#w - this.#r;
  }
  // Reads a new chunk into the buffer.
  #fill = async ()=>{
    // Slide existing data to beginning.
    if (this.#r > 0) {
      this.#buf.copyWithin(0, this.#r, this.#w);
      this.#w -= this.#r;
      this.#r = 0;
    }
    if (this.#w >= this.#buf.byteLength) {
      throw Error("bufio: tried to fill full buffer");
    }
    // Read new data: try a limited number of times.
    for(let i = MAX_CONSECUTIVE_EMPTY_READS; i > 0; i--){
      const rr = await this.#rd.read(this.#buf.subarray(this.#w));
      if (rr === null) {
        this.#eof = true;
        return;
      }
      assert(rr >= 0, "negative read");
      this.#w += rr;
      if (rr > 0) {
        return;
      }
    }
    throw new Error(`No progress after ${MAX_CONSECUTIVE_EMPTY_READS} read() calls`);
  };
  /** Discards any buffered data, resets all state, and switches
   * the buffered reader to read from r.
   */ reset(r) {
    this.#reset(this.#buf, r);
  }
  #reset = (buf, rd)=>{
    this.#buf = buf;
    this.#rd = rd;
    this.#eof = false;
  // this.lastByte = -1;
  // this.lastCharSize = -1;
  };
  /** reads data into p.
   * It returns the number of bytes read into p.
   * The bytes are taken from at most one Read on the underlying Reader,
   * hence n may be less than len(p).
   * To read exactly len(p) bytes, use io.ReadFull(b, p).
   */ async read(p) {
    let rr = p.byteLength;
    if (p.byteLength === 0) return rr;
    if (this.#r === this.#w) {
      if (p.byteLength >= this.#buf.byteLength) {
        // Large read, empty buffer.
        // Read directly into p to avoid copy.
        const rr = await this.#rd.read(p);
        const nread = rr ?? 0;
        assert(nread >= 0, "negative read");
        // if (rr.nread > 0) {
        //   this.lastByte = p[rr.nread - 1];
        //   this.lastCharSize = -1;
        // }
        return rr;
      }
      // One read.
      // Do not use this.fill, which will loop.
      this.#r = 0;
      this.#w = 0;
      rr = await this.#rd.read(this.#buf);
      if (rr === 0 || rr === null) return rr;
      assert(rr >= 0, "negative read");
      this.#w += rr;
    }
    // copy as much as we can
    const copied = copy(this.#buf.subarray(this.#r, this.#w), p, 0);
    this.#r += copied;
    // this.lastByte = this.buf[this.r - 1];
    // this.lastCharSize = -1;
    return copied;
  }
  /** reads exactly `p.length` bytes into `p`.
   *
   * If successful, `p` is returned.
   *
   * If the end of the underlying stream has been reached, and there are no more
   * bytes available in the buffer, `readFull()` returns `null` instead.
   *
   * An error is thrown if some bytes could be read, but not enough to fill `p`
   * entirely before the underlying stream reported an error or EOF. Any error
   * thrown will have a `partial` property that indicates the slice of the
   * buffer that has been successfully filled with data.
   *
   * Ported from https://golang.org/pkg/io/#ReadFull
   */ async readFull(p) {
    let bytesRead = 0;
    while(bytesRead < p.length){
      try {
        const rr = await this.read(p.subarray(bytesRead));
        if (rr === null) {
          if (bytesRead === 0) {
            return null;
          } else {
            throw new PartialReadError();
          }
        }
        bytesRead += rr;
      } catch (err) {
        if (err instanceof PartialReadError) {
          err.partial = p.subarray(0, bytesRead);
        }
        throw err;
      }
    }
    return p;
  }
  /** Returns the next byte [0, 255] or `null`. */ async readByte() {
    while(this.#r === this.#w){
      if (this.#eof) return null;
      await this.#fill(); // buffer is empty.
    }
    const c = this.#buf[this.#r];
    this.#r++;
    // this.lastByte = c;
    return c;
  }
  /** readString() reads until the first occurrence of delim in the input,
   * returning a string containing the data up to and including the delimiter.
   * If ReadString encounters an error before finding a delimiter,
   * it returns the data read before the error and the error itself
   * (often `null`).
   * ReadString returns err != nil if and only if the returned data does not end
   * in delim.
   * For simple uses, a Scanner may be more convenient.
   */ async readString(delim) {
    if (delim.length !== 1) {
      throw new Error("Delimiter should be a single character");
    }
    const buffer = await this.readSlice(delim.charCodeAt(0));
    if (buffer === null) return null;
    return new TextDecoder().decode(buffer);
  }
  /** `readLine()` is a low-level line-reading primitive. Most callers should
   * use `readString('\n')` instead or use a Scanner.
   *
   * `readLine()` tries to return a single line, not including the end-of-line
   * bytes. If the line was too long for the buffer then `more` is set and the
   * beginning of the line is returned. The rest of the line will be returned
   * from future calls. `more` will be false when returning the last fragment
   * of the line. The returned buffer is only valid until the next call to
   * `readLine()`.
   *
   * The text returned from ReadLine does not include the line end ("\r\n" or
   * "\n").
   *
   * When the end of the underlying stream is reached, the final bytes in the
   * stream are returned. No indication or error is given if the input ends
   * without a final line end. When there are no more trailing bytes to read,
   * `readLine()` returns `null`.
   *
   * Calling `unreadByte()` after `readLine()` will always unread the last byte
   * read (possibly a character belonging to the line end) even if that byte is
   * not part of the line returned by `readLine()`.
   */ async readLine() {
    let line = null;
    try {
      line = await this.readSlice(LF);
    } catch (err) {
      let partial;
      if (err instanceof PartialReadError) {
        partial = err.partial;
        assert(partial instanceof Uint8Array, "bufio: caught error from `readSlice()` without `partial` property");
      }
      // Don't throw if `readSlice()` failed with `BufferFullError`, instead we
      // just return whatever is available and set the `more` flag.
      if (!(err instanceof BufferFullError)) {
        throw err;
      }
      partial = err.partial;
      // Handle the case where "\r\n" straddles the buffer.
      if (!this.#eof && partial && partial.byteLength > 0 && partial[partial.byteLength - 1] === CR) {
        // Put the '\r' back on buf and drop it from line.
        // Let the next call to ReadLine check for "\r\n".
        assert(this.#r > 0, "bufio: tried to rewind past start of buffer");
        this.#r--;
        partial = partial.subarray(0, partial.byteLength - 1);
      }
      if (partial) {
        return {
          line: partial,
          more: !this.#eof
        };
      }
    }
    if (line === null) {
      return null;
    }
    if (line.byteLength === 0) {
      return {
        line,
        more: false
      };
    }
    if (line[line.byteLength - 1] == LF) {
      let drop = 1;
      if (line.byteLength > 1 && line[line.byteLength - 2] === CR) {
        drop = 2;
      }
      line = line.subarray(0, line.byteLength - drop);
    }
    return {
      line,
      more: false
    };
  }
  /** `readSlice()` reads until the first occurrence of `delim` in the input,
   * returning a slice pointing at the bytes in the buffer. The bytes stop
   * being valid at the next read.
   *
   * If `readSlice()` encounters an error before finding a delimiter, or the
   * buffer fills without finding a delimiter, it throws an error with a
   * `partial` property that contains the entire buffer.
   *
   * If `readSlice()` encounters the end of the underlying stream and there are
   * any bytes left in the buffer, the rest of the buffer is returned. In other
   * words, EOF is always treated as a delimiter. Once the buffer is empty,
   * it returns `null`.
   *
   * Because the data returned from `readSlice()` will be overwritten by the
   * next I/O operation, most clients should use `readString()` instead.
   */ async readSlice(delim) {
    let s = 0; // search start index
    let slice;
    while(true){
      // Search buffer.
      let i = this.#buf.subarray(this.#r + s, this.#w).indexOf(delim);
      if (i >= 0) {
        i += s;
        slice = this.#buf.subarray(this.#r, this.#r + i + 1);
        this.#r += i + 1;
        break;
      }
      // EOF?
      if (this.#eof) {
        if (this.#r === this.#w) {
          return null;
        }
        slice = this.#buf.subarray(this.#r, this.#w);
        this.#r = this.#w;
        break;
      }
      // Buffer full?
      if (this.buffered() >= this.#buf.byteLength) {
        this.#r = this.#w;
        // #4521 The internal buffer should not be reused across reads because it causes corruption of data.
        const oldbuf = this.#buf;
        const newbuf = this.#buf.slice(0);
        this.#buf = newbuf;
        throw new BufferFullError(oldbuf);
      }
      s = this.#w - this.#r; // do not rescan area we scanned before
      // Buffer is not full.
      try {
        await this.#fill();
      } catch (err) {
        if (err instanceof PartialReadError) {
          err.partial = slice;
        }
        throw err;
      }
    }
    // Handle last byte, if any.
    // const i = slice.byteLength - 1;
    // if (i >= 0) {
    //   this.lastByte = slice[i];
    //   this.lastCharSize = -1
    // }
    return slice;
  }
  /** `peek()` returns the next `n` bytes without advancing the reader. The
   * bytes stop being valid at the next read call.
   *
   * When the end of the underlying stream is reached, but there are unread
   * bytes left in the buffer, those bytes are returned. If there are no bytes
   * left in the buffer, it returns `null`.
   *
   * If an error is encountered before `n` bytes are available, `peek()` throws
   * an error with the `partial` property set to a slice of the buffer that
   * contains the bytes that were available before the error occurred.
   */ async peek(n) {
    if (n < 0) {
      throw Error("negative count");
    }
    let avail = this.#w - this.#r;
    while(avail < n && avail < this.#buf.byteLength && !this.#eof){
      try {
        await this.#fill();
      } catch (err) {
        if (err instanceof PartialReadError) {
          err.partial = this.#buf.subarray(this.#r, this.#w);
        }
        throw err;
      }
      avail = this.#w - this.#r;
    }
    if (avail === 0 && this.#eof) {
      return null;
    } else if (avail < n && this.#eof) {
      return this.#buf.subarray(this.#r, this.#r + avail);
    } else if (avail < n) {
      throw new BufferFullError(this.#buf.subarray(this.#r, this.#w));
    }
    return this.#buf.subarray(this.#r, this.#r + n);
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjE5MS4wL2lvL2J1Zl9yZWFkZXIudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyMyB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIFRoaXMgbW9kdWxlIGlzIGJyb3dzZXIgY29tcGF0aWJsZS5cblxuaW1wb3J0IHsgYXNzZXJ0IH0gZnJvbSBcIi4uL191dGlsL2Fzc2VydHMudHNcIjtcbmltcG9ydCB7IGNvcHkgfSBmcm9tIFwiLi4vYnl0ZXMvY29weS50c1wiO1xuaW1wb3J0IHR5cGUgeyBSZWFkZXIgfSBmcm9tIFwiLi4vdHlwZXMuZC50c1wiO1xuXG5jb25zdCBERUZBVUxUX0JVRl9TSVpFID0gNDA5NjtcbmNvbnN0IE1JTl9CVUZfU0laRSA9IDE2O1xuY29uc3QgTUFYX0NPTlNFQ1VUSVZFX0VNUFRZX1JFQURTID0gMTAwO1xuY29uc3QgQ1IgPSBcIlxcclwiLmNoYXJDb2RlQXQoMCk7XG5jb25zdCBMRiA9IFwiXFxuXCIuY2hhckNvZGVBdCgwKTtcblxuZXhwb3J0IGNsYXNzIEJ1ZmZlckZ1bGxFcnJvciBleHRlbmRzIEVycm9yIHtcbiAgb3ZlcnJpZGUgbmFtZSA9IFwiQnVmZmVyRnVsbEVycm9yXCI7XG4gIGNvbnN0cnVjdG9yKHB1YmxpYyBwYXJ0aWFsOiBVaW50OEFycmF5KSB7XG4gICAgc3VwZXIoXCJCdWZmZXIgZnVsbFwiKTtcbiAgfVxufVxuXG5leHBvcnQgY2xhc3MgUGFydGlhbFJlYWRFcnJvciBleHRlbmRzIEVycm9yIHtcbiAgb3ZlcnJpZGUgbmFtZSA9IFwiUGFydGlhbFJlYWRFcnJvclwiO1xuICBwYXJ0aWFsPzogVWludDhBcnJheTtcbiAgY29uc3RydWN0b3IoKSB7XG4gICAgc3VwZXIoXCJFbmNvdW50ZXJlZCBVbmV4cGVjdGVkRW9mLCBkYXRhIG9ubHkgcGFydGlhbGx5IHJlYWRcIik7XG4gIH1cbn1cblxuLyoqIFJlc3VsdCB0eXBlIHJldHVybmVkIGJ5IG9mIEJ1ZlJlYWRlci5yZWFkTGluZSgpLiAqL1xuZXhwb3J0IGludGVyZmFjZSBSZWFkTGluZVJlc3VsdCB7XG4gIGxpbmU6IFVpbnQ4QXJyYXk7XG4gIG1vcmU6IGJvb2xlYW47XG59XG5cbmV4cG9ydCBjbGFzcyBCdWZSZWFkZXIgaW1wbGVtZW50cyBSZWFkZXIge1xuICAjYnVmITogVWludDhBcnJheTtcbiAgI3JkITogUmVhZGVyOyAvLyBSZWFkZXIgcHJvdmlkZWQgYnkgY2FsbGVyLlxuICAjciA9IDA7IC8vIGJ1ZiByZWFkIHBvc2l0aW9uLlxuICAjdyA9IDA7IC8vIGJ1ZiB3cml0ZSBwb3NpdGlvbi5cbiAgI2VvZiA9IGZhbHNlO1xuICAvLyBwcml2YXRlIGxhc3RCeXRlOiBudW1iZXI7XG4gIC8vIHByaXZhdGUgbGFzdENoYXJTaXplOiBudW1iZXI7XG5cbiAgLyoqIHJldHVybiBuZXcgQnVmUmVhZGVyIHVubGVzcyByIGlzIEJ1ZlJlYWRlciAqL1xuICBzdGF0aWMgY3JlYXRlKHI6IFJlYWRlciwgc2l6ZTogbnVtYmVyID0gREVGQVVMVF9CVUZfU0laRSk6IEJ1ZlJlYWRlciB7XG4gICAgcmV0dXJuIHIgaW5zdGFuY2VvZiBCdWZSZWFkZXIgPyByIDogbmV3IEJ1ZlJlYWRlcihyLCBzaXplKTtcbiAgfVxuXG4gIGNvbnN0cnVjdG9yKHJkOiBSZWFkZXIsIHNpemU6IG51bWJlciA9IERFRkFVTFRfQlVGX1NJWkUpIHtcbiAgICBpZiAoc2l6ZSA8IE1JTl9CVUZfU0laRSkge1xuICAgICAgc2l6ZSA9IE1JTl9CVUZfU0laRTtcbiAgICB9XG4gICAgdGhpcy4jcmVzZXQobmV3IFVpbnQ4QXJyYXkoc2l6ZSksIHJkKTtcbiAgfVxuXG4gIC8qKiBSZXR1cm5zIHRoZSBzaXplIG9mIHRoZSB1bmRlcmx5aW5nIGJ1ZmZlciBpbiBieXRlcy4gKi9cbiAgc2l6ZSgpOiBudW1iZXIge1xuICAgIHJldHVybiB0aGlzLiNidWYuYnl0ZUxlbmd0aDtcbiAgfVxuXG4gIGJ1ZmZlcmVkKCk6IG51bWJlciB7XG4gICAgcmV0dXJuIHRoaXMuI3cgLSB0aGlzLiNyO1xuICB9XG5cbiAgLy8gUmVhZHMgYSBuZXcgY2h1bmsgaW50byB0aGUgYnVmZmVyLlxuICAjZmlsbCA9IGFzeW5jICgpID0+IHtcbiAgICAvLyBTbGlkZSBleGlzdGluZyBkYXRhIHRvIGJlZ2lubmluZy5cbiAgICBpZiAodGhpcy4jciA+IDApIHtcbiAgICAgIHRoaXMuI2J1Zi5jb3B5V2l0aGluKDAsIHRoaXMuI3IsIHRoaXMuI3cpO1xuICAgICAgdGhpcy4jdyAtPSB0aGlzLiNyO1xuICAgICAgdGhpcy4jciA9IDA7XG4gICAgfVxuXG4gICAgaWYgKHRoaXMuI3cgPj0gdGhpcy4jYnVmLmJ5dGVMZW5ndGgpIHtcbiAgICAgIHRocm93IEVycm9yKFwiYnVmaW86IHRyaWVkIHRvIGZpbGwgZnVsbCBidWZmZXJcIik7XG4gICAgfVxuXG4gICAgLy8gUmVhZCBuZXcgZGF0YTogdHJ5IGEgbGltaXRlZCBudW1iZXIgb2YgdGltZXMuXG4gICAgZm9yIChsZXQgaSA9IE1BWF9DT05TRUNVVElWRV9FTVBUWV9SRUFEUzsgaSA+IDA7IGktLSkge1xuICAgICAgY29uc3QgcnIgPSBhd2FpdCB0aGlzLiNyZC5yZWFkKHRoaXMuI2J1Zi5zdWJhcnJheSh0aGlzLiN3KSk7XG4gICAgICBpZiAocnIgPT09IG51bGwpIHtcbiAgICAgICAgdGhpcy4jZW9mID0gdHJ1ZTtcbiAgICAgICAgcmV0dXJuO1xuICAgICAgfVxuICAgICAgYXNzZXJ0KHJyID49IDAsIFwibmVnYXRpdmUgcmVhZFwiKTtcbiAgICAgIHRoaXMuI3cgKz0gcnI7XG4gICAgICBpZiAocnIgPiAwKSB7XG4gICAgICAgIHJldHVybjtcbiAgICAgIH1cbiAgICB9XG5cbiAgICB0aHJvdyBuZXcgRXJyb3IoXG4gICAgICBgTm8gcHJvZ3Jlc3MgYWZ0ZXIgJHtNQVhfQ09OU0VDVVRJVkVfRU1QVFlfUkVBRFN9IHJlYWQoKSBjYWxsc2AsXG4gICAgKTtcbiAgfTtcblxuICAvKiogRGlzY2FyZHMgYW55IGJ1ZmZlcmVkIGRhdGEsIHJlc2V0cyBhbGwgc3RhdGUsIGFuZCBzd2l0Y2hlc1xuICAgKiB0aGUgYnVmZmVyZWQgcmVhZGVyIHRvIHJlYWQgZnJvbSByLlxuICAgKi9cbiAgcmVzZXQocjogUmVhZGVyKSB7XG4gICAgdGhpcy4jcmVzZXQodGhpcy4jYnVmLCByKTtcbiAgfVxuXG4gICNyZXNldCA9IChidWY6IFVpbnQ4QXJyYXksIHJkOiBSZWFkZXIpID0+IHtcbiAgICB0aGlzLiNidWYgPSBidWY7XG4gICAgdGhpcy4jcmQgPSByZDtcbiAgICB0aGlzLiNlb2YgPSBmYWxzZTtcbiAgICAvLyB0aGlzLmxhc3RCeXRlID0gLTE7XG4gICAgLy8gdGhpcy5sYXN0Q2hhclNpemUgPSAtMTtcbiAgfTtcblxuICAvKiogcmVhZHMgZGF0YSBpbnRvIHAuXG4gICAqIEl0IHJldHVybnMgdGhlIG51bWJlciBvZiBieXRlcyByZWFkIGludG8gcC5cbiAgICogVGhlIGJ5dGVzIGFyZSB0YWtlbiBmcm9tIGF0IG1vc3Qgb25lIFJlYWQgb24gdGhlIHVuZGVybHlpbmcgUmVhZGVyLFxuICAgKiBoZW5jZSBuIG1heSBiZSBsZXNzIHRoYW4gbGVuKHApLlxuICAgKiBUbyByZWFkIGV4YWN0bHkgbGVuKHApIGJ5dGVzLCB1c2UgaW8uUmVhZEZ1bGwoYiwgcCkuXG4gICAqL1xuICBhc3luYyByZWFkKHA6IFVpbnQ4QXJyYXkpOiBQcm9taXNlPG51bWJlciB8IG51bGw+IHtcbiAgICBsZXQgcnI6IG51bWJlciB8IG51bGwgPSBwLmJ5dGVMZW5ndGg7XG4gICAgaWYgKHAuYnl0ZUxlbmd0aCA9PT0gMCkgcmV0dXJuIHJyO1xuXG4gICAgaWYgKHRoaXMuI3IgPT09IHRoaXMuI3cpIHtcbiAgICAgIGlmIChwLmJ5dGVMZW5ndGggPj0gdGhpcy4jYnVmLmJ5dGVMZW5ndGgpIHtcbiAgICAgICAgLy8gTGFyZ2UgcmVhZCwgZW1wdHkgYnVmZmVyLlxuICAgICAgICAvLyBSZWFkIGRpcmVjdGx5IGludG8gcCB0byBhdm9pZCBjb3B5LlxuICAgICAgICBjb25zdCByciA9IGF3YWl0IHRoaXMuI3JkLnJlYWQocCk7XG4gICAgICAgIGNvbnN0IG5yZWFkID0gcnIgPz8gMDtcbiAgICAgICAgYXNzZXJ0KG5yZWFkID49IDAsIFwibmVnYXRpdmUgcmVhZFwiKTtcbiAgICAgICAgLy8gaWYgKHJyLm5yZWFkID4gMCkge1xuICAgICAgICAvLyAgIHRoaXMubGFzdEJ5dGUgPSBwW3JyLm5yZWFkIC0gMV07XG4gICAgICAgIC8vICAgdGhpcy5sYXN0Q2hhclNpemUgPSAtMTtcbiAgICAgICAgLy8gfVxuICAgICAgICByZXR1cm4gcnI7XG4gICAgICB9XG5cbiAgICAgIC8vIE9uZSByZWFkLlxuICAgICAgLy8gRG8gbm90IHVzZSB0aGlzLmZpbGwsIHdoaWNoIHdpbGwgbG9vcC5cbiAgICAgIHRoaXMuI3IgPSAwO1xuICAgICAgdGhpcy4jdyA9IDA7XG4gICAgICByciA9IGF3YWl0IHRoaXMuI3JkLnJlYWQodGhpcy4jYnVmKTtcbiAgICAgIGlmIChyciA9PT0gMCB8fCByciA9PT0gbnVsbCkgcmV0dXJuIHJyO1xuICAgICAgYXNzZXJ0KHJyID49IDAsIFwibmVnYXRpdmUgcmVhZFwiKTtcbiAgICAgIHRoaXMuI3cgKz0gcnI7XG4gICAgfVxuXG4gICAgLy8gY29weSBhcyBtdWNoIGFzIHdlIGNhblxuICAgIGNvbnN0IGNvcGllZCA9IGNvcHkodGhpcy4jYnVmLnN1YmFycmF5KHRoaXMuI3IsIHRoaXMuI3cpLCBwLCAwKTtcbiAgICB0aGlzLiNyICs9IGNvcGllZDtcbiAgICAvLyB0aGlzLmxhc3RCeXRlID0gdGhpcy5idWZbdGhpcy5yIC0gMV07XG4gICAgLy8gdGhpcy5sYXN0Q2hhclNpemUgPSAtMTtcbiAgICByZXR1cm4gY29waWVkO1xuICB9XG5cbiAgLyoqIHJlYWRzIGV4YWN0bHkgYHAubGVuZ3RoYCBieXRlcyBpbnRvIGBwYC5cbiAgICpcbiAgICogSWYgc3VjY2Vzc2Z1bCwgYHBgIGlzIHJldHVybmVkLlxuICAgKlxuICAgKiBJZiB0aGUgZW5kIG9mIHRoZSB1bmRlcmx5aW5nIHN0cmVhbSBoYXMgYmVlbiByZWFjaGVkLCBhbmQgdGhlcmUgYXJlIG5vIG1vcmVcbiAgICogYnl0ZXMgYXZhaWxhYmxlIGluIHRoZSBidWZmZXIsIGByZWFkRnVsbCgpYCByZXR1cm5zIGBudWxsYCBpbnN0ZWFkLlxuICAgKlxuICAgKiBBbiBlcnJvciBpcyB0aHJvd24gaWYgc29tZSBieXRlcyBjb3VsZCBiZSByZWFkLCBidXQgbm90IGVub3VnaCB0byBmaWxsIGBwYFxuICAgKiBlbnRpcmVseSBiZWZvcmUgdGhlIHVuZGVybHlpbmcgc3RyZWFtIHJlcG9ydGVkIGFuIGVycm9yIG9yIEVPRi4gQW55IGVycm9yXG4gICAqIHRocm93biB3aWxsIGhhdmUgYSBgcGFydGlhbGAgcHJvcGVydHkgdGhhdCBpbmRpY2F0ZXMgdGhlIHNsaWNlIG9mIHRoZVxuICAgKiBidWZmZXIgdGhhdCBoYXMgYmVlbiBzdWNjZXNzZnVsbHkgZmlsbGVkIHdpdGggZGF0YS5cbiAgICpcbiAgICogUG9ydGVkIGZyb20gaHR0cHM6Ly9nb2xhbmcub3JnL3BrZy9pby8jUmVhZEZ1bGxcbiAgICovXG4gIGFzeW5jIHJlYWRGdWxsKHA6IFVpbnQ4QXJyYXkpOiBQcm9taXNlPFVpbnQ4QXJyYXkgfCBudWxsPiB7XG4gICAgbGV0IGJ5dGVzUmVhZCA9IDA7XG4gICAgd2hpbGUgKGJ5dGVzUmVhZCA8IHAubGVuZ3RoKSB7XG4gICAgICB0cnkge1xuICAgICAgICBjb25zdCByciA9IGF3YWl0IHRoaXMucmVhZChwLnN1YmFycmF5KGJ5dGVzUmVhZCkpO1xuICAgICAgICBpZiAocnIgPT09IG51bGwpIHtcbiAgICAgICAgICBpZiAoYnl0ZXNSZWFkID09PSAwKSB7XG4gICAgICAgICAgICByZXR1cm4gbnVsbDtcbiAgICAgICAgICB9IGVsc2Uge1xuICAgICAgICAgICAgdGhyb3cgbmV3IFBhcnRpYWxSZWFkRXJyb3IoKTtcbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgICAgYnl0ZXNSZWFkICs9IHJyO1xuICAgICAgfSBjYXRjaCAoZXJyKSB7XG4gICAgICAgIGlmIChlcnIgaW5zdGFuY2VvZiBQYXJ0aWFsUmVhZEVycm9yKSB7XG4gICAgICAgICAgZXJyLnBhcnRpYWwgPSBwLnN1YmFycmF5KDAsIGJ5dGVzUmVhZCk7XG4gICAgICAgIH1cbiAgICAgICAgdGhyb3cgZXJyO1xuICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gcDtcbiAgfVxuXG4gIC8qKiBSZXR1cm5zIHRoZSBuZXh0IGJ5dGUgWzAsIDI1NV0gb3IgYG51bGxgLiAqL1xuICBhc3luYyByZWFkQnl0ZSgpOiBQcm9taXNlPG51bWJlciB8IG51bGw+IHtcbiAgICB3aGlsZSAodGhpcy4jciA9PT0gdGhpcy4jdykge1xuICAgICAgaWYgKHRoaXMuI2VvZikgcmV0dXJuIG51bGw7XG4gICAgICBhd2FpdCB0aGlzLiNmaWxsKCk7IC8vIGJ1ZmZlciBpcyBlbXB0eS5cbiAgICB9XG4gICAgY29uc3QgYyA9IHRoaXMuI2J1Zlt0aGlzLiNyXTtcbiAgICB0aGlzLiNyKys7XG4gICAgLy8gdGhpcy5sYXN0Qnl0ZSA9IGM7XG4gICAgcmV0dXJuIGM7XG4gIH1cblxuICAvKiogcmVhZFN0cmluZygpIHJlYWRzIHVudGlsIHRoZSBmaXJzdCBvY2N1cnJlbmNlIG9mIGRlbGltIGluIHRoZSBpbnB1dCxcbiAgICogcmV0dXJuaW5nIGEgc3RyaW5nIGNvbnRhaW5pbmcgdGhlIGRhdGEgdXAgdG8gYW5kIGluY2x1ZGluZyB0aGUgZGVsaW1pdGVyLlxuICAgKiBJZiBSZWFkU3RyaW5nIGVuY291bnRlcnMgYW4gZXJyb3IgYmVmb3JlIGZpbmRpbmcgYSBkZWxpbWl0ZXIsXG4gICAqIGl0IHJldHVybnMgdGhlIGRhdGEgcmVhZCBiZWZvcmUgdGhlIGVycm9yIGFuZCB0aGUgZXJyb3IgaXRzZWxmXG4gICAqIChvZnRlbiBgbnVsbGApLlxuICAgKiBSZWFkU3RyaW5nIHJldHVybnMgZXJyICE9IG5pbCBpZiBhbmQgb25seSBpZiB0aGUgcmV0dXJuZWQgZGF0YSBkb2VzIG5vdCBlbmRcbiAgICogaW4gZGVsaW0uXG4gICAqIEZvciBzaW1wbGUgdXNlcywgYSBTY2FubmVyIG1heSBiZSBtb3JlIGNvbnZlbmllbnQuXG4gICAqL1xuICBhc3luYyByZWFkU3RyaW5nKGRlbGltOiBzdHJpbmcpOiBQcm9taXNlPHN0cmluZyB8IG51bGw+IHtcbiAgICBpZiAoZGVsaW0ubGVuZ3RoICE9PSAxKSB7XG4gICAgICB0aHJvdyBuZXcgRXJyb3IoXCJEZWxpbWl0ZXIgc2hvdWxkIGJlIGEgc2luZ2xlIGNoYXJhY3RlclwiKTtcbiAgICB9XG4gICAgY29uc3QgYnVmZmVyID0gYXdhaXQgdGhpcy5yZWFkU2xpY2UoZGVsaW0uY2hhckNvZGVBdCgwKSk7XG4gICAgaWYgKGJ1ZmZlciA9PT0gbnVsbCkgcmV0dXJuIG51bGw7XG4gICAgcmV0dXJuIG5ldyBUZXh0RGVjb2RlcigpLmRlY29kZShidWZmZXIpO1xuICB9XG5cbiAgLyoqIGByZWFkTGluZSgpYCBpcyBhIGxvdy1sZXZlbCBsaW5lLXJlYWRpbmcgcHJpbWl0aXZlLiBNb3N0IGNhbGxlcnMgc2hvdWxkXG4gICAqIHVzZSBgcmVhZFN0cmluZygnXFxuJylgIGluc3RlYWQgb3IgdXNlIGEgU2Nhbm5lci5cbiAgICpcbiAgICogYHJlYWRMaW5lKClgIHRyaWVzIHRvIHJldHVybiBhIHNpbmdsZSBsaW5lLCBub3QgaW5jbHVkaW5nIHRoZSBlbmQtb2YtbGluZVxuICAgKiBieXRlcy4gSWYgdGhlIGxpbmUgd2FzIHRvbyBsb25nIGZvciB0aGUgYnVmZmVyIHRoZW4gYG1vcmVgIGlzIHNldCBhbmQgdGhlXG4gICAqIGJlZ2lubmluZyBvZiB0aGUgbGluZSBpcyByZXR1cm5lZC4gVGhlIHJlc3Qgb2YgdGhlIGxpbmUgd2lsbCBiZSByZXR1cm5lZFxuICAgKiBmcm9tIGZ1dHVyZSBjYWxscy4gYG1vcmVgIHdpbGwgYmUgZmFsc2Ugd2hlbiByZXR1cm5pbmcgdGhlIGxhc3QgZnJhZ21lbnRcbiAgICogb2YgdGhlIGxpbmUuIFRoZSByZXR1cm5lZCBidWZmZXIgaXMgb25seSB2YWxpZCB1bnRpbCB0aGUgbmV4dCBjYWxsIHRvXG4gICAqIGByZWFkTGluZSgpYC5cbiAgICpcbiAgICogVGhlIHRleHQgcmV0dXJuZWQgZnJvbSBSZWFkTGluZSBkb2VzIG5vdCBpbmNsdWRlIHRoZSBsaW5lIGVuZCAoXCJcXHJcXG5cIiBvclxuICAgKiBcIlxcblwiKS5cbiAgICpcbiAgICogV2hlbiB0aGUgZW5kIG9mIHRoZSB1bmRlcmx5aW5nIHN0cmVhbSBpcyByZWFjaGVkLCB0aGUgZmluYWwgYnl0ZXMgaW4gdGhlXG4gICAqIHN0cmVhbSBhcmUgcmV0dXJuZWQuIE5vIGluZGljYXRpb24gb3IgZXJyb3IgaXMgZ2l2ZW4gaWYgdGhlIGlucHV0IGVuZHNcbiAgICogd2l0aG91dCBhIGZpbmFsIGxpbmUgZW5kLiBXaGVuIHRoZXJlIGFyZSBubyBtb3JlIHRyYWlsaW5nIGJ5dGVzIHRvIHJlYWQsXG4gICAqIGByZWFkTGluZSgpYCByZXR1cm5zIGBudWxsYC5cbiAgICpcbiAgICogQ2FsbGluZyBgdW5yZWFkQnl0ZSgpYCBhZnRlciBgcmVhZExpbmUoKWAgd2lsbCBhbHdheXMgdW5yZWFkIHRoZSBsYXN0IGJ5dGVcbiAgICogcmVhZCAocG9zc2libHkgYSBjaGFyYWN0ZXIgYmVsb25naW5nIHRvIHRoZSBsaW5lIGVuZCkgZXZlbiBpZiB0aGF0IGJ5dGUgaXNcbiAgICogbm90IHBhcnQgb2YgdGhlIGxpbmUgcmV0dXJuZWQgYnkgYHJlYWRMaW5lKClgLlxuICAgKi9cbiAgYXN5bmMgcmVhZExpbmUoKTogUHJvbWlzZTxSZWFkTGluZVJlc3VsdCB8IG51bGw+IHtcbiAgICBsZXQgbGluZTogVWludDhBcnJheSB8IG51bGwgPSBudWxsO1xuXG4gICAgdHJ5IHtcbiAgICAgIGxpbmUgPSBhd2FpdCB0aGlzLnJlYWRTbGljZShMRik7XG4gICAgfSBjYXRjaCAoZXJyKSB7XG4gICAgICBsZXQgcGFydGlhbDtcbiAgICAgIGlmIChlcnIgaW5zdGFuY2VvZiBQYXJ0aWFsUmVhZEVycm9yKSB7XG4gICAgICAgIHBhcnRpYWwgPSBlcnIucGFydGlhbDtcbiAgICAgICAgYXNzZXJ0KFxuICAgICAgICAgIHBhcnRpYWwgaW5zdGFuY2VvZiBVaW50OEFycmF5LFxuICAgICAgICAgIFwiYnVmaW86IGNhdWdodCBlcnJvciBmcm9tIGByZWFkU2xpY2UoKWAgd2l0aG91dCBgcGFydGlhbGAgcHJvcGVydHlcIixcbiAgICAgICAgKTtcbiAgICAgIH1cblxuICAgICAgLy8gRG9uJ3QgdGhyb3cgaWYgYHJlYWRTbGljZSgpYCBmYWlsZWQgd2l0aCBgQnVmZmVyRnVsbEVycm9yYCwgaW5zdGVhZCB3ZVxuICAgICAgLy8ganVzdCByZXR1cm4gd2hhdGV2ZXIgaXMgYXZhaWxhYmxlIGFuZCBzZXQgdGhlIGBtb3JlYCBmbGFnLlxuICAgICAgaWYgKCEoZXJyIGluc3RhbmNlb2YgQnVmZmVyRnVsbEVycm9yKSkge1xuICAgICAgICB0aHJvdyBlcnI7XG4gICAgICB9XG5cbiAgICAgIHBhcnRpYWwgPSBlcnIucGFydGlhbDtcblxuICAgICAgLy8gSGFuZGxlIHRoZSBjYXNlIHdoZXJlIFwiXFxyXFxuXCIgc3RyYWRkbGVzIHRoZSBidWZmZXIuXG4gICAgICBpZiAoXG4gICAgICAgICF0aGlzLiNlb2YgJiYgcGFydGlhbCAmJlxuICAgICAgICBwYXJ0aWFsLmJ5dGVMZW5ndGggPiAwICYmXG4gICAgICAgIHBhcnRpYWxbcGFydGlhbC5ieXRlTGVuZ3RoIC0gMV0gPT09IENSXG4gICAgICApIHtcbiAgICAgICAgLy8gUHV0IHRoZSAnXFxyJyBiYWNrIG9uIGJ1ZiBhbmQgZHJvcCBpdCBmcm9tIGxpbmUuXG4gICAgICAgIC8vIExldCB0aGUgbmV4dCBjYWxsIHRvIFJlYWRMaW5lIGNoZWNrIGZvciBcIlxcclxcblwiLlxuICAgICAgICBhc3NlcnQodGhpcy4jciA+IDAsIFwiYnVmaW86IHRyaWVkIHRvIHJld2luZCBwYXN0IHN0YXJ0IG9mIGJ1ZmZlclwiKTtcbiAgICAgICAgdGhpcy4jci0tO1xuICAgICAgICBwYXJ0aWFsID0gcGFydGlhbC5zdWJhcnJheSgwLCBwYXJ0aWFsLmJ5dGVMZW5ndGggLSAxKTtcbiAgICAgIH1cblxuICAgICAgaWYgKHBhcnRpYWwpIHtcbiAgICAgICAgcmV0dXJuIHsgbGluZTogcGFydGlhbCwgbW9yZTogIXRoaXMuI2VvZiB9O1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChsaW5lID09PSBudWxsKSB7XG4gICAgICByZXR1cm4gbnVsbDtcbiAgICB9XG5cbiAgICBpZiAobGluZS5ieXRlTGVuZ3RoID09PSAwKSB7XG4gICAgICByZXR1cm4geyBsaW5lLCBtb3JlOiBmYWxzZSB9O1xuICAgIH1cblxuICAgIGlmIChsaW5lW2xpbmUuYnl0ZUxlbmd0aCAtIDFdID09IExGKSB7XG4gICAgICBsZXQgZHJvcCA9IDE7XG4gICAgICBpZiAobGluZS5ieXRlTGVuZ3RoID4gMSAmJiBsaW5lW2xpbmUuYnl0ZUxlbmd0aCAtIDJdID09PSBDUikge1xuICAgICAgICBkcm9wID0gMjtcbiAgICAgIH1cbiAgICAgIGxpbmUgPSBsaW5lLnN1YmFycmF5KDAsIGxpbmUuYnl0ZUxlbmd0aCAtIGRyb3ApO1xuICAgIH1cbiAgICByZXR1cm4geyBsaW5lLCBtb3JlOiBmYWxzZSB9O1xuICB9XG5cbiAgLyoqIGByZWFkU2xpY2UoKWAgcmVhZHMgdW50aWwgdGhlIGZpcnN0IG9jY3VycmVuY2Ugb2YgYGRlbGltYCBpbiB0aGUgaW5wdXQsXG4gICAqIHJldHVybmluZyBhIHNsaWNlIHBvaW50aW5nIGF0IHRoZSBieXRlcyBpbiB0aGUgYnVmZmVyLiBUaGUgYnl0ZXMgc3RvcFxuICAgKiBiZWluZyB2YWxpZCBhdCB0aGUgbmV4dCByZWFkLlxuICAgKlxuICAgKiBJZiBgcmVhZFNsaWNlKClgIGVuY291bnRlcnMgYW4gZXJyb3IgYmVmb3JlIGZpbmRpbmcgYSBkZWxpbWl0ZXIsIG9yIHRoZVxuICAgKiBidWZmZXIgZmlsbHMgd2l0aG91dCBmaW5kaW5nIGEgZGVsaW1pdGVyLCBpdCB0aHJvd3MgYW4gZXJyb3Igd2l0aCBhXG4gICAqIGBwYXJ0aWFsYCBwcm9wZXJ0eSB0aGF0IGNvbnRhaW5zIHRoZSBlbnRpcmUgYnVmZmVyLlxuICAgKlxuICAgKiBJZiBgcmVhZFNsaWNlKClgIGVuY291bnRlcnMgdGhlIGVuZCBvZiB0aGUgdW5kZXJseWluZyBzdHJlYW0gYW5kIHRoZXJlIGFyZVxuICAgKiBhbnkgYnl0ZXMgbGVmdCBpbiB0aGUgYnVmZmVyLCB0aGUgcmVzdCBvZiB0aGUgYnVmZmVyIGlzIHJldHVybmVkLiBJbiBvdGhlclxuICAgKiB3b3JkcywgRU9GIGlzIGFsd2F5cyB0cmVhdGVkIGFzIGEgZGVsaW1pdGVyLiBPbmNlIHRoZSBidWZmZXIgaXMgZW1wdHksXG4gICAqIGl0IHJldHVybnMgYG51bGxgLlxuICAgKlxuICAgKiBCZWNhdXNlIHRoZSBkYXRhIHJldHVybmVkIGZyb20gYHJlYWRTbGljZSgpYCB3aWxsIGJlIG92ZXJ3cml0dGVuIGJ5IHRoZVxuICAgKiBuZXh0IEkvTyBvcGVyYXRpb24sIG1vc3QgY2xpZW50cyBzaG91bGQgdXNlIGByZWFkU3RyaW5nKClgIGluc3RlYWQuXG4gICAqL1xuICBhc3luYyByZWFkU2xpY2UoZGVsaW06IG51bWJlcik6IFByb21pc2U8VWludDhBcnJheSB8IG51bGw+IHtcbiAgICBsZXQgcyA9IDA7IC8vIHNlYXJjaCBzdGFydCBpbmRleFxuICAgIGxldCBzbGljZTogVWludDhBcnJheSB8IHVuZGVmaW5lZDtcblxuICAgIHdoaWxlICh0cnVlKSB7XG4gICAgICAvLyBTZWFyY2ggYnVmZmVyLlxuICAgICAgbGV0IGkgPSB0aGlzLiNidWYuc3ViYXJyYXkodGhpcy4jciArIHMsIHRoaXMuI3cpLmluZGV4T2YoZGVsaW0pO1xuICAgICAgaWYgKGkgPj0gMCkge1xuICAgICAgICBpICs9IHM7XG4gICAgICAgIHNsaWNlID0gdGhpcy4jYnVmLnN1YmFycmF5KHRoaXMuI3IsIHRoaXMuI3IgKyBpICsgMSk7XG4gICAgICAgIHRoaXMuI3IgKz0gaSArIDE7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfVxuXG4gICAgICAvLyBFT0Y/XG4gICAgICBpZiAodGhpcy4jZW9mKSB7XG4gICAgICAgIGlmICh0aGlzLiNyID09PSB0aGlzLiN3KSB7XG4gICAgICAgICAgcmV0dXJuIG51bGw7XG4gICAgICAgIH1cbiAgICAgICAgc2xpY2UgPSB0aGlzLiNidWYuc3ViYXJyYXkodGhpcy4jciwgdGhpcy4jdyk7XG4gICAgICAgIHRoaXMuI3IgPSB0aGlzLiN3O1xuICAgICAgICBicmVhaztcbiAgICAgIH1cblxuICAgICAgLy8gQnVmZmVyIGZ1bGw/XG4gICAgICBpZiAodGhpcy5idWZmZXJlZCgpID49IHRoaXMuI2J1Zi5ieXRlTGVuZ3RoKSB7XG4gICAgICAgIHRoaXMuI3IgPSB0aGlzLiN3O1xuICAgICAgICAvLyAjNDUyMSBUaGUgaW50ZXJuYWwgYnVmZmVyIHNob3VsZCBub3QgYmUgcmV1c2VkIGFjcm9zcyByZWFkcyBiZWNhdXNlIGl0IGNhdXNlcyBjb3JydXB0aW9uIG9mIGRhdGEuXG4gICAgICAgIGNvbnN0IG9sZGJ1ZiA9IHRoaXMuI2J1ZjtcbiAgICAgICAgY29uc3QgbmV3YnVmID0gdGhpcy4jYnVmLnNsaWNlKDApO1xuICAgICAgICB0aGlzLiNidWYgPSBuZXdidWY7XG4gICAgICAgIHRocm93IG5ldyBCdWZmZXJGdWxsRXJyb3Iob2xkYnVmKTtcbiAgICAgIH1cblxuICAgICAgcyA9IHRoaXMuI3cgLSB0aGlzLiNyOyAvLyBkbyBub3QgcmVzY2FuIGFyZWEgd2Ugc2Nhbm5lZCBiZWZvcmVcblxuICAgICAgLy8gQnVmZmVyIGlzIG5vdCBmdWxsLlxuICAgICAgdHJ5IHtcbiAgICAgICAgYXdhaXQgdGhpcy4jZmlsbCgpO1xuICAgICAgfSBjYXRjaCAoZXJyKSB7XG4gICAgICAgIGlmIChlcnIgaW5zdGFuY2VvZiBQYXJ0aWFsUmVhZEVycm9yKSB7XG4gICAgICAgICAgZXJyLnBhcnRpYWwgPSBzbGljZTtcbiAgICAgICAgfVxuICAgICAgICB0aHJvdyBlcnI7XG4gICAgICB9XG4gICAgfVxuXG4gICAgLy8gSGFuZGxlIGxhc3QgYnl0ZSwgaWYgYW55LlxuICAgIC8vIGNvbnN0IGkgPSBzbGljZS5ieXRlTGVuZ3RoIC0gMTtcbiAgICAvLyBpZiAoaSA+PSAwKSB7XG4gICAgLy8gICB0aGlzLmxhc3RCeXRlID0gc2xpY2VbaV07XG4gICAgLy8gICB0aGlzLmxhc3RDaGFyU2l6ZSA9IC0xXG4gICAgLy8gfVxuXG4gICAgcmV0dXJuIHNsaWNlO1xuICB9XG5cbiAgLyoqIGBwZWVrKClgIHJldHVybnMgdGhlIG5leHQgYG5gIGJ5dGVzIHdpdGhvdXQgYWR2YW5jaW5nIHRoZSByZWFkZXIuIFRoZVxuICAgKiBieXRlcyBzdG9wIGJlaW5nIHZhbGlkIGF0IHRoZSBuZXh0IHJlYWQgY2FsbC5cbiAgICpcbiAgICogV2hlbiB0aGUgZW5kIG9mIHRoZSB1bmRlcmx5aW5nIHN0cmVhbSBpcyByZWFjaGVkLCBidXQgdGhlcmUgYXJlIHVucmVhZFxuICAgKiBieXRlcyBsZWZ0IGluIHRoZSBidWZmZXIsIHRob3NlIGJ5dGVzIGFyZSByZXR1cm5lZC4gSWYgdGhlcmUgYXJlIG5vIGJ5dGVzXG4gICAqIGxlZnQgaW4gdGhlIGJ1ZmZlciwgaXQgcmV0dXJucyBgbnVsbGAuXG4gICAqXG4gICAqIElmIGFuIGVycm9yIGlzIGVuY291bnRlcmVkIGJlZm9yZSBgbmAgYnl0ZXMgYXJlIGF2YWlsYWJsZSwgYHBlZWsoKWAgdGhyb3dzXG4gICAqIGFuIGVycm9yIHdpdGggdGhlIGBwYXJ0aWFsYCBwcm9wZXJ0eSBzZXQgdG8gYSBzbGljZSBvZiB0aGUgYnVmZmVyIHRoYXRcbiAgICogY29udGFpbnMgdGhlIGJ5dGVzIHRoYXQgd2VyZSBhdmFpbGFibGUgYmVmb3JlIHRoZSBlcnJvciBvY2N1cnJlZC5cbiAgICovXG4gIGFzeW5jIHBlZWsobjogbnVtYmVyKTogUHJvbWlzZTxVaW50OEFycmF5IHwgbnVsbD4ge1xuICAgIGlmIChuIDwgMCkge1xuICAgICAgdGhyb3cgRXJyb3IoXCJuZWdhdGl2ZSBjb3VudFwiKTtcbiAgICB9XG5cbiAgICBsZXQgYXZhaWwgPSB0aGlzLiN3IC0gdGhpcy4jcjtcbiAgICB3aGlsZSAoYXZhaWwgPCBuICYmIGF2YWlsIDwgdGhpcy4jYnVmLmJ5dGVMZW5ndGggJiYgIXRoaXMuI2VvZikge1xuICAgICAgdHJ5IHtcbiAgICAgICAgYXdhaXQgdGhpcy4jZmlsbCgpO1xuICAgICAgfSBjYXRjaCAoZXJyKSB7XG4gICAgICAgIGlmIChlcnIgaW5zdGFuY2VvZiBQYXJ0aWFsUmVhZEVycm9yKSB7XG4gICAgICAgICAgZXJyLnBhcnRpYWwgPSB0aGlzLiNidWYuc3ViYXJyYXkodGhpcy4jciwgdGhpcy4jdyk7XG4gICAgICAgIH1cbiAgICAgICAgdGhyb3cgZXJyO1xuICAgICAgfVxuICAgICAgYXZhaWwgPSB0aGlzLiN3IC0gdGhpcy4jcjtcbiAgICB9XG5cbiAgICBpZiAoYXZhaWwgPT09IDAgJiYgdGhpcy4jZW9mKSB7XG4gICAgICByZXR1cm4gbnVsbDtcbiAgICB9IGVsc2UgaWYgKGF2YWlsIDwgbiAmJiB0aGlzLiNlb2YpIHtcbiAgICAgIHJldHVybiB0aGlzLiNidWYuc3ViYXJyYXkodGhpcy4jciwgdGhpcy4jciArIGF2YWlsKTtcbiAgICB9IGVsc2UgaWYgKGF2YWlsIDwgbikge1xuICAgICAgdGhyb3cgbmV3IEJ1ZmZlckZ1bGxFcnJvcih0aGlzLiNidWYuc3ViYXJyYXkodGhpcy4jciwgdGhpcy4jdykpO1xuICAgIH1cblxuICAgIHJldHVybiB0aGlzLiNidWYuc3ViYXJyYXkodGhpcy4jciwgdGhpcy4jciArIG4pO1xuICB9XG59XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsMEVBQTBFO0FBQzFFLHFDQUFxQztBQUVyQyxTQUFTLE1BQU0sUUFBUSxzQkFBc0I7QUFDN0MsU0FBUyxJQUFJLFFBQVEsbUJBQW1CO0FBR3hDLE1BQU0sbUJBQW1CO0FBQ3pCLE1B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QUksQ0FBQyxDQUFBLENBQUU7SUFDM0I7SUFFQSxJQUFJLFVBQVUsS0FBSyxJQUFJLENBQUMsQ0FBQSxHQUFJLEVBQUU7TUFDNUIsT0FBTztJQUNULE9BQU8sSUFBSSxRQUFRLEtBQUssSUFBSSxDQUFDLENBQUEsR0FBSSxFQUFFO01BQ2pDLE9BQU8sSUFBSSxDQUFDLENBQUEsR0FBSSxDQUFDLFFBQVEsQ0FBQyxJQUFJLENBQUMsQ0FBQSxDQUFFLEVBQUUsSUFBSSxDQUFDLENBQUEsQ0FBRSxHQUFHO0lBQy9DLE9BQU8sSUFBSSxRQUFRLEdBQUc7TUFDcEIsTUFBTSxJQUFJLGdCQUFnQixJQUFJLENBQUMsQ0FBQSxHQUFJLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQyxDQUFBLENBQUUsRUFBRSxJQUFJLENBQUMsQ0FBQSxDQUFFO0lBQy9EO0lBRUEsT0FBTyxJQUFJLENBQUMsQ0FBQSxHQUFJLENBQUMsUUFBUSxDQUFDLElBQUksQ0FBQyxDQUFBLENBQUUsRUFBRSxJQUFJLENBQUMsQ0FBQSxDQUFFLEdBQUc7RUFDL0M7QUFDRiJ9
// denoCacheMetadata=10712164313377964607,15727447567051977928