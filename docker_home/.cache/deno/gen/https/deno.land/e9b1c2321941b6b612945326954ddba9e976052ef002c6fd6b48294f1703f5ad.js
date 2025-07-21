// Copyright 2018-2024 the Deno authors. All rights reserved. MIT license.
// This module is browser compatible.
import { assertArg } from "../_common/normalize.ts";
import { CHAR_COLON } from "../_common/constants.ts";
import { normalizeString } from "../_common/normalize_string.ts";
import { isPathSeparator, isWindowsDeviceRoot } from "./_util.ts";
/**
 * Normalize the `path`, resolving `'..'` and `'.'` segments.
 * Note that resolving these segments does not necessarily mean that all will be eliminated.
 * A `'..'` at the top-level will be preserved, and an empty path is canonically `'.'`.
 * @param path to be normalized
 */ export function normalize(path) {
  assertArg(path);
  const len = path.length;
  let rootEnd = 0;
  let device;
  let isAbsolute = false;
  const code = path.charCodeAt(0);
  // Try to match a root
  if (len > 1) {
    if (isPathSeparator(code)) {
      // Possible UNC root
      // If we started with a separator, we know we at least have an absolute
      // path of some kind (UNC or otherwise)
      isAbsolute = true;
      if (isPathSeparator(path.charCodeAt(1))) {
        // Matched double path separator at beginning
        let j = 2;
        let last = j;
        // Match 1 or more non-path separators
        for(; j < len; ++j){
          if (isPathSeparator(path.charCodeAt(j))) break;
        }
        if (j < len && j !== last) {
          const firstPart = path.slice(last, j);
          // Matched!
          last = j;
          // Match 1 or more path separators
          for(; j < len; ++j){
            if (!isPathSeparator(path.charCodeAt(j))) break;
          }
          if (j < len && j !== last) {
            // Matched!
            last = j;
            // Match 1 or more non-path separators
            for(; j < len; ++j){
              if (isPathSeparator(path.charCodeAt(j))) break;
            }
            if (j === len) {
              // We matched a UNC root only
              // Return the normalized version of the UNC root since there
              // is nothing left to process
              return `\\\\${firstPart}\\${path.slice(last)}\\`;
            } else if (j !== last) {
              // We matched a UNC root with leftovers
              device = `\\\\${firstPart}\\${path.slice(last, j)}`;
              rootEnd = j;
            }
          }
        }
      } else {
        rootEnd = 1;
      }
    } else if (isWindowsDeviceRoot(code)) {
      // Possible device root
      if (path.charCodeAt(1) === CHAR_COLON) {
        device = path.slice(0, 2);
        rootEnd = 2;
        if (len > 2) {
          if (isPathSeparator(path.charCodeAt(2))) {
            // Treat separator following drive name as an absolute path
            // indicator
            isAbsolute = true;
            rootEnd = 3;
          }
        }
      }
    }
  } else if (isPathSeparator(code)) {
    // `path` contains just a path separator, exit early to avoid unnecessary
    // work
    return "\\";
  }
  let tail;
  if (rootEnd < len) {
    tail = normalizeString(path.slice(rootEnd), !isAbsolute, "\\", isPathSeparator);
  } else {
    tail = "";
  }
  if (tail.length === 0 && !isAbsolute) tail = ".";
  if (tail.length > 0 && isPathSeparator(path.charCodeAt(len - 1))) {
    tail += "\\";
  }
  if (device === undefined) {
    if (isAbsolute) {
      if (tail.length > 0) return `\\${tail}`;
      else return "\\";
    } else if (tail.length > 0) {
      return tail;
    } else {
      return "";
    }
  } else if (isAbsolute) {
    if (tail.length > 0) return `${device}\\${tail}`;
    else return `${device}\\`;
  } else if (tail.length > 0) {
    return device + tail;
  } else {
    return device;
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjIxNC4wL3BhdGgvd2luZG93cy9ub3JtYWxpemUudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyNCB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIFRoaXMgbW9kdWxlIGlzIGJyb3dzZXIgY29tcGF0aWJsZS5cblxuaW1wb3J0IHsgYXNzZXJ0QXJnIH0gZnJvbSBcIi4uL19jb21tb24vbm9ybWFsaXplLnRzXCI7XG5pbXBvcnQgeyBDSEFSX0NPTE9OIH0gZnJvbSBcIi4uL19jb21tb24vY29uc3RhbnRzLnRzXCI7XG5pbXBvcnQgeyBub3JtYWxpemVTdHJpbmcgfSBmcm9tIFwiLi4vX2NvbW1vbi9ub3JtYWxpemVfc3RyaW5nLnRzXCI7XG5pbXBvcnQgeyBpc1BhdGhTZXBhcmF0b3IsIGlzV2luZG93c0RldmljZVJvb3QgfSBmcm9tIFwiLi9fdXRpbC50c1wiO1xuXG4vKipcbiAqIE5vcm1hbGl6ZSB0aGUgYHBhdGhgLCByZXNvbHZpbmcgYCcuLidgIGFuZCBgJy4nYCBzZWdtZW50cy5cbiAqIE5vdGUgdGhhdCByZXNvbHZpbmcgdGhlc2Ugc2VnbWVudHMgZG9lcyBub3QgbmVjZXNzYXJpbHkgbWVhbiB0aGF0IGFsbCB3aWxsIGJlIGVsaW1pbmF0ZWQuXG4gKiBBIGAnLi4nYCBhdCB0aGUgdG9wLWxldmVsIHdpbGwgYmUgcHJlc2VydmVkLCBhbmQgYW4gZW1wdHkgcGF0aCBpcyBjYW5vbmljYWxseSBgJy4nYC5cbiAqIEBwYXJhbSBwYXRoIHRvIGJlIG5vcm1hbGl6ZWRcbiAqL1xuZXhwb3J0IGZ1bmN0aW9uIG5vcm1hbGl6ZShwYXRoOiBzdHJpbmcpOiBzdHJpbmcge1xuICBhc3NlcnRBcmcocGF0aCk7XG5cbiAgY29uc3QgbGVuID0gcGF0aC5sZW5ndGg7XG4gIGxldCByb290RW5kID0gMDtcbiAgbGV0IGRldmljZTogc3RyaW5nIHwgdW5kZWZpbmVkO1xuICBsZXQgaXNBYnNvbHV0ZSA9IGZhbHNlO1xuICBjb25zdCBjb2RlID0gcGF0aC5jaGFyQ29kZUF0KDApO1xuXG4gIC8vIFRyeSB0byBtYXRjaCBhIHJvb3RcbiAgaWYgKGxlbiA+IDEpIHtcbiAgICBpZiAoaXNQYXRoU2VwYXJhdG9yKGNvZGUpKSB7XG4gICAgICAvLyBQb3NzaWJsZSBVTkMgcm9vdFxuXG4gICAgICAvLyBJZiB3ZSBzdGFydGVkIHdpdGggYSBzZXBhcmF0b3IsIHdlIGtub3cgd2UgYXQgbGVhc3QgaGF2ZSBhbiBhYnNvbHV0ZVxuICAgICAgLy8gcGF0aCBvZiBzb21lIGtpbmQgKFVOQyBvciBvdGhlcndpc2UpXG4gICAgICBpc0Fic29sdXRlID0gdHJ1ZTtcblxuICAgICAgaWYgKGlzUGF0aFNlcGFyYXRvcihwYXRoLmNoYXJDb2RlQXQoMSkpKSB7XG4gICAgICAgIC8vIE1hdGNoZWQgZG91YmxlIHBhdGggc2VwYXJhdG9yIGF0IGJlZ2lubmluZ1xuICAgICAgICBsZXQgaiA9IDI7XG4gICAgICAgIGxldCBsYXN0ID0gajtcbiAgICAgICAgLy8gTWF0Y2ggMSBvciBtb3JlIG5vbi1wYXRoIHNlcGFyYXRvcnNcbiAgICAgICAgZm9yICg7IGogPCBsZW47ICsraikge1xuICAgICAgICAgIGlmIChpc1BhdGhTZXBhcmF0b3IocGF0aC5jaGFyQ29kZUF0KGopKSkgYnJlYWs7XG4gICAgICAgIH1cbiAgICAgICAgaWYgKGogPCBsZW4gJiYgaiAhPT0gbGFzdCkge1xuICAgICAgICAgIGNvbnN0IGZpcnN0UGFydCA9IHBhdGguc2xpY2UobGFzdCwgaik7XG4gICAgICAgICAgLy8gTWF0Y2hlZCFcbiAgICAgICAgICBsYXN0ID0gajtcbiAgICAgICAgICAvLyBNYXRjaCAxIG9yIG1vcmUgcGF0aCBzZXBhcmF0b3JzXG4gICAgICAgICAgZm9yICg7IGogPCBsZW47ICsraikge1xuICAgICAgICAgICAgaWYgKCFpc1BhdGhTZXBhcmF0b3IocGF0aC5jaGFyQ29kZUF0KGopKSkgYnJlYWs7XG4gICAgICAgICAgfVxuICAgICAgICAgIGlmIChqIDwgbGVuICYmIGogIT09IGxhc3QpIHtcbiAgICAgICAgICAgIC8vIE1hdGNoZWQhXG4gICAgICAgICAgICBsYXN0ID0gajtcbiAgICAgICAgICAgIC8vIE1hdGNoIDEgb3IgbW9yZSBub24tcGF0aCBzZXBhcmF0b3JzXG4gICAgICAgICAgICBmb3IgKDsgaiA8IGxlbjsgKytqKSB7XG4gICAgICAgICAgICAgIGlmIChpc1BhdGhTZXBhcmF0b3IocGF0aC5jaGFyQ29kZUF0KGopKSkgYnJlYWs7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgICBpZiAoaiA9PT0gbGVuKSB7XG4gICAgICAgICAgICAgIC8vIFdlIG1hdGNoZWQgYSBVTkMgcm9vdCBvbmx5XG4gICAgICAgICAgICAgIC8vIFJldHVybiB0aGUgbm9ybWFsaXplZCB2ZXJzaW9uIG9mIHRoZSBVTkMgcm9vdCBzaW5jZSB0aGVyZVxuICAgICAgICAgICAgICAvLyBpcyBub3RoaW5nIGxlZnQgdG8gcHJvY2Vzc1xuXG4gICAgICAgICAgICAgIHJldHVybiBgXFxcXFxcXFwke2ZpcnN0UGFydH1cXFxcJHtwYXRoLnNsaWNlKGxhc3QpfVxcXFxgO1xuICAgICAgICAgICAgfSBlbHNlIGlmIChqICE9PSBsYXN0KSB7XG4gICAgICAgICAgICAgIC8vIFdlIG1hdGNoZWQgYSBVTkMgcm9vdCB3aXRoIGxlZnRvdmVyc1xuXG4gICAgICAgICAgICAgIGRldmljZSA9IGBcXFxcXFxcXCR7Zmlyc3RQYXJ0fVxcXFwke3BhdGguc2xpY2UobGFzdCwgail9YDtcbiAgICAgICAgICAgICAgcm9vdEVuZCA9IGo7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICB9XG4gICAgICB9IGVsc2Uge1xuICAgICAgICByb290RW5kID0gMTtcbiAgICAgIH1cbiAgICB9IGVsc2UgaWYgKGlzV2luZG93c0RldmljZVJvb3QoY29kZSkpIHtcbiAgICAgIC8vIFBvc3NpYmxlIGRldmljZSByb290XG5cbiAgICAgIGlmIChwYXRoLmNoYXJDb2RlQXQoMSkgPT09IENIQVJfQ09MT04pIHtcbiAgICAgICAgZGV2aWNlID0gcGF0aC5zbGljZSgwLCAyKTtcbiAgICAgICAgcm9vdEVuZCA9IDI7XG4gICAgICAgIGlmIChsZW4gPiAyKSB7XG4gICAgICAgICAgaWYgKGlzUGF0aFNlcGFyYXRvcihwYXRoLmNoYXJDb2RlQXQoMikpKSB7XG4gICAgICAgICAgICAvLyBUcmVhdCBzZXBhcmF0b3IgZm9sbG93aW5nIGRyaXZlIG5hbWUgYXMgYW4gYWJzb2x1dGUgcGF0aFxuICAgICAgICAgICAgLy8gaW5kaWNhdG9yXG4gICAgICAgICAgICBpc0Fic29sdXRlID0gdHJ1ZTtcbiAgICAgICAgICAgIHJvb3RFbmQgPSAzO1xuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cbiAgfSBlbHNlIGlmIChpc1BhdGhTZXBhcmF0b3IoY29kZSkpIHtcbiAgICAvLyBgcGF0aGAgY29udGFpbnMganVzdCBhIHBhdGggc2VwYXJhdG9yLCBleGl0IGVhcmx5IHRvIGF2b2lkIHVubmVjZXNzYXJ5XG4gICAgLy8gd29ya1xuICAgIHJldHVybiBcIlxcXFxcIjtcbiAgfVxuXG4gIGxldCB0YWlsOiBzdHJpbmc7XG4gIGlmIChyb290RW5kIDwgbGVuKSB7XG4gICAgdGFpbCA9IG5vcm1hbGl6ZVN0cmluZyhcbiAgICAgIHBhdGguc2xpY2Uocm9vdEVuZCksXG4gICAgICAhaXNBYnNvbHV0ZSxcbiAgICAgIFwiXFxcXFwiLFxuICAgICAgaXNQYXRoU2VwYXJhdG9yLFxuICAgICk7XG4gIH0gZWxzZSB7XG4gICAgdGFpbCA9IFwiXCI7XG4gIH1cbiAgaWYgKHRhaWwubGVuZ3RoID09PSAwICYmICFpc0Fic29sdXRlKSB0YWlsID0gXCIuXCI7XG4gIGlmICh0YWlsLmxlbmd0aCA+IDAgJiYgaXNQYXRoU2VwYXJhdG9yKHBhdGguY2hhckNvZGVBdChsZW4gLSAxKSkpIHtcbiAgICB0YWlsICs9IFwiXFxcXFwiO1xuICB9XG4gIGlmIChkZXZpY2UgPT09IHVuZGVmaW5lZCkge1xuICAgIGlmIChpc0Fic29sdXRlKSB7XG4gICAgICBpZiAodGFpbC5sZW5ndGggPiAwKSByZXR1cm4gYFxcXFwke3RhaWx9YDtcbiAgICAgIGVsc2UgcmV0dXJuIFwiXFxcXFwiO1xuICAgIH0gZWxzZSBpZiAodGFpbC5sZW5ndGggPiAwKSB7XG4gICAgICByZXR1cm4gdGFpbDtcbiAgICB9IGVsc2Uge1xuICAgICAgcmV0dXJuIFwiXCI7XG4gICAgfVxuICB9IGVsc2UgaWYgKGlzQWJzb2x1dGUpIHtcbiAgICBpZiAodGFpbC5sZW5ndGggPiAwKSByZXR1cm4gYCR7ZGV2aWNlfVxcXFwke3RhaWx9YDtcbiAgICBlbHNlIHJldHVybiBgJHtkZXZpY2V9XFxcXGA7XG4gIH0gZWxzZSBpZiAodGFpbC5sZW5ndGggPiAwKSB7XG4gICAgcmV0dXJuIGRldmljZSArIHRhaWw7XG4gIH0gZWxzZSB7XG4gICAgcmV0dXJuIGRldmljZTtcbiAgfVxufVxuIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBLDBFQUEwRTtBQUMxRSxxQ0FBcUM7QUFFckMsU0FBUyxTQUFTLFFBQVEsMEJBQTBCO0FBQ3BELFNBQVMsVUFBVSxRQUFRLDBCQUEwQjtBQUNyRCxTQUFTLGVBQWUsUUFBUSxpQ0FBaUM7QUFDakUsU0FBUyxlQUFlLEVBQUUsbUJBQW1CLFFBQVEsYUFBYTtBQUVsRTs7Ozs7Q0FLQyxHQUNELE9BQU8sU0FBUyxVQUFVLElBQVk7RUFDcEMsVUFBVTtFQUVWLE1BQU0sTUFBTSxLQUFLLE1BQU07RUFDdkIsSUFBSSxVQUFVO0VBQ2QsSUFBSTtFQUNKLElBQUksYUFBYTtFQUNqQixNQUFNLE9BQU8sS0FBSyxVQUFVLENBQUM7RUFFN0Isc0JBQXNCO0VBQ3RCLElBQUksTUFBTSxHQUFHO0lBQ1gsSUFBSSxnQkFBZ0IsT0FBTztNQUN6QixvQkFBb0I7TUFFcEIsdUVBQXVFO01BQ3ZFLHVDQUF1QztNQUN2QyxhQUFhO01BRWIsSUFB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
// denoCacheMetadata=1016805957230802384,18326365435210133912