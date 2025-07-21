// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
// Copyright the Browserify authors. MIT License.
// Ported from https://github.com/browserify/path-browserify/
// This module is browser compatible.
import { CHAR_DOT, CHAR_FORWARD_SLASH } from "./_constants.ts";
import { _format, assertPath, encodeWhitespace, isPosixPathSeparator, normalizeString } from "./_util.ts";
export const sep = "/";
export const delimiter = ":";
// path.resolve([from ...], to)
/**
 * Resolves `pathSegments` into an absolute path.
 * @param pathSegments an array of path segments
 */ export function resolve(...pathSegments) {
  let resolvedPath = "";
  let resolvedAbsolute = false;
  for(let i = pathSegments.length - 1; i >= -1 && !resolvedAbsolute; i--){
    let path;
    if (i >= 0) path = pathSegments[i];
    else {
      // deno-lint-ignore no-explicit-any
      const { Deno } = globalThis;
      if (typeof Deno?.cwd !== "function") {
        throw new TypeError("Resolved a relative path without a CWD.");
      }
      path = Deno.cwd();
    }
    assertPath(path);
    // Skip empty entries
    if (path.length === 0) {
      continue;
    }
    resolvedPath = `${path}/${resolvedPath}`;
    resolvedAbsolute = path.charCodeAt(0) === CHAR_FORWARD_SLASH;
  }
  // At this point the path should be resolved to a full absolute path, but
  // handle relative paths to be safe (might happen when process.cwd() fails)
  // Normalize the path
  resolvedPath = normalizeString(resolvedPath, !resolvedAbsolute, "/", isPosixPathSeparator);
  if (resolvedAbsolute) {
    if (resolvedPath.length > 0) return `/${resolvedPath}`;
    else return "/";
  } else if (resolvedPath.length > 0) return resolvedPath;
  else return ".";
}
/**
 * Normalize the `path`, resolving `'..'` and `'.'` segments.
 * @param path to be normalized
 */ export function normalize(path) {
  assertPath(path);
  if (path.length === 0) return ".";
  const isAbsolute = path.charCodeAt(0) === CHAR_FORWARD_SLASH;
  const trailingSeparator = path.charCodeAt(path.length - 1) === CHAR_FORWARD_SLASH;
  // Normalize the path
  path = normalizeString(path, !isAbsolute, "/", isPosixPathSeparator);
  if (path.length === 0 && !isAbsolute) path = ".";
  if (path.length > 0 && trailingSeparator) path += "/";
  if (isAbsolute) return `/${path}`;
  return path;
}
/**
 * Verifies whether provided path is absolute
 * @param path to be verified as absolute
 */ export function isAbsolute(path) {
  assertPath(path);
  return path.length > 0 && path.charCodeAt(0) === CHAR_FORWARD_SLASH;
}
/**
 * Join all given a sequence of `paths`,then normalizes the resulting path.
 * @param paths to be joined and normalized
 */ export function join(...paths) {
  if (paths.length === 0) return ".";
  let joined;
  for(let i = 0, len = paths.length; i < len; ++i){
    const path = paths[i];
    assertPath(path);
    if (path.length > 0) {
      if (!joined) joined = path;
      else joined += `/${path}`;
    }
  }
  if (!joined) return ".";
  return normalize(joined);
}
/**
 * Return the relative path from `from` to `to` based on current working directory.
 * @param from path in current working directory
 * @param to path in current working directory
 */ export function relative(from, to) {
  assertPath(from);
  assertPath(to);
  if (from === to) return "";
  from = resolve(from);
  to = resolve(to);
  if (from === to) return "";
  // Trim any leading backslashes
  let fromStart = 1;
  const fromEnd = from.length;
  for(; fromStart < fromEnd; ++fromStart){
    if (from.charCodeAt(fromStart) !== CHAR_FORWARD_SLASH) break;
  }
  const fromLen = fromEnd - fromStart;
  // Trim any leading backslashes
  let toStart = 1;
  const toEnd = to.length;
  for(; toStart < toEnd; ++toStart){
    if (to.charCodeAt(toStart) !== CHAR_FORWARD_SLASH) break;
  }
  const toLen = toEnd - toStart;
  // Compare paths to find the longest common path from root
  const length = fromLen < toLen ? fromLen : toLen;
  let lastCommonSep = -1;
  let i = 0;
  for(; i <= length; ++i){
    if (i === length) {
      if (toLen > length) {
        if (to.charCodeAt(toStart + i) === CHAR_FORWARD_SLASH) {
          // We get here if `from` is the exact base path for `to`.
          // For example: from='/foo/bar'; to='/foo/bar/baz'
          return to.slice(toStart + i + 1);
        } else if (i === 0) {
          // We get here if `from` is the root
          // For example: from='/'; to='/foo'
          return to.slice(toStart + i);
        }
      } else if (fromLen > length) {
        if (from.charCodeAt(fromStart + i) === CHAR_FORWARD_SLASH) {
          // We get here if `to` is the exact base path for `from`.
          // For example: from='/foo/bar/baz'; to='/foo/bar'
          lastCommonSep = i;
        } else if (i === 0) {
          // We get here if `to` is the root.
          // For example: from='/foo'; to='/'
          lastCommonSep = 0;
        }
      }
      break;
    }
    const fromCode = from.charCodeAt(fromStart + i);
    const toCode = to.charCodeAt(toStart + i);
    if (fromCode !== toCode) break;
    else if (fromCode === CHAR_FORWARD_SLASH) lastCommonSep = i;
  }
  let out = "";
  // Generate the relative path based on the path difference between `to`
  // and `from`
  for(i = fromStart + lastCommonSep + 1; i <= fromEnd; ++i){
    if (i === fromEnd || from.charCodeAt(i) === CHAR_FORWARD_SLASH) {
      if (out.length === 0) out += "..";
      else out += "/..";
    }
  }
  // Lastly, append the rest of the destination (`to`) path that comes after
  // the common path parts
  if (out.length > 0) return out + to.slice(toStart + lastCommonSep);
  else {
    toStart += lastCommonSep;
    if (to.charCodeAt(toStart) === CHAR_FORWARD_SLASH) ++toStart;
    return to.slice(toStart);
  }
}
/**
 * Resolves path to a namespace path
 * @param path to resolve to namespace
 */ export function toNamespacedPath(path) {
  // Non-op on posix systems
  return path;
}
/**
 * Return the directory name of a `path`.
 * @param path to determine name for
 */ export function dirname(path) {
  assertPath(path);
  if (path.length === 0) return ".";
  const hasRoot = path.charCodeAt(0) === CHAR_FORWARD_SLASH;
  let end = -1;
  let matchedSlash = true;
  for(let i = path.length - 1; i >= 1; --i){
    if (path.charCodeAt(i) === CHAR_FORWARD_SLASH) {
      if (!matchedSlash) {
        end = i;
        break;
      }
    } else {
      // We saw the first non-path separator
      matchedSlash = false;
    }
  }
  if (end === -1) return hasRoot ? "/" : ".";
  if (hasRoot && end === 1) return "//";
  return path.slice(0, end);
}
/**
 * Return the last portion of a `path`. Trailing directory separators are ignored.
 * @param path to process
 * @param ext of path directory
 */ export function basename(path, ext = "") {
  if (ext !== undefined && typeof ext !== "string") {
    throw new TypeError('"ext" argument must be a string');
  }
  assertPath(path);
  let start = 0;
  let end = -1;
  let matchedSlash = true;
  let i;
  if (ext !== undefined && ext.length > 0 && ext.length <= path.length) {
    if (ext.length === path.length && ext === path) return "";
    let extIdx = ext.length - 1;
    let firstNonSlashEnd = -1;
    for(i = path.length - 1; i >= 0; --i){
      const code = path.charCodeAt(i);
      if (code === CHAR_FORWARD_SLASH) {
        // If we reached a path separator that was not part of a set of path
        // separators at the end of the string, stop now
        if (!matchedSlash) {
          start = i + 1;
          break;
        }
      } else {
        if (firstNonSlashEnd === -1) {
          // We saw the first non-path separator, remember this index in case
          // we need it if the extension ends up not matching
          matchedSlash = false;
          firstNonSlashEnd = i + 1;
        }
        if (extIdx >= 0) {
          // Try to match the explicit extension
          if (code === ext.charCodeAt(extIdx)) {
            if (--extIdx === -1) {
              // We matched the extension, so mark this as the end of our path
              // component
              end = i;
            }
          } else {
            // Extension does not match, so our result is the entire path
            // component
            extIdx = -1;
            end = firstNonSlashEnd;
          }
        }
      }
    }
    if (start === end) end = firstNonSlashEnd;
    else if (end === -1) end = path.length;
    return path.slice(start, end);
  } else {
    for(i = path.length - 1; i >= 0; --i){
      if (path.charCodeAt(i) === CHAR_FORWARD_SLASH) {
        // If we reached a path separator that was not part of a set of path
        // separators at the end of the string, stop now
        if (!matchedSlash) {
          start = i + 1;
          break;
        }
      } else if (end === -1) {
        // We saw the first non-path separator, mark this as the end of our
        // path component
        matchedSlash = false;
        end = i + 1;
      }
    }
    if (end === -1) return "";
    return path.slice(start, end);
  }
}
/**
 * Return the extension of the `path`.
 * @param path with extension
 */ export function extname(path) {
  assertPath(path);
  let startDot = -1;
  let startPart = 0;
  let end = -1;
  let matchedSlash = true;
  // Track the state of characters (if any) we see before our first dot and
  // after any path separator we find
  let preDotState = 0;
  for(let i = path.length - 1; i >= 0; --i){
    const code = path.charCodeAt(i);
    if (code === CHAR_FORWARD_SLASH) {
      // If we reached a path separator that was not part of a set of path
      // separators at the end of the string, stop now
      if (!matchedSlash) {
        startPart = i + 1;
        break;
      }
      continue;
    }
    if (end === -1) {
      // We saw the first non-path separator, mark this as the end of our
      // extension
      matchedSlash = false;
      end = i + 1;
    }
    if (code === CHAR_DOT) {
      // If this is our first dot, mark it as the start of our extension
      if (startDot === -1) startDot = i;
      else if (preDotState !== 1) preDotState = 1;
    } else if (startDot !== -1) {
      // We saw a non-dot and non-path separator before our dot, so we should
      // have a good chance at having a non-empty extension
      preDotState = -1;
    }
  }
  if (startDot === -1 || end === -1 || // We saw a non-dot character immediately before the dot
  preDotState === 0 || // The (right-most) trimmed path component is exactly '..'
  preDotState === 1 && startDot === end - 1 && startDot === startPart + 1) {
    return "";
  }
  return path.slice(startDot, end);
}
/**
 * Generate a path from `FormatInputPathObject` object.
 * @param pathObject with path
 */ export function format(pathObject) {
  if (pathObject === null || typeof pathObject !== "object") {
    throw new TypeError(`The "pathObject" argument must be of type Object. Received type ${typeof pathObject}`);
  }
  return _format("/", pathObject);
}
/**
 * Return a `ParsedPath` object of the `path`.
 * @param path to process
 */ export function parse(path) {
  assertPath(path);
  const ret = {
    root: "",
    dir: "",
    base: "",
    ext: "",
    name: ""
  };
  if (path.length === 0) return ret;
  const isAbsolute = path.charCodeAt(0) === CHAR_FORWARD_SLASH;
  let start;
  if (isAbsolute) {
    ret.root = "/";
    start = 1;
  } else {
    start = 0;
  }
  let startDot = -1;
  let startPart = 0;
  let end = -1;
  let matchedSlash = true;
  let i = path.length - 1;
  // Track the state of characters (if any) we see before our first dot and
  // after any path separator we find
  let preDotState = 0;
  // Get non-dir info
  for(; i >= start; --i){
    const code = path.charCodeAt(i);
    if (code === CHAR_FORWARD_SLASH) {
      // If we reached a path separator that was not part of a set of path
      // separators at the end of the string, stop now
      if (!matchedSlash) {
        startPart = i + 1;
        break;
      }
      continue;
    }
    if (end === -1) {
      // We saw the first non-path separator, mark this as the end of our
      // extension
      matchedSlash = false;
      end = i + 1;
    }
    if (code === CHAR_DOT) {
      // If this is our first dot, mark it as the start of our extension
      if (startDot === -1) startDot = i;
      else if (preDotState !== 1) preDotState = 1;
    } else if (startDot !== -1) {
      // We saw a non-dot and non-path separator before our dot, so we should
      // have a good chance at having a non-empty extension
      preDotState = -1;
    }
  }
  if (startDot === -1 || end === -1 || // We saw a non-dot character immediately before the dot
  preDotState === 0 || // The (right-most) trimmed path component is exactly '..'
  preDotState === 1 && startDot === end - 1 && startDot === startPart + 1) {
    if (end !== -1) {
      if (startPart === 0 && isAbsolute) {
        ret.base = ret.name = path.slice(1, end);
      } else {
        ret.base = ret.name = path.slice(startPart, end);
      }
    }
  } else {
    if (startPart === 0 && isAbsolute) {
      ret.name = path.slice(1, startDot);
      ret.base = path.slice(1, end);
    } else {
      ret.name = path.slice(startPart, startDot);
      ret.base = path.slice(startPart, end);
    }
    ret.ext = path.slice(startDot, end);
  }
  if (startPart > 0) ret.dir = path.slice(0, startPart - 1);
  else if (isAbsolute) ret.dir = "/";
  return ret;
}
/**
 * Converts a file URL to a path string.
 *
 * ```ts
 *      import { fromFileUrl } from "./posix.ts";
 *      fromFileUrl("file:///home/foo"); // "/home/foo"
 * ```
 * @param url of a file URL
 */ export function fromFileUrl(url) {
  url = url instanceof URL ? url : new URL(url);
  if (url.protocol != "file:") {
    throw new TypeError("Must be a file URL.");
  }
  return decodeURIComponent(url.pathname.replace(/%(?![0-9A-Fa-f]{2})/g, "%25"));
}
/**
 * Converts a path string to a file URL.
 *
 * ```ts
 *      import { toFileUrl } from "./posix.ts";
 *      toFileUrl("/home/foo"); // new URL("file:///home/foo")
 * ```
 * @param path to convert to file URL
 */ export function toFileUrl(path) {
  if (!isAbsolute(path)) {
    throw new TypeError("Must be an absolute path.");
  }
  const url = new URL("file:///");
  url.pathname = encodeWhitespace(path.replace(/%/g, "%25").replace(/\\/g, "%5C"));
  return url;
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjEzMy4wL3BhdGgvcG9zaXgudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyMiB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIENvcHlyaWdodCB0aGUgQnJvd3NlcmlmeSBhdXRob3JzLiBNSVQgTGljZW5zZS5cbi8vIFBvcnRlZCBmcm9tIGh0dHBzOi8vZ2l0aHViLmNvbS9icm93c2VyaWZ5L3BhdGgtYnJvd3NlcmlmeS9cbi8vIFRoaXMgbW9kdWxlIGlzIGJyb3dzZXIgY29tcGF0aWJsZS5cblxuaW1wb3J0IHR5cGUgeyBGb3JtYXRJbnB1dFBhdGhPYmplY3QsIFBhcnNlZFBhdGggfSBmcm9tIFwiLi9faW50ZXJmYWNlLnRzXCI7XG5pbXBvcnQgeyBDSEFSX0RPVCwgQ0hBUl9GT1JXQVJEX1NMQVNIIH0gZnJvbSBcIi4vX2NvbnN0YW50cy50c1wiO1xuXG5pbXBvcnQge1xuICBfZm9ybWF0LFxuICBhc3NlcnRQYXRoLFxuICBlbmNvZGVXaGl0ZXNwYWNlLFxuICBpc1Bvc2l4UGF0aFNlcGFyYXRvcixcbiAgbm9ybWFsaXplU3RyaW5nLFxufSBmcm9tIFwiLi9fdXRpbC50c1wiO1xuXG5leHBvcnQgY29uc3Qgc2VwID0gXCIvXCI7XG5leHBvcnQgY29uc3QgZGVsaW1pdGVyID0gXCI6XCI7XG5cbi8vIHBhdGgucmVzb2x2ZShbZnJvbSAuLi5dLCB0bylcbi8qKlxuICogUmVzb2x2ZXMgYHBhdGhTZWdtZW50c2AgaW50byBhbiBhYnNvbHV0ZSBwYXRoLlxuICogQHBhcmFtIHBhdGhTZWdtZW50cyBhbiBhcnJheSBvZiBwYXRoIHNlZ21lbnRzXG4gKi9cbmV4cG9ydCBmdW5jdGlvbiByZXNvbHZlKC4uLnBhdGhTZWdtZW50czogc3RyaW5nW10pOiBzdHJpbmcge1xuICBsZXQgcmVzb2x2ZWRQYXRoID0gXCJcIjtcbiAgbGV0IHJlc29sdmVkQWJzb2x1dGUgPSBmYWxzZTtcblxuICBmb3IgKGxldCBpID0gcGF0aFNlZ21lbnRzLmxlbmd0aCAtIDE7IGkgPj0gLTEgJiYgIXJlc29sdmVkQWJzb2x1dGU7IGktLSkge1xuICAgIGxldCBwYXRoOiBzdHJpbmc7XG5cbiAgICBpZiAoaSA+PSAwKSBwYXRoID0gcGF0aFNlZ21lbnRzW2ldO1xuICAgIGVsc2Uge1xuICAgICAgLy8gZGVuby1saW50LWlnbm9yZSBuby1leHBsaWNpdC1hbnlcbiAgICAgIGNvbnN0IHsgRGVubyB9ID0gZ2xvYmFsVGhpcyBhcyBhbnk7XG4gICAgICBpZiAodHlwZW9mIERlbm8/LmN3ZCAhPT0gXCJmdW5jdGlvblwiKSB7XG4gICAgICAgIHRocm93IG5ldyBUeXBlRXJyb3IoXCJSZXNvbHZlZCBhIHJlbGF0aXZlIHBhdGggd2l0aG91dCBhIENXRC5cIik7XG4gICAgICB9XG4gICAgICBwYXRoID0gRGVuby5jd2QoKTtcbiAgICB9XG5cbiAgICBhc3NlcnRQYXRoKHBhdGgpO1xuXG4gICAgLy8gU2tpcCBlbXB0eSBlbnRyaWVzXG4gICAgaWYgKHBhdGgubGVuZ3RoID09PSAwKSB7XG4gICAgICBjb250aW51ZTtcbiAgICB9XG5cbiAgICByZXNvbHZlZFBhdGggPSBgJHtwYXRofS8ke3Jlc29sdmVkUGF0aH1gO1xuICAgIHJlc29sdmVkQWJzb2x1dGUgPSBwYXRoLmNoYXJDb2RlQXQoMCkgPT09IENIQVJfRk9SV0FSRF9TTEFTSDtcbiAgfVxuXG4gIC8vIEF0IHRoaXMgcG9pbnQgdGhlIHBhdGggc2hvdWxkIGJlIHJlc29sdmVkIHRvIGEgZnVsbCBhYnNvbHV0ZSBwYXRoLCBidXRcbiAgLy8gaGFuZGxlIHJlbGF0aXZlIHBhdGhzIHRvIGJlIHNhZmUgKG1pZ2h0IGhhcHBlbiB3aGVuIHByb2Nlc3MuY3dkKCkgZmFpbHMpXG5cbiAgLy8gTm9ybWFsaXplIHRoZSBwYXRoXG4gIHJlc29sdmVkUGF0aCA9IG5vcm1hbGl6ZVN0cmluZyhcbiAgICByZXNvbHZlZFBhdGgsXG4gICAgIXJlc29sdmVkQWJzb2x1dGUsXG4gICAgXCIvXCIsXG4gICAgaXNQb3NpeFBhdGhTZXBhcmF0b3IsXG4gICk7XG5cbiAgaWYgKHJlc29sdmVkQWJzb2x1dGUpIHtcbiAgICBpZiAocmVzb2x2ZWRQYXRoLmxlbmd0aCA+IDApIHJldHVybiBgLyR7cmVzb2x2ZWRQYXRofWA7XG4gICAgZWxzZSByZXR1cm4gXCIvXCI7XG4gIH0gZWxzZSBpZiAocmVzb2x2ZWRQYXRoLmxlbmd0aCA+IDApIHJldHVybiByZXNvbHZlZFBhdGg7XG4gIGVsc2UgcmV0dXJuIFwiLlwiO1xufVxuXG4vKipcbiAqIE5vcm1hbGl6ZSB0aGUgYHBhdGhgLCByZXNvbHZpbmcgYCcuLidgIGFuZCBgJy4nYCBzZWdtZW50cy5cbiAqIEBwYXJhbSBwYXRoIHRvIGJlIG5vcm1hbGl6ZWRcbiAqL1xuZXhwb3J0IGZ1bmN0aW9uIG5vcm1hbGl6ZShwYXRoOiBzdHJpbmcpOiBzdHJpbmcge1xuICBhc3NlcnRQYXRoKHBhdGgpO1xuXG4gIGlmIChwYXRoLmxlbmd0aCA9PT0gMCkgcmV0dXJuIFwiLlwiO1xuXG4gIGNvbnN0IGlzQWJzb2x1dGUgPSBwYXRoLmNoYXJDb2RlQXQoMCkgPT09IENIQVJfRk9SV0FSRF9TTEFTSDtcbiAgY29uc3QgdHJhaWxpbmdTZXBhcmF0b3IgPVxuICAgIHBhdGguY2hhckNvZGVBdChwYXRoLmxlbmd0aCAtIDEpID09PSBDSEFSX0ZPUldBUkRfU0xBU0g7XG5cbiAgLy8gTm9ybWFsaXplIHRoZSBwYXRoXG4gIHBhdGggPSBub3JtYWxpemVTdHJpbmcocGF0aCwgIWlzQWJzb2x1dGUsIFwiL1wiLCBpc1Bvc2l4UGF0aFNlcGFyYXRvcik7XG5cbiAgaWYgKHBhdGgubGVuZ3RoID09PSAwICYmICFpc0Fic29sdXRlKSBwYXRoID0gXCIuXCI7XG4gIGlmIChwYXRoLmxlbmd0aCA+IDAgJiYgdHJhaWxpbmdTZXBhcmF0b3IpIHBhdGggKz0gXCIvXCI7XG5cbiAgaWYgKGlzQWJzb2x1dGUpIHJldHVybiBgLyR7cGF0aH1gO1xuICByZXR1cm4gcGF0aDtcbn1cblxuLyoqXG4gKiBWZXJpZmllcyB3aGV0aGVyIHByb3ZpZGVkIHBhdGggaXMgYWJzb2x1dGVcbiAqIEBwYXJhbSBwYXRoIHRvIGJlIHZlcmlmaWVkIGFzIGFic29sdXRlXG4gKi9cbmV4cG9ydCBmdW5jdGlvbiBpc0Fic29sdXRlKHBhdGg6IHN0cmluZyk6IGJvb2xlYW4ge1xuICBhc3NlcnRQYXRoKHBhdGgpO1xuICByZXR1cm4gcGF0aC5sZW5ndGggPiAwICYmIHBhdGguY2hhckNvZGVBdCgwKSA9PT0gQ0hBUl9GT1JXQVJEX1NMQVNIO1xufVxuXG4vKipcbiAqIEpvaW4gYWxsIGdpdmVuIGEgc2VxdWVuY2Ugb2YgYHBhdGhzYCx0aGVuIG5vcm1hbGl6ZXMgdGhlIHJlc3VsdGluZyBwYXRoLlxuICogQHBhcmFtIHBhdGhzIHRvIGJlIGpvaW5lZCBhbmQgbm9ybWFsaXplZFxuICovXG5leHBvcnQgZnVuY3Rpb24gam9pbiguLi5wYXRoczogc3RyaW5nW10pOiBzdHJpbmcge1xuICBpZiAocGF0aHMubGVuZ3RoID09PSAwKSByZXR1cm4gXCIuXCI7XG4gIGxldCBqb2luZWQ6IHN0cmluZyB8IHVuZGVmaW5lZDtcbiAgZm9yIChsZXQgaSA9IDAsIGxlbiA9IHBhdGhzLmxlbmd0aDsgaSA8IGxlbjsgKytpKSB7XG4gICAgY29uc3QgcGF0aCA9IHBhdGhzW2ldO1xuICAgIGFzc2VydFBhdGgocGF0aCk7XG4gICAgaWYgKHBhdGgubGVuZ3RoID4gMCkge1xuICAgICAgaWYgKCFqb2luZWQpIGpvaW5lZCA9IHBhdGg7XG4gICAgICBlbHNlIGpvaW5lZCArPSBgLyR7cGF0aH1gO1xuICAgIH1cbiAgfVxuICBpZiAoIWpvaW5lZCkgcmV0dXJuIFwiLlwiO1xuICByZXR1cm4gbm9ybWFsaXplKGpvaW5lZCk7XG59XG5cbi8qKlxuICogUmV0dXJuIHRoZSByZWxhdGl2ZSBwYXRoIGZyb20gYGZyb21gIHRvIGB0b2AgYmFzZWQgb24gY3VycmVudCB3b3JraW5nIGRpcmVjdG9yeS5cbiAqIEBwYXJhbSBmcm9tIHBhdGggaW4gY3VycmVudCB3b3JraW5nIGRpcmVjdG9yeVxuICogQHBhcmFtIHRvIHBhdGggaW4gY3VycmVudCB3b3JraW5nIGRpcmVjdG9yeVxuICovXG5leHBvcnQgZnVuY3Rpb24gcmVsYXRpdmUoZnJvbTogc3RyaW5nLCB0bzogc3RyaW5nKTogc3RyaW5nIHtcbiAgYXNzZXJ0UGF0aChmcm9tKTtcbiAgYXNzZXJ0UGF0aCh0byk7XG5cbiAgaWYgKGZyb20gPT09IHRvKSByZXR1cm4gXCJcIjtcblxuICBmcm9tID0gcmVzb2x2ZShmcm9tKTtcbiAgdG8gPSByZXNvbHZlKHRvKTtcblxuICBpZiAoZnJvbSA9PT0gdG8pIHJldHVybiBcIlwiO1xuXG4gIC8vIFRyaW0gYW55IGxlYWRpbmcgYmFja3NsYXNoZXNcbiAgbGV0IGZyb21TdGFydCA9IDE7XG4gIGNvbnN0IGZyb21FbmQgPSBmcm9tLmxlbmd0aDtcbiAgZm9yICg7IGZyb21TdGFydCA8IGZyb21FbmQ7ICsrZnJvbVN0YXJ0KSB7XG4gICAgaWYgKGZyb20uY2hhckNvZGVBdChmcm9tU3RhcnQpICE9PSBDSEFSX0ZPUldBUkRfU0xBU0gpIGJyZWFrO1xuICB9XG4gIGNvbnN0IGZyb21MZW4gPSBmcm9tRW5kIC0gZnJvbVN0YXJ0O1xuXG4gIC8vIFRyaW0gYW55IGxlYWRpbmcgYmFja3NsYXNoZXNcbiAgbGV0IHRvU3RhcnQgPSAxO1xuICBjb25zdCB0b0VuZCA9IHRvLmxlbmd0aDtcbiAgZm9yICg7IHRvU3RhcnQgPCB0b0VuZDsgKyt0b1N0YXJ0KSB7XG4gICAgaWYgKHRvLmNoYXJDb2RlQXQodG9TdGFydCkgIT09IENIQVJfRk9SV0FSRF9TTEFTSCkgYnJlYWs7XG4gIH1cbiAgY29uc3QgdG9MZW4gPSB0b0VuZCAtIHRvU3RhcnQ7XG5cbiAgLy8gQ29tcGFyZSBwYXRocyB0byBmaW5kIHRoZSBsb25nZXN0IGNvbW1vbiBwYXRoIGZyb20gcm9vdFxuICBjb25zdCBsZW5ndGggPSBmcm9tTGVuIDwgdG9MZW4gPyBmcm9tTGVuIDogdG9MZW47XG4gIGxldCBsYXN0Q29tbW9uU2VwID0gLTE7XG4gIGxldCBpID0gMDtcbiAgZm9yICg7IGkgPD0gbGVuZ3RoOyArK2kpIHtcbiAgICBpZiAoaSA9PT0gbGVuZ3RoKSB7XG4gICAgICBpZiAodG9MZW4gPiBsZW5ndGgpIHtcbiAgICAgICAgaWYgKHRvLmNoYXJDb2RlQXQodG9TdGFydCArIGkpID09PSBDSEFSX0ZPUldBUkRfU0xBU0gpIHtcbiAgICAgICAgICAvLyBXZSBnZXQgaGVyZSBpZiBgZnJvbWAgaXMgdGhlIGV4YWN0IGJhc2UgcGF0aCBmb3IgYHRvYC5cbiAgICAgICAgICAvLyBGb3IgZXhhbXBsZTogZnJvbT0nL2Zvby9iYXInOyB0bz0nL2Zvby9iYXIvYmF6J1xuICAgICAgICAgIHJldHVybiB0by5zbGljZSh0b1N0YXJ0ICsgaSArIDEpO1xuICAgICAgICB9IGVsc2UgaWYgKGkgPT09IDApIHtcbiAgICAgICAgICAvLyBXZSBnZXQgaGVyZSBpZiBgZnJvbWAgaXMgdGhlIHJvb3RcbiAgICAgICAgICAvLyBGb3IgZXhhbXBsZTogZnJvbT0nLyc7IHRvPScvZm9vJ1xuICAgICAgICAgIHJldHVybiB0by5zbGljZSh0b1N0YXJ0ICsgaSk7XG4gICAgICAgIH1cbiAgICAgIH0gZWxzZSBpZiAoZnJvbUxlbiA+IGxlbmd0aCkge1xuICAgICAgICBpZiAoZnJvbS5jaGFyQ29kZUF0KGZyb21TdGFydCArIGkpID09PSBDSEFSX0ZPUldBUkRfU0xBU0gpIHtcbiAgICAgICAgICAvLyBXZSBnZXQgaGVyZSBpZiBgdG9gIGlzIHRoZSBleGFjdCBiYXNlIHBhdGggZm9yIGBmcm9tYC5cbiAgICAgICAgICAvLyBGb3IgZXhhbXBsZTogZnJvbT0nL2Zvby9iYXIvYmF6JzsgdG89Jy9mb28vYmFyJ1xuICAgICAgICAgIGxhc3RDb21tb25TZXAgPSBpO1xuICAgICAgICB9IGVsc2UgaWYgKGkgPT09IDApIHtcbiAgICAgICAgICAvLyBXZSBnZXQgaGVyZSBpZiBgdG9gIGlzIHRoZSByb290LlxuICAgICAgICAgIC8vIEZvciBleGFtcGxlOiBmcm9tPScvZm9vJzsgdG89Jy8nXG4gICAgICAgICAgbGFzdENvbW1vblNlcCA9IDA7XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICAgIGJyZWFrO1xuICAgIH1cbiAgICBjb25zdCBmcm9tQ29kZSA9IGZyb20uY2hhckNvZGVBdChmcm9tU3RhcnQgKyBpKTtcbiAgICBjb25zdCB0b0NvZGUgPSB0by5jaGFyQ29kZUF0KHRvU3RhcnQgKyBpKTtcbiAgICBpZiAoZnJvbUNvZGUgIT09IHRvQ29kZSkgYnJlYWs7XG4gICAgZWxzZSBpZiAoZnJvbUNvZGUgPT09IENIQVJfRk9SV0FSRF9TTEFTSCkgbGFzdENvbW1vblNlcCA9IGk7XG4gIH1cblxuICBsZXQgb3V0ID0gXCJcIjtcbiAgLy8gR2VuZXJhdGUgdGhlIHJlbGF0aXZlIHBhdGggYmFzZWQgb24gdGhlIHBhdGggZGlmZmVyZW5jZSBiZXR3ZWVuIGB0b2BcbiAgLy8gYW5kIGBmcm9tYFxuICBmb3IgKGkgPSBmcm9tU3RhcnQgKyBsYXN0Q29tbW9uU2VwICsgMTsgaSA8PSBmcm9tRW5kOyArK2kpIHtcbiAgICBpZiAoaSA9PT0gZnJvbUVuZCB8fCBmcm9tLmNoYXJDb2RlQXQoaSkgPT09IENIQVJfRk9SV0FSRF9TTEFTSCkge1xuICAgICAgaWYgKG91dC5sZW5ndGggPT09IDApIG91dCArPSBcIi4uXCI7XG4gICAgICBlbHNlIG91dCArPSBcIi8uLlwiO1xuICAgIH1cbiAgfVxuXG4gIC8vIExhc3RseSwgYXBwZW5kIHRoZSByZXN0IG9mIHRoZSBkZXN0aW5hdGlvbiAoYHRvYCkgcGF0aCB0aGF0IGNvbWVzIGFmdGVyXG4gIC8vIHRoZSBjb21tb24gcGF0aCBwYXJ0c1xuICBpZiAob3V0Lmxlbmd0aCA+IDApIHJldHVybiBvdXQgKyB0by5zbGljZSh0b1N0YXJ0ICsgbGFzdENvbW1vblNlcCk7XG4gIGVsc2Uge1xuICAgIHRvU3RhcnQgKz0gbGFzdENvbW1vblNlcDtcbiAgICBpZiAodG8uY2hhckNvZGVBdCh0b1N0YXJ0KSA9PT0gQ0hBUl9GT1JXQVJEX1NMQVNIKSArK3RvU3RhcnQ7XG4gICAgcmV0dXJuIHRvLnNsaWNlKHRvU3RhcnQpO1xuICB9XG59XG5cbi8qKlxuICogUmVzb2x2ZXMgcGF0aCB0byBhIG5hbWVzcGFjZSBwYXRoXG4gKiBAcGFyYW0gcGF0aCB0byByZXNvbHZlIHRvIG5hbWVzcGFjZVxuICovXG5leHBvcnQgZnVuY3Rpb24gdG9OYW1lc3BhY2VkUGF0aChwYXRoOiBzdHJpbmcpOiBzdHJpbmcge1xuICAvLyBOb24tb3Agb24gcG9zaXggc3lzdGVtc1xuICByZXR1cm4gcGF0aDtcbn1cblxuLyoqXG4gKiBSZXR1cm4gdGhlIGRpcmVjdG9yeSBuYW1lIG9mIGEgYHBhdGhgLlxuICogQHBhcmFtIHBhdGggdG8gZGV0ZXJtaW5lIG5hbWUgZm9yXG4gKi9cbmV4cG9ydCBmdW5jdGlvbiBkaXJuYW1lKHBhdGg6IHN0cmluZyk6IHN0cmluZyB7XG4gIGFzc2VydFBhdGgocGF0aCk7XG4gIGlmIChwYXRoLmxlbmd0aCA9PT0gMCkgcmV0dXJuIFwiLlwiO1xuICBjb25zdCBoYXNSb290ID0gcGF0aC5jaGFyQ29kZUF0KDApID09PSBDSEFSX0ZPUldBUkRfU0xBU0g7XG4gIGxldCBlbmQgPSAtMTtcbiAgbGV0IG1hdGNoZWRTbGFzaCA9IHRydWU7XG4gIGZvciAobGV0IGkgPSBwYXRoLmxlbmd0aCAtIDE7IGkgPj0gMTsgLS1pKSB7XG4gICAgaWYgKHBhdGguY2hhckNvZGVBdChpKSA9PT0gQ0hBUl9GT1JXQVJEX1NMQVNIKSB7XG4gICAgICBpZiAoIW1hdGNoZWRTbGFzaCkge1xuICAgICAgICBlbmQgPSBpO1xuICAgICAgICBicmVhaztcbiAgICAgIH1cbiAgICB9IGVsc2Uge1xuICAgICAgLy8gV2Ugc2F3IHRoZSBmaXJzdCBub24tcGF0aCBzZXBhcmF0b3JcbiAgICAgIG1hdGNoZWRTbGFzaCA9IGZhbHNlO1xuICAgIH1cbiAgfVxuXG4gIGlmIChlbmQgPT09IC0xKSByZXR1cm4gaGFzUm9vdCA/IFwiL1wiIDogXCIuXCI7XG4gIGlmIChoYXNSb290ICYmIGVuZCA9PT0gMSkgcmV0dXJuIFwiLy9cIjtcbiAgcmV0dXJuIHBhdGguc2xpY2UoMCwgZW5kKTtcbn1cblxuLyoqXG4gKiBSZXR1cm4gdGhlIGxhc3QgcG9ydGlvbiBvZiBhIGBwYXRoYC4gVHJhaWxpbmcgZGlyZWN0b3J5IHNlcGFyYXRvcnMgYXJlIGlnbm9yZWQuXG4gKiBAcGFyYW0gcGF0aCB0byBwcm9jZXNzXG4gKiBAcGFyYW0gZXh0IG9mIHBhdGggZGlyZWN0b3J5XG4gKi9cbmV4cG9ydCBmdW5jdGlvbiBiYXNlbmFtZShwYXRoOiBzdHJpbmcsIGV4dCA9IFwiXCIpOiBzdHJpbmcge1xuICBpZiAoZXh0ICE9PSB1bmRlZmluZWQgJiYgdHlwZW9mIGV4dCAhPT0gXCJzdHJpbmdcIikge1xuICAgIHRocm93IG5ldyBUeXBlRXJyb3IoJ1wiZXh0XCIgYXJndW1lbnQgbXVzdCBiZSBhIHN0cmluZycpO1xuICB9XG4gIGFzc2VydFBhdGgocGF0aCk7XG5cbiAgbGV0IHN0YXJ0ID0gMDtcbiAgbGV0IGVuZCA9IC0xO1xuICBsZXQgbWF0Y2hlZFNsYXNoID0gdHJ1ZTtcbiAgbGV0IGk6IG51bWJlcjtcblxuICBpZiAoZXh0ICE9PSB1bmRlZmluZWQgJiYgZXh0Lmxlbmd0aCA+IDAgJiYgZXh0Lmxlbmd0aCA8PSBwYXRoLmxlbmd0aCkge1xuICAgIGlmIChleHQubGVuZ3RoID09PSBwYXRoLmxlbmd0aCAmJiBleHQgPT09IHBhdGgpIHJldHVybiBcIlwiO1xuICAgIGxldCBleHRJZHggPSBleHQubGVuZ3RoIC0gMTtcbiAgICBsZXQgZmlyc3ROb25TbGFzaEVuZCA9IC0xO1xuICAgIGZvciAoaSA9IHBhdGgubGVuZ3RoIC0gMTsgaSA+PSAwOyAtLWkpIHtcbiAgICAgIGNvbnN0IGNvZGUgPSBwYXRoLmNoYXJDb2RlQXQoaSk7XG4gICAgICBpZiAoY29kZSA9PT0gQ0hBUl9GT1JXQVJEX1NMQVNIKSB7XG4gICAgICAgIC8vIElmIHdlIHJlYWNoZWQgYSBwYXRoIHNlcGFyYXRvciB0aGF0IHdhcyBub3QgcGFydCBvZiBhIHNldCBvZiBwYXRoXG4gICAgICAgIC8vIHNlcGFyYXRvcnMgYXQgdGhlIGVuZCBvZiB0aGUgc3RyaW5nLCBzdG9wIG5vd1xuICAgICAgICBpZiAoIW1hdGNoZWRTbGFzaCkge1xuICAgICAgICAgIHN0YXJ0ID0gaSArIDE7XG4gICAgICAgICAgYnJlYWs7XG4gICAgICAgIH1cbiAgICAgIH0gZWxzZSB7XG4gICAgICAgIGlmIChmaXJzdE5vblNsYXNoRW5kID09PSAtMSkge1xuICAgICAgICAgIC8vIFdlIHNhdyB0aGUgZmlyc3Qgbm9uLXBhdGggc2VwYXJhdG9yLCByZW1lbWJlciB0aGlzIGluZGV4IGluIGNhc2VcbiAgICAgICAgICAvLyB3ZSBuZWVkIGl0IGlmIHRoZSBleHRlbnNpb24gZW5kcyB1cCBub3QgbWF0Y2hpbmdcbiAgICAgICAgICBtYXRjaGVkU2xhc2ggPSBmYWxzZTtcbiAgICAgICAgICBmaXJzdE5vblNsYXNoRW5kID0gaSArIDE7XG4gICAgICAgIH1cbiAgICAgICAgaWYgKGV4dElkeCA+PSAwKSB7XG4gICAgICAgICAgLy8gVHJ5IHRvIG1hdGNoIHRoZSBleHBsaWNpdCBleHRlbnNpb25cbiAgICAgICAgICBpZiAoY29kZSA9PT0gZXh0LmNoYXJDb2RlQXQoZXh0SWR4KSkge1xuICAgICAgICAgICAgaWYgKC0tZXh0SWR4ID09PSAtMSkge1xuICAgICAgICAgICAgICAvLyBXZSBtYXRjaGVkIHRoZSBleHRlbnNpb24sIHNvIG1hcmsgdGhpcyBhcyB0aGUgZW5kIG9mIG91ciBwYXRoXG4gICAgICAgICAgICAgIC8vIGNvbXBvbmVudFxuICAgICAgICAgICAgICBlbmQgPSBpO1xuICAgICAgICAgICAgfVxuICAgICAgICAgIH0gZWxzZSB7XG4gICAgICAgICAgICAvLyBFeHRlbnNpb24gZG9lcyBub3QgbWF0Y2gsIHNvIG91ciByZXN1bHQgaXMgdGhlIGVudGlyZSBwYXRoXG4gICAgICAgICAgICAvLyBjb21wb25lbnRcbiAgICAgICAgICAgIGV4dElkeCA9IC0xO1xuICAgICAgICAgICAgZW5kID0gZmlyc3ROb25TbGFzaEVuZDtcbiAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICAgIH1cbiAgICB9XG5cbiAgICBpZiAoc3RhcnQgPT09IGVuZCkgZW5kID0gZmlyc3ROb25TbGFzaEVuZDtcbiAgICBlbHNlIGlmIChlbmQgPT09IC0xKSBlbmQgPSBwYXRoLmxlbmd0aDtcbiAgICByZXR1cm4gcGF0aC5zbGljZShzdGFydCwgZW5kKTtcbiAgfSBlbHNlIHtcbiAgICBmb3IgKGkgPSBwYXRoLmxlbmd0aCAtIDE7IGkgPj0gMDsgLS1pKSB7XG4gICAgICBpZiAocGF0aC5jaGFyQ29kZUF0KGkpID09PSBDSEFSX0ZPUldBUkRfU0xBU0gpIHtcbiAgICAgICAgLy8gSWYgd2UgcmVhY2hlZCBhIHBhdGggc2VwYXJhdG9yIHRoYXQgd2FzIG5vdCBwYXJ0IG9mIGEgc2V0IG9mIHBhdGhcbiAgICAgICAgLy8gc2VwYXJhdG9ycyBhdCB0aGUgZW5kIG9mIHRoZSBzdHJpbmcsIHN0b3Agbm93XG4gICAgICAgIGlmICghbWF0Y2hlZFNsYXNoKSB7XG4gICAgICAgICAgc3RhcnQgPSBpICsgMTtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgICAgfSBlbHNlIGlmIChlbmQgPT09IC0xKSB7XG4gICAgICAgIC8vIFdlIHNhdyB0aGUgZmlyc3Qgbm9uLXBhdGggc2VwYXJhdG9yLCBtYXJrIHRoaXMgYXMgdGhlIGVuZCBvZiBvdXJcbiAgICAgICAgLy8gcGF0aCBjb21wb25lbnRcbiAgICAgICAgbWF0Y2hlZFNsYXNoID0gZmFsc2U7XG4gICAgICAgIGVuZCA9IGkgKyAxO1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChlbmQgPT09IC0xKSByZXR1cm4gXCJcIjtcbiAgICByZXR1cm4gcGF0aC5zbGljZShzdGFydCwgZW5kKTtcbiAgfVxufVxuXG4vKipcbiAqIFJldHVybiB0aGUgZXh0ZW5zaW9uIG9mIHRoZSBgcGF0aGAuXG4gKiBAcGFyYW0gcGF0aCB3aXRoIGV4dGVuc2lvblxuICovXG5leHBvcnQgZnVuY3Rpb24gZXh0bmFtZShwYXRoOiBzdHJpbmcpOiBzdHJpbmcge1xuICBhc3NlcnRQYXRoKHBhdGgpO1xuICBsZXQgc3RhcnREb3QgPSAtMTtcbiAgbGV0IHN0YXJ0UGFydCA9IDA7XG4gIGxldCBlbmQgPSAtMTtcbiAgbGV0IG1hdGNoZWRTbGFzaCA9IHRydWU7XG4gIC8vIFRyYWNrIHRoZSBzdGF0ZSBvZiBjaGFyYWN0ZXJzIChpZiBhbnkpIHdlIHNlZSBiZWZvcmUgb3VyIGZpcnN0IGRvdCBhbmRcbiAgLy8gYWZ0ZXIgYW55IHBhdGggc2VwYXJhdG9yIHdlIGZpbmRcbiAgbGV0IHByZURvdFN0YXRlID0gMDtcbiAgZm9yIChsZXQgaSA9IHBhdGgubGVuZ3RoIC0gMTsgaSA+PSAwOyAtLWkpIHtcbiAgICBjb25zdCBjb2RlID0gcGF0aC5jaGFyQ29kZUF0KGkpO1xuICAgIGlmIChjb2RlID09PSBDSEFSX0ZPUldBUkRfU0xBU0gpIHtcbiAgICAgIC8vIElmIHdlIHJlYWNoZWQgYSBwYXRoIHNlcGFyYXRvciB0aGF0IHdhcyBub3QgcGFydCBvZiBhIHNldCBvZiBwYXRoXG4gICAgICAvLyBzZXBhcmF0b3JzIGF0IHRoZSBlbmQgb2YgdGhlIHN0cmluZywgc3RvcCBub3dcbiAgICAgIGlmICghbWF0Y2hlZFNsYXNoKSB7XG4gICAgICAgIHN0YXJ0UGFydCA9IGkgKyAxO1xuICAgICAgICBicmVhaztcbiAgICAgIH1cbiAgICAgIGNvbnRpbnVlO1xuICAgIH1cbiAgICBpZiAoZW5kID09PSAtMSkge1xuICAgICAgLy8gV2Ugc2F3IHRoZSBmaXJzdCBub24tcGF0aCBzZXBhcmF0b3IsIG1hcmsgdGhpcyBhcyB0aGUgZW5kIG9mIG91clxuICAgICAgLy8gZXh0ZW5zaW9uXG4gICAgICBtYXRjaGVkU2xhc2ggPSBmYWxzZTtcbiAgICAgIGVuZCA9IGkgKyAxO1xuICAgIH1cbiAgICBpZiAoY29kZSA9PT0gQ0hBUl9ET1QpIHtcbiAgICAgIC8vIElmIHRoaXMgaXMgb3VyIGZpcnN0IGRvdCwgbWFyayBpdCBhcyB0aGUgc3RhcnQgb2Ygb3VyIGV4dGVuc2lvblxuICAgICAgaWYgKHN0YXJ0RG90ID09PSAtMSkgc3RhcnREb3QgPSBpO1xuICAgICAgZWxzZSBpZiAocHJlRG90U3RhdGUgIT09IDEpIHByZURvdFN0YXRlID0gMTtcbiAgICB9IGVsc2UgaWYgKHN0YXJ0RG90ICE9PSAtMSkge1xuICAgICAgLy8gV2Ugc2F3IGEgbm9uLWRvdCBhbmQgbm9uLXBhdGggc2VwYXJhdG9yIGJlZm9yZSBvdXIgZG90LCBzbyB3ZSBzaG91bGRcbiAgICAgIC8vIGhhdmUgYSBnb29kIGNoYW5jZSBhdCBoYXZpbmcgYSBub24tZW1wdHkgZXh0ZW5zaW9uXG4gICAgICBwcmVEb3RTdGF0ZSA9IC0xO1xuICAgIH1cbiAgfVxuXG4gIGlmIChcbiAgICBzdGFydERvdCA9PT0gLTEgfHxcbiAgICBlbmQgPT09IC0xIHx8XG4gICAgLy8gV2Ugc2F3IGEgbm9uLWRvdCBjaGFyYWN0ZXIgaW1tZWRpYXRlbHkgYmVmb3JlIHRoZSBkb3RcbiAgICBwcmVEb3RTdGF0ZSA9PT0gMCB8fFxuICAgIC8vIFRoZSAocmlnaHQtbW9zdCkgdHJpbW1lZCBwYXRoIGNvbXBvbmVudCBpcyBleGFjdGx5ICcuLidcbiAgICAocHJlRG90U3RhdGUgPT09IDEgJiYgc3RhcnREb3QgPT09IGVuZCAtIDEgJiYgc3RhcnREb3QgPT09IHN0YXJ0UGFydCArIDEpXG4gICkge1xuICAgIHJldHVybiBcIlwiO1xuICB9XG4gIHJldHVybiBwYXRoLnNsaWNlKHN0YXJ0RG90LCBlbmQpO1xufVxuXG4vKipcbiAqIEdlbmVyYXRlIGEgcGF0aCBmcm9tIGBGb3JtYXRJbnB1dFBhdGhPYmplY3RgIG9iamVjdC5cbiAqIEBwYXJhbSBwYXRoT2JqZWN0IHdpdGggcGF0aFxuICovXG5leHBvcnQgZnVuY3Rpb24gZm9ybWF0KHBhdGhPYmplY3Q6IEZvcm1hdElucHV0UGF0aE9iamVjdCk6IHN0cmluZyB7XG4gIGlmIChwYXRoT2JqZWN0ID09PSBudWxsIHx8IHR5cGVvZiBwYXRoT2JqZWN0ICE9PSBcIm9iamVjdFwiKSB7XG4gICAgdGhyb3cgbmV3IFR5cGVFcnJvcihcbiAgICAgIGBUaGUgXCJwYXRoT2JqZWN0XCIgYXJndW1lbnQgbXVzdCBiZSBvZiB0eXBlIE9iamVjdC4gUmVjZWl2ZWQgdHlwZSAke3R5cGVvZiBwYXRoT2JqZWN0fWAsXG4gICAgKTtcbiAgfVxuICByZXR1cm4gX2Zvcm1hdChcIi9cIiwgcGF0aE9iamVjdCk7XG59XG5cbi8qKlxuICogUmV0dXJuIGEgYFBhcnNlZFBhdGhgIG9iamVjdCBvZiB0aGUgYHBhdGhgLlxuICogQHBhcmFtIHBhdGggdG8gcHJvY2Vzc1xuICovXG5leHBvcnQgZnVuY3Rpb24gcGFyc2UocGF0aDogc3RyaW5nKTogUGFyc2VkUGF0aCB7XG4gIGFzc2VydFBhdGgocGF0aCk7XG5cbiAgY29uc3QgcmV0OiBQYXJzZWRQYXRoID0geyByb290OiBcIlwiLCBkaXI6IFwiXCIsIGJhc2U6IFwiXCIsIGV4dDogXCJcIiwgbmFtZTogXCJcIiB9O1xuICBpZiAocGF0aC5sZW5ndGggPT09IDApIHJldHVybiByZXQ7XG4gIGNvbnN0IGlzQWJzb2x1dGUgPSBwYXRoLmNoYXJDb2RlQXQoMCkgPT09IENIQVJfRk9SV0FSRF9TTEFTSDtcbiAgbGV0IHN0YXJ0OiBudW1iZXI7XG4gIGlmIChpc0Fic29sdXRlKSB7XG4gICAgcmV0LnJvb3QgPSBcIi9cIjtcbiAgICBzdGFydCA9IDE7XG4gIH0gZWxzZSB7XG4gICAgc3RhcnQgPSAwO1xuICB9XG4gIGxldCBzdGFydERvdCA9IC0xO1xuICBsZXQgc3RhcnRQYXJ0ID0gMDtcbiAgbGV0IGVuZCA9IC0xO1xuICBsZXQgbWF0Y2hlZFNsYXNoID0gdHJ1ZTtcbiAgbGV0IGkgPSBwYXRoLmxlbmd0aCAtIDE7XG5cbiAgLy8gVHJhY2sgdGhlIHN0YXRlIG9mIGNoYXJhY3RlcnMgKGlmIGFueSkgd2Ugc2VlIGJlZm9yZSBvdXIgZmlyc3QgZG90IGFuZFxuICAvLyBhZnRlciBhbnkgcGF0aCBzZXBhcmF0b3Igd2UgZmluZFxuICBsZXQgcHJlRG90U3RhdGUgPSAwO1xuXG4gIC8vIEdldCBub24tZGlyIGluZm9cbiAgZm9yICg7IGkgPj0gc3RhcnQ7IC0taSkge1xuICAgIGNvbnN0IGNvZGUgPSBwYXRoLmNoYXJDb2RlQXQoaSk7XG4gICAgaWYgKGNvZGUgPT09IENIQVJfRk9SV0FSRF9TTEFTSCkge1xuICAgICAgLy8gSWYgd2UgcmVhY2hlZCBhIHBhdGggc2VwYXJhdG9yIHRoYXQgd2FzIG5vdCBwYXJ0IG9mIGEgc2V0IG9mIHBhdGhcbiAgICAgIC8vIHNlcGFyYXRvcnMgYXQgdGhlIGVuZCBvZiB0aGUgc3RyaW5nLCBzdG9wIG5vd1xuICAgICAgaWYgKCFtYXRjaGVkU2xhc2gpIHtcbiAgICAgICAgc3RhcnRQYXJ0ID0gaSArIDE7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfVxuICAgICAgY29udGludWU7XG4gICAgfVxuICAgIGlmIChlbmQgPT09IC0xKSB7XG4gICAgICAvLyBXZSBzYXcgdGhlIGZpcnN0IG5vbi1wYXRoIHNlcGFyYXRvciwgbWFyayB0aGlzIGFzIHRoZSBlbmQgb2Ygb3VyXG4gICAgICAvLyBleHRlbnNpb25cbiAgICAgIG1hdGNoZWRTbGFzaCA9IGZhbHNlO1xuICAgICAgZW5kID0gaSArIDE7XG4gICAgfVxuICAgIGlmIChjb2RlID09PSBDSEFSX0RPVCkge1xuICAgICAgLy8gSWYgdGhpcyBpcyBvdXIgZmlyc3QgZG90LCBtYXJrIGl0IGFzIHRoZSBzdGFydCBvZiBvdXIgZXh0ZW5zaW9uXG4gICAgICBpZiAoc3RhcnREb3QgPT09IC0xKSBzdGFydERvdCA9IGk7XG4gICAgICBlbHNlIGlmIChwcmVEb3RTdGF0ZSAhPT0gMSkgcHJlRG90U3RhdGUgPSAxO1xuICAgIH0gZWxzZSBpZiAoc3RhcnREb3QgIT09IC0xKSB7XG4gICAgICAvLyBXZSBzYXcgYSBub24tZG90IGFuZCBub24tcGF0aCBzZXBhcmF0b3IgYmVmb3JlIG91ciBkb3QsIHNvIHdlIHNob3VsZFxuICAgICAgLy8gaGF2ZSBhIGdvb2QgY2hhbmNlIGF0IGhhdmluZyBhIG5vbi1lbXB0eSBleHRlbnNpb25cbiAgICAgIHByZURvdFN0YXRlID0gLTE7XG4gICAgfVxuICB9XG5cbiAgaWYgKFxuICAgIHN0YXJ0RG90ID09PSAtMSB8fFxuICAgIGVuZCA9PT0gLTEgfHxcbiAgICAvLyBXZSBzYXcgYSBub24tZG90IGNoYXJhY3RlciBpbW1lZGlhdGVseSBiZWZvcmUgdGhlIGRvdFxuICAgIHByZURvdFN0YXRlID09PSAwIHx8XG4gICAgLy8gVGhlIChyaWdodC1tb3N0KSB0cmltbWVkIHBhdGggY29tcG9uZW50IGlzIGV4YWN0bHkgJy4uJ1xuICAgIChwcmVEb3RTdGF0ZSA9PT0gMSAmJiBzdGFydERvdCA9PT0gZW5kIC0gMSAmJiBzdGFydERvdCA9PT0gc3RhcnRQYXJ0ICsgMSlcbiAgKSB7XG4gICAgaWYgKGVuZCAhPT0gLTEpIHtcbiAgICAgIGlmIChzdGFydFBhcnQgPT09IDAgJiYgaXNBYnNvbHV0ZSkge1xuICAgICAgICByZXQuYmFzZSA9IHJldC5uYW1lID0gcGF0aC5zbGljZSgxLCBlbmQpO1xuICAgICAgfSBlbHNlIHtcbiAgICAgICAgcmV0LmJhc2UgPSByZXQubmFtZSA9IHBhdGguc2xpY2Uoc3RhcnRQYXJ0LCBlbmQpO1xuICAgICAgfVxuICAgIH1cbiAgfSBlbHNlIHtcbiAgICBpZiAoc3RhcnRQYXJ0ID09PSAwICYmIGlzQWJzb2x1dGUpIHtcbiAgICAgIHJldC5uYW1lID0gcGF0aC5zbGljZSgxLCBzdGFydERvdCk7XG4gICAgICByZXQuYmFzZSA9IHBhdGguc2xpY2UoMSwgZW5kKTtcbiAgICB9IGVsc2Uge1xuICAgICAgcmV0Lm5hbWUgPSBwYXRoLnNsaWNlKHN0YXJ0UGFydCwgc3RhcnREb3QpO1xuICAgICAgcmV0LmJhc2UgPSBwYXRoLnNsaWNlKHN0YXJ0UGFydCwgZW5kKTtcbiAgICB9XG4gICAgcmV0LmV4dCA9IHBhdGguc2xpY2Uoc3RhcnREb3QsIGVuZCk7XG4gIH1cblxuICBpZiAoc3RhcnRQYXJ0ID4gMCkgcmV0LmRpciA9IHBhdGguc2xpY2UoMCwgc3RhcnRQYXJ0IC0gMSk7XG4gIGVsc2UgaWYgKGlzQWJzb2x1dGUpIHJldC5kaXIgPSBcIi9cIjtcblxuICByZXR1cm4gcmV0O1xufVxuXG4vKipcbiAqIENvbnZlcnRzIGEgZmlsZSBVUkwgdG8gYSBwYXRoIHN0cmluZy5cbiAqXG4gKiBgYGB0c1xuICogICAgICBpbXBvcnQgeyBmcm9tRmlsZVVybCB9IGZyb20gXCIuL3Bvc2l4LnRzXCI7XG4gKiAgICAgIGZyb21GaWxlVXJsKFwiZmlsZTovLy9ob21lL2Zvb1wiKTsgLy8gXCIvaG9tZS9mb29cIlxuICogYGBgXG4gKiBAcGFyYW0gdXJsIG9mIGEgZmlsZSBVUkxcbiAqL1xuZXhwb3J0IGZ1bmN0aW9uIGZyb21GaWxlVXJsKHVybDogc3RyaW5nIHwgVVJMKTogc3RyaW5nIHtcbiAgdXJsID0gdXJsIGluc3RhbmNlb2YgVVJMID8gdXJsIDogbmV3IFVSTCh1cmwpO1xuICBpZiAodXJsLnByb3RvY29sICE9IFwiZmlsZTpcIikge1xuICAgIHRocm93IG5ldyBUeXBlRXJyb3IoXCJNdXN0IGJlIGEgZmlsZSBVUkwuXCIpO1xuICB9XG4gIHJldHVybiBkZWNvZGVVUklDb21wb25lbnQoXG4gICAgdXJsLnBhdGhuYW1lLnJlcGxhY2UoLyUoPyFbMC05QS1GYS1mXXsyfSkvZywgXCIlMjVcIiksXG4gICk7XG59XG5cbi8qKlxuICogQ29udmVydHMgYSBwYXRoIHN0cmluZyB0byBhIGZpbGUgVVJMLlxuICpcbiAqIGBgYHRzXG4gKiAgICAgIGltcG9ydCB7IHRvRmlsZVVybCB9IGZyb20gXCIuL3Bvc2l4LnRzXCI7XG4gKiAgICAgIHRvRmlsZVVybChcIi9ob21lL2Zvb1wiKTsgLy8gbmV3IFVSTChcImZpbGU6Ly8vaG9tZS9mb29cIilcbiAqIGBgYFxuICogQHBhcmFtIHBhdGggdG8gY29udmVydCB0byBmaWxlIFVSTFxuICovXG5leHBvcnQgZnVuY3Rpb24gdG9GaWxlVXJsKHBhdGg6IHN0cmluZyk6IFVSTCB7XG4gIGlmICghaXNBYnNvbHV0ZShwYXRoKSkge1xuICAgIHRocm93IG5ldyBUeXBlRXJyb3IoXCJNdXN0IGJlIGFuIGFic29sdXRlIHBhdGguXCIpO1xuICB9XG4gIGNvbnN0IHVybCA9IG5ldyBVUkwoXCJmaWxlOi8vL1wiKTtcbiAgdXJsLnBhdGhuYW1lID0gZW5jb2RlV2hpdGVzcGFjZShcbiAgICBwYXRoLnJlcGxhY2UoLyUvZywgXCIlMjVcIikucmVwbGFjZSgvXFxcXC9nLCBcIiU1Q1wiKSxcbiAgKTtcbiAgcmV0dXJuIHVybDtcbn1cbiJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQSwwRUFBMEU7QUFDMUUsaURBQWlEO0FBQ2pELDZEQUE2RDtBQUM3RCxxQ0FBcUM7QUFHckMsU0FBUyxRQUFRLEVBQUUsa0JBQWtCLFFBQVEsa0JBQWtCO0FBRS9ELFNBQ0UsT0FBTyxFQUNQLFVBQVUsRUFDVixnQkFBZ0IsRUFDaEIsb0JBQW9CLEVBQ3BCLGVBQWUsUUFDVixhQUFhO0FBRXBCLE9BQU8sTUFBTSxNQUFNLElBQUk7QUFDdkIsT0FBTyxNQUFNLFlBQVksSUFBSTtBQUU3QiwrQkFBK0I7QUFDL0I7OztDQUdDLEdBQ0QsT0FBTyxTQUFTLFFBQVEsR0FBRyxZQUFzQjtFQUMvQyxJQUFJLGVBQWU7RUFDbkIsSUFBSSxtQkFBbUI7RUFFdkIsSUFBSyxJQUFJLElBQUksYUFBYSxNQUFNLEdBQUcsR0FBRyxLQUFLLENBQUMsS0FBSyxDQUFDLGtCQUFrQixJQUFLO0lBQ3ZFLElBQUk7SUFFSixJQUFJLEtBQUssR0FBRyxPQUFPLFlBQVksQ0FBQyxFQUFFO1NBQzdCO01BQ0gsbUNBQW1DO01BQ25DLE1BQU0sRUFBRSxJQUFJLEVBQUUsR0FBRztNQUNqQixJQUFJLE9BQU8sTUFBTSxRQUFRLFlBQVk7UUFDbkMsTUFBTSxJQUFJLFVBQVU7TUFDdEI7TUFD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
// denoCacheMetadata=7360162745429795103,18043264681729543696