// Copyright 2018-2022 the Deno authors. All rights reserved. MIT license.
// Documentation and interface for walk were adapted from Go
// https://golang.org/pkg/path/filepath/#Walk
// Copyright 2009 The Go Authors. All rights reserved. BSD license.
import { assert } from "../_util/assert.ts";
import { basename, join, normalize } from "../path/mod.ts";
/** Create WalkEntry for the `path` synchronously */ export function _createWalkEntrySync(path) {
  path = normalize(path);
  const name = basename(path);
  const info = Deno.statSync(path);
  return {
    path,
    name,
    isFile: info.isFile,
    isDirectory: info.isDirectory,
    isSymlink: info.isSymlink
  };
}
/** Create WalkEntry for the `path` asynchronously */ export async function _createWalkEntry(path) {
  path = normalize(path);
  const name = basename(path);
  const info = await Deno.stat(path);
  return {
    path,
    name,
    isFile: info.isFile,
    isDirectory: info.isDirectory,
    isSymlink: info.isSymlink
  };
}
function include(path, exts, match, skip) {
  if (exts && !exts.some((ext)=>path.endsWith(ext))) {
    return false;
  }
  if (match && !match.some((pattern)=>!!path.match(pattern))) {
    return false;
  }
  if (skip && skip.some((pattern)=>!!path.match(pattern))) {
    return false;
  }
  return true;
}
function wrapErrorWithRootPath(err, root) {
  if (err instanceof Error && "root" in err) return err;
  const e = new Error();
  e.root = root;
  e.message = err instanceof Error ? `${err.message} for path "${root}"` : `[non-error thrown] for path "${root}"`;
  e.stack = err instanceof Error ? err.stack : undefined;
  e.cause = err instanceof Error ? err.cause : undefined;
  return e;
}
/** Walks the file tree rooted at root, yielding each file or directory in the
 * tree filtered according to the given options. The files are walked in lexical
 * order, which makes the output deterministic but means that for very large
 * directories walk() can be inefficient.
 *
 * Options:
 * - maxDepth?: number = Infinity;
 * - includeFiles?: boolean = true;
 * - includeDirs?: boolean = true;
 * - followSymlinks?: boolean = false;
 * - exts?: string[];
 * - match?: RegExp[];
 * - skip?: RegExp[];
 *
 * ```ts
 *       import { walk } from "./walk.ts";
 *       import { assert } from "../testing/asserts.ts";
 *
 *       for await (const entry of walk(".")) {
 *         console.log(entry.path);
 *         assert(entry.isFile);
 *       }
 * ```
 */ export async function* walk(root, { maxDepth = Infinity, includeFiles = true, includeDirs = true, followSymlinks = false, exts = undefined, match = undefined, skip = undefined } = {}) {
  if (maxDepth < 0) {
    return;
  }
  if (includeDirs && include(root, exts, match, skip)) {
    yield await _createWalkEntry(root);
  }
  if (maxDepth < 1 || !include(root, undefined, undefined, skip)) {
    return;
  }
  try {
    for await (const entry of Deno.readDir(root)){
      assert(entry.name != null);
      let path = join(root, entry.name);
      let { isSymlink, isDirectory } = entry;
      if (isSymlink) {
        if (!followSymlinks) continue;
        path = await Deno.realPath(path);
        // Caveat emptor: don't assume |path| is not a symlink. realpath()
        // resolves symlinks but another process can replace the file system
        // entity with a different type of entity before we call lstat().
        ({ isSymlink, isDirectory } = await Deno.lstat(path));
      }
      if (isSymlink || isDirectory) {
        yield* walk(path, {
          maxDepth: maxDepth - 1,
          includeFiles,
          includeDirs,
          followSymlinks,
          exts,
          match,
          skip
        });
      } else if (includeFiles && include(path, exts, match, skip)) {
        yield {
          path,
          ...entry
        };
      }
    }
  } catch (err) {
    throw wrapErrorWithRootPath(err, normalize(root));
  }
}
/** Same as walk() but uses synchronous ops */ export function* walkSync(root, { maxDepth = Infinity, includeFiles = true, includeDirs = true, followSymlinks = false, exts = undefined, match = undefined, skip = undefined } = {}) {
  if (maxDepth < 0) {
    return;
  }
  if (includeDirs && include(root, exts, match, skip)) {
    yield _createWalkEntrySync(root);
  }
  if (maxDepth < 1 || !include(root, undefined, undefined, skip)) {
    return;
  }
  let entries;
  try {
    entries = Deno.readDirSync(root);
  } catch (err) {
    throw wrapErrorWithRootPath(err, normalize(root));
  }
  for (const entry of entries){
    assert(entry.name != null);
    let path = join(root, entry.name);
    let { isSymlink, isDirectory } = entry;
    if (isSymlink) {
      if (!followSymlinks) continue;
      path = Deno.realPathSync(path);
      // Caveat emptor: don't assume |path| is not a symlink. realpath()
      // resolves symlinks but another process can replace the file system
      // entity with a different type of entity before we call lstat().
      ({ isSymlink, isDirectory } = Deno.lstatSync(path));
    }
    if (isSymlink || isDirectory) {
      yield* walkSync(path, {
        maxDepth: maxDepth - 1,
        includeFiles,
        includeDirs,
        followSymlinks,
        exts,
        match,
        skip
      });
    } else if (includeFiles && include(path, exts, match, skip)) {
      yield {
        path,
        ...entry
      };
    }
  }
}
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjEzMy4wL2ZzL3dhbGsudHMiXSwic291cmNlc0NvbnRlbnQiOlsiLy8gQ29weXJpZ2h0IDIwMTgtMjAyMiB0aGUgRGVubyBhdXRob3JzLiBBbGwgcmlnaHRzIHJlc2VydmVkLiBNSVQgbGljZW5zZS5cbi8vIERvY3VtZW50YXRpb24gYW5kIGludGVyZmFjZSBmb3Igd2FsayB3ZXJlIGFkYXB0ZWQgZnJvbSBHb1xuLy8gaHR0cHM6Ly9nb2xhbmcub3JnL3BrZy9wYXRoL2ZpbGVwYXRoLyNXYWxrXG4vLyBDb3B5cmlnaHQgMjAwOSBUaGUgR28gQXV0aG9ycy4gQWxsIHJpZ2h0cyByZXNlcnZlZC4gQlNEIGxpY2Vuc2UuXG5pbXBvcnQgeyBhc3NlcnQgfSBmcm9tIFwiLi4vX3V0aWwvYXNzZXJ0LnRzXCI7XG5pbXBvcnQgeyBiYXNlbmFtZSwgam9pbiwgbm9ybWFsaXplIH0gZnJvbSBcIi4uL3BhdGgvbW9kLnRzXCI7XG5cbi8qKiBDcmVhdGUgV2Fsa0VudHJ5IGZvciB0aGUgYHBhdGhgIHN5bmNocm9ub3VzbHkgKi9cbmV4cG9ydCBmdW5jdGlvbiBfY3JlYXRlV2Fsa0VudHJ5U3luYyhwYXRoOiBzdHJpbmcpOiBXYWxrRW50cnkge1xuICBwYXRoID0gbm9ybWFsaXplKHBhdGgpO1xuICBjb25zdCBuYW1lID0gYmFzZW5hbWUocGF0aCk7XG4gIGNvbnN0IGluZm8gPSBEZW5vLnN0YXRTeW5jKHBhdGgpO1xuICByZXR1cm4ge1xuICAgIHBhdGgsXG4gICAgbmFtZSxcbiAgICBpc0ZpbGU6IGluZm8uaXNGaWxlLFxuICAgIGlzRGlyZWN0b3J5OiBpbmZvLmlzRGlyZWN0b3J5LFxuICAgIGlzU3ltbGluazogaW5mby5pc1N5bWxpbmssXG4gIH07XG59XG5cbi8qKiBDcmVhdGUgV2Fsa0VudHJ5IGZvciB0aGUgYHBhdGhgIGFzeW5jaHJvbm91c2x5ICovXG5leHBvcnQgYXN5bmMgZnVuY3Rpb24gX2NyZWF0ZVdhbGtFbnRyeShwYXRoOiBzdHJpbmcpOiBQcm9taXNlPFdhbGtFbnRyeT4ge1xuICBwYXRoID0gbm9ybWFsaXplKHBhdGgpO1xuICBjb25zdCBuYW1lID0gYmFzZW5hbWUocGF0aCk7XG4gIGNvbnN0IGluZm8gPSBhd2FpdCBEZW5vLnN0YXQocGF0aCk7XG4gIHJldHVybiB7XG4gICAgcGF0aCxcbiAgICBuYW1lLFxuICAgIGlzRmlsZTogaW5mby5pc0ZpbGUsXG4gICAgaXNEaXJlY3Rvcnk6IGluZm8uaXNEaXJlY3RvcnksXG4gICAgaXNTeW1saW5rOiBpbmZvLmlzU3ltbGluayxcbiAgfTtcbn1cblxuZXhwb3J0IGludGVyZmFjZSBXYWxrT3B0aW9ucyB7XG4gIG1heERlcHRoPzogbnVtYmVyO1xuICBpbmNsdWRlRmlsZXM/OiBib29sZWFuO1xuICBpbmNsdWRlRGlycz86IGJvb2xlYW47XG4gIGZvbGxvd1N5bWxpbmtzPzogYm9vbGVhbjtcbiAgZXh0cz86IHN0cmluZ1tdO1xuICBtYXRjaD86IFJlZ0V4cFtdO1xuICBza2lwPzogUmVnRXhwW107XG59XG5cbmZ1bmN0aW9uIGluY2x1ZGUoXG4gIHBhdGg6IHN0cmluZyxcbiAgZXh0cz86IHN0cmluZ1tdLFxuICBtYXRjaD86IFJlZ0V4cFtdLFxuICBza2lwPzogUmVnRXhwW10sXG4pOiBib29sZWFuIHtcbiAgaWYgKGV4dHMgJiYgIWV4dHMuc29tZSgoZXh0KTogYm9vbGVhbiA9PiBwYXRoLmVuZHNXaXRoKGV4dCkpKSB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG4gIGlmIChtYXRjaCAmJiAhbWF0Y2guc29tZSgocGF0dGVybik6IGJvb2xlYW4gPT4gISFwYXRoLm1hdGNoKHBhdHRlcm4pKSkge1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuICBpZiAoc2tpcCAmJiBza2lwLnNvbWUoKHBhdHRlcm4pOiBib29sZWFuID0+ICEhcGF0aC5tYXRjaChwYXR0ZXJuKSkpIHtcbiAgICByZXR1cm4gZmFsc2U7XG4gIH1cbiAgcmV0dXJuIHRydWU7XG59XG5cbmZ1bmN0aW9uIHdyYXBFcnJvcldpdGhSb290UGF0aChlcnI6IHVua25vd24sIHJvb3Q6IHN0cmluZykge1xuICBpZiAoZXJyIGluc3RhbmNlb2YgRXJyb3IgJiYgXCJyb290XCIgaW4gZXJyKSByZXR1cm4gZXJyO1xuICBjb25zdCBlID0gbmV3IEVycm9yKCkgYXMgRXJyb3IgJiB7IHJvb3Q6IHN0cmluZyB9O1xuICBlLnJvb3QgPSByb290O1xuICBlLm1lc3NhZ2UgPSBlcnIgaW5zdGFuY2VvZiBFcnJvclxuICAgID8gYCR7ZXJyLm1lc3NhZ2V9IGZvciBwYXRoIFwiJHtyb290fVwiYFxuICAgIDogYFtub24tZXJyb3IgdGhyb3duXSBmb3IgcGF0aCBcIiR7cm9vdH1cImA7XG4gIGUuc3RhY2sgPSBlcnIgaW5zdGFuY2VvZiBFcnJvciA/IGVyci5zdGFjayA6IHVuZGVmaW5lZDtcbiAgZS5jYXVzZSA9IGVyciBpbnN0YW5jZW9mIEVycm9yID8gZXJyLmNhdXNlIDogdW5kZWZpbmVkO1xuICByZXR1cm4gZTtcbn1cblxuZXhwb3J0IGludGVyZmFjZSBXYWxrRW50cnkgZXh0ZW5kcyBEZW5vLkRpckVudHJ5IHtcbiAgcGF0aDogc3RyaW5nO1xufVxuXG4vKiogV2Fsa3MgdGhlIGZpbGUgdHJlZSByb290ZWQgYXQgcm9vdCwgeWllbGRpbmcgZWFjaCBmaWxlIG9yIGRpcmVjdG9yeSBpbiB0aGVcbiAqIHRyZWUgZmlsdGVyZWQgYWNjb3JkaW5nIHRvIHRoZSBnaXZlbiBvcHRpb25zLiBUaGUgZmlsZXMgYXJlIHdhbGtlZCBpbiBsZXhpY2FsXG4gKiBvcmRlciwgd2hpY2ggbWFrZXMgdGhlIG91dHB1dCBkZXRlcm1pbmlzdGljIGJ1dCBtZWFucyB0aGF0IGZvciB2ZXJ5IGxhcmdlXG4gKiBkaXJlY3RvcmllcyB3YWxrKCkgY2FuIGJlIGluZWZmaWNpZW50LlxuICpcbiAqIE9wdGlvbnM6XG4gKiAtIG1heERlcHRoPzogbnVtYmVyID0gSW5maW5pdHk7XG4gKiAtIGluY2x1ZGVGaWxlcz86IGJvb2xlYW4gPSB0cnVlO1xuICogLSBpbmNsdWRlRGlycz86IGJvb2xlYW4gPSB0cnVlO1xuICogLSBmb2xsb3dTeW1saW5rcz86IGJvb2xlYW4gPSBmYWxzZTtcbiAqIC0gZXh0cz86IHN0cmluZ1tdO1xuICogLSBtYXRjaD86IFJlZ0V4cFtdO1xuICogLSBza2lwPzogUmVnRXhwW107XG4gKlxuICogYGBgdHNcbiAqICAgICAgIGltcG9ydCB7IHdhbGsgfSBmcm9tIFwiLi93YWxrLnRzXCI7XG4gKiAgICAgICBpbXBvcnQgeyBhc3NlcnQgfSBmcm9tIFwiLi4vdGVzdGluZy9hc3NlcnRzLnRzXCI7XG4gKlxuICogICAgICAgZm9yIGF3YWl0IChjb25zdCBlbnRyeSBvZiB3YWxrKFwiLlwiKSkge1xuICogICAgICAgICBjb25zb2xlLmxvZyhlbnRyeS5wYXRoKTtcbiAqICAgICAgICAgYXNzZXJ0KGVudHJ5LmlzRmlsZSk7XG4gKiAgICAgICB9XG4gKiBgYGBcbiAqL1xuZXhwb3J0IGFzeW5jIGZ1bmN0aW9uKiB3YWxrKFxuICByb290OiBzdHJpbmcsXG4gIHtcbiAgICBtYXhEZXB0aCA9IEluZmluaXR5LFxuICAgIGluY2x1ZGVGaWxlcyA9IHRydWUsXG4gICAgaW5jbHVkZURpcnMgPSB0cnVlLFxuICAgIGZvbGxvd1N5bWxpbmtzID0gZmFsc2UsXG4gICAgZXh0cyA9IHVuZGVmaW5lZCxcbiAgICBtYXRjaCA9IHVuZGVmaW5lZCxcbiAgICBza2lwID0gdW5kZWZpbmVkLFxuICB9OiBXYWxrT3B0aW9ucyA9IHt9LFxuKTogQXN5bmNJdGVyYWJsZUl0ZXJhdG9yPFdhbGtFbnRyeT4ge1xuICBpZiAobWF4RGVwdGggPCAwKSB7XG4gICAgcmV0dXJuO1xuICB9XG4gIGlmIChpbmNsdWRlRGlycyAmJiBpbmNsdWRlKHJvb3QsIGV4dHMsIG1hdGNoLCBza2lwKSkge1xuICAgIHlpZWxkIGF3YWl0IF9jcmVhdGVXYWxrRW50cnkocm9vdCk7XG4gIH1cbiAgaWYgKG1heERlcHRoIDwgMSB8fCAhaW5jbHVkZShyb290LCB1bmRlZmluZWQsIHVuZGVmaW5lZCwgc2tpcCkpIHtcbiAgICByZXR1cm47XG4gIH1cbiAgdHJ5IHtcbiAgICBmb3IgYXdhaXQgKGNvbnN0IGVudHJ5IG9mIERlbm8ucmVhZERpcihyb290KSkge1xuICAgICAgYXNzZXJ0KGVudHJ5Lm5hbWUgIT0gbnVsbCk7XG4gICAgICBsZXQgcGF0aCA9IGpvaW4ocm9vdCwgZW50cnkubmFtZSk7XG5cbiAgICAgIGxldCB7IGlzU3ltbGluaywgaXNEaXJlY3RvcnkgfSA9IGVudHJ5O1xuXG4gICAgICBpZiAoaXNTeW1saW5rKSB7XG4gICAgICAgIGlmICghZm9sbG93U3ltbGlua3MpIGNvbnRpbnVlO1xuICAgICAgICBwYXRoID0gYXdhaXQgRGVuby5yZWFsUGF0aChwYXRoKTtcbiAgICAgICAgLy8gQ2F2ZWF0IGVtcHRvcjogZG9uJ3QgYXNzdW1lIHxwYXRofCBpcyBub3QgYSBzeW1saW5rLiByZWFscGF0aCgpXG4gICAgICAgIC8vIHJlc29sdmVzIHN5bWxpbmtzIGJ1dCBhbm90aGVyIHByb2Nlc3MgY2FuIHJlcGxhY2UgdGhlIGZpbGUgc3lzdGVtXG4gICAgICAgIC8vIGVudGl0eSB3aXRoIGEgZGlmZmVyZW50IHR5cGUgb2YgZW50aXR5IGJlZm9yZSB3ZSBjYWxsIGxzdGF0KCkuXG4gICAgICAgICh7IGlzU3ltbGluaywgaXNEaXJlY3RvcnkgfSA9IGF3YWl0IERlbm8ubHN0YXQocGF0aCkpO1xuICAgICAgfVxuXG4gICAgICBpZiAoaXNTeW1saW5rIHx8IGlzRGlyZWN0b3J5KSB7XG4gICAgICAgIHlpZWxkKiB3YWxrKHBhdGgsIHtcbiAgICAgICAgICBtYXhEZXB0aDogbWF4RGVwdGggLSAxLFxuICAgICAgICAgIGluY2x1ZGVGaWxlcyxcbiAgICAgICAgICBpbmNsdWRlRGlycyxcbiAgICAgICAgICBmb2xsb3dTeW1saW5rcyxcbiAgICAgICAgICBleHRzLFxuICAgICAgICAgIG1hdGNoLFxuICAgICAgICAgIHNraXAsXG4gICAgICAgIH0pO1xuICAgICAgfSBlbHNlIGlmIChpbmNsdWRlRmlsZXMgJiYgaW5jbHVkZShwYXRoLCBleHRzLCBtYXRjaCwgc2tpcCkpIHtcbiAgICAgICAgeWllbGQgeyBwYXRoLCAuLi5lbnRyeSB9O1xuICAgICAgfVxuICAgIH1cbiAgfSBjYXRjaCAoZXJyKSB7XG4gICAgdGhyb3cgd3JhcEVycm9yV2l0aFJvb3RQYXRoKGVyciwgbm9ybWFsaXplKHJvb3QpKTtcbiAgfVxufVxuXG4vKiogU2FtZSBhcyB3YWxrKCkgYnV0IHVzZXMgc3luY2hyb25vdXMgb3BzICovXG5leHBvcnQgZnVuY3Rpb24qIHdhbGtTeW5jKFxuICByb290OiBzdHJpbmcsXG4gIHtcbiAgICBtYXhEZXB0aCA9IEluZmluaXR5LFxuICAgIGluY2x1ZGVGaWxlcyA9IHRydWUsXG4gICAgaW5jbHVkZURpcnMgPSB0cnVlLFxuICAgIGZvbGxvd1N5bWxpbmtzID0gZmFsc2UsXG4gICAgZXh0cyA9IHVuZGVmaW5lZCxcbiAgICBtYXRjaCA9IHVuZGVmaW5lZCxcbiAgICBza2lwID0gdW5kZWZpbmVkLFxuICB9OiBXYWxrT3B0aW9ucyA9IHt9LFxuKTogSXRlcmFibGVJdGVyYXRvcjxXYWxrRW50cnk+IHtcbiAgaWYgKG1heERlcHRoIDwgMCkge1xuICAgIHJldHVybjtcbiAgfVxuICBpZiAoaW5jbHVkZURpcnMgJiYgaW5jbHVkZShyb290LCBleHRzLCBtYXRjaCwgc2tpcCkpIHtcbiAgICB5aWVsZCBfY3JlYXRlV2Fsa0VudHJ5U3luYyhyb290KTtcbiAgfVxuICBpZiAobWF4RGVwdGggPCAxIHx8ICFpbmNsdWRlKHJvb3QsIHVuZGVmaW5lZCwgdW5kZWZpbmVkLCBza2lwKSkge1xuICAgIHJldHVybjtcbiAgfVxuICBsZXQgZW50cmllcztcbiAgdHJ5IHtcbiAgICBlbnRyaWVzID0gRGVuby5yZWFkRGlyU3luYyhyb290KTtcbiAgfSBjYXRjaCAoZXJyKSB7XG4gICAgdGhyb3cgd3JhcEVycm9yV2l0aFJvb3RQYXRoKGVyciwgbm9ybWFsaXplKHJvb3QpKTtcbiAgfVxuICBmb3IgKGNvbnN0IGVudHJ5IG9mIGVudHJpZXMpIHtcbiAgICBhc3NlcnQoZW50cnkubmFtZSAhPSBudWxsKTtcbiAgICBsZXQgcGF0aCA9IGpvaW4ocm9vdCwgZW50cnkubmFtZSk7XG5cbiAgICBsZXQgeyBpc1N5bWxpbmssIGlzRGlyZWN0b3J5IH0gPSBlbnRyeTtcblxuICAgIGlmIChpc1N5bWxpbmspIHtcbiAgICAgIGlmICghZm9sbG93U3ltbGlua3MpIGNvbnRpbnVlO1xuICAgICAgcGF0aCA9IERlbm8ucmVhbFBhdGhTeW5jKHBhdGgpO1xuICAgICAgLy8gQ2F2ZWF0IGVtcHRvcjogZG9uJ3QgYXNzdW1lIHxwYXRofCBpcyBub3QgYSBzeW1saW5rLiByZWFscGF0aCgpXG4gICAgICAvLyByZXNvbHZlcyBzeW1saW5rcyBidXQgYW5vdGhlciBwcm9jZXNzIGNhbiByZXBsYWNlIHRoZSBmaWxlIHN5c3RlbVxuICAgICAgLy8gZW50aXR5IHdpdGggYSBkaWZmZXJlbnQgdHlwZSBvZiBlbnRpdHkgYmVmb3JlIHdlIGNhbGwgbHN0YXQoKS5cbiAgICAgICh7IGlzU3ltbGluaywgaXNEaXJlY3RvcnkgfSA9IERlbm8ubHN0YXRTeW5jKHBhdGgpKTtcbiAgICB9XG5cbiAgICBpZiAoaXNTeW1saW5rIHx8IGlzRGlyZWN0b3J5KSB7XG4gICAgICB5aWVsZCogd2Fsa1N5bmMocGF0aCwge1xuICAgICAgICBtYXhEZXB0aDogbWF4RGVwdGggLSAxLFxuICAgICAgICBpbmNsdWRlRmlsZXMsXG4gICAgICAgIGluY2x1ZGVEaXJzLFxuICAgICAgICBmb2xsb3dTeW1saW5rcyxcbiAgICAgICAgZXh0cyxcbiAgICAgICAgbWF0Y2gsXG4gICAgICAgIHNraXAsXG4gICAgICB9KTtcbiAgICB9IGVsc2UgaWYgKGluY2x1ZGVGaWxlcyAmJiBpbmNsdWRlKHBhdGgsIGV4dHMsIG1hdGNoLCBza2lwKSkge1xuICAgICAgeWllbGQgeyBwYXRoLCAuLi5lbnRyeSB9O1xuICAgIH1cbiAgfVxufVxuIl0sIm5h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
// denoCacheMetadata=8945540673078767167,13194446033005371964