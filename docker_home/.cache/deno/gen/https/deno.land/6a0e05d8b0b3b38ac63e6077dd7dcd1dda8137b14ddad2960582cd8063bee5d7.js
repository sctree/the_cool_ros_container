// Copyright 2018-2024 the Deno authors. All rights reserved. MIT license.
// This module is browser compatible.
export { /**
   * @deprecated (will be removed in 0.215.0) Import from {@link https://deno.land/std/path/is_glob.ts} instead.
   *
   * Test whether the given string is a glob
   */ isGlob } from "./is_glob.ts";
export { /**
   * @deprecated (will be removed in 0.215.0) Import from {@link https://deno.land/std/path/glob_to_regexp.ts} instead.
   *
   * Convert a glob string to a regular expression.
   *
   * Tries to match bash glob expansion as closely as possible.
   *
   * Basic glob syntax:
   * - `*` - Matches everything without leaving the path segment.
   * - `?` - Matches any single character.
   * - `{foo,bar}` - Matches `foo` or `bar`.
   * - `[abcd]` - Matches `a`, `b`, `c` or `d`.
   * - `[a-d]` - Matches `a`, `b`, `c` or `d`.
   * - `[!abcd]` - Matches any single character besides `a`, `b`, `c` or `d`.
   * - `[[:<class>:]]` - Matches any character belonging to `<class>`.
   *     - `[[:alnum:]]` - Matches any digit or letter.
   *     - `[[:digit:]abc]` - Matches any digit, `a`, `b` or `c`.
   *     - See https://facelessuser.github.io/wcmatch/glob/#posix-character-classes
   *       for a complete list of supported character classes.
   * - `\` - Escapes the next character for an `os` other than `"windows"`.
   * - \` - Escapes the next character for `os` set to `"windows"`.
   * - `/` - Path separator.
   * - `\` - Additional path separator only for `os` set to `"windows"`.
   *
   * Extended syntax:
   * - Requires `{ extended: true }`.
   * - `?(foo|bar)` - Matches 0 or 1 instance of `{foo,bar}`.
   * - `@(foo|bar)` - Matches 1 instance of `{foo,bar}`. They behave the same.
   * - `*(foo|bar)` - Matches _n_ instances of `{foo,bar}`.
   * - `+(foo|bar)` - Matches _n > 0_ instances of `{foo,bar}`.
   * - `!(foo|bar)` - Matches anything other than `{foo,bar}`.
   * - See https://www.linuxjournal.com/content/bash-extended-globbing.
   *
   * Globstar syntax:
   * - Requires `{ globstar: true }`.
   * - `**` - Matches any number of any path segments.
   *     - Must comprise its entire path segment in the provided glob.
   * - See https://www.linuxjournal.com/content/globstar-new-bash-globbing-option.
   *
   * Note the following properties:
   * - The generated `RegExp` is anchored at both start and end.
   * - Repeating and trailing separators are tolerated. Trailing separators in the
   *   provided glob have no meaning and are discarded.
   * - Absolute globs will only match absolute paths, etc.
   * - Empty globs will match nothing.
   * - Any special glob syntax must be contained to one path segment. For example,
   *   `?(foo|bar/baz)` is invalid. The separator will take precedence and the
   *   first segment ends with an unclosed group.
   * - If a path segment ends with unclosed groups or a dangling escape prefix, a
   *   parse error has occurred. Every character for that segment is taken
   *   literally in this event.
   *
   * Limitations:
   * - A negative group like `!(foo|bar)` will wrongly be converted to a negative
   *   look-ahead followed by a wildcard. This means that `!(foo).js` will wrongly
   *   fail to match `foobar.js`, even though `foobar` is not `foo`. Effectively,
   *   `!(foo|bar)` is treated like `!(@(foo|bar)*)`. This will work correctly if
   *   the group occurs not nested at the end of the segment.
   */ globToRegExp } from "./glob_to_regexp.ts";
export { /**
   * @deprecated (will be removed in 0.215.0) Import from {@link https://deno.land/std/path/normalize_glob.ts} instead.
   *
   * Like normalize(), but doesn't collapse "**\/.." when `globstar` is true.
   */ normalizeGlob } from "./normalize_glob.ts";
export { /**
   * @deprecated (will be removed in 0.215.0) Import from {@link https://deno.land/std/path/join_globs.ts} instead.
   *
   * Like join(), but doesn't collapse "**\/.." when `globstar` is true.
   */ joinGlobs } from "./join_globs.ts";
//# sourceMappingURL=data:application/json;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbImh0dHBzOi8vZGVuby5sYW5kL3N0ZEAwLjIxNC4wL3BhdGgvZ2xvYi50cyJdLCJzb3VyY2VzQ29udGVudCI6WyIvLyBDb3B5cmlnaHQgMjAxOC0yMDI0IHRoZSBEZW5vIGF1dGhvcnMuIEFsbCByaWdodHMgcmVzZXJ2ZWQuIE1JVCBsaWNlbnNlLlxuLy8gVGhpcyBtb2R1bGUgaXMgYnJvd3NlciBjb21wYXRpYmxlLlxuXG5leHBvcnQgdHlwZSB7IEdsb2JPcHRpb25zIH0gZnJvbSBcIi4vX2NvbW1vbi9nbG9iX3RvX3JlZ19leHAudHNcIjtcbmV4cG9ydCB0eXBlIHsgR2xvYlRvUmVnRXhwT3B0aW9ucyB9IGZyb20gXCIuL2dsb2JfdG9fcmVnZXhwLnRzXCI7XG5cbmV4cG9ydCB7XG4gIC8qKlxuICAgKiBAZGVwcmVjYXRlZCAod2lsbCBiZSByZW1vdmVkIGluIDAuMjE1LjApIEltcG9ydCBmcm9tIHtAbGluayBodHRwczovL2Rlbm8ubGFuZC9zdGQvcGF0aC9pc19nbG9iLnRzfSBpbnN0ZWFkLlxuICAgKlxuICAgKiBUZXN0IHdoZXRoZXIgdGhlIGdpdmVuIHN0cmluZyBpcyBhIGdsb2JcbiAgICovXG4gIGlzR2xvYixcbn0gZnJvbSBcIi4vaXNfZ2xvYi50c1wiO1xuXG5leHBvcnQge1xuICAvKipcbiAgICogQGRlcHJlY2F0ZWQgKHdpbGwgYmUgcmVtb3ZlZCBpbiAwLjIxNS4wKSBJbXBvcnQgZnJvbSB7QGxpbmsgaHR0cHM6Ly9kZW5vLmxhbmQvc3RkL3BhdGgvZ2xvYl90b19yZWdleHAudHN9IGluc3RlYWQuXG4gICAqXG4gICAqIENvbnZlcnQgYSBnbG9iIHN0cmluZyB0byBhIHJlZ3VsYXIgZXhwcmVzc2lvbi5cbiAgICpcbiAgICogVHJpZXMgdG8gbWF0Y2ggYmFzaCBnbG9iIGV4cGFuc2lvbiBhcyBjbG9zZWx5IGFzIHBvc3NpYmxlLlxuICAgKlxuICAgKiBCYXNpYyBnbG9iIHN5bnRheDpcbiAgICogLSBgKmAgLSBNYXRjaGVzIGV2ZXJ5dGhpbmcgd2l0aG91dCBsZWF2aW5nIHRoZSBwYXRoIHNlZ21lbnQuXG4gICAqIC0gYD9gIC0gTWF0Y2hlcyBhbnkgc2luZ2xlIGNoYXJhY3Rlci5cbiAgICogLSBge2ZvbyxiYXJ9YCAtIE1hdGNoZXMgYGZvb2Agb3IgYGJhcmAuXG4gICAqIC0gYFthYmNkXWAgLSBNYXRjaGVzIGBhYCwgYGJgLCBgY2Agb3IgYGRgLlxuICAgKiAtIGBbYS1kXWAgLSBNYXRjaGVzIGBhYCwgYGJgLCBgY2Agb3IgYGRgLlxuICAgKiAtIGBbIWFiY2RdYCAtIE1hdGNoZXMgYW55IHNpbmdsZSBjaGFyYWN0ZXIgYmVzaWRlcyBgYWAsIGBiYCwgYGNgIG9yIGBkYC5cbiAgICogLSBgW1s6PGNsYXNzPjpdXWAgLSBNYXRjaGVzIGFueSBjaGFyYWN0ZXIgYmVsb25naW5nIHRvIGA8Y2xhc3M+YC5cbiAgICogICAgIC0gYFtbOmFsbnVtOl1dYCAtIE1hdGNoZXMgYW55IGRpZ2l0IG9yIGxldHRlci5cbiAgICogICAgIC0gYFtbOmRpZ2l0Ol1hYmNdYCAtIE1hdGNoZXMgYW55IGRpZ2l0LCBgYWAsIGBiYCBvciBgY2AuXG4gICAqICAgICAtIFNlZSBodHRwczovL2ZhY2VsZXNzdXNlci5naXRodWIuaW8vd2NtYXRjaC9nbG9iLyNwb3NpeC1jaGFyYWN0ZXItY2xhc3Nlc1xuICAgKiAgICAgICBmb3IgYSBjb21wbGV0ZSBsaXN0IG9mIHN1cHBvcnRlZCBjaGFyYWN0ZXIgY2xhc3Nlcy5cbiAgICogLSBgXFxgIC0gRXNjYXBlcyB0aGUgbmV4dCBjaGFyYWN0ZXIgZm9yIGFuIGBvc2Agb3RoZXIgdGhhbiBgXCJ3aW5kb3dzXCJgLlxuICAgKiAtIFxcYCAtIEVzY2FwZXMgdGhlIG5leHQgY2hhcmFjdGVyIGZvciBgb3NgIHNldCB0byBgXCJ3aW5kb3dzXCJgLlxuICAgKiAtIGAvYCAtIFBhdGggc2VwYXJhdG9yLlxuICAgKiAtIGBcXGAgLSBBZGRpdGlvbmFsIHBhdGggc2VwYXJhdG9yIG9ubHkgZm9yIGBvc2Agc2V0IHRvIGBcIndpbmRvd3NcImAuXG4gICAqXG4gICAqIEV4dGVuZGVkIHN5bnRheDpcbiAgICogLSBSZXF1aXJlcyBgeyBleHRlbmRlZDogdHJ1ZSB9YC5cbiAgICogLSBgPyhmb298YmFyKWAgLSBNYXRjaGVzIDAgb3IgMSBpbnN0YW5jZSBvZiBge2ZvbyxiYXJ9YC5cbiAgICogLSBgQChmb298YmFyKWAgLSBNYXRjaGVzIDEgaW5zdGFuY2Ugb2YgYHtmb28sYmFyfWAuIFRoZXkgYmVoYXZlIHRoZSBzYW1lLlxuICAgKiAtIGAqKGZvb3xiYXIpYCAtIE1hdGNoZXMgX25fIGluc3RhbmNlcyBvZiBge2ZvbyxiYXJ9YC5cbiAgICogLSBgKyhmb298YmFyKWAgLSBNYXRjaGVzIF9uID4gMF8gaW5zdGFuY2VzIG9mIGB7Zm9vLGJhcn1gLlxuICAgKiAtIGAhKGZvb3xiYXIpYCAtIE1hdGNoZXMgYW55dGhpbmcgb3RoZXIgdGhhbiBge2ZvbyxiYXJ9YC5cbiAgICogLSBTZWUgaHR0cHM6Ly93d3cubGludXhqb3VybmFsLmNvbS9jb250ZW50L2Jhc2gtZXh0ZW5kZWQtZ2xvYmJpbmcuXG4gICAqXG4gICAqIEdsb2JzdGFyIHN5bnRheDpcbiAgICogLSBSZXF1aXJlcyBgeyBnbG9ic3RhcjogdHJ1ZSB9YC5cbiAgICogLSBgKipgIC0gTWF0Y2hlcyBhbnkgbnVtYmVyIG9mIGFueSBwYXRoIHNlZ21lbnRzLlxuICAgKiAgICAgLSBNdXN0IGNvbXByaXNlIGl0cyBlbnRpcmUgcGF0aCBzZWdtZW50IGluIHRoZSBwcm92aWRlZCBnbG9iLlxuICAgKiAtIFNlZSBodHRwczovL3d3dy5saW51eGpvdXJuYWwuY29tL2NvbnRlbnQvZ2xvYnN0YXItbmV3LWJhc2gtZ2xvYmJpbmctb3B0aW9uLlxuICAgKlxuICAgKiBOb3RlIHRoZSBmb2xsb3dpbmcgcHJvcGVydGllczpcbiAgICogLSBUaGUgZ2VuZXJhdGVkIGBSZWdFeHBgIGlzIGFuY2hvcmVkIGF0IGJvdGggc3RhcnQgYW5kIGVuZC5cbiAgICogLSBSZXBlYXRpbmcgYW5kIHRyYWlsaW5nIHNlcGFyYXRvcnMgYXJlIHRvbGVyYXRlZC4gVHJhaWxpbmcgc2VwYXJhdG9ycyBpbiB0aGVcbiAgICogICBwcm92aWRlZCBnbG9iIGhhdmUgbm8gbWVhbmluZyBhbmQgYXJlIGRpc2NhcmRlZC5cbiAgICogLSBBYnNvbHV0ZSBnbG9icyB3aWxsIG9ubHkgbWF0Y2ggYWJzb2x1dGUgcGF0aHMsIGV0Yy5cbiAgICogLSBFbXB0eSBnbG9icyB3aWxsIG1hdGNoIG5vdGhpbmcuXG4gICAqIC0gQW55IHNwZWNpYWwgZ2xvYiBzeW50YXggbXVzdCBiZSBjb250YWluZWQgdG8gb25lIHBhdGggc2VnbWVudC4gRm9yIGV4YW1wbGUsXG4gICAqICAgYD8oZm9vfGJhci9iYXopYCBpcyBpbnZhbGlkLiBUaGUgc2VwYXJhdG9yIHdpbGwgdGFrZSBwcmVjZWRlbmNlIGFuZCB0aGVcbiAgICogICBmaXJzdCBzZWdtZW50IGVuZHMgd2l0aCBhbiB1bmNsb3NlZCBncm91cC5cbiAgICogLSBJZiBhIHBhdGggc2VnbWVudCBlbmRzIHdpdGggdW5jbG9zZWQgZ3JvdXBzIG9yIGEgZGFuZ2xpbmcgZXNjYXBlIHByZWZpeCwgYVxuICAgKiAgIHBhcnNlIGVycm9yIGhhcyBvY2N1cnJlZC4gRXZlcnkgY2hhcmFjdGVyIGZvciB0aGF0IHNlZ21lbnQgaXMgdGFrZW5cbiAgICogICBsaXRlcmFsbHkgaW4gdGhpcyBldmVudC5cbiAgICpcbiAgICogTGltaXRhdGlvbnM6XG4gICAqIC0gQSBuZWdhdGl2ZSBncm91cCBsaWtlIGAhKGZvb3xiYXIpYCB3aWxsIHdyb25nbHkgYmUgY29udmVydGVkIHRvIGEgbmVnYXRpdmVcbiAgICogICBsb29rLWFoZWFkIGZvbGxvd2VkIGJ5IGEgd2lsZGNhcmQuIFRoaXMgbWVhbnMgdGhhdCBgIShmb28pLmpzYCB3aWxsIHdyb25nbHlcbiAgICogICBmYWlsIHRvIG1hdGNoIGBmb29iYXIuanNgLCBldmVuIHRob3VnaCBgZm9vYmFyYCBpcyBub3QgYGZvb2AuIEVmZmVjdGl2ZWx5LFxuICAgKiAgIGAhKGZvb3xiYXIpYCBpcyB0cmVhdGVkIGxpa2UgYCEoQChmb298YmFyKSopYC4gVGhpcyB3aWxsIHdvcmsgY29ycmVjdGx5IGlmXG4gICAqICAgdGhlIGdyb3VwIG9jY3VycyBub3QgbmVzdGVkIGF0IHRoZSBlbmQgb2YgdGhlIHNlZ21lbnQuXG4gICAqL1xuICBnbG9iVG9SZWdFeHAsXG59IGZyb20gXCIuL2dsb2JfdG9fcmVnZXhwLnRzXCI7XG5cbmV4cG9ydCB7XG4gIC8qKlxuICAgKiBAZGVwcmVjYXRlZCAod2lsbCBiZSByZW1vdmVkIGluIDAuMjE1LjApIEltcG9ydCBmcm9tIHtAbGluayBodHRwczovL2Rlbm8ubGFuZC9zdGQvcGF0aC9ub3JtYWxpemVfZ2xvYi50c30gaW5zdGVhZC5cbiAgICpcbiAgICogTGlrZSBub3JtYWxpemUoKSwgYnV0IGRvZXNuJ3QgY29sbGFwc2UgXCIqKlxcLy4uXCIgd2hlbiBgZ2xvYnN0YXJgIGlzIHRydWUuXG4gICAqL1xuICBub3JtYWxpemVHbG9iLFxufSBmcm9tIFwiLi9ub3JtYWxpemVfZ2xvYi50c1wiO1xuXG5leHBvcnQge1xuICAvKipcbiAgICogQGRlcHJlY2F0ZWQgKHdpbGwgYmUgcmVtb3ZlZCBpbiAwLjIxNS4wKSBJbXBvcnQgZnJvbSB7QGxpbmsgaHR0cHM6Ly9kZW5vLmxhbmQvc3RkL3BhdGgvam9pbl9nbG9icy50c30gaW5zdGVhZC5cbiAgICpcbiAgICogTGlrZSBqb2luKCksIGJ1dCBkb2Vzbid0IGNvbGxhcHNlIFwiKipcXC8uLlwiIHdoZW4gYGdsb2JzdGFyYCBpcyB0cnVlLlxuICAgKi9cbiAgam9pbkdsb2JzLFxufSBmcm9tIFwiLi9qb2luX2dsb2JzLnRzXCI7XG4iXSwibmFtZXMiOltdLCJtYXBwaW5ncyI6IkFBQUEsMEVBQTBFO0FBQzFFLHFDQUFxQztBQUtyQyxTQUNFOzs7O0dBSUMsR0FDRCxNQUFNLFFBQ0QsZUFBZTtBQUV0QixTQUNFOzs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7Ozs7O0dBMERDLEdBQ0QsWUFBWSxRQUNQLHNCQUFzQjtBQUU3QixTQUNFOzs7O0dBSUMsR0FDRCxhQUFhLFFBQ1Isc0JBQXNCO0FBRTdCLFNBQ0U7Ozs7R0FJQyxHQUNELFNBQVMsUUFDSixrQkFBa0IifQ==
// denoCacheMetadata=8570325798503277939,8359785845093352801